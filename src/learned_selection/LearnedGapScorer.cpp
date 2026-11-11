#include <dynamic_gap/learned_selection/LearnedGapScorer.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace dynamic_gap
{
    namespace
    {
        const std::string FORMAT = "dynamic_gap_gap_scorer";
        const int FORMAT_VERSION = 2;  // 2: the scene; version 1 exports still load
        // DynamicGap's Q_f in every recorded session (il_model.Q_F)
        const double Q_F = 0.05;

        // non-finite inputs become 0 (il_model.finite)
        double finiteOr0(const double & x)
        {
            return std::isfinite(x) ? x : 0.0;
        }

        // il_dataset.yaw
        double yaw(const geometry_msgs::Quaternion & q)
        {
            return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }

        Eigen::MatrixXf tensor(const YAML::Node & tensors, const std::string & name, const int & rows, const int & cols)
        {
            const YAML::Node t = tensors[name];
            if (!t)
                throw std::runtime_error("missing tensor " + name);
            std::vector<int> shape = t["shape"].as<std::vector<int>>();
            int size = 1;
            for (const int & s : shape)
                size *= s;
            int expectedRows = rows < 0 ? (shape.empty() ? 1 : shape.front()) : rows;
            int expectedCols = cols < 0 ? (expectedRows == 0 ? 0 : size / expectedRows) : cols;
            if (size != expectedRows * expectedCols)
            {
                std::ostringstream msg;
                msg << "tensor " << name << " has " << size << " values, expected " << expectedRows << " x " << expectedCols;
                throw std::runtime_error(msg.str());
            }
            const YAML::Node data = t["data"];
            if (!data || int(data.size()) != size)
                throw std::runtime_error("tensor " + name + " data does not match its shape");
            Eigen::MatrixXf m(expectedRows, expectedCols);
            // row-major, as torch flattens
            for (int i = 0; i < size; i++)
                m(i / expectedCols, i % expectedCols) = data[i].as<float>();
            return m;
        }

        Eigen::VectorXf vector(const YAML::Node & tensors, const std::string & name, const int & size)
        {
            Eigen::MatrixXf m = tensor(tensors, name, size, 1);
            return Eigen::Map<Eigen::VectorXf>(m.data(), m.size());
        }

        // il_model.mlp layers prefix.<k>: the Linear layers, in order
        std::vector<int> layerIndices(const YAML::Node & tensors, const std::string & prefix)
        {
            std::vector<int> idx;
            for (YAML::const_iterator it = tensors.begin(); it != tensors.end(); ++it)
            {
                const std::string name = it->first.as<std::string>();
                const std::string head = prefix + ".";
                const std::string tail = ".weight";
                if (name.size() > head.size() + tail.size() && name.compare(0, head.size(), head) == 0
                    && name.compare(name.size() - tail.size(), tail.size(), tail) == 0)
                {
                    const std::string k = name.substr(head.size(), name.size() - head.size() - tail.size());
                    if (!k.empty() && std::all_of(k.begin(), k.end(), ::isdigit))
                        idx.push_back(std::stoi(k));
                }
            }
            std::sort(idx.begin(), idx.end());
            return idx;
        }
    }

    const std::vector<std::string> & LearnedGapScorer::candidateFeatures()
    {
        static const std::vector<std::string> names = {
            "left_x", "left_y", "right_x", "right_y", "width", "bearing_cos", "bearing_sin",
            "left_model_x", "left_model_y", "left_model_vx", "left_model_vy",
            "right_model_x", "right_model_y", "right_model_vx", "right_model_vy",
            "terminal_x", "terminal_y",
            "cost_infinite",
            "left_gvx", "left_gvy", "right_gvx", "right_gvy", "edge_vel_known"};
        return names;
    }

    const std::vector<std::string> & LearnedGapScorer::contextFeatures()
    {
        static const std::vector<std::string> names = {
            "vel_x", "vel_y", "vel_yaw", "goal_x", "goal_y", "waypoint_x", "waypoint_y"};
        return names;
    }

    const std::vector<std::string> & LearnedGapScorer::sceneFeatures()
    {
        static const std::vector<std::string> names = {
            "is_obstacle", "a_x", "a_y", "a_gvx", "a_gvy", "b_x", "b_y", "b_gvx", "b_gvy"};
        return names;
    }

    bool LearnedGapScorer::groundVelocity(const manual_gap_msgs::ManualGapEdge & edge,
                                          const geometry_msgs::Twist & rbtVel,
                                          double & vx, double & vy)
    {
        // il_model.ground_velocity: relative to the robot plus the robot's own
        if (!std::isfinite(edge.model_vel.x) || !std::isfinite(edge.model_vel.y))
            return false;
        vx = edge.model_vel.x + rbtVel.linear.x;
        vy = edge.model_vel.y + rbtVel.linear.y;
        return true;
    }

    int LearnedGapScorer::edgeAt(const std::vector<manual_gap_msgs::ManualGapEdge> & edges, const int & scanIdx) const
    {
        // il_model.edge_at: the nearest edge (the first of equals), at most edgeMatchRays_ away
        if (scanIdx < 0 || edges.empty())
            return -1;
        int best = 0;
        for (int i = 1; i < int(edges.size()); i++)
            if (std::abs(edges[i].scan_idx - scanIdx) < std::abs(edges[best].scan_idx - scanIdx))
                best = i;
        return std::abs(edges[best].scan_idx - scanIdx) <= edgeMatchRays_ ? best : -1;
    }

    Eigen::MatrixXf LearnedGapScorer::Linear::operator()(const Eigen::MatrixXf & x) const
    {
        return (x * weight.transpose()).rowwise() + bias.transpose();
    }

    Eigen::MatrixXf LearnedGapScorer::LayerNorm::operator()(const Eigen::MatrixXf & x) const
    {
        // torch.nn.LayerNorm: biased variance, eps 1e-5
        Eigen::MatrixXf y(x.rows(), x.cols());
        for (int i = 0; i < x.rows(); i++)
        {
            float mean = x.row(i).mean();
            Eigen::RowVectorXf centred = x.row(i).array() - mean;
            float var = centred.squaredNorm() / x.cols();
            y.row(i) = (centred / std::sqrt(var + 1e-5f)).cwiseProduct(weight.transpose()) + bias.transpose();
        }
        return y;
    }

    Eigen::MatrixXf LearnedGapScorer::runMlp(const Mlp & mlp, const Eigen::MatrixXf & x)
    {
        Eigen::MatrixXf h = x;
        for (size_t i = 0; i < mlp.size(); i++)
        {
            h = mlp[i](h);
            if (i + 1 < mlp.size())
                h = h.cwiseMax(0.0f);
        }
        return h;
    }

    bool LearnedGapScorer::load(const std::string & path, std::string & error)
    {
        loaded_ = false;
        try
        {
            // the export is JSON, which yaml-cpp reads
            const YAML::Node root = YAML::LoadFile(path);
            if (!root["format"] || root["format"].as<std::string>() != FORMAT)
                throw std::runtime_error("not a gap scorer export (format is not " + FORMAT + ")");
            const int version = root["version"].as<int>();
            if (version < 1 || version > FORMAT_VERSION)
                throw std::runtime_error("export format version " + root["version"].as<std::string>()
                                         + ", the planner reads versions 1 to " + std::to_string(FORMAT_VERSION));

            // the features the model was trained on must be the ones computed here; a
            // model from before a feature was added has a leading part of the list
            const std::vector<std::string> modelCand = root["candidate_features"].as<std::vector<std::string>>();
            if (modelCand.size() > candidateFeatures().size()
                || !std::equal(modelCand.begin(), modelCand.end(), candidateFeatures().begin())
                || root["context_features"].as<std::vector<std::string>>() != contextFeatures())
                throw std::runtime_error("feature lists differ from the planner's (model exported by another il_model version)");

            candIdx_.clear();
            ctxIdx_.clear();
            for (const std::string & name : root["inputs"].as<std::vector<std::string>>())
            {
                auto c = std::find(modelCand.begin(), modelCand.end(), name);
                auto x = std::find(contextFeatures().begin(), contextFeatures().end(), name);
                if (c != modelCand.end())
                    candIdx_.push_back(std::distance(modelCand.begin(), c));
                else if (x != contextFeatures().end())
                    ctxIdx_.push_back(std::distance(contextFeatures().begin(), x));
                else
                    throw std::runtime_error("unknown input " + name);
            }

            network_ = root["network"].as<bool>();
            residual_ = root["residual"].as<bool>();
            attention_ = network_ && root["attention"].as<bool>();
            heads_ = root["heads"].as<int>();
            terminalCostCap_ = root["terminal_cost_cap"].as<float>();
            clearanceCostCap_ = root["clearance_cost_cap"].as<float>();
            maskedScore_ = root["masked_score"].as<float>();
            if (!network_ && !residual_)
                throw std::runtime_error("model has neither the network nor the linear term");
            scene_ = network_ && root["scene"] && root["scene"].as<bool>();
            if (scene_ && root["scene_features"].as<std::vector<std::string>>() != sceneFeatures())
                throw std::runtime_error("scene feature list differs from the planner's");
            edgeMatchRays_ = root["edge_match_rays"] ? root["edge_match_rays"].as<int>() : 3;

            const YAML::Node tensors = root["tensors"];
            const int nCand = modelCand.size(), nCtx = contextFeatures().size();
            candMean_ = vector(tensors, "cand_mean", nCand);
            candStd_ = vector(tensors, "cand_std", nCand);
            ctxMean_ = vector(tensors, "ctx_mean", nCtx);
            ctxStd_ = vector(tensors, "ctx_std", nCtx);
            if (residual_)
                dgWeight_ = vector(tensors, "dg_weight", 2);

            auto loadMlp = [&](const std::string & prefix, const int & inputs)
            {
                Mlp mlp;
                int in = inputs;
                for (const int & k : layerIndices(tensors, prefix))
                {
                    const std::string name = prefix + "." + std::to_string(k);
                    Linear l;
                    l.weight = tensor(tensors, name + ".weight", -1, in);
                    l.bias = vector(tensors, name + ".bias", l.weight.rows());
                    in = l.weight.rows();
                    mlp.push_back(l);
                }
                if (mlp.empty())
                    throw std::runtime_error("missing layers " + prefix);
                return mlp;
            };

            candEnc_.clear(); ctxEnc_.clear(); head_.clear(); ff_.clear(); sceneEnc_.clear(); sceneFf_.clear();
            if (network_)
            {
                candEnc_ = loadMlp("cand_enc", candIdx_.size());
                ctxEnc_ = loadMlp("ctx_enc", ctxIdx_.size());
                const int hidden = candEnc_.back().weight.rows();
                if (ctxEnc_.back().weight.rows() != hidden)
                    throw std::runtime_error("candidate and context encoders differ in size");
                if ((attention_ || scene_) && (heads_ <= 0 || hidden % heads_ != 0))
                    throw std::runtime_error("attention heads do not divide the hidden size");
                if (scene_)
                {
                    const int nScene = sceneFeatures().size();
                    sceneMean_ = vector(tensors, "scene_mean", nScene);
                    sceneStd_ = vector(tensors, "scene_std", nScene);
                    sceneEnc_ = loadMlp("scene_enc", nScene);
                    if (sceneEnc_.back().weight.rows() != hidden)
                        throw std::runtime_error("scene encoder differs in size from the candidate encoder");
                    const std::string p = "read_scene.";
                    for (auto & norm : {std::make_pair(&sceneNormQ_, "norm_q"), std::make_pair(&sceneNormKv_, "norm_kv"),
                                        std::make_pair(&sceneNorm2_, "norm2")})
                    {
                        norm.first->weight = vector(tensors, p + norm.second + ".weight", hidden);
                        norm.first->bias = vector(tensors, p + norm.second + ".bias", hidden);
                    }
                    sceneQ_.weight = tensor(tensors, p + "q.weight", hidden, hidden);
                    sceneQ_.bias = vector(tensors, p + "q.bias", hidden);
                    sceneKv_.weight = tensor(tensors, p + "kv.weight", 2 * hidden, hidden);
                    sceneKv_.bias = vector(tensors, p + "kv.bias", 2 * hidden);
                    sceneOut_.weight = tensor(tensors, p + "out.weight", hidden, hidden);
                    sceneOut_.bias = vector(tensors, p + "out.bias", hidden);
                    sceneFf_ = loadMlp("read_scene.ff", hidden);
                    if (sceneFf_.back().weight.rows() != hidden)
                        throw std::runtime_error("scene feed-forward does not return the hidden size");
                }
                if (attention_)
                {
                    norm1_.weight = vector(tensors, "mix.norm1.weight", hidden);
                    norm1_.bias = vector(tensors, "mix.norm1.bias", hidden);
                    norm2_.weight = vector(tensors, "mix.norm2.weight", hidden);
                    norm2_.bias = vector(tensors, "mix.norm2.bias", hidden);
                    qkv_.weight = tensor(tensors, "mix.qkv.weight", 3 * hidden, hidden);
                    qkv_.bias = vector(tensors, "mix.qkv.bias", 3 * hidden);
                    out_.weight = tensor(tensors, "mix.out.weight", hidden, hidden);
                    out_.bias = vector(tensors, "mix.out.bias", hidden);
                    ff_ = loadMlp("mix.ff", hidden);
                    if (ff_.back().weight.rows() != hidden)
                        throw std::runtime_error("attention feed-forward does not return the hidden size");
                }
                head_ = loadMlp("head", 2 * hidden);
                if (head_.back().weight.rows() != 1)
                    throw std::runtime_error("score head does not return one value");
            }
        } catch (const std::exception & e)
        {
            error = e.what();
            return false;
        }
        loaded_ = true;
        return true;
    }

    GapScorerInput LearnedGapScorer::featurize(const manual_gap_msgs::ManualGapSample & sample) const
    {
        // il_dataset.example + il_model.featurize; computed in double, stored as float
        const int n = sample.candidates.size();
        GapScorerInput in;
        in.cand = Eigen::MatrixXf::Zero(n, candidateFeatures().size());
        in.dg = Eigen::MatrixXf::Zero(n, 2);
        in.dgAvail = Eigen::VectorXf::Zero(n);
        in.mask.assign(n, false);

        const geometry_msgs::Pose & pose = sample.robot_pose_odom;
        const double c = std::cos(yaw(pose.orientation)), s = std::sin(yaw(pose.orientation));

        for (int i = 0; i < n; i++)
        {
            const manual_gap_msgs::ManualGapCandidate & cand = sample.candidates[i];
            const double lx = double(cand.left_range) * std::cos(double(cand.left_theta));
            const double ly = double(cand.left_range) * std::sin(double(cand.left_theta));
            const double rx = double(cand.right_range) * std::cos(double(cand.right_theta));
            const double ry = double(cand.right_range) * std::sin(double(cand.right_theta));
            double terminalX = 0.0, terminalY = 0.0;
            if (cand.feasible)
            {
                // il_dataset.to_robot_frame
                const double dx = cand.terminal_pose.position.x - pose.position.x;
                const double dy = cand.terminal_pose.position.y - pose.position.y;
                terminalX = c * dx + s * dy;
                terminalY = -s * dx + c * dy;
            }
            // ground velocities of the scene edges the gap points are
            double gv[4] = {0.0, 0.0, 0.0, 0.0};
            bool known = true;
            const int sides[2] = {edgeAt(sample.edges, cand.left_scan_idx), edgeAt(sample.edges, cand.right_scan_idx)};
            for (int side = 0; side < 2; side++)
            {
                double vx = 0.0, vy = 0.0;
                if (sides[side] >= 0 && groundVelocity(sample.edges[sides[side]], sample.robot_vel, vx, vy))
                {
                    gv[2 * side] = vx;
                    gv[2 * side + 1] = vy;
                } else
                    known = false;
            }
            const double row[] = {
                lx, ly, rx, ry, std::hypot(lx - rx, ly - ry),
                std::cos(double(cand.bearing)), std::sin(double(cand.bearing)),
                cand.left_model_pos.x, cand.left_model_pos.y, cand.left_model_vel.x, cand.left_model_vel.y,
                cand.right_model_pos.x, cand.right_model_pos.y, cand.right_model_vel.x, cand.right_model_vel.y,
                terminalX, terminalY,
                std::isinf(cand.dg_cost) ? 1.0 : 0.0,
                gv[0], gv[1], gv[2], gv[3], known ? 1.0 : 0.0};
            for (int k = 0; k < in.cand.cols(); k++)
                in.cand(i, k) = float(finiteOr0(row[k]));

            // il_model.dg_terms: NaN means not recorded; infinite costs are capped
            const float t = cand.dg_terminal_cost, k = cand.dg_clearance_cost;
            const double wx = sample.local_waypoint_robot_frame.x, wy = sample.local_waypoint_robot_frame.y;
            if (!std::isnan(t) && !std::isnan(k))
            {
                in.dg(i, 0) = std::min(t, terminalCostCap_);
                in.dg(i, 1) = std::min(k, clearanceCostCap_);
                in.dgAvail(i) = 1.0f;
            } else if (cand.feasible && std::isfinite(wx) && std::isfinite(wy))
            {
                // il_model.rebuilt_terminal_cost: Q_f * |trajectory end - local waypoint|, no clearance cost
                const float rebuilt = float(Q_F * std::hypot(terminalX - wx, terminalY - wy));
                in.dg(i, 0) = std::min(rebuilt, terminalCostCap_);
                in.dgAvail(i) = 1.0f;
            }
            in.mask[i] = cand.feasible && cand.shown;
        }

        const double ctx[] = {
            sample.robot_vel.linear.x, sample.robot_vel.linear.y, sample.robot_vel.angular.z,
            sample.goal_robot_frame.x, sample.goal_robot_frame.y,
            sample.local_waypoint_robot_frame.x, sample.local_waypoint_robot_frame.y};
        in.ctx = Eigen::VectorXf(contextFeatures().size());
        for (int k = 0; k < in.ctx.size(); k++)
            in.ctx(k) = float(finiteOr0(ctx[k]));

        // il_model.scene_links: a gap link from each right point (even position) to its
        // left point, an obstacle link from each left point to the next edge
        const std::vector<manual_gap_msgs::ManualGapEdge> & edges = sample.edges;
        const int nEdges = edges.size();
        std::vector<std::vector<double>> links;
        auto state = [&](const int & i, std::vector<double> & link)
        {
            double vx = 0.0, vy = 0.0;
            groundVelocity(edges[i], sample.robot_vel, vx, vy);
            link.push_back(double(edges[i].range) * std::cos(double(edges[i].theta)));
            link.push_back(double(edges[i].range) * std::sin(double(edges[i].theta)));
            link.push_back(vx);
            link.push_back(vy);
        };
        for (int i = 0; i < nEdges; i++)
        {
            const bool gapLink = (i % 2 == 0);
            const int other = gapLink ? edges[i].gap_neighbor : edges[i].obstacle_neighbor;
            if (other < 0 || other >= nEdges)
                continue;
            std::vector<double> link = {gapLink ? 0.0 : 1.0};
            state(i, link);
            state(other, link);
            links.push_back(link);
        }
        in.scene = Eigen::MatrixXf::Zero(links.size(), sceneFeatures().size());
        for (size_t l = 0; l < links.size(); l++)
            for (int k = 0; k < in.scene.cols(); k++)
                in.scene(l, k) = float(finiteOr0(links[l][k]));
        return in;
    }

    Eigen::MatrixXf LearnedGapScorer::readScene(const Eigen::MatrixXf & h, const Eigen::MatrixXf & s) const
    {
        // il_model.SceneAttention: each candidate reads the scene's links, then feed-forward;
        // without links the read is zero and only the feed-forward applies
        const int n = h.rows(), m = s.rows(), d = h.cols(), hd = d / heads_;
        Eigen::MatrixXf out = h;
        if (m > 0)
        {
            const Eigen::MatrixXf q = sceneQ_(sceneNormQ_(h));
            const Eigen::MatrixXf kv = sceneKv_(sceneNormKv_(s));
            Eigen::MatrixXf read(n, d);
            for (int k = 0; k < heads_; k++)
            {
                const Eigen::MatrixXf key = kv.block(0, k * hd, m, hd);
                const Eigen::MatrixXf v = kv.block(0, d + k * hd, m, hd);
                Eigen::MatrixXf att = (q.block(0, k * hd, n, hd) * key.transpose()) / std::sqrt(float(hd));
                for (int i = 0; i < n; i++)
                {
                    const float mx = att.row(i).maxCoeff();
                    att.row(i) = (att.row(i).array() - mx).exp();
                    att.row(i) /= att.row(i).sum();
                }
                read.block(0, k * hd, n, hd) = att * v;
            }
            out = h + sceneOut_(read);
        }
        return out + runMlp(sceneFf_, sceneNorm2_(out));
    }

    Eigen::MatrixXf LearnedGapScorer::attention(const Eigen::MatrixXf & h) const
    {
        // il_model.CandidateAttention: pre-norm self-attention, then feed-forward
        const int n = h.rows(), d = h.cols(), hd = d / heads_;
        const Eigen::MatrixXf qkv = qkv_(norm1_(h));
        Eigen::MatrixXf mixed(n, d);
        for (int k = 0; k < heads_; k++)
        {
            const Eigen::MatrixXf q = qkv.block(0, k * hd, n, hd);
            const Eigen::MatrixXf key = qkv.block(0, d + k * hd, n, hd);
            const Eigen::MatrixXf v = qkv.block(0, 2 * d + k * hd, n, hd);
            Eigen::MatrixXf att = (q * key.transpose()) / std::sqrt(float(hd));
            for (int i = 0; i < n; i++)
            {
                const float m = att.row(i).maxCoeff();
                att.row(i) = (att.row(i).array() - m).exp();
                att.row(i) /= att.row(i).sum();
            }
            mixed.block(0, k * hd, n, hd) = att * v;
        }
        Eigen::MatrixXf out = h + out_(mixed);
        return out + runMlp(ff_, norm2_(out));
    }

    Eigen::VectorXf LearnedGapScorer::score(const GapScorerInput & input) const
    {
        const int n = input.cand.rows();
        if (!loaded_ || n == 0)
            return Eigen::VectorXf::Zero(n);

        // only the options reach the model; the other candidates get no score of their own
        std::vector<int> options;
        for (int i = 0; i < n; i++)
            if (input.mask[i])
                options.push_back(i);
        const int m = options.size();
        Eigen::VectorXf scores = Eigen::VectorXf::Constant(n, maskedScore_);
        if (m == 0)
            return scores;

        Eigen::VectorXf optionScores = Eigen::VectorXf::Zero(m);
        if (residual_)
            for (int j = 0; j < m; j++)
                optionScores(j) -= input.dg.row(options[j]).dot(dgWeight_) * input.dgAvail(options[j]);

        if (network_)
        {
            Eigen::MatrixXf cand(m, candIdx_.size());
            for (size_t k = 0; k < candIdx_.size(); k++)
            {
                const int f = candIdx_[k];
                for (int j = 0; j < m; j++)
                    cand(j, k) = (input.cand(options[j], f) - candMean_(f)) / candStd_(f);
            }
            Eigen::MatrixXf ctx(1, ctxIdx_.size());
            for (size_t k = 0; k < ctxIdx_.size(); k++)
            {
                const int f = ctxIdx_[k];
                ctx(0, k) = (input.ctx(f) - ctxMean_(f)) / ctxStd_(f);
            }

            Eigen::MatrixXf e = runMlp(candEnc_, cand);
            const Eigen::RowVectorXf c = runMlp(ctxEnc_, ctx).row(0);
            // context goes in with every candidate, each reads the scene, then they see each other
            if (scene_ || attention_)
                e = e.rowwise() + c;
            if (scene_)
            {
                Eigen::MatrixXf s(input.scene.rows(), input.scene.cols());
                for (int k = 0; k < s.cols(); k++)
                    s.col(k) = (input.scene.col(k).array() - sceneMean_(k)) / sceneStd_(k);
                e = readScene(e, s.rows() > 0 ? runMlp(sceneEnc_, s) : Eigen::MatrixXf(0, e.cols()));
            }
            if (attention_)
                e = attention(e);
            Eigen::MatrixXf h(m, 2 * e.cols());
            h << e, c.replicate(m, 1);
            optionScores += runMlp(head_, h).col(0);
        }

        for (int j = 0; j < m; j++)
            scores(options[j]) = optionScores(j);
        return scores;
    }

    int LearnedGapScorer::choose(const manual_gap_msgs::ManualGapSample & sample,
                                 const std::vector<bool> & allowed,
                                 Eigen::VectorXf & scores) const
    {
        const GapScorerInput input = featurize(sample);
        scores = score(input);
        if (!loaded_)
            return -1;

        int best = -1;
        for (int i = 0; i < scores.size(); i++)
        {
            if (!input.mask[i] || (!allowed.empty() && !allowed[i]))
                continue;
            if (!std::isfinite(scores(i)))
                return -1;
            if (best < 0 || scores(i) > scores(best))
                best = i;
        }
        return best;
    }
}
