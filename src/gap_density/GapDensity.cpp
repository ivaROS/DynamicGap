#include <dynamic_gap/gap_density/GapDensity.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace dynamic_gap
{
bool densityIndexInside(int index, int right, int left, int size)
{
    if (size <= 0 || index < 0 || index >= size || right < 0 || right >= size || left < 0 || left >= size)
        return false;
    return right <= left ? index >= right && index <= left : index >= right || index <= left;
}

DensitySector computeDensitySector(const std::vector<DensityEndpoint>& points,
                                   int right, int left, int size, float increment, float threshold)
{
    DensitySector sector;
    if (!densityIndexInside(right, right, left, size) || !std::isfinite(increment) || increment <= 0)
        return sector;
    sector.angle = ((left - right + size) % size) * increment;
    for (const auto& point : points)
    {
        if (!densityIndexInside(point.scan_index, right, left, size) || !point.position.allFinite())
            continue;
        ++sector.count;
        sector.radius = std::max(sector.radius, point.position.norm());
        if (std::isfinite(point.speed) && point.speed >= threshold) ++sector.dynamic_count;
        if (point.ground_truth_dynamic) ++sector.ground_truth_count;
    }
    sector.area = 0.5f * sector.radius * sector.radius * sector.angle;
    if (sector.area > 0 && std::isfinite(sector.area))
    {
        sector.density = sector.dynamic_count / sector.area;
        sector.ground_truth_density = sector.ground_truth_count / sector.area;
    }
    return sector;
}

void GapDensity::initialize(ros::NodeHandle& topics, ros::NodeHandle& parameters, const std::string& frame)
{
    frame_ = frame;
    parameters.param("gap_density/enabled", enabled_, true);
    parameters.param<std::string>("gap_density/source", source_, "ground_truth");
    if (source_ != "ground_truth" && source_ != "gru")
        throw std::invalid_argument("gap_density/source must be ground_truth or gru");
    parameters.param("gap_density/max_ground_truth_age", max_ground_truth_age_, 0.5);
    parameters.param("gap_density/weight", weight_, 5.0);
    parameters.param("gap_density/max_prediction_age", max_age_, 3.0);
    parameters.param("gap_density/dynamic_speed_threshold", dynamic_threshold_, 0.05);
    parameters.param("gap_density/ground_truth_match_distance", match_threshold_, 0.6);
    if (!std::isfinite(weight_) || weight_ < 0 || !std::isfinite(max_age_) || max_age_ <= 0 ||
        !std::isfinite(max_ground_truth_age_) || max_ground_truth_age_ <= 0 ||
        !std::isfinite(dynamic_threshold_) || dynamic_threshold_ < 0 ||
        !std::isfinite(match_threshold_) || match_threshold_ <= 0)
        throw std::invalid_argument("Invalid gap_density parameters");
    std::string observation, prediction, marker, csv_path;
    parameters.param<std::string>("gap_density/observation_topic", observation, "gap_feature_observation");
    parameters.param<std::string>("gap_density/prediction_topic", prediction, "gru_gap_feature_prediction");
    parameters.param<std::string>("gap_density/marker_topic", marker, "contained_raw_gap_points");
    parameters.param<std::string>("gap_density/csv_path", csv_path, "");
    observations_ = topics.advertise<GapFeatureObservation>(observation, 100);
    markers_ = topics.advertise<visualization_msgs::MarkerArray>(marker, 1);
    if (source_ == "gru")
        predictions_ = topics.subscribe(prediction, 100, &GapDensity::predictionCB, this);
    else
        predictions_.shutdown();
    ROS_INFO_STREAM("Gap density scoring source: " << source_);
    if (!csv_path.empty())
    {
        std::error_code error;
        const auto parent = std::filesystem::path(csv_path).parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent, error);
        // Never overwrite an earlier collection session.
        if (!error && !std::filesystem::exists(csv_path)) csv_.open(csv_path);
        if (!csv_.is_open()) ROS_WARN_STREAM("Cannot create density CSV (or file already exists): " << csv_path);
        else csv_ << "sample_idx,stamp,model_id,gap_index,side,sector_density,sector_dynamic_raw_gap_point_count,"
                     "contained_raw_gap_point_count,sector_area,sector_angle_rad,sector_radius,"
                     "gt_sector_density,gt_sector_dynamic_raw_gap_point_count\n";
    }
}

void GapDensity::prediction(const GapFeaturePrediction& message)
{
    if (source_ != "gru" || message.model_id < 0 || message.side != "left") return;
    boost::mutex::scoped_lock lock(mutex_);
    const int id = message.model_id;
    if (!active_ids_.count(id)) return;
    auto previous = estimates_.find(id);
    if (previous != estimates_.end() && message.header.stamp < previous->second.stamp) return;
    auto name = std::find(message.output_names.begin(), message.output_names.end(), "gt_sector_density");
    const auto index = std::distance(message.output_names.begin(), name);
    if (!message.valid || message.seq_len_used <= 0 || message.header.stamp.isZero() ||
        message.header.frame_id != frame_ || message.output_names.size() != message.output_values.size() ||
        name == message.output_names.end() || !std::isfinite(message.output_values[index]))
    {
        estimates_.erase(id);
        return;
    }
    estimates_[id] = {message.header.stamp, std::max(0.0f, message.output_values[index])};
}

float GapDensity::cost(int id, const ros::Time& now) const
{
    if (!enabled_ || id < 0) return 0;
    boost::mutex::scoped_lock lock(mutex_);
    const auto it = estimates_.find(id);
    if (it == estimates_.end() || !active_ids_.count(id) ||
        std::abs((now - it->second.stamp).toSec()) >
            (source_ == "ground_truth" ? max_ground_truth_age_ : max_age_)) return 0;
    const double penalty = weight_ * it->second.density;
    return std::isfinite(penalty) && penalty <= std::numeric_limits<float>::max() ? float(penalty) : 0;
}

void GapDensity::update(const std::vector<Gap*>& raw, const std::vector<Gap*>& simplified,
                        const sensor_msgs::LaserScan& scan,
                        const std::map<std::string, geometry_msgs::Pose>& poses,
                        const std::map<std::string, geometry_msgs::Vector3Stamped>& velocities,
                        bool ground_truth_available)
{
    {
        boost::mutex::scoped_lock lock(mutex_);
        if (!last_scan_stamp_.isZero() && scan.header.stamp < last_scan_stamp_) estimates_.clear();
        // GT is recomputed from this scan only; unavailable labels invalidate all old costs.
        if (source_ == "ground_truth") estimates_.clear();
        last_scan_stamp_ = scan.header.stamp;
        active_ids_.clear();
        for (auto* gap : simplified)
            if (gap && gap->getLeftGapPt()->getModel())
                active_ids_.insert(gap->getLeftGapPt()->getModel()->getID());
        for (auto it = estimates_.begin(); it != estimates_.end(); )
            if (!active_ids_.count(it->first) || std::abs((scan.header.stamp - it->second.stamp).toSec()) > max_age_)
                it = estimates_.erase(it);
            else ++it;
    }
    std::vector<DensityEndpoint> points;
    // Compute isolated speeds once per raw endpoint, outside the per-sector loop.
    for (auto* gap : raw)
    {
        if (!gap) continue;
        for (bool left : {false, true})
        {
            float x, y;
            if (left) gap->getLCartesian(x, y); else gap->getRCartesian(x, y);
            auto* model = (left ? gap->getLeftGapPt() : gap->getRightGapPt())->getModel();
            float speed = 0;
            if (model) { model->isolateGapDynamics(); speed = model->getGapVelocity().norm(); }
            bool dynamic = false;
            float nearest = match_threshold_;
            if (ground_truth_available)
                for (const auto& agent : poses)
                {
                    const float distance = std::hypot(agent.second.position.x - x, agent.second.position.y - y);
                    if (distance < nearest)
                    {
                        nearest = distance;
                        auto velocity = velocities.find(agent.first);
                        dynamic = velocity != velocities.end() &&
                            std::hypot(velocity->second.vector.x, velocity->second.vector.y) >= dynamic_threshold_;
                    }
                }
            points.push_back({left ? gap->LIdx() : gap->RIdx(), {x,y}, speed, dynamic, left});
        }
    }
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    const float palette[6][3] = {{1,.3f,.3f},{.3f,1,.3f},{.3f,.5f,1},{1,1,.3f},{1,.3f,1},{.3f,1,1}};
    for (std::size_t i = 0; i < simplified.size(); ++i)
    {
        auto* gap = simplified[i];
        if (!gap || !gap->getLeftGapPt()->getModel()) continue;
        const auto sector = computeDensitySector(points, gap->RIdx(), gap->LIdx(), scan.ranges.size(),
                                                scan.angle_increment, dynamic_threshold_);
        GapFeatureObservation observation;
        observation.header.stamp = scan.header.stamp;
        observation.header.frame_id = frame_;
        observation.gap_index = i;
        observation.model_id = gap->getLeftGapPt()->getModel()->getID();
        observation.side = "left";
        float lx, ly, rx, ry;
        gap->getLCartesian(lx,ly); gap->getRCartesian(rx,ry);
        observation.vis_x = (lx+rx)/2; observation.vis_y = (ly+ry)/2;
        observation.feature_names = {"sector_density", "sector_dynamic_raw_gap_point_count",
            "contained_raw_gap_point_count", "sector_area", "sector_angle_rad", "sector_radius"};
        observation.feature_values = {sector.density, float(sector.dynamic_count), float(sector.count),
                                     sector.area, sector.angle, sector.radius};
        if (ground_truth_available)
        {
            observation.target_names = {"gt_sector_density", "gt_sector_dynamic_raw_gap_point_count"};
            observation.target_values = {sector.ground_truth_density, float(sector.ground_truth_count)};
        }
        if (source_ == "ground_truth" && ground_truth_available &&
            sector.area > 0 && std::isfinite(sector.ground_truth_density))
        {
            boost::mutex::scoped_lock lock(mutex_);
            estimates_[observation.model_id] = {scan.header.stamp, sector.ground_truth_density};
        }
        observations_.publish(observation);
        if (csv_.is_open())
        {
            csv_ << std::fixed << std::setprecision(9) << sample_++ << ',' << scan.header.stamp.toSec() << ','
                 << observation.model_id << ',' << i << ",left";
            for (float value : observation.feature_values) csv_ << ',' << value;
            if (ground_truth_available) csv_ << ',' << sector.ground_truth_density << ',' << sector.ground_truth_count;
            else csv_ << ",,";
            csv_ << '\n';
        }
        if (source_ == "ground_truth")
        {
            visualization_msgs::Marker text;
            text.header = observation.header;
            text.ns = "ground_truth_density_text";
            text.id = i;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose.position.x = observation.vis_x; text.pose.position.y = observation.vis_y;
            text.pose.position.z = 0.35; text.pose.orientation.w = 1;
            text.scale.z = 0.28;
            text.color.g = text.color.a = 1;
            text.text = "gap " + std::to_string(i) + " | GT: " +
                (ground_truth_available ? std::to_string(sector.ground_truth_density) : "N/A");
            markers.markers.push_back(std::move(text));
        }
        for (bool left : {true, false})
        {
            visualization_msgs::Marker marker;
            marker.header = observation.header;
            marker.ns = left ? "contained_raw_left" : "contained_raw_right";
            marker.id = i;
            marker.type = left ? visualization_msgs::Marker::SPHERE_LIST : visualization_msgs::Marker::CUBE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18;
            marker.color.r = palette[i%6][0]; marker.color.g = palette[i%6][1]; marker.color.b = palette[i%6][2];
            marker.color.a = 1;
            for (const auto& point : points)
                if (point.left == left && point.position.allFinite() &&
                    densityIndexInside(point.scan_index, gap->RIdx(), gap->LIdx(), scan.ranges.size()))
                {
                    geometry_msgs::Point p;
                    p.x = point.position.x(); p.y = point.position.y(); p.z = 0.15;
                    marker.points.push_back(p);
                }
            markers.markers.push_back(std::move(marker));
        }
    }
    if (csv_.is_open()) csv_.flush();
    markers_.publish(markers);
}
}
