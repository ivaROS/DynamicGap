#pragma once

#include <manual_gap_msgs/ManualGapSample.h>

#include <Eigen/Core>

#include <map>
#include <string>
#include <vector>

namespace dynamic_gap
{
    /**
    * \brief Inputs of the gap scorer for one planning tick, as il_model.featurize()
    * computes them in il_tools: one row per candidate
    */
    struct GapScorerInput
    {
        Eigen::MatrixXf cand; /**< candidates x CANDIDATE_FEATURES */
        Eigen::VectorXf ctx; /**< CONTEXT_FEATURES */
        Eigen::MatrixXf dg; /**< candidates x 2: DynamicGap's terminal and clearance cost, capped */
        Eigen::VectorXf dgAvail; /**< 1 where DynamicGap's costs were recorded */
        std::vector<bool> mask; /**< selectable candidates: feasible and shown */
    };

    /**
    * \brief Learned gap scorer (il_tools/il_model.py GapScorer) run natively with
    * Eigen from the gap_scorer.json that il_train / il_export_scorer write.
    *
    * Data in, choice out: the scorer reads the tick's ManualGapSample (the same
    * message the recorder publishes and the training set is built from), scores
    * the selectable candidates and returns the highest-scoring one.
    */
    class LearnedGapScorer
    {
        public:
            /**
            * \brief Feature names in the order the planner computes them; must
            * equal il_model.CANDIDATE_FEATURES / CONTEXT_FEATURES
            */
            static const std::vector<std::string> & candidateFeatures();
            static const std::vector<std::string> & contextFeatures();

            /**
            * \brief Load a model exported by il_tools
            * \param path gap_scorer.json
            * \param error why loading failed
            * \return whether the model was loaded; on failure the scorer stays unloaded
            */
            bool load(const std::string & path, std::string & error);

            bool isLoaded() const { return loaded_; }

            /**
            * \brief The scorer's inputs from a sample: candidates, robot state and
            * goal, with terminal poses converted to the robot frame at
            * sample.robot_pose_odom, as il_dataset.py does
            */
            GapScorerInput featurize(const manual_gap_msgs::ManualGapSample & sample) const;

            /**
            * \brief Runs the model on the candidates in the mask only; the others get the masked score
            */
            Eigen::VectorXf score(const GapScorerInput & input) const;

            /**
            * \brief Highest-scoring candidate that is selectable and allowed
            * \param allowed extra restriction per candidate (empty: none)
            * \param scores scores of all candidates (output)
            * \return position in sample.candidates, -1 if none or a score is not finite
            */
            int choose(const manual_gap_msgs::ManualGapSample & sample,
                       const std::vector<bool> & allowed,
                       Eigen::VectorXf & scores) const;

        private:
            struct Linear
            {
                Eigen::MatrixXf weight; /**< out x in */
                Eigen::VectorXf bias;
                Eigen::MatrixXf operator()(const Eigen::MatrixXf & x) const; /**< rows are samples */
            };

            struct LayerNorm
            {
                Eigen::VectorXf weight, bias;
                Eigen::MatrixXf operator()(const Eigen::MatrixXf & x) const;
            };

            /** Linear layers with ReLU between them (il_model.mlp) */
            typedef std::vector<Linear> Mlp;
            static Eigen::MatrixXf runMlp(const Mlp & mlp, const Eigen::MatrixXf & x);

            /** h: the options of the tick only */
            Eigen::MatrixXf attention(const Eigen::MatrixXf & h) const;

            bool loaded_ = false;
            bool network_ = false, residual_ = false, attention_ = false;
            int heads_ = 0;
            float terminalCostCap_ = 0.0f, clearanceCostCap_ = 0.0f, maskedScore_ = -1e9f;

            std::vector<int> candIdx_, ctxIdx_; /**< columns the network uses */
            Eigen::VectorXf candMean_, candStd_, ctxMean_, ctxStd_;
            Eigen::Vector2f dgWeight_;
            Mlp candEnc_, ctxEnc_, head_, ff_;
            LayerNorm norm1_, norm2_;
            Linear qkv_, out_;
    };
}
