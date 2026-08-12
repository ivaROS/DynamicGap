#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Trajectory.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <vector>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <boost/thread/mutex.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <dynamic_gap/GapFeaturePrediction.h>
#include <ros/ros.h>
#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <cmath>
namespace dynamic_gap
{
    struct GruGapFeatureDensityEstimate
    {
        float pred_sector_density = 0.0f;
        ros::Time stamp;
        bool valid = false;
        int seq_len_used = 0;
    };

    /**
    * \brief Class responsible for scoring candidate trajectory according to
    * trajectory's proximity to local environment and global path's local waypoint
    */
    class TrajectoryEvaluator
    {
        public:
            TrajectoryEvaluator(
            ros::NodeHandle& nh,
            const DynamicGapConfig& cfg);

            /**
            * \brief receive new laser scan and update member variable accordingly
            * \param scan new laser scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            // void updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan);

            /**
            * \brief Helper function for transforming global path local waypoint into robot frame
            * \param globalPathLocalWaypointOdomFrame Current local waypoint along global plan in robot frame
            * \param odom2rbt transformation from odom frame to robot frame
            */
            void transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame,
                                                            const geometry_msgs::TransformStamped & odom2rbt);

            /**
            * \brief Function for evaluating pose-wise scores along candidate trajectory
            *
            * \param traj candidate trajectory to score
            * \param terminalGoalCostOnlyOut optional out: Q_f * terminal goal distance ONLY,
            *        with no density / width-clearance / endpoint terms. Ablation channel.
            *        (Formerly named terminalPoseCostNoDensity, which excluded more than
            *        just the density term and so read misleadingly.)
            * \param gapWidth candidate gap mouth width [m]. Negative or non-finite means
            *        "not measured": the width-clearance cost is skipped and the minimum-width
            *        gate does NOT fire, so callers that omit gap geometry are unaffected.
            *        A measured width below requiredGapWidth() is treated as impassable.
            * \param gapDepth candidate gap radial depth [m]. DIAGNOSTIC ONLY as of the
            *        width-clearance change -- it no longer contributes to any cost.
            * \param aspectRatioOut optional out: computed depth/width ratio (-1 if unavailable).
            *        Retained so the RViz "AR=" labels and the offline analysis keep working.
            */
            void evaluateTrajectory(
                const Trajectory & traj,
                std::vector<float> & posewiseCosts,
                float & terminalPoseCost,
                const std::vector<sensor_msgs::LaserScan> & futureScans,
                const int & scanIdx,
                const int & densityModelID,
                float* terminalGoalCostOnlyOut = nullptr,
                float gapWidth = -1.0f,
                float gapDepth = -1.0f,
                float* aspectRatioOut = nullptr,
                float gapLeftX = 0.0f,
                float gapLeftY = 0.0f,
                float gapRightX = 0.0f,
                float gapRightY = 0.0f,
                bool  haveGapEndpoints = false,
                float* endpointProximityCostOut = nullptr);

            /**
             * \brief Stores latest GRU density prediction for a gap model
             * \param msg incoming GRU density prediction message
             */
            void gruGapFeatureDensityCB(
                const dynamic_gap::GapFeaturePrediction::ConstPtr& msg);

            /**
             * \brief Gets latest valid GRU density prediction for a model
             * \param modelID gap model ID to query
             * \param currentStamp current time for freshness check
             * \param predDensityOut returned predicted density
             * \return true if a fresh valid prediction was found
             */
            bool getLatestGruGapFeatureDensityForModel(
                const int& modelID,
                const ros::Time& currentStamp,
                float& predDensityOut) const;

        private:

            //////////////////////////////////////////////////////
            // NOTE ON DEFAULTS
            //
            // The constructor in TrajectoryEvaluator.cpp assigns every
            // flag and weight below and is the authoritative source.
            // The in-class initialisers here are kept only so a member
            // can never be read uninitialised, and are held IN SYNC
            // with the constructor -- if you change one, change both.
            //////////////////////////////////////////////////////

            //////////////////////////////////////////////////////
            // GRU gap-density trajectory cost
            //////////////////////////////////////////////////////

            ros::Subscriber gruGapFeatureDensitySub_;

            bool useGruGapFeatureDensityCost_ = false;
            double maxGruGapFeaturePredictionAgeSec_ = 1.5;
            float gruGapDensityCostWeight_ = 0.5f;

            mutable boost::mutex gruGapFeatureDensityMutex_;

            std::map<int, GruGapFeatureDensityEstimate>
                latestGruGapFeatureDensityByModelID_;

            //////////////////////////////////////////////////////
            // Bounding the map above.
            //
            // Model IDs are monotonic and never reused, so without
            // an explicit sweep the map grows for the lifetime of
            // the node and the hot-path find() slows down over a
            // long multi-episode run.
            //
            // Interval is measured on the SCAN/message clock, not
            // wall time, to stay consistent with the freshness
            // check under use_sim_time.
            //////////////////////////////////////////////////////

            ros::Time lastGruDensityPruneStamp_;

            double gruDensityPruneIntervalSec_ = 2.0;  ///< min gap between sweeps
            double gruDensityPruneMarginSec_   = 1.0;  ///< grace beyond the freshness horizon

            /**
             * \brief Erases GRU density entries already too stale to be returned.
             *
             * Entries older than maxGruGapFeaturePredictionAgeSec_ (plus
             * gruDensityPruneMarginSec_) are rejected on read anyway, so
             * removing them cannot change any computed cost. Also clears the
             * map outright when the clock jumps backwards, which is what a
             * world reset between episodes looks like.
             *
             * \warning The caller MUST already hold gruGapFeatureDensityMutex_.
             *          boost::mutex is not recursive; this function does not
             *          lock, and locking it again would deadlock.
             *
             * \param currentStamp stamp of the message driving this sweep.
             */
            void pruneStaleGruGapFeatureDensities(
                const ros::Time& currentStamp);

            //////////////////////////////////////////////////////
            // Gap width-clearance trajectory cost (geometric)
            //
            // Replaces the former depth/width aspect-ratio cost. That
            // term was correctly SIGNED (narrow+deep was penalised)
            // but was the only unbounded term in the evaluator, and it
            // penalised depth -- which is reachable progress, not risk.
            // See analysis/code_review_TrajectoryEvaluator.md.
            //
            //   slack = gapWidth - requiredGapWidth()
            //   C_w   = weight * exp(-decay * max(slack, 0))   in [0, weight]
            //////////////////////////////////////////////////////

            bool  useGapWidthClearanceCost_     = true;
            float gapWidthClearanceCostWeight_  = 0.35f;  ///< c_w : max cost, at slack = 0
            float gapWidthClearanceDecayWeight_ = 1.5f;   ///< w_w : 1/w_w = length scale [m]

            //////////////////////////////////////////////////////
            // Hard minimum-width feasibility gate
            //
            // Separate flag from the cost above so the two can be
            // ablated independently. Rejects a candidate gap outright
            // when its MEASURED width cannot admit the robot. Does not
            // fire when the width is unavailable.
            //////////////////////////////////////////////////////

            bool  useGapMinWidthGate_      = true;
            float gapMinWidthSafetyMargin_ = 0.0f;   ///< extra [m] on top of 2*r_infl

            //////////////////////////////////////////////////////
            // Aspect ratio: published diagnostic, not a cost
            //////////////////////////////////////////////////////

            bool publishGapAspectRatioDiagnostic_ = false;

            //////////////////////////////////////////////////////
            // Gap endpoint-proximity trajectory cost (geometric)
            //
            // Two available formulations, selectable so they can be
            // A/B'd on one build.
            //
            // PathNormalised (default):
            //   d_min  = closest approach to either raw endpoint over
            //            the WHOLE path, pose 0 excluded
            //   offset = clamp(1 - d_min/(gapWidth/2), 0, 1)
            //   C_ep   = weight * offset^sharpness      in [0, weight]
            //
            //   Width-invariant, so it measures AIMING rather than
            //   width: a centred pass scores 0 in a 0.5 m gap and in a
            //   3.0 m gap alike. With the gap goal pinned at k = 0.5
            //   this correctly evaluates to 0 for every candidate.
            //
            // TerminalExponential (legacy):
            //   C_ep = weight * exp(-decay * ||p_terminal - nearest||)
            //
            //   Degenerates to weight*exp(-decay*gapWidth/2) when k is
            //   pinned at 0.5 -- i.e. a second ungated width heuristic
            //   with the aiming signal removed -- and scores the end of
            //   a 5 s rollout rather than the closest approach, which
            //   rewards trajectories that abandon their gap. Retained
            //   for comparison only. Set the weight back to 0.5 to
            //   reproduce the original behaviour.
            //////////////////////////////////////////////////////

            enum class EndpointCostForm
            {
                TerminalExponential = 0,  ///< legacy: exponential in terminal-pose distance
                PathNormalised      = 1   ///< closest approach along path, normalised by gap width
            };

            bool  useGapEndpointProximityCost_     = false;
            EndpointCostForm gapEndpointCostForm_  = EndpointCostForm::PathNormalised;
            float gapEndpointProximityCostWeight_  = 0.15f;  ///< c_ep : cost at offset = 1
            float gapEndpointProximitySharpness_   = 2.0f;   ///< exponent on offset (PathNormalised)
            float gapEndpointProximityDecayWeight_ = 3.0f;   ///< w_ep : 1/w_ep length scale [m] (legacy only)

            /**
            * \brief Outcome of classifying a candidate gap's measured mouth width.
            *
            * Deliberately separates "no usable measurement" (fail-soft: skip the
            * width term) from "measured, and too narrow for the robot" (fail-closed:
            * maximum cost, or rejection under the gate). Collapsing the two into a
            * single sentinel is what previously allowed an essentially closed gap to
            * receive the cheapest geometric term of any candidate.
            */
            enum class GapWidthStatus
            {
                Unavailable = 0,  ///< non-finite or negative width; width term skipped
                Impassable  = 1,  ///< measured, narrower than requiredGapWidth()
                Passable    = 2   ///< measured, with positive clearance slack
            };

            /**
            * \brief minimum gap mouth width the robot can pass through
            * \return 2 * (cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) + gapMinWidthSafetyMargin_
            *
            * Single source of truth shared by the minimum-width gate and the
            * width-clearance cost, so the two can never drift apart. Reads r_inscr
            * and inf_ratio live from config, so raising inf_ratio tightens the gate
            * without a rebuild.
            */
            float requiredGapWidth() const;

            /**
            * \brief classify a candidate gap's mouth width
            * \param gapWidth gap mouth width [m]
            * \param slackOut out: metres of clearance beyond requiredGapWidth().
            *        Set to 0 unless the result is Passable.
            * \return Unavailable, Impassable or Passable
            */
            GapWidthStatus classifyGapWidth(const float& gapWidth,
                                            float& slackOut) const;

            /**
            * \brief bounded clearance penalty for a passable gap
            * \param widthSlack metres of clearance beyond requiredGapWidth() (>= 0)
            * \return gapWidthClearanceCostWeight_ * exp(-decay * slack), in [0, weight]
            */
            float gapWidthClearanceCost(const float& widthSlack) const;

            /**
            * \brief aspect ratio (depth / width) of a candidate gap -- DIAGNOSTIC ONLY
            * \param gapWidth gap mouth width [m]
            * \param gapDepth gap radial depth [m]
            * \return depth/width, or -1.0f if geometry is invalid/degenerate
            *
            * No longer contributes to terminalPoseCost. Published through
            * evaluateTrajectory()'s aspectRatioOut purely for visualisation and
            * offline analysis, so a -1.0f here can no longer make a trajectory
            * look cheap.
            */
            float gapAspectRatio(const float& gapWidth,
                                 const float& gapDepth) const;


            /**
            * \brief LEGACY: distance from the trajectory's TERMINAL pose to the
            *        nearer raw gap endpoint. Used only by
            *        EndpointCostForm::TerminalExponential.
            * \return min distance [m], or -1.0f if any input is non-finite
            */
            float gapEndpointMinDistanceToTerminal(
                        const float& termX, const float& termY,
                        const float& gapLeftX, const float& gapLeftY,
                        const float& gapRightX, const float& gapRightY) const;

            /**
            * \brief closest approach to either raw gap endpoint over the whole path
            * \param path candidate trajectory in the robot frame
            * \return min distance [m], or -1.0f if the endpoints are non-finite or
            *         the path has fewer than two poses
            *
            * Pose 0 is deliberately excluded: every candidate in a planning cycle
            * shares the same start pose, so including it can only clamp the minimum
            * down uniformly and destroy discrimination between candidates.
            */
            float gapEndpointMinDistanceAlongPath(
                        const geometry_msgs::PoseArray& path,
                        const float& gapLeftX, const float& gapLeftY,
                        const float& gapRightX, const float& gapRightY) const;

            /**
            * \brief normalised lateral offset of a path within its gap
            * \param minDistAlongPath closest approach to either endpoint [m]
            * \param gapWidth gap mouth width [m], required for normalisation
            * \return clamp(1 - minDistAlongPath/(gapWidth/2), 0, 1), where 0 means
            *         centred or wider and 1 means a pose touched an endpoint;
            *         or -1.0f when it cannot be computed
            *
            * No fallback to an unnormalised scale when gapWidth is unusable -- the
            * term is skipped instead, so the weight always means one thing.
            */
            float gapEndpointOffset(const float& minDistAlongPath,
                                    const float& gapWidth) const;

            /**
            * \brief function for evaluating terminal waypoint cost for candidate trajectory
            * \param pose final pose in candidate trajectory to check against terminal waypoint
            * \return terminal waypoint cost for candidate trajectory
            */
            float terminalGoalCost(const geometry_msgs::Pose & pose);

            /**
            * \brief function for evaluating intermediate cost of pose for candidate trajectory (in static environment)
            * \param pose pose within candidate trajectory to evaluate
            * \param scan_k future scan to evaluate this pose against
            * \return intermediate cost of pose
            *
            * scan_k is taken by const reference: it was previously passed by value,
            * copying an entire LaserScan once per pose, per candidate trajectory,
            * per planning cycle.
            */
            float evaluatePose(const geometry_msgs::Pose & pose,
                                const sensor_msgs::LaserScan & scan_k) ;

            /**
            * \brief function for calculating intermediate trajectory cost (in static environment)
            * \param rbtToScanDist minimum distance from robot pose to current scan
            * \return intermediate cost of pose
            */
            float chapterCost(const float & rbtToScanDist);

            boost::mutex globalPlanMutex_; /**< mutex locking thread for updating current global plan */
            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan (guards scan_ only; evaluatePose does not take it) */

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            const DynamicGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */

            // sensor_msgs::LaserScan staticScan_;
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_; /**< Current local waypoint along global plan in robot frame */
    };
}
