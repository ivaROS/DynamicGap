//////////////////////////////////////////////////////////////////////
// GapEndpointWeightFix_TrajectoryEvaluator.cpp
//
// ALTERNATIVE to TrajectoryEvaluator.cpp -- NOT an addition.
// It defines the same dynamic_gap::TrajectoryEvaluator members, so
// exactly one of the two may be in the build at a time or you will
// get duplicate-symbol link errors. Swap the filename in
// CMakeLists.txt; do not add both.
//
// Delta versus TrajectoryEvaluator.cpp: the gap endpoint-proximity
// term is reformulated.
//
//   Old:  C_ep = w * exp(-decay * ||p_terminal - nearest endpoint||)
//
//   Two problems, both visible when k is pinned at 0.5:
//
//   (a) IT MEASURED WIDTH, NOT AIMING. The term exists to punish gap
//       goals that hug an endpoint (k -> 0 or 1). With
//       p_g = k*p_l + (1-k)*p_r, the distance to the nearer endpoint
//       is min(k, 1-k) * gapWidth. Fix k at 0.5 and that collapses to
//       gapWidth/2 for every candidate -- the aiming signal vanishes
//       and the term degenerates into a second, ungated width
//       heuristic that saturates by ~2 m of width.
//
//   (b) IT SCORED THE WRONG POINT. path.poses.back() is the end of a
//       5 s rollout (integrate_maxt), up to ~6 m of travel. A
//       trajectory that abandons the gap entirely ends far from both
//       endpoints and is rewarded for it, while one that correctly
//       threads the gap and stops just past it is charged more.
//
//   New:  d_min  = closest approach to either endpoint over the WHOLE
//                  path (pose 0 excluded)
//         offset = clamp(1 - d_min / (gapWidth/2), 0, 1)
//         C_ep   = w * offset^sharpness
//
//         offset = 0  -> passed no closer than half the gap width,
//                        i.e. centred or better. No cost.
//         offset = 1  -> a pose touched an endpoint.
//
//   This is width-invariant by construction: a centred pass through a
//   0.5 m gap and a centred pass through a 3.0 m gap both score 0. So
//   with k = 0.5 the term correctly does NOTHING, which is the null
//   result the previous form failed to produce. It can therefore be
//   swept alongside the width-clearance term instead of mutually
//   exclusive with it.
//
//   Residual correlation with width is real but no longer spurious:
//   10 cm off-centre gives offset 0.40 in a 0.5 m gap and 0.07 in a
//   3.0 m gap. Tight gaps genuinely punish lateral error harder.
//
//   The legacy form is retained behind gapEndpointCostForm_ so the
//   two can be A/B'd directly.
//////////////////////////////////////////////////////////////////////

#include <dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h>

#include <algorithm>
#include <cmath>
#include <limits>


namespace dynamic_gap
{
    TrajectoryEvaluator::TrajectoryEvaluator(
    ros::NodeHandle& nh,
    const DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;

        //////////////////////////////////////////////////////
        // GRU gap-feature density subscriber
        //////////////////////////////////////////////////////

        useGruGapFeatureDensityCost_ = false;
        maxGruGapFeaturePredictionAgeSec_ = 1.5;
        gruGapDensityCostWeight_ = 0.4f;

        const std::string gruGapFeatureDensityTopic =
            "/rto/gru_gap_feature_prediction";

        gruGapFeatureDensitySub_ =
            nh.subscribe(
                gruGapFeatureDensityTopic,
                100,
                &TrajectoryEvaluator::gruGapFeatureDensityCB,
                this
            );

        ROS_WARN_STREAM_NAMED(
            "GRUGapFeatureDensityCost",
            "TrajectoryEvaluator GRU density cost enabled: "
            << useGruGapFeatureDensityCost_
            << ", topic: "
            << gruGapFeatureDensityTopic
            << ", max age: "
            << maxGruGapFeaturePredictionAgeSec_
            << ", cost weight: "
            << gruGapDensityCostWeight_
        );

        //////////////////////////////////////////////////////
        // Gap width-clearance terminal cost (geometric)
        //
        // Replaces the former depth/width aspect-ratio cost. That term
        // was correctly SIGNED (narrow+deep was penalised) but was the
        // only unbounded term in the evaluator, and it penalised depth
        // -- which is reachable progress, not risk.
        //
        //   slack = gapWidth - requiredGapWidth()
        //   C_w   = weight * exp(-decay * max(slack, 0))   in [0, weight]
        //////////////////////////////////////////////////////

        useGapWidthClearanceCost_       = false;
        gapWidthClearanceCostWeight_    = 0.35f;  // c_w : max cost at slack = 0
        gapWidthClearanceDecayWeight_   = 1.5f;   // w_w : 1/w_w = length scale (m)

        //////////////////////////////////////////////////////
        // Hard minimum-width feasibility gate
        //
        //   reject iff gapWidth < 2*(r_inscr*inf_ratio) + margin
        //
        // Keep this ON when sweeping the endpoint term. The endpoint
        // cost is a ranking nudge with nothing underneath it: on its
        // own it can reorder candidates but can never reject one the
        // robot does not fit through.
        //////////////////////////////////////////////////////

        useGapMinWidthGate_       = true;
        gapMinWidthSafetyMargin_  = 0.0f;   // extra metres on top of 2*r_infl

        //////////////////////////////////////////////////////
        // Aspect-ratio DIAGNOSTIC (no longer a cost)
        //////////////////////////////////////////////////////

        publishGapAspectRatioDiagnostic_ = false;

        ROS_WARN_STREAM_NAMED(
            "GapWidthClearanceCost",
            "TrajectoryEvaluator gap width-clearance cost enabled: "
            << useGapWidthClearanceCost_
            << ", cost weight: "
            << gapWidthClearanceCostWeight_
            << ", decay weight: "
            << gapWidthClearanceDecayWeight_
            << " | min-width gate enabled: "
            << useGapMinWidthGate_
            << ", extra margin: "
            << gapMinWidthSafetyMargin_
            << " | aspect-ratio diagnostic: "
            << publishGapAspectRatioDiagnostic_
        );

        //////////////////////////////////////////////////////
        // Gap endpoint-proximity terminal cost (geometric)
        //
        // See the file header for the reformulation and why.
        //
        // PathNormalised (default):
        //   C_ep = weight * clamp(1 - d_min/(gapWidth/2), 0, 1)^sharpness
        //   d_min = closest approach to either raw endpoint over the
        //           whole path, excluding pose 0.
        //   Requires a finite positive gapWidth to normalise against;
        //   without one the term is skipped rather than silently
        //   falling back to a different scale.
        //
        // TerminalExponential (legacy, for A/B):
        //   C_ep = weight * exp(-decay * ||p_terminal - nearest endpoint||)
        //   To reproduce the original run set the weight back to 0.5.
        //
        // WEIGHT. 0.15, down from 0.5. At Q_f = 0.05 the goal term's
        // whole dynamic range is about 0.1 cost units, so 0.5 gave a
        // pure ranking nudge five times the authority of the objective
        // it was nudging. 0.15 keeps it at roughly a third of the goal
        // term. It is bounded, so 0.15 is the exact worst case.
        //
        // Still DEFAULT OFF. Turn on deliberately, one form at a time.
        //////////////////////////////////////////////////////

        useGapEndpointProximityCost_     = false;
        gapEndpointCostForm_             = EndpointCostForm::PathNormalised;
        gapEndpointProximityCostWeight_  = 0.5f;  // c_ep : cost at offset = 1
        gapEndpointProximitySharpness_   = 2.0f;   // exponent on offset (PathNormalised)
        gapEndpointProximityDecayWeight_ = 3.0f;   // w_ep : 1/w_ep length scale (legacy only)

        ROS_WARN_STREAM_NAMED(
            "GapEndpointProximityCost",
            "TrajectoryEvaluator gap endpoint-proximity cost enabled: "
            << useGapEndpointProximityCost_
            << ", form: "
            << (gapEndpointCostForm_ == EndpointCostForm::PathNormalised
                    ? "PathNormalised" : "TerminalExponential")
            << ", cost weight: "
            << gapEndpointProximityCostWeight_
            << ", sharpness: "
            << gapEndpointProximitySharpness_
            << ", legacy decay weight: "
            << gapEndpointProximityDecayWeight_
        );
    }

    void TrajectoryEvaluator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan)
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    void TrajectoryEvaluator::transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame,
                                                                            const geometry_msgs::TransformStamped & odom2rbt)
    {
        boost::mutex::scoped_lock lock(globalPlanMutex_);
        tf2::doTransform(globalPathLocalWaypointOdomFrame, globalPathLocalWaypointRobotFrame_, odom2rbt);
    }


    //////////////////////////////////////////////////////
    // Drop GRU density entries that are already too old to be
    // returned by getLatestGruGapFeatureDensityForModel().
    //
    // PRECONDITION: gruGapFeatureDensityMutex_ is already held by
    // the caller. This function does NOT take the lock itself --
    // it is called from inside the locked block in
    // gruGapFeatureDensityCB(), and boost::mutex is not
    // recursive, so locking again here would deadlock.
    //
    // Uses the message clock, not ros::Time::now(), for the same
    // reason the freshness check does: under use_sim_time with a
    // real-time factor well below 1, wall time and sim time
    // diverge badly and a wall-clock sweep would erase live data.
    //////////////////////////////////////////////////////

    void TrajectoryEvaluator::pruneStaleGruGapFeatureDensities(
        const ros::Time& currentStamp)
    {
        //////////////////////////////////////////////////////
        // A zero stamp means the publisher did not set a header.
        // Ages computed against it are meaningless, so skip the
        // sweep rather than erase the whole map.
        //////////////////////////////////////////////////////

        if (currentStamp.isZero())
            return;

        //////////////////////////////////////////////////////
        // Rate-limit. Sweeping on every callback would be
        // O(n) per message; this keeps it O(n) per interval.
        //////////////////////////////////////////////////////

        if (!lastGruDensityPruneStamp_.isZero())
        {
            const double sinceLastPrune =
                (currentStamp - lastGruDensityPruneStamp_).toSec();

            //////////////////////////////////////////////////////
            // Negative means the clock jumped backwards, which is
            // exactly what a Gazebo world reset between episodes
            // looks like. Every stamp in the map is now from the
            // previous episode, so clear it outright.
            //////////////////////////////////////////////////////

            if (sinceLastPrune < 0.0)
            {
                ROS_WARN_STREAM_NAMED(
                    "GRUGapFeatureDensityCost",
                    "scan clock moved backwards ("
                    << sinceLastPrune
                    << "s), clearing "
                    << latestGruGapFeatureDensityByModelID_.size()
                    << " GRU density entries from the previous episode"
                );

                latestGruGapFeatureDensityByModelID_.clear();
                lastGruDensityPruneStamp_ = currentStamp;

                return;
            }

            if (sinceLastPrune < gruDensityPruneIntervalSec_)
                return;
        }

        lastGruDensityPruneStamp_ = currentStamp;

        //////////////////////////////////////////////////////
        // Erase anything past the freshness horizon. The extra
        // margin means an entry is only dropped once it is
        // comfortably unreadable, never on the boundary.
        //////////////////////////////////////////////////////

        const double staleAfterSec =
            maxGruGapFeaturePredictionAgeSec_ +
            gruDensityPruneMarginSec_;

        const std::size_t sizeBefore =
            latestGruGapFeatureDensityByModelID_.size();

        for (auto it = latestGruGapFeatureDensityByModelID_.begin();
             it != latestGruGapFeatureDensityByModelID_.end(); )
        {
            const double age =
                std::abs((currentStamp - it->second.stamp).toSec());

            if (age > staleAfterSec)
                it = latestGruGapFeatureDensityByModelID_.erase(it);
            else
                ++it;
        }

        const std::size_t sizeAfter =
            latestGruGapFeatureDensityByModelID_.size();

        if (sizeAfter != sizeBefore)
        {
            ROS_DEBUG_STREAM_NAMED(
                "GRUGapFeatureDensityCost",
                "pruned GRU density map | erased="
                << (sizeBefore - sizeAfter)
                << " remaining="
                << sizeAfter
                << " stale_after="
                << staleAfterSec
                << "s"
            );
        }
    }

    bool TrajectoryEvaluator::getLatestGruGapFeatureDensityForModel(
    const int& modelID,
    const ros::Time& currentStamp,
    float& predDensityOut) const
    {
        boost::mutex::scoped_lock lock(
            gruGapFeatureDensityMutex_
        );

        auto it =
            latestGruGapFeatureDensityByModelID_.find(modelID);

        if (it ==
            latestGruGapFeatureDensityByModelID_.end())
        {
            return false;
        }

        const GruGapFeatureDensityEstimate& estimate =
            it->second;

        if (!estimate.valid)
            return false;

        const double age =
            std::abs(
                (currentStamp - estimate.stamp).toSec()
            );

        if (age > maxGruGapFeaturePredictionAgeSec_)
            return false;

        if (!std::isfinite(estimate.pred_sector_density))
            return false;

        //////////////////////////////////////////////////////
        // Prevent slightly negative network outputs from
        // reducing trajectory cost.
        //////////////////////////////////////////////////////

        predDensityOut =
            std::max(
                0.0f,
                estimate.pred_sector_density
            );

        return true;
    }

    //////////////////////////////////////////////////////
    // Minimum width the robot can physically pass through:
    //
    //   2 * (r_inscr * inf_ratio) + safety margin
    //////////////////////////////////////////////////////

    float TrajectoryEvaluator::requiredGapWidth() const
    {
        const float inflRbtRad =
            cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

        return 2.0f * inflRbtRad + gapMinWidthSafetyMargin_;
    }

    //////////////////////////////////////////////////////
    // Classify a candidate gap's width. Separates "geometry
    // unavailable" (fail-soft) from "measured and too narrow"
    // (fail-closed).
    //////////////////////////////////////////////////////

    TrajectoryEvaluator::GapWidthStatus
    TrajectoryEvaluator::classifyGapWidth(
    const float& gapWidth,
    float& slackOut) const
    {
        slackOut = 0.0f;

        if (!std::isfinite(gapWidth) || gapWidth < 0.0f)
        {
            ROS_WARN_STREAM_THROTTLE_NAMED(
                5.0,
                "GapWidthClearanceCost",
                "gap width unavailable or negative (gapWidth="
                << gapWidth
                << "), skipping width-clearance term"
            );

            return GapWidthStatus::Unavailable;
        }

        const float slack = gapWidth - requiredGapWidth();

        if (slack <= 0.0f)
            return GapWidthStatus::Impassable;

        slackOut = slack;

        return GapWidthStatus::Passable;
    }

    //////////////////////////////////////////////////////
    // Bounded width-clearance cost, in [0, weight].
    //////////////////////////////////////////////////////

    float TrajectoryEvaluator::gapWidthClearanceCost(
    const float& widthSlack) const
    {
        const float slack = std::max(0.0f, widthSlack);

        return gapWidthClearanceCostWeight_ *
               std::exp(-gapWidthClearanceDecayWeight_ * slack);
    }

    //////////////////////////////////////////////////////
    // Gap aspect ratio = depth / width -- DIAGNOSTIC ONLY.
    //////////////////////////////////////////////////////

    float TrajectoryEvaluator::gapAspectRatio(
    const float& gapWidth,
    const float& gapDepth) const
    {
        const float minValidWidth = 1e-3f;

        if (!std::isfinite(gapWidth) ||
            !std::isfinite(gapDepth) ||
            gapWidth <= minValidWidth ||
            gapDepth < 0.0f)
        {
            return -1.0f;
        }

        return gapDepth / gapWidth;
    }

    //////////////////////////////////////////////////////
    // LEGACY: distance from the trajectory's TERMINAL pose to the
    // nearer raw gap endpoint. Retained only so the original
    // formulation can be A/B'd against the new one; see the file
    // header for why it measures the wrong point.
    //////////////////////////////////////////////////////

    float TrajectoryEvaluator::gapEndpointMinDistanceToTerminal(
    const float& termX,
    const float& termY,
    const float& gapLeftX,
    const float& gapLeftY,
    const float& gapRightX,
    const float& gapRightY) const
    {
        if (!std::isfinite(termX)    || !std::isfinite(termY)    ||
            !std::isfinite(gapLeftX) || !std::isfinite(gapLeftY) ||
            !std::isfinite(gapRightX)|| !std::isfinite(gapRightY))
        {
            return -1.0f;
        }

        const float dLx = termX - gapLeftX;
        const float dLy = termY - gapLeftY;
        const float dRx = termX - gapRightX;
        const float dRy = termY - gapRightY;

        const float distToLeft  = std::sqrt(dLx * dLx + dLy * dLy);
        const float distToRight = std::sqrt(dRx * dRx + dRy * dRy);

        return std::min(distToLeft, distToRight);
    }

    //////////////////////////////////////////////////////
    // Closest approach to EITHER raw gap endpoint over the whole
    // path -- the quantity the endpoint term should have been
    // measuring all along. "Did you scrape the doorframe on the way
    // through", not "where did you happen to stop".
    //
    // Pose 0 is deliberately EXCLUDED. Every candidate trajectory in
    // a planning cycle starts from the same robot pose, so including
    // it can only clamp the minimum down by the same amount for all
    // of them -- pure loss of discrimination. If the robot really is
    // about to clip an endpoint, pose 1 (one integration step later)
    // reports it.
    //
    // Returns -1.0f if the endpoint geometry is non-finite or the
    // path is too short to have a pose past the start.
    //////////////////////////////////////////////////////

    float TrajectoryEvaluator::gapEndpointMinDistanceAlongPath(
    const geometry_msgs::PoseArray& path,
    const float& gapLeftX,
    const float& gapLeftY,
    const float& gapRightX,
    const float& gapRightY) const
    {
        if (!std::isfinite(gapLeftX) || !std::isfinite(gapLeftY) ||
            !std::isfinite(gapRightX)|| !std::isfinite(gapRightY))
        {
            return -1.0f;
        }

        if (path.poses.size() < 2)
            return -1.0f;

        float best = std::numeric_limits<float>::infinity();

        for (size_t i = 1; i < path.poses.size(); ++i)
        {
            const float px = static_cast<float>(path.poses.at(i).position.x);
            const float py = static_cast<float>(path.poses.at(i).position.y);

            if (!std::isfinite(px) || !std::isfinite(py))
                continue;

            const float dLx = px - gapLeftX;
            const float dLy = py - gapLeftY;
            const float dRx = px - gapRightX;
            const float dRy = py - gapRightY;

            const float dL = std::sqrt(dLx * dLx + dLy * dLy);
            const float dR = std::sqrt(dRx * dRx + dRy * dRy);

            best = std::min(best, std::min(dL, dR));
        }

        if (!std::isfinite(best))
            return -1.0f;

        return best;
    }

    //////////////////////////////////////////////////////
    // Normalised lateral offset of a path within its gap.
    //
    //   offset = clamp(1 - d_min / (gapWidth/2), 0, 1)
    //
    //     0 -> never came closer to an endpoint than half the gap
    //          width: centred, or wider of the endpoints than a
    //          centred pass would be. No penalty.
    //     1 -> a pose touched an endpoint.
    //
    // Width-invariant: a centred pass scores 0 in a 0.5 m gap and in
    // a 3.0 m gap alike. This is what makes the term measure AIMING
    // rather than width, and what makes a k = 0.5 sweep correctly
    // produce no effect.
    //
    // Returns -1.0f when it cannot be computed (no usable width, or
    // no usable distance), which the caller treats as "term skipped".
    // No fallback to an unnormalised scale -- mixing the two would
    // make the weight mean two different things.
    //////////////////////////////////////////////////////

    float TrajectoryEvaluator::gapEndpointOffset(
    const float& minDistAlongPath,
    const float& gapWidth) const
    {
        const float minValidWidth = 1e-3f;

        if (!std::isfinite(gapWidth) || gapWidth <= minValidWidth)
        {
            ROS_WARN_STREAM_THROTTLE_NAMED(
                5.0,
                "GapEndpointProximityCost",
                "gap width unusable (gapWidth="
                << gapWidth
                << "), cannot normalise endpoint offset, skipping term"
            );

            return -1.0f;
        }

        if (!std::isfinite(minDistAlongPath) || minDistAlongPath < 0.0f)
            return -1.0f;

        const float halfWidth = 0.5f * gapWidth;

        float offset = 1.0f - (minDistAlongPath / halfWidth);

        return std::min(1.0f, std::max(0.0f, offset));
    }

    void TrajectoryEvaluator::gruGapFeatureDensityCB(
    const dynamic_gap::GapFeaturePrediction::ConstPtr& msg)
    {
        if (!msg->valid)
        {
            ROS_INFO_STREAM_NAMED(
                "GRUGapFeatureDensityCost",
                "received invalid GRU density prediction for model "
                << msg->model_id
                << " side="
                << msg->side
                << ", ignoring"
            );

            return;
        }

        if (msg->output_names.size() != msg->output_values.size())
        {
            ROS_WARN_STREAM_NAMED(
                "GRUGapFeatureDensityCost",
                "output_names/output_values size mismatch for model "
                << msg->model_id
                << ", ignoring"
            );

            return;
        }

        bool foundDensity = false;
        float predDensity = 0.0f;

        for (size_t i = 0; i < msg->output_names.size(); ++i)
        {
            if (msg->output_names.at(i) == "gt_sector_density" ||
                msg->output_names.at(i).find("future_sector_density") != std::string::npos)
            {
                predDensity = msg->output_values.at(i);
                foundDensity = true;
                break;
            }
        }

        if (!foundDensity)
        {
            ROS_WARN_STREAM_NAMED(
                "GRUGapFeatureDensityCost",
                "GRU prediction for model "
                << msg->model_id
                << " did not contain gt_sector_density, ignoring"
            );

            return;
        }

        if (!std::isfinite(predDensity))
        {
            ROS_WARN_STREAM_NAMED(
                "GRUGapFeatureDensityCost",
                "received non-finite predicted density for model "
                << msg->model_id
                << ", ignoring"
            );

            return;
        }

        GruGapFeatureDensityEstimate estimate;
        estimate.pred_sector_density = predDensity;
        estimate.stamp = msg->header.stamp;
        estimate.valid = msg->valid;
        estimate.seq_len_used = msg->seq_len_used;

        {
            boost::mutex::scoped_lock lock(
                gruGapFeatureDensityMutex_
            );

            latestGruGapFeatureDensityByModelID_[
                msg->model_id
            ] = estimate;

            //////////////////////////////////////////////////////
            // Prune stale entries.
            //
            // Gap model IDs are assigned monotonically as gaps are
            // instantiated and are never reused, so without this
            // the map grows for the entire lifetime of the node.
            // Individual entries are small, but the map is read on
            // the planning hot path via
            // getLatestGruGapFeatureDensityForModel(), so an
            // ever-growing map makes late episodes measurably
            // slower than early ones within a single process --
            // which biases a multi-episode benchmark rather than
            // just slowing it down.
            //
            // Anything older than maxGruGapFeaturePredictionAgeSec_
            // would be rejected on read anyway, so dropping it here
            // cannot change any cost that would have been computed.
            // A margin is applied so an entry is never erased in
            // the same instant it becomes unreadable.
            //
            // Amortised: the sweep runs on a message whose stamp
            // has advanced past the last sweep, not on every
            // message.
            //////////////////////////////////////////////////////

            pruneStaleGruGapFeatureDensities(estimate.stamp);
        }

        ROS_INFO_STREAM_NAMED(
            "GRUGapFeatureDensityCost",
            "stored GRU density prediction | model_id="
            << msg->model_id
            << " side="
            << msg->side
            << " pred_sector_density="
            << estimate.pred_sector_density
            << " seq_len="
            << estimate.seq_len_used
        );
    }

    void TrajectoryEvaluator::evaluateTrajectory(
    const Trajectory & traj,
    std::vector<float> & posewiseCosts,
    float & terminalPoseCost,
    const std::vector<sensor_msgs::LaserScan> & futureScans,
    const int & scanIdx,
    const int & densityModelID,
        float* terminalGoalCostOnlyOut,
    float gapWidth,
    float gapDepth,
    float* aspectRatioOut,
    float gapLeftX,
    float gapLeftY,
    float gapRightX,
    float gapRightY,
    bool haveGapEndpoints,
    float* endpointProximityCostOut
)
    {
        //////////////////////////////////////////////////////
        // 0. Defensive initialisation of ALL outputs, so no code path
        //    can leave a stale or uninitialised value behind.
        //////////////////////////////////////////////////////

        const float INF = std::numeric_limits<float>::infinity();

        terminalPoseCost = INF;

        if (terminalGoalCostOnlyOut)   *terminalGoalCostOnlyOut   = INF;
        if (aspectRatioOut)            *aspectRatioOut            = -1.0f;
        if (endpointProximityCostOut)  *endpointProximityCostOut  = 0.0f;

        try
        {
            ROS_INFO_STREAM_NAMED(
                "TrajectoryEvaluator",
                "         [evaluateTrajectory()]"
            );

            //////////////////////////////////////////////////////
            // Requires LOCAL FRAME
            //////////////////////////////////////////////////////

            geometry_msgs::PoseArray path =
                traj.getPathRbtFrame();

            std::vector<float> pathTiming =
                traj.getPathTiming();

            posewiseCosts =
                std::vector<float>(path.poses.size());

            if (path.poses.size() == 0)
            {
                ROS_WARN_STREAM_NAMED(
                    "TrajectoryEvaluator",
                    "            empty trajectory passed to evaluateTrajectory()"
                );

                return;   // terminalPoseCost already INF
            }

           if (path.poses.size() > futureScans.size())
            {
                ROS_WARN_STREAM_NAMED(
                    "TrajectoryEvaluator",
                    "            posewiseCosts-futureScans size mismatch: "
                    << "path size="
                    << path.poses.size()
                    << ", scanIdx="
                    << scanIdx
                    << ", futureScans size="
                    << futureScans.size()
                );

                return;
            }

            if (posewiseCosts.size() != path.poses.size())
            {
                ROS_WARN_STREAM_NAMED(
                    "TrajectoryEvaluator",
                    "            posewiseCosts-pathPoses size mismatch: "
                    << posewiseCosts.size()
                    << " vs "
                    << path.poses.size()
                );

                return;
            }

            //////////////////////////////////////////////////////
            // 1. Aspect-ratio diagnostic (no cost contribution)
            //////////////////////////////////////////////////////

            float aspectRatio = -1.0f;

            if (publishGapAspectRatioDiagnostic_)
                aspectRatio = gapAspectRatio(gapWidth, gapDepth);

            if (aspectRatioOut)
                *aspectRatioOut = aspectRatio;

            //////////////////////////////////////////////////////
            // 2. Width classification + hard min-width gate,
            //    before the O(poses x ranges) posewise loop.
            //////////////////////////////////////////////////////

            float widthSlack = 0.0f;

            const GapWidthStatus widthStatus =
                classifyGapWidth(gapWidth, widthSlack);

            const float baseTerminalGoalCost =
                cfg_->traj.Q_f *
                terminalGoalCost(path.poses.back());

            if (terminalGoalCostOnlyOut)
                *terminalGoalCostOnlyOut = baseTerminalGoalCost;

            if (useGapMinWidthGate_ &&
                widthStatus == GapWidthStatus::Impassable)
            {
                ROS_INFO_STREAM_NAMED(
                    "GapWidthClearanceCost",
                    "min-width gate REJECT | gap_width="
                    << gapWidth
                    << " required_width="
                    << requiredGapWidth()
                    << " (r_inscr="
                    << cfg_->rbt.r_inscr
                    << ", inf_ratio="
                    << cfg_->traj.inf_ratio
                    << ", margin="
                    << gapMinWidthSafetyMargin_
                    << ")"
                );

                return;   // terminalPoseCost stays INF
            }

            //////////////////////////////////////////////////////
            // 3. Posewise obstacle/chapter cost
            //////////////////////////////////////////////////////

            for (int i = 0;
                i < static_cast<int>(posewiseCosts.size());
                i++)
            {
                ROS_INFO_STREAM_NAMED(
                    "TrajectoryEvaluator",
                    "           pose "
                    << i
                    << " (total scan idx: "
                    << (scanIdx + i)
                    << "): "
                );

                posewiseCosts.at(i) =
                    evaluatePose(
                        path.poses.at(i),
                        futureScans.at(scanIdx + i)
                    );
            }

            float averagePosewiseCost =
                std::accumulate(
                    posewiseCosts.begin(),
                    posewiseCosts.end(),
                    float(0)
                ) / posewiseCosts.size();

            ROS_INFO_STREAM_NAMED(
                "TrajectoryEvaluator",
                "             avg pose-wise cost: "
                << averagePosewiseCost
            );

            //////////////////////////////////////////////////////
            // 4. Fresh GRU gap-density cost. Freshness is measured
            //    against the scan clock, not ros::Time::now(), since
            //    wall time and sim time diverge under use_sim_time
            //    with a real-time factor well below 1.
            //////////////////////////////////////////////////////

            float predGruDensity = 0.0f;
            float weightedGruDensityCost = 0.0f;
            bool usedGruDensity = false;

            ros::Time currentStamp =
                futureScans.at(scanIdx).header.stamp;

            if (currentStamp.isZero())
                currentStamp = ros::Time::now();

            if (useGruGapFeatureDensityCost_ &&
                densityModelID >= 0)
            {
                bool haveFreshGruDensity =
                    getLatestGruGapFeatureDensityForModel(
                        densityModelID,
                        currentStamp,
                        predGruDensity
                    );

                if (haveFreshGruDensity)
                {
                    weightedGruDensityCost =
                        gruGapDensityCostWeight_ *
                        predGruDensity;

                    usedGruDensity = true;
                }
            }

            //////////////////////////////////////////////////////
            // 5. Gap width-clearance cost (geometric), in [0, weight].
            //    Depth plays no part.
            //////////////////////////////////////////////////////

            float weightedWidthClearanceCost = 0.0f;
            bool usedWidthClearance = false;

            if (useGapWidthClearanceCost_)
            {
                switch (widthStatus)
                {
                    case GapWidthStatus::Passable:
                        weightedWidthClearanceCost =
                            gapWidthClearanceCost(widthSlack);
                        usedWidthClearance = true;
                        break;

                    case GapWidthStatus::Impassable:
                        weightedWidthClearanceCost =
                            gapWidthClearanceCostWeight_;
                        usedWidthClearance = true;
                        break;

                    case GapWidthStatus::Unavailable:
                    default:
                        weightedWidthClearanceCost = 0.0f;
                        usedWidthClearance = false;
                        break;
                }
            }

            //////////////////////////////////////////////////////
            // 6. Gap endpoint-proximity cost (geometric)
            //
            // PathNormalised (default): closest approach to either
            // endpoint over the whole path, normalised by half the gap
            // width, raised to `sharpness`, scaled by weight. Bounded
            // in [0, weight]. Measures AIMING, not width -- a centred
            // pass scores 0 at any gap size.
            //
            // TerminalExponential: the original formulation, kept so
            // the two can be compared directly on the same build.
            //////////////////////////////////////////////////////

            float endpointMinDist   = -1.0f;
            float endpointOffset    = -1.0f;
            float weightedEndpointCost = 0.0f;
            bool  usedEndpointProximity = false;

            if (useGapEndpointProximityCost_ && haveGapEndpoints)
            {
                if (gapEndpointCostForm_ == EndpointCostForm::PathNormalised)
                {
                    endpointMinDist =
                        gapEndpointMinDistanceAlongPath(
                            path,
                            gapLeftX,
                            gapLeftY,
                            gapRightX,
                            gapRightY
                        );

                    endpointOffset =
                        gapEndpointOffset(endpointMinDist, gapWidth);

                    if (endpointOffset >= 0.0f)
                    {
                        weightedEndpointCost =
                            gapEndpointProximityCostWeight_ *
                            std::pow(
                                endpointOffset,
                                gapEndpointProximitySharpness_
                            );

                        usedEndpointProximity = true;
                    }
                }
                else   // EndpointCostForm::TerminalExponential (legacy)
                {
                    const geometry_msgs::Pose& termPose =
                        path.poses.back();

                    endpointMinDist =
                        gapEndpointMinDistanceToTerminal(
                            static_cast<float>(termPose.position.x),
                            static_cast<float>(termPose.position.y),
                            gapLeftX,
                            gapLeftY,
                            gapRightX,
                            gapRightY
                        );

                    if (endpointMinDist >= 0.0f)
                    {
                        weightedEndpointCost =
                            gapEndpointProximityCostWeight_ *
                            std::exp(
                                -gapEndpointProximityDecayWeight_ *
                                endpointMinDist
                            );

                        usedEndpointProximity = true;
                    }
                }
            }

            if (endpointProximityCostOut)
            {
                *endpointProximityCostOut = weightedEndpointCost;
            }

            //////////////////////////////////////////////////////
            // 7. Final terminalPoseCost returned to caller
            //
            //    base (Q_f goal) + density + width-clearance
            //                    + endpoint-proximity
            //
            // Every geometric term is bounded by its own weight. With
            // the defaults above the geometric ceiling is 0.35 (width)
            // + 0.15 (endpoint) = 0.50, against a Q_f * dist goal term
            // of roughly 0.1 at Q_f = 0.05. Raising Q_f is a separate
            // config change and is still recommended.
            //////////////////////////////////////////////////////

            terminalPoseCost =
                baseTerminalGoalCost +
                weightedGruDensityCost +
                weightedWidthClearanceCost +
                weightedEndpointCost;

            ROS_INFO_STREAM_NAMED(
                "GapWidthClearanceCost",
                "width-clearance terminal cost | "
                << "gap_width=" << gapWidth
                << " required_width=" << requiredGapWidth()
                << " width_slack=" << widthSlack
                << " width_status=" << static_cast<int>(widthStatus)
                << " used_width_clearance=" << usedWidthClearance
                << " weighted_width_clearance_cost=" << weightedWidthClearanceCost
                << " base_terminal_goal_cost=" << baseTerminalGoalCost
                << " weighted_density_cost=" << weightedGruDensityCost
                << " final_terminal_pose_cost=" << terminalPoseCost
            );

            ROS_INFO_STREAM_NAMED(
                "GapAspectRatioCost",
                "aspect-ratio diagnostic (no cost contribution) | "
                << "gap_width=" << gapWidth
                << " gap_depth=" << gapDepth
                << " aspect_ratio=" << aspectRatio
            );

            //////////////////////////////////////////////////////
            // endpoint_min_dist and endpoint_offset are the two
            // numbers to watch when sweeping this term.
            //
            //   offset near 0 across the board  -> the term is inert,
            //       every candidate is passing centred or wider.
            //   offset near 1 on the SELECTED gap -> the planner is
            //       choosing paths that graze a doorframe.
            //   min_dist consistently larger than gap_width in the
            //       legacy form -> trajectories are being rewarded for
            //       abandoning their gap. That is the failure mode the
            //       PathNormalised form removes.
            //////////////////////////////////////////////////////

            ROS_INFO_STREAM_NAMED(
                "GapEndpointProximityCost",
                "endpoint-proximity terminal cost | "
                << "form="
                << (gapEndpointCostForm_ == EndpointCostForm::PathNormalised
                        ? "PathNormalised" : "TerminalExponential")
                << " have_endpoints=" << haveGapEndpoints
                << " gap_width=" << gapWidth
                << " endpoint_min_dist=" << endpointMinDist
                << " endpoint_offset=" << endpointOffset
                << " used_endpoint_proximity=" << usedEndpointProximity
                << " weighted_endpoint_cost=" << weightedEndpointCost
                << " final_terminal_pose_cost=" << terminalPoseCost
            );

            ROS_INFO_STREAM_NAMED(
                "TrajectoryEvaluator",
                "            terminal cost: "
                << terminalPoseCost
            );
        }
        catch (const std::out_of_range& e)
        {
            ROS_WARN_STREAM_NAMED(
                "TrajectoryEvaluator",
                "            evaluateTrajectory out of range exception: "
                << e.what()
            );

            terminalPoseCost = INF;

            std::fill(posewiseCosts.begin(), posewiseCosts.end(), INF);
        }
        catch (const std::exception& e)
        {
            ROS_WARN_STREAM_NAMED(
                "TrajectoryEvaluator",
                "            evaluateTrajectory exception: "
                << e.what()
            );

            terminalPoseCost = INF;

            std::fill(posewiseCosts.begin(), posewiseCosts.end(), INF);
        }

        return;
    }

    float TrajectoryEvaluator::terminalGoalCost(const geometry_msgs::Pose & pose)
    {
        boost::mutex::scoped_lock planlock(globalPlanMutex_);
        // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", pose);
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            final pose: (" << pose.position.x << ", " << pose.position.y << "), local goal: (" << globalPathLocalWaypointRobotFrame_.pose.position.x << ", " << globalPathLocalWaypointRobotFrame_.pose.position.y << ")");
        float dx = pose.position.x - globalPathLocalWaypointRobotFrame_.pose.position.x;
        float dy = pose.position.y - globalPathLocalWaypointRobotFrame_.pose.position.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    //////////////////////////////////////////////////////
    // Pose-wise obstacle cost.
    //
    // scan_k is taken by const reference (was by value, copying an
    // entire LaserScan per pose). scanMutex_ is not taken here -- it
    // guards scan_, which this function never touches. Early exit once
    // the running minimum falls inside the inflated robot radius,
    // since chapterCost() returns +infinity from that point on
    // regardless of the remaining beams.
    //////////////////////////////////////////////////////

    float TrajectoryEvaluator::evaluatePose(const geometry_msgs::Pose & pose, const sensor_msgs::LaserScan & scan_k)
    {
        const int numRanges = static_cast<int>(scan_k.ranges.size());

        if (numRanges == 0)
            return 0.0f;

        const float inflRbtRad =
            cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

        float minScan2RbtDist = std::numeric_limits<float>::infinity();
        int   minDistIdx      = -1;

        for (int i = 0; i < numRanges; i++)
        {
            const float scan2RbtDist =
                dist2Pose(idx2theta(i), scan_k.ranges.at(i), pose);

            if (!std::isfinite(scan2RbtDist))
                continue;

            if (scan2RbtDist < minScan2RbtDist)
            {
                minScan2RbtDist = scan2RbtDist;
                minDistIdx      = i;

                if (minScan2RbtDist < inflRbtRad)
                    break;
            }
        }

        if (minDistIdx < 0)
            return 0.0f;   // no finite returns in this scan

        const float cost = chapterCost(minScan2RbtDist);

        const float range = scan_k.ranges.at(minDistIdx);
        const float theta = idx2theta(minDistIdx);

        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            robot pose: " << pose.position.x << ", " << pose.position.y <<
                                                        ", closest scan point: " << range * std::cos(theta) << ", " << range * std::sin(theta) << ", cost: " << cost);
        return cost;
    }

    float TrajectoryEvaluator::chapterCost(const float & rbtToScanDist)
    {
        // if the distance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterCost with distance: " << d << std::endl;
        float inflRbtRad = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

        float inflRbtToScanDist = rbtToScanDist - inflRbtRad;

        if (inflRbtToScanDist < 0.0)
        {
            return std::numeric_limits<float>::infinity();
        }

        // if pose is sufficiently far away from scan, return no cost
        if (rbtToScanDist > cfg_->traj.max_pose_to_scan_dist)
            return 0;

        /*   y
         *   ^
         * Q |\
         *   | \
         *   |  \
         *       `
         *   |    `
         *         ` ---
         *   |          _________
         *   --------------------->x
         *
         */


        return cfg_->traj.Q * std::exp(-cfg_->traj.pen_exp_weight * inflRbtToScanDist);
    }
}
