#include <dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h>


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

        useGruGapFeatureDensityCost_ = true;
        maxGruGapFeaturePredictionAgeSec_ = 3.0;
        gruGapDensityCostWeight_ = 2.0f;

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
            if (msg->output_names.at(i) == "gt_sector_density")
            {
                predDensity =
                    msg->output_values.at(i);

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
    const int & densityModelID)
    {
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

                terminalPoseCost =
                    std::numeric_limits<float>::infinity();

                return;
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

                terminalPoseCost =
                    std::numeric_limits<float>::infinity();

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

                terminalPoseCost =
                    std::numeric_limits<float>::infinity();

                return;
            }

            //////////////////////////////////////////////////////
            // 1. Posewise obstacle/chapter cost
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
            // 2. Existing goal-progress terminal cost
            //////////////////////////////////////////////////////

            const float baseTerminalGoalCost =
                cfg_->traj.Q_f *
                terminalGoalCost(path.poses.back());

            //////////////////////////////////////////////////////
            // 3. Fresh GRU gap-density cost
            //
            // This lookup happens RIGHT NOW, at evaluation time.
            // Therefore reducedCurrentTraj scoring also uses the
            // latest available GRU prediction.
            //////////////////////////////////////////////////////

            float predGruDensity = 0.0f;
            float weightedGruDensityCost = 0.0f;
            bool usedGruDensity = false;

            ros::Time currentStamp =
                ros::Time::now();

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
            // 4. Final terminalPoseCost returned to caller
            //////////////////////////////////////////////////////

            terminalPoseCost =
                baseTerminalGoalCost +
                weightedGruDensityCost;

            // ROS_WARN_STREAM_NAMED(
            //     "GRUGapFeatureDensityCost",
            //     "evaluateTrajectory density cost | "
            //     << "density_model_id="
            //     << densityModelID
            //     << " used_gru_density="
            //     << usedGruDensity
            //     << " pred_density="
            //     << predGruDensity
            //     << " weighted_density_cost="
            //     << weightedGruDensityCost
            //     << " base_terminal_goal_cost="
            //     << baseTerminalGoalCost
            //     << " final_terminal_pose_cost="
            //     << terminalPoseCost
            // );

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
        }
        catch (const std::exception& e)
        {
            ROS_WARN_STREAM_NAMED(
                "TrajectoryEvaluator",
                "            evaluateTrajectory exception: "
                << e.what()
            );
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

    float TrajectoryEvaluator::evaluatePose(const geometry_msgs::Pose & pose, const sensor_msgs::LaserScan scan_k) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        // sensor_msgs::LaserScan scan = *scan_.get();

        // obtain orientation and idx of pose

        // dist is size of scan
        std::vector<float> scan2RbtDists(scan_k.ranges.size());

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        // float currScan2RbtDist = 0.0;
        for (int i = 0; i < scan2RbtDists.size(); i++) 
        {
            scan2RbtDists.at(i) = dist2Pose(idx2theta(i), scan_k.ranges.at(i), pose);
        }

        auto iter = std::min_element(scan2RbtDists.begin(), scan2RbtDists.end());
        // std::cout << "robot pose: " << pose.position.x << ", " << pose.position.y << ")" << std::endl;
        int minDistIdx = std::distance(scan2RbtDists.begin(), iter);
        float range = scan_k.ranges.at(minDistIdx);
        float theta = idx2theta(minDistIdx);
        float cost = chapterCost(*iter);
        //std::cout << *iter << ", regular cost: " << cost << std::endl;
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