#include <dynamic_gap/Planner.h>

// #include "tf/transform_datatypes.h"
// #include <tf/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>

namespace dynamic_gap
{   
    Planner::Planner()
    {
        // Do something? maybe set names
        ros::NodeHandle nh("planner_node");
    }

    Planner::~Planner() {}

    bool Planner::initialize(const ros::NodeHandle& unh)
    {
        // ROS_INFO_STREAM("starting initialize");
        if (initialized_)
        {
            ROS_WARN("DynamicGap Planner already initalized");
            return true;
        }

        // Config Setup
        cfg_.loadRosParamFromNodeHandle(unh);

        // Visualization Setup
        currentTrajectoryPublisher_ = nh.advertise<geometry_msgs::PoseArray>("curr_exec_dg_traj", 1);
        staticScanPublisher_ = nh.advertise<sensor_msgs::LaserScan>("static_scan", 1);

        // TF Lookup setup
        tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
        initialized_ = true;

        gapDetector_ = new dynamic_gap::GapDetector(cfg_);
        staticScanSeparator_ = new dynamic_gap::StaticScanSeparator(cfg_);
        gapVisualizer_ = new dynamic_gap::GapVisualizer(nh, cfg_);
        goalSelector_ = new dynamic_gap::GoalSelector(nh, cfg_);
        trajVisualizer_ = new dynamic_gap::TrajectoryVisualizer(nh, cfg_);
        trajScorer_ = new dynamic_gap::TrajectoryScorer(nh, cfg_);
        gapTrajGenerator_ = new dynamic_gap::GapTrajectoryGenerator(nh, cfg_);
        goalvisualizer = new dynamic_gap::GoalVisualizer(nh, cfg_);
        gapManipulator_ = new dynamic_gap::GapManipulator(nh, cfg_);
        trajController_ = new dynamic_gap::TrajectoryController(nh, cfg_);
        gapAssociator_ = new dynamic_gap::GapAssociator(nh, cfg_);
        gapFeasibilityChecker_ = new dynamic_gap::GapFeasibilityChecker(nh, cfg_);

        // MAP FRAME ID SHOULD BE: known_map
        // ODOM FRAME ID SHOULD BE: map_static

        map2rbt_.transform.rotation.w = 1;
        odom2rbt_.transform.rotation.w = 1;
        rbt2odom_.transform.rotation.w = 1;
        rbt_in_rbt.pose.orientation.w = 1;
        rbt_in_rbt.header.frame_id = cfg_.robot_frame_id;

        log_vel_comp.set_capacity(cfg_.planning.halt_size);

        currentRbtVel_ = geometry_msgs::TwistStamped();
        currentRbtAcc_ = geometry_msgs::TwistStamped();
        
        currentModelIdx_ = 0;

        currLeftGapPtModel_ = NULL;
        currRightGapPtModel_ = NULL;

        robotPoseOdomFrame_ = geometry_msgs::PoseStamped();
        globalGoalRobotFrame_ = geometry_msgs::PoseStamped();

        currentAgentCount_ = cfg_.env.num_obsts;

        currentTrueAgentPoses_ = std::vector<geometry_msgs::Pose>(currentAgentCount_);
        currentTrueAgentVels_ = std::vector<geometry_msgs::Vector3Stamped>(currentAgentCount_);

        sensor_msgs::LaserScan tmp_scan = sensor_msgs::LaserScan();
        futureScans_.push_back(tmp_scan);
        for (float t_iplus1 = cfg_.traj.integrate_stept; t_iplus1 <= cfg_.traj.integrate_maxt; t_iplus1 += cfg_.traj.integrate_stept) 
            futureScans_.push_back(tmp_scan);

        staticScan_ = sensor_msgs::LaserScan();
        // ROS_INFO_STREAM("future_scans size: " << future_scans.size());
        // ROS_INFO_STREAM("done initializing");
        trajectoryChangeCount_ = 0;

        intermediateRbtVels_.clear();
        intermediateRbtAccs_.clear();

        hasGlobalGoal_ = false;

        tPreviousFilterUpdate_ = ros::Time::now();


        return true;
    }

    bool Planner::isGoalReached()
    {
        float dx = globalGoalOdomFrame_.pose.position.x - robotPoseOdomFrame_.pose.position.x;
        float dy = globalGoalOdomFrame_.pose.position.y - robotPoseOdomFrame_.pose.position.y;
        bool result = sqrt(pow(dx, 2) + pow(dy, 2)) < cfg_.goal.goal_tolerance;
        
        if (result)
        {
            ROS_INFO_STREAM("[Reset] Goal Reached");
            return true;
        }

        float waydx = globalPathLocalWaypointOdomFrame_.pose.position.x - robotPoseOdomFrame_.pose.position.x;
        float waydy = globalPathLocalWaypointOdomFrame_.pose.position.y - robotPoseOdomFrame_.pose.position.y;
        bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) < cfg_.goal.waypoint_tolerance;
        if (wayres) {
            ROS_INFO_STREAM("[Reset] Waypoint reached, getting new one");
        }
        return false;
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        scan_ = msg;

        ros::Time curr_time = scan_->header.stamp;

        cfg_.updateParamFromScan(scan_);

        ros::Time tCurrentFilterUpdate = scan_->header.stamp;
        if (hasGlobalGoal_)
        {
            // grabbing current intermediate robot velocities and accelerations
            std::vector<geometry_msgs::TwistStamped> intermediateRbtVels = intermediateRbtVels_;
            std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs = intermediateRbtAccs_;


            // std::chrono::steady_clock::time_point gap_detection_start_time = std::chrono::steady_clock::now();
            // ROS_INFO_STREAM("Time elapsed before raw gaps processing: " << (ros::WallTime::now().toSec() - start_time));
            rawGaps_ = gapDetector_->gapDetection(scan_, globalGoalRobotFrame_);
            // float gap_detection_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_detection_start_time).count() / 1.0e6;
            // ROS_INFO_STREAM("gapDetection: " << gap_detection_time << " seconds");

            if (cfg_.debug.raw_gaps_debug_log) ROS_INFO_STREAM("RAW GAP ASSOCIATING");    
            rawDistMatrix_ = gapAssociator_->obtainDistMatrix(rawGaps_, previousRawGaps_);
            rawAssocation_ = gapAssociator_->associateGaps(rawDistMatrix_);         // ASSOCIATE GAPS PASSES BY REFERENCE
            gapAssociator_->assignModels(rawAssocation_, rawDistMatrix_, 
                                        rawGaps_, previousRawGaps_, 
                                        currentModelIdx_, tCurrentFilterUpdate,
                                        intermediateRbtVels, intermediateRbtAccs,
                                        cfg_.debug.raw_gaps_debug_log);
            
            associatedRawGaps_ = updateModels(rawGaps_, intermediateRbtVels, 
                                                intermediateRbtAccs, tCurrentFilterUpdate,
                                                cfg_.debug.raw_gaps_debug_log);
            // ROS_INFO_STREAM("Time elapsed after raw gaps processing: " << (ros::WallTime::now().toSec() - start_time));

            staticScan_ = staticScanSeparator_->staticDynamicScanSeparation(associatedRawGaps_, scan_, cfg_.debug.static_scan_separation_debug_log);
            staticScanPublisher_.publish(staticScan_);
            trajScorer_->updateStaticEgoCircle(staticScan_);
            gapManipulator_->updateStaticEgoCircle(staticScan_);
            currentEstimatedAgentStates_ = staticScanSeparator_->getCurrAgents();

            // std::chrono::steady_clock::time_point gap_simplification_start_time = std::chrono::steady_clock::now();
            simplifiedGaps_ = gapDetector_->gapSimplification(rawGaps_);
            // float gap_simplification_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_simplification_start_time).count() / 1.0e6;
            // ROS_INFO_STREAM("gapSimplification: " << gap_simplification_time << " seconds");

            if (cfg_.debug.simplified_gaps_debug_log) ROS_INFO_STREAM("SIMPLIFIED GAP ASSOCIATING");    
            simpDistMatrix_ = gapAssociator_->obtainDistMatrix(simplifiedGaps_, previousSimplifiedGaps_);
            simpAssociation_ = gapAssociator_->associateGaps(simpDistMatrix_); // must finish this and therefore change the association
            gapAssociator_->assignModels(simpAssociation_, simpDistMatrix_, 
                                        simplifiedGaps_, previousSimplifiedGaps_, 
                                        currentModelIdx_, tCurrentFilterUpdate,
                                        intermediateRbtVels, intermediateRbtAccs,
                                        cfg_.debug.simplified_gaps_debug_log);
            associatedSimplifiedGaps_ = updateModels(simplifiedGaps_, intermediateRbtVels, 
                                                        intermediateRbtAccs, tCurrentFilterUpdate,
                                                        cfg_.debug.simplified_gaps_debug_log);
            // ROS_INFO_STREAM("Time elapsed after observed gaps processing: " << (ros::WallTime::now().toSec() - start_time));

            gapVisualizer_->drawGaps(associatedRawGaps_, std::string("raw"));
            // gapVisualizer_->drawGaps(associatedSimplifiedGaps_, std::string("simp"));
            gapVisualizer_->drawGapsModels(associatedRawGaps_);
        }

        geometry_msgs::PoseStamped local_goal;

        goalSelector_->updateEgoCircle(scan_);
        goalSelector_->updateLocalGoal(map2rbt_);
        local_goal = goalSelector_->transformLocalGoalToOdomFrame(rbt2odom_);
        goalvisualizer->localGoal(local_goal);

        // ROS_INFO_STREAM("Time elapsed after updating goal selector: " << (ros::WallTime::now().toSec() - start_time));

        trajScorer_->updateEgoCircle(scan_);
        trajScorer_->updateLocalGoal(local_goal, odom2rbt_);

        // ROS_INFO_STREAM("Time elapsed after updating arbiter: " << (ros::WallTime::now().toSec() - start_time));

        gapManipulator_->updateEgoCircle(scan_);
        trajController_->updateEgoCircle(scan_);
        // gapFeasibilityChecker_->updateEgoCircle(scan_);
        // ROS_INFO_STREAM("Time elapsed after updating rest: " << (ros::WallTime::now().toSec() - start_time));

        // ROS_INFO_STREAM("laserscan time elapsed: " << ros::WallTime::now().toSec() - start_time);

        previousRawGaps_ = associatedRawGaps_;
        previousSimplifiedGaps_ = associatedSimplifiedGaps_;
        tPreviousFilterUpdate_ = tCurrentFilterUpdate;
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    std::vector<dynamic_gap::Gap> Planner::updateModels(const std::vector<dynamic_gap::Gap> & gaps, 
                                                         const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                                                         const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                                                         const ros::Time & tCurrentFilterUpdate,
                                                         bool print) 
    {
        if (print) ROS_INFO_STREAM("[updateModels()]");
        std::vector<dynamic_gap::Gap> associatedGaps = gaps;
        
        // double start_time = ros::WallTime::now().toSec();
        for (int i = 0; i < 2*associatedGaps.size(); i++) 
        {
            if (print) ROS_INFO_STREAM("    update gap model " << i << " of " << 2*associatedGaps.size());
            updateModel(i, associatedGaps, intermediateRbtVels, intermediateRbtAccs, tCurrentFilterUpdate, print);
            if (print) ROS_INFO_STREAM("");
		}

        //ROS_INFO_STREAM("updateModels time elapsed: " << ros::WallTime::now().toSec() - start_time);
        return associatedGaps;
    }

    void Planner::updateModel(int idx, std::vector<dynamic_gap::Gap>& gaps, 
                               const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                               const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                               const ros::Time & tCurrentFilterUpdate,
                               bool print) 
    {
		dynamic_gap::Gap gap = gaps[int(idx / 2.0)];
 
        // float thetaTilde, rangeTilde;
        float rX, rY;
		if (idx % 2 == 0) 
        {
			// thetaTilde = idx2theta(gap.RIdx()); // float(g.RIdx() - g.half_scan) / g.half_scan * M_PI;
            // rangeTilde = gap.RDist();
            gap.getRCartesian(rX, rY);
		} else 
        {
            // thetaTilde = idx2theta(gap.LIdx()); // float(g.LIdx() - g.half_scan) / g.half_scan * M_PI;
            // rangeTilde = gap.LDist();
            gap.getLCartesian(rX, rY);
		}

        // ROS_INFO_STREAM("rX: " << rX << ", rY: " << rY);

		// Eigen::Vector2f laserscan_measurement(rangeTilde, thetaTilde);
        Eigen::Vector2f measurement(rX, rY);

        if (idx % 2 == 0) 
        {
            //std::cout << "entering left model update" << std::endl;
            try {
                gap.rightGapPtModel_->update(measurement, 
                                            intermediateRbtVels, intermediateRbtAccs, 
                                            print, currentTrueAgentPoses_, 
                                            currentTrueAgentVels_,
                                            tCurrentFilterUpdate);
            } catch (...) 
            {
                ROS_INFO_STREAM("kf_update_loop fails");
            }
        } else {
            //std::cout << "entering right model update" << std::endl;
            try {
                gap.leftGapPtModel_->update(measurement, 
                                            intermediateRbtVels, intermediateRbtAccs, 
                                            print, currentTrueAgentPoses_, 
                                            currentTrueAgentVels_,
                                            tCurrentFilterUpdate);
            } catch (...) 
            {
                ROS_INFO_STREAM("kf_update_loop fails");
            }
        }
    }

    void Planner::jointPoseAccCB(const nav_msgs::Odometry::ConstPtr &odom_msg, 
                                 const geometry_msgs::TwistStamped::ConstPtr &accel_msg)
    {
        // ROS_INFO_STREAM("joint pose acc cb");

        // ROS_INFO_STREAM("accel time stamp: " << accel_msg->header.stamp.toSec());
        currentRbtAcc_ = *accel_msg;
        intermediateRbtAccs_.push_back(currentRbtAcc_);

        // deleting old sensor measurements already used in an update
        for (int i = 0; i < intermediateRbtAccs_.size(); i++)
        {
            if (intermediateRbtAccs_[i].header.stamp <= tPreviousFilterUpdate_)
            {
                intermediateRbtAccs_.erase(intermediateRbtAccs_.begin() + i);
                i--;
            }
        }    

        // ROS_INFO_STREAM("odom time stamp: " << odom_msg->header.stamp.toSec());

        // ROS_INFO_STREAM("acc - odom time difference: " << (accel_msg->header.stamp - odom_msg->header.stamp).toSec());

        updateTF();

        // Transform the msg to odom frame
        if(odom_msg->header.frame_id != cfg_.odom_frame_id)
        {
            //std::cout << "odom msg is not in odom frame" << std::endl;
            geometry_msgs::TransformStamped robot_pose_odom_trans = tfBuffer_.lookupTransform(cfg_.odom_frame_id, odom_msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = odom_msg->header;
            in_pose.pose = odom_msg->pose.pose;

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;

            tf2::doTransform(in_pose, out_pose, robot_pose_odom_trans);
            robotPoseOdomFrame_ = out_pose;
        }
        else
        {
            robotPoseOdomFrame_.pose = odom_msg->pose.pose;
        }

        //--------------- VELOCITY -------------------//
        // velocity always comes in wrt robot frame in STDR
        geometry_msgs::TwistStamped ego_rbt_vel;
        ego_rbt_vel.header = odom_msg->header;
        ego_rbt_vel.header.frame_id = accel_msg->header.frame_id;
        ego_rbt_vel.twist = odom_msg->twist.twist;

        currentRbtVel_ = ego_rbt_vel;
        intermediateRbtVels_.push_back(currentRbtVel_);

        // deleting old sensor measurements already used in an update
        for (int i = 0; i < intermediateRbtVels_.size(); i++)
        {
            if (intermediateRbtVels_[i].header.stamp <= tPreviousFilterUpdate_)
            {
                intermediateRbtVels_.erase(intermediateRbtVels_.begin() + i);
                i--;
            }
        }

    }
    
    void Planner::agentOdomCB(const nav_msgs::Odometry::ConstPtr& msg) 
    {
        std::string robot_namespace = msg->child_frame_id;
        // ROS_INFO_STREAM("robot_namespace: " << robot_namespace);
        robot_namespace.erase(0,5); // removing "robot"
        char *robot_name_char = strdup(robot_namespace.c_str());
        int robot_id = std::atoi(robot_name_char);
        // ROS_INFO_STREAM("robot_id: " << robot_id);
        // I need BOTH odom and vel in robot2 frame
        //std::cout << "odom msg is not in odom frame" << std::endl;
        try 
        {
            // transforming Odometry message from map_static to robotN
            geometry_msgs::TransformStamped agent_to_robot_odom_trans = tfBuffer_.lookupTransform(cfg_.robot_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;
            // ROS_INFO_STREAM("updating inpose " << robot_namespace << " to: (" << in_pose.pose.position.x << ", " << in_pose.pose.position.y << ")");

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;
            tf2::doTransform(in_pose, out_pose, agent_to_robot_odom_trans);
            
            // ROS_INFO_STREAM("updating " << robot_namespace << " odom from " << agent_odom_vects[robot_id][0] << ", " << agent_odom_vects[robot_id][1] << " to " << odom_vect[0] << ", " << odom_vect[1]);
            currentTrueAgentPoses_[robot_id] = out_pose.pose;
        } catch (tf2::TransformException &ex) {
            ROS_INFO_STREAM("Odometry transform failed for " << robot_namespace);
        }
        
        try 
        {
            std::string source_frame = msg->child_frame_id; 
            // std::cout << "in agentOdomCB" << std::endl;
            // std::cout << "transforming from " << source_frame << " to " << cfg_.robot_frame_id << std::endl;
            geometry_msgs::TransformStamped agent_to_robot_trans = tfBuffer_.lookupTransform(cfg_.robot_frame_id, source_frame, ros::Time(0));
            geometry_msgs::Vector3Stamped in_vel, out_vel;
            in_vel.header = msg->header;
            in_vel.header.frame_id = source_frame;
            in_vel.vector = msg->twist.twist.linear;
            // std::cout << "incoming vector: " << in_vel.vector.x << ", " << in_vel.vector.y << std::endl;
            tf2::doTransform(in_vel, out_vel, agent_to_robot_trans);
            // std::cout << "outcoming vector: " << out_vel.vector.x << ", " << out_vel.vector.y << std::endl;

            currentTrueAgentVels_[robot_id] = out_vel;
        } catch (tf2::TransformException &ex) {
            ROS_INFO_STREAM("Velocity transform failed for " << robot_namespace);
        }            
    }

    bool Planner::setGoal(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (plan.size() == 0) return true;
        globalGoalOdomFrame_ = *std::prev(plan.end());
        tf2::doTransform(globalGoalOdomFrame_, globalGoalOdomFrame_, map2odom_);
        tf2::doTransform(globalGoalOdomFrame_, globalGoalRobotFrame_, odom2rbt_);
        // Store New Global Plan to Goal Selector
        goalSelector_->setGoal(plan);
        
        // Obtaining Local Goal by using global plan
        goalSelector_->updateLocalGoal(map2rbt_);
        // return local goal (odom) frame
        auto new_local_waypoint = goalSelector_->transformLocalGoalToOdomFrame(rbt2odom_);

        {
            // Plan New
            float waydx = globalPathLocalWaypointOdomFrame_.pose.position.x - new_local_waypoint.pose.position.x;
            float waydy = globalPathLocalWaypointOdomFrame_.pose.position.y - new_local_waypoint.pose.position.y;
            bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) > cfg_.goal.waypoint_tolerance;
            if (wayres) {
                globalPathLocalWaypointOdomFrame_ = new_local_waypoint;
            }
        }

        // Set new local goal to trajectory arbiter
        trajScorer_->updateLocalGoal(globalPathLocalWaypointOdomFrame_, odom2rbt_);

        // Visualization only
        std::vector<geometry_msgs::PoseStamped> traj = goalSelector_->getRelevantGlobalPlan(map2rbt_);
        trajVisualizer_->drawRelevantGlobalPlanSnippet(traj);
        // trajVisualizer_->drawEntireGlobalPlan(goalselector->getOdomGlobalPlan());

        hasGlobalGoal_ = true;
        return true;
    }

    void Planner::updateTF()
    {
        try 
        {
            map2rbt_  = tfBuffer_.lookupTransform(cfg_.robot_frame_id, cfg_.map_frame_id, ros::Time(0));
            odom2rbt_ = tfBuffer_.lookupTransform(cfg_.robot_frame_id, cfg_.odom_frame_id, ros::Time(0));
            rbt2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.robot_frame_id, ros::Time(0));
            map2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.map_frame_id, ros::Time(0));
            cam2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.sensor_frame_id, ros::Time(0));
            rbt2cam_ = tfBuffer_.lookupTransform(cfg_.sensor_frame_id, cfg_.robot_frame_id, ros::Time(0));

            tf2::doTransform(rbt_in_rbt, rbt_in_cam, rbt2cam_);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            return;
        }
    }

    std::vector<dynamic_gap::Gap> Planner::gapManipulate(const std::vector<dynamic_gap::Gap> & _observed_gaps) 
    {
        if (cfg_.debug.manipulation_debug_log) ROS_INFO_STREAM("[gapManipulate()]");

        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<dynamic_gap::Gap> manip_set = _observed_gaps;

        for (size_t i = 0; i < manip_set.size(); i++)
        {
            if (cfg_.debug.manipulation_debug_log) ROS_INFO_STREAM("    manipulating initial gap " << i);
            // MANIPULATE POINTS AT T=0
            manip_set.at(i).initManipIndices();
            
            // gapManipulator_->reduceGap(manip_set.at(i), goalSelector_->rbtFrameLocalGoal(), true);
            gapManipulator_->convertRadialGap(manip_set.at(i), true); 
            gapManipulator_->inflateGapSides(manip_set.at(i), true);
            gapManipulator_->radialExtendGap(manip_set.at(i), true);
            gapManipulator_->setGapWaypoint(manip_set.at(i), goalSelector_->rbtFrameLocalGoal(), true);
            
            
            // MANIPULATE POINTS AT T=1
            if (cfg_.debug.manipulation_debug_log) ROS_INFO_STREAM("    manipulating terminal gap " << i);
            gapManipulator_->updateDynamicEgoCircle(manip_set.at(i), futureScans_);
            if ((!manip_set.at(i).crossed_ && !manip_set.at(i).closed_) || (manip_set.at(i).crossedBehind_)) 
            {
                // gapManipulator_->reduceGap(manip_set.at(i), goalSelector_->rbtFrameLocalGoal(), false);
                gapManipulator_->convertRadialGap(manip_set.at(i), false);
            }
            gapManipulator_->inflateGapSides(manip_set.at(i), false);
            gapManipulator_->radialExtendGap(manip_set.at(i), false);
            gapManipulator_->setTerminalGapWaypoint(manip_set.at(i), goalSelector_->rbtFrameLocalGoal());
        }

        return manip_set;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<float>> Planner::initialTrajGen(std::vector<dynamic_gap::Gap>& vec, 
                                                            std::vector<geometry_msgs::PoseArray>& res, 
                                                            std::vector<std::vector<float>>& res_time_traj) 
    {
        if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("[initialTrajGen()]");
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<geometry_msgs::PoseArray> ret_traj(vec.size());
        std::vector<std::vector<float>> ret_time_traj(vec.size());
        std::vector<std::vector<float>> ret_traj_scores(vec.size());
        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam; // lc as local copy

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associatedRawGaps_;
        try {
            for (size_t i = 0; i < vec.size(); i++) 
            {
                if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    generating traj for gap: " << i);
                // std::cout << "starting generate trajectory with rbt_in_cam_lc: " << rbt_in_cam_lc.pose.position.x << ", " << rbt_in_cam_lc.pose.position.y << std::endl;
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                std::tuple<geometry_msgs::PoseArray, std::vector<float>> return_tuple;
                
                // TRAJECTORY GENERATED IN RBT FRAME
                bool run_g2g = true; // (vec.at(i).goal.goalwithin || vec.at(i).artificial);
                if (run_g2g) 
                {
                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        running g2g");
                    std::tuple<geometry_msgs::PoseArray, std::vector<float>> g2g_tuple;
                    g2g_tuple = gapTrajGenerator_->generateTrajectory(vec.at(i), rbt_in_cam_lc, currentRbtVel_, run_g2g);
                    g2g_tuple = gapTrajGenerator_->forwardPassTrajectory(g2g_tuple);
                    std::vector<float> g2g_score_vec = trajScorer_->scoreTrajectory(std::get<0>(g2g_tuple), std::get<1>(g2g_tuple), curr_raw_gaps, 
                                                                                     currentTrueAgentPoses_, currentTrueAgentVels_, futureScans_, false, false);
                    float g2g_score = std::accumulate(g2g_score_vec.begin(), g2g_score_vec.end(), float(0));
                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        g2g_score: " << g2g_score);

                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        running ahpf");
                    std::tuple<geometry_msgs::PoseArray, std::vector<float>> ahpf_tuple;
                    ahpf_tuple = gapTrajGenerator_->generateTrajectory(vec.at(i), rbt_in_cam_lc, currentRbtVel_, !run_g2g);
                    ahpf_tuple = gapTrajGenerator_->forwardPassTrajectory(ahpf_tuple);
                    std::vector<float> ahpf_score_vec = trajScorer_->scoreTrajectory(std::get<0>(ahpf_tuple), std::get<1>(ahpf_tuple), curr_raw_gaps, 
                                                                                        currentTrueAgentPoses_, currentTrueAgentVels_, futureScans_, false, false);
                    float ahpf_score = std::accumulate(ahpf_score_vec.begin(), ahpf_score_vec.end(), float(0));
                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        ahpf_score: " << ahpf_score);

                    return_tuple = (g2g_score > ahpf_score) ? g2g_tuple : ahpf_tuple;
                    ret_traj_scores.at(i) = (g2g_score > ahpf_score) ? g2g_score_vec : ahpf_score_vec;
                } else {
                    return_tuple = gapTrajGenerator_->generateTrajectory(vec.at(i), rbt_in_cam_lc, currentRbtVel_, run_g2g);
                    return_tuple = gapTrajGenerator_->forwardPassTrajectory(return_tuple);

                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    scoring trajectory for gap: " << i);
                    ret_traj_scores.at(i) = trajScorer_->scoreTrajectory(std::get<0>(return_tuple), std::get<1>(return_tuple), curr_raw_gaps, 
                                                                         currentTrueAgentPoses_, currentTrueAgentVels_, futureScans_, false, false);  
                    // ROS_INFO_STREAM("done with scoreTrajectory");
                }

                // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                ret_traj.at(i) = gapTrajGenerator_->transformBackTrajectory(std::get<0>(return_tuple), cam2odom_);
                ret_time_traj.at(i) = std::get<1>(return_tuple);
            }

        } catch (...) {
            ROS_FATAL_STREAM("initialTrajGen");
        }

        trajVisualizer_->pubAllScore(ret_traj, ret_traj_scores);
        trajVisualizer_->pubAllTraj(ret_traj);
        res = ret_traj;
        res_time_traj = ret_time_traj;
        return ret_traj_scores;
    }

    int Planner::pickTraj(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<float>> score) 
    {
        if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("[pickTraj()]");
        // ROS_INFO_STREAM_NAMED("pg_trajCount", "pg_trajCount, " << prr.size());
        if (prr.size() == 0) {
            ROS_WARN_STREAM("No traj synthesized");
            return -1;
        }

        if (prr.size() != score.size()) {
            ROS_FATAL_STREAM("pickTraj size mismatch: prr = " << prr.size() << " != score =" << score.size());
            return -1;
        }

        // poses here are in odom frame 
        std::vector<float> result_score(prr.size());
        int counts;
        try {
            // if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < result_score.size(); i++) {
                // ROS_WARN_STREAM("prr(" << i << "): size " << prr.at(i).poses.size());
                counts = std::min(cfg_.planning.num_feasi_check, int(score.at(i).size()));

                result_score.at(i) = std::accumulate(score.at(i).begin(), score.at(i).begin() + counts, float(0));
                result_score.at(i) = prr.at(i).poses.size() == 0 ? -std::numeric_limits<float>::infinity() : result_score.at(i);
                if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    for gap " << i << " (length: " << prr.at(i).poses.size() << "), returning score of " << result_score.at(i));
                /*
                if (result_score.at(i) == -std::numeric_limits<float>::infinity()) {
                    for (size_t j = 0; j < counts; j++) {
                        if (score.at(i).at(j) == -std::numeric_limits<float>::infinity()) {
                            std::cout << "-inf score at idx " << j << " of " << counts << std::endl;
                        }
                    }
                }
                */
            }
        } catch (...) {
            ROS_FATAL_STREAM("pickTraj");
        }

        auto iter = std::max_element(result_score.begin(), result_score.end());
        int idx = std::distance(result_score.begin(), iter);

        if (result_score.at(idx) == -std::numeric_limits<float>::infinity()) 
        {    
            ROS_INFO_STREAM("    all -infinity");
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (float val : result_score) 
            {
                ROS_INFO_STREAM("Score: " << val);
            }
            ROS_INFO_STREAM("------------------");
        }

        if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    picking gap: " << idx);
        
        return idx;
    }

    geometry_msgs::PoseArray Planner::changeTrajectoryHelper(dynamic_gap::Gap incoming_gap, geometry_msgs::PoseArray incoming, 
                                                                std::vector<float> time_arr, bool switching_to_empty) 
    {
        trajectoryChangeCount_++;

        if (switching_to_empty) {
            geometry_msgs::PoseArray empty_traj = geometry_msgs::PoseArray();
            empty_traj.header = incoming.header;
            std::vector<float> empty_time_arr;
            setCurrentTraj(empty_traj);
            setCurrentTimeArr(empty_time_arr);
            setCurrentLeftModel(NULL);
            setCurrentRightModel(NULL);
            setCurrentGapPeakVelocities(0.0, 0.0);
            currentTrajectoryPublisher_.publish(empty_traj);
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, empty_traj);
            return empty_traj;
        } else {
            setCurrentTraj(incoming);
            setCurrentTimeArr(time_arr);
            setCurrentLeftModel(incoming_gap.leftGapPtModel_);
            setCurrentRightModel(incoming_gap.rightGapPtModel_);
            setCurrentGapPeakVelocities(incoming_gap.peakVelX_, incoming_gap.peakVelY_);
            currentTrajectoryPublisher_.publish(incoming);          
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, incoming);

            return incoming;  
        }                       
    }

    geometry_msgs::PoseArray Planner::compareToOldTraj(geometry_msgs::PoseArray incoming, 
                                                       dynamic_gap::Gap incoming_gap, 
                                                       std::vector<dynamic_gap::Gap> feasible_gaps, 
                                                       std::vector<float> time_arr,
                                                       bool curr_exec_gap_assoc,
                                                       bool curr_exec_gap_feas) 
    {
        if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("[compareToOldTraj()]");
        boost::mutex::scoped_lock gapset(gapset_mutex);
        auto curr_traj = getCurrentTraj();
        auto curr_time_arr = getCurrentTimeArr();

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associatedRawGaps_;

        try {
            float curr_time = ros::WallTime::now().toSec();
            
            // FORCING OFF CURRENT TRAJ IF NO LONGER FEASIBLE
            // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapIndex() << ", current right gap index: " << getCurrentRightGapIndex());
            // bool curr_gap_feasible = (curr_exec_gap_assoc && curr_exec_gap_feas);
            bool curr_gap_feasible = true;
            for (dynamic_gap::Gap g : feasible_gaps) 
            {
                // ROS_INFO_STREAM("feasible left gap index: " << g.leftGapPtModel_->get_index() << ", feasible right gap index: " << g.rightGapPtModel_->get_index());
                if (g.leftGapPtModel_->get_index() == getCurrentLeftGapIndex() &&
                    g.rightGapPtModel_->get_index() == getCurrentRightGapIndex()) {
                    setCurrentGapPeakVelocities(g.peakVelX_, g.peakVelY_);
                    break;
                }
            }

            // std::cout << "current traj length: " << curr_traj.poses.size() << std::endl;
            // ROS_INFO_STREAM("current gap indices: " << getCurrentLeftGapIndex() << ", " << getCurrentRightGapIndex());
            //std::cout << "current time length: " << curr_time_arr.size() << std::endl;
            //std::cout << "incoming traj length: " << incoming.poses.size() << std::endl;
            //std::cout << "incoming time length: " << time_arr.size() << std::endl;
            
            // Both Args are in Odom frame
            auto incom_rbt = gapTrajGenerator_->transformBackTrajectory(incoming, odom2rbt_);
            incom_rbt.header.frame_id = cfg_.robot_frame_id;
            // why do we have to rescore here?

            if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    scoring incoming trajectory");
            auto incom_score = trajScorer_->scoreTrajectory(incom_rbt, time_arr, curr_raw_gaps, 
                                                            currentTrueAgentPoses_, currentTrueAgentVels_, futureScans_, false, true);
            // int counts = std::min(cfg_.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));

            int counts = std::min(cfg_.planning.num_feasi_check, (int) incom_score.size());
            auto incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, float(0));

            if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    incoming trajectory subscore: " << incom_subscore);
            bool curr_traj_length_zero = curr_traj.poses.size() == 0;
            bool curr_gap_not_feasible = !curr_gap_feasible;

            // CURRENT TRAJECTORY LENGTH ZERO OR IS NOT FEASIBLE
            if (curr_traj_length_zero || curr_gap_not_feasible) {
                bool valid_incoming_traj = incoming.poses.size() > 0 && incom_subscore > -std::numeric_limits<float>::infinity();
                
                std::string curr_traj_status = (curr_traj_length_zero) ? "curr traj length 0" : "curr traj is not feasible";
                if (!curr_exec_gap_assoc) {
                    curr_traj_status = "curr exec gap is not associated (left: " + std::to_string(getCurrentLeftGapIndex()) + ", right: " + std::to_string(getCurrentRightGapIndex()) + ")";
                } else if (!curr_exec_gap_feas) {
                    curr_traj_status = "curr exec gap is deemed infeasible";
                }
                
                if (valid_incoming_traj) 
                {
                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ << " to incoming: " << curr_traj_status << ", incoming score finite");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
                } else  {
                    std::string incoming_traj_status = (incoming.poses.size() > 0) ? "incoming traj length 0" : "incoming score infinite";

                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ <<  " to empty: " << curr_traj_status << ", " << incoming_traj_status);
                    
                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, true);
                }
            } 

            // CURRENT TRAJECTORY HAS ENDED
            auto curr_rbt = gapTrajGenerator_->transformBackTrajectory(curr_traj, odom2rbt_);
            curr_rbt.header.frame_id = cfg_.robot_frame_id;
            int start_position = egoTrajPosition(curr_rbt);
            geometry_msgs::PoseArray reduced_curr_rbt = curr_rbt;
            std::vector<float> reduced_curr_time_arr = curr_time_arr;
            reduced_curr_rbt.poses = std::vector<geometry_msgs::Pose>(curr_rbt.poses.begin() + start_position, curr_rbt.poses.end());
            reduced_curr_time_arr = std::vector<float>(curr_time_arr.begin() + start_position, curr_time_arr.end());
            if (reduced_curr_rbt.poses.size() < 2) {
                if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ <<  " to incoming: old traj length less than 2");

                return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
            }

            counts = std::min(cfg_.planning.num_feasi_check, (int) std::min(incoming.poses.size(), reduced_curr_rbt.poses.size()));
            incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, float(0));
            if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    re-scored incoming trajectory subscore: " << incom_subscore);
            if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    scoring current trajectory");            
            auto curr_score = trajScorer_->scoreTrajectory(reduced_curr_rbt, reduced_curr_time_arr, curr_raw_gaps, 
                                                           currentTrueAgentPoses_, currentTrueAgentVels_, futureScans_, false, false);
            auto curr_subscore = std::accumulate(curr_score.begin(), curr_score.begin() + counts, float(0));
            if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    current trajectory subscore: " << curr_subscore);

            std::vector<std::vector<float>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incom_score;
            ret_traj_scores.at(1) = curr_score;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incom_rbt;
            viz_traj.at(1) = reduced_curr_rbt;
            trajVisualizer_->pubAllScore(viz_traj, ret_traj_scores);

            if (curr_subscore == -std::numeric_limits<float>::infinity()) 
            {
                if (incom_subscore == -std::numeric_limits<float>::infinity()) 
                {
                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ <<  " to empty: both -infinity");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, true);
                } else {
                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ << " to incoming: swapping trajectory due to collision");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
                }
            }

            /*
            if (incom_subscore > curr_subscore) {
                if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: higher score");
                changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
            }
            */
          
            if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory maintain");
            currentTrajectoryPublisher_.publish(curr_traj);
        } catch (...) {
            ROS_FATAL_STREAM("compareToOldTraj");
        }
        return curr_traj;
    }

    int Planner::egoTrajPosition(const geometry_msgs::PoseArray & curr) 
    {
        std::vector<float> pose_diff(curr.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            pose_diff[i] = sqrt(pow(curr.poses.at(i).position.x, 2) + 
                                pow(curr.poses.at(i).position.y, 2));
        }

        auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
        int closest_pose = std::distance(pose_diff.begin(), min_element_iter) + 1;
        return std::min(closest_pose, int(curr.poses.size() - 1));
    }

    void Planner::setCurrentRightModel(dynamic_gap::Estimator * rightModel) 
    { 
        currRightGapPtModel_ = rightModel; 
    }

    void Planner::setCurrentLeftModel(dynamic_gap::Estimator * leftModel) 
    { 
        currLeftGapPtModel_ = leftModel; 
    }

    void Planner::setCurrentGapPeakVelocities(float _peakVelX_, float _peakVelY_) 
    {
        currentPeakSplineVel_.twist.linear.x = _peakVelX_;
        currentPeakSplineVel_.twist.linear.y = _peakVelY_;
    }

    int Planner::getCurrentRightGapIndex() 
    {
        // std::cout << "get current left" << std::endl;
        if (currRightGapPtModel_ != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return currRightGapPtModel_->get_index();
        } else {
            // std::cout << "model is null" << std::endl;
            return -1;
        }
    }
    
    int Planner::getCurrentLeftGapIndex() 
    {
        // std::cout << "get current right" << std::endl;
        if (currLeftGapPtModel_ != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return currLeftGapPtModel_->get_index();
        } else {
            // std::cout << "model is null" << std::endl;
            return -1;
        }    
    }

    void Planner::reset()
    {
        simplifiedGaps_.clear();
        setCurrentTraj(geometry_msgs::PoseArray());
        currentRbtVel_ = geometry_msgs::TwistStamped();
        currentRbtAcc_ = geometry_msgs::TwistStamped();
        ROS_INFO_STREAM("log_vel_comp size: " << log_vel_comp.size());
        log_vel_comp.clear();
        ROS_INFO_STREAM("log_vel_comp size after clear: " << log_vel_comp.size() << ", is full: " << log_vel_comp.capacity());
        return;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(geometry_msgs::PoseArray traj) 
    {
        if (cfg_.debug.control_debug_log) ROS_INFO_STREAM("[ctrlGeneration()]");
        geometry_msgs::Twist raw_cmd_vel = geometry_msgs::Twist();

        if (cfg_.man.man_ctrl) { // MANUAL CONTROL
            raw_cmd_vel = trajController_->manualControlLaw();
        } else if (traj.poses.size() < 2) { // OBSTACLE AVOIDANCE CONTROL
            sensor_msgs::LaserScan stored_scan_msgs;

            stored_scan_msgs = *scan_.get();

            ROS_INFO_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 2");
            raw_cmd_vel = trajController_->obstacleAvoidanceControlLaw(stored_scan_msgs);
            return raw_cmd_vel;
        } else { // FEEDBACK CONTROL
            // Know Current Pose
            geometry_msgs::PoseStamped curr_pose_local;
            curr_pose_local.header.frame_id = cfg_.robot_frame_id;
            curr_pose_local.pose.orientation.w = 1;
            geometry_msgs::PoseStamped curr_pose_odom;
            curr_pose_odom.header.frame_id = cfg_.odom_frame_id;
            tf2::doTransform(curr_pose_local, curr_pose_odom, rbt2odom_);
            geometry_msgs::Pose curr_pose = curr_pose_odom.pose;

            // obtain current robot pose in odom frame

            // traj in odom frame here
            // returns a TrajPlan (poses and velocities, velocities are zero here)
            dynamic_gap::TrajPlan orig_ref = trajController_->trajGen(traj);
            
            // get point along trajectory to target/move towards
            targetTrajectoryPoseIdx_ = trajController_->targetPoseIdx(curr_pose, orig_ref);
            nav_msgs::Odometry ctrl_target_pose;
            ctrl_target_pose.header = orig_ref.header;
            ctrl_target_pose.pose.pose = orig_ref.poses.at(targetTrajectoryPoseIdx_);
            ctrl_target_pose.twist.twist = orig_ref.twist.at(targetTrajectoryPoseIdx_);
            
            geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;
            // sensor_msgs::LaserScan static_scan = *static_scan_ptr.get();
            
            raw_cmd_vel = trajController_->controlLaw(curr_pose, ctrl_target_pose, 
                                                    staticScan_, currentPeakSplineVel_);
        }
        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;

        geometry_msgs::Twist cmd_vel = trajController_->processCmdVel(raw_cmd_vel,
                        staticScan_, rbt_in_cam_lc, 
                        currRightGapPtModel_, currLeftGapPtModel_,
                        currentRbtVel_, currentRbtAcc_); 

        return cmd_vel;
    }

    std::vector<dynamic_gap::Gap> Planner::gapSetFeasibilityCheck(bool & curr_exec_gap_assoc, 
                                                                  bool & curr_exec_gap_feas) 
    {
        if (cfg_.debug.feasibility_debug_log) ROS_INFO_STREAM("[gapSetFeasibilityCheck()]");

        boost::mutex::scoped_lock gapset(gapset_mutex);
        //std::cout << "PULLING MODELS TO ACT ON" << std::endl;
        std::vector<dynamic_gap::Gap> curr_observed_gaps = associatedSimplifiedGaps_;

        //std::cout << "curr_raw_gaps size: " << curr_raw_gaps.size() << std::endl;
        //std::cout << "curr_observed_gaps size: " << curr_observed_gaps.size() << std::endl;

        //std::cout << "prev_raw_gaps size: " << prev_raw_gaps.size() << std::endl;
        //std::cout << "prev_observed_gaps size: " << prev_observed_gaps.size() << std::endl;
        //std::cout << "current robot velocity. Linear: " << currentRbtVel_.linear.x << ", " << currentRbtVel_.linear.y << ", angular: " << currentRbtVel_.angular.z << std::endl;
        //std::cout << "current raw gaps:" << std::endl;
        //printGapModels(curr_raw_gaps);

        //std::cout << "pulled current simplified associations:" << std::endl;
        //printGapAssociations(curr_observed_gaps, prev_observed_gaps, _simpAssociation_);
        
        /*
        ROS_INFO_STREAM("current raw gaps:");
        printGapModels(curr_raw_gaps);
        */
        
        int curr_left_idx = getCurrentLeftGapIndex();
        int curr_right_idx = getCurrentRightGapIndex();

        if (cfg_.debug.feasibility_debug_log) 
        {
            ROS_INFO_STREAM("    current simplified gaps:");
            printGapModels(curr_observed_gaps);
            ROS_INFO_STREAM("    current left/right indices: " << curr_left_idx << ", " << curr_right_idx);
        }
        curr_exec_gap_assoc = false;
        curr_exec_gap_feas = false;

        bool gap_i_feasible;
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        for (size_t i = 0; i < curr_observed_gaps.size(); i++) 
        {
            // obtain crossing point

            if (cfg_.debug.feasibility_debug_log) ROS_INFO_STREAM("    feasibility check for gap " << i);
            gap_i_feasible = gapFeasibilityChecker_->indivGapFeasibilityCheck(curr_observed_gaps.at(i));
            gap_i_feasible = true;

            if (gap_i_feasible) {
                curr_observed_gaps.at(i).addTerminalRightInformation();
                feasible_gap_set.push_back(curr_observed_gaps.at(i));
                // ROS_INFO_STREAM("Pushing back gap with peak velocity of : " << curr_observed_gaps.at(i).peakVelX_ << ", " << curr_observed_gaps.at(i).peakVelY_);
            }

            if (curr_observed_gaps.at(i).leftGapPtModel_->get_index() == curr_left_idx && 
                curr_observed_gaps.at(i).rightGapPtModel_->get_index() == curr_right_idx) {
                curr_exec_gap_assoc = true;
                curr_exec_gap_feas = true;
            }
        }

        /*
        if (gap_associated) {
            ROS_INFO_STREAM("currently executing gap associated");
        } else {
            ROS_INFO_STREAM("currently executing gap NOT associated");
        }
        */

        return feasible_gap_set;
    }

    void Planner::getFutureScans() 
    {
        // boost::mutex::scoped_lock gapset(gapset_mutex);
        if (cfg_.debug.future_scan_propagation_debug_log) ROS_INFO_STREAM("[getFutureScans()]");
        sensor_msgs::LaserScan stored_scan = *scan_.get();
        sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();
        dynamic_laser_scan.header = stored_scan.header;
        dynamic_laser_scan.angle_min = stored_scan.angle_min;
        dynamic_laser_scan.angle_max = stored_scan.angle_max;
        dynamic_laser_scan.angle_increment = stored_scan.angle_increment;
        dynamic_laser_scan.time_increment = stored_scan.time_increment;
        dynamic_laser_scan.scan_time = stored_scan.scan_time;
        dynamic_laser_scan.range_min = stored_scan.range_min;
        dynamic_laser_scan.range_max = stored_scan.range_max;
        dynamic_laser_scan.ranges = stored_scan.ranges;

        float t_i = 0.0;
        futureScans_[0] = dynamic_laser_scan; // at t = 0.0

        std::vector<Eigen::Matrix<float, 4, 1> > currentAgents;
        
        if (cfg_.planning.egocircle_prop_cheat) 
        {
            Eigen::Matrix<float, 4, 1> agent_i_state;
            currentAgents.clear();
            for (int i = 0; i < currentAgentCount_; i++) 
            {
                agent_i_state << currentTrueAgentPoses_[i].position.x, currentTrueAgentPoses_[i].position.y, 
                                 currentTrueAgentVels_[i].vector.x, currentTrueAgentVels_[i].vector.y;
                currentAgents.push_back(agent_i_state);
            }
        } else {
            currentAgents = currentEstimatedAgentStates_;
        }

        if (cfg_.debug.future_scan_propagation_debug_log) 
        {
            ROS_INFO_STREAM("    detected agents: ");
            for (int i = 0; i < currentAgents.size(); i++)
                ROS_INFO_STREAM("        agent" << i << " position: " << currentAgents[i][0] << ", " << currentAgents[i][1] << ", velocity: " << currentAgents[i][2] << ", " << currentAgents[i][3]);
        }
        
        int future_scan_idx;
        for (float t_iplus1 = cfg_.traj.integrate_stept; t_iplus1 <= cfg_.traj.integrate_maxt; t_iplus1 += cfg_.traj.integrate_stept) 
        {
            dynamic_laser_scan.ranges = stored_scan.ranges;

            //trajScorer_->recoverDynamicEgocircleCheat(t_i, t_iplus1, agentPoses__lc, agentVels__lc, dynamic_laser_scan, print);
            trajScorer_->recoverDynamicEgoCircle(t_i, t_iplus1, currentAgents, dynamic_laser_scan, cfg_.debug.future_scan_propagation_debug_log);
            
            future_scan_idx = (int) (t_iplus1 / cfg_.traj.integrate_stept);
            // ROS_INFO_STREAM("adding scan from " << t_i << " to " << t_iplus1 << " at idx: " << future_scan_idx);
            futureScans_[future_scan_idx] = dynamic_laser_scan;

            t_i = t_iplus1;
        }
    }

    geometry_msgs::PoseArray Planner::runPlanningLoop() 
    {
        // float getPlan_start_time = ros::WallTime::now().toSec();

        // float start_time = ros::WallTime::now().toSec();      
        // std::chrono::steady_clock::time_point start_time_c;

        bool curr_exec_gap_assoc, curr_exec_gap_feas;

        std::chrono::steady_clock::time_point plan_loop_start_time = std::chrono::steady_clock::now();

        ///////////////////////////
        // GAP FEASIBILITY CHECK //
        ///////////////////////////
        int gaps_size;
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        std::chrono::steady_clock::time_point feasibility_start_time = std::chrono::steady_clock::now();
        if (cfg_.planning.dynamic_feasibility_check)
        {
            try 
            { 
                feasible_gap_set = gapSetFeasibilityCheck(curr_exec_gap_assoc, curr_exec_gap_feas);
            } catch (...) 
            {
                ROS_FATAL_STREAM("out of range in gapSetFeasibilityCheck");
            }
            gaps_size = feasible_gap_set.size();
        } else
        {
            feasible_gap_set = associatedSimplifiedGaps_;
            // need to set feasible to true for all gaps as well
        }


        float feasibility_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - feasibility_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[gapSetFeasibilityCheck() for " << gaps_size << " gaps: " << feasibility_time << " seconds]");
        
        /////////////////////////////
        // FUTURE SCAN PROPAGATION //
        /////////////////////////////
        std::chrono::steady_clock::time_point future_scans_start_time = std::chrono::steady_clock::now();
        try 
        {
            getFutureScans();
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in getFutureScans");
        }

        float future_scans_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - future_scans_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[getFutureScans() for " << gaps_size << " gaps: " << future_scans_time << " seconds]");
        
        //////////////////////
        // GAP MANIPULATION //
        //////////////////////
        std::chrono::steady_clock::time_point gap_manipulation_start_time = std::chrono::steady_clock::now();
        std::vector<dynamic_gap::Gap> manip_gap_set;
        try 
        {
            manip_gap_set = gapManipulate(feasible_gap_set);
        } catch (std::out_of_range) 
        {
            ROS_INFO_STREAM("out of range in gapManipulate");
        }

        float gap_manipulation_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_manipulation_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[gapManipulate() for " << gaps_size << " gaps: " << gap_manipulation_time << " seconds]");

        ///////////////////////////////////////////
        // GAP TRAJECTORY GENERATION AND SCORING //
        ///////////////////////////////////////////
        std::chrono::steady_clock::time_point gap_trajectory_generation_start_time = std::chrono::steady_clock::now();
        std::vector<geometry_msgs::PoseArray> traj_set;
        std::vector<std::vector<float>> time_set;
        std::vector<std::vector<float>> score_set; 
        try 
        {
            score_set = initialTrajGen(manip_gap_set, traj_set, time_set);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in initialTrajGen");
        }

        float gap_trajectory_generation_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_trajectory_generation_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[initialTrajGen() for " << gaps_size << " gaps: " << gap_trajectory_generation_time << " seconds]");

        visualizeComponents(manip_gap_set); // need to run after initialTrajGen to see what weights for reachable gap are

        //////////////////////////////
        // GAP TRAJECTORY SELECTION //
        //////////////////////////////
        std::chrono::steady_clock::time_point gap_trajectory_selection_start_time = std::chrono::steady_clock::now();
        int traj_idx;
        try 
        {
            traj_idx = pickTraj(traj_set, score_set);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in pickTraj");
        }

        float gap_trajectory_selection_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_trajectory_selection_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[pickTraj() for " << gaps_size << " gaps: " << gap_trajectory_selection_time << " seconds]");

        geometry_msgs::PoseArray chosen_traj;
        std::vector<float> chosen_time_arr;
        dynamic_gap::Gap chosen_gap;
        if (traj_idx >= 0) 
        {
            chosen_traj = traj_set[traj_idx];
            chosen_time_arr = time_set[traj_idx];
            chosen_gap = manip_gap_set[traj_idx];
        } else {
            chosen_traj = geometry_msgs::PoseArray();
            chosen_gap = dynamic_gap::Gap();
        }

        ///////////////////////////////
        // GAP TRAJECTORY COMPARISON //
        ///////////////////////////////
        std::chrono::steady_clock::time_point gap_trajectory_comparison_start_time = std::chrono::steady_clock::now();

        geometry_msgs::PoseArray final_traj;
        try 
        {
            final_traj = compareToOldTraj(chosen_traj, chosen_gap, manip_gap_set, chosen_time_arr, curr_exec_gap_assoc, curr_exec_gap_feas);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in compareToOldTraj");
        }

        float gap_trajectory_comparison_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_trajectory_comparison_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[compareToOldTraj() for " << gaps_size << " gaps: "  << gap_trajectory_comparison_time << " seconds]");

        float plan_loop_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - plan_loop_start_time).count() / 1.0e6;

        ROS_INFO_STREAM("planning loop for " << gaps_size << " gaps: "  << plan_loop_time << " seconds");
        
        // geometry_msgs::PoseArray final_traj;

        return final_traj;
    }

    void Planner::visualizeComponents(std::vector<dynamic_gap::Gap> manip_gap_set) {
        boost::mutex::scoped_lock gapset(gapset_mutex);

        gapVisualizer_->drawManipGaps(manip_gap_set, std::string("manip"));
        // gapVisualizer_->drawReachableGaps(manip_gap_set);        
        // gapVisualizer_->drawReachableGapsCenters(manip_gap_set); 

        // gapVisualizer_->drawGapSplines(manip_gap_set);
        // goalvisualizer->drawGapGoals(manip_gap_set);
    }

    void Planner::printGapAssociations(const std::vector<dynamic_gap::Gap> & current_gaps, 
                                       const std::vector<dynamic_gap::Gap> & previous_gaps, 
                                       const std::vector<int> & association) 
    {
        std::cout << "current simplified associations" << std::endl;
        std::cout << "number of gaps: " << current_gaps.size() << ", number of previous gaps: " << previous_gaps.size() << std::endl;
        std::cout << "association size: " << association.size() << std::endl;
        for (int i = 0; i < association.size(); i++) {
            std::cout << association[i] << ", ";
        }
        std::cout << "" << std::endl;

        float curr_x, curr_y, prev_x, prev_y;
        for (int i = 0; i < association.size(); i++) {
            std::vector<int> pair{i, association[i]};
            std::cout << "pair (" << i << ", " << association[i] << "). ";
            int current_gap_idx = int(std::floor(pair[0] / 2.0));
            int previous_gap_idx = int(std::floor(pair[1] / 2.0));

            if (pair[0] % 2 == 0) {  // curr left
                    current_gaps.at(current_gap_idx).getSimplifiedRCartesian(curr_x, curr_y);
                } else { // curr right
                    current_gaps.at(current_gap_idx).getSimplifiedLCartesian(curr_x, curr_y);
            }
            
            if (i >= 0 && association[i] >= 0) {
                if (pair[1] % 2 == 0) { // prev left
                    previous_gaps.at(previous_gap_idx).getSimplifiedRCartesian(prev_x, prev_y);
                } else { // prev right
                    previous_gaps.at(previous_gap_idx).getSimplifiedLCartesian(prev_x, prev_y);
                }
                std::cout << "From (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << simpDistMatrix_[pair[0]][pair[1]] << std::endl;
            } else {
                std::cout << "From NULL to (" << curr_x << ", " <<  curr_y << ")" << std::endl;
            }
        }
    }

    void Planner::printGapModels(const std::vector<dynamic_gap::Gap> & gaps) 
    {
        // THIS IS NOT FOR MANIPULATED GAPS
        float x,y;
        for (size_t i = 0; i < gaps.size(); i++)
        {
            dynamic_gap::Gap gap = gaps.at(i);
            ROS_INFO_STREAM("    gap " << i << ", indices: " << gap.RIdx() << " to "  << gap.LIdx() << ", left model: " << gap.leftGapPtModel_->get_index() << ", rightGapPtModel: " << gap.rightGapPtModel_->get_index());
            Eigen::Matrix<float, 4, 1> left_state = gap.leftGapPtModel_->getState();
            gap.getLCartesian(x, y);            
            ROS_INFO_STREAM("        left point: (" << x << ", " << y << "), left model: (" << left_state[0] << ", " << left_state[1] << ", " << left_state[2] << ", " << left_state[3] << ")");
            Eigen::Matrix<float, 4, 1> right_state = gap.rightGapPtModel_->getState();
            gap.getRCartesian(x, y);
            ROS_INFO_STREAM("        right point: (" << x << ", " << y << "), right model: (" << right_state[0] << ", " << right_state[1] << ", " << right_state[2] << ", " << right_state[3] << ")");
        }
    }

    bool Planner::recordAndCheckVel(geometry_msgs::Twist cmd_vel) 
    {
        float val = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.linear.y) + std::abs(cmd_vel.angular.z);
        log_vel_comp.push_back(val);
        float cum_vel_sum = std::accumulate(log_vel_comp.begin(), log_vel_comp.end(), float(0));
        bool ret_val = cum_vel_sum > 1.0 || !log_vel_comp.full();
        if (!ret_val && !cfg_.man.man_ctrl) {
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            ROS_INFO_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        return ret_val || cfg_.man.man_ctrl;
    }

}