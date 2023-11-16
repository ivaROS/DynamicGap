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
        rbtPoseInRbtFrame_.pose.orientation.w = 1;
        rbtPoseInRbtFrame_.header.frame_id = cfg_.robot_frame_id;

        cmdVelBuffer_.set_capacity(cfg_.planning.halt_size);

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

        // TODO: change this to a vector of pointers to laser scans
        sensor_msgs::LaserScan tmpScan = sensor_msgs::LaserScan();
        futureScans_ = std::vector<sensor_msgs::LaserScan>(int(cfg_.traj.integrate_maxt/cfg_.traj.integrate_stept) + 1, tmpScan);
        // futureScans_.push_back(tmpScan);
        // for (float t_iplus1 = cfg_.traj.integrate_stept; t_iplus1 <= cfg_.traj.integrate_maxt; t_iplus1 += cfg_.traj.integrate_stept) 
        //     futureScans_.push_back(tmpScan);

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
        float globalGoalXDiff = globalGoalOdomFrame_.pose.position.x - robotPoseOdomFrame_.pose.position.x;
        float globalGoalYDiff = globalGoalOdomFrame_.pose.position.y - robotPoseOdomFrame_.pose.position.y;
        bool reachedGlobalGoal = sqrt(pow(globalGoalXDiff, 2) + pow(globalGoalYDiff, 2)) < cfg_.goal.goal_tolerance;
        
        if (reachedGlobalGoal)
        {
            ROS_INFO_STREAM("[Reset] Goal Reached");
            return true;
        }

        float globalPathLocalWaypointDiffX = globalPathLocalWaypointOdomFrame_.pose.position.x - robotPoseOdomFrame_.pose.position.x;
        float globalPathLocalWaypointDiffY = globalPathLocalWaypointOdomFrame_.pose.position.y - robotPoseOdomFrame_.pose.position.y;
        bool reachedGlobalPathLocalWaypoint = sqrt(pow(globalPathLocalWaypointDiffX, 2) + pow(globalPathLocalWaypointDiffY, 2)) < cfg_.goal.waypoint_tolerance;
        if (reachedGlobalPathLocalWaypoint)
            ROS_INFO_STREAM("[Reset] Waypoint reached, getting new one");

        return false;
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock gapset(gapsetMutex);
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

            // gapVisualizer_->drawGaps(associatedRawGaps_, std::string("raw"));
            // gapVisualizer_->drawGapsModels(associatedRawGaps_);
            gapVisualizer_->drawGaps(associatedSimplifiedGaps_, std::string("simp"));
            gapVisualizer_->drawGapsModels(associatedSimplifiedGaps_);

            readyToPlan = true;
        }

        goalSelector_->updateEgoCircle(scan_);
        goalSelector_->generateGlobalPathLocalWaypoint(map2rbt_);
        geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame = goalSelector_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);
        goalvisualizer->drawGlobalPathLocalWaypoint(globalPathLocalWaypointOdomFrame);

        // ROS_INFO_STREAM("Time elapsed after updating goal selector: " << (ros::WallTime::now().toSec() - start_time));

        trajScorer_->updateEgoCircle(scan_);
        trajScorer_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame, odom2rbt_);


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
            // thetaTilde = idx2theta(gap.LIdx()); // float(g.LIdx() - g.half_scan) / g.half_scan * M_PI;
            // rangeTilde = gap.LDist();
            gap.getLCartesian(rX, rY);            
		} else 
        {
			// thetaTilde = idx2theta(gap.RIdx()); // float(g.RIdx() - g.half_scan) / g.half_scan * M_PI;
            // rangeTilde = gap.RDist();
            gap.getRCartesian(rX, rY);            

		}

        // ROS_INFO_STREAM("rX: " << rX << ", rY: " << rY);

		// Eigen::Vector2f laserscan_measurement(rangeTilde, thetaTilde);
        Eigen::Vector2f measurement(rX, rY);

        if (idx % 2 == 0) 
        {
            //std::cout << "entering left model update" << std::endl;
            try 
            {
                gap.leftGapPtModel_->update(measurement, 
                                            intermediateRbtVels, intermediateRbtAccs, 
                                            print, currentTrueAgentPoses_, 
                                            currentTrueAgentVels_,
                                            tCurrentFilterUpdate);
            } catch (...) 
            {
                ROS_INFO_STREAM("left estimator update fails");
            }
        } else {
            //std::cout << "entering right model update" << std::endl;
            try 
            {
                gap.rightGapPtModel_->update(measurement, 
                                            intermediateRbtVels, intermediateRbtAccs, 
                                            print, currentTrueAgentPoses_, 
                                            currentTrueAgentVels_,
                                            tCurrentFilterUpdate);
            } catch (...) 
            {
                ROS_INFO_STREAM("right estimator update fails");
            }
        }
    }

    void Planner::jointPoseAccCB(const nav_msgs::Odometry::ConstPtr & rbtOdomMsg, 
                                 const geometry_msgs::TwistStamped::ConstPtr & rbtAccelMsg)
    {
        // ROS_INFO_STREAM("joint pose acc cb");

        // ROS_INFO_STREAM("accel time stamp: " << rbtAccelMsg->header.stamp.toSec());
        currentRbtAcc_ = *rbtAccelMsg;
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

        // ROS_INFO_STREAM("odom time stamp: " << rbtOdomMsg->header.stamp.toSec());

        // ROS_INFO_STREAM("acc - odom time difference: " << (rbtAccelMsg->header.stamp - rbtOdomMsg->header.stamp).toSec());

        updateTF();

        // Transform the msg to odom frame
        if (rbtOdomMsg->header.frame_id != cfg_.odom_frame_id)
        {
            //std::cout << "odom msg is not in odom frame" << std::endl;
            geometry_msgs::TransformStamped msgFrame2OdomFrame = tfBuffer_.lookupTransform(cfg_.odom_frame_id, rbtOdomMsg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped rbtPoseMsgFrame, rbtPoseOdomFrame;
            rbtPoseMsgFrame.header = rbtOdomMsg->header;
            rbtPoseMsgFrame.pose = rbtOdomMsg->pose.pose;

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;

            tf2::doTransform(rbtPoseMsgFrame, rbtPoseOdomFrame, msgFrame2OdomFrame);
            robotPoseOdomFrame_ = rbtPoseOdomFrame;
        }
        else
        {
            robotPoseOdomFrame_.pose = rbtOdomMsg->pose.pose;
        }

        //--------------- VELOCITY -------------------//
        // velocity always comes in wrt robot frame in STDR
        geometry_msgs::TwistStamped incomingRbtVel;
        incomingRbtVel.header = rbtOdomMsg->header;
        incomingRbtVel.header.frame_id = rbtAccelMsg->header.frame_id;
        incomingRbtVel.twist = rbtOdomMsg->twist.twist;

        currentRbtVel_ = incomingRbtVel;
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
    
    void Planner::agentOdomCB(const nav_msgs::Odometry::ConstPtr& agentOdomMsg) 
    {
        std::string agentNamespace = agentOdomMsg->child_frame_id;
        // ROS_INFO_STREAM("agentNamespace: " << agentNamespace);
        agentNamespace.erase(0,5); // removing "robot" from "robotN"
        char * robotChars = strdup(agentNamespace.c_str());
        int agentID = std::atoi(robotChars);
        // ROS_INFO_STREAM("agentID: " << agentID);

        try 
        {
            // transforming Odometry message from map_static to robotN
            geometry_msgs::TransformStamped msgFrame2RobotFrame = tfBuffer_.lookupTransform(cfg_.robot_frame_id, agentOdomMsg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped agentPoseMsgFrame, agentPoseRobotFrame;
            agentPoseMsgFrame.header = agentOdomMsg->header;
            agentPoseMsgFrame.pose = agentOdomMsg->pose.pose;
            // ROS_INFO_STREAM("updating inpose " << agentNamespace << " to: (" << agentPoseMsgFrame.pose.position.x << ", " << agentPoseMsgFrame.pose.position.y << ")");

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;
            tf2::doTransform(agentPoseMsgFrame, agentPoseRobotFrame, msgFrame2RobotFrame);
            
            // ROS_INFO_STREAM("updating " << agentNamespace << " odom from " << agent_odom_vects[agentID][0] << ", " << agent_odom_vects[agentID][1] << " to " << odom_vect[0] << ", " << odom_vect[1]);
            currentTrueAgentPoses_[agentID] = agentPoseRobotFrame.pose;
        } catch (tf2::TransformException &ex) 
        {
            ROS_INFO_STREAM("Odometry transform failed for " << agentNamespace);
        }
        
        try 
        {
            std::string source_frame = agentOdomMsg->child_frame_id; 
            // std::cout << "in agentOdomCB" << std::endl;
            // std::cout << "transforming from " << source_frame << " to " << cfg_.robot_frame_id << std::endl;
            geometry_msgs::TransformStamped msgFrame2RobotFrame = tfBuffer_.lookupTransform(cfg_.robot_frame_id, source_frame, ros::Time(0));
            geometry_msgs::Vector3Stamped agentVelMsgFrame, agentVelRobotFrame;
            agentVelMsgFrame.header = agentOdomMsg->header;
            agentVelMsgFrame.header.frame_id = source_frame;
            agentVelMsgFrame.vector = agentOdomMsg->twist.twist.linear;
            // std::cout << "incoming vector: " << agentVelMsgFrame.vector.x << ", " << agentVelMsgFrame.vector.y << std::endl;
            tf2::doTransform(agentVelMsgFrame, agentVelRobotFrame, msgFrame2RobotFrame);
            // std::cout << "outcoming vector: " << agentVelRobotFrame.vector.x << ", " << agentVelRobotFrame.vector.y << std::endl;

            currentTrueAgentVels_[agentID] = agentVelRobotFrame;
        } catch (tf2::TransformException &ex) 
        {
            ROS_INFO_STREAM("Velocity transform failed for " << agentNamespace);
        }            
    }

    bool Planner::setGoal(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        if (globalPlanMapFrame.size() == 0) return true;
        geometry_msgs::PoseStamped globalGoalMapFrame = *std::prev(globalPlanMapFrame.end());
        tf2::doTransform(globalGoalMapFrame, globalGoalOdomFrame_, map2odom_); // to update odom frame parameter
        tf2::doTransform(globalGoalOdomFrame_, globalGoalRobotFrame_, odom2rbt_); // to update robot frame parameter
        
        // Store New Global Plan to Goal Selector
        goalSelector_->updateGlobalPathMapFrame(globalPlanMapFrame);
        
        // Generate global path local waypoint (furthest part along global path that we can still see)
        goalSelector_->generateGlobalPathLocalWaypoint(map2rbt_);

        // return local goal (odom) frame
        geometry_msgs::PoseStamped newglobalPathLocalWaypointOdomFrame = goalSelector_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);

        {
            // Plan New
            float diffX = globalPathLocalWaypointOdomFrame_.pose.position.x - newglobalPathLocalWaypointOdomFrame.pose.position.x;
            float diffY = globalPathLocalWaypointOdomFrame_.pose.position.y - newglobalPathLocalWaypointOdomFrame.pose.position.y;
            
            if (sqrt(pow(diffX, 2) + pow(diffY, 2)) > cfg_.goal.waypoint_tolerance)
                globalPathLocalWaypointOdomFrame_ = newglobalPathLocalWaypointOdomFrame;
        }

        // Set new local goal to trajectory arbiter
        trajScorer_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame_, odom2rbt_);

        // Visualization only
        std::vector<geometry_msgs::PoseStamped> visibleGlobalPlanSnippetRobotFrame = goalSelector_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
        trajVisualizer_->drawRelevantGlobalPlanSnippet(visibleGlobalPlanSnippetRobotFrame);
        // trajVisualizer_->drawGlobalPlan(goalselector->getGlobalPathOdomFrame());

        hasGlobalGoal_ = true;
        return true;
    }

    void Planner::updateTF()
    {
        try 
        {
            // for lookupTransform, parameters are (destination frame, source frame)
            map2rbt_  = tfBuffer_.lookupTransform(cfg_.robot_frame_id, cfg_.map_frame_id, ros::Time(0));
            odom2rbt_ = tfBuffer_.lookupTransform(cfg_.robot_frame_id, cfg_.odom_frame_id, ros::Time(0));
            rbt2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.robot_frame_id, ros::Time(0));
            map2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.map_frame_id, ros::Time(0));
            cam2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.sensor_frame_id, ros::Time(0));
            rbt2cam_ = tfBuffer_.lookupTransform(cfg_.sensor_frame_id, cfg_.robot_frame_id, ros::Time(0));

            tf2::doTransform(rbtPoseInRbtFrame_, rbtPoseInSensorFrame_, rbt2cam_);
        } catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            return;
        }
    }

    std::vector<dynamic_gap::Gap> Planner::gapManipulate(const std::vector<dynamic_gap::Gap> & gaps) 
    {
        if (cfg_.debug.manipulation_debug_log) ROS_INFO_STREAM("[gapManipulate()]");

        boost::mutex::scoped_lock gapset(gapsetMutex);
        std::vector<dynamic_gap::Gap> manipulatedGaps = gaps;

        for (size_t i = 0; i < manipulatedGaps.size(); i++)
        {
            if (cfg_.debug.manipulation_debug_log) ROS_INFO_STREAM("    manipulating initial gap " << i);

            manipulatedGaps[i].initManipIndices();

            // MANIPULATE POINTS AT T=0            
            // gapManipulator_->reduceGap(manipulatedGaps[i], goalSelector_->getGlobalPathLocalWaypointRobotFrame(), true);
            gapManipulator_->convertRadialGap(manipulatedGaps[i], true); 
            gapManipulator_->inflateGapSides(manipulatedGaps[i], true);
            gapManipulator_->radialExtendGap(manipulatedGaps[i], true);
            gapManipulator_->setGapWaypoint(manipulatedGaps[i], goalSelector_->getGlobalPathLocalWaypointRobotFrame(), true);
            
            // MANIPULATE POINTS AT T=1
            if (cfg_.debug.manipulation_debug_log) ROS_INFO_STREAM("    manipulating terminal gap " << i);
            gapManipulator_->updateDynamicEgoCircle(manipulatedGaps[i], futureScans_);
            if ((!manipulatedGaps[i].crossed_ && !manipulatedGaps[i].closed_) || (manipulatedGaps[i].crossedBehind_)) 
            {
                // gapManipulator_->reduceGap(manipulatedGaps[i], goalSelector_->getGlobalPathLocalWaypointRobotFrame(), false);
                gapManipulator_->convertRadialGap(manipulatedGaps[i], false);
            }
            gapManipulator_->inflateGapSides(manipulatedGaps[i], false);
            gapManipulator_->radialExtendGap(manipulatedGaps[i], false);
            gapManipulator_->setTerminalGapWaypoint(manipulatedGaps[i], goalSelector_->getGlobalPathLocalWaypointRobotFrame());
        }

        return manipulatedGaps;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<float>> Planner::generateGapTrajs(std::vector<dynamic_gap::Gap>& gaps, 
                                                            std::vector<geometry_msgs::PoseArray>& generatedPaths, 
                                                            std::vector<std::vector<float>>& generatedPathTimings) 
    {
        boost::mutex::scoped_lock gapset(gapsetMutex);

        if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("[generateGapTrajs()]");
        std::vector<geometry_msgs::PoseArray> paths(gaps.size());
        std::vector<std::vector<float>> pathTimings(gaps.size());
        std::vector<std::vector<float>> pathPoseScores(gaps.size());
        // geometry_msgs::PoseStamped rbtPoseInSensorFrame_lc = rbtPoseInSensorFrame; // lc as local copy

        // std::vector<dynamic_gap::Gap> rawGaps = associatedRawGaps_;
        try 
        {
            for (size_t i = 0; i < gaps.size(); i++) 
            {
                if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    generating traj for gap: " << i);
                // std::cout << "starting generate trajectory with rbtPoseInSensorFrame_lc: " << rbtPoseInSensorFrame_lc.pose.position.x << ", " << rbtPoseInSensorFrame_lc.pose.position.y << std::endl;
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                std::tuple<geometry_msgs::PoseArray, std::vector<float>> traj;
                
                // TRAJECTORY GENERATED IN RBT FRAME
                bool runGoToGoal = true; // (vec.at(i).goal.goalwithin || vec.at(i).artificial);
                if (runGoToGoal) 
                {
                    if (cfg_.debug.traj_debug_log) 
                        ROS_INFO_STREAM("        running goToGoal");

                    std::tuple<geometry_msgs::PoseArray, std::vector<float>> goToGoalTraj;
                    goToGoalTraj = gapTrajGenerator_->generateTrajectory(gaps[i], rbtPoseInSensorFrame_, currentRbtVel_, runGoToGoal);
                    goToGoalTraj = gapTrajGenerator_->processTrajectory(goToGoalTraj);
                    std::vector<float> goToGoalPoseScores = trajScorer_->scoreTrajectory(std::get<0>(goToGoalTraj), std::get<1>(goToGoalTraj), 
                                                                                        associatedRawGaps_, 
                                                                                        futureScans_);
                    float goToGoalScore = std::accumulate(goToGoalPoseScores.begin(), goToGoalPoseScores.end(), float(0));
                    if (cfg_.debug.traj_debug_log) 
                        ROS_INFO_STREAM("        goToGoalScore: " << goToGoalScore);

                    if (cfg_.debug.traj_debug_log) 
                        ROS_INFO_STREAM("        running ahpf");
                    std::tuple<geometry_msgs::PoseArray, std::vector<float>> ahpfTraj;
                    ahpfTraj = gapTrajGenerator_->generateTrajectory(gaps[i], rbtPoseInSensorFrame_, currentRbtVel_, !runGoToGoal);
                    ahpfTraj = gapTrajGenerator_->processTrajectory(ahpfTraj);
                    std::vector<float> ahpfPoseScores = trajScorer_->scoreTrajectory(std::get<0>(ahpfTraj), std::get<1>(ahpfTraj), 
                                                                                     associatedRawGaps_, 
                                                                                     futureScans_);
                    float ahpfScore = std::accumulate(ahpfPoseScores.begin(), ahpfPoseScores.end(), float(0));
                    if (cfg_.debug.traj_debug_log) 
                        ROS_INFO_STREAM("        ahpfScore: " << ahpfScore);

                    if (goToGoalScore > ahpfScore)
                    {
                        traj = goToGoalTraj;
                        pathPoseScores[i] = goToGoalPoseScores;
                    } else
                    {
                        traj = ahpfTraj;
                        pathPoseScores[i] = ahpfPoseScores;
                    }
                    // traj =  ? goToGoalTraj : ahpfTraj;
                    // pathPoseScores.at(i) = (goToGoalScore > ahpfScore) ? goToGoalPoseScores : ahpfPoseScores;
                } else 
                {
                    traj = gapTrajGenerator_->generateTrajectory(gaps[i], rbtPoseInSensorFrame_, currentRbtVel_, runGoToGoal);
                    traj = gapTrajGenerator_->processTrajectory(traj);

                    if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    scoring trajectory for gap: " << i);
                    pathPoseScores.at(i) = trajScorer_->scoreTrajectory(std::get<0>(traj), std::get<1>(traj), 
                                                                        associatedRawGaps_, 
                                                                        futureScans_);  
                    // ROS_INFO_STREAM("done with scoreTrajectory");
                }

                // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                paths.at(i) = gapTrajGenerator_->transformLocalTrajectory(std::get<0>(traj), cam2odom_, cfg_.sensor_frame_id, cfg_.odom_frame_id);
                pathTimings.at(i) = std::get<1>(traj);
            }
        } catch (...) 
        {
            ROS_FATAL_STREAM("generateGapTrajs");
        }

        trajVisualizer_->drawGapTrajectoryPoseScores(paths, pathPoseScores);
        trajVisualizer_->drawGapTrajectories(paths);
        generatedPaths = paths;
        generatedPathTimings = pathTimings;
        return pathPoseScores;
    }

    int Planner::pickTraj(const std::vector<geometry_msgs::PoseArray> & paths, const std::vector<std::vector<float>> & pathPoseScores) 
    {
        if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("[pickTraj()]");
        // ROS_INFO_STREAM_NAMED("pg_trajCount", "pg_trajCount, " << paths.size());
        if (paths.size() == 0) 
        {
            ROS_WARN_STREAM("No traj synthesized");
            return -1;
        }

        if (paths.size() != pathPoseScores.size()) 
        {
            ROS_FATAL_STREAM("pickTraj size mismatch: paths = " << paths.size() << " != pathPoseScores =" << pathPoseScores.size());
            return -1;
        }

        // poses here are in odom frame 
        std::vector<float> pathScores(paths.size());
        // int counts;
        // try 
        // {
            // if (omp_get_dynamic()) omp_set_dynamic(0);
        for (size_t i = 0; i < pathScores.size(); i++) 
        {
            // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());
            // counts = std::min(cfg_.planning.num_feasi_check, int(pathPoseScores.at(i).size()));

            pathScores.at(i) = std::accumulate(pathPoseScores.at(i).begin(), pathPoseScores.at(i).end(), float(0));
            pathScores.at(i) = paths.at(i).poses.size() == 0 ? -std::numeric_limits<float>::infinity() : pathScores.at(i);
            
            if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    for gap " << i << " (length: " << paths.at(i).poses.size() << "), returning score of " << pathScores.at(i));
            /*
            if (pathScores.at(i) == -std::numeric_limits<float>::infinity()) {
                for (size_t j = 0; j < counts; j++) {
                    if (score.at(i).at(j) == -std::numeric_limits<float>::infinity()) {
                        std::cout << "-inf score at idx " << j << " of " << counts << std::endl;
                    }
                }
            }
            */
        }
        // } catch (...) 
        // {
        //     ROS_FATAL_STREAM("pickTraj");
        // }

        auto highestPathScoreIter = std::max_element(pathScores.begin(), pathScores.end());
        int highestPathScoreIdx = std::distance(pathScores.begin(), highestPathScoreIter);

        if (pathScores.at(highestPathScoreIdx) == -std::numeric_limits<float>::infinity()) 
        {    
            ROS_INFO_STREAM("    all -infinity");
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (float pathScore : pathScores) 
            {
                ROS_INFO_STREAM("Score: " << pathScore);
            }
            ROS_INFO_STREAM("------------------");
        }

        if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("    picking gap: " << highestPathScoreIdx);
        
        return highestPathScoreIdx;
    }

    geometry_msgs::PoseArray Planner::changeTrajectoryHelper(const dynamic_gap::Gap & incomingGap, 
                                                             const geometry_msgs::PoseArray & incomingPath, 
                                                             const std::vector<float> & incomingPathTiming, 
                                                             bool switchToIncoming) 
    {
        trajectoryChangeCount_++;

        if (switchToIncoming) 
        {
            setCurrentPath(incomingPath);
            setCurrentPathTiming(incomingPathTiming);
            setCurrentLeftModel(incomingGap.leftGapPtModel_);
            setCurrentRightModel(incomingGap.rightGapPtModel_);
            setCurrentGapPeakVelocities(incomingGap.peakVelX_, incomingGap.peakVelY_);
            currentTrajectoryPublisher_.publish(incomingPath);          
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, incomingPath);

            return incomingPath;  
        } else 
        {
            geometry_msgs::PoseArray emptyPath = geometry_msgs::PoseArray();
            emptyPath.header = incomingPath.header;
            std::vector<float> emptyPathTiming;
            setCurrentPath(emptyPath);
            setCurrentPathTiming(emptyPathTiming);
            setCurrentLeftModel(NULL);
            setCurrentRightModel(NULL);
            setCurrentGapPeakVelocities(0.0, 0.0);
            currentTrajectoryPublisher_.publish(emptyPath);
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, emptyPath);
            return emptyPath;            
        }                       
    }

    geometry_msgs::PoseArray Planner::compareToCurrentTraj(const dynamic_gap::Gap & incomingGap, 
                                                       const geometry_msgs::PoseArray & incomingPathOdomFrame,                                                        
                                                       const std::vector<float> & incomingPathTiming,
                                                       const std::vector<dynamic_gap::Gap> & feasibleGaps, 
                                                       bool isIncomingGapFeasibleInput) // bool isIncomingGapAssociated,
    {
        if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("[compareToCurrentTraj()]");
        boost::mutex::scoped_lock gapset(gapsetMutex);
        
        geometry_msgs::PoseArray currentPath = getCurrentPath();
        std::vector<float> currentPathTiming = getCurrentPathTiming();

        // std::vector<dynamic_gap::Gap> curr_raw_gaps = associatedRawGaps_;

        try 
        {
            // float curr_time = ros::WallTime::now().toSec();
            
            // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapPtModelID() << ", current right gap index: " << getCurrentRightGapPtModelID());

            // First, checking if the current gap we are within is still valid (associated and feasible)
            // TODO: remove associated because feasible assumes associated
            bool isIncomingGapFeasible = true; // (isIncomingGapFeasibleInput);
            for (const dynamic_gap::Gap & gap : feasibleGaps) 
            {
                // ROS_INFO_STREAM("feasible left gap index: " << g.leftGapPtModel_->getID() << ", feasible right gap index: " << g.rightGapPtModel_->getID());
                if (gap.leftGapPtModel_->getID() == getCurrentLeftGapPtModelID() && gap.rightGapPtModel_->getID() == getCurrentRightGapPtModelID()) 
                {
                    setCurrentGapPeakVelocities(gap.peakVelX_, gap.peakVelY_);
                    break;
                }
            }

            // std::cout << "current traj length: " << currentPath.poses.size() << std::endl;
            // ROS_INFO_STREAM("current gap indices: " << getCurrentLeftGapPtModelID() << ", " << getCurrentRightGapPtModelID());
            //std::cout << "current time length: " << currentPathTiming.size() << std::endl;
            //std::cout << "chosenPath traj length: " << chosenPath.poses.size() << std::endl;
            //std::cout << "chosenPath time length: " << time_arr.size() << std::endl;
            
            //////////////////////////////////////////////////////////////////////////////
            // Transform into the current robot frame to score against the current scan //
            //////////////////////////////////////////////////////////////////////////////
            geometry_msgs::PoseArray incomingPathRobotFrame = gapTrajGenerator_->transformLocalTrajectory(incomingPathOdomFrame, odom2rbt_, 
                                                                                                        cfg_.odom_frame_id, cfg_.robot_frame_id);
            // incomingPathRobotFrame.header.frame_id = cfg_.robot_frame_id;

            if (cfg_.debug.traj_debug_log) 
                ROS_INFO_STREAM("    scoring incoming trajectory");
            std::vector<float> incomingPathPoseScores = trajScorer_->scoreTrajectory(incomingPathRobotFrame, incomingPathTiming, 
                                                                                     associatedRawGaps_, 
                                                                                     futureScans_);
            // int counts = std::min(cfg_.planning.num_feasi_check, (int) std::min(incomingPathPoseScores.size(), curr_score.size()));

            // int counts = std::min(cfg_.planning.num_feasi_check, (int) incomingPathPoseScores.size());
            float incomingPathScore = std::accumulate(incomingPathPoseScores.begin(), incomingPathPoseScores.end(), float(0));
            if (cfg_.debug.traj_debug_log) 
                ROS_INFO_STREAM("    incoming trajectory received a score of: " << incomingPathScore);
          

            ///////////////////////////////////////////////////////////////////////
            //  Evaluate the incoming path to determine if we can switch onto it //
            ///////////////////////////////////////////////////////////////////////
            std::string incomingPathStatus = "incoming path is safe to switch onto"; // (incomingPathOdomFrame.poses.size() > 0) ? "incoming traj length 0" : "incoming score infinite";

            bool ableToSwitchToIncomingPath = true;
            if (incomingPathOdomFrame.poses.size() == 0)
            {
                incomingPathStatus = "incoming path is of length zero.";
                ableToSwitchToIncomingPath = false;
            } else if (incomingPathScore == -std::numeric_limits<float>::infinity())
            {
                incomingPathStatus = "incoming path is of score -infinity.";
                ableToSwitchToIncomingPath = false;
            } else if (!isIncomingGapFeasible)
            {
                incomingPathStatus = "incoming path is not feasible.";
                ableToSwitchToIncomingPath = false;
            }
                

            ///////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing path is empty (size: 0) //
            ///////////////////////////////////////////////////////////////////////////////////
            bool isCurrentPathEmpty = currentPath.poses.size() == 0;
            // bool curr_gap_not_feasible = ;
            if (isCurrentPathEmpty) // || !isIncomingGapValid 
            {
                std::string currentPathStatus = "";

                // std::string currentPathStatus = (isCurrentPathEmpty) ? "curr traj length 0" : "curr traj is not feasible";
                
                // if (!isIncomingGapAssociated) {
                //     currentPathStatus = "curr exec gap is not associated (left: " + std::to_string(getCurrentLeftGapPtModelID()) + ", right: " + std::to_string(getCurrentRightGapPtModelID()) + ")";
                // } else if (!isIncomingGapFeasible) {
                //     currentPathStatus = "curr exec gap is deemed infeasible";
                // }
                
                if (cfg_.debug.traj_debug_log) 
                    ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ <<  
                                            ": current path is of length zero, " << incomingPathStatus);                
                return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, ableToSwitchToIncomingPath);

                /*
                if (ableToSwitchToIncomingPath) 
                {
                    if (cfg_.debug.traj_debug_log) 
                        ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ << 
                                                " to incoming: " << currentPathStatus << ", incoming score finite");

                    return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, false);
                } else  
                {
                    if (cfg_.debug.traj_debug_log) 
                        ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ <<  
                                                " to empty: " << currentPathStatus << ", " << incomingPathStatus);
                    
                    return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, true);
                }
                */
            } 

            ////////////////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing path has been completely tracked //
            ////////////////////////////////////////////////////////////////////////////////////////////   
            geometry_msgs::PoseArray currentPathRobotFrame = gapTrajGenerator_->transformLocalTrajectory(currentPath, odom2rbt_, 
                                                                                                         cfg_.odom_frame_id, cfg_.robot_frame_id);
            // currentPathRobotFrame.header.frame_id = cfg_.robot_frame_id;
            int currentPathPoseIdx = egoTrajPosition(currentPathRobotFrame);
            geometry_msgs::PoseArray reducedCurrentPathRobotFrame = currentPathRobotFrame;
            reducedCurrentPathRobotFrame.poses = std::vector<geometry_msgs::Pose>(currentPathRobotFrame.poses.begin() + currentPathPoseIdx, currentPathRobotFrame.poses.end());

            std::vector<float> reducedCurrentPathTiming = currentPathTiming;
            reducedCurrentPathTiming = std::vector<float>(currentPathTiming.begin() + currentPathPoseIdx, currentPathTiming.end());
            if (reducedCurrentPathRobotFrame.poses.size() < 2) 
            {
                if (cfg_.debug.traj_debug_log) 
                    ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ <<  
                                            ": old path length less than 2, " << incomingPathStatus);

                return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, ableToSwitchToIncomingPath);
            }


            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Compare the scores of the incoming trajectory with the score of the current trajectory to see if we need to switch //
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            int poseCheckCount = std::min(incomingPathOdomFrame.poses.size(), reducedCurrentPathRobotFrame.poses.size()); // cfg_.planning.num_feasi_check, 

            incomingPathScore = std::accumulate(incomingPathPoseScores.begin(), incomingPathPoseScores.begin() + poseCheckCount, float(0));
            if (cfg_.debug.traj_debug_log) 
            {
                ROS_INFO_STREAM("    re-scored incoming trajectory received a subscore of: " << incomingPathScore);
                ROS_INFO_STREAM("    scoring current trajectory");            
            }
            
            std::vector<float> currentPathPoseScores = trajScorer_->scoreTrajectory(reducedCurrentPathRobotFrame, reducedCurrentPathTiming, associatedRawGaps_, futureScans_);
            float currentPathSubscore = std::accumulate(currentPathPoseScores.begin(), currentPathPoseScores.begin() + poseCheckCount, float(0));
            if (cfg_.debug.traj_debug_log) 
                ROS_INFO_STREAM("    current trajectory received a subscore of: " << currentPathSubscore);

            std::vector<std::vector<float>> pathPoseScores(2);
            pathPoseScores.at(0) = incomingPathPoseScores;
            pathPoseScores.at(1) = currentPathPoseScores;
            std::vector<geometry_msgs::PoseArray> paths(2);
            paths.at(0) = incomingPathRobotFrame;
            paths.at(1) = reducedCurrentPathRobotFrame;
            trajVisualizer_->drawGapTrajectoryPoseScores(paths, pathPoseScores);

            
            if (currentPathSubscore == -std::numeric_limits<float>::infinity()) 
            {
                if (cfg_.debug.traj_debug_log) 
                    ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ << 
                                                            ": current trajectory is of score -infinity," << incomingPathStatus);
                return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, ableToSwitchToIncomingPath);

                // if (incomingPathScore == -std::numeric_limits<float>::infinity()) 
                // {
                //     if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ <<  " to empty: both -infinity");

                //     return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, true);
                // } else {
                //     if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << trajectoryChangeCount_ << " to incoming: swapping trajectory due to collision");

                //     return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, false);
                // }
            }

            /*
            if (incomingPathScore > currentPathSubscore) {
                if (cfg_.debug.traj_debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: higher score");
                changeTrajectoryHelper(incomingGap, incoming, time_arr, false);
            }
            */
          
            if (cfg_.debug.traj_debug_log) 
                ROS_INFO_STREAM("        trajectory maintain");
            currentTrajectoryPublisher_.publish(currentPath);
        } catch (...) 
        {
            ROS_FATAL_STREAM("compareToCurrentTraj");
        }


        return currentPath;
    }

    int Planner::egoTrajPosition(const geometry_msgs::PoseArray & curr) 
    {
        std::vector<float> pathPoseNorms(curr.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pathPoseNorms.size(); i++) // i will always be positive, so this is fine
        {
            pathPoseNorms[i] = sqrt(pow(curr.poses.at(i).position.x, 2) + 
                                    pow(curr.poses.at(i).position.y, 2));
        }

        auto minPoseNormIter = std::min_element(pathPoseNorms.begin(), pathPoseNorms.end());
        int minPoseNormIdx = std::distance(pathPoseNorms.begin(), minPoseNormIter) + 1;
        return std::min(minPoseNormIdx, int(curr.poses.size() - 1));
    }

    void Planner::setCurrentRightModel(dynamic_gap::Estimator * rightModel) 
    { 
        currRightGapPtModel_ = rightModel; 
    }

    void Planner::setCurrentLeftModel(dynamic_gap::Estimator * leftModel) 
    { 
        currLeftGapPtModel_ = leftModel; 
    }

    void Planner::setCurrentGapPeakVelocities(float peakVelX, float peakVelY) 
    {
        currentPeakSplineVel_.twist.linear.x = peakVelX;
        currentPeakSplineVel_.twist.linear.y = peakVelY;
    }

    int Planner::getCurrentRightGapPtModelID() 
    {
        // std::cout << "get current left" << std::endl;
        if (currRightGapPtModel_ != NULL) 
        {
            // std::cout << "model is not  null" << std::endl;
            return currRightGapPtModel_->getID();
        } else 
        {
            // std::cout << "model is null" << std::endl;
            return -1;
        }
    }
    
    int Planner::getCurrentLeftGapPtModelID() 
    {
        // std::cout << "get current right" << std::endl;
        if (currLeftGapPtModel_ != NULL) 
        {
            // std::cout << "model is not  null" << std::endl;
            return currLeftGapPtModel_->getID();
        } else 
        {
            // std::cout << "model is null" << std::endl;
            return -1;
        }    
    }

    void Planner::reset()
    {
        simplifiedGaps_.clear();
        setCurrentPath(geometry_msgs::PoseArray());
        currentRbtVel_ = geometry_msgs::TwistStamped();
        currentRbtAcc_ = geometry_msgs::TwistStamped();
        ROS_INFO_STREAM("cmdVelBuffer_ size: " << cmdVelBuffer_.size());
        cmdVelBuffer_.clear();
        ROS_INFO_STREAM("cmdVelBuffer_ size after clear: " << cmdVelBuffer_.size() << ", is full: " << cmdVelBuffer_.capacity());
        return;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(const geometry_msgs::PoseArray & localTrajectory) 
    {
        if (cfg_.debug.control_debug_log) ROS_INFO_STREAM("[ctrlGeneration()]");
        geometry_msgs::Twist rawCmdVel = geometry_msgs::Twist();

        if (cfg_.man.man_ctrl)  // MANUAL CONTROL 
        {
            rawCmdVel = trajController_->manualControlLaw();
        } else if (localTrajectory.poses.size() < 2) // OBSTACLE AVOIDANCE CONTROL 
        { 
            // sensor_msgs::LaserScan scan_msgs;

            // scan_msgs = ;

            ROS_INFO_STREAM("Available Execution Traj length: " << localTrajectory.poses.size() << " < 2");
            rawCmdVel = trajController_->obstacleAvoidanceControlLaw(*scan_.get());
            return rawCmdVel;
        } else // FEEDBACK CONTROL 
        {
            // Know Current Pose
            geometry_msgs::PoseStamped currPoseStRobotFrame;
            currPoseStRobotFrame.header.frame_id = cfg_.robot_frame_id;
            currPoseStRobotFrame.pose.orientation.w = 1;
            geometry_msgs::PoseStamped currPoseStWorldFrame;
            currPoseStWorldFrame.header.frame_id = cfg_.odom_frame_id;
            tf2::doTransform(currPoseStRobotFrame, currPoseStWorldFrame, rbt2odom_);
            geometry_msgs::Pose currPoseWorldFrame = currPoseStWorldFrame.pose;

            // obtain current robot pose in odom frame

            // traj in odom frame here
            // returns a TrajPlan (poses and velocities, velocities are zero here)
            // dynamic_gap::TrajPlan orig_ref = trajController_->trajGen(traj);
            
            // get point along trajectory to target/move towards
            targetTrajectoryPoseIdx_ = trajController_->targetPoseIdx(currPoseWorldFrame, localTrajectory);
            // nav_msgs::Odometry ctrl_target_pose;
            // geometry_msgs::Pose targetTrajectoryPose;
            // ctrl_target_pose.header = traj.header;
            // ctrl_target_pose.pose.pose = traj.poses.at(targetTrajectoryPoseIdx_);
            geometry_msgs::Pose targetTrajectoryPose = localTrajectory.poses.at(targetTrajectoryPoseIdx_);
            // ctrl_target_pose.twist.twist = geometry_msgs::Twist(); // orig_ref.twist.at(targetTrajectoryPoseIdx_);
            
            // geometry_msgs::PoseStamped rbtPoseInSensorFrame_lc = rbtPoseInSensorFrame;
            // sensor_msgs::LaserScan static_scan = *static_scan_ptr.get();
            
            rawCmdVel = trajController_->controlLaw(currPoseWorldFrame, targetTrajectoryPose, 
                                                    staticScan_, currentPeakSplineVel_);
        }
        // geometry_msgs::PoseStamped rbtPoseInSensorFrame_lc = rbtPoseInSensorFrame;

        geometry_msgs::Twist cmdVel = trajController_->processCmdVel(rawCmdVel,
                                                                        staticScan_, rbtPoseInSensorFrame_, 
                                                                        currRightGapPtModel_, currLeftGapPtModel_,
                                                                        currentRbtVel_, currentRbtAcc_); 

        return cmdVel;
    }

    std::vector<dynamic_gap::Gap> Planner::gapSetFeasibilityCheck(bool & isCurrentGapFeasible) // bool & isCurrentGapAssociated, 
                                                                  
    {
        if (cfg_.debug.feasibility_debug_log) ROS_INFO_STREAM("[gapSetFeasibilityCheck()]");

        boost::mutex::scoped_lock gapset(gapsetMutex);
        //std::cout << "PULLING MODELS TO ACT ON" << std::endl;
        std::vector<dynamic_gap::Gap> gaps = associatedSimplifiedGaps_;

        //std::cout << "curr_raw_gaps size: " << curr_raw_gaps.size() << std::endl;
        //std::cout << "gaps size: " << gaps.size() << std::endl;

        //std::cout << "prev_raw_gaps size: " << prev_raw_gaps.size() << std::endl;
        //std::cout << "prev_observed_gaps size: " << prev_observed_gaps.size() << std::endl;
        //std::cout << "current robot velocity. Linear: " << currentRbtVel_.linear.x << ", " << currentRbtVel_.linear.y << ", angular: " << currentRbtVel_.angular.z << std::endl;
        //std::cout << "current raw gaps:" << std::endl;
        //printGapModels(curr_raw_gaps);

        //std::cout << "pulled current simplified associations:" << std::endl;
        //printGapAssociations(gaps, prev_observed_gaps, _simpAssociation_);
        
        /*
        ROS_INFO_STREAM("current raw gaps:");
        printGapModels(curr_raw_gaps);
        */
        
        int currentLeftGapPtModelID = getCurrentLeftGapPtModelID();
        int currentRightGapPtModelID = getCurrentRightGapPtModelID();

        if (cfg_.debug.feasibility_debug_log) 
        {
            ROS_INFO_STREAM("    current simplified gaps:");
            printGapModels(gaps);
            ROS_INFO_STREAM("    current left/right model IDs: " << currentLeftGapPtModelID << ", " << currentRightGapPtModelID);
        }
        
        // isCurrentGapAssociated = false;
        isCurrentGapFeasible = false;

        bool isGapFeasible;
        std::vector<dynamic_gap::Gap> feasibleGaps;
        for (size_t i = 0; i < gaps.size(); i++) 
        {
            // obtain crossing point

            if (cfg_.debug.feasibility_debug_log) ROS_INFO_STREAM("    feasibility check for gap " << i);
            isGapFeasible = gapFeasibilityChecker_->indivGapFeasibilityCheck(gaps[i]);
            isGapFeasible = true;

            if (isGapFeasible) 
            {
                gaps[i].addTerminalRightInformation();
                feasibleGaps.push_back(gaps[i]);
                // ROS_INFO_STREAM("Pushing back gap with peak velocity of : " << gaps[i].peakVelX_ << ", " << gaps[i].peakVelY_);
            }

            if (gaps[i].leftGapPtModel_->getID() == currentLeftGapPtModelID && gaps[i].rightGapPtModel_->getID() == currentRightGapPtModelID) 
            {
                // isCurrentGapAssociated = true;
                isCurrentGapFeasible = true;
            }
        }

        /*
        if (gap_associated) {
            ROS_INFO_STREAM("currently executing gap associated");
        } else {
            ROS_INFO_STREAM("currently executing gap NOT associated");
        }
        */

        return feasibleGaps;
    }

    void Planner::getFutureScans() 
    {
        // boost::mutex::scoped_lock gapset(gapsetMutex);
        if (cfg_.debug.future_scan_propagation_debug_log) ROS_INFO_STREAM("[getFutureScans()]");
        sensor_msgs::LaserScan scan = *scan_.get();
        sensor_msgs::LaserScan dynamicScan = sensor_msgs::LaserScan();
        dynamicScan.header = scan.header;
        dynamicScan.angle_min = scan.angle_min;
        dynamicScan.angle_max = scan.angle_max;
        dynamicScan.angle_increment = scan.angle_increment;
        dynamicScan.time_increment = scan.time_increment;
        dynamicScan.scan_time = scan.scan_time;
        dynamicScan.range_min = scan.range_min;
        dynamicScan.range_max = scan.range_max;
        dynamicScan.ranges = scan.ranges;

        float t_i = 0.0;
        futureScans_[0] = dynamicScan; // at t = 0.0

        std::vector<Eigen::Matrix<float, 4, 1> > currentAgents;
        
        if (cfg_.planning.egocircle_prop_cheat) 
        {
            Eigen::Matrix<float, 4, 1> ithAgentState;
            currentAgents.clear();
            for (int i = 0; i < currentAgentCount_; i++) 
            {
                ithAgentState << currentTrueAgentPoses_[i].position.x, currentTrueAgentPoses_[i].position.y, 
                                 currentTrueAgentVels_[i].vector.x, currentTrueAgentVels_[i].vector.y;
                currentAgents.push_back(ithAgentState);
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
        
        int futureScanTimeIdx;
        for (float t_iplus1 = cfg_.traj.integrate_stept; t_iplus1 <= cfg_.traj.integrate_maxt; t_iplus1 += cfg_.traj.integrate_stept) 
        {
            dynamicScan.ranges = scan.ranges;

            //trajScorer_->recoverDynamicEgocircleCheat(t_i, t_iplus1, agentPoses__lc, agentVels__lc, dynamicScan, print);
            // trajScorer_->recoverDynamicEgoCircle(t_i, t_iplus1, currentAgents, dynamicScan, cfg_.debug.future_scan_propagation_debug_log);
            
            futureScanTimeIdx = (int) (t_iplus1 / cfg_.traj.integrate_stept);
            // ROS_INFO_STREAM("adding scan from " << t_i << " to " << t_iplus1 << " at idx: " << futureScanTimeIdx);
            futureScans_[futureScanTimeIdx] = dynamicScan;

            t_i = t_iplus1;
        }
    }

    geometry_msgs::PoseArray Planner::runPlanningLoop() 
    {
        if (!readyToPlan)
            return geometry_msgs::PoseArray();

        ROS_INFO_STREAM("[runPlanningLoop()]");

        // float getPlan_start_time = ros::WallTime::now().toSec();

        // float start_time = ros::WallTime::now().toSec();      
        // std::chrono::steady_clock::time_point start_time_c;

        bool isCurrentGapFeasible; // isCurrentGapAssociated, 

        std::chrono::steady_clock::time_point planningLoopStartTime = std::chrono::steady_clock::now();

        ///////////////////////////
        // GAP FEASIBILITY CHECK //
        ///////////////////////////
        int gapCount;
        std::vector<dynamic_gap::Gap> feasibleGaps;
        std::chrono::steady_clock::time_point feasibilityStartTime = std::chrono::steady_clock::now();
        if (cfg_.planning.dynamic_feasibility_check)
        {
            try 
            { 
                feasibleGaps = gapSetFeasibilityCheck(isCurrentGapFeasible); // isCurrentGapAssociated, 
            } catch (...) 
            {
                ROS_FATAL_STREAM("out of range in gapSetFeasibilityCheck");
            }
            gapCount = feasibleGaps.size();
        } else
        {
            feasibleGaps = associatedSimplifiedGaps_;
            // need to set feasible to true for all gaps as well
        }


        float feasibilityTimeTaken = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - feasibilityStartTime).count() / 1.0e6;
        ROS_INFO_STREAM("[gapSetFeasibilityCheck() for " << gapCount << " gaps: " << feasibilityTimeTaken << " seconds]");
        
        /////////////////////////////
        // FUTURE SCAN PROPAGATION //
        /////////////////////////////
        std::chrono::steady_clock::time_point scanPropagationStartTime = std::chrono::steady_clock::now();
        try 
        {
            getFutureScans();
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in getFutureScans");
        }

        float scanPropagationTimeTaken = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - scanPropagationStartTime).count() / 1.0e6;
        ROS_INFO_STREAM("[getFutureScans() for " << gapCount << " gaps: " << scanPropagationTimeTaken << " seconds]");
        
        //////////////////////
        // GAP MANIPULATION //
        //////////////////////
        std::chrono::steady_clock::time_point gapManipulateStartTime = std::chrono::steady_clock::now();
        std::vector<dynamic_gap::Gap> manipulatedGaps;
        try 
        {
            manipulatedGaps = gapManipulate(feasibleGaps);
        } catch (std::out_of_range) 
        {
            ROS_INFO_STREAM("out of range in gapManipulate");
        }

        float gapManipulationTimeTaken = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gapManipulateStartTime).count() / 1.0e6;
        ROS_INFO_STREAM("[gapManipulate() for " << gapCount << " gaps: " << gapManipulationTimeTaken << " seconds]");

        ///////////////////////////////////////////
        // GAP TRAJECTORY GENERATION AND SCORING //
        ///////////////////////////////////////////
        std::chrono::steady_clock::time_point generateGapTrajsStartTime = std::chrono::steady_clock::now();
        std::vector<geometry_msgs::PoseArray> paths;
        std::vector<std::vector<float>> pathTimings;
        std::vector<std::vector<float>> pathPoseScores; 
        try 
        {
            pathPoseScores = generateGapTrajs(manipulatedGaps, paths, pathTimings);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in generateGapTrajs");
        }

        float generateGapTrajsTimeTaken = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - generateGapTrajsStartTime).count() / 1.0e6;
        ROS_INFO_STREAM("[generateGapTrajs() for " << gapCount << " gaps: " << generateGapTrajsTimeTaken << " seconds]");

        visualizeComponents(manipulatedGaps); // need to run after generateGapTrajs to see what weights for reachable gap are

        //////////////////////////////
        // GAP TRAJECTORY SELECTION //
        //////////////////////////////
        std::chrono::steady_clock::time_point pickTrajStartTime = std::chrono::steady_clock::now();
        int highestScoreTrajIdx;
        try 
        {
            highestScoreTrajIdx = pickTraj(paths, pathPoseScores);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in pickTraj");
        }

        float pickTrajTimeTaken = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - pickTrajStartTime).count() / 1.0e6;
        ROS_INFO_STREAM("[pickTraj() for " << gapCount << " gaps: " << pickTrajTimeTaken << " seconds]");

        geometry_msgs::PoseArray highestScorePath;
        std::vector<float> highestScorePathTiming;
        dynamic_gap::Gap highestScoreGap;
        if (highestScoreTrajIdx >= 0) 
        {
            highestScorePath = paths[highestScoreTrajIdx];
            highestScorePathTiming = pathTimings[highestScoreTrajIdx];
            highestScoreGap = manipulatedGaps[highestScoreTrajIdx];
        } else 
        {
            highestScorePath = geometry_msgs::PoseArray();
            highestScoreGap = dynamic_gap::Gap();
        }

        ///////////////////////////////
        // GAP TRAJECTORY COMPARISON //
        ///////////////////////////////
        std::chrono::steady_clock::time_point compareToCurrentTrajStartTime = std::chrono::steady_clock::now();

        geometry_msgs::PoseArray chosenTraj;
        try 
        {
            chosenTraj = compareToCurrentTraj(highestScoreGap, highestScorePath, highestScorePathTiming, manipulatedGaps, isCurrentGapFeasible); // isCurrentGapAssociated,
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in compareToCurrentTraj");
        }

        float compareToCurrentTrajTimeTaken = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - compareToCurrentTrajStartTime).count() / 1.0e6;
        ROS_INFO_STREAM("[compareToCurrentTraj() for " << gapCount << " gaps: "  << compareToCurrentTrajTimeTaken << " seconds]");

        float planningLoopTimeTaken = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - planningLoopStartTime).count() / 1.0e6;

        ROS_INFO_STREAM("planning loop for " << gapCount << " gaps: "  << planningLoopTimeTaken << " seconds");
        
        // geometry_msgs::PoseArray chosenTraj;

        return chosenTraj;
    }

    void Planner::visualizeComponents(const std::vector<dynamic_gap::Gap> & manipulatedGaps) 
    {
        boost::mutex::scoped_lock gapset(gapsetMutex);

        gapVisualizer_->drawManipGaps(manipulatedGaps, std::string("manip"));
        // gapVisualizer_->drawReachableGaps(manipulatedGaps);        
        // gapVisualizer_->drawReachableGapsCenters(manipulatedGaps); 

        // gapVisualizer_->drawGapSplines(manipulatedGaps);
        // goalvisualizer->drawGapGoals(manipulatedGaps);
    }

    void Planner::printGapAssociations(const std::vector<dynamic_gap::Gap> & currentGaps, 
                                       const std::vector<dynamic_gap::Gap> & previousGaps, 
                                       const std::vector<int> & association) 
    {
        std::cout << "current simplified associations" << std::endl;
        std::cout << "number of gaps: " << currentGaps.size() << ", number of previous gaps: " << previousGaps.size() << std::endl;
        std::cout << "association size: " << association.size() << std::endl;

        for (int i = 0; i < association.size(); i++)
            std::cout << association[i] << ", ";
        std::cout << "" << std::endl;

        float currX, currY, prevX, prevY;
        for (int i = 0; i < association.size(); i++) 
        {
            std::vector<int> pair{i, association[i]};
            std::cout << "pair (" << i << ", " << association[i] << "). ";
            int currentGapIdx = int(std::floor(pair[0] / 2.0));
            int previousGapIdx = int(std::floor(pair[1] / 2.0));

            if (pair[0] % 2 == 0)  // curr left
                currentGaps.at(currentGapIdx).getSimplifiedRCartesian(currX, currY);
            else // curr right
                currentGaps.at(currentGapIdx).getSimplifiedLCartesian(currX, currY);
            
            if (association[i] >= 0) 
            {
                if (pair[1] % 2 == 0) // prev left
                    previousGaps.at(previousGapIdx).getSimplifiedRCartesian(prevX, prevY);
                else // prev right
                    previousGaps.at(previousGapIdx).getSimplifiedLCartesian(prevX, prevY);
                
                std::cout << "From (" << prevX << ", " << prevY << ") to (" << currX << ", " << currY << ") with a distance of " << simpDistMatrix_[pair[0]][pair[1]] << std::endl;
            } else 
            {
                std::cout << "From NULL to (" << currX << ", " <<  currY << ")" << std::endl;
            }
        }
    }

    void Planner::printGapModels(const std::vector<dynamic_gap::Gap> & gaps) 
    {
        // THIS IS NOT FOR MANIPULATED GAPS
        float x, y;
        for (size_t i = 0; i < gaps.size(); i++)
        {
            dynamic_gap::Gap gap = gaps[i];
            ROS_INFO_STREAM("    gap " << i << ", indices: " << gap.RIdx() << " to "  << gap.LIdx() << ", left model: " << gap.leftGapPtModel_->getID() << ", rightGapPtModel: " << gap.rightGapPtModel_->getID());
            Eigen::Matrix<float, 4, 1> left_state = gap.leftGapPtModel_->getState();
            gap.getLCartesian(x, y);            
            ROS_INFO_STREAM("        left point: (" << x << ", " << y << "), left model: (" << left_state[0] << ", " << left_state[1] << ", " << left_state[2] << ", " << left_state[3] << ")");
            Eigen::Matrix<float, 4, 1> right_state = gap.rightGapPtModel_->getState();
            gap.getRCartesian(x, y);
            ROS_INFO_STREAM("        right point: (" << x << ", " << y << "), right model: (" << right_state[0] << ", " << right_state[1] << ", " << right_state[2] << ", " << right_state[3] << ")");
        }
    }

    bool Planner::recordAndCheckVel(const geometry_msgs::Twist & cmdVel) 
    {
        float val = std::abs(cmdVel.linear.x) + std::abs(cmdVel.linear.y) + std::abs(cmdVel.angular.z);

        cmdVelBuffer_.push_back(val);
        float cmdVelBufferSum = std::accumulate(cmdVelBuffer_.begin(), cmdVelBuffer_.end(), float(0));
        bool keepPlanning = cmdVelBufferSum > 1.0 || !cmdVelBuffer_.full();
        
        if (!keepPlanning && !cfg_.man.man_ctrl) 
        {
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            ROS_INFO_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        return keepPlanning || cfg_.man.man_ctrl;
    }

}