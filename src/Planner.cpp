#include <dynamic_gap/Planner.h>

namespace dynamic_gap
{   
    Planner::Planner()
    {
        // Do something? maybe set names
        ros::NodeHandle nh("planner_node");
    }

    Planner::~Planner()
    {
        // delete current raw, simplified, and selected gaps
        for (dynamic_gap::Gap * rawGap : currRawGaps_)
            delete rawGap;
        currRawGaps_.clear();

        for (dynamic_gap::Gap * simplifiedGap : currSimplifiedGaps_)
            delete simplifiedGap;
        currSimplifiedGaps_.clear();

        delete currentGap_;

        // delete objects
        delete tfListener_;
        
        delete gapDetector_;
        delete gapAssociator_;
        delete gapVisualizer_;

        delete goalSelector_;
        delete goalVisualizer_;

        delete gapFeasibilityChecker_;

        delete gapManipulator_;

        delete gapTrajGenerator_;

        delete trajScorer_;
        delete trajController_;
        delete trajVisualizer_;
    }

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
        gapAssociator_ = new dynamic_gap::GapAssociator(nh, cfg_);
        // staticScanSeparator_ = new dynamic_gap::StaticScanSeparator(cfg_);
        gapVisualizer_ = new dynamic_gap::GapVisualizer(nh, cfg_);

        goalSelector_ = new dynamic_gap::GoalSelector(nh, cfg_);
        goalVisualizer_ = new dynamic_gap::GoalVisualizer(nh, cfg_);

        gapFeasibilityChecker_ = new dynamic_gap::GapFeasibilityChecker(nh, cfg_);

        gapManipulator_ = new dynamic_gap::GapManipulator(nh, cfg_);

        gapTrajGenerator_ = new dynamic_gap::GapTrajectoryGenerator(nh, cfg_);

        trajScorer_ = new dynamic_gap::TrajectoryScorer(nh, cfg_);
        trajController_ = new dynamic_gap::TrajectoryController(nh, cfg_);
        trajVisualizer_ = new dynamic_gap::TrajectoryVisualizer(nh, cfg_);


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
            ROS_INFO_STREAM_NAMED("Planner", "[Reset] Goal Reached");
            return true;
        }

        float globalPathLocalWaypointDiffX = globalPathLocalWaypointOdomFrame_.pose.position.x - robotPoseOdomFrame_.pose.position.x;
        float globalPathLocalWaypointDiffY = globalPathLocalWaypointOdomFrame_.pose.position.y - robotPoseOdomFrame_.pose.position.y;
        bool reachedGlobalPathLocalWaypoint = sqrt(pow(globalPathLocalWaypointDiffX, 2) + pow(globalPathLocalWaypointDiffY, 2)) < cfg_.goal.waypoint_tolerance;
        if (reachedGlobalPathLocalWaypoint)
            ROS_INFO_STREAM_NAMED("Planner", "[Reset] Waypoint reached, getting new one");

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
            currRawGaps_ = gapDetector_->gapDetection(scan_, globalGoalRobotFrame_);
            // float gap_detection_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_detection_start_time).count() / 1.0e6;
            // ROS_INFO_STREAM("gapDetection: " << gap_detection_time << " seconds");

            ROS_INFO_STREAM_NAMED("GapAssociator", "RAW GAP ASSOCIATING");    
            rawDistMatrix_ = gapAssociator_->obtainDistMatrix(currRawGaps_, prevRawGaps_);
            rawAssocation_ = gapAssociator_->associateGaps(rawDistMatrix_);
            gapAssociator_->assignModels(rawAssocation_, rawDistMatrix_, 
                                        currRawGaps_, prevRawGaps_, 
                                        currentModelIdx_, tCurrentFilterUpdate,
                                        intermediateRbtVels, intermediateRbtAccs);
            updateModels(currRawGaps_, intermediateRbtVels, 
                         intermediateRbtAccs, tCurrentFilterUpdate);

            // staticScan_ = staticScanSeparator_->staticDynamicScanSeparation(currRawGaps_, scan_);
            // staticScanPublisher_.publish(staticScan_);
            // trajScorer_->updateStaticEgoCircle(staticScan_);
            // gapManipulator_->updateStaticEgoCircle(staticScan_);
            
            // currentEstimatedAgentStates_ = staticScanSeparator_->getCurrAgents();

            // std::chrono::steady_clock::time_point gap_simplification_start_time = std::chrono::steady_clock::now();
            
            currSimplifiedGaps_ = gapDetector_->gapSimplification(currRawGaps_);
            // float gap_simplification_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_simplification_start_time).count() / 1.0e6;
            // ROS_INFO_STREAM("gapSimplification: " << gap_simplification_time << " seconds");
            
            ROS_INFO_STREAM_NAMED("GapAssociator", "SIMPLIFIED GAP ASSOCIATING");    
            simpDistMatrix_ = gapAssociator_->obtainDistMatrix(currSimplifiedGaps_, prevSimplifiedGaps_);
            simpAssociation_ = gapAssociator_->associateGaps(simpDistMatrix_); // must finish this and therefore change the association
            gapAssociator_->assignModels(simpAssociation_, simpDistMatrix_, 
                                        currSimplifiedGaps_, prevSimplifiedGaps_, 
                                        currentModelIdx_, tCurrentFilterUpdate,
                                        intermediateRbtVels, intermediateRbtAccs);
            updateModels(currSimplifiedGaps_, intermediateRbtVels, 
                         intermediateRbtAccs, tCurrentFilterUpdate);

            // gapVisualizer_->drawGaps(currRawGaps_, std::string("raw"));
            // gapVisualizer_->drawGapsModels(currRawGaps_);
            gapVisualizer_->drawGaps(currSimplifiedGaps_, std::string("simp"));
            gapVisualizer_->drawGapsModels(currSimplifiedGaps_);

            readyToPlan = true;
        }
        
        // update current scan for proper classes
        updateEgoCircle();

        // update global path local waypoint according to new scan
        goalSelector_->generateGlobalPathLocalWaypoint(map2rbt_);
        geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame = goalSelector_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);
        goalVisualizer_->drawGlobalPathLocalWaypoint(globalPathLocalWaypointOdomFrame);
        trajScorer_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame, odom2rbt_);
        
        // delete previous gaps
        for (dynamic_gap::Gap * prevRawGap : prevRawGaps_)
            delete prevRawGap;
        prevRawGaps_.clear();
        
        for (dynamic_gap::Gap * prevSimplifiedGap : prevSimplifiedGaps_)
            delete prevSimplifiedGap;
        prevSimplifiedGaps_.clear();

        // update previous gaps
        prevRawGaps_ = currRawGaps_;
        prevSimplifiedGaps_ = currSimplifiedGaps_;

        // update estimator update time
        tPreviousFilterUpdate_ = tCurrentFilterUpdate;
    }

    
    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    void Planner::updateModels(std::vector<dynamic_gap::Gap *> & gaps, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                                const ros::Time & tCurrentFilterUpdate) 
    {
        ROS_INFO_STREAM_NAMED("GapEstimation", "[updateModels()]");
        
        try
        {
            for (int i = 0; i < 2*gaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("GapEstimation", "    update gap model " << i << " of " << 2*gaps.size());
                updateModel(i, gaps, intermediateRbtVels, intermediateRbtAccs, tCurrentFilterUpdate);
                ROS_INFO_STREAM_NAMED("GapEstimation", "");
            }
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("GapEstimation", "updateModels failed");
        }

        return;
    }

    void Planner::updateModel(const int & idx, std::vector<dynamic_gap::Gap *> & gaps, 
                               const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                               const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                               const ros::Time & tCurrentFilterUpdate) 
    {
		try
        {
            dynamic_gap::Gap * gap = gaps[int(idx / 2.0)];
    
            float rX = 0.0, rY = 0.0;
            if (idx % 2 == 0) 
               gap->getLCartesian(rX, rY);            
            else 
               gap->getRCartesian(rX, rY);            

            // ROS_INFO_STREAM("rX: " << rX << ", rY: " << rY);

            Eigen::Vector2f measurement(rX, rY);

            if (idx % 2 == 0) 
            {
                if (gap->leftGapPtModel_)
                {
                   gap->leftGapPtModel_->update(measurement, 
                                                intermediateRbtVels, intermediateRbtAccs, 
                                                currentTrueAgentPoses_, 
                                                currentTrueAgentVels_,
                                                tCurrentFilterUpdate);                    
                } else
                {
                    ROS_WARN_STREAM_NAMED("GapEstimation", "left model is null");
                }
            } else 
            {
                if (gap->rightGapPtModel_)
                {
                   gap->rightGapPtModel_->update(measurement, 
                                                intermediateRbtVels, intermediateRbtAccs, 
                                                currentTrueAgentPoses_, 
                                                currentTrueAgentVels_,
                                                tCurrentFilterUpdate);                    
                } else
                {                
                    ROS_WARN_STREAM_NAMED("GapEstimation", "right model is null");
                }
            } 
        } catch (...) 
        {
            ROS_WARN_STREAM_NAMED("GapEstimation", "updateModel failed");
        }        
    }
    
    void Planner::updateEgoCircle()
    {
        goalSelector_->updateEgoCircle(scan_);
        gapManipulator_->updateEgoCircle(scan_);
        trajScorer_->updateEgoCircle(scan_);
        trajController_->updateEgoCircle(scan_);
    }

    void Planner::jointPoseAccCB(const nav_msgs::Odometry::ConstPtr & rbtOdomMsg, 
                                 const geometry_msgs::TwistStamped::ConstPtr & rbtAccelMsg)
    {
        try
        {
            // ROS_INFO_STREAM("joint pose acc cb");

            // ROS_INFO_STREAM("accel time stamp: " << rbtAccelMsg->header.stamp.toSec());
            currentRbtAcc_ = *rbtAccelMsg;
            intermediateRbtAccs_.push_back(currentRbtAcc_);

            // deleting old sensor measurements already used in an update
            for (int i = 0; i < intermediateRbtAccs_.size(); i++)
            {
                if (intermediateRbtAccs_.at(i).header.stamp <= tPreviousFilterUpdate_)
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
                if (intermediateRbtVels_.at(i).header.stamp <= tPreviousFilterUpdate_)
                {
                    intermediateRbtVels_.erase(intermediateRbtVels_.begin() + i);
                    i--;
                }
            }
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("Planner", "jointPoseAccCB failed");
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
            
            // ROS_INFO_STREAM("updating " << agentNamespace << " odom from " << agent_odom_vects.at(agentID)[0] << ", " << agent_odom_vects.at(agentID)[1] << " to " << odom_vect[0] << ", " << odom_vect[1]);
            currentTrueAgentPoses_.at(agentID) = agentPoseRobotFrame.pose;
        } catch (...) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "agentOdomCB odometry failed for " << agentNamespace);
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

            currentTrueAgentVels_.at(agentID) = agentVelRobotFrame;
        } catch (...) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "agentOdomCB velocity failed for " << agentNamespace);
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
        } catch (...) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "updateTF failed");
            ros::Duration(0.1).sleep();
            return;
        }
    }

    std::vector<dynamic_gap::Gap *> Planner::gapManipulate(const std::vector<dynamic_gap::Gap *> & feasibleGaps) 
    {

        ROS_INFO_STREAM_NAMED("GapManipulator", "[gapManipulate()]");

        boost::mutex::scoped_lock gapset(gapsetMutex);
        std::vector<dynamic_gap::Gap *> manipulatedGaps = feasibleGaps; // shallow copy

        try
        {
            for (size_t i = 0; i < manipulatedGaps.size(); i++)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "    manipulating initial gap " << i);

                // manipulatedGaps.at(i)->initManipIndices();

                // MANIPULATE POINTS AT T=0            
                // gapManipulator_->reduceGap(manipulatedGaps.at(i), goalSelector_->getGlobalPathLocalWaypointRobotFrame(), true);
                gapManipulator_->convertRadialGap(manipulatedGaps.at(i), true);
                gapManipulator_->inflateGapSides(manipulatedGaps.at(i), true);
                gapManipulator_->radialExtendGap(manipulatedGaps.at(i), true);
                gapManipulator_->setGapGoal(manipulatedGaps.at(i), goalSelector_->getGlobalPathLocalWaypointRobotFrame(), true);
                
                // MANIPULATE POINTS AT T=1
                ROS_INFO_STREAM_NAMED("GapManipulator", "    manipulating terminal gap " << i);
                // gapManipulator_->updateDynamicEgoCircle(manipulatedGaps.at(i), futureScans_);
                
                if ((!manipulatedGaps.at(i)->crossed_ && !manipulatedGaps.at(i)->closed_) || (manipulatedGaps.at(i)->crossedBehind_)) 
                {
                    // gapManipulator_->reduceGap(manipulatedGaps.at(i), goalSelector_->getGlobalPathLocalWaypointRobotFrame(), false);
                    gapManipulator_->convertRadialGap(manipulatedGaps.at(i), false);
                }
                gapManipulator_->inflateGapSides(manipulatedGaps.at(i), false);
                gapManipulator_->radialExtendGap(manipulatedGaps.at(i), false);
                gapManipulator_->setGapTerminalGoal(manipulatedGaps.at(i), goalSelector_->getGlobalPathLocalWaypointRobotFrame());
            }
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("GapManipulator", "   gapManipulate failed");
        }

        return manipulatedGaps;
    }

    std::vector<std::vector<float>> Planner::generateGapTrajs(std::vector<dynamic_gap::Gap *>& gaps, 
                                                              std::vector<dynamic_gap::Trajectory> & generatedTrajs) 
    {
        boost::mutex::scoped_lock gapset(gapsetMutex);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateGapTrajs()]");
        std::vector<geometry_msgs::PoseArray> paths(gaps.size());
        std::vector<std::vector<float>> pathTimings(gaps.size());
        std::vector<std::vector<float>> pathPoseScores(gaps.size());
        // geometry_msgs::PoseStamped rbtPoseInSensorFrame_lc = rbtPoseInSensorFrame; // lc as local copy

        // std::vector<dynamic_gap::Gap *> rawGaps = currRawGaps_;
        try 
        {
            for (size_t i = 0; i < gaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    generating traj for gap: " << i);
                // std::cout << "starting generate trajectory with rbtPoseInSensorFrame_lc: " << rbtPoseInSensorFrame_lc.pose.position.x << ", " << rbtPoseInSensorFrame_lc.pose.position.y << std::endl;
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                dynamic_gap::Trajectory traj;
                
                // TRAJECTORY GENERATED IN RBT FRAME
                bool runGoToGoal = true; // (vec.at(i).goal.goalwithin || vec.at(i).artificial);
                if (runGoToGoal) 
                {
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        running goToGoal");

                    dynamic_gap::Trajectory goToGoalTraj;
                    goToGoalTraj = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbtPoseInSensorFrame_, currentRbtVel_, runGoToGoal);
                    goToGoalTraj = gapTrajGenerator_->processTrajectory(goToGoalTraj);
                    std::vector<float> goToGoalPoseScores = trajScorer_->scoreTrajectory(goToGoalTraj, futureScans_);
                    float goToGoalScore = std::accumulate(goToGoalPoseScores.begin(), goToGoalPoseScores.end(), float(0));
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        goToGoalScore: " << goToGoalScore);

                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        running ahpf");
                    dynamic_gap::Trajectory ahpfTraj;
                    ahpfTraj = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbtPoseInSensorFrame_, currentRbtVel_, !runGoToGoal);
                    ahpfTraj = gapTrajGenerator_->processTrajectory(ahpfTraj);
                    std::vector<float> ahpfPoseScores = trajScorer_->scoreTrajectory(ahpfTraj, futureScans_);
                    float ahpfScore = std::accumulate(ahpfPoseScores.begin(), ahpfPoseScores.end(), float(0));
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        ahpfScore: " << ahpfScore);

                    if (goToGoalScore > ahpfScore)
                    {
                        traj = goToGoalTraj;
                        pathPoseScores.at(i) = goToGoalPoseScores;
                    } else
                    {
                        traj = ahpfTraj;
                        pathPoseScores.at(i) = ahpfPoseScores;
                    }
                    // traj =  ? goToGoalTraj : ahpfTraj;
                    // pathPoseScores.at(i) = (goToGoalScore > ahpfScore) ? goToGoalPoseScores : ahpfPoseScores;
                } else 
                {
                    traj = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbtPoseInSensorFrame_, currentRbtVel_, runGoToGoal);
                    traj = gapTrajGenerator_->processTrajectory(traj);

                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    scoring trajectory for gap: " << i);
                    pathPoseScores.at(i) = trajScorer_->scoreTrajectory(traj, futureScans_);  
                    // ROS_INFO_STREAM("done with scoreTrajectory");
                }

                // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                traj.setPathOdomFrame(gapTrajGenerator_->transformPath(traj.getPathRbtFrame(), cam2odom_));
                // pathTimings.at(i) = std::get<1>(traj);
                generatedTrajs.push_back(traj);
            }
        } catch (...) 
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "generateGapTrajs failed");
        }

        trajVisualizer_->drawGapTrajectoryPoseScores(generatedTrajs, pathPoseScores);
        trajVisualizer_->drawGapTrajectories(generatedTrajs);
        // generatedPaths = paths;
        // generatedPathTimings = pathTimings;
        return pathPoseScores;
    }

    int Planner::pickTraj(const std::vector<dynamic_gap::Trajectory> & trajs, 
                          const std::vector<std::vector<float>> & pathPoseScores) 
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[pickTraj()]");
        
        int highestPathScoreIdx = -1;        
        
        try
        {
            // ROS_INFO_STREAM_NAMED("pg_trajCount", "pg_trajCount, " << paths.size());
            if (trajs.size() == 0) 
            {
                ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "No traj synthesized");
                return -1;
            }

            if (trajs.size() != pathPoseScores.size()) 
            {
                ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "pickTraj size mismatch: paths = " << trajs.size() << " != pathPoseScores =" << pathPoseScores.size());
                return -1;
            }

            // poses here are in odom frame 
            std::vector<float> pathScores(trajs.size());
            // int counts;
            // try 
            // {
                // if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < pathScores.size(); i++) 
            {
                // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());
                // counts = std::min(cfg_.planning.num_feasi_check, int(pathPoseScores.at(i).size()));

                pathScores.at(i) = std::accumulate(pathPoseScores.at(i).begin(), pathPoseScores.at(i).end(), float(0));
                pathScores.at(i) = trajs.at(i).getPathRbtFrame().poses.size() == 0 ? -std::numeric_limits<float>::infinity() : pathScores.at(i);
                
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    for gap " << i << " (length: " << trajs.at(i).getPathRbtFrame().poses.size() << "), returning score of " << pathScores.at(i));
                
                // if (pathScores.at(i) == -std::numeric_limits<float>::infinity()) {
                //     for (size_t j = 0; j < counts; j++) {
                //         if (score.at(i).at(j) == -std::numeric_limits<float>::infinity()) {
                //             std::cout << "-inf score at idx " << j << " of " << counts << std::endl;
                //         }
                //     }
                // }
                
            }
            // } catch (...) 
            // {
            //     ROS_FATAL_STREAM("pickTraj");
            // }

            auto highestPathScoreIter = std::max_element(pathScores.begin(), pathScores.end());
            highestPathScoreIdx = std::distance(pathScores.begin(), highestPathScoreIter);

            if (pathScores.at(highestPathScoreIdx) == -std::numeric_limits<float>::infinity()) 
            {    
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    all -infinity");
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "No executable trajectory, values: ");
                for (float pathScore : pathScores) 
                {
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Score: " << pathScore);
                }
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "------------------");
            }

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    picking gap: " << highestPathScoreIdx);
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "   pickTraj failed");
        }
        return highestPathScoreIdx;
    }

    dynamic_gap::Trajectory Planner::changeTrajectoryHelper(dynamic_gap::Gap * incomingGap, 
                                                             const dynamic_gap::Trajectory & incomingTraj, 
                                                             const bool & switchToIncoming) 
    {
        trajectoryChangeCount_++;

        if (switchToIncoming) 
        {
            setCurrentGap(incomingGap);
            setCurrentTraj(incomingTraj);
            // setCurrentPath(incomingPath);
            // setCurrentPathTiming(incomingPathTiming);
            setCurrentLeftModel(incomingGap->leftGapPtModel_);
            setCurrentRightModel(incomingGap->rightGapPtModel_);
            setCurrentGapPeakVelocities(incomingGap->peakSplineVelX_, incomingGap->peakSplineVelY_);
            currentTrajectoryPublisher_.publish(incomingTraj.getPathRbtFrame());          
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, incomingTraj);

            return incomingTraj;  
        } else 
        {
            dynamic_gap::Trajectory emptyTraj;
            geometry_msgs::PoseArray emptyPath = geometry_msgs::PoseArray();
            emptyPath.header = incomingTraj.getPathRbtFrame().header;
            std::vector<float> emptyPathTiming;
            setCurrentGap(NULL);
            setCurrentTraj(emptyTraj);
            // setCurrentPath(emptyPath);
            // setCurrentPathTiming(emptyPathTiming);
            setCurrentLeftModel(NULL);
            setCurrentRightModel(NULL);
            setCurrentGapPeakVelocities(0.0, 0.0);
            currentTrajectoryPublisher_.publish(emptyTraj.getPathRbtFrame());
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, emptyTraj);
            return emptyTraj;
        }                       
    }

    dynamic_gap::Trajectory Planner::compareToCurrentTraj(dynamic_gap::Gap * incomingGap, 
                                                            const dynamic_gap::Trajectory & incomingTraj,                                                        
                                                            const std::vector<dynamic_gap::Gap *> & feasibleGaps, 
                                                            const bool & isIncomingGapFeasibleInput) // bool isIncomingGapAssociated,
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[compareToCurrentTraj()]");
        boost::mutex::scoped_lock gapset(gapsetMutex);
        
        dynamic_gap::Trajectory currentTraj = getCurrentTraj();
        // geometry_msgs::PoseArray currentPath = currentTraj.getPathRbtFrame(); // getCurrentPath();
        // std::vector<float> currentPathTiming = currentTraj.getPathTiming(); // getCurrentPathTiming();

        // std::vector<dynamic_gap::Gap *> curr_raw_gaps = currRawGaps_;

        try 
        {            
            // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapPtModelID() << ", current right gap index: " << getCurrentRightGapPtModelID());

            // First, checking if the current gap we are within is still valid (associated and feasible)
            // TODO: remove associated because feasible assumes associated
            bool isIncomingGapFeasible = true; // (isIncomingGapFeasibleInput);
            for (dynamic_gap::Gap * gap : feasibleGaps) 
            {
                // ROS_INFO_STREAM("feasible left gap index: " << g.leftGapPtModel_->getID() << ", feasible right gap index: " << g.rightGapPtModel_->getID());
                if (gap->leftGapPtModel_->getID() == getCurrentLeftGapPtModelID() && gap->rightGapPtModel_->getID() == getCurrentRightGapPtModelID()) 
                {
                    setCurrentGapPeakVelocities(gap->peakSplineVelX_, gap->peakSplineVelY_);
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
            geometry_msgs::PoseArray incomingPathRobotFrame = gapTrajGenerator_->transformPath(incomingTraj.getPathOdomFrame(), odom2rbt_);
            // incomingPathRobotFrame.header.frame_id = cfg_.robot_frame_id;

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    scoring incoming trajectory");
            std::vector<float> incomingPathPoseScores = trajScorer_->scoreTrajectory(incomingTraj,
                                                                                     futureScans_);
            // int counts = std::min(cfg_.planning.num_feasi_check, (int) std::min(incomingPathPoseScores.size(), curr_score.size()));

            // int counts = std::min(cfg_.planning.num_feasi_check, (int) incomingPathPoseScores.size());
            float incomingPathScore = std::accumulate(incomingPathPoseScores.begin(), incomingPathPoseScores.end(), float(0));
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    incoming trajectory received a score of: " << incomingPathScore);
          

            ///////////////////////////////////////////////////////////////////////
            //  Evaluate the incoming path to determine if we can switch onto it //
            ///////////////////////////////////////////////////////////////////////
            std::string incomingPathStatus = "incoming path is safe to switch onto"; // (incomingPathOdomFrame.poses.size() > 0) ? "incoming traj length 0" : "incoming score infinite";

            bool ableToSwitchToIncomingPath = true;
            if (incomingTraj.getPathRbtFrame().poses.size() == 0)
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
            bool isCurrentPathEmpty = currentTraj.getPathRbtFrame().poses.size() == 0;
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
                
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  
                                            ": current path is of length zero, " << incomingPathStatus);                
                return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);

                
                // if (ableToSwitchToIncomingPath) 
                // {
                //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ << 
                //                                 " to incoming: " << currentPathStatus << ", incoming score finite");

                //     return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, false);
                // } else  
                // {
                //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  
                //                                 " to empty: " << currentPathStatus << ", " << incomingPathStatus);
                    
                //     return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, true);
                // }
                
            } 

            ////////////////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing path has been completely tracked //
            ////////////////////////////////////////////////////////////////////////////////////////////   
            geometry_msgs::PoseArray currentPathRobotFrame = gapTrajGenerator_->transformPath(currentTraj.getPathOdomFrame(), odom2rbt_);
            // currentPathRobotFrame.header.frame_id = cfg_.robot_frame_id;
            int currentPathPoseIdx = egoTrajPosition(currentPathRobotFrame);
            geometry_msgs::PoseArray reducedCurrentPathRobotFrame = currentPathRobotFrame;
            reducedCurrentPathRobotFrame.poses = std::vector<geometry_msgs::Pose>(currentPathRobotFrame.poses.begin() + currentPathPoseIdx, currentPathRobotFrame.poses.end());

            std::vector<float> currentPathTiming = currentTraj.getPathTiming();
            std::vector<float> reducedCurrentPathTiming = currentPathTiming;
            reducedCurrentPathTiming = std::vector<float>(currentPathTiming.begin() + currentPathPoseIdx, currentPathTiming.end());
            if (reducedCurrentPathRobotFrame.poses.size() < 2) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  
                                            ": old path length less than 2, " << incomingPathStatus);

                return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
            }


            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Compare the scores of the incoming trajectory with the score of the current trajectory to see if we need to switch //
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            int poseCheckCount = std::min(incomingTraj.getPathRbtFrame().poses.size(), reducedCurrentPathRobotFrame.poses.size()); // cfg_.planning.num_feasi_check, 

            incomingPathScore = std::accumulate(incomingPathPoseScores.begin(), incomingPathPoseScores.begin() + poseCheckCount, float(0));

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    re-scored incoming trajectory received a subscore of: " << incomingPathScore);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    scoring current trajectory");            
            
            dynamic_gap::Trajectory reducedCurrentTraj(reducedCurrentPathRobotFrame, reducedCurrentPathTiming);
            std::vector<float> currentPathPoseScores = trajScorer_->scoreTrajectory(reducedCurrentTraj, futureScans_);
            float currentPathSubscore = std::accumulate(currentPathPoseScores.begin(), currentPathPoseScores.begin() + poseCheckCount, float(0));
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    current trajectory received a subscore of: " << currentPathSubscore);

            std::vector<std::vector<float>> pathPoseScores(2);
            pathPoseScores.at(0) = incomingPathPoseScores;
            pathPoseScores.at(1) = currentPathPoseScores;
            // std::vector<dynamic_gap::Trajectory> trajs(2);
            // trajs.at(0) = incomingTraj;
            // trajs.at(1) = reducedCurrentTraj;
            // trajVisualizer_->drawGapTrajectoryPoseScores(trajs, pathPoseScores);

            
            if (currentPathSubscore == -std::numeric_limits<float>::infinity()) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ << 
                                                            ": current trajectory is of score -infinity," << incomingPathStatus);
                return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);

                // if (incomingPathScore == -std::numeric_limits<float>::infinity()) 
                // {
                //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  " to empty: both -infinity");

                //     return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, true);
                // } else {
                //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ << " to incoming: swapping trajectory due to collision");

                //     return changeTrajectoryHelper(incomingGap, incomingPathOdomFrame, incomingPathTiming, false);
                // }
            }

            
            // if (incomingPathScore > currentPathSubscore) {
            //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "TRAJECTORY CHANGE TO INCOMING: higher score");
            //     changeTrajectoryHelper(incomingGap, incoming, time_arr, false);
            // }
            
          
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory maintain");
            currentTrajectoryPublisher_.publish(currentTraj.getPathRbtFrame());
        } catch (...) 
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "compareToCurrentTraj");
        }

        return currentTraj;
    }


    int Planner::egoTrajPosition(const geometry_msgs::PoseArray & curr) 
    {
        std::vector<float> pathPoseNorms(curr.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pathPoseNorms.size(); i++) // i will always be positive, so this is fine
        {
            pathPoseNorms.at(i) = sqrt(pow(curr.poses.at(i).position.x, 2) + 
                                    pow(curr.poses.at(i).position.y, 2));
        }

        auto minPoseNormIter = std::min_element(pathPoseNorms.begin(), pathPoseNorms.end());
        int minPoseNormIdx = std::distance(pathPoseNorms.begin(), minPoseNormIter) + 1;
        return std::min(minPoseNormIdx, int(curr.poses.size() - 1));
    }

    void Planner::setCurrentLeftModel(dynamic_gap::Estimator * leftModel) 
    { 
        currLeftGapPtModel_ = leftModel; 
    }

    void Planner::setCurrentRightModel(dynamic_gap::Estimator * rightModel) 
    { 
        currRightGapPtModel_ = rightModel; 
    }

    void Planner::setCurrentGapPeakVelocities(const float & peakVelX, const float & peakVelY)
    {
        currentPeakSplineVel_.twist.linear.x = peakVelX;
        currentPeakSplineVel_.twist.linear.y = peakVelY;
    }

    int Planner::getCurrentLeftGapPtModelID() 
    {
        // ROS_INFO_STREAM_NAMED("Planner", "[getCurrentLeftGapPtModelID()]");
        if (currLeftGapPtModel_) 
        {
            // ROS_INFO_STREAM_NAMED("Planner", "      model not null, has ID: " << currLeftGapPtModel_->getID());
            return currLeftGapPtModel_->getID();
        } else 
        {
            // ROS_INFO_STREAM_NAMED("Planner", "      model is null");
            return -1;
        }    
    }

    int Planner::getCurrentRightGapPtModelID() 
    {
        if (currRightGapPtModel_) 
        {
            return currRightGapPtModel_->getID();
        } else 
        {
            return -1;
        }
    }


    void Planner::reset()
    {
        // currSimplifiedGaps_.clear();
        setCurrentTraj(dynamic_gap::Trajectory());
        currentRbtVel_ = geometry_msgs::TwistStamped();
        currentRbtAcc_ = geometry_msgs::TwistStamped();
        ROS_INFO_STREAM_NAMED("Planner", "velocity buffer size: " << cmdVelBuffer_.size());
        cmdVelBuffer_.clear();
        ROS_INFO_STREAM_NAMED("Planner", "velocity buffer size after clear: " << cmdVelBuffer_.size() << ", is full: " << cmdVelBuffer_.capacity());
        return;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(const geometry_msgs::PoseArray & localTrajectory) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "[ctrlGeneration()]");
        geometry_msgs::Twist rawCmdVel = geometry_msgs::Twist();
        geometry_msgs::Twist cmdVel = rawCmdVel;

        try
        {

            if (cfg_.man.man_ctrl)  // MANUAL CONTROL 
            {
                ROS_INFO_STREAM_NAMED("Controller", "Manual control chosen.");
                rawCmdVel = trajController_->manualControlLaw();
            } else if (localTrajectory.poses.size() < 2) // OBSTACLE AVOIDANCE CONTROL 
            { 
                ROS_INFO_STREAM_NAMED("Planner", "Available Execution Traj length: " << localTrajectory.poses.size() << " < 2, obstacle avoidance control chosen.");
                rawCmdVel = trajController_->obstacleAvoidanceControlLaw();
                return rawCmdVel;
            } else // FEEDBACK CONTROL 
            {
                ROS_INFO_STREAM_NAMED("Controller", "Trajectory tracking control chosen.");

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
                targetTrajectoryPoseIdx_ = trajController_->extractTargetPoseIdx(currPoseWorldFrame, localTrajectory);
                // nav_msgs::Odometry ctrl_target_pose;
                // geometry_msgs::Pose targetTrajectoryPose;
                // ctrl_target_pose.header = traj.header;
                // ctrl_target_pose.pose.pose = traj.poses.at(targetTrajectoryPoseIdx_);
                geometry_msgs::Pose targetTrajectoryPose = localTrajectory.poses.at(targetTrajectoryPoseIdx_);
                // ctrl_target_pose.twist.twist = geometry_msgs::Twist(); // orig_ref.twist.at(targetTrajectoryPoseIdx_);
                
                // geometry_msgs::PoseStamped rbtPoseInSensorFrame_lc = rbtPoseInSensorFrame;
                // sensor_msgs::LaserScan static_scan = *static_scan_ptr.get();
                
                rawCmdVel = trajController_->controlLaw(currPoseWorldFrame, targetTrajectoryPose, 
                                                        currentPeakSplineVel_);
            }
            // geometry_msgs::PoseStamped rbtPoseInSensorFrame_lc = rbtPoseInSensorFrame;

            cmdVel = trajController_->processCmdVel(rawCmdVel,
                                                    rbtPoseInSensorFrame_, 
                                                    currLeftGapPtModel_, currRightGapPtModel_,
                                                    currentRbtVel_, currentRbtAcc_); 
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("Controller", "ctrlGeneration failed");
        }

        return cmdVel;
    }

    
    std::vector<dynamic_gap::Gap *> Planner::gapSetFeasibilityCheck(bool & isCurrentGapFeasible)                                             
    {
        boost::mutex::scoped_lock gapset(gapsetMutex);        
        ROS_INFO_STREAM_NAMED("GapFeasibility", "[gapSetFeasibilityCheck()]");
        std::vector<dynamic_gap::Gap *> feasibleGaps;

        try
        {
            // grabbing the current set of gaps
            std::vector<dynamic_gap::Gap *> currGaps = currSimplifiedGaps_;

            //std::cout << "pulled current simplified associations:" << std::endl;

            int currentRightGapPtModelID = getCurrentRightGapPtModelID();
            int currentLeftGapPtModelID = getCurrentLeftGapPtModelID();

            ROS_INFO_STREAM_NAMED("GapFeasibility", "    current simplified gaps:");
            printGapModels(currGaps);
            ROS_INFO_STREAM_NAMED("GapFeasibility", "    current left/right model IDs: " << currentLeftGapPtModelID << ", " << currentRightGapPtModelID);
            
            // isCurrentGapAssociated = false;
            isCurrentGapFeasible = false;

            bool isGapFeasible = false;
            for (size_t i = 0; i < currGaps.size(); i++) 
            {
                // obtain crossing point

                ROS_INFO_STREAM_NAMED("GapFeasibility", "    feasibility check for gap " << i);
                isGapFeasible = gapFeasibilityChecker_->indivGapFeasibilityCheck(currGaps.at(i));
                isGapFeasible = true;

                if (isGapFeasible) 
                {
                    currGaps.at(i)->addTerminalRightInformation();
                    feasibleGaps.push_back(new dynamic_gap::Gap(*currGaps.at(i)));
                    // ROS_INFO_STREAM("Pushing back gap with peak velocity of : " << gaps.at(i)->peakSplineVelX_ << ", " << gaps.at(i)->peakSplineVelY_);
                }

                if (currGaps.at(i)->leftGapPtModel_->getID() == currentLeftGapPtModelID && currGaps.at(i)->rightGapPtModel_->getID() == currentRightGapPtModelID) 
                {
                    // isCurrentGapAssociated = true;
                    isCurrentGapFeasible = true;
                }
            }

            
            // if (gap_associated) {
            //     ROS_INFO_STREAM("currently executing gap associated");
            // } else {
            //     ROS_INFO_STREAM("currently executing gap NOT associated");
            // }
            
        } catch(...)
        {
            ROS_WARN_STREAM_NAMED("GapFeasibility", "   gapSetFeasibilityCheck failed");
        }

        return feasibleGaps;
    }

    void Planner::getFutureScans() 
    {
        try
        {
            ROS_INFO_STREAM_NAMED("ScanPropagation", "[getFutureScans()]");
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
            futureScans_.at(0) = dynamicScan; // at t = 0.0

            std::vector<Eigen::Vector4f> currentAgents;
            
            if (cfg_.planning.egocircle_prop_cheat) 
            {
                Eigen::Vector4f ithAgentState;
                currentAgents.clear();
                for (int i = 0; i < currentAgentCount_; i++) 
                {
                    ithAgentState << currentTrueAgentPoses_.at(i).position.x, currentTrueAgentPoses_.at(i).position.y, 
                                    currentTrueAgentVels_.at(i).vector.x, currentTrueAgentVels_.at(i).vector.y;
                    currentAgents.push_back(ithAgentState);
                }
            } else 
            {
                currentAgents = currentEstimatedAgentStates_;
            }

            ROS_INFO_STREAM_NAMED("ScanPropagation", "    detected agents: ");
            for (int i = 0; i < currentAgents.size(); i++)
                ROS_INFO_STREAM_NAMED("ScanPropagation", "        agent" << i << " position: " << currentAgents.at(i)[0] << ", " << currentAgents.at(i)[1] << ", velocity: " << currentAgents.at(i)[2] << ", " << currentAgents.at(i)[3]);
            
            int futureScanTimeIdx = -1;
            for (float t_iplus1 = cfg_.traj.integrate_stept; t_iplus1 <= cfg_.traj.integrate_maxt; t_iplus1 += cfg_.traj.integrate_stept) 
            {
                dynamicScan.ranges = scan.ranges;

                //trajScorer_->recoverDynamicEgocircleCheat(t_i, t_iplus1, agentPoses__lc, agentVels__lc, dynamicScan);
                // trajScorer_->recoverDynamicEgoCircle(t_i, t_iplus1, currentAgents, dynamicScan);
                
                futureScanTimeIdx = (int) (t_iplus1 / cfg_.traj.integrate_stept);
                // ROS_INFO_STREAM("adding scan from " << t_i << " to " << t_iplus1 << " at idx: " << futureScanTimeIdx);
                futureScans_[futureScanTimeIdx] = dynamicScan;

                t_i = t_iplus1;
            }
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("ScanPropagation", "  getFutureScans failed");
        }
    }


    void Planner::visualizeComponents(const std::vector<dynamic_gap::Gap *> & manipulatedGaps) 
    {
        boost::mutex::scoped_lock gapset(gapsetMutex);

        gapVisualizer_->drawManipGaps(manipulatedGaps, std::string("manip"));
        // gapVisualizer_->drawReachableGaps(manipulatedGaps);        
        // gapVisualizer_->drawReachableGapsCenters(manipulatedGaps); 

        // gapVisualizer_->drawGapSplines(manipulatedGaps);
        // goalVisualizer_->drawGapGoals(manipulatedGaps);
    }

    /*
    void Planner::printGapAssociations(const std::vector<dynamic_gap::Gap *> & currentGaps, 
                                       const std::vector<dynamic_gap::Gap *> & previousGaps, 
                                       const std::vector<int> & association) 
    {
        std::cout << "current simplified associations" << std::endl;
        std::cout << "number of gaps: " << currentGaps.size() << ", number of previous gaps: " << previousGaps.size() << std::endl;
        std::cout << "association size: " << association.size() << std::endl;

        for (int i = 0; i < association.size(); i++)
            std::cout << association.at(i) << ", ";
        std::cout << "" << std::endl;

        float currX = 0.0, currY = 0.0, prevX = 0.0, prevY = 0.0;
        for (int i = 0; i < association.size(); i++) 
        {
            std::vector<int> pair{i, association.at(i)};
            std::cout << "pair (" << i << ", " << association.at(i) << "). ";
            int currentGapIdx = int(std::floor(pair.at(0) / 2.0));
            int previousGapIdx = int(std::floor(pair.at(1) / 2.0));

            if (pair.at(0) % 2 == 0)  // curr left
                currentGaps.at(currentGapIdx)->getSimplifiedRCartesian(currX, currY);
            else // curr right
                currentGaps.at(currentGapIdx)->getSimplifiedLCartesian(currX, currY);
            
            if (association.at(i) >= 0 && association.at(i) > 0) 
            {
                if (pair.at(1) % 2 == 0) // prev left
                    previousGaps.at(previousGapIdx)->getSimplifiedRCartesian(prevX, prevY);
                else // prev right
                    previousGaps.at(previousGapIdx)->getSimplifiedLCartesian(prevX, prevY);
                
                std::cout << "From (" << prevX << ", " << prevY << ") to (" << currX << ", " << currY << ") with a distance of " << simpDistMatrix_.at(pair.at(0)).at(pair.at(1)) << std::endl;
            } else 
            {
                std::cout << "From NULL to (" << currX << ", " <<  currY << ")" << std::endl;
            }
        }
    }
    */

    void Planner::printGapModels(const std::vector<dynamic_gap::Gap *> & gaps) 
    {
        // THIS IS NOT FOR MANIPULATED GAPS
        float x = 0.0, y = 0.0;
        for (size_t i = 0; i < gaps.size(); i++)
        {
            dynamic_gap::Gap * gap = gaps.at(i);
            ROS_INFO_STREAM_NAMED("Planner", "    gap " << i << ", indices: " << gap->RIdx() << " to "  << gap->LIdx() << ", left model: " << gap->leftGapPtModel_->getID() << ", rightGapPtModel: " << gap->rightGapPtModel_->getID());
            Eigen::Vector4f left_state = gap->leftGapPtModel_->getState();
            gap->getLCartesian(x, y);            
            ROS_INFO_STREAM_NAMED("Planner", "        left point: (" << x << ", " << y << "), left model: (" << left_state[0] << ", " << left_state[1] << ", " << left_state[2] << ", " << left_state[3] << ")");
            Eigen::Vector4f right_state = gap->rightGapPtModel_->getState();
            gap->getRCartesian(x, y);
            ROS_INFO_STREAM_NAMED("Planner", "        right point: (" << x << ", " << y << "), right model: (" << right_state[0] << ", " << right_state[1] << ", " << right_state[2] << ", " << right_state[3] << ")");
        }
    }


    dynamic_gap::Trajectory Planner::runPlanningLoop() 
    {
        if (!readyToPlan)
            return dynamic_gap::Trajectory();

        ROS_INFO_STREAM_NAMED("Planner", "[runPlanningLoop()]");

        bool isCurrentGapFeasible = false; // isCurrentGapAssociated, 

        std::chrono::steady_clock::time_point planningLoopStartTime = std::chrono::steady_clock::now();

        ///////////////////////////
        // GAP FEASIBILITY CHECK //
        ///////////////////////////
        std::vector<dynamic_gap::Gap *> feasibleGaps;
        std::chrono::steady_clock::time_point feasibilityStartTime = std::chrono::steady_clock::now();
        if (cfg_.planning.gap_feasibility_check)
        {
            feasibleGaps = gapSetFeasibilityCheck(isCurrentGapFeasible);
        } else
        {
            for (dynamic_gap::Gap * currSimplifiedGap : currSimplifiedGaps_)
                feasibleGaps.push_back(new dynamic_gap::Gap(*currSimplifiedGap));
            // feasibleGaps = currSimplifiedGaps_;
            // need to set feasible to true for all gaps as well
        }
        int gapCount = feasibleGaps.size();
        float feasibilityTimeTaken = timeTaken(feasibilityStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[gapSetFeasibilityCheck() for " << gapCount << " gaps: " << feasibilityTimeTaken << " seconds]");

        /////////////////////////////
        // FUTURE SCAN PROPAGATION //
        /////////////////////////////
        std::chrono::steady_clock::time_point scanPropagationStartTime = std::chrono::steady_clock::now();
        if (cfg_.planning.future_scan_propagation)
        {
            getFutureScans();
        } else 
        {
            sensor_msgs::LaserScan currentScan = *scan_.get();
            for (int i = 0; i < futureScans_.size(); i++)
                futureScans_.at(i) = currentScan;
        }
        float scanPropagationTimeTaken = timeTaken(scanPropagationStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[getFutureScans() for " << gapCount << " gaps: " << scanPropagationTimeTaken << " seconds]");
        
        //////////////////////
        // GAP MANIPULATION //
        //////////////////////
        std::chrono::steady_clock::time_point gapManipulateStartTime = std::chrono::steady_clock::now();
        std::vector<dynamic_gap::Gap *> manipulatedGaps = gapManipulate(feasibleGaps);
        float gapManipulationTimeTaken = timeTaken(gapManipulateStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[gapManipulate() for " << gapCount << " gaps: " << gapManipulationTimeTaken << " seconds]");

        ///////////////////////////////////////////
        // GAP TRAJECTORY GENERATION AND SCORING //
        ///////////////////////////////////////////
        std::chrono::steady_clock::time_point generateGapTrajsStartTime = std::chrono::steady_clock::now();
        std::vector<dynamic_gap::Trajectory> trajs;
        std::vector<std::vector<float>> pathPoseScores; 
        pathPoseScores = generateGapTrajs(manipulatedGaps, trajs);
        float generateGapTrajsTimeTaken = timeTaken(generateGapTrajsStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[generateGapTrajs() for " << gapCount << " gaps: " << generateGapTrajsTimeTaken << " seconds]");

        // need to run after generateGapTrajs to see what weights for reachable gap are
        visualizeComponents(manipulatedGaps); 

        //////////////////////////////
        // GAP TRAJECTORY SELECTION //
        //////////////////////////////
        std::chrono::steady_clock::time_point pickTrajStartTime = std::chrono::steady_clock::now();
        int highestScoreTrajIdx = pickTraj(trajs, pathPoseScores);
        float pickTrajTimeTaken = timeTaken(pickTrajStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[pickTraj() for " << gapCount << " gaps: " << pickTrajTimeTaken << " seconds]");

        dynamic_gap::Trajectory chosenTraj;
        if (highestScoreTrajIdx >= 0) 
        {
            ///////////////////////////////
            // GAP TRAJECTORY COMPARISON //
            ///////////////////////////////
            std::chrono::steady_clock::time_point compareToCurrentTrajStartTime = std::chrono::steady_clock::now();

            chosenTraj = compareToCurrentTraj(manipulatedGaps.at(highestScoreTrajIdx), 
                                              trajs.at(highestScoreTrajIdx),
                                              manipulatedGaps, 
                                              isCurrentGapFeasible);
            float compareToCurrentTrajTimeTaken = timeTaken(compareToCurrentTrajStartTime);
            ROS_INFO_STREAM_NAMED("Planner", "[compareToCurrentTraj() for " << gapCount << " gaps: "  << compareToCurrentTrajTimeTaken << " seconds]");
        } else 
        {
            // highestScorePath = geometry_msgs::PoseArray();
            // highestScoreGap = dynamic_gap::Gap();
        }

        float planningLoopTimeTaken = timeTaken(planningLoopStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[runPlanningLoop() for " << gapCount << " gaps: "  << planningLoopTimeTaken << " seconds]");

        // delete feasible/manipulated set of gaps
        for (dynamic_gap::Gap * manipulatedGap : manipulatedGaps)
        {
            // KEEP CURRENT GAP
            if (manipulatedGap == currentGap_)
                continue;
            else
                delete manipulatedGap;
        }
        // feasibleGaps.clear();
        // manipulatedGaps.clear();

        return chosenTraj;
    }

    bool Planner::recordAndCheckVel(const geometry_msgs::Twist & cmdVel) 
    {
        float val = std::abs(cmdVel.linear.x) + std::abs(cmdVel.linear.y) + std::abs(cmdVel.angular.z);

        cmdVelBuffer_.push_back(val);
        float cmdVelBufferSum = std::accumulate(cmdVelBuffer_.begin(), cmdVelBuffer_.end(), float(0));
        bool keepPlanning = cmdVelBufferSum > 1.0 || !cmdVelBuffer_.full();
        
        if (!keepPlanning && !cfg_.man.man_ctrl) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "--------------------------Planning Failed--------------------------");
            ROS_WARN_STREAM_NAMED("Planner", "--------------------------Planning Failed--------------------------");
            reset();
        }
        return keepPlanning || cfg_.man.man_ctrl;
    }

}