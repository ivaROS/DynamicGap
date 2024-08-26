#include <dynamic_gap/Planner.h>

namespace dynamic_gap
{   
    Planner::Planner()
    {
        // Do something? maybe set names
        ros::NodeHandle nh_("planner_node");
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

        // delete objects
        delete tfListener_;
        
        delete gapDetector_;
        delete gapAssociator_;
        delete gapVisualizer_;

        delete globalPlanManager_;
        delete goalVisualizer_;

        delete gapFeasibilityChecker_;

        delete gapManipulator_;

        delete gapTrajGenerator_;

        delete dynamicScanPropagator_;

        delete trajScorer_;
        delete trajController_;
        delete trajVisualizer_;
    }

    bool Planner::initialize(const ros::NodeHandle& nh)
    {
        // ROS_INFO_STREAM("starting initialize");
        if (initialized_)
        {
            ROS_WARN("DynamicGap Planner already initalized");
            return true;
        }

        // Config Setup
        cfg_.loadRosParamFromNodeHandle(nh);

        // Visualization Setup
        currentTrajectoryPublisher_ = nh_.advertise<geometry_msgs::PoseArray>("curr_exec_dg_traj", 1);
        // staticScanPublisher_ = nh_.advertise<sensor_msgs::LaserScan>("static_scan", 1);

        // TF Lookup setup
        tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
        initialized_ = true;

        gapDetector_ = new dynamic_gap::GapDetector(cfg_);
        gapAssociator_ = new dynamic_gap::GapAssociator(nh_, cfg_);
        gapVisualizer_ = new dynamic_gap::GapVisualizer(nh_, cfg_);

        globalPlanManager_ = new dynamic_gap::GlobalPlanManager(nh_, cfg_);
        goalVisualizer_ = new dynamic_gap::GoalVisualizer(nh_, cfg_);

        gapFeasibilityChecker_ = new dynamic_gap::GapFeasibilityChecker(nh_, cfg_);

        gapManipulator_ = new dynamic_gap::GapManipulator(nh_, cfg_);

        gapTrajGenerator_ = new dynamic_gap::GapTrajectoryGenerator(cfg_);

        dynamicScanPropagator_ = new dynamic_gap::DynamicScanPropagator(nh_, cfg_); 

        trajScorer_ = new dynamic_gap::TrajectoryScorer(nh_, cfg_);
        trajController_ = new dynamic_gap::TrajectoryController(nh_, cfg_);
        trajVisualizer_ = new dynamic_gap::TrajectoryVisualizer(nh_, cfg_);


        // MAP FRAME ID SHOULD BE: known_map
        // ODOM FRAME ID SHOULD BE: map_static

        map2rbt_.transform.rotation.w = 1;
        odom2rbt_.transform.rotation.w = 1;
        rbt2odom_.transform.rotation.w = 1;
        rbtPoseInRbtFrame_.pose.orientation.w = 1;
        rbtPoseInRbtFrame_.header.frame_id = cfg_.robot_frame_id;

        int bufferSize = 5;
        cmdVelBuffer_.set_capacity(bufferSize);

        currentRbtVel_ = geometry_msgs::TwistStamped();
        currentRbtAcc_ = geometry_msgs::TwistStamped();
        
        rbtPoseInOdomFrame_ = geometry_msgs::PoseStamped();
        globalGoalRobotFrame_ = geometry_msgs::PoseStamped();

        currentAgentCount_ = cfg_.env.num_agents;

        currentTrueAgentPoses_ = std::vector<geometry_msgs::Pose>(currentAgentCount_);
        currentTrueAgentVels_ = std::vector<geometry_msgs::Vector3Stamped>(currentAgentCount_);

        // ROS_INFO_STREAM("future_scans size: " << future_scans.size());
        // ROS_INFO_STREAM("done initializing");

        intermediateRbtVels_.clear();
        intermediateRbtAccs_.clear();

        tPreviousModelUpdate_ = ros::Time::now();

        return true;
    }

    bool Planner::isGoalReached()
    {
        float globalGoalXDiff = globalGoalOdomFrame_.pose.position.x - rbtPoseInOdomFrame_.pose.position.x;
        float globalGoalYDiff = globalGoalOdomFrame_.pose.position.y - rbtPoseInOdomFrame_.pose.position.y;
        bool reachedGlobalGoal = sqrt(pow(globalGoalXDiff, 2) + pow(globalGoalYDiff, 2)) < cfg_.goal.goal_tolerance;
        
        if (reachedGlobalGoal)
        {
            ROS_INFO_STREAM_NAMED("Planner", "[Reset] Goal Reached");
            return true;
        }

        return false;
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> scan)
    {
        boost::mutex::scoped_lock gapset(gapMutex_);
        scan_ = scan;

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

            ROS_INFO_STREAM_NAMED("GapAssociator", "Raw gap association");    
            rawDistMatrix_ = gapAssociator_->obtainDistMatrix(currRawGaps_, prevRawGaps_);
            rawAssocation_ = gapAssociator_->associateGaps(rawDistMatrix_);
            gapAssociator_->assignModels(rawAssocation_, rawDistMatrix_, 
                                        currRawGaps_, prevRawGaps_, 
                                        currentModelIdx_, tCurrentFilterUpdate,
                                        intermediateRbtVels, intermediateRbtAccs);
            ROS_INFO_STREAM_NAMED("GapEstimator", "Raw gap model updates");    
            updateModels(currRawGaps_, intermediateRbtVels, 
                         intermediateRbtAccs, tCurrentFilterUpdate);
            
            // std::chrono::steady_clock::time_point gap_simplification_start_time = std::chrono::steady_clock::now();
            
            currSimplifiedGaps_ = gapDetector_->gapSimplification(currRawGaps_);
            // float gap_simplification_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_simplification_start_time).count() / 1.0e6;
            // ROS_INFO_STREAM("gapSimplification: " << gap_simplification_time << " seconds");
            
            ROS_INFO_STREAM_NAMED("GapAssociator", "simplified gap associating");    
            simpDistMatrix_ = gapAssociator_->obtainDistMatrix(currSimplifiedGaps_, prevSimplifiedGaps_);
            simpAssociation_ = gapAssociator_->associateGaps(simpDistMatrix_); // must finish this and therefore change the association
            gapAssociator_->assignModels(simpAssociation_, simpDistMatrix_, 
                                        currSimplifiedGaps_, prevSimplifiedGaps_, 
                                        currentModelIdx_, tCurrentFilterUpdate,
                                        intermediateRbtVels, intermediateRbtAccs);
            ROS_INFO_STREAM_NAMED("GapEstimator", "Simplified gap model updates");    
            updateModels(currSimplifiedGaps_, intermediateRbtVels, 
                         intermediateRbtAccs, tCurrentFilterUpdate);

            gapVisualizer_->drawGaps(currRawGaps_, std::string("raw"));
            gapVisualizer_->drawGapsModels(currRawGaps_);
            gapVisualizer_->drawGaps(currSimplifiedGaps_, std::string("simp"));
            gapVisualizer_->drawGapsModels(currSimplifiedGaps_);

            readyToPlan = true;
        }
        
        // update current scan for proper classes
        updateEgoCircle();

        // update global path local waypoint according to new scan
        globalPlanManager_->generateGlobalPathLocalWaypoint(map2rbt_);
        geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame = globalPlanManager_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);
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
        tPreviousModelUpdate_ = tCurrentFilterUpdate;
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    void Planner::updateModels(std::vector<dynamic_gap::Gap *> & gaps, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                                const ros::Time & tCurrentFilterUpdate) 
    {
        // ROS_INFO_STREAM_NAMED("GapEstimation", "[updateModels()]");
        
        try
        {
            for (int i = 0; i < 2*gaps.size(); i++) 
            {
                // ROS_INFO_STREAM_NAMED("GapEstimation", "    update gap model " << i << " of " << 2*gaps.size());
                updateModel(i, gaps, intermediateRbtVels, intermediateRbtAccs, tCurrentFilterUpdate);
                // ROS_INFO_STREAM_NAMED("GapEstimation", "");
            }
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("GapEstimation", "updateModels failed");
        }

        return;
    }

    void Planner::updateModel(const int & idx, 
                                std::vector<dynamic_gap::Gap *> & gaps, 
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
                if (intermediateRbtAccs_.at(i).header.stamp <= tPreviousModelUpdate_)
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
                rbtPoseInOdomFrame_ = rbtPoseOdomFrame;
            }
            else
            {
                rbtPoseInOdomFrame_.pose = rbtOdomMsg->pose.pose;
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
                if (intermediateRbtVels_.at(i).header.stamp <= tPreviousModelUpdate_)
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
        // ROS_INFO_STREAM_NAMED("Planner", "[agentOdomCB()]");        
        std::string agentNamespace = agentOdomMsg->child_frame_id;
        // ROS_INFO_STREAM_NAMED("Planner", "      agentNamespace: " << agentNamespace);
        agentNamespace.erase(0,5); // removing "robot" from "robotN"
        char * robotChars = strdup(agentNamespace.c_str());
        int agentID = std::atoi(robotChars);
        // ROS_INFO_STREAM_NAMED("Planner", "      agentID: " << agentID);

        try 
        {
            // transforming Odometry message from map_static to robotN
            geometry_msgs::TransformStamped msgFrame2RobotFrame = tfBuffer_.lookupTransform(cfg_.robot_frame_id, agentOdomMsg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped agentPoseMsgFrame, agentPoseRobotFrame;
            agentPoseMsgFrame.header = agentOdomMsg->header;
            agentPoseMsgFrame.pose = agentOdomMsg->pose.pose;
            // ROS_INFO_STREAM_NAMED("Planner", "      incoming pose: (" << agentPoseMsgFrame.pose.position.x << ", " << agentPoseMsgFrame.pose.position.y << ")");

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;
            tf2::doTransform(agentPoseMsgFrame, agentPoseRobotFrame, msgFrame2RobotFrame);
            
            // ROS_INFO_STREAM_NAMED("Planner", "      outgoing pose: (" << agentPoseRobotFrame.pose.position.x << ", " << agentPoseRobotFrame.pose.position.y << ")");

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
        if (globalPlanMapFrame.size() == 0) 
            return true;
        
        geometry_msgs::PoseStamped globalGoalMapFrame = *std::prev(globalPlanMapFrame.end());
        tf2::doTransform(globalGoalMapFrame, globalGoalOdomFrame_, map2odom_); // to update odom frame parameter
        tf2::doTransform(globalGoalOdomFrame_, globalGoalRobotFrame_, odom2rbt_); // to update robot frame parameter
        
        // Store New Global Plan to Goal Selector
        globalPlanManager_->updateGlobalPathMapFrame(globalPlanMapFrame);
        
        // Generate global path local waypoint (furthest part along global path that we can still see)
        globalPlanManager_->generateGlobalPathLocalWaypoint(map2rbt_);

        // return local goal (odom) frame
        geometry_msgs::PoseStamped newglobalPathLocalWaypointOdomFrame = globalPlanManager_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);

        // Plan New
        float diffX = globalPathLocalWaypointOdomFrame_.pose.position.x - newglobalPathLocalWaypointOdomFrame.pose.position.x;
        float diffY = globalPathLocalWaypointOdomFrame_.pose.position.y - newglobalPathLocalWaypointOdomFrame.pose.position.y;
        
        if (sqrt(pow(diffX, 2) + pow(diffY, 2)) > cfg_.goal.waypoint_tolerance)
            globalPathLocalWaypointOdomFrame_ = newglobalPathLocalWaypointOdomFrame;

        // Set new local goal to trajectory arbiter
        trajScorer_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame_, odom2rbt_);

        // Visualization only
        std::vector<geometry_msgs::PoseStamped> visibleGlobalPlanSnippetRobotFrame = globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
        trajVisualizer_->drawRelevantGlobalPlanSnippet(visibleGlobalPlanSnippetRobotFrame);

        hasGlobalGoal_ = true;

        return true;
    }

    void Planner::updateTF()
    {
        try 
        {
            // ROS_INFO_STREAM_NAMED("Planner", "cfg_.robot_frame_id: " << cfg_.robot_frame_id);        
            // ROS_INFO_STREAM_NAMED("Planner", "cfg_.map_frame_id: " << cfg_.map_frame_id);        
            // ROS_INFO_STREAM_NAMED("Planner", "cfg_.odom_frame_id: " << cfg_.odom_frame_id);        
            // ROS_INFO_STREAM_NAMED("Planner", "cfg_.sensor_frame_id: " << cfg_.sensor_frame_id);        

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

    void Planner::updateEgoCircle()
    {
        globalPlanManager_->updateEgoCircle(scan_);
        gapManipulator_->updateEgoCircle(scan_);
        dynamicScanPropagator_->updateEgoCircle(scan_);
        trajScorer_->updateEgoCircle(scan_);
        trajController_->updateEgoCircle(scan_);
    }

    void Planner::propagateGapPoints(const std::vector<dynamic_gap::Gap *> & planningGaps)                                             
    {
        boost::mutex::scoped_lock gapset(gapMutex_);        
        ROS_INFO_STREAM_NAMED("GapFeasibility", "[propagateGapPoints()]");

        // grabbing the current set of gaps
        // std::vector<dynamic_gap::Gap *> propagatedGaps = currSimplifiedGaps_;

        try
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "    current raw gaps:");
            printGapModels(currRawGaps_);

            ROS_INFO_STREAM_NAMED("GapFeasibility", "    current simplified gaps:");
            printGapModels(planningGaps);
            
            for (size_t i = 0; i < planningGaps.size(); i++) 
            {
                // propagate gap forward in time to determine lifespan
                gapFeasibilityChecker_->propagateGapPoints(planningGaps.at(i));
            }

        } catch(...)
        {
            ROS_WARN_STREAM_NAMED("GapFeasibility", "   propagateGapPoints failed");
        }

        return;

    }

    std::vector<dynamic_gap::Gap *> Planner::gapManipulate(const std::vector<dynamic_gap::Gap *> & planningGaps) 
    {

        ROS_INFO_STREAM_NAMED("GapManipulator", "[gapManipulate()]");

        boost::mutex::scoped_lock gapset(gapMutex_);
        std::vector<dynamic_gap::Gap *> manipulatedGaps;

        try
        {
            for (size_t i = 0; i < planningGaps.size(); i++)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "    manipulating initial gap " << i);

                // MANIPULATE POINTS AT T=0            
                bool success = gapManipulator_->inflateGapSides(planningGaps.at(i));
                
                if (success)
                {
                    ROS_INFO_STREAM_NAMED("GapManipulator", "    pushing back manipulated gap " << i);

                    // gapManipulator_->radialExtendGap(manipulatedGaps.at(i)); // to set s
                    gapManipulator_->setGapGoal(planningGaps.at(i), 
                                                globalPlanManager_->getGlobalPathLocalWaypointRobotFrame(),
                                                globalGoalRobotFrame_);
                    
                    // MANIPULATE POINTS AT T=1
                    // ROS_INFO_STREAM_NAMED("GapManipulator", "    manipulating terminal gap " << i);
                
                    manipulatedGaps.push_back(planningGaps.at(i)); // shallow copy
                }
            }
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("GapManipulator", "   gapManipulate failed");
        }

        return manipulatedGaps;
    }

    std::vector<dynamic_gap::Gap *> Planner::gapSetFeasibilityCheck(const std::vector<dynamic_gap::Gap *> & manipulatedGaps, 
                                                                    bool & isCurrentGapFeasible)                                             
    {
        boost::mutex::scoped_lock gapset(gapMutex_);        
        ROS_INFO_STREAM_NAMED("GapFeasibility", "[gapSetFeasibilityCheck()]");
        std::vector<dynamic_gap::Gap *> feasibleGaps;

        try
        {
            // grabbing the current set of gaps

            int currentLeftGapPtModelID = getCurrentLeftGapPtModelID();
            int currentRightGapPtModelID = getCurrentRightGapPtModelID();
            ROS_INFO_STREAM_NAMED("GapFeasibility", "    current left/right model IDs: " << currentLeftGapPtModelID << ", " << currentRightGapPtModelID);

            isCurrentGapFeasible = false;

            bool isGapFeasible = false;
            for (size_t i = 0; i < manipulatedGaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("GapFeasibility", "    feasibility check for gap " << i);

                // run pursuit guidance analysis on gap to determine feasibility
                isGapFeasible = gapFeasibilityChecker_->pursuitGuidanceAnalysis(manipulatedGaps.at(i));

                if (isGapFeasible) 
                {
                    // manipulatedGaps.at(i)->addTerminalRightInformation();
                    
                    feasibleGaps.push_back(manipulatedGaps.at(i)); // shallow copy

                    // feasibleGaps.push_back(new dynamic_gap::Gap(*manipulatedGaps.at(i)));
                    // ROS_INFO_STREAM("Pushing back gap with peak velocity of : " << gaps.at(i)->peakSplineVelX_ << ", " << gaps.at(i)->peakSplineVelY_);
                
                    if (manipulatedGaps.at(i)->leftGapPtModel_->getID() == currentLeftGapPtModelID && 
                        manipulatedGaps.at(i)->rightGapPtModel_->getID() == currentRightGapPtModelID) 
                    {
                        isCurrentGapFeasible = true;
                    }
                }
            }
            
        } catch(...)
        {
            ROS_WARN_STREAM_NAMED("GapFeasibility", "   gapSetFeasibilityCheck failed");
        }

        return feasibleGaps;
    }

    void Planner::generateGapTrajs(std::vector<dynamic_gap::Gap *> & gaps, 
                                    std::vector<dynamic_gap::Trajectory> & generatedTrajs,
                                    std::vector<std::vector<float>> & pathPoseScores,
                                    std::vector<float> & pathTerminalPoseScores,
                                    const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateGapTrajs()]");
        
        pathPoseScores = std::vector<std::vector<float>>(gaps.size());
        pathTerminalPoseScores = std::vector<float>(gaps.size());

        try 
        {
            for (size_t i = 0; i < gaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    generating traj for gap: " << i);
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                
                // Run go to goal behavior
                bool runGoToGoal = gaps.at(i)->globalGoalWithin; // (vec.at(i).goal.goalwithin || vec.at(i).artificial);

                dynamic_gap::Trajectory traj, goToGoalTraj, pursuitGuidanceTraj;
                std::vector<float> goToGoalPoseScores, pursuitGuidancePoseScores;
                float goToGoalTerminalPoseScore, pursuitGuidanceTerminalPoseScore;
                float goToGoalScore, pursuitGuidancePoseScore;

                if (runGoToGoal) 
                {
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        running goToGoal");

                    goToGoalTraj = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbtPoseInSensorFrame_, 
                                                                        currentRbtVel_, 
                                                                        globalGoalRobotFrame_,
                                                                        true);
                    goToGoalTraj = gapTrajGenerator_->processTrajectory(goToGoalTraj);
                    trajScorer_->scoreTrajectory(goToGoalTraj, goToGoalPoseScores, goToGoalTerminalPoseScore, futureScans);
                    goToGoalScore = goToGoalTerminalPoseScore + std::accumulate(goToGoalPoseScores.begin(), goToGoalPoseScores.end(), float(0));
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        goToGoalScore: " << goToGoalScore);
                }

                // Run pursuit guidance behavior
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        running pursuit guidance");
                pursuitGuidanceTraj = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbtPoseInSensorFrame_, 
                                                                    currentRbtVel_, 
                                                                    globalGoalRobotFrame_,
                                                                    false);

                pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj);
                trajScorer_->scoreTrajectory(pursuitGuidanceTraj, pursuitGuidancePoseScores, pursuitGuidanceTerminalPoseScore, futureScans);
                pursuitGuidancePoseScore = pursuitGuidanceTerminalPoseScore + std::accumulate(pursuitGuidancePoseScores.begin(), pursuitGuidancePoseScores.end(), float(0));
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        pursuitGuidancePoseScore: " << pursuitGuidancePoseScore);

                if (runGoToGoal && goToGoalScore > pursuitGuidancePoseScore)
                {
                    traj = goToGoalTraj;
                    pathPoseScores.at(i) = goToGoalPoseScores;
                    pathTerminalPoseScores.at(i) = goToGoalTerminalPoseScore;
                } else
                {
                    traj = pursuitGuidanceTraj;
                    pathPoseScores.at(i) = pursuitGuidancePoseScores;
                    pathTerminalPoseScores.at(i) = pursuitGuidanceTerminalPoseScore;
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

        return;
    }

    int Planner::pickTraj(const std::vector<dynamic_gap::Trajectory> & trajs, 
                          const std::vector<std::vector<float>> & pathPoseScores, 
                          const std::vector<float> & pathTerminalPoseScores) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

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

            for (size_t i = 0; i < pathScores.size(); i++) 
            {
                // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());

                pathScores.at(i) = pathTerminalPoseScores.at(i) + std::accumulate(pathPoseScores.at(i).begin(), 
                                                                                  pathPoseScores.at(i).end(), float(0));
                
                pathScores.at(i) = trajs.at(i).getPathRbtFrame().poses.size() == 0 ? std::numeric_limits<float>::infinity() : pathScores.at(i);
                
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    for gap " << i << " (length: " << trajs.at(i).getPathRbtFrame().poses.size() << "), returning score of " << pathScores.at(i));
                
            }

            auto highestPathScoreIter = std::min_element(pathScores.begin(), pathScores.end());
            highestPathScoreIdx = std::distance(pathScores.begin(), highestPathScoreIter);

            if (pathScores.at(highestPathScoreIdx) == std::numeric_limits<float>::infinity()) 
            {    
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    all infinity");
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
        if (switchToIncoming) 
        {
            setCurrentTraj(incomingTraj);
            // setCurrentPath(incomingPath);
            // setCurrentPathTiming(incomingPathTiming);
            setCurrentLeftGapPtModelID(incomingGap->leftGapPtModel_);
            setCurrentRightGapPtModelID(incomingGap->rightGapPtModel_);
            currentTrajectoryPublisher_.publish(incomingTraj.getPathRbtFrame());          
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, incomingTraj);
            trajectoryChangeCount_++;

            return incomingTraj;  
        } else 
        {
            dynamic_gap::Trajectory emptyTraj;
            geometry_msgs::PoseArray emptyPath = geometry_msgs::PoseArray();
            emptyPath.header = incomingTraj.getPathRbtFrame().header;
            std::vector<float> emptyPathTiming;
            setCurrentTraj(emptyTraj);
            // setCurrentPath(emptyPath);
            // setCurrentPathTiming(emptyPathTiming);
            setCurrentLeftGapPtModelID(nullptr);
            setCurrentRightGapPtModelID(nullptr);
            currentTrajectoryPublisher_.publish(emptyTraj.getPathRbtFrame());
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, emptyTraj);
            trajectoryChangeCount_++;            

            return emptyTraj;
        }        

    }

    dynamic_gap::Trajectory Planner::compareToCurrentTraj(dynamic_gap::Gap * incomingGap, 
                                                            const dynamic_gap::Trajectory & incomingTraj,                                                        
                                                            const std::vector<dynamic_gap::Gap *> & feasibleGaps, 
                                                            const bool & isIncomingGapFeasible,
                                                            const std::vector<sensor_msgs::LaserScan> & futureScans) // bool isIncomingGapAssociated,
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[compareToCurrentTraj()]");
        boost::mutex::scoped_lock gapset(gapMutex_);
        
        dynamic_gap::Trajectory currentTraj = getCurrentTraj();

        try 
        {            
            // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapPtModelID() << ", current right gap index: " << getCurrentRightGapPtModelID());

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
            std::vector<float> incomingPathPoseScores;
            float incomingPathTerminalPoseScore;
            trajScorer_->scoreTrajectory(incomingTraj, incomingPathPoseScores, incomingPathTerminalPoseScore, futureScans);

            float incomingPathScore = incomingPathTerminalPoseScore + std::accumulate(incomingPathPoseScores.begin(), incomingPathPoseScores.end(), float(0));
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
            } else if (incomingPathScore == std::numeric_limits<float>::infinity())
            {
                incomingPathStatus = "incoming path is of score infinity.";
                ableToSwitchToIncomingPath = false;
            }
                

            ///////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing path is empty (size: 0) //
            ///////////////////////////////////////////////////////////////////////////////////
            bool isCurrentPathEmpty = currentTraj.getPathRbtFrame().poses.size() == 0;
            if (isCurrentPathEmpty) 
            {
                std::string currentPathStatus = "";
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  
                                            ": current path is of length zero, " << incomingPathStatus);                
                return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
            } 

            ///////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing path is empty (size: 0) //
            ///////////////////////////////////////////////////////////////////////////////////
            if (!isIncomingGapFeasible) 
            {
                std::string currentPathStatus = "";
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  
                                            ": current gap is not feasible, " << incomingPathStatus);                
                return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
            } 

            ////////////////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing path has been completely tracked //
            ////////////////////////////////////////////////////////////////////////////////////////////   
            geometry_msgs::PoseArray currentPathRobotFrame = gapTrajGenerator_->transformPath(currentTraj.getPathOdomFrame(), odom2rbt_);
            // currentPathRobotFrame.header.frame_id = cfg_.robot_frame_id;
            int currentPathPoseIdx = getClosestTrajectoryPoseIdx(currentPathRobotFrame);
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
            int poseCheckCount = std::min(incomingTraj.getPathRbtFrame().poses.size(), reducedCurrentPathRobotFrame.poses.size());

            incomingPathScore = std::accumulate(incomingPathPoseScores.begin(), incomingPathPoseScores.begin() + poseCheckCount, float(0));

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    re-scored incoming trajectory received a subscore of: " << incomingPathScore);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    scoring current trajectory");            
            
            dynamic_gap::Trajectory reducedCurrentTraj(reducedCurrentPathRobotFrame, reducedCurrentPathTiming);
            std::vector<float> currentPathPoseScores;
            float currentPathTerminalPoseScore;
            trajScorer_->scoreTrajectory(reducedCurrentTraj, currentPathPoseScores, currentPathTerminalPoseScore, futureScans);
            float currentPathSubscore = currentPathTerminalPoseScore + std::accumulate(currentPathPoseScores.begin(), currentPathPoseScores.begin() + poseCheckCount, float(0));
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    current trajectory received a subscore of: " << currentPathSubscore);

            std::vector<std::vector<float>> pathPoseScores(2);
            pathPoseScores.at(0) = incomingPathPoseScores;
            pathPoseScores.at(1) = currentPathPoseScores;

            
            if (currentPathSubscore == std::numeric_limits<float>::infinity()) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ << 
                                                            ": current trajectory is of score infinity," << incomingPathStatus);
                return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
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


    int Planner::getClosestTrajectoryPoseIdx(const geometry_msgs::PoseArray & currTrajRbtFrame) 
    {
        std::vector<float> pathPoseNorms(currTrajRbtFrame.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pathPoseNorms.size(); i++) // i will always be positive, so this is fine
        {
            pathPoseNorms.at(i) = sqrt(pow(currTrajRbtFrame.poses.at(i).position.x, 2) + 
                                    pow(currTrajRbtFrame.poses.at(i).position.y, 2));
        }

        auto minPoseNormIter = std::min_element(pathPoseNorms.begin(), pathPoseNorms.end());
        int minPoseNormIdx = std::distance(pathPoseNorms.begin(), minPoseNormIter) + 1;
        return std::min(minPoseNormIdx, int(currTrajRbtFrame.poses.size() - 1));
    }

    std::vector<dynamic_gap::Gap *> Planner::deepCopyCurrentRawGaps()
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        std::vector<dynamic_gap::Gap *> copiedRawGaps;

        for (dynamic_gap::Gap * currRawGap : currRawGaps_)
            copiedRawGaps.push_back(new dynamic_gap::Gap(*currRawGap));

        return copiedRawGaps;
    }

    std::vector<dynamic_gap::Gap *> Planner::deepCopyCurrentSimplifiedGaps()
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        std::vector<dynamic_gap::Gap *> planningGaps;

        for (dynamic_gap::Gap * currSimplifiedGap : currSimplifiedGaps_)
            planningGaps.push_back(new dynamic_gap::Gap(*currSimplifiedGap));

        return planningGaps;
    }

    dynamic_gap::Trajectory Planner::runPlanningLoop() 
    {
        if (!readyToPlan)
            return dynamic_gap::Trajectory();

        ROS_INFO_STREAM_NAMED("Planner", "[runPlanningLoop()]");

        std::vector<dynamic_gap::Gap *> copiedRawGaps = deepCopyCurrentRawGaps();
        std::vector<dynamic_gap::Gap *> planningGaps = deepCopyCurrentSimplifiedGaps();

        bool isCurrentGapFeasible = false;

        std::chrono::steady_clock::time_point planningLoopStartTime = std::chrono::steady_clock::now();

        ///////////////////////////
        // GAP POINT PROPAGATION //
        ///////////////////////////
        propagateGapPoints(planningGaps);

        //////////////////////
        // GAP MANIPULATION //
        //////////////////////
        std::chrono::steady_clock::time_point gapManipulateStartTime = std::chrono::steady_clock::now();
        std::vector<dynamic_gap::Gap *> manipulatedGaps = gapManipulate(planningGaps);
        float gapManipulationTimeTaken = timeTaken(gapManipulateStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[gapManipulate(): " << gapManipulationTimeTaken << " seconds]");

        ///////////////////////////
        // GAP FEASIBILITY CHECK //
        ///////////////////////////
        std::vector<dynamic_gap::Gap *> feasibleGaps;
        std::chrono::steady_clock::time_point feasibilityStartTime = std::chrono::steady_clock::now();
        if (cfg_.planning.gap_feasibility_check)
        {
            feasibleGaps = gapSetFeasibilityCheck(manipulatedGaps, isCurrentGapFeasible);
        } else
        {
            for (dynamic_gap::Gap * currSimplifiedGap : currSimplifiedGaps_)
                feasibleGaps.push_back(currSimplifiedGap);
                // feasibleGaps.push_back(new dynamic_gap::Gap(*currSimplifiedGap));
            // TODO: need to set feasible to true for all gaps as well
        }
        int gapCount = feasibleGaps.size();
        float feasibilityTimeTaken = timeTaken(feasibilityStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[gapSetFeasibilityCheck() for " << gapCount << " gaps: " << feasibilityTimeTaken << " seconds]");

        visualizeNavigableGaps(feasibleGaps); 

        /////////////////////////////
        // FUTURE SCAN PROPAGATION //
        /////////////////////////////
        std::chrono::steady_clock::time_point scanPropagationStartTime = std::chrono::steady_clock::now();
        std::vector<sensor_msgs::LaserScan> futureScans;

        if (cfg_.planning.future_scan_propagation)
        {
            // futureScans = dynamicScanPropagator_->propagateCurrentLaserScanCheat(currentTrueAgentPoses_, currentTrueAgentVels_);
            // throw std::runtime_error("Egocircle propagation is not implemented yet!");
            futureScans = dynamicScanPropagator_->propagateCurrentLaserScan(copiedRawGaps);
        
        } else 
        {
            sensor_msgs::LaserScan currentScan = *scan_.get();
            futureScans = std::vector<sensor_msgs::LaserScan>(int(cfg_.traj.integrate_maxt/cfg_.traj.integrate_stept) + 1, currentScan);
        }
        float scanPropagationTimeTaken = timeTaken(scanPropagationStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[getFutureScans() for " << gapCount << " gaps: " << scanPropagationTimeTaken << " seconds]");
    
        ///////////////////////////////////////////
        // GAP TRAJECTORY GENERATION AND SCORING //
        ///////////////////////////////////////////
        std::chrono::steady_clock::time_point generateGapTrajsStartTime = std::chrono::steady_clock::now();
        std::vector<dynamic_gap::Trajectory> trajs;
        std::vector<std::vector<float>> pathPoseScores; 
        std::vector<float> pathTerminalPoseScores; 

        generateGapTrajs(feasibleGaps, trajs, pathPoseScores, pathTerminalPoseScores, futureScans);
        float generateGapTrajsTimeTaken = timeTaken(generateGapTrajsStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[generateGapTrajs() for " << gapCount << " gaps: " << generateGapTrajsTimeTaken << " seconds]");

        //////////////////////////////
        // GAP TRAJECTORY SELECTION //
        //////////////////////////////
        std::chrono::steady_clock::time_point pickTrajStartTime = std::chrono::steady_clock::now();
        int highestScoreTrajIdx = pickTraj(trajs, pathPoseScores, pathTerminalPoseScores);
        float pickTrajTimeTaken = timeTaken(pickTrajStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[pickTraj() for " << gapCount << " gaps: " << pickTrajTimeTaken << " seconds]");

        dynamic_gap::Trajectory chosenTraj;
        if (highestScoreTrajIdx >= 0) 
        {
            ///////////////////////////////
            // GAP TRAJECTORY COMPARISON //
            ///////////////////////////////
            std::chrono::steady_clock::time_point compareToCurrentTrajStartTime = std::chrono::steady_clock::now();

            chosenTraj = compareToCurrentTraj(feasibleGaps.at(highestScoreTrajIdx), 
                                              trajs.at(highestScoreTrajIdx),
                                              feasibleGaps, 
                                              isCurrentGapFeasible,
                                              futureScans);
            float compareToCurrentTrajTimeTaken = timeTaken(compareToCurrentTrajStartTime);
            ROS_INFO_STREAM_NAMED("Planner", "[compareToCurrentTraj() for " << gapCount << " gaps: "  << compareToCurrentTrajTimeTaken << " seconds]");
        } 

        float planningLoopTimeTaken = timeTaken(planningLoopStartTime);
        ROS_INFO_STREAM_NAMED("Planner", "[runPlanningLoop() for " << gapCount << " gaps: "  << planningLoopTimeTaken << " seconds]");

        // delete set of planning gaps
        for (dynamic_gap::Gap * planningGap : planningGaps)
            delete planningGap;

        // delete set of planning gaps
        for (dynamic_gap::Gap * copiedRawGap : copiedRawGaps)
            delete copiedRawGap;

        return chosenTraj;
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
                geometry_msgs::PoseStamped currPoseStampedOdomFrame;
                currPoseStampedOdomFrame.header.frame_id = cfg_.odom_frame_id;
                tf2::doTransform(currPoseStRobotFrame, currPoseStampedOdomFrame, rbt2odom_);
                geometry_msgs::Pose currPoseOdomFrame = currPoseStampedOdomFrame.pose;

                // obtain current robot pose in odom frame

                // traj in odom frame here

                // get point along trajectory to target/move towards
                targetTrajectoryPoseIdx_ = trajController_->extractTargetPoseIdx(currPoseOdomFrame, localTrajectory);

                geometry_msgs::Pose targetTrajectoryPose = localTrajectory.poses.at(targetTrajectoryPoseIdx_);

                rawCmdVel = trajController_->constantVelocityControlLaw(currPoseOdomFrame, targetTrajectoryPose);
            }

            cmdVel = trajController_->processCmdVel(rawCmdVel,
                                                    rbtPoseInSensorFrame_, 
                                                    currentRbtVel_, currentRbtAcc_); 
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("Controller", "ctrlGeneration failed");
        }

        return cmdVel;
    }

    void Planner::setCurrentLeftGapPtModelID(dynamic_gap::Estimator * leftModel) 
    { 
        if (leftModel) 
        {
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentLeftGapPtModelID]: setting current left ID to " << leftModel->getID());
            currentLeftGapPtModelID = leftModel->getID(); 
        } else
        {
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentLeftGapPtModelID]: null, setting current left ID to " << -1);
            currentLeftGapPtModelID = -1;
        }
    }

    void Planner::setCurrentRightGapPtModelID(dynamic_gap::Estimator * rightModel) 
    { 
        if (rightModel) 
        {        
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentRightGapPtModelID]: setting current right ID to " << rightModel->getID());
            currentRightGapPtModelID = rightModel->getID();
        } else
        {
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentRightGapPtModelID]: null, setting current right ID to " << -1);
            currentRightGapPtModelID = -1;
        } 
    }

    int Planner::getCurrentLeftGapPtModelID() 
    {
        return currentLeftGapPtModelID;  
    }

    int Planner::getCurrentRightGapPtModelID() 
    {
        return currentRightGapPtModelID;
    }

    void Planner::reset()
    {
        setCurrentTraj(dynamic_gap::Trajectory());
        currentRbtVel_ = geometry_msgs::TwistStamped();
        currentRbtAcc_ = geometry_msgs::TwistStamped();
        ROS_INFO_STREAM_NAMED("Planner", "velocity buffer size: " << cmdVelBuffer_.size());
        cmdVelBuffer_.clear();
        ROS_INFO_STREAM_NAMED("Planner", "velocity buffer size after clear: " << cmdVelBuffer_.size() << ", is full: " << cmdVelBuffer_.capacity());
        return;
    }

    void Planner::visualizeNavigableGaps(const std::vector<dynamic_gap::Gap *> & manipulatedGaps) 
    {
        // boost::mutex::scoped_lock gapset(gapMutex_);

        gapVisualizer_->drawManipGaps(manipulatedGaps, std::string("manip"));
        // gapVisualizer_->drawNavigableGaps(manipulatedGaps, highestScoreTrajIdx);        
        // gapVisualizer_->drawNavigableGapsCenters(manipulatedGaps); 

        // gapVisualizer_->drawGapSplines(manipulatedGaps);
        goalVisualizer_->drawGapGoals(manipulatedGaps);
    }

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

    bool Planner::recordAndCheckVel(const geometry_msgs::Twist & cmdVel) 
    {
        float cmdVelNorm = std::abs(cmdVel.linear.x) + std::abs(cmdVel.linear.y) + std::abs(cmdVel.angular.z);

        cmdVelBuffer_.push_back(cmdVelNorm);
        float cmdVelBufferSum = std::accumulate(cmdVelBuffer_.begin(), cmdVelBuffer_.end(), float(0));
        
        bool keepPlanning = cmdVelBufferSum > 1.0 || !cmdVelBuffer_.full();
        
        if (!keepPlanning && !cfg_.man.man_ctrl) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "--------------------------Planning Failed--------------------------");
            reset();
        }
        return keepPlanning || cfg_.man.man_ctrl;
    }

}