#include <dynamic_gap/Planner.h>

namespace dynamic_gap
{   
    Planner::~Planner()
    {
        // delete current raw, simplified, and selected gaps
        for (Gap * rawGap : currRawGaps_)
            delete rawGap;
        currRawGaps_.clear();

        for (Gap * simplifiedGap : currSimplifiedGaps_)
            delete simplifiedGap;
        currSimplifiedGaps_.clear();

        // delete objects
        delete tfListener_;
        
        delete gapDetector_;
        delete gapPointAssociator_;
        delete gapVisualizer_;

        delete globalPlanManager_;
        delete goalVisualizer_;

        delete gapPropagator_;

        delete gapFeasibilityChecker_;

        delete gapManipulator_;

        delete gapTrajGenerator_;

        delete gapGoalPlacer_;

        delete ungapFeasibilityChecker_;

        delete dynamicScanPropagator_;

        delete trajEvaluator_;
        delete trajController_;
        delete trajVisualizer_;
    }

    bool Planner::initialize(const std::string & name)
    {
        ROS_INFO_STREAM("starting initialize");
        if (initialized_)
        {
            ROS_WARN("DynamicGap Planner already initalized");
            return true;
        }

        // Config Setup
        cfg_.loadRosParamFromNodeHandle(name);

        ROS_INFO_STREAM_NAMED("Planner", "cfg_.scan_topic: " << cfg_.scan_topic);
        ROS_INFO_STREAM_NAMED("Planner", "cfg_.odom_topic: " << cfg_.odom_topic);
        ROS_INFO_STREAM_NAMED("Planner", "cfg_.acc_topic: " << cfg_.acc_topic);
        ROS_INFO_STREAM_NAMED("Planner", "cfg_.ped_topic: " << cfg_.ped_topic);

        ROS_INFO_STREAM_NAMED("Planner", "cfg_.robot_frame_id: " << cfg_.robot_frame_id);
        ROS_INFO_STREAM_NAMED("Planner", "cfg_.sensor_frame_id: " << cfg_.sensor_frame_id);
        ROS_INFO_STREAM_NAMED("Planner", "cfg_.map_frame_id: " << cfg_.map_frame_id);
        ROS_INFO_STREAM_NAMED("Planner", "cfg_.odom_frame_id: " << cfg_.odom_frame_id);

        // Initialize everything
        gapDetector_ = new GapDetector(cfg_);
        gapPointAssociator_ = new GapPointAssociator(cfg_);

        globalPlanManager_ = new GlobalPlanManager(cfg_);

        gapFeasibilityChecker_ = new GapFeasibilityChecker(cfg_);

        gapPropagator_ = new GapPropagator(cfg_);

        gapManipulator_ = new GapManipulator(cfg_);

        gapGoalPlacer_ = new GapGoalPlacer(cfg_);

        gapTrajGenerator_ = new GapTrajectoryGenerator(cfg_);

        ungapTrajGenerator_ = new UngapTrajectoryGenerator(cfg_);

        dynamicScanPropagator_ = new DynamicScanPropagator(nh_, cfg_); 

        trajEvaluator_ = new TrajectoryEvaluator(cfg_);
        trajController_ = new TrajectoryController(nh_, cfg_);

        gapVisualizer_ = new GapVisualizer(nh_, cfg_);
        goalVisualizer_ = new GoalVisualizer(nh_, cfg_);
        trajVisualizer_ = new TrajectoryVisualizer(nh_, cfg_);

        ungapFeasibilityChecker_ = new UngapFeasibilityChecker(cfg_);

        // TF Lookup setup
        tfListener_ = new tf2_ros::TransformListener(tfBuffer_);        
        
        tfSub_ = nh_.subscribe("/tf", 10, &Planner::tfCB, this);

        rbtPoseSub_.subscribe(nh_, cfg_.odom_topic, 10);
        rbtAccSub_.subscribe(nh_, cfg_.acc_topic, 10);
        sync_.reset(new CustomSynchronizer(rbtPoseAndAccSyncPolicy(10), rbtPoseSub_, rbtAccSub_));
        sync_->registerCallback(boost::bind(&Planner::jointPoseAccCB, this, _1, _2));

        // Robot laser scan message subscriber
        laserSub_ = nh_.subscribe(cfg_.scan_topic, 5, &Planner::laserScanCB, this);

        pedOdomSub_ = nh_.subscribe(cfg_.ped_topic, 10, &Planner::pedOdomCB, this);

        // Visualization Setup

        // mpcInputPublisher_ = nh_.advertise<geometry_msgs::PoseArray>("mpc_input", 1);
        // mpcOutputSubscriber_ = nh_.subscribe("mpc_output", 1, &Planner::mpcOutputCB, this);

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

        intermediateRbtVels_.clear();
        intermediateRbtAccs_.clear();

        tPreviousModelUpdate_ = ros::Time::now();

        initialized_ = true;

        return true;
    }

    void Planner::setParams(const EstimationParameters & estParams, const ControlParameters & ctrlParams)
    {
        gapPointAssociator_->updateParams(estParams);
        trajController_->updateParams(ctrlParams);
    }


    bool Planner::isGoalReached()
    {
        float globalGoalXDiff = globalGoalOdomFrame_.pose.position.x - rbtPoseInOdomFrame_.pose.position.x;
        float globalGoalYDiff = globalGoalOdomFrame_.pose.position.y - rbtPoseInOdomFrame_.pose.position.y;
        float globalGoalDist = sqrt(pow(globalGoalXDiff, 2) + pow(globalGoalYDiff, 2));

        float globalGoalOrientation = quaternionToYaw(globalGoalOdomFrame_.pose.orientation);
        float rbtPoseOrientation = quaternionToYaw(rbtPoseInOdomFrame_.pose.orientation);
        float globalGoalAngDist = normalize_theta(globalGoalOrientation - rbtPoseOrientation);
        reachedGlobalGoal_ = globalGoalDist < cfg_.goal.goal_tolerance &&
                             globalGoalAngDist < cfg_.goal.yaw_goal_tolerance;
        
        if (reachedGlobalGoal_)
            ROS_INFO_STREAM_NAMED("Planner", "[Reset] Goal Reached");
        // else
        //     ROS_INFO_STREAM_NAMED("Planner", "Distance from goal: " << globalGoalDist << 
        //                                      ", Goal tolerance: " << cfg_.goal.goal_tolerance);

        return reachedGlobalGoal_;
    }

    // void Planner::mpcOutputCB(boost::shared_ptr<geometry_msgs::PoseArray> mpcOutput)
    // {
    //     if (mpcOutput->poses.size() == 0)
    //     {
    //         ROS_WARN_STREAM("MPC output length zero");
    //         return;
    //     }

    //     if (mpcOutput->poses.size() > 1)
    //     {
    //         // first entry is initial condition

    //         // second entry should be what we want
    //         geometry_msgs::Twist dummyTwist;
    //         dummyTwist.linear.x = mpcOutput->poses[1].position.x;
    //         dummyTwist.linear.y = mpcOutput->poses[1].position.y;
    //         mpcTwist_ = dummyTwist;
    //     } else
    //     {
    //         ROS_WARN_STREAM("MPC output length one");
    //     }
    // }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan> scan)
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("Scan", "[laserScanCB()]");
        ROS_INFO_STREAM_NAMED("Scan", "       timestamp: " << scan->header.stamp);

        std::chrono::steady_clock::time_point scanStartTime = std::chrono::steady_clock::now();
        
        /////////////////////////////////////
        //////// SCAN PRE-PROCESSING ////////
        /////////////////////////////////////

        gapDetector_->preprocessScan(scan);

        scan_ = scan;

        float minScanDist = *std::min_element(scan_->ranges.begin(), scan_->ranges.end());

        if (minScanDist < cfg_.rbt.r_inscr)
        {
            ROS_INFO_STREAM_NAMED("Scan", "       in collision!");
            colliding_ = true;
            return;
        } else
        {
            colliding_ = false;
        }

        cfg_.updateParamFromScan(scan_);

        ros::Time tCurrentFilterUpdate = scan_->header.stamp;

        // grabbing current intermediate robot velocities and accelerations
        std::vector<geometry_msgs::TwistStamped> intermediateRbtVels = intermediateRbtVels_;
        std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs = intermediateRbtAccs_;

        ///////////////////////////////
        //////// GAP DETECTION ////////
        ///////////////////////////////
        std::chrono::steady_clock::time_point gapDetectionStartTime = std::chrono::steady_clock::now();
        currRawGaps_ = gapDetector_->gapDetection(scan_, globalGoalRobotFrame_);
        float gapDetectionTimeTaken = timeTaken(gapDetectionStartTime);
        float avgGapDetectionTimeTaken = computeAverageTimeTaken(gapDetectionTimeTaken, GAP_DET);
        ROS_INFO_STREAM_NAMED("Timing", "      [Gap Detection for " << currRawGaps_.size() << " gaps took " << gapDetectionTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "      [Gap Detection average time: " << avgGapDetectionTimeTaken << " seconds (" << (1.0 / avgGapDetectionTimeTaken) << " Hz) ]");

        /////////////////////////////////////
        //////// RAW GAP ASSOCIATION ////////
        /////////////////////////////////////
        std::chrono::steady_clock::time_point rawGapAssociationStartTime = std::chrono::steady_clock::now();
        rawDistMatrix_ = gapPointAssociator_->populateDistMatrix(currRawGaps_, prevRawGaps_);
        rawAssocation_ = gapPointAssociator_->associate(rawDistMatrix_);
        gapPointAssociator_->assignModels(rawAssocation_, rawDistMatrix_, 
                                    currRawGaps_, prevRawGaps_, 
                                    currentModelIdx_, tCurrentFilterUpdate,
                                    intermediateRbtVels, intermediateRbtAccs);
        float rawGapAssociationTimeTaken = timeTaken(rawGapAssociationStartTime);
        float avgRawGapAssociationTimeTaken = computeAverageTimeTaken(rawGapAssociationTimeTaken, GAP_ASSOC);
        ROS_INFO_STREAM_NAMED("Timing", "      [Raw Gap Association for " << currRawGaps_.size() << " gaps took " << rawGapAssociationTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "      [Raw Gap Association average time: " << avgRawGapAssociationTimeTaken << " seconds (" << (1.0 / avgRawGapAssociationTimeTaken) << " Hz) ]");

        ////////////////////////////////////
        //////// RAW GAP ESTIMATION ////////
        ////////////////////////////////////
        std::chrono::steady_clock::time_point rawGapEstimationStartTime = std::chrono::steady_clock::now();
        updateModels(currRawGaps_, intermediateRbtVels, 
                        intermediateRbtAccs, tCurrentFilterUpdate);
        float rawGapEstimationTimeTaken = timeTaken(rawGapEstimationStartTime);
        float avgRawGapEstimationTimeTaken = computeAverageTimeTaken(rawGapEstimationTimeTaken, GAP_EST);
        ROS_INFO_STREAM_NAMED("Timing", "      [Raw Gap Estimation for " << currRawGaps_.size() << " gaps took " << rawGapEstimationTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "      [Raw Gap Estimation average time: " << avgRawGapEstimationTimeTaken << " seconds (" << (1.0 / avgRawGapEstimationTimeTaken) << " Hz) ]");

        ////////////////////////////////////
        //////// GAP SIMPLIFICATION ////////
        ////////////////////////////////////       
        std::chrono::steady_clock::time_point gapSimplificationStartTime = std::chrono::steady_clock::now();
        currSimplifiedGaps_ = gapDetector_->gapSimplification(currRawGaps_);
        float gapSimplificationTimeTaken = timeTaken(gapSimplificationStartTime);
        float avgGapSimplificationTimeTaken = computeAverageTimeTaken(gapSimplificationTimeTaken, GAP_SIMP);
        ROS_INFO_STREAM_NAMED("Timing", "      [Gap Simplification for " << currSimplifiedGaps_.size() << " gaps took " << gapSimplificationTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "      [Gap Simplification average time: " << avgGapSimplificationTimeTaken << " seconds (" << (1.0 / avgGapSimplificationTimeTaken) << " Hz) ]");

        ////////////////////////////////////////////
        //////// SIMPLIFIED GAP ASSOCIATION ////////
        ////////////////////////////////////////////
        std::chrono::steady_clock::time_point simpGapAssociationStartTime = std::chrono::steady_clock::now();
        simpDistMatrix_ = gapPointAssociator_->populateDistMatrix(currSimplifiedGaps_, prevSimplifiedGaps_);
        simpAssociation_ = gapPointAssociator_->associate(simpDistMatrix_); // must finish this and therefore change the association
        gapPointAssociator_->assignModels(simpAssociation_, simpDistMatrix_, 
                                    currSimplifiedGaps_, prevSimplifiedGaps_, 
                                    currentModelIdx_, tCurrentFilterUpdate,
                                    intermediateRbtVels, intermediateRbtAccs);
        float simpGapAssociationTimeTaken = timeTaken(simpGapAssociationStartTime);
        float avgSimpGapAssociationTimeTaken = computeAverageTimeTaken(simpGapAssociationTimeTaken, GAP_ASSOC);
        ROS_INFO_STREAM_NAMED("Timing", "      [Simplified Gap Association for " << currSimplifiedGaps_.size() << " gaps took " << simpGapAssociationTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "      [Simplified Gap Association average time: " << avgSimpGapAssociationTimeTaken << " seconds (" << (1.0 / avgSimpGapAssociationTimeTaken) << " Hz) ]");

        ///////////////////////////////////////////
        //////// SIMPLIFIED GAP ESTIMATION ////////
        ///////////////////////////////////////////     
        std::chrono::steady_clock::time_point simpGapEstimationStartTime = std::chrono::steady_clock::now();
        updateModels(currSimplifiedGaps_, intermediateRbtVels, 
                        intermediateRbtAccs, tCurrentFilterUpdate);
        float simpGapEstimationTimeTaken = timeTaken(simpGapEstimationStartTime);
        float avgSimpGapEstimationTimeTaken = computeAverageTimeTaken(simpGapEstimationTimeTaken, GAP_EST);
        ROS_INFO_STREAM_NAMED("Timing", "      [Simplified Gap Estimation for " << currRawGaps_.size() << " gaps took " << simpGapEstimationTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "      [Simplified Gap Estimation average time: " << avgSimpGapEstimationTimeTaken << " seconds (" << (1.0 / avgSimpGapEstimationTimeTaken) << " Hz) ]");

        gapVisualizer_->drawGaps(currRawGaps_, std::string("raw"));
        gapVisualizer_->drawGapsModels(currRawGaps_);
        gapVisualizer_->drawGaps(currSimplifiedGaps_, std::string("simp"));
        gapVisualizer_->drawGaps(prevSimplifiedGaps_, std::string("simp_tmin1"));
        gapVisualizer_->drawGapsModels(currSimplifiedGaps_);

        hasLaserScan_ = true;
        
        // update current scan for proper classes
        updateEgoCircle();

        if (hasGlobalGoal_)
        {
            // update global path local waypoint according to new scan
            globalPlanManager_->generateGlobalPathLocalWaypoint(map2rbt_);
            geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame = globalPlanManager_->getGlobalPathLocalWaypointOdomFrame(rbt2odom_);
            goalVisualizer_->drawGlobalPathLocalWaypoint(globalPathLocalWaypointOdomFrame);
            goalVisualizer_->drawGlobalGoal(globalGoalOdomFrame_);
            trajEvaluator_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame, odom2rbt_);
        }

        // delete previous gaps
        for (Gap * prevRawGap : prevRawGaps_)
            delete prevRawGap;
        prevRawGaps_.clear();
        
        for (Gap * prevSimplifiedGap : prevSimplifiedGaps_)
            delete prevSimplifiedGap;
        prevSimplifiedGaps_.clear();

        // update previous gaps
        prevRawGaps_ = currRawGaps_;
        prevSimplifiedGaps_ = currSimplifiedGaps_;

        // update estimator update time
        tPreviousModelUpdate_ = tCurrentFilterUpdate;

        float scanTimeTaken = timeTaken(scanStartTime);
        float avgScanTimeTaken = computeAverageTimeTaken(scanTimeTaken, SCAN);
        ROS_INFO_STREAM_NAMED("Timing", "      [Scan Processing took " << scanTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "      [Scan Processing average time: " << avgScanTimeTaken << " seconds (" << (1.0 / avgScanTimeTaken) << " Hz) ]");
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    void Planner::updateModels(std::vector<Gap *> & gaps, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                                const ros::Time & tCurrentFilterUpdate) 
    {
        // ROS_INFO_STREAM_NAMED("GapEstimation", "[updateModels()]");

        for (int i = 0; i < 2*gaps.size(); i++) 
        {
            // ROS_INFO_STREAM_NAMED("GapEstimation", "    update gap model " << i << " of " << 2*gaps.size());
            updateModel(i, gaps, intermediateRbtVels, intermediateRbtAccs, tCurrentFilterUpdate);
            // ROS_INFO_STREAM_NAMED("GapEstimation", "");
        }

        return;
    }

    void Planner::updateModel(const int & idx, 
                                std::vector<Gap *> & gaps, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                                const ros::Time & tCurrentFilterUpdate) 
    {
        Gap * gap = gaps[int(0.5 * idx)];

        float rX = 0.0, rY = 0.0;
        if (idx % 2 == 0) 
            gap->getLCartesian(rX, rY);            
        else 
            gap->getRCartesian(rX, rY);            

        // ROS_INFO_STREAM("rX: " << rX << ", rY: " << rY);

        Eigen::Vector2f measurement(rX, rY);

        if (idx % 2 == 0) 
        {
            if (gap->getLeftGapPt()->getModel())
            {
                gap->getLeftGapPt()->getModel()->update(measurement, 
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
            if (gap->getRightGapPt()->getModel())
            {
                gap->getRightGapPt()->getModel()->update(measurement, 
                                                        intermediateRbtVels, intermediateRbtAccs, 
                                                        currentTrueAgentPoses_, 
                                                        currentTrueAgentVels_,
                                                        tCurrentFilterUpdate);                    
            } else
            {                
                ROS_WARN_STREAM_NAMED("GapEstimation", "right model is null");
            }
        }    
    }
    
void Planner::jointPoseAccCB(const nav_msgs::Odometry::ConstPtr & rbtOdomMsg, 
                                const geometry_msgs::TwistStamped::ConstPtr & rbtAccelMsg)
    {
        // ROS_INFO_STREAM("jointPoseAccCB");

        // odom coming in wrt rto/odom frame
        // velocity coming in wrt rto/odom frame

        if (rbtOdomMsg->header.frame_id != cfg_.odom_frame_id)
            ROS_WARN_STREAM("Odom msg header frame (for ego-robot pose) " << rbtOdomMsg->header.frame_id << " not same as cfg odom frame:" << cfg_.odom_frame_id);

        if (rbtOdomMsg->child_frame_id != cfg_.robot_frame_id)
            ROS_WARN_STREAM("Odom msg child frame (for ego-robot velocity) " << rbtOdomMsg->child_frame_id << " not same as cfg rbt frame:" << cfg_.robot_frame_id);
        
        if (rbtAccelMsg->header.frame_id != cfg_.robot_frame_id)
            ROS_WARN_STREAM("Accel msg header frame (for ego-robot acceleration) " << rbtAccelMsg->header.frame_id << " not same as cfg rbt frame:" << cfg_.robot_frame_id);

        // acceleration coming in wrt rto/base_link frame

        // ROS_INFO_STREAM("   rbtOdomMsg: " << *rbtOdomMsg);

        // ROS_INFO_STREAM("   rbtAccelMsg: " << *rbtAccelMsg);


        if (!haveTFs_)
            return;

        // ROS_INFO_STREAM("   tPreviousModelUpdate_: " << tPreviousModelUpdate_);

        // deleting old sensor measurements already used in an update
        for (int i = 0; i < intermediateRbtAccs_.size(); i++)
        {
            // ROS_INFO_STREAM("   intermediateRbtAccs_.at(" << i << "):");
            // ROS_INFO_STREAM("       header.stamp: " << intermediateRbtAccs_.at(i).header.stamp);                     
            if (intermediateRbtAccs_.at(i).header.stamp <= tPreviousModelUpdate_)
            {
                // ROS_INFO_STREAM("           erasing!");
                intermediateRbtAccs_.erase(intermediateRbtAccs_.begin() + i);
                i--;
            }
        }

        currentRbtAcc_ = *rbtAccelMsg;
        if (intermediateRbtAccs_.size() > 0 && (intermediateRbtAccs_.back().header.stamp == currentRbtAcc_.header.stamp))
        {
            // ROS_INFO_STREAM("   redundant timestamp, skipping currentRbtAcc_");
        } else
        {
            // assuming acceleration message comes in wrt robot frame, no transforms
            intermediateRbtAccs_.push_back(currentRbtAcc_);

            // ROS_INFO_STREAM("   pushing back currentRbtAcc_: ");
            // ROS_INFO_STREAM("       header.stamp: " << currentRbtAcc_.header.stamp);
            // ROS_INFO_STREAM("       header.frame_id: " << currentRbtAcc_.header.frame_id);
            // ROS_INFO_STREAM("       linear: ");
            // ROS_INFO_STREAM("           x: " << currentRbtAcc_.twist.linear.x);
            // ROS_INFO_STREAM("           y: " << currentRbtAcc_.twist.linear.y);
        }

        //--------------- POSE -------------------//

        // ROS_INFO_STREAM("odom msg is not in odom frame");

        geometry_msgs::PoseStamped rbtPoseOdomFrame;
        rbtPoseOdomFrame.header = rbtOdomMsg->header;
        rbtPoseOdomFrame.pose = rbtOdomMsg->pose.pose;
        rbtPoseInOdomFrame_ = rbtPoseOdomFrame;

        // ROS_INFO_STREAM("   rbtPoseInOdomFrame_: " << rbtPoseInOdomFrame_);

        //--------------- VELOCITY -------------------//
        // assuming velocity always comes in wrt robot frame
        geometry_msgs::Vector3Stamped rbtVelOdomFrame, rbtVelRbtFrame;
        rbtVelOdomFrame.header = rbtOdomMsg->header; // TODO: make sure this is correct frame
        rbtVelOdomFrame.vector = rbtOdomMsg->twist.twist.linear;

        // transform into robot frame

        geometry_msgs::TransformStamped odomFrame2RbtFrame = tfBuffer_.lookupTransform(rbtOdomMsg->child_frame_id, 
                                                                                        rbtOdomMsg->header.frame_id, 
                                                                                        ros::Time(0));

        // ROS_INFO_STREAM("   rbtVelOdomFrame: " << rbtVelOdomFrame);

        tf2::doTransform(rbtVelOdomFrame, rbtVelRbtFrame, odomFrame2RbtFrame);

        // ROS_INFO_STREAM("   rbtVelRbtFrame: " << rbtVelRbtFrame);

        currentRbtVel_.header = rbtVelRbtFrame.header;
        currentRbtVel_.twist.linear = rbtVelRbtFrame.vector;
        currentRbtVel_.twist.angular = rbtOdomMsg->twist.twist.angular; // z is same between frames

        // deleting old sensor measurements already used in an update
        for (int i = 0; i < intermediateRbtVels_.size(); i++)
        {
            // ROS_INFO_STREAM("   intermediateRbtVels_.at(" << i << "):");
            // ROS_INFO_STREAM("       header.stamp: " << intermediateRbtVels_.at(i).header.stamp);                
            if (intermediateRbtVels_.at(i).header.stamp <= tPreviousModelUpdate_)
            {
                // ROS_INFO_STREAM("           erasing!");
                intermediateRbtVels_.erase(intermediateRbtVels_.begin() + i);
                i--;
            }
        }

        if (intermediateRbtVels_.size() > 0 && (intermediateRbtVels_.back().header.stamp == currentRbtVel_.header.stamp))
        {
            // ROS_INFO_STREAM("   redundant timestamp, skipping currentRbtVel_");
        } else
        {
            intermediateRbtVels_.push_back(currentRbtVel_);

            // ROS_INFO_STREAM("   pushing back currentRbtVel_: ");
            // ROS_INFO_STREAM("       header.stamp: " << currentRbtVel_.header.stamp);
            // ROS_INFO_STREAM("       header.frame_id: " << currentRbtVel_.header.frame_id);
            // ROS_INFO_STREAM("       linear: ");
            // ROS_INFO_STREAM("           x: " << currentRbtVel_.twist.linear.x);
            // ROS_INFO_STREAM("           y: " << currentRbtVel_.twist.linear.y);
        }
    }

    void Planner::pedOdomCB(const pedsim_msgs::AgentStatesConstPtr& pedOdomMsg) 
    {
        // ROS_INFO_STREAM("pedOdomCB()");        
        
        if (!haveTFs_)
            return;

        for (int i = 0; i < pedOdomMsg->agent_states.size(); i++)
        {
            pedsim_msgs::AgentState agentIState = pedOdomMsg->agent_states[i];
            std::string source_frame = agentIState.header.frame_id; 

            // transforming Odometry message from map_static to robotN
            geometry_msgs::TransformStamped msgFrame2RobotFrame = tfBuffer_.lookupTransform(cfg_.robot_frame_id, 
                                                                                            source_frame, 
                                                                                            ros::Time(0));
            geometry_msgs::PoseStamped agentPoseMsgFrame, agentPoseRobotFrame;
            geometry_msgs::Vector3Stamped agentVelMsgFrame, agentVelRobotFrame;

            agentPoseMsgFrame.header = agentIState.header;
            agentPoseMsgFrame.pose = agentIState.pose;
            // ROS_INFO_STREAM_NAMED("Planner", "      incoming pose: (" << agentPoseMsgFrame.pose.position.x << ", " << agentPoseMsgFrame.pose.position.y << ")");

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;
            tf2::doTransform(agentPoseMsgFrame, agentPoseRobotFrame, msgFrame2RobotFrame);
            
            // ROS_INFO_STREAM_NAMED("Planner", "      outgoing pose: (" << agentPoseRobotFrame.pose.position.x << ", " << agentPoseRobotFrame.pose.position.y << ")");

            // ROS_INFO_STREAM("updating " << agentNamespace << " odom from " << agent_odom_vects.at(agentID)[0] << ", " << agent_odom_vects.at(agentID)[1] << " to " << odom_vect[0] << ", " << odom_vect[1]);
            currentTrueAgentPoses_[agentIState.id] = agentPoseRobotFrame.pose;

            // std::cout << "in agentOdomCB" << std::endl;
            // std::cout << "transforming from " << source_frame << " to " << cfg_.robot_frame_id << std::endl;
            agentVelMsgFrame.header = agentIState.header;
            // agentVelMsgFrame.header.frame_id = source_frame; // TODO: determine if frame for position is same as frame for velocity
            agentVelMsgFrame.vector = agentIState.twist.linear;
            // std::cout << "incoming vector: " << agentVelMsgFrame.vector.x << ", " << agentVelMsgFrame.vector.y << std::endl;
            tf2::doTransform(agentVelMsgFrame, agentVelRobotFrame, msgFrame2RobotFrame);
            // std::cout << "outcoming vector: " << agentVelRobotFrame.vector.x << ", " << agentVelRobotFrame.vector.y << std::endl;

            currentTrueAgentVels_[agentIState.id] = agentVelRobotFrame;
        
        }
    }

    bool Planner::setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        if (globalPlanMapFrame.size() == 0) 
            return true;
        
        if (!haveTFs_)
            return false;

        geometry_msgs::PoseStamped globalGoalMapFrame = *std::prev(globalPlanMapFrame.end());

        ROS_INFO_STREAM("globalGoalMapFrame: " << globalGoalMapFrame);

        tf2::doTransform(globalGoalMapFrame, globalGoalOdomFrame_, map2odom_); // to update odom frame parameter

        ROS_INFO_STREAM("globalGoalOdomFrame_: " << globalGoalOdomFrame_);

        tf2::doTransform(globalGoalOdomFrame_, globalGoalRobotFrame_, odom2rbt_); // to update robot frame parameter

        ROS_INFO_STREAM("globalGoalOdomFrame_: " << globalGoalRobotFrame_);

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
        trajEvaluator_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame_, odom2rbt_);

        // Visualization only
        std::vector<geometry_msgs::PoseStamped> visibleGlobalPlanSnippetRobotFrame = globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
        // trajVisualizer_->drawRelevantGlobalPlanSnippet(visibleGlobalPlanSnippetRobotFrame);

        hasGlobalGoal_ = true;
        setReachedGlobalGoal(false);

        return true;
    }

    void Planner::tfCB(const tf2_msgs::TFMessage& msg)
    {
        // ignoring message entirely, just making separate thread for tfs to not
        // tether to other callbacks

        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.robot_frame_id: " << cfg_.robot_frame_id);        
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.map_frame_id: " << cfg_.map_frame_id);        
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.odom_frame_id: " << cfg_.odom_frame_id);        
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.sensor_frame_id: " << cfg_.sensor_frame_id);        

        // for lookupTransform, parameters are (destination frame, source frame)
        map2rbt_  = tfBuffer_.lookupTransform(cfg_.robot_frame_id, cfg_.map_frame_id, ros::Time(0));
        odom2rbt_ = tfBuffer_.lookupTransform(cfg_.robot_frame_id, cfg_.odom_frame_id, ros::Time(0));
        rbt2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.robot_frame_id, ros::Time(0));
        map2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.map_frame_id, ros::Time(0));
        // cam2odom_ = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.sensor_frame_id, ros::Time(0));
        // rbt2cam_ = tfBuffer_.lookupTransform(cfg_.sensor_frame_id, cfg_.robot_frame_id, ros::Time(0));

        haveTFs_ = true;

        // tf2::doTransform(rbtPoseInRbtFrame_, rbtPoseInSensorFrame_, rbt2cam_);

        // ROS_INFO_STREAM("tfCB succeeded");
    }

    void Planner::updateEgoCircle()
    {
        globalPlanManager_->updateEgoCircle(scan_);
        gapManipulator_->updateEgoCircle(scan_);
        gapGoalPlacer_->updateEgoCircle(scan_);
        gapFeasibilityChecker_->updateEgoCircle(scan_);
        ungapFeasibilityChecker_->updateEgoCircle(scan_);
        dynamicScanPropagator_->updateEgoCircle(scan_);
        trajEvaluator_->updateEgoCircle(scan_);
        trajController_->updateEgoCircle(scan_);
    }

    void Planner::propagateGapPointsV2(const std::vector<Gap *> & planningGaps,
                                        std::vector<GapTube *> & gapTubes)
    {
        boost::mutex::scoped_lock gapset(gapMutex_);        
        ROS_INFO_STREAM_NAMED("Planner", "[propagateGapPointsV2()]");

        if (planningGaps.empty())
        {
            ROS_WARN_STREAM_NAMED("Planner", "    planningGaps is empty, returning");
            return;
        }

        // ROS_INFO_STREAM_NAMED("Planner", "    current raw gaps:");
        // printGapModels(currRawGaps_);
        checkGapModels(currRawGaps_);

        // ROS_INFO_STREAM_NAMED("Planner", "    current simplified gaps:");
        // printGapModels(planningGaps);
        checkGapModels(planningGaps);

        gapPropagator_->propagateGapPointsV2(planningGaps, gapTubes);

        return;
    }

    void Planner::propagateGapPoints(const std::vector<Gap *> & planningGaps)                                             
    {
        boost::mutex::scoped_lock gapset(gapMutex_);        
        ROS_INFO_STREAM_NAMED("GapPropagator", "[propagateGapPoints()]");

        if (planningGaps.empty())
        {
            ROS_WARN_STREAM_NAMED("GapPropagator", "    planningGaps is empty, returning");
            return;
        }

        ROS_INFO_STREAM_NAMED("GapPropagator", "    current raw gaps:");
        printGapModels(currRawGaps_);
        checkGapModels(currRawGaps_);

        ROS_INFO_STREAM_NAMED("GapPropagator", "    current simplified gaps:");
        printGapModels(planningGaps);
        checkGapModels(planningGaps);

        for (size_t i = 0; i < planningGaps.size(); i++) 
        {
            ROS_INFO_STREAM_NAMED("GapPropagator", "   gap " << i);
            // propagate gap forward in time to determine lifespan
            gapPropagator_->propagateGapPoints(planningGaps.at(i));
        }

        return;

    }

    std::vector<Gap *> Planner::manipulateGaps(const std::vector<Gap *> & planningGaps) 
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "[manipulateGaps()]");

        boost::mutex::scoped_lock gapset(gapMutex_);
        std::vector<Gap *> manipulatedGaps;

        for (size_t i = 0; i < planningGaps.size(); i++)
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "    manipulating initial gap " << i);

            // MANIPULATE POINTS AT T=0            
            gapManipulator_->convertRadialGap(planningGaps.at(i));

            bool success = gapManipulator_->inflateGapSides(planningGaps.at(i));
            
            bool valid = planningGaps.at(i)->checkPoints();

            if (!valid)
            {
                ROS_WARN_STREAM_NAMED("GapManipulator", "    invalid gap after manipulation " << i);
            }

            if (success)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "    pushing back manipulated gap " << i);
                
                gapGoalPlacer_->setGapGoal(planningGaps.at(i), 
                                            globalPlanManager_->getGlobalPathLocalWaypointRobotFrame(),
                                            globalGoalRobotFrame_);                    

                manipulatedGaps.push_back(planningGaps.at(i)); // shallow copy
            }
        }

        return manipulatedGaps;
    }

    void Planner::gapGoalPlacementV2(std::vector<GapTube *> & gapTubes) 
    {
        ROS_INFO_STREAM_NAMED("Planner", "[gapGoalPlacementV2()]");

        for (int i = 0; i < gapTubes.size(); i++)
        {
            ROS_INFO_STREAM_NAMED("Planner", "    tube " << i);

            GapTube * gapTube = gapTubes.at(i);
            for (int j = 0; j < gapTube->size(); j++)
            {
                ROS_INFO_STREAM_NAMED("Planner", "       gap " << j);

                Gap * gap = gapTube->at(j);

                if (j < (gapTube->size() - 1) && !gapTube->at(j+1)->isAvailable()) // if next gap is not available
                {
                    ROS_INFO_STREAM_NAMED("Planner", "          next gap unavailable, placing goal inside");

                    Gap * nextGap = gapTube->at(j+1);

                    // place goal inside gap
                    gapGoalPlacer_->setGapGoalFromNextV2(gap, nextGap);   

                } else if (!gap->isAvailable()) // if curr gap is not available (should mean that previous gap existed)
                {
                    ROS_INFO_STREAM_NAMED("Planner", "          current gap unavailable, placing goal inside");

                    if ( (j-1) < 0)
                    {
                        ROS_WARN_STREAM_NAMED("Planner", "       gap " << j << " is not available and no previous gap exists");
                        continue;
                    }

                    Gap * priorGap = gapTube->at(j-1);
                    // place goal inside gap

                    gapGoalPlacer_->setGapGoalFromPriorV2(gap, priorGap);

                } else
                {
                    ROS_INFO_STREAM_NAMED("Planner", "          current gap available, placing goal beyond");

                    // place goal beyond gap
                    // should use manipulate points to set gaps
                    gapGoalPlacer_->setGapGoalV2(gap, 
                        globalPlanManager_->getGlobalPathLocalWaypointRobotFrame(),
                        globalGoalRobotFrame_);                    
                }
            }
        }
        return;
    }

    void Planner::ungapGoalPlacement(const std::vector<Ungap *> & recedingUngaps)
    {

        for (Ungap * ungap : recedingUngaps)
        {
            // Use original points, not manipulated
            ungap->getLeftUngapPt()->getModel()->isolateGapDynamics();
            ungap->getRightUngapPt()->getModel()->isolateGapDynamics();

            Eigen::Vector4f leftState = ungap->getLeftUngapPt()->getModel()->getGapState();
            Eigen::Vector4f rightState = ungap->getRightUngapPt()->getModel()->getGapState();

            // calculate avg point and diameter
            Eigen::Vector2f leftPos = leftState.head(2);
            Eigen::Vector2f rightPos = rightState.head(2);
            Eigen::Vector2f avgPos = 0.5 * (leftPos + rightPos);
            float diameter = (rightPos - leftPos).norm();
            float radius = 0.5 * diameter;

            Eigen::Vector2f leftVel = leftState.tail(2);
            Eigen::Vector2f rightVel = rightState.tail(2);
            Eigen::Vector2f avgVel = 0.5 * (leftVel + rightVel);

            // Get worst case point using radius
            Eigen::Vector2f ungapRadialOffset = avgPos.normalized() * radius;
            Eigen::Vector2f worstCaseUngapPt = avgPos - ungapRadialOffset;

            // Inflate inwards by radius
            Eigen::Vector2f gapGoalRadialOffset = 2 * avgPos.normalized() * cfg_.rbt.r_inscr * cfg_.traj.inf_ratio;
            Eigen::Vector2f inflatedCenterPt = worstCaseUngapPt - gapGoalRadialOffset;

            // set
            ungap->getGoal()->setOrigGoalPos(inflatedCenterPt);
            ungap->getGoal()->setOrigGoalVel(avgVel);   
        }

        return;
    }

    std::vector<Gap *> Planner::gapSetFeasibilityCheck(const std::vector<Gap *> & manipulatedGaps, 
                                                        bool & isCurrentGapFeasible)                                             
    {
        boost::mutex::scoped_lock gapset(gapMutex_);        
        ROS_INFO_STREAM_NAMED("Planner", "[gapSetFeasibilityCheck()]");
        std::vector<Gap *> feasibleGaps;

        // grabbing the current set of gaps

        int currentLeftGapPtModelID = getCurrentLeftGapPtModelID();
        int currentRightGapPtModelID = getCurrentRightGapPtModelID();
        ROS_INFO_STREAM_NAMED("Planner", "    current left/right model IDs: " << currentLeftGapPtModelID << ", " << currentRightGapPtModelID);

        isCurrentGapFeasible = false;

        bool isGapFeasible = false;
        for (size_t i = 0; i < manipulatedGaps.size(); i++) 
        {
            ROS_INFO_STREAM_NAMED("Planner", "    feasibility check for gap " << i);

            // run pursuit guidance analysis on gap to determine feasibility
            isGapFeasible = gapFeasibilityChecker_->pursuitGuidanceAnalysis(manipulatedGaps.at(i));

            if (isGapFeasible) 
            {                    
                feasibleGaps.push_back(manipulatedGaps.at(i)); // shallow copy
            
                if (manipulatedGaps.at(i)->getLeftGapPt()->getModel()->getID() == currentLeftGapPtModelID && 
                    manipulatedGaps.at(i)->getRightGapPt()->getModel()->getID() == currentRightGapPtModelID) 
                {
                    isCurrentGapFeasible = true;
                }
            }
        }
            
        return feasibleGaps;
    }

    void Planner::ungapSetFeasibilityCheck(const std::vector<Ungap *> & recedingUngaps)
    {
        for (size_t i = 0; i < recedingUngaps.size(); i++) 
        {
            ROS_INFO_STREAM_NAMED("Planner", "    feasibility check for ungap " << i);

            // run pursuit guidance analysis on ungap to calculate gamma (feasibility not really a thing here)
            ungapFeasibilityChecker_->pursuitGuidanceAnalysis(recedingUngaps.at(i));

        }

        return;
    }       

    std::vector<GapTube *> Planner::gapSetFeasibilityCheckV2(const std::vector<GapTube *> & gapTubes, 
                                                                bool & isCurrentGapFeasible)                                             
    {
        ROS_INFO_STREAM_NAMED("Planner", "[gapSetFeasibilityCheckV2()]");

        std::vector<GapTube *> feasibleGapTubes;

        // boost::mutex::scoped_lock gapset(gapMutex_);        
        // std::vector<GapTube *> feasibleGaps;

        // grabbing the current set of gaps

        int currentLeftGapPtModelID = getCurrentLeftGapPtModelID();
        int currentRightGapPtModelID = getCurrentRightGapPtModelID();
        ROS_INFO_STREAM_NAMED("Planner", "    current left/right model IDs: " << currentLeftGapPtModelID << ", " << currentRightGapPtModelID);

        isCurrentGapFeasible = false;

        bool isGapFeasible = false;

        for (int i = 0; i < gapTubes.size(); i++)
        {
            GapTube * gapTube = gapTubes.at(i);
            bool isTubeFeasible = true;
            ROS_INFO_STREAM_NAMED("Planner", "    feasibility check for tube " << i);

            Eigen::Vector2f startPt(0.0, 0.0);

            for (int j = 0; j < gapTube->size(); j++) 
            {
                bool isGapFeasible = false;

                Gap * gap = gapTube->at(j);
                ROS_INFO_STREAM_NAMED("Planner", "       gap " << j);
    
                // run pursuit guidance analysis on gap to determine feasibility
                isGapFeasible = gapFeasibilityChecker_->pursuitGuidanceAnalysisV2(gap, startPt);

                if (gap->getLeftGapPt()->getModel()->getID() == currentLeftGapPtModelID && 
                    gap->getRightGapPt()->getModel()->getID() == currentRightGapPtModelID) 
                {
                    isCurrentGapFeasible = true;
                }

                if (!isGapFeasible)
                {
                    isTubeFeasible = false;
                    break;
                } else
                {
                    float gammaIntercept = gap->getGammaInterceptGoal();
                    Eigen::Vector2f trajDir(cos(gammaIntercept), sin(gammaIntercept));
                    startPt = gap->getGapLifespan() * trajDir; 
                    // startPt = gap->getGoal()->getTermGoalPos();
                }
            }
    
            if (isTubeFeasible) 
            {                    
                feasibleGapTubes.push_back(gapTube); // shallow copy
            }        

        }

        return feasibleGapTubes;
    }

    void Planner::generateGapTrajsV2(std::vector<GapTube *> & gapTubes, 
                                        std::vector<Trajectory> & gapTubeTrajs,
                                        std::vector<std::vector<float>> & gapTubeTrajPoseCosts,
                                        std::vector<float> & gapTubeTrajTerminalPoseCosts,
                                        const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "[generateGapTrajsV2()]");
        
        gapTubeTrajPoseCosts = std::vector<std::vector<float>>(gapTubes.size());
        gapTubeTrajTerminalPoseCosts = std::vector<float>(gapTubes.size());

        for (int i = 0; i < gapTubes.size(); i++)
        {
            GapTube * gapTube = gapTubes.at(i);
            bool isTubeFeasible = true;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generating traj for tube " << i);

            geometry_msgs::PoseStamped currPose = rbtPoseInSensorFrame_;
            geometry_msgs::TwistStamped currVel = currentRbtVel_;

            Trajectory runningTraj;
            std::vector<float> runningPoseCosts;
            float runningTerminalPoseCost;

            for (int j = 0; j < gapTube->size(); j++) 
            {
                Trajectory traj; 
                Trajectory goToGoalTraj, pursuitGuidanceTraj;
                
                std::vector<float> poseCosts;
                float terminalPoseCost;

                std::vector<float> goToGoalPoseCosts, pursuitGuidancePoseCosts;
                float goToGoalTerminalPoseCost, pursuitGuidanceTerminalPoseCost;
                float goToGoalCost, pursuitGuidancePoseCost;

                Gap * gap = gapTube->at(j);

                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "         gap: " << j);
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                
                // Run go to goal behavior
                bool runGoToGoal = gap->getGlobalGoalWithin();

                if (runGoToGoal) 
                {
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running goToGoal");
    
                    goToGoalTraj = gapTrajGenerator_->generateGoToGoalTrajectoryV2(gap, 
                                                                                    currPose, 
                                                                                    currVel, 
                                                                                    globalGoalRobotFrame_);
                    goToGoalTraj = gapTrajGenerator_->processTrajectory(goToGoalTraj, 
                                                                        currPose,
                                                                        currVel,
                                                                        false);
                    trajEvaluator_->evaluateTrajectory(goToGoalTraj, goToGoalPoseCosts, goToGoalTerminalPoseCost, futureScans);
                    goToGoalCost = goToGoalTerminalPoseCost + std::accumulate(goToGoalPoseCosts.begin(), goToGoalPoseCosts.end(), float(0));
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        goToGoalCost: " << goToGoalCost);
                }
    
                // Run pursuit guidance behavior
                if (gap->isAvailable())
                {
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running pursuit guidance (available)");
                    pursuitGuidanceTraj = gapTrajGenerator_->generateTrajectoryV2(gap, 
                                                                                    currPose, 
                                                                                    currVel, 
                                                                                    globalGoalRobotFrame_);
                    pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj, 
                                                                                currPose,
                                                                                currVel,
                                                                                false);
                    trajEvaluator_->evaluateTrajectory(pursuitGuidanceTraj, pursuitGuidancePoseCosts, pursuitGuidanceTerminalPoseCost, futureScans);
                    pursuitGuidancePoseCost = pursuitGuidanceTerminalPoseCost + std::accumulate(pursuitGuidancePoseCosts.begin(), pursuitGuidancePoseCosts.end(), float(0));
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        pursuitGuidancePoseCost: " << pursuitGuidancePoseCost);
                } else
                {
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running pursuit guidance (not available)");
                    pursuitGuidanceTraj = gapTrajGenerator_->generateIdlingTrajectoryV2(gap, 
                                                                                        currPose, 
                                                                                        rbtPoseInOdomFrame_);
                    trajEvaluator_->evaluateTrajectory(pursuitGuidanceTraj, pursuitGuidancePoseCosts, pursuitGuidanceTerminalPoseCost, futureScans);
                    pursuitGuidancePoseCost = pursuitGuidanceTerminalPoseCost + std::accumulate(pursuitGuidancePoseCosts.begin(), pursuitGuidancePoseCosts.end(), float(0));
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        pursuitGuidancePoseCost: " << pursuitGuidancePoseCost);                
                
                }

                if (runGoToGoal && goToGoalCost < pursuitGuidancePoseCost)
                {
                    traj = goToGoalTraj;
                    poseCosts = goToGoalPoseCosts;
                    terminalPoseCost = goToGoalTerminalPoseCost;
                    // gapTubeTrajPoseCosts.at(i) = goToGoalPoseCosts;
                    // gapTubeTrajTerminalPoseCosts.at(i) = goToGoalTerminalPoseCost;
                } else
                {
                    traj = pursuitGuidanceTraj;
                    poseCosts = pursuitGuidancePoseCosts;
                    terminalPoseCost = pursuitGuidanceTerminalPoseCost;
                    // gapTubeTrajPoseCosts.at(i) = pursuitGuidancePoseCosts;
                    // gapTubeTrajTerminalPoseCosts.at(i) = pursuitGuidanceTerminalPoseCost;
                }
    
                // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                traj.setPathOdomFrame(gapTrajGenerator_->transformPath(traj.getPathRbtFrame(), rbt2odom_));

                // update current pose and time
                currPose.pose = traj.getPathRbtFrame().poses.back();

                float trajDuration = traj.getPathTiming().back() - traj.getPathTiming().front();
                currPose.header.stamp = traj.getPathRbtFrame().header.stamp += ros::Duration(trajDuration);

                // append traj
                if (j == 0)
                {
                    runningTraj = traj;
                    runningPoseCosts = poseCosts;
                    runningTerminalPoseCost = terminalPoseCost;
                } else
                {
                    runningTraj.appendTraj(traj);
                    // append costs
                    runningPoseCosts.insert(runningPoseCosts.end(), poseCosts.begin(), poseCosts.end());
                    runningTerminalPoseCost = terminalPoseCost; //  += terminalPoseCost;
                }
            }

            gapTubeTrajs.push_back(runningTraj);
            gapTubeTrajPoseCosts.at(i) = runningPoseCosts;
            gapTubeTrajTerminalPoseCosts.at(i) = runningTerminalPoseCost;     
            
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    trajectory: ");
            geometry_msgs::Pose pose;
            float time;
            float cost;
            for (int k = 0; k < runningTraj.getPathRbtFrame().poses.size(); k++)
            {
                pose = runningTraj.getPathRbtFrame().poses.at(k);
                time = runningTraj.getPathTiming().at(k);
                cost = runningPoseCosts.at(k);
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        t: " << time << ", p: (" << pose.position.x << ", " << pose.position.y << "), cost: " << cost);
            }
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    terminal cost: " << runningTerminalPoseCost);
        }        

        // trajVisualizer_->drawGapTrajectoryPoseScores(gapTubeTrajs, gapTubeTrajPoseCosts);
        trajVisualizer_->drawGapTubeTrajectories(gapTubeTrajs);

        return;
    }

    void Planner::generateGapTrajs(std::vector<Gap *> & gaps, 
                                    std::vector<Trajectory> & gapTrajs,
                                    std::vector<std::vector<float>> & gapTrajPoseCosts,
                                    std::vector<float> & gapTrajTerminalPoseCosts,
                                    const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateGapTrajs()]");
        
        gapTrajPoseCosts = std::vector<std::vector<float>>(gaps.size());
        gapTrajTerminalPoseCosts = std::vector<float>(gaps.size());

        for (size_t i = 0; i < gaps.size(); i++) 
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    generating traj for gap: " << i);
            // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
            
            // Run go to goal behavior
            bool runGoToGoal = gaps.at(i)->getGlobalGoalWithin();

            Trajectory traj, goToGoalTraj, pursuitGuidanceTraj;
            std::vector<float> goToGoalPoseCosts, pursuitGuidancePoseCosts;
            float goToGoalTerminalPoseCost, pursuitGuidanceTerminalPoseCost;
            float goToGoalCost, pursuitGuidancePoseCost;

            if (runGoToGoal) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        running goToGoal");

                goToGoalTraj = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbtPoseInSensorFrame_, 
                                                                        currentRbtVel_, 
                                                                        globalGoalRobotFrame_,
                                                                        true);
                goToGoalTraj = gapTrajGenerator_->processTrajectory(goToGoalTraj, 
                                                                    rbtPoseInSensorFrame_,
                                                                    currentRbtVel_,
                                                                    true);
                trajEvaluator_->evaluateTrajectory(goToGoalTraj, goToGoalPoseCosts, goToGoalTerminalPoseCost, futureScans);
                goToGoalCost = goToGoalTerminalPoseCost + std::accumulate(goToGoalPoseCosts.begin(), goToGoalPoseCosts.end(), float(0));
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        goToGoalCost: " << goToGoalCost);
            }

            // Run pursuit guidance behavior
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        running pursuit guidance");
            pursuitGuidanceTraj = gapTrajGenerator_->generateTrajectory(gaps.at(i), rbtPoseInSensorFrame_, 
                                                                            currentRbtVel_, 
                                                                            globalGoalRobotFrame_,
                                                                            false);

            pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj, 
                                                                        rbtPoseInSensorFrame_,
                                                                        currentRbtVel_,
                                                                        true);
            trajEvaluator_->evaluateTrajectory(pursuitGuidanceTraj, pursuitGuidancePoseCosts, pursuitGuidanceTerminalPoseCost, futureScans);
            pursuitGuidancePoseCost = pursuitGuidanceTerminalPoseCost + std::accumulate(pursuitGuidancePoseCosts.begin(), pursuitGuidancePoseCosts.end(), float(0));
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        pursuitGuidancePoseCost: " << pursuitGuidancePoseCost);

            if (runGoToGoal && goToGoalCost < pursuitGuidancePoseCost)
            {
                traj = goToGoalTraj;
                gapTrajPoseCosts.at(i) = goToGoalPoseCosts;
                gapTrajTerminalPoseCosts.at(i) = goToGoalTerminalPoseCost;
            } else
            {
                traj = pursuitGuidanceTraj;
                gapTrajPoseCosts.at(i) = pursuitGuidancePoseCosts;
                gapTrajTerminalPoseCosts.at(i) = pursuitGuidanceTerminalPoseCost;
            }

            // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
            traj.setPathOdomFrame(gapTrajGenerator_->transformPath(traj.getPathRbtFrame(), rbt2odom_));
            gapTrajs.push_back(traj);
        }    

        trajVisualizer_->drawGapTrajectoryPoseScores(gapTrajs, gapTrajPoseCosts);
        trajVisualizer_->drawGapTrajectories(gapTrajs);

        return;
    }

    void Planner::generateUngapTrajs(std::vector<Ungap *> & ungaps, 
                                        std::vector<Trajectory> & ungapTrajs,
                                        std::vector<std::vector<float>> & ungapTrajPoseCosts,
                                        std::vector<float> & ungapTrajTerminalPoseCosts,
                                        const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateUngapTrajs()]");
        
        ungapTrajPoseCosts = std::vector<std::vector<float>>(ungaps.size());
        ungapTrajTerminalPoseCosts = std::vector<float>(ungaps.size());

        for (size_t i = 0; i < ungaps.size(); i++) 
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    generating traj for un-gap: " << i);
            // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
            
            // Run go to goal behavior

            Trajectory traj, pursuitGuidanceTraj;
            std::vector<float> pursuitGuidancePoseCosts;
            float pursuitGuidanceTerminalPoseCost;
            float pursuitGuidancePoseCost;

            // Run pursuit guidance behavior
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        running pursuit guidance");
            pursuitGuidanceTraj = ungapTrajGenerator_->generateTrajectory(ungaps.at(i), rbtPoseInSensorFrame_, 
                                                                            currentRbtVel_);

            pursuitGuidanceTraj = ungapTrajGenerator_->processTrajectory(pursuitGuidanceTraj, 
                                                                            rbtPoseInSensorFrame_,
                                                                            currentRbtVel_,
                                                                            true);
            trajEvaluator_->evaluateTrajectory(pursuitGuidanceTraj, 
                                                pursuitGuidancePoseCosts, 
                                                pursuitGuidanceTerminalPoseCost, 
                                                futureScans);
            pursuitGuidancePoseCost = pursuitGuidanceTerminalPoseCost + std::accumulate(pursuitGuidancePoseCosts.begin(), pursuitGuidancePoseCosts.end(), float(0));
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        pursuitGuidancePoseCost: " << pursuitGuidancePoseCost);

            traj = pursuitGuidanceTraj;
            ungapTrajPoseCosts.at(i) = pursuitGuidancePoseCosts;
            ungapTrajTerminalPoseCosts.at(i) = pursuitGuidanceTerminalPoseCost;

            // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
            traj.setPathOdomFrame(ungapTrajGenerator_->transformPath(traj.getPathRbtFrame(), rbt2odom_));
            ungapTrajs.push_back(traj);
        }

        // trajVisualizer_->drawGapTrajectoryPoseScores(ungapTrajs, ungapTrajPoseCosts);
        trajVisualizer_->drawUngapTrajectories(ungapTrajs);        

        return;
    }

    void Planner::generateIdlingTraj(std::vector<Trajectory> & idlingTrajs,
                                        std::vector<std::vector<float>> & idlingTrajPoseCosts,
                                        std::vector<float> & idlingTrajTerminalPoseCosts,
                                        const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateIdlingTraj()]");

        // push back idling trajectory as another option
        std::vector<float> idlingPoseCosts;
        float idlingTerminalPoseCost, idlingCost;

        Trajectory idlingTrajectory;
        idlingTrajectory = gapTrajGenerator_->generateIdlingTrajectory(rbtPoseInOdomFrame_);
        
        trajEvaluator_->evaluateTrajectory(idlingTrajectory, idlingPoseCosts, idlingTerminalPoseCost, futureScans);
        idlingCost = idlingTerminalPoseCost + std::accumulate(idlingPoseCosts.begin(), idlingPoseCosts.end(), float(0));
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        idlingCost: " << idlingCost);

        idlingTrajPoseCosts.push_back(idlingPoseCosts);
        idlingTrajTerminalPoseCosts.push_back(idlingTerminalPoseCost);

        idlingTrajectory.setPathOdomFrame(gapTrajGenerator_->transformPath(idlingTrajectory.getPathRbtFrame(), rbt2odom_));
        idlingTrajs.push_back(idlingTrajectory);        

        return;
    }

    void Planner::pickTraj(int & trajFlag,
                            int & lowestCostTrajIdx,
                            const std::vector<Trajectory> & gapTrajs, 
                            const std::vector<std::vector<float>> & gapTrajPoseCosts, 
                            const std::vector<float> & gapTrajTerminalPoseCosts,
                            const std::vector<Trajectory> & ungapTrajs, 
                            const std::vector<std::vector<float>> & ungapTrajPoseCosts, 
                            const std::vector<float> & ungapTrajTerminalPoseCosts,                          
                            const std::vector<Trajectory> & idlingTrajs, 
                            const std::vector<std::vector<float>> & idlingTrajPoseCosts, 
                            const std::vector<float> & idlingTrajTerminalPoseCosts)
                           
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("Planner", "[pickTraj()]");
        
        // int lowestCostTrajIdx = -1;        
        
        // ROS_INFO_STREAM_NAMED("pg_trajCount", "pg_trajCount, " << paths.size());
        if (gapTrajs.size() == 0)
        {
            ROS_WARN_STREAM_NAMED("Planner", "No traj synthesized");
            trajFlag = NONE;
            return;
        }

        if (gapTrajs.size() != gapTrajPoseCosts.size() ||
            gapTrajs.size() != gapTrajTerminalPoseCosts.size())
        {
            ROS_WARN_STREAM_NAMED("Planner", "pickTraj size mismatch: paths = " << gapTrajs.size() << 
                                                " != gapTrajPoseCosts =" << gapTrajPoseCosts.size());
            trajFlag = NONE;
            return;
        }

        // poses here are in odom frame 
        int numGapTrajs = gapTrajs.size();
        int numUngapTrajs = ungapTrajs.size();
        int numIdlingTrajs = idlingTrajs.size();
        std::vector<float> pathCosts(numGapTrajs + numUngapTrajs + numIdlingTrajs);

        // evaluate gap trajectories
        int runningTrajIdx = 0;
        for (size_t i = 0; i < gapTrajs.size(); i++) 
        {
            // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());

            pathCosts.at(runningTrajIdx) = gapTrajTerminalPoseCosts.at(i) + std::accumulate(gapTrajPoseCosts.at(i).begin(), 
                                                                                gapTrajPoseCosts.at(i).end(), float(0));
            
            pathCosts.at(runningTrajIdx) = gapTrajs.at(i).getPathRbtFrame().poses.size() == 0 ? std::numeric_limits<float>::infinity() : 
                                                                                                pathCosts.at(runningTrajIdx);
            
            ROS_INFO_STREAM_NAMED("Planner", "    for gap " << i << " (length: " << gapTrajs.at(i).getPathRbtFrame().poses.size() << "), returning cost of " << pathCosts.at(runningTrajIdx));
        
            runningTrajIdx++;
        }

        // evaluate ungap trajectories
        for (size_t i = 0; i < ungapTrajs.size(); i++) 
        {
            // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());

            pathCosts.at(runningTrajIdx) = ungapTrajTerminalPoseCosts.at(i) + std::accumulate(ungapTrajPoseCosts.at(i).begin(), 
                                                                                                ungapTrajPoseCosts.at(i).end(), float(0));
            
            pathCosts.at(runningTrajIdx) = ungapTrajs.at(i).getPathRbtFrame().poses.size() == 0 ? std::numeric_limits<float>::infinity() : 
                                                                                                    pathCosts.at(runningTrajIdx);
            
            ROS_INFO_STREAM_NAMED("Planner", "    for ungap " << i << " (length: " << ungapTrajs.at(i).getPathRbtFrame().poses.size() << "), returning cost of " << pathCosts.at(runningTrajIdx));
        
            runningTrajIdx++;
        }

        // evaluate idling trajectory
        for (size_t i = 0; i < idlingTrajs.size(); i++) 
        {
            // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());

            pathCosts.at(runningTrajIdx) = idlingTrajTerminalPoseCosts.at(i) + std::accumulate(idlingTrajPoseCosts.at(i).begin(), 
                                                                                                idlingTrajPoseCosts.at(i).end(), float(0));
            
            pathCosts.at(runningTrajIdx) = idlingTrajs.at(i).getPathRbtFrame().poses.size() == 0 ? std::numeric_limits<float>::infinity() : 
                                                                                                    pathCosts.at(runningTrajIdx);
            
            ROS_INFO_STREAM_NAMED("Planner", "    for idling " << i << " (length: " << idlingTrajs.at(i).getPathRbtFrame().poses.size() << "), returning cost of " << pathCosts.at(runningTrajIdx));
        
            runningTrajIdx++;
        }

        // selecting best trajectory
        auto lowestCostTrajIterIter = std::min_element(pathCosts.begin(), pathCosts.end());
        int candidateLowestCostTrajIdx = std::distance(pathCosts.begin(), lowestCostTrajIterIter);

        if (pathCosts.at(candidateLowestCostTrajIdx) == std::numeric_limits<float>::infinity()) 
        {    
            ROS_INFO_STREAM_NAMED("Planner", "    all infinity");
            ROS_INFO_STREAM_NAMED("Planner", "No executable trajectory, values: ");
            for (float pathCost : pathCosts) 
            {
                ROS_INFO_STREAM_NAMED("Planner", "Cost: " << pathCost);
            }
            ROS_INFO_STREAM_NAMED("Planner", "------------------");

            trajFlag = NONE;
            // do not update candidateLowestCostTrajIdx
            return;
        }

        if (candidateLowestCostTrajIdx >= 0 && candidateLowestCostTrajIdx < numGapTrajs)
        {
            lowestCostTrajIdx = candidateLowestCostTrajIdx;
            trajFlag = GAP;
            ROS_INFO_STREAM_NAMED("Planner", "    picking gap traj: " << lowestCostTrajIdx);
        } else if (candidateLowestCostTrajIdx >= numGapTrajs && candidateLowestCostTrajIdx < (numGapTrajs + numUngapTrajs))
        {
            lowestCostTrajIdx = candidateLowestCostTrajIdx - numGapTrajs;
            trajFlag = UNGAP;
            ROS_INFO_STREAM_NAMED("Planner", "    picking ungap traj: " << lowestCostTrajIdx);
        } else if (candidateLowestCostTrajIdx >= (numGapTrajs + numUngapTrajs) && candidateLowestCostTrajIdx < (numGapTrajs + numUngapTrajs + numIdlingTrajs))
        {
            lowestCostTrajIdx = candidateLowestCostTrajIdx - (numGapTrajs + numUngapTrajs);
            trajFlag = IDLING;
            ROS_INFO_STREAM_NAMED("Planner", "    picking idling traj: " << lowestCostTrajIdx);
        } else
        {
            ROS_WARN_STREAM_NAMED("Planner", "    trajFlag not set");
        }

        return;
    }

    Trajectory Planner::changeTrajectoryHelper(Gap * incomingGap, 
                                                const Trajectory & incomingTraj, 
                                                const bool & switchToIncoming) 
    {
        // publishToMpc_ = true;
        
        if (switchToIncoming) 
        {
            setCurrentTraj(incomingTraj);
            if (incomingGap) // actual trajectory
            {
                setCurrentLeftGapPtModelID(incomingGap->getLeftGapPt()->getModel());
                setCurrentRightGapPtModelID(incomingGap->getRightGapPt()->getModel());
            } else // idling trajectory
            {
                setCurrentLeftGapPtModelID(nullptr);
                setCurrentRightGapPtModelID(nullptr);
            }

            trajVisualizer_->drawCurrentTrajectory(incomingTraj);
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, incomingTraj);
            trajectoryChangeCount_++;

            return incomingTraj;  
        } else 
        {
            geometry_msgs::PoseArray emptyPath = geometry_msgs::PoseArray();
            emptyPath.header = incomingTraj.getPathRbtFrame().header;
            std::vector<float> emptyPathTiming;
            Trajectory emptyTraj(emptyPath, emptyPathTiming);

            setCurrentTraj(emptyTraj);
            setCurrentLeftGapPtModelID(nullptr);
            setCurrentRightGapPtModelID(nullptr);

            trajVisualizer_->drawCurrentTrajectory(emptyTraj);
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, emptyTraj);
            trajectoryChangeCount_++;            

            return emptyTraj;
        }        
    }

    Trajectory Planner::compareToCurrentTraj(Gap * incomingGap,
                                                const Trajectory & incomingTraj,
                                                const bool & isIncomingGapFeasible,
                                                const std::vector<sensor_msgs::LaserScan> & futureScans) // bool isIncomingGapAssociated,
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[compareToCurrentTraj()]");
        boost::mutex::scoped_lock gapset(gapMutex_);
        
        // Gap * incomingGap = (trajFlag == 0 ? feasibleGaps.at(lowestCostTrajIdx) : nullptr);
        // Trajectory incomingTraj = trajs.at(lowestCostTrajIdx);
        Trajectory currentTraj = getCurrentTraj();

        // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapPtModelID() << ", current right gap index: " << getCurrentRightGapPtModelID());

        // ROS_INFO_STREAM("current gap indices: " << getCurrentLeftGapPtModelID() << ", " << getCurrentRightGapPtModelID());
        
        //////////////////////////////////////////////////////////////////////////////
        // Transform into the current robot frame to score against the current scan //
        //////////////////////////////////////////////////////////////////////////////
        geometry_msgs::PoseArray incomingPathRobotFrame = gapTrajGenerator_->transformPath(incomingTraj.getPathOdomFrame(), odom2rbt_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    evaluating incoming trajectory");
        std::vector<float> incomingPathPoseCosts;
        float incomingPathTerminalPoseCost;
        trajEvaluator_->evaluateTrajectory(incomingTraj, incomingPathPoseCosts, incomingPathTerminalPoseCost, futureScans);

        float incomingPathCost = incomingPathTerminalPoseCost + std::accumulate(incomingPathPoseCosts.begin(), 
                                                                                incomingPathPoseCosts.end(), 
                                                                                float(0));
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    incoming trajectory received a cost of: " << incomingPathCost);
        

        ///////////////////////////////////////////////////////////////////////
        //  Evaluate the incoming path to determine if we can switch onto it //
        ///////////////////////////////////////////////////////////////////////
        std::string incomingPathStatus = "incoming path is safe to switch onto";

        bool ableToSwitchToIncomingPath = true;
        if (incomingTraj.getPathRbtFrame().poses.size() == 0)
        {
            incomingPathStatus = "incoming path is of length zero.";
            ableToSwitchToIncomingPath = false;
        } else if (incomingPathCost == std::numeric_limits<float>::infinity())
        {
            incomingPathStatus = "incoming path is of cost infinity.";
            ableToSwitchToIncomingPath = false;
        }
            
        ///////////////////////////////////////////////////////////////////////////////////
        //  Enact a trajectory switch if the currently executing path is empty (size: 0) //
        ///////////////////////////////////////////////////////////////////////////////////
        bool isCurrentPathEmpty = currentTraj.getPathRbtFrame().poses.size() == 0;
        if (isCurrentPathEmpty) 
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  
                                                            ": current path is of length zero, " << incomingPathStatus);                
            return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
        } 

        ///////////////////////////////////////////////////////////////////////////////////
        //  Enact a trajectory switch if the currently executing path is empty (size: 0) //
        ///////////////////////////////////////////////////////////////////////////////////
        if (!isIncomingGapFeasible) 
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  
                                                            ": current gap is not feasible, " << incomingPathStatus);                
            return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
        } 

        // Transform current traj into most recent robot frame to score against the current scan
        geometry_msgs::PoseArray updatedCurrentPathRobotFrame = gapTrajGenerator_->transformPath(currentTraj.getPathOdomFrame(), odom2rbt_);
        std::vector<float> updatedCurrentPathTiming = currentTraj.getPathTiming();
        Trajectory updatedCurrentTraj(updatedCurrentPathRobotFrame, updatedCurrentPathTiming);
        updatedCurrentTraj.setPathOdomFrame(currentTraj.getPathOdomFrame()); // odom frame traj will not change

        ////////////////////////////////////////////////////////////////////////////////////////////
        //  Enact a trajectory switch if the currently executing path has been completely tracked //
        ////////////////////////////////////////////////////////////////////////////////////////////   

        int updatedCurrentPathPoseIdx = getClosestTrajectoryPoseIdx(updatedCurrentPathRobotFrame);
        geometry_msgs::PoseArray reducedCurrentPathRobotFrame = updatedCurrentPathRobotFrame;
        reducedCurrentPathRobotFrame.poses = std::vector<geometry_msgs::Pose>(updatedCurrentPathRobotFrame.poses.begin() + updatedCurrentPathPoseIdx, 
                                                                                updatedCurrentPathRobotFrame.poses.end());

        std::vector<float> reducedCurrentPathTiming = updatedCurrentPathTiming;
        reducedCurrentPathTiming = std::vector<float>(updatedCurrentPathTiming.begin() + updatedCurrentPathPoseIdx, updatedCurrentPathTiming.end());
        
        if (reducedCurrentPathRobotFrame.poses.size() < 2) 
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ <<  
                                                            ": old path length less than 2, " << incomingPathStatus);

            return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //  Compare the costs of the incoming trajectory with the cost of the current trajectory to see if we need to switch //
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        int poseCheckCount = std::min(incomingTraj.getPathRbtFrame().poses.size(), reducedCurrentPathRobotFrame.poses.size());
        incomingPathCost = std::accumulate(incomingPathPoseCosts.begin(), incomingPathPoseCosts.begin() + poseCheckCount, float(0));

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    re-evaluating incoming trajectory received a subcost of: " << incomingPathCost);
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    evaluating current trajectory");            
        
        Trajectory reducedCurrentTraj(reducedCurrentPathRobotFrame, reducedCurrentPathTiming);
        std::vector<float> reducedCurrentPathPoseCosts;
        float reducedCurrentPathTerminalPoseCost;
        trajEvaluator_->evaluateTrajectory(reducedCurrentTraj, reducedCurrentPathPoseCosts, reducedCurrentPathTerminalPoseCost, futureScans);
        float reducedCurrentPathSubCost = reducedCurrentPathTerminalPoseCost + std::accumulate(reducedCurrentPathPoseCosts.begin(), 
                                                                                                reducedCurrentPathPoseCosts.begin() + poseCheckCount, 
                                                                                                float(0));
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    reduced current trajectory received a subcost of: " << reducedCurrentPathSubCost);

        if (reducedCurrentPathSubCost == std::numeric_limits<float>::infinity()) 
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ << 
                                                        ": current trajectory is of cost infinity," << incomingPathStatus);
            return changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
        }
        
        // if (incomingPathCost < currentPathSubCost) 
        // {
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory change " << trajectoryChangeCount_ << 
        //                                                 ": incoming trajectory is lower score");
        //     changeTrajectoryHelper(incomingGap, incomingTraj, ableToSwitchToIncomingPath);
        // }
        
        
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        trajectory maintain");
        // currentTrajectoryPublisher_.publish(currentTraj.getPathRbtFrame());
        trajVisualizer_->drawCurrentTrajectory(reducedCurrentTraj);

        return updatedCurrentTraj;
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

    std::vector<Gap *> Planner::deepCopyCurrentRawGaps()
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        std::vector<Gap *> copiedRawGaps;

        for (Gap * currRawGap : currRawGaps_)
            copiedRawGaps.push_back(new Gap(*currRawGap));

        return copiedRawGaps;
    }

    std::vector<Gap *> Planner::deepCopyCurrentSimplifiedGaps()
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        std::vector<Gap *> planningGaps;

        for (Gap * currSimplifiedGap : currSimplifiedGaps_)
            planningGaps.push_back(new Gap(*currSimplifiedGap));

        return planningGaps;
    }

    void Planner::runPlanningLoop(Trajectory & chosenTraj, int & trajFlag) 
    {
        ROS_INFO_STREAM_NAMED("Planner", "[runPlanningLoop()]: count " << planningLoopCalls);

        if (!initialized_ || !hasLaserScan_ || !hasGlobalGoal_)
        {
            ROS_WARN_STREAM_NAMED("Planner", "Not ready to plan, initialized: " << initialized_ << ", laser scan: " << hasLaserScan_ << ", global goal: " << hasGlobalGoal_);
            chosenTraj = Trajectory();
            return;            
        }

        if (colliding_)
        {
            ROS_WARN_STREAM_NAMED("Planner", "In collision");
            chosenTraj = Trajectory();
            return;
        }

        trajVisualizer_->drawPlanningLoopIdx(planningLoopCalls);

        isGoalReached();

        std::vector<Gap *> copiedRawGaps = deepCopyCurrentRawGaps();
        std::vector<Gap *> planningGaps = deepCopyCurrentSimplifiedGaps();
        std::vector<Ungap *> ungaps;

        // Gap Tubes
        std::vector<GapTube *> gapTubes; /**< Set of gap tubes */

        std::chrono::steady_clock::time_point planningLoopStartTime = std::chrono::steady_clock::now();

        int gapCount = planningGaps.size();

        if (gapCount == 0)
        {
            ROS_WARN_STREAM_NAMED("Planner", "No gaps to plan for");
            chosenTraj = Trajectory();
            trajFlag = NONE;
            return;
        }

        //////////////////////////////////////////////////////////////////////////////////////
        //                                      ATTACH UNGAP IDS                            //
        //////////////////////////////////////////////////////////////////////////////////////

        attachUngapIDs(planningGaps, ungaps);
        
        //////////////////////////////////////////////////////////////////////////////////////
        //                              PRUNE APPROACHING UNGAPS                            //
        //////////////////////////////////////////////////////////////////////////////////////

        std::vector<Ungap *> recedingUngaps = pruneApproachingUngaps(ungaps);

        //////////////////////////////////////////////////////////////////////////////////////
        //                              GAP POINT PROPAGATION                               //
        //////////////////////////////////////////////////////////////////////////////////////

        std::chrono::steady_clock::time_point gapPropagateStartTime = std::chrono::steady_clock::now();
        propagateGapPoints(planningGaps);
        float gapPropagateTimeTaken = timeTaken(gapPropagateStartTime);
        float avgGapPropagationTimeTaken = computeAverageTimeTaken(gapPropagateTimeTaken, GAP_PROP);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Propagation for " << gapCount << " gaps took " << gapPropagateTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Propagation average time: " << avgGapPropagationTimeTaken << " seconds (" << (1.0 / avgGapPropagationTimeTaken) << " Hz) ]");

        //////////////////////////////////////////////////////////////////////////////////////
        //                                  GAP MANIPULATION                                //
        //////////////////////////////////////////////////////////////////////////////////////

        std::chrono::steady_clock::time_point manipulateGapsStartTime = std::chrono::steady_clock::now();
        std::vector<Gap *> manipulatedGaps = manipulateGaps(planningGaps);
        float gapManipulationTimeTaken = timeTaken(manipulateGapsStartTime);
        float avgGapManipulationTimeTaken = computeAverageTimeTaken(gapManipulationTimeTaken, GAP_MANIP);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Manipulation for " << gapCount << " gaps took " << gapManipulationTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Manipulation average time: " << avgGapManipulationTimeTaken << " seconds (" << (1.0 / avgGapManipulationTimeTaken) << " Hz) ]");

        //////////////////////////////////////////////////////////////////////////////////////
        //                            GAP POINT PROPAGATION (v2)                            //
        //////////////////////////////////////////////////////////////////////////////////////

        std::chrono::steady_clock::time_point gapPropagateV2StartTime = std::chrono::steady_clock::now();
        propagateGapPointsV2(manipulatedGaps, gapTubes);
        float gapPropagateV2TimeTaken = timeTaken(gapPropagateV2StartTime);
        // float avgGapPropagationTimeTaken = computeAverageTimeTaken(gapPropagateTimeTaken, GAP_PROP);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Propagation (v2) for " << gapCount << " gaps took " << gapPropagateV2TimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Propagation (v2) average time: " << avgGapPropagationTimeTaken << " seconds (" << (1.0 / avgGapPropagationTimeTaken) << " Hz) ]");

        //////////////////////////////////////////////////////////////////////////////////////
        //                              GAP MANIPULATION                                    //
        //////////////////////////////////////////////////////////////////////////////////////

        gapGoalPlacementV2(gapTubes);

        ungapGoalPlacement(recedingUngaps);

        //////////////////////////////////////////////////////////////////////////////////////
        //                           GAP FEASIBILITY CHECK (v2)                             //
        //////////////////////////////////////////////////////////////////////////////////////

        bool isCurrentGapFeasibleDummy = false;
        gapSetFeasibilityCheckV2(gapTubes, isCurrentGapFeasibleDummy);

        //////////////////////////////////////////////////////////////////////////////////////
        //                         UNGAP FEASIBILITY CHECK                                  //
        //////////////////////////////////////////////////////////////////////////////////////

        ungapSetFeasibilityCheck(recedingUngaps);

        //////////////////////////////////////////////////////////////////////////////////////
        //                            GAP FEASIBILITY CHECK                                 //
        //////////////////////////////////////////////////////////////////////////////////////

        bool isCurrentGapFeasible = false;
        std::vector<Gap *> feasibleGaps;
        std::chrono::steady_clock::time_point feasibilityStartTime = std::chrono::steady_clock::now();
        if (cfg_.planning.gap_feasibility_check)
        {
            feasibleGaps = gapSetFeasibilityCheck(manipulatedGaps, isCurrentGapFeasible);
        } else
        {
            for (Gap * currSimplifiedGap : currSimplifiedGaps_)
                feasibleGaps.push_back(currSimplifiedGap);

            // TODO: need to set feasible to true for all gaps as well
        }
        float feasibilityTimeTaken = timeTaken(feasibilityStartTime);
        float avgFeasibilityTimeTaken = computeAverageTimeTaken(feasibilityTimeTaken, GAP_FEAS);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Feasibility Analysis for " << gapCount << " gaps took " << feasibilityTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Feasibility Analysis average time: " << avgFeasibilityTimeTaken << " seconds (" << (1.0 / avgFeasibilityTimeTaken) << " Hz) ]");

        // Have to run here because terminal gap goals are set during feasibility check
        // gapVisualizer_->drawManipGaps(manipulatedGaps, std::string("manip"));
        // goalVisualizer_->drawGapGoals(manipulatedGaps);

        gapCount = feasibleGaps.size();

        //////////////////////////////////////////////////////////////////////////////////////
        //                              FUTURE SCAN PROPAGATION                             //
        //////////////////////////////////////////////////////////////////////////////////////
        std::vector<sensor_msgs::LaserScan> futureScans;
        std::chrono::steady_clock::time_point scanPropagationStartTime = std::chrono::steady_clock::now();
        if (cfg_.planning.future_scan_propagation)
        {
            if (cfg_.planning.egocircle_prop_cheat)
                throw std::runtime_error("cheat not implemented"); // futureScans = dynamicScanPropagator_->propagateCurrentLaserScanCheat(currentTrueAgentPoses_, currentTrueAgentVels_);
            else
                futureScans = dynamicScanPropagator_->propagateCurrentLaserScan(copiedRawGaps);        
        } else 
        {
            sensor_msgs::LaserScan currentScan = *scan_.get();
            futureScans = std::vector<sensor_msgs::LaserScan>(int(cfg_.traj.integrate_maxt/cfg_.traj.integrate_stept) + 1, currentScan);
        }
        float scanPropagationTimeTaken = timeTaken(scanPropagationStartTime);
        float avgScanPropagationTimeTaken = computeAverageTimeTaken(scanPropagationTimeTaken, SCAN_PROP);
        ROS_INFO_STREAM_NAMED("Timing", "       [Future Scan Propagation for " << gapCount << " gaps took " << scanPropagationTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Future Scan Propagation average time: " << avgScanPropagationTimeTaken << " seconds (" << (1.0 / avgScanPropagationTimeTaken) << " Hz) ]");
    

        //////////////////////////////////////////////////////////////////////////////////////
        //                        GAP TRAJECTORY GENERATION AND SCORING (V2)                //
        //////////////////////////////////////////////////////////////////////////////////////
        std::vector<Trajectory> gapTubeTrajs;
        std::vector<std::vector<float>> gapTubeTrajPoseCosts; 
        std::vector<float> gapTubeTrajTerminalPoseCosts; 
        // std::chrono::steady_clock::time_point generateGapTrajsStartTime = std::chrono::steady_clock::now();
        generateGapTrajsV2(gapTubes, gapTubeTrajs, gapTubeTrajPoseCosts, gapTubeTrajTerminalPoseCosts, futureScans);
        // float generateGapTrajsTimeTaken = timeTaken(generateGapTrajsStartTime);
        // float avgGenerateGapTrajsTimeTaken = computeAverageTimeTaken(generateGapTrajsTimeTaken, TRAJ_GEN);
        // ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Generation (v2) for " << gapCount << " gaps took " << generateGapTrajsTimeTaken << " seconds]");
        // ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Generation (v2) average time: " << avgGenerateGapTrajsTimeTaken << " seconds (" << (1.0 / avgGenerateGapTrajsTimeTaken) << " Hz) ]");
    
        //////////////////////////////////////////////////////////////////////////////////////
        //                          GAP TRAJECTORY GENERATION AND SCORING                   //
        //////////////////////////////////////////////////////////////////////////////////////
        std::vector<Trajectory> gapTrajs;
        std::vector<std::vector<float>> gapTrajPoseCosts; 
        std::vector<float> gapTrajTerminalPoseCosts; 
        std::chrono::steady_clock::time_point generateGapTrajsStartTime = std::chrono::steady_clock::now();
        generateGapTrajs(feasibleGaps, gapTrajs, gapTrajPoseCosts, gapTrajTerminalPoseCosts, futureScans);
        float generateGapTrajsTimeTaken = timeTaken(generateGapTrajsStartTime);
        float avgGenerateGapTrajsTimeTaken = computeAverageTimeTaken(generateGapTrajsTimeTaken, GAP_TRAJ_GEN);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Generation for " << gapCount << " gaps took " << generateGapTrajsTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Generation average time: " << avgGenerateGapTrajsTimeTaken << " seconds (" << (1.0 / avgGenerateGapTrajsTimeTaken) << " Hz) ]");
    
        //////////////////////////////////////////////////////////////////////////////////////
        //                         UNGAP TRAJECTORY GENERATION AND SCORING                  //
        //////////////////////////////////////////////////////////////////////////////////////
        std::vector<Trajectory> ungapTrajs;
        std::vector<std::vector<float>> ungapTrajPoseCosts; 
        std::vector<float> ungapTrajTerminalPoseCosts; 
        std::chrono::steady_clock::time_point generateUngapTrajsStartTime = std::chrono::steady_clock::now();
        generateUngapTrajs(recedingUngaps, ungapTrajs, ungapTrajPoseCosts, ungapTrajTerminalPoseCosts, futureScans);
        float generateUngapTrajsTimeTaken = timeTaken(generateUngapTrajsStartTime);
        float avgGenerateUngapTrajsTimeTaken = computeAverageTimeTaken(generateUngapTrajsTimeTaken, UNGAP_TRAJ_GEN);
        ROS_INFO_STREAM_NAMED("Timing", "       [Ungap Trajectory Generation for " << recedingUngaps.size() << " un-gaps took " << generateUngapTrajsTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Ungap Trajectory Generation average time: " << avgGenerateUngapTrajsTimeTaken << " seconds (" << (1.0 / avgGenerateUngapTrajsTimeTaken) << " Hz) ]");

        //////////////////////////////////////////////////////////////////////////////////////
        //                        IDLING TRAJECTORY GENERATION AND SCORING                  //
        //////////////////////////////////////////////////////////////////////////////////////
        std::vector<Trajectory> idlingTrajs;
        std::vector<std::vector<float>> idlingPathPoseCosts; 
        std::vector<float> idlingPathTerminalPoseCosts; 
        std::chrono::steady_clock::time_point generateIdlingTrajsStartTime = std::chrono::steady_clock::now();
        generateIdlingTraj(idlingTrajs, idlingPathPoseCosts, idlingPathTerminalPoseCosts, futureScans);         
        float generateIdlingTrajsTimeTaken = timeTaken(generateIdlingTrajsStartTime);
        float avgGenerateIdlingTrajsTimeTaken = computeAverageTimeTaken(generateIdlingTrajsTimeTaken, IDLING_TRAJ_GEN);
        ROS_INFO_STREAM_NAMED("Timing", "       [Idling Trajectory Generation took " << generateIdlingTrajsTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Idling Trajectory Generation average time: " << avgGenerateIdlingTrajsTimeTaken << " seconds (" << (1.0 / avgGenerateIdlingTrajsTimeTaken) << " Hz) ]");
    
        // Aggregate trajs/costs

        ROS_INFO_STREAM_NAMED("Planner", "       ungap trajs size: " << ungapTrajs.size());
        ROS_INFO_STREAM_NAMED("Planner", "       gap trajs size: " << gapTrajs.size());
        ROS_INFO_STREAM_NAMED("Planner", "       idling gap trajs size: " << idlingTrajs.size());

        //////////////////////////////////////////////////////////////////////////////////////
        //                               GAP TRAJECTORY SELECTION                           //
        //////////////////////////////////////////////////////////////////////////////////////

        std::chrono::steady_clock::time_point pickTrajStartTime = std::chrono::steady_clock::now();
        int lowestCostTrajIdx = -1;
        trajFlag = NONE;
        
        pickTraj(trajFlag, lowestCostTrajIdx, 
                    gapTrajs, gapTrajPoseCosts, gapTrajTerminalPoseCosts,
                    ungapTrajs, ungapTrajPoseCosts, ungapTrajTerminalPoseCosts,
                    idlingTrajs, idlingPathPoseCosts, idlingPathTerminalPoseCosts);
        float pickTrajTimeTaken = timeTaken(pickTrajStartTime);
        float avgPickTrajTimeTaken = computeAverageTimeTaken(pickTrajTimeTaken, TRAJ_PICK);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Selection for " << gapCount << " gaps took " << pickTrajTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Selection average time: " << avgPickTrajTimeTaken << " seconds (" << (1.0 / avgPickTrajTimeTaken) << " Hz) ]");
    
        //////////////////////////////////////////////////////////////////////////////////////
        //                              GAP TRAJECTORY COMPARISON                           //
        //////////////////////////////////////////////////////////////////////////////////////

        if (trajFlag != NONE) 
        {
            std::chrono::steady_clock::time_point compareToCurrentTrajStartTime = std::chrono::steady_clock::now();

            Gap * incomingGap = nullptr;
            Trajectory incomingTraj;

            if (trajFlag == GAP) // comparing to traj within one of the gaps
            {
                ROS_INFO_STREAM_NAMED("Planner", "       comparing to actual trajectory");
                incomingGap = feasibleGaps.at(lowestCostTrajIdx);
                incomingTraj = gapTrajs.at(lowestCostTrajIdx);
            } else if (trajFlag == UNGAP) // comparing to un-gap traj
            {
                ROS_INFO_STREAM_NAMED("Planner", "       comparing to un-gap trajectory");
                incomingTraj = ungapTrajs.at(lowestCostTrajIdx);
            } else if (trajFlag == IDLING)
            {
                ROS_INFO_STREAM_NAMED("Planner", "       comparing to idling trajectory");
                incomingTraj = idlingTrajs.at(lowestCostTrajIdx);
            } else
            {
                ROS_WARN_STREAM_NAMED("Planner", "       unknown trajectory flag");
            }


            chosenTraj = compareToCurrentTraj(incomingGap,
                                                incomingTraj,           
                                                isCurrentGapFeasible,
                                                futureScans);

            float compareToCurrentTrajTimeTaken = timeTaken(compareToCurrentTrajStartTime);
            float avgCompareToCurrentTrajTimeTaken = computeAverageTimeTaken(compareToCurrentTrajTimeTaken, TRAJ_COMP);        

            ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Comparison for " << gapCount << " gaps took " << compareToCurrentTrajTimeTaken << " seconds]");
            ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Comparison average time: " << avgCompareToCurrentTrajTimeTaken << " seconds (" << (1.0 / avgCompareToCurrentTrajTimeTaken) << " Hz) ]");
        } 

        // // publish for safety module
        // if (currentLeftGapPtModel && currentRightGapPtModel)
        // {
        //     geometry_msgs::PoseArray mpcInput;
        //     mpcInput.header.stamp = ros::Time::now();
        //     mpcInput.header.frame_id = cfg_.robot_frame_id;
            
        //     // current rbt vel
        //     geometry_msgs::Pose currentRbtVelAsPose;
        //     currentRbtVelAsPose.position.x = currentRbtVel_.twist.linear.x;
        //     currentRbtVelAsPose.position.y = currentRbtVel_.twist.linear.y;
        //     mpcInput.poses.push_back(currentRbtVelAsPose);

        //     // three slice points            
        //     currentLeftGapPtModel->isolateGapDynamics();
        //     currentRightGapPtModel->isolateGapDynamics();

        //     Eigen::Vector4f leftGapState = currentLeftGapPtModel->getGapState();
        //     Eigen::Vector4f rightGapState = currentRightGapPtModel->getGapState();

        //     Eigen::Vector2f terminalLeftPt = leftGapState.head(2);


        //     mpcInputPublisher_.publish(mpcInput);
        // }

        float planningLoopTimeTaken = timeTaken(planningLoopStartTime);
        float avgPlanningLoopTimeTaken = computeAverageTimeTaken(planningLoopTimeTaken, PLAN);        
        float avgNumberGaps = computeAverageNumberGaps(gapCount);

        ROS_INFO_STREAM_NAMED("Timing", "       [Planning Loop for " << gapCount << " gaps took " << planningLoopTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Planning Loop average time: " << avgPlanningLoopTimeTaken << " seconds (" << (1.0 / avgPlanningLoopTimeTaken) << " Hz) ]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Planning Loop average number of gaps: " << avgNumberGaps << "]");

        // delete set of planning gaps
        for (Gap * planningGap : planningGaps)
            delete planningGap;

        // delete set of planning gaps
        for (Gap * copiedRawGap : copiedRawGaps)
            delete copiedRawGap;

        for (Ungap * ungap : ungaps)
            delete ungap;

        // delete gap tubes
        for (GapTube * tube : gapTubes)
        {
            delete tube;
        }

        return;
    }

    void Planner::attachUngapIDs(const std::vector<Gap *> & planningGaps,
                                    std::vector<Ungap *> & ungaps)
    {
        std::vector<Eigen::Vector4f> gapPtStates;

        // get list of points
        for (const Gap * planningGap : planningGaps)
        {
            // right
            planningGap->getRightGapPt()->getModel()->isolateGapDynamics();

            gapPtStates.push_back(planningGap->getRightGapPt()->getModel()->getGapState());

            // left
            planningGap->getLeftGapPt()->getModel()->isolateGapDynamics();

            gapPtStates.push_back(planningGap->getLeftGapPt()->getModel()->getGapState());
        }

        // for each point, check if it is an ungap point
        for (int i = 0; i < gapPtStates.size(); i++)
        {
            Eigen::Vector4f ptIState = gapPtStates.at(i);

            int nextIdx = (i + 1) % gapPtStates.size();
            Eigen::Vector4f ptJState = gapPtStates.at(nextIdx);

            int ptIGapID = i / 2;
            int ptJGapID = nextIdx / 2;

            if (ptIGapID != ptJGapID && isUngap(ptIState, ptJState))
            {
                int ungapID = i / 2;
                if (i % 2 == 0) // attach ungap id to the point
                {
                    planningGaps.at(i / 2)->getRightGapPt()->setUngapID(ungapID);
                    planningGaps.at(nextIdx / 2)->getLeftGapPt()->setUngapID(ungapID);
                    ungaps.push_back(new Ungap(planningGaps.at(i / 2)->getRightGapPt(), 
                                                planningGaps.at(nextIdx / 2)->getLeftGapPt(),
                                                ungapID));
                } else
                {
                    planningGaps.at(i / 2)->getLeftGapPt()->setUngapID(ungapID);
                    planningGaps.at(nextIdx / 2)->getRightGapPt()->setUngapID(ungapID);
                    ungaps.push_back(new Ungap(planningGaps.at(nextIdx / 2)->getRightGapPt(), 
                                                planningGaps.at(i / 2)->getLeftGapPt(),
                                                ungapID));
                }
            }
        }
    }

    bool Planner::isUngap(const Eigen::Vector4f & ptIState, const Eigen::Vector4f & ptJState)
    {
        Eigen::Vector2f ptIPos = ptIState.head(2);
        Eigen::Vector2f ptJPos = ptJState.head(2);

        Eigen::Vector2f ptIVel = ptIState.tail(2);
        Eigen::Vector2f ptJVel = ptJState.tail(2);

        // check if distance between points is less than 4 * r_inscr * inf_ratio
        bool distCheck = (ptIPos - ptJPos).norm() < 4 * cfg_.rbt.r_inscr * cfg_.traj.inf_ratio;

        // // check if speed of points is greater than 0.10
        bool speedCheck = (ptIVel.norm() >= 0.10 && ptJVel.norm() >= 0.10);

        // // run angle check on LHS and RHS model velocities
        float vectorProj = ptIVel.dot(ptJVel) / (ptIVel.norm() * ptJVel.norm() + eps);
        bool angleCheck = (vectorProj > 0.0);

        return distCheck; // (distCheck && speedCheck && angleCheck);
    }

    std::vector<Ungap *> Planner::pruneApproachingUngaps(const std::vector<Ungap *> & ungaps)
    {
        ROS_INFO_STREAM_NAMED("Planner", "[pruneApproachingUngaps()]");

        std::vector<Ungap *> recedingUngaps;

        ROS_INFO_STREAM_NAMED("Planner", "       pruning ungaps ...");
        for (int i = 0; i < ungaps.size(); i++)
        {
            ROS_INFO_STREAM_NAMED("Planner", "          ungap " << i);
            Ungap * ungap = ungaps.at(i);

            ungap->getLeftUngapPt()->getModel()->isolateGapDynamics();
            ungap->getRightUngapPt()->getModel()->isolateGapDynamics();

            Eigen::Vector4f leftState = ungap->getLeftUngapPt()->getModel()->getGapState();
            Eigen::Vector4f rightState = ungap->getRightUngapPt()->getModel()->getGapState();

            Eigen::Vector2f leftPos = leftState.head(2);
            Eigen::Vector2f rightPos = rightState.head(2);

            Eigen::Vector2f avgPos = (leftPos + rightPos) / 2;

            ROS_INFO_STREAM_NAMED("Planner", "          avgPos: " << avgPos.transpose());

            Eigen::Vector2f leftVel = leftState.tail(2);
            Eigen::Vector2f rightVel = rightState.tail(2);

            Eigen::Vector2f avgVel = (leftVel + rightVel) / 2;

            ROS_INFO_STREAM_NAMED("Planner", "          avgVel: " << avgVel.transpose());

            if (avgPos.dot(avgVel) > 0)
            {
                ROS_INFO_STREAM_NAMED("Planner", "          ungap is receding, adding");
                recedingUngaps.push_back(ungap);
            }        
        }


        return recedingUngaps;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(const geometry_msgs::PoseArray & localTrajectory,
                                                    int & trajFlag) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "[ctrlGeneration()]");
        std::chrono::steady_clock::time_point controlStartTime = std::chrono::steady_clock::now();
        
        geometry_msgs::Twist rawCmdVel = geometry_msgs::Twist();
        geometry_msgs::Twist cmdVel = rawCmdVel;

        if (!haveTFs_)
            return cmdVel;
        
        // Know Current Pose
        geometry_msgs::PoseStamped currPoseStRobotFrame;
        currPoseStRobotFrame.header.frame_id = cfg_.robot_frame_id;
        currPoseStRobotFrame.pose.orientation.w = 1;
        geometry_msgs::PoseStamped currPoseStampedOdomFrame;
        currPoseStampedOdomFrame.header.frame_id = cfg_.odom_frame_id;
        tf2::doTransform(currPoseStRobotFrame, currPoseStampedOdomFrame, rbt2odom_);
        geometry_msgs::Pose currPoseOdomFrame = currPoseStampedOdomFrame.pose;

        if (trajFlag == IDLING) // idling
        {
            ROS_INFO_STREAM_NAMED("Planner", "planner opting to idle, no trajectory chosen.");
            rawCmdVel = geometry_msgs::Twist();
            return rawCmdVel;                
        } else if (trajFlag == NONE) // OBSTACLE AVOIDANCE CONTROL 
        { 
            ROS_INFO_STREAM_NAMED("Planner", "Available Execution Traj length: " << localTrajectory.poses.size() << " == 0, obstacle avoidance control chosen.");
            rawCmdVel = trajController_->obstacleAvoidanceControlLaw();
            return rawCmdVel;
        } else if (cfg_.ctrl.man_ctrl)  // MANUAL CONTROL 
        {
            ROS_INFO_STREAM_NAMED("Controller", "Manual control chosen.");
            // rawCmdVel = trajController_->manualControlLawKeyboard();
            // rawCmdVel = trajController_->manualControlLawReconfig();
            rawCmdVel = trajController_->manualControlLawPrescribed(currPoseOdomFrame);
        } else if (cfg_.ctrl.mpc_ctrl) // MPC CONTROL
        { 
            rawCmdVel = mpcTwist_;
        } else if (cfg_.ctrl.feedback_ctrl && (trajFlag == UNGAP || trajFlag == GAP)) // FEEDBACK CONTROL 
        {
            ROS_INFO_STREAM_NAMED("Controller", "Trajectory tracking control chosen.");

            // obtain current robot pose in odom frame

            // traj in odom frame here

            // get point along trajectory to target/move towards
            targetTrajectoryPoseIdx_ = trajController_->extractTargetPoseIdx(currPoseOdomFrame, localTrajectory);

            geometry_msgs::Pose targetTrajectoryPose = localTrajectory.poses.at(targetTrajectoryPoseIdx_);

            std::chrono::steady_clock::time_point feedbackControlStartTime = std::chrono::steady_clock::now();
            rawCmdVel = trajController_->constantVelocityControlLaw(currPoseOdomFrame, targetTrajectoryPose);
            float feedbackControlTimeTaken = timeTaken(feedbackControlStartTime);
            float avgFeedbackControlTimeTaken = computeAverageTimeTaken(feedbackControlTimeTaken, FEEBDACK);        
            ROS_INFO_STREAM_NAMED("Timing", "       [Feedback Control took " << feedbackControlTimeTaken << " seconds]");
            ROS_INFO_STREAM_NAMED("Timing", "       [Feedback Control average time: " << avgFeedbackControlTimeTaken << " seconds (" << (1.0 / avgFeedbackControlTimeTaken) << " Hz) ]");        
        } else
        {
            ROS_ERROR_STREAM_NAMED("Controller", "No control method selected");
        }

        std::chrono::steady_clock::time_point projOpStartTime = std::chrono::steady_clock::now();
        cmdVel = trajController_->processCmdVel(rawCmdVel,
                                                rbtPoseInSensorFrame_, 
                                                currentRbtVel_, currentRbtAcc_); 
        float projOpTimeTaken = timeTaken(projOpStartTime);
        float avgProjOpTimeTaken = computeAverageTimeTaken(projOpTimeTaken, PO);        
        ROS_INFO_STREAM_NAMED("Timing", "       [Projection Operator took " << projOpTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Projection Operator average time: " << avgProjOpTimeTaken << " seconds (" << (1.0 / avgProjOpTimeTaken) << " Hz) ]");        

        float controlTimeTaken = timeTaken(controlStartTime);
        float avgControlTimeTaken = computeAverageTimeTaken(controlTimeTaken, CONTROL);        
        ROS_INFO_STREAM_NAMED("Timing", "       [Control Loop took " << controlTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Control Loop average time: " << avgControlTimeTaken << " seconds (" << (1.0 / avgControlTimeTaken) << " Hz) ]");        

        return cmdVel;
    }

    void Planner::setCurrentLeftGapPtModelID(Estimator * leftModel) 
    { 
        if (leftModel) 
        {
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentLeftGapPtModelID]: setting current left ID to " << leftModel->getID());
            currentLeftGapPtModelID = leftModel->getID(); 
      
            // leftModel->isolateGapDynamics();
            // currentLeftGapPtState = leftModel->getGapState(); 
        } else
        {
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentLeftGapPtModelID]: null, setting current left ID to " << -1);
            currentLeftGapPtModelID = -1;
            // currentLeftGapPtModel = nullptr;
        }
    }

    void Planner::setCurrentRightGapPtModelID(Estimator * rightModel) 
    { 
        if (rightModel) 
        {        
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentRightGapPtModelID]: setting current right ID to " << rightModel->getID());
            currentRightGapPtModelID = rightModel->getID();

            // rightModel->isolateGapDynamics();
            // currentRightGapPtState = rightModel->getGapState();             
        } else
        {
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentRightGapPtModelID]: null, setting current right ID to " << -1);
            currentRightGapPtModelID = -1;
            // currentRightGapPtModel = nullptr;
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
        setCurrentTraj(Trajectory());
        currentRbtVel_ = geometry_msgs::TwistStamped();
        currentRbtAcc_ = geometry_msgs::TwistStamped();
        setCurrentLeftGapPtModelID(nullptr);
        setCurrentRightGapPtModelID(nullptr);
        ROS_INFO_STREAM_NAMED("Planner", "velocity buffer size: " << cmdVelBuffer_.size());
        cmdVelBuffer_.clear();
        ROS_INFO_STREAM_NAMED("Planner", "velocity buffer size after clear: " << cmdVelBuffer_.size() << ", is full: " << cmdVelBuffer_.capacity());
        return;
    }

    void Planner::checkGapModels(const std::vector<Gap *> & gaps) 
    {
        for (size_t i = 0; i < gaps.size(); i++)
        {
            Gap * gap = gaps.at(i);        
            Eigen::Vector4f left_state = gap->getLeftGapPt()->getModel()->getState();
            Eigen::Vector4f right_state = gap->getRightGapPt()->getModel()->getState();

            if (!checkModelState(left_state) || !checkModelState(right_state))
            {
                ROS_WARN_STREAM_NAMED("Planner", "    model state is not OK, returning");
                return;
            }
        }
    }

    void Planner::printGapModels(const std::vector<Gap *> & gaps) 
    {
        // THIS IS NOT FOR MANIPULATED GAPS
        ROS_INFO_STREAM_NAMED("GapDetector", "gaps size: " << gaps.size());
        float x = 0.0, y = 0.0;
        for (size_t i = 0; i < gaps.size(); i++)
        {
            Gap * gap = gaps.at(i);
            ROS_INFO_STREAM_NAMED("Planner", "    gap " << i << ", indices: " << gap->RIdx() << " to "  << gap->LIdx() << 
                                                ", left model ID: " << gap->getLeftGapPt()->getModel()->getID() << 
                                                ", right model ID: " << gap->getRightGapPt()->getModel()->getID());
            Eigen::Vector4f left_state = gap->getLeftGapPt()->getModel()->getState();
            gap->getLCartesian(x, y);            
            ROS_INFO_STREAM_NAMED("Planner", "        left point: (" << x << ", " << y << "), left model: (" << left_state.transpose() << ")");
            Eigen::Vector4f right_state = gap->getRightGapPt()->getModel()->getState();
            gap->getRCartesian(x, y);
            ROS_INFO_STREAM_NAMED("Planner", "        right point: (" << x << ", " << y << "), right model: (" << right_state.transpose() << ")");
        }
    }

    bool Planner::recordAndCheckVel(const geometry_msgs::Twist & cmdVel, const int & trajFlag) 
    {
        float cmdVelNorm = std::abs(cmdVel.linear.x) + std::abs(cmdVel.linear.y) + std::abs(cmdVel.angular.z);

        cmdVelBuffer_.push_back(cmdVelNorm);
        float cmdVelBufferSum = std::accumulate(cmdVelBuffer_.begin(), cmdVelBuffer_.end(), float(0));
        
        bool keepPlanning = cmdVelBufferSum > 1.0 || !cmdVelBuffer_.full();
        
        if ((trajFlag == GAP || trajFlag == UNGAP) && !keepPlanning && !cfg_.ctrl.man_ctrl) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "--------------------------Planning Failed--------------------------");
            ROS_INFO_STREAM_NAMED("Planner", "--------------------------Planning Failed--------------------------");
            reset();
        }
        return keepPlanning || cfg_.ctrl.man_ctrl;
    }

    // Run after computeAverageTimeTaken, we are using planningLoopCalls
    float Planner::computeAverageNumberGaps(const int & gapCount)
    {
        totalNumGaps += gapCount;
        return (totalNumGaps / (float) planningLoopCalls);
    }

    // 0: gap detection
    // 1: gap simplification
    // 2: gap association
    // 3: gap estimation

    // 4: gap propagation
    // 5: gap manipulation
    // 6: gap feasibility
    // 7: scan propagation
    // 8: gap traj generation
    // 9: gap comparison
    // 10: total planning loop

    // 11: feedback control
    // 12: projection operator
    float Planner::computeAverageTimeTaken(const float & timeTaken, const int & planningStepIdx)
    {
        float averageTimeTaken = 0.0f;
        switch(planningStepIdx)
        {
            case GAP_DET:
                totalGapDetectionTimeTaken += timeTaken;
                gapDetectionCalls++;
                averageTimeTaken = (totalGapDetectionTimeTaken / gapDetectionCalls);
                break;
            case GAP_SIMP:
                totalGapSimplificationTimeTaken += timeTaken;
                gapSimplificationCalls++;
                averageTimeTaken = (totalGapSimplificationTimeTaken / gapSimplificationCalls);
                break;
            case GAP_ASSOC:
                totalGapAssociationCheckTimeTaken += timeTaken;
                gapAssociationCheckCalls++;
                averageTimeTaken = (totalGapAssociationCheckTimeTaken / gapAssociationCheckCalls);
                break;                
            case GAP_EST:
                totalGapEstimationTimeTaken += timeTaken;
                gapEstimationCalls++;
                averageTimeTaken = (totalGapEstimationTimeTaken / gapEstimationCalls);
                break;      
            case SCAN:
                totalScanningTimeTaken += timeTaken;
                scanningLoopCalls++;
                averageTimeTaken = (totalScanningTimeTaken / scanningLoopCalls);
                break;   
            case GAP_PROP:
                totalGapPropagationTimeTaken += timeTaken;
                gapPropagationCalls++;
                averageTimeTaken = (totalGapPropagationTimeTaken / gapPropagationCalls);
                break;                   
            case GAP_MANIP:
                totalGapManipulationTimeTaken += timeTaken;
                gapManipulationCalls++;
                averageTimeTaken = (totalGapManipulationTimeTaken / gapManipulationCalls);
                break;                                                 
            case GAP_FEAS:
                totalGapFeasibilityCheckTimeTaken += timeTaken;
                gapFeasibilityCheckCalls++;
                averageTimeTaken = (totalGapFeasibilityCheckTimeTaken / gapFeasibilityCheckCalls);
                break;   
            case SCAN_PROP:
                totalScanPropagationTimeTaken += timeTaken;
                scanPropagationCalls++;
                averageTimeTaken = (totalScanPropagationTimeTaken / scanPropagationCalls);
                break;    
            case GAP_TRAJ_GEN:
                totalGenerateGapTrajTimeTaken += timeTaken;
                generateGapTrajCalls++;
                averageTimeTaken = (totalGenerateGapTrajTimeTaken / generateGapTrajCalls);
                break;
            case UNGAP_TRAJ_GEN:
                totalGenerateUngapTrajTimeTaken += timeTaken;
                generateUngapTrajCalls++;
                averageTimeTaken = (totalGenerateUngapTrajTimeTaken / generateUngapTrajCalls);
                break;
            case IDLING_TRAJ_GEN:
                totalGenerateIdlingTrajTimeTaken += timeTaken;
                generateIdlingTrajCalls++;
                averageTimeTaken = (totalGenerateIdlingTrajTimeTaken / generateIdlingTrajCalls);
                break;                                
            case TRAJ_PICK:
                totalSelectGapTrajTimeTaken += timeTaken;
                selectGapTrajCalls++;
                averageTimeTaken = (totalSelectGapTrajTimeTaken / selectGapTrajCalls);
                break;                                                        
            case TRAJ_COMP:
                totalCompareToCurrentTrajTimeTaken += timeTaken;
                compareToCurrentTrajCalls++;
                averageTimeTaken = (totalCompareToCurrentTrajTimeTaken / compareToCurrentTrajCalls);
                break;       
            case PLAN:
                totalPlanningTimeTaken += timeTaken;
                planningLoopCalls++;
                averageTimeTaken = (totalPlanningTimeTaken / planningLoopCalls);
                break; 
            case FEEBDACK:
                totalFeedbackControlTimeTaken += timeTaken;
                feedbackControlCalls++;
                averageTimeTaken = (totalFeedbackControlTimeTaken / feedbackControlCalls);
                break;      
            case PO:
                totalProjectionOperatorTimeTaken += timeTaken;
                projectionOperatorCalls++;
                averageTimeTaken = (totalProjectionOperatorTimeTaken / projectionOperatorCalls);
                break;       
            case CONTROL:
                totalControlTimeTaken += timeTaken;
                controlCalls++;
                averageTimeTaken = (totalControlTimeTaken / controlCalls);
                break;                                                                   
        }

        return averageTimeTaken;
    }
}