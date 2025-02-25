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
        delete gapAssociator_;
        delete gapVisualizer_;

        delete globalPlanManager_;
        delete goalVisualizer_;

        delete gapFeasibilityChecker_;

        delete gapManipulator_;

        delete gapTrajGenerator_;

        delete dynamicScanPropagator_;

        delete trajEvaluator_;
        delete trajController_;
        delete trajVisualizer_;
    }

    bool Planner::initialize(const std::string & name)
    {
        // ROS_INFO_STREAM("starting initialize");
        if (initialized_)
        {
            ROS_WARN("DynamicGap Planner already initalized");
            return true;
        }

        // Config Setup
        cfg_.loadRosParamFromNodeHandle(name);

        ROS_INFO_STREAM("cfg_.scan_topic: " << cfg_.scan_topic);
        ROS_INFO_STREAM("cfg_.odom_topic: " << cfg_.odom_topic);
        ROS_INFO_STREAM("cfg_.acc_topic: " << cfg_.acc_topic);
        ROS_INFO_STREAM("cfg_.ped_topic: " << cfg_.ped_topic);

        ROS_INFO_STREAM("cfg_.robot_frame_id: " << cfg_.robot_frame_id);
        ROS_INFO_STREAM("cfg_.sensor_frame_id: " << cfg_.sensor_frame_id);
        ROS_INFO_STREAM("cfg_.map_frame_id: " << cfg_.map_frame_id);
        ROS_INFO_STREAM("cfg_.odom_frame_id: " << cfg_.odom_frame_id);

        // Initialize everything
        gapDetector_ = new GapDetector(cfg_);
        gapAssociator_ = new GapAssociator(cfg_);

        globalPlanManager_ = new GlobalPlanManager(cfg_);

        gapFeasibilityChecker_ = new GapFeasibilityChecker(cfg_);

        gapManipulator_ = new GapManipulator(cfg_);

        gapTrajGenerator_ = new GapTrajectoryGenerator(cfg_);

        dynamicScanPropagator_ = new DynamicScanPropagator(nh_, cfg_); 

        trajEvaluator_ = new TrajectoryEvaluator(cfg_);
        trajController_ = new TrajectoryController(nh_, cfg_);

        gapVisualizer_ = new GapVisualizer(nh_, cfg_);
        goalVisualizer_ = new GoalVisualizer(nh_, cfg_);
        trajVisualizer_ = new TrajectoryVisualizer(nh_, cfg_);

        // TF Lookup setup
        tfListener_ = new tf2_ros::TransformListener(tfBuffer_);        
        
        tfSub_ = nh_.subscribe("/tf", 10, &Planner::tfCB, this);

        rbtPoseSub_.subscribe(nh_, cfg_.odom_topic, 10);
        rbtAccSub_.subscribe(nh_, cfg_.acc_topic, 10);
        sync_.reset(new CustomSynchronizer(rbtPoseAndAccSyncPolicy(10), rbtPoseSub_, rbtAccSub_));
        sync_->registerCallback(boost::bind(&Planner::jointPoseAccCB, this, _1, _2));

        // Robot laser scan message subscriber
        ROS_INFO_STREAM("before laserSub_");
        laserSub_ = nh_.subscribe(cfg_.scan_topic, 5, &Planner::laserScanCB, this);
        // ROS_INFO_STREAM("after laserSub_");

        pedOdomSub_ = nh_.subscribe(cfg_.ped_topic, 10, &Planner::pedOdomCB, this);

        // Visualization Setup
        currentTrajectoryPublisher_ = nh_.advertise<geometry_msgs::PoseArray>("curr_exec_dg_traj", 1);

        mpcInputPublisher_ = nh_.advertise<geometry_msgs::PoseArray>("mpc_input", 1);
        mpcOutputSubscriber_ = nh_.subscribe("mpc_output", 1, &Planner::mpcOutputCB, this);

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
        gapAssociator_->updateParams(estParams);
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

    void Planner::mpcOutputCB(boost::shared_ptr<geometry_msgs::PoseArray> mpcOutput)
    {
        if (mpcOutput->poses.size() == 0)
        {
            ROS_WARN_STREAM("MPC output length zero");
            return;
        }

        if (mpcOutput->poses.size() > 1)
        {
            // first entry is initial condition

            // second entry should be what we want
            geometry_msgs::Twist dummyTwist;
            dummyTwist.linear.x = mpcOutput->poses[1].position.x;
            dummyTwist.linear.y = mpcOutput->poses[1].position.y;
            mpcTwist_ = dummyTwist;
        } else
        {
            ROS_WARN_STREAM("MPC output length one");
        }
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan> scan)
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("Planner", "[laserScanCB()]");
        ROS_INFO_STREAM("       timestamp: " << scan->header.stamp);

        std::chrono::steady_clock::time_point scanStartTime = std::chrono::steady_clock::now();
        
        /////////////////////////////////////
        //////// SCAN PRE-PROCESSING ////////
        /////////////////////////////////////

        gapDetector_->preprocessScan(scan);

        scan_ = scan;

        float minScanDist = *std::min_element(scan_->ranges.begin(), scan_->ranges.end());

        if (minScanDist < cfg_.rbt.r_inscr)
        {
            ROS_INFO_STREAM("       in collision!");
            colliding_ = true;
        } else
        {
            colliding_ = false;
        }

        cfg_.updateParamFromScan(scan_);

        // ROS_INFO_STREAM("scan: " << *scan_);

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
        rawDistMatrix_ = gapAssociator_->obtainDistMatrix(currRawGaps_, prevRawGaps_);
        rawAssocation_ = gapAssociator_->associateGaps(rawDistMatrix_);
        gapAssociator_->assignModels(rawAssocation_, rawDistMatrix_, 
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
        simpDistMatrix_ = gapAssociator_->obtainDistMatrix(currSimplifiedGaps_, prevSimplifiedGaps_);
        simpAssociation_ = gapAssociator_->associateGaps(simpDistMatrix_); // must finish this and therefore change the association
        gapAssociator_->assignModels(simpAssociation_, simpDistMatrix_, 
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
                                std::vector<Gap *> & gaps, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                                const ros::Time & tCurrentFilterUpdate) 
    {
		try
        {
            Gap * gap = gaps[int(idx / 2.0)];
    
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

        try
        {
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

        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("Planner", "jointPoseAccCB failed");
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

            try 
            {
                agentPoseMsgFrame.header = agentIState.header;
                agentPoseMsgFrame.pose = agentIState.pose;
                // ROS_INFO_STREAM_NAMED("Planner", "      incoming pose: (" << agentPoseMsgFrame.pose.position.x << ", " << agentPoseMsgFrame.pose.position.y << ")");

                //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;
                tf2::doTransform(agentPoseMsgFrame, agentPoseRobotFrame, msgFrame2RobotFrame);
                
                // ROS_INFO_STREAM_NAMED("Planner", "      outgoing pose: (" << agentPoseRobotFrame.pose.position.x << ", " << agentPoseRobotFrame.pose.position.y << ")");

                // ROS_INFO_STREAM("updating " << agentNamespace << " odom from " << agent_odom_vects.at(agentID)[0] << ", " << agent_odom_vects.at(agentID)[1] << " to " << odom_vect[0] << ", " << odom_vect[1]);
                currentTrueAgentPoses_[agentIState.id] = agentPoseRobotFrame.pose;
            } catch (...) 
            {
                ROS_WARN_STREAM_NAMED("Planner", "pedOdomCB odometry failed for " << agentIState.id);
            }
            
            try 
            {
                // std::cout << "in agentOdomCB" << std::endl;
                // std::cout << "transforming from " << source_frame << " to " << cfg_.robot_frame_id << std::endl;
                agentVelMsgFrame.header = agentIState.header;
                // agentVelMsgFrame.header.frame_id = source_frame; // TODO: determine if frame for position is same as frame for velocity
                agentVelMsgFrame.vector = agentIState.twist.linear;
                // std::cout << "incoming vector: " << agentVelMsgFrame.vector.x << ", " << agentVelMsgFrame.vector.y << std::endl;
                tf2::doTransform(agentVelMsgFrame, agentVelRobotFrame, msgFrame2RobotFrame);
                // std::cout << "outcoming vector: " << agentVelRobotFrame.vector.x << ", " << agentVelRobotFrame.vector.y << std::endl;

                currentTrueAgentVels_[agentIState.id] = agentVelRobotFrame;
            } catch (...) 
            {
                ROS_WARN_STREAM_NAMED("Planner", "pedOdomCB velocity failed for " << agentIState.id);
            }            
        }
    }

    bool Planner::setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        if (globalPlanMapFrame.size() == 0) 
            return true;
        
        if (!haveTFs_)
            return false;

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
        trajEvaluator_->transformGlobalPathLocalWaypointToRbtFrame(globalPathLocalWaypointOdomFrame_, odom2rbt_);

        // Visualization only
        std::vector<geometry_msgs::PoseStamped> visibleGlobalPlanSnippetRobotFrame = globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
        trajVisualizer_->drawRelevantGlobalPlanSnippet(visibleGlobalPlanSnippetRobotFrame);

        hasGlobalGoal_ = true;
        setReachedGlobalGoal(false);

        return true;
    }

    void Planner::tfCB(const tf2_msgs::TFMessage& msg)
    {
        // ignoring message entirely, just making separate thread for tfs to not
        // tether to other callbacks

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

            haveTFs_ = true;

            tf2::doTransform(rbtPoseInRbtFrame_, rbtPoseInSensorFrame_, rbt2cam_);

            // ROS_INFO_STREAM("tfCB succeeded");
        } catch (...) 
        {
            ROS_WARN_STREAM("tfCB failed");
            ros::Duration(0.1).sleep();
            return;
        }
    }

    void Planner::updateEgoCircle()
    {
        globalPlanManager_->updateEgoCircle(scan_);
        gapManipulator_->updateEgoCircle(scan_);
        gapFeasibilityChecker_->updateEgoCircle(scan_);
        dynamicScanPropagator_->updateEgoCircle(scan_);
        trajEvaluator_->updateEgoCircle(scan_);
        trajController_->updateEgoCircle(scan_);
    }

    void Planner::propagateGapPoints(const std::vector<Gap *> & planningGaps)                                             
    {
        boost::mutex::scoped_lock gapset(gapMutex_);        
        ROS_INFO_STREAM_NAMED("GapFeasibility", "[propagateGapPoints()]");

        if (planningGaps.empty())
        {
            ROS_WARN_STREAM_NAMED("GapFeasibility", "    planningGaps is empty, returning");
            return;
        }

        try
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "    current raw gaps:");
            printGapModels(currRawGaps_);

            ROS_INFO_STREAM_NAMED("GapFeasibility", "    current simplified gaps:");
            printGapModels(planningGaps);
            
            for (size_t i = 0; i < planningGaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("GapFeasibility", "   gap " << i);
                // propagate gap forward in time to determine lifespan
                gapFeasibilityChecker_->propagateGapPoints(planningGaps.at(i));
            }

        } catch(...)
        {
            ROS_WARN_STREAM_NAMED("GapFeasibility", "   propagateGapPoints failed");
        }

        return;

    }

    std::vector<Gap *> Planner::manipulateGaps(const std::vector<Gap *> & planningGaps) 
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "[manipulateGaps()]");

        boost::mutex::scoped_lock gapset(gapMutex_);
        std::vector<Gap *> manipulatedGaps;

        try
        {
            for (size_t i = 0; i < planningGaps.size(); i++)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "    manipulating initial gap " << i);

                // MANIPULATE POINTS AT T=0            
                gapManipulator_->convertRadialGap(planningGaps.at(i));

                bool success = gapManipulator_->inflateGapSides(planningGaps.at(i));
                
                if (success)
                {
                    ROS_INFO_STREAM_NAMED("GapManipulator", "    pushing back manipulated gap " << i);
                    
                    gapManipulator_->setGapGoal(planningGaps.at(i), 
                                                globalPlanManager_->getGlobalPathLocalWaypointRobotFrame(),
                                                globalGoalRobotFrame_);                    

                    manipulatedGaps.push_back(planningGaps.at(i)); // shallow copy
                }
            }
        } catch (...)
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "   gapManipulate failed");
            ROS_WARN_STREAM_NAMED("GapManipulator", "   gapManipulate failed");
        }

        return manipulatedGaps;
    }

    std::vector<Gap *> Planner::gapSetFeasibilityCheck(const std::vector<Gap *> & manipulatedGaps, 
                                                                    bool & isCurrentGapFeasible)                                             
    {
        boost::mutex::scoped_lock gapset(gapMutex_);        
        ROS_INFO_STREAM_NAMED("GapFeasibility", "[gapSetFeasibilityCheck()]");
        std::vector<Gap *> feasibleGaps;

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
                    feasibleGaps.push_back(manipulatedGaps.at(i)); // shallow copy

                    // feasibleGaps.push_back(new Gap(*manipulatedGaps.at(i)));
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

    void Planner::generateGapTrajs(std::vector<Gap *> & gaps, 
                                    std::vector<Trajectory> & generatedTrajs,
                                    std::vector<std::vector<float>> & pathPoseCosts,
                                    std::vector<float> & pathTerminalPoseCosts,
                                    const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[generateGapTrajs()]");
        
        pathPoseCosts = std::vector<std::vector<float>>(gaps.size());
        pathTerminalPoseCosts = std::vector<float>(gaps.size());

        try 
        {
            for (size_t i = 0; i < gaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    generating traj for gap: " << i);
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                
                // Run go to goal behavior
                bool runGoToGoal = (gaps.at(i)->globalGoalWithin);

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
                    goToGoalTraj = gapTrajGenerator_->processTrajectory(goToGoalTraj, true);
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

                pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj, true);
                trajEvaluator_->evaluateTrajectory(pursuitGuidanceTraj, pursuitGuidancePoseCosts, pursuitGuidanceTerminalPoseCost, futureScans);
                pursuitGuidancePoseCost = pursuitGuidanceTerminalPoseCost + std::accumulate(pursuitGuidancePoseCosts.begin(), pursuitGuidancePoseCosts.end(), float(0));
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        pursuitGuidancePoseCost: " << pursuitGuidancePoseCost);

                if (runGoToGoal && goToGoalCost < pursuitGuidancePoseCost)
                {
                    traj = goToGoalTraj;
                    pathPoseCosts.at(i) = goToGoalPoseCosts;
                    pathTerminalPoseCosts.at(i) = goToGoalTerminalPoseCost;
                } else
                {
                    traj = pursuitGuidanceTraj;
                    pathPoseCosts.at(i) = pursuitGuidancePoseCosts;
                    pathTerminalPoseCosts.at(i) = pursuitGuidanceTerminalPoseCost;
                }

                // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                traj.setPathOdomFrame(gapTrajGenerator_->transformPath(traj.getPathRbtFrame(), cam2odom_));
                // pathTimings.at(i) = std::get<1>(traj);
                generatedTrajs.push_back(traj);
            }

            // push back idling trajectory as another option
            std::vector<float> idlingPoseCosts;
            float idlingTerminalPoseCost, idlingCost;

            Trajectory idlingTrajectory;
            idlingTrajectory = gapTrajGenerator_->generateIdlingTrajectory(rbtPoseInOdomFrame_);
            
            idlingTrajectory = gapTrajGenerator_->processTrajectory(idlingTrajectory, false);
            trajEvaluator_->evaluateTrajectory(idlingTrajectory, idlingPoseCosts, idlingTerminalPoseCost, futureScans);
            idlingCost = idlingTerminalPoseCost + std::accumulate(idlingPoseCosts.begin(), idlingPoseCosts.end(), float(0));
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        idlingCost: " << idlingCost);

            pathPoseCosts.push_back(idlingPoseCosts);
            pathTerminalPoseCosts.push_back(idlingTerminalPoseCost);

            idlingTrajectory.setPathOdomFrame(gapTrajGenerator_->transformPath(idlingTrajectory.getPathRbtFrame(), cam2odom_));
            generatedTrajs.push_back(idlingTrajectory);            

        } catch (...) 
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "generateGapTrajs failed");
        }

        trajVisualizer_->drawGapTrajectoryPoseScores(generatedTrajs, pathPoseCosts);
        trajVisualizer_->drawGapTrajectories(generatedTrajs);

        return;
    }

    int Planner::pickTraj(const std::vector<Trajectory> & trajs, 
                          const std::vector<std::vector<float>> & pathPoseCosts, 
                          const std::vector<float> & pathTerminalPoseCosts) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[pickTraj()]");
        
        int lowestCostTrajIdx = -1;        
        
        try
        {
            // ROS_INFO_STREAM_NAMED("pg_trajCount", "pg_trajCount, " << paths.size());
            if (trajs.size() == 0) 
            {
                ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "No traj synthesized");
                return -1;
            }

            if (trajs.size() != pathPoseCosts.size()) 
            {
                ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "pickTraj size mismatch: paths = " << trajs.size() << " != pathPoseCosts =" << pathPoseCosts.size());
                return -1;
            }

            // poses here are in odom frame 
            std::vector<float> pathCosts(trajs.size());

            for (size_t i = 0; i < pathCosts.size(); i++) 
            {
                // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());

                pathCosts.at(i) = pathTerminalPoseCosts.at(i) + std::accumulate(pathPoseCosts.at(i).begin(), 
                                                                                  pathPoseCosts.at(i).end(), float(0));
                
                pathCosts.at(i) = trajs.at(i).getPathRbtFrame().poses.size() == 0 ? std::numeric_limits<float>::infinity() : pathCosts.at(i);
                
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    for gap " << i << " (length: " << trajs.at(i).getPathRbtFrame().poses.size() << "), returning cost of " << pathCosts.at(i));
                
            }

            auto lowestCostTrajIterIter = std::min_element(pathCosts.begin(), pathCosts.end());
            lowestCostTrajIdx = std::distance(pathCosts.begin(), lowestCostTrajIterIter);

            if (pathCosts.at(lowestCostTrajIdx) == std::numeric_limits<float>::infinity()) 
            {    
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    all infinity");
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "No executable trajectory, values: ");
                for (float pathCost : pathCosts) 
                {
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Cost: " << pathCost);
                }
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "------------------");
            }

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    picking gap: " << lowestCostTrajIdx);
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "   pickTraj failed");
        }
        return lowestCostTrajIdx;
    }

    Trajectory Planner::changeTrajectoryHelper(Gap * incomingGap, 
                                                             const Trajectory & incomingTraj, 
                                                             const bool & switchToIncoming) 
    {
        publishToMpc_ = true;
        
        if (switchToIncoming) 
        {
            setCurrentTraj(incomingTraj);
            if (incomingGap) // actual trajectory
            {
                setCurrentLeftGapPtModelID(incomingGap->leftGapPtModel_);
                setCurrentRightGapPtModelID(incomingGap->rightGapPtModel_);
            } else // idling trajectory
            {
                setCurrentLeftGapPtModelID(nullptr);
                setCurrentRightGapPtModelID(nullptr);
            }
            // currentInterceptTime_ = incomingGap->t_intercept;
            // currentMinSafeDist_ = incomingGap->minSafeDist_;

            currentTrajectoryPublisher_.publish(incomingTraj.getPathRbtFrame());          
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, incomingTraj);
            trajectoryChangeCount_++;

            return incomingTraj;  
        } else 
        {
            Trajectory emptyTraj;
            geometry_msgs::PoseArray emptyPath = geometry_msgs::PoseArray();
            emptyPath.header = incomingTraj.getPathRbtFrame().header;
            std::vector<float> emptyPathTiming;
            setCurrentTraj(emptyTraj);
            setCurrentLeftGapPtModelID(nullptr);
            setCurrentRightGapPtModelID(nullptr);
            // currentInterceptTime_ = 0.0;
            // currentMinSafeDist_ = 0.0;

            currentTrajectoryPublisher_.publish(emptyTraj.getPathRbtFrame());
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, emptyTraj);
            trajectoryChangeCount_++;            

            return emptyTraj;
        }        
    }

    Trajectory Planner::compareToCurrentTraj(const std::vector<Gap *> & feasibleGaps, 
                                                            const std::vector<Trajectory> & trajs,
                                                            const int & lowestCostTrajIdx,
                                                            const int & trajFlag,
                                                            const bool & isIncomingGapFeasible,
                                                            const std::vector<sensor_msgs::LaserScan> & futureScans) // bool isIncomingGapAssociated,
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "[compareToCurrentTraj()]");
        boost::mutex::scoped_lock gapset(gapMutex_);
        
        Gap * incomingGap = (trajFlag == 0 ? feasibleGaps.at(lowestCostTrajIdx) : nullptr);
        Trajectory incomingTraj = trajs.at(lowestCostTrajIdx);
        Trajectory currentTraj = getCurrentTraj();

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

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    evaluating incoming trajectory");
            std::vector<float> incomingPathPoseCosts;
            float incomingPathTerminalPoseCost;
            trajEvaluator_->evaluateTrajectory(incomingTraj, incomingPathPoseCosts, incomingPathTerminalPoseCost, futureScans);

            float incomingPathCost = incomingPathTerminalPoseCost + std::accumulate(incomingPathPoseCosts.begin(), incomingPathPoseCosts.end(), float(0));
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    incoming trajectory received a cost of: " << incomingPathCost);
          

            ///////////////////////////////////////////////////////////////////////
            //  Evaluate the incoming path to determine if we can switch onto it //
            ///////////////////////////////////////////////////////////////////////
            std::string incomingPathStatus = "incoming path is safe to switch onto"; // (incomingPathOdomFrame.poses.size() > 0) ? "incoming traj length 0" : "incoming cost infinite";

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
            //  Compare the costs of the incoming trajectory with the cost of the current trajectory to see if we need to switch //
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            int poseCheckCount = std::min(incomingTraj.getPathRbtFrame().poses.size(), reducedCurrentPathRobotFrame.poses.size());

            incomingPathCost = std::accumulate(incomingPathPoseCosts.begin(), incomingPathPoseCosts.begin() + poseCheckCount, float(0));

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    re-evaluating incoming trajectory received a subcost of: " << incomingPathCost);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    evaluating current trajectory");            
            
            Trajectory reducedCurrentTraj(reducedCurrentPathRobotFrame, reducedCurrentPathTiming);
            std::vector<float> currentPathPoseCosts;
            float currentPathTerminalPoseCost;
            trajEvaluator_->evaluateTrajectory(reducedCurrentTraj, currentPathPoseCosts, currentPathTerminalPoseCost, futureScans);
            float currentPathSubCost = currentPathTerminalPoseCost + std::accumulate(currentPathPoseCosts.begin(), currentPathPoseCosts.begin() + poseCheckCount, float(0));
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "    current trajectory received a subcost of: " << currentPathSubCost);

            std::vector<std::vector<float>> pathPoseCosts(2);
            pathPoseCosts.at(0) = incomingPathPoseCosts;
            pathPoseCosts.at(1) = currentPathPoseCosts;

            
            if (currentPathSubCost == std::numeric_limits<float>::infinity()) 
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

        std::chrono::steady_clock::time_point planningLoopStartTime = std::chrono::steady_clock::now();

        ///////////////////////////
        // GAP POINT PROPAGATION //
        ///////////////////////////
        int gapCount = planningGaps.size();

        std::chrono::steady_clock::time_point gapPropagateStartTime = std::chrono::steady_clock::now();
        propagateGapPoints(planningGaps);
        float gapPropagateTimeTaken = timeTaken(gapPropagateStartTime);
        float avgGapPropagationTimeTaken = computeAverageTimeTaken(gapPropagateTimeTaken, GAP_PROP);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Propagation for " << gapCount << " gaps took " << gapPropagateTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Propagation average time: " << avgGapPropagationTimeTaken << " seconds (" << (1.0 / avgGapPropagationTimeTaken) << " Hz) ]");

        //////////////////////
        // GAP MANIPULATION //
        //////////////////////
        std::chrono::steady_clock::time_point manipulateGapsStartTime = std::chrono::steady_clock::now();
        std::vector<Gap *> manipulatedGaps = manipulateGaps(planningGaps);
        float gapManipulationTimeTaken = timeTaken(manipulateGapsStartTime);
        float avgGapManipulationTimeTaken = computeAverageTimeTaken(gapManipulationTimeTaken, GAP_MANIP);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Manipulation for " << gapCount << " gaps took " << gapManipulationTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Manipulation average time: " << avgGapManipulationTimeTaken << " seconds (" << (1.0 / avgGapManipulationTimeTaken) << " Hz) ]");

        ///////////////////////////
        // GAP FEASIBILITY CHECK //
        ///////////////////////////
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
                // feasibleGaps.push_back(new Gap(*currSimplifiedGap));
            // TODO: need to set feasible to true for all gaps as well
        }
        float feasibilityTimeTaken = timeTaken(feasibilityStartTime);
        float avgFeasibilityTimeTaken = computeAverageTimeTaken(feasibilityTimeTaken, GAP_FEAS);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Feasibility Analysis for " << gapCount << " gaps took " << feasibilityTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Feasibility Analysis average time: " << avgFeasibilityTimeTaken << " seconds (" << (1.0 / avgFeasibilityTimeTaken) << " Hz) ]");

        // Have to run here because terminal gap goals are set during feasibility check
        gapVisualizer_->drawManipGaps(manipulatedGaps, std::string("manip"));
        
        /*
            // gapManipulator_->radialExtendGap(manipulatedGaps.at(i)); // to set s
            gapManipulator_->setGapGoal(planningGaps.at(i), 
                                        globalPlanManager_->getGlobalPathLocalWaypointRobotFrame(),
                                        globalGoalRobotFrame_);

            Eigen::Vector2f terminalGoal = p_target + v_target * gap->t_intercept;

            // clip at scan
            float terminalGoalTheta = std::atan2(terminalGoal[1], terminalGoal[0]);
            int terminalGoalScanIdx = theta2idx(terminalGoalTheta);

            // if terminal goal lives beyond scan
            if (scan_->ranges.at(terminalGoalScanIdx) < (terminalGoal.norm() + cfg_->traj.max_pose_to_scan_dist))
            {
                float newTerminalGoalRange = scan_->ranges.at(terminalGoalScanIdx) - cfg_->traj.max_pose_to_scan_dist;
                terminalGoal << newTerminalGoalRange * std::cos(terminalGoalTheta),
                                newTerminalGoalRange * std::sin(terminalGoalTheta);
            }


            gap->setTerminalGoal(terminalGoal);

        */
        
        
        goalVisualizer_->drawGapGoals(manipulatedGaps);

        gapCount = feasibleGaps.size();

        /////////////////////////////
        // FUTURE SCAN PROPAGATION //
        /////////////////////////////
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
    
        ///////////////////////////////////////////
        // GAP TRAJECTORY GENERATION AND SCORING //
        ///////////////////////////////////////////
        std::vector<Trajectory> trajs;
        std::vector<std::vector<float>> pathPoseCosts; 
        std::vector<float> pathTerminalPoseCosts; 
        std::chrono::steady_clock::time_point generateGapTrajsStartTime = std::chrono::steady_clock::now();
        generateGapTrajs(feasibleGaps, trajs, pathPoseCosts, pathTerminalPoseCosts, futureScans);
        float generateGapTrajsTimeTaken = timeTaken(generateGapTrajsStartTime);
        float avgGenerateGapTrajsTimeTaken = computeAverageTimeTaken(generateGapTrajsTimeTaken, TRAJ_GEN);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Generation for " << gapCount << " gaps took " << generateGapTrajsTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Generation average time: " << avgGenerateGapTrajsTimeTaken << " seconds (" << (1.0 / avgGenerateGapTrajsTimeTaken) << " Hz) ]");
    
        //////////////////////////////
        // GAP TRAJECTORY SELECTION //
        //////////////////////////////
        std::chrono::steady_clock::time_point pickTrajStartTime = std::chrono::steady_clock::now();
        int lowestCostTrajIdx = pickTraj(trajs, pathPoseCosts, pathTerminalPoseCosts);
        float pickTrajTimeTaken = timeTaken(pickTrajStartTime);
        float avgPickTrajTimeTaken = computeAverageTimeTaken(pickTrajTimeTaken, TRAJ_PICK);
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Selection for " << gapCount << " gaps took " << pickTrajTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Selection average time: " << avgPickTrajTimeTaken << " seconds (" << (1.0 / avgPickTrajTimeTaken) << " Hz) ]");
    
        if (lowestCostTrajIdx >= 0) 
        {
            ///////////////////////////////
            // GAP TRAJECTORY COMPARISON //
            ///////////////////////////////
            std::chrono::steady_clock::time_point compareToCurrentTrajStartTime = std::chrono::steady_clock::now();

            if (lowestCostTrajIdx < feasibleGaps.size()) // comparing to traj within one of the gaps
            {
                ROS_INFO_STREAM("       comparing to actual trajectory");
                trajFlag = 0;
            } else // comparing to idling traj
            {
                ROS_INFO_STREAM("       comparing to idling trajectory");
                trajFlag = 1;               
            }

            chosenTraj = compareToCurrentTraj(feasibleGaps, 
                                            trajs,
                                            lowestCostTrajIdx,
                                            trajFlag,
                                            isCurrentGapFeasible,
                                            futureScans);

            float compareToCurrentTrajTimeTaken = timeTaken(compareToCurrentTrajStartTime);
            float avgCompareToCurrentTrajTimeTaken = computeAverageTimeTaken(compareToCurrentTrajTimeTaken, TRAJ_COMP);        

            ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Comparison for " << gapCount << " gaps took " << compareToCurrentTrajTimeTaken << " seconds]");
            ROS_INFO_STREAM_NAMED("Timing", "       [Gap Trajectory Comparison average time: " << avgCompareToCurrentTrajTimeTaken << " seconds (" << (1.0 / avgCompareToCurrentTrajTimeTaken) << " Hz) ]");
        } 

        // publish for safety module
        // if (publishToMpc_)
        // {    
        //     publishToMpc_ = false;
        //     geometry_msgs::PoseArray mpcInput;
        //     mpcInput.header = chosenTraj.getPathRbtFrame().header;
            
        //     // current rbt vel
        //     geometry_msgs::Pose currentRbtVelAsPose;
        //     currentRbtVelAsPose.position.x = currentRbtVel_.twist.linear.x;
        //     currentRbtVelAsPose.position.y = currentRbtVel_.twist.linear.y;
        //     mpcInput.poses.push_back(currentRbtVelAsPose);

        //     // three slice points            

        //     // where to get t_intercept
        //     Eigen::Vector2f terminalLeftPt = currentLeftGapPtState.head(2) + currentLeftGapPtState.tail(2) * currentInterceptTime_;
        //     Eigen::Vector2f terminalRightPt = currentRightGapPtState.head(2) + currentRightGapPtState.tail(2) * currentInterceptTime_;

        //     // get center bearing
        //     Eigen::Vector2f terminalLeftBearingVect = terminalLeftPt / terminalLeftPt.norm();
        //     float terminalThetaLeft = std::atan2(terminalLeftPt[1], terminalLeftPt[0]);
        //     Eigen::Vector2f terminalRightBearingVect = terminalRightPt / terminalRightPt.norm();
        //     float leftToRightAngle = getSweptLeftToRightAngle(terminalLeftBearingVect, terminalRightBearingVect);
        //     float thetaCenter = (terminalThetaLeft - 0.5 * leftToRightAngle);

        //     Eigen::Vector2f centralBearingVect(std::cos(thetaCenter), std::sin(thetaCenter));
                    
        //     // take opposite direction and scale by r_min
        //     Eigen::Vector2f terminalRadialPt = - currentMinSafeDist_ * centralBearingVect;

        //     geometry_msgs::Pose terminalRadialPtAsPose;
        //     terminalRadialPtAsPose.position.x = terminalRadialPt[0];
        //     terminalRadialPtAsPose.position.y = terminalRadialPt[1];
        //     mpcInput.poses.push_back(terminalRadialPtAsPose);

        //     geometry_msgs::Pose terminalRightPtAsPose;
        //     terminalRightPtAsPose.position.x = terminalRightPt[0];
        //     terminalRightPtAsPose.position.y = terminalRightPt[1];
        //     mpcInput.poses.push_back(terminalRightPtAsPose);

        //     geometry_msgs::Pose terminalLeftPtAsPose;
        //     terminalLeftPtAsPose.position.x = terminalLeftPt[0];
        //     terminalLeftPtAsPose.position.y = terminalLeftPt[1];
        //     mpcInput.poses.push_back(terminalLeftPtAsPose);            

        //     // trajectory
        //     for (int k = 0; k < chosenTraj.getPathRbtFrame().poses.size(); k++)
        //         mpcInput.poses.push_back(chosenTraj.getPathRbtFrame().poses[k]);

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

        return;
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
        
        try
        {
            if (trajFlag == 1) // idling
            {
                ROS_INFO_STREAM_NAMED("Planner", "planner opting to idle, no trajectory chosen.");
                rawCmdVel = geometry_msgs::Twist();
                return rawCmdVel;                
            } else if (localTrajectory.poses.size() == 0) // OBSTACLE AVOIDANCE CONTROL 
            { 
                ROS_INFO_STREAM_NAMED("Planner", "Available Execution Traj length: " << localTrajectory.poses.size() << " == 0, obstacle avoidance control chosen.");
                rawCmdVel = trajController_->obstacleAvoidanceControlLaw();
                return rawCmdVel;
            } else if (cfg_.ctrl.man_ctrl)  // MANUAL CONTROL 
            {
                ROS_INFO_STREAM_NAMED("Controller", "Manual control chosen.");
                // rawCmdVel = trajController_->manualControlLawKeyboard();
                rawCmdVel = trajController_->manualControlLawReconfig();
            } else if (cfg_.ctrl.mpc_ctrl) // MPC CONTROL
            { 
                rawCmdVel = mpcTwist_;
            } else if (cfg_.ctrl.feedback_ctrl) // FEEDBACK CONTROL 
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

                std::chrono::steady_clock::time_point feedbackControlStartTime = std::chrono::steady_clock::now();
                rawCmdVel = trajController_->constantVelocityControlLaw(currPoseOdomFrame, targetTrajectoryPose);
                float feedbackControlTimeTaken = timeTaken(feedbackControlStartTime);
                float avgFeedbackControlTimeTaken = computeAverageTimeTaken(feedbackControlTimeTaken, FEEBDACK);        
                ROS_INFO_STREAM_NAMED("Timing", "       [Feedback Control took " << feedbackControlTimeTaken << " seconds]");
                ROS_INFO_STREAM_NAMED("Timing", "       [Feedback Control average time: " << avgFeedbackControlTimeTaken << " seconds (" << (1.0 / avgFeedbackControlTimeTaken) << " Hz) ]");        
            } else
            {
                throw std::runtime_error("No control method selected");
            }

            std::chrono::steady_clock::time_point projOpStartTime = std::chrono::steady_clock::now();
            cmdVel = trajController_->processCmdVel(rawCmdVel,
                                                    rbtPoseInSensorFrame_, 
                                                    currentRbtVel_, currentRbtAcc_); 
            float projOpTimeTaken = timeTaken(projOpStartTime);
            float avgProjOpTimeTaken = computeAverageTimeTaken(projOpTimeTaken, PO);        
            ROS_INFO_STREAM_NAMED("Timing", "       [Projection Operator took " << projOpTimeTaken << " seconds]");
            ROS_INFO_STREAM_NAMED("Timing", "       [Projection Operator average time: " << avgProjOpTimeTaken << " seconds (" << (1.0 / avgProjOpTimeTaken) << " Hz) ]");        

        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("Controller", "ctrlGeneration failed");
        }

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
      
            leftModel->isolateGapDynamics();
            currentLeftGapPtState = leftModel->getGapState(); 
        } else
        {
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentLeftGapPtModelID]: null, setting current left ID to " << -1);
            currentLeftGapPtModelID = -1;
        }
    }

    void Planner::setCurrentRightGapPtModelID(Estimator * rightModel) 
    { 
        if (rightModel) 
        {        
            ROS_INFO_STREAM_NAMED("Planner", "[setCurrentRightGapPtModelID]: setting current right ID to " << rightModel->getID());
            currentRightGapPtModelID = rightModel->getID();

            rightModel->isolateGapDynamics();
            currentRightGapPtState = rightModel->getGapState();             
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

    void Planner::printGapModels(const std::vector<Gap *> & gaps) 
    {
        // THIS IS NOT FOR MANIPULATED GAPS
        float x = 0.0, y = 0.0;
        for (size_t i = 0; i < gaps.size(); i++)
        {
            Gap * gap = gaps.at(i);
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
        
        if (!keepPlanning && !cfg_.ctrl.man_ctrl) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "--------------------------Planning Failed--------------------------");
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
            case TRAJ_GEN:
                totalGenerateGapTrajTimeTaken += timeTaken;
                generateGapTrajCalls++;
                averageTimeTaken = (totalGenerateGapTrajTimeTaken / generateGapTrajCalls);
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