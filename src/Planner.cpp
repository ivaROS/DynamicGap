#include <dynamic_gap/Planner.h>
#include <dynamic_gap/utils/bezier_utils.h>
#include <dynamic_gap/utils/dwa_traj.h>
#include <sstream>
#include <string>

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

        delete agentVisualizer_;

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

        delete timeKeeper_;
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

        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.scan_topic: " << cfg_.scan_topic);
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.odom_topic: " << cfg_.odom_topic);
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.acc_topic: " << cfg_.acc_topic);
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.ped_topic: " << cfg_.ped_topic);

        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.robot_frame_id: " << cfg_.robot_frame_id);
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.sensor_frame_id: " << cfg_.sensor_frame_id);
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.map_frame_id: " << cfg_.map_frame_id);
        // ROS_INFO_STREAM_NAMED("Planner", "cfg_.odom_frame_id: " << cfg_.odom_frame_id);

        // Initialize everything
        gapDetector_ = new GapDetector(cfg_);
        gapPointAssociator_ = new GapPointAssociator(cfg_);

        globalPlanManager_ = new GlobalPlanManager(cfg_);

        gapFeasibilityChecker_ = new GapFeasibilityChecker(cfg_);

        gapPropagator_ = new GapPropagator(nh_, cfg_);

        gapManipulator_ = new GapManipulator(cfg_);

        gapGoalPlacer_ = new GapGoalPlacer(cfg_);

        gapTrajGenerator_ = new GapTrajectoryGenerator(cfg_);

        ungapTrajGenerator_ = new UngapTrajectoryGenerator(cfg_);

        dynamicScanPropagator_ = new DynamicScanPropagator(nh_, cfg_); 

        trajEvaluator_ = new TrajectoryEvaluator(cfg_);
        trajController_ = new TrajectoryController(nh_, cfg_);

        gapVisualizer_ = new GapVisualizer(nh_, cfg_);
        goalVisualizer_ = new GoalVisualizer(nh_, cfg_);
        agentVisualizer_ = new AgentVisualizer(nh_, cfg_);
        trajVisualizer_ = new TrajectoryVisualizer(nh_, cfg_);
        bezierVisualizer_ = new BezierVisualizer(nh_, cfg_);


        ungapFeasibilityChecker_ = new UngapFeasibilityChecker(cfg_);

        timeKeeper_ = new TimeKeeper();

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
        minDistCirclePub_ = nh_.advertise<visualization_msgs::Marker>("min_scan_circle", 1);
        ros::NodeHandle nh;  // or pass an existing NodeHandle
        traj_pub_ = nh.advertise<geometry_msgs::PoseArray>("bezier_traj", 1);
        transformed_path_pub_ = nh_.advertise<visualization_msgs::Marker>("current_traj_transformed_viz", 1);


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

        // prevTrajSwitchTime_ = std::chrono::steady_clock::now();
        // prevIdlingTime_ = std::chrono::steady_clock::now();
        currentTrajTrackingStartTime_ = ros::Time::now();
        currentTrajLifespan_ = ros::Duration(0.0);

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
        reachedGlobalGoal_ = globalGoalDist < cfg_.goal.xy_global_goal_tolerance &&
                             globalGoalAngDist < cfg_.goal.yaw_global_goal_tolerance;
        
        if (reachedGlobalGoal_)
            ROS_INFO_STREAM_NAMED("Planner", "[Reset] Goal Reached");

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

        timeKeeper_->startTimer(SCAN);
        /////////////////////////////////////
        //////// SCAN PRE-PROCESSING ////////
        /////////////////////////////////////

        gapDetector_->preprocessScan(scan);

        scan_ = scan;

        float minScanDist = *std::min_element(scan_->ranges.begin(), scan_->ranges.end());
        min_scan_dist_ = minScanDist;


        if (minScanDist < (cfg_.rbt.r_inscr + 0.0075))
        {
            ROS_INFO_STREAM_NAMED("Planner", "       in collision!");
            ROS_WARN_STREAM_NAMED("Planner", "       in collision!");
            colliding_ = true;
            return;
        } else
        {
            colliding_ = false;
        }

        // publish as circle marker in RViz
        visualization_msgs::Marker circle;
        circle.header.frame_id = cfg_.sensor_frame_id;
        circle.header.stamp = ros::Time::now();
        circle.ns = "min_scan_circle";
        circle.id = 0;
        circle.type = visualization_msgs::Marker::CYLINDER;
        circle.action = visualization_msgs::Marker::ADD;
        circle.pose.position.x = 0.0;
        circle.pose.position.y = 0.0;
        circle.pose.position.z = 0.0;
        circle.pose.orientation.w = 1.0;
        circle.scale.x = 2.0 * minScanDist;
        circle.scale.y = 2.0 * minScanDist;
        circle.scale.z = 0.01;
        circle.color.r = 1.0;
        circle.color.g = 0.0;
        circle.color.b = 0.0;
        circle.color.a = 0.3;
        circle.lifetime = ros::Duration(0.1);
        minDistCirclePub_.publish(circle);

        cfg_.updateParamFromScan(scan_);

        ros::Time tCurrentFilterUpdate = scan_->header.stamp;

        // grabbing current intermediate robot velocities and accelerations
        std::vector<geometry_msgs::TwistStamped> intermediateRbtVels = intermediateRbtVels_;
        std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs = intermediateRbtAccs_;

        ///////////////////////////////
        //////// GAP DETECTION ////////
        ///////////////////////////////
        timeKeeper_->startTimer(GAP_DET);
        currRawGaps_ = gapDetector_->gapDetection(scan_);
        timeKeeper_->stopTimer(GAP_DET);

        /////////////////////////////////////
        //////// RAW GAP ASSOCIATION ////////
        /////////////////////////////////////
        timeKeeper_->startTimer(GAP_ASSOC);
        rawDistMatrix_ = gapPointAssociator_->populateDistMatrix(currRawGaps_, prevRawGaps_);
        rawAssocation_ = gapPointAssociator_->associate(rawDistMatrix_);
        gapPointAssociator_->assignModels(rawAssocation_, rawDistMatrix_, 
                                    currRawGaps_, prevRawGaps_, 
                                    currentModelIdx_, tCurrentFilterUpdate,
                                    intermediateRbtVels, intermediateRbtAccs);
        timeKeeper_->stopTimer(GAP_ASSOC);

        ////////////////////////////////////
        //////// RAW GAP ESTIMATION ////////
        ////////////////////////////////////
        timeKeeper_->startTimer(GAP_EST);
        updateModels(currRawGaps_, intermediateRbtVels, 
                        intermediateRbtAccs, tCurrentFilterUpdate);
        timeKeeper_->stopTimer(GAP_EST);

        ////////////////////////////////////
        //////// GAP SIMPLIFICATION ////////
        ////////////////////////////////////       
        timeKeeper_->startTimer(GAP_SIMP);
        currSimplifiedGaps_ = gapDetector_->gapSimplification(currRawGaps_);
        timeKeeper_->stopTimer(GAP_SIMP);

        ////////////////////////////////////////////
        //////// SIMPLIFIED GAP ASSOCIATION ////////
        ////////////////////////////////////////////
        timeKeeper_->startTimer(GAP_ASSOC);
        simpDistMatrix_ = gapPointAssociator_->populateDistMatrix(currSimplifiedGaps_, prevSimplifiedGaps_);
        simpAssociation_ = gapPointAssociator_->associate(simpDistMatrix_); // must finish this and therefore change the association
        gapPointAssociator_->assignModels(simpAssociation_, simpDistMatrix_, 
                                    currSimplifiedGaps_, prevSimplifiedGaps_, 
                                    currentModelIdx_, tCurrentFilterUpdate,
                                    intermediateRbtVels, intermediateRbtAccs);
        timeKeeper_->stopTimer(GAP_ASSOC);

        ///////////////////////////////////////////
        //////// SIMPLIFIED GAP ESTIMATION ////////
        ///////////////////////////////////////////     
        timeKeeper_->startTimer(GAP_EST);
        updateModels(currSimplifiedGaps_, intermediateRbtVels, 
                        intermediateRbtAccs, tCurrentFilterUpdate);
        timeKeeper_->stopTimer(GAP_EST);

        gapVisualizer_->drawGaps(currRawGaps_, std::string("raw"));
        gapVisualizer_->drawGaps(prevRawGaps_, std::string("raw_tmin1"));
        gapVisualizer_->drawGapModels(currRawGaps_, std::string("raw"));
        gapVisualizer_->drawGaps(currSimplifiedGaps_, std::string("simp"));
        // gapVisualizer_->drawGaps(prevSimplifiedGaps_, std::string("simp_tmin1"));
        gapVisualizer_->drawGapModels(currSimplifiedGaps_, std::string("simp"));

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

        timeKeeper_->stopTimer(SCAN);
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
        
        //just for finding v_dir which is for the dwa_method 
        float vx = currentRbtVel_.twist.linear.x;
float vy = currentRbtVel_.twist.linear.y;
float norm = std::sqrt(vx*vx + vy*vy);
if (norm > 1e-5)  // avoid division by zero
    v_dir_ = Eigen::Vector2f(vx / norm, vy / norm);
else
    v_dir_ = Eigen::Vector2f(1.0, 0.0);  // default forward


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

        agentVisualizer_->drawEgoRobot(rbtPoseInOdomFrame_);

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
        // ROS_INFO_STREAM_NAMED("Planner", "[pedOdomCB()]");   
        
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

        agentVisualizer_->drawPedestrianHistories(currentTrueAgentPoses_, currentTrueAgentVels_, cfg_.robot_frame_id);
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
        
        if (sqrt(pow(diffX, 2) + pow(diffY, 2)) > cfg_.goal.xy_waypoint_tolerance)
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

    // Apply 2D rotation and translation (equivalent to egodwa's BezierTrajectoryGenerator::transform2d)
    Eigen::Vector2f rotatePoint2D(const Eigen::Vector2f& point, float theta)
    {
        float c = std::cos(theta);
        float s = std::sin(theta);
        return Eigen::Vector2f(c*point.x() - s*point.y(), s*point.x() + c*point.y());
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

        try
        {
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
        } catch (const std::out_of_range & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    propagateGapPointsV2 out of range exception: " << e.what());
        } catch (const std::exception & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    propagateGapPointsV2 exception: " << e.what());
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
                gapManipulator_->convertRadialGap(planningGaps, i);

                gapManipulator_->inflateGapSides(planningGaps.at(i)); // bool success = 
                
                bool valid = planningGaps.at(i)->checkPoints();

                if (!valid)
                {
                    ROS_WARN_STREAM_NAMED("GapManipulator", "    invalid gap after manipulation " << i);
                    continue;
                }

                // if (success)
                // {
                ROS_INFO_STREAM_NAMED("GapManipulator", "    pushing back manipulated gap " << i);
                    
                    // gapGoalPlacer_->setGapGoal(planningGaps.at(i), 
                    //                             globalPlanManager_->getGlobalPathLocalWaypointRobotFrame(),
                    //                             globalGoalRobotFrame_);                    

                manipulatedGaps.push_back(planningGaps.at(i)); // shallow copy
                // }
            }
        } catch (const std::exception & e)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "    exception: " << e.what());
        }

        return manipulatedGaps;
    }

    void Planner::gapGoalPlacementV2(std::vector<GapTube *> & gapTubes) 
    {
        try
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

                    if (!gap->isAvailable()) // if curr gap is not available (should mean that previous gap existed)
                    {
                        ROS_INFO_STREAM_NAMED("Planner", "          current gap unavailable, placing goal inside");

                        if ( (j-1) < 0)
                        {
                            ROS_INFO_STREAM_NAMED("Planner", "       gap " << j << " is not available and no previous gap exists");
                            ROS_WARN_STREAM_NAMED("Planner", "       gap " << j << " is not available and no previous gap exists");
                            continue;
                        }

                        Gap * priorGap = gapTube->at(j-1);
                        // place goal inside gap

                        gapGoalPlacer_->setGapGoalFromPriorV2(gap, priorGap);

                    } else // curr gap is available
                    {
                        if (j < (gapTube->size() - 1) && !gapTube->at(j+1)->isAvailable()) // if next gap is not available
                        {
                            ROS_INFO_STREAM_NAMED("Planner", "          next gap unavailable, placing goal inside");

                            Gap * nextGap = gapTube->at(j+1);

                            // place goal inside gap
                            gapGoalPlacer_->setGapGoalFromNextV2(gap, nextGap);   

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
            }
        } catch (const std::out_of_range & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    gapGoalPlacementV2 out of range exception: " << e.what());
        }
        return;
    }

    void Planner::ungapGoalPlacement(const std::vector<Ungap *> & recedingUngaps)
    {
        try
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
        } catch (const std::out_of_range & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    ungapGoalPlacement out of range exception: " << e.what());
        } catch (const std::exception & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    ungapGoalPlacement exception: " << e.what());
        }

        return;
    }

    std::vector<GapTube *> Planner::gapSetFeasibilityCheckV2(const std::vector<GapTube *> & gapTubes, 
                                                                bool & isCurrentGapFeasible)                                             
    {
        ROS_INFO_STREAM_NAMED("Planner", "[gapSetFeasibilityCheckV2()]");

        std::vector<GapTube *> feasibleGapTubes;

        try
        {
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
        
                    if (j == 0 && !gap->isAvailable()) // if first gap is not available
                    {
                        ROS_INFO_STREAM_NAMED("Planner", "          first gap unavailable, skipping");
                        isTubeFeasible = false;
                        break;
                    }

                    // run pursuit guidance analysis on gap to determine feasibility
                    isGapFeasible = gapFeasibilityChecker_->pursuitGuidanceAnalysisV2(gap, startPt);

                    if (j == 0 &&
                        gap->getLeftGapPt()->getModel()->getID() == currentLeftGapPtModelID && 
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
        } catch (const std::out_of_range & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    gapSetFeasibilityCheckV2 out of range exception: " << e.what());
        } catch (const std::exception & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    gapSetFeasibilityCheckV2 exception: " << e.what());
        }

        return feasibleGapTubes;
    }


    void Planner::ungapSetFeasibilityCheck(const std::vector<Ungap *> & recedingUngaps)
    {
        try
        {
            for (size_t i = 0; i < recedingUngaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("Planner", "    feasibility check for ungap " << i);

                // run pursuit guidance analysis on ungap to calculate gamma (feasibility not really a thing here)
                ungapFeasibilityChecker_->pursuitGuidanceAnalysis(recedingUngaps.at(i));

            }
        } catch (const std::out_of_range & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    ungapSetFeasibilityCheck out of range exception: " << e.what());
        } catch (const std::exception & e)
        {
            ROS_ERROR_STREAM_NAMED("Planner", "    ungapSetFeasibilityCheck exception: " << e.what());
        }

        return;
    }       

    void Planner::generateGapTrajsV2(std::vector<GapTube *> & gapTubes, 
                                        std::vector<Trajectory> & gapTubeTrajs,
                                        std::vector<std::vector<float>> & gapTubeTrajPoseCosts,
                                        std::vector<float> & gapTubeTrajTerminalPoseCosts,
                                        const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "[generateGapTrajsV2()]");
        
        try
        {
            auto SZ = [](const Trajectory& t) {
    std::ostringstream ss;
    ss << "poses="  << t.getPathRbtFrame().poses.size()
       << " timing=" << t.getPathTiming().size();
    return ss.str();
};

auto VZ = [](const std::vector<float>& v) {
    std::ostringstream ss;
    ss << "size=" << v.size();
    return ss.str();
};

auto dumpSizes = [&](const std::string& tag,
                     int i, int j, int scanIdx,
                     const Trajectory& traj,
                     const Trajectory& runningTraj,
                     const std::vector<float>& poseCosts,
                     const std::vector<float>& runningPoseCosts,
                     const std::vector<sensor_msgs::LaserScan>& futureScans) {
    ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2",
        tag << " | tube i=" << i << " gap j=" << j << " scanIdx=" << scanIdx
            << " | traj(" << SZ(traj) << ")"
            << " | runningTraj(" << SZ(runningTraj) << ")"
            << " | poseCosts(" << VZ(poseCosts) << ")"
            << " | runningPoseCosts(" << VZ(runningPoseCosts) << ")"
            << " | futureScans.size=" << futureScans.size());
};


            gapTubeTrajs = std::vector<Trajectory>(gapTubes.size());
            gapTubeTrajPoseCosts = std::vector<std::vector<float>>(gapTubes.size());
            gapTubeTrajTerminalPoseCosts = std::vector<float>(gapTubes.size());

            geometry_msgs::TwistStamped currentRbtVel_;
            float current_linear_velocity = std::sqrt(
            std::pow(currentRbtVel_.twist.linear.x, 2) +
            std::pow(currentRbtVel_.twist.linear.y, 2)
            );


            for (int i = 0; i < gapTubes.size(); i++)
            {
                static int planCycle = 0;
                // ROS_ERROR_STREAM("==== Planning cycle " << planCycle++ << " ====");

                GapTube * gapTube = gapTubes.at(i);
                bool isTubeFeasible = true;
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generating traj for tube " << i);

                geometry_msgs::PoseStamped currPose = rbtPoseInSensorFrame_;
                // geometry_msgs::TwistStamped currVel = currentRbtVel_;

                int scanIdx = 0;

                Trajectory runningTraj;
                std::vector<float> runningPoseCosts;
                float runningTerminalPoseCost;

                bool disable_gap_tubes = true; 
                int iterate;
                if(disable_gap_tubes)
                { 
                    iterate = 1; 
                }
                else
                { 
                    iterate = gapTube->size(); 
                }

                for (int j = 0;j < iterate; j++) 
                {
                    Trajectory traj; 
                    Trajectory goToGoalTraj, pursuitGuidanceTraj;
                    
                    std::vector<float> poseCosts;
                    float terminalPoseCost;

                    std::vector<float> goToGoalPoseCosts, pursuitGuidancePoseCosts;
                    float goToGoalTerminalPoseCost, pursuitGuidanceTerminalPoseCost;
                    float goToGoalCost, pursuitGuidancePoseCost;

                    //variables for dwa
                    std::vector<float> dwa_goToGoalPoseCosts, dwa_PoseCosts; // IMPORTANT NOTE: dwa_PoseCosts is only the distance-related cost not other costs like path cost
                    std::vector<float> dwa_PathCosts; 
                    float dwa_goToGoalTerminalPoseCost, dwa_TerminalPoseCost;
                    float dwa_goToGoalCost, dwa_PoseCost;


                    Gap * gap = gapTube->at(j);
                    // dumpSizes("ITER START", i, j, scanIdx, traj, runningTraj, poseCosts, runningPoseCosts, futureScans);


                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "         gap: " << j);
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "             scanIdx: " << scanIdx);

                    // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                    
                    // Run go to goal behavior
                    bool runGoToGoal = gap->getGlobalGoalWithin();

                    if (runGoToGoal) 
                    {
                        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running goToGoal");
        
                        goToGoalTraj = gapTrajGenerator_->generateGoToGoalTrajectoryV2(gap, 
                                                                                        currPose, 
                                                                                        // currVel, 
                                                                                        globalGoalRobotFrame_);
                        goToGoalTraj = gapTrajGenerator_->processTrajectory(goToGoalTraj); 
                                                                            // currPose,
                                                                            // currVel,
                                                                            // false);
                        trajEvaluator_->evaluateTrajectory(goToGoalTraj, goToGoalPoseCosts, goToGoalTerminalPoseCost, futureScans, scanIdx);


                        goToGoalCost = goToGoalTerminalPoseCost + std::accumulate(goToGoalPoseCosts.begin(), goToGoalPoseCosts.end(), float(0)) / goToGoalPoseCosts.size();
                        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        goToGoalCost: " << goToGoalCost);
                    }
        
                    // Run pursuit guidance behavior
                    float totalTrajCost = 0;
                    // geometry_msgs::PoseArray pose_array;   // will be filled if DWA runs
                    // dwa_Trajectory dwa_traj;        
                    dwa_Trajectory cheapest_dwa;        
                    std::vector<dwa_Trajectory> dwa_trajs;

                    if (gap->isAvailable())
                    {
                        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running pursuit guidance (available)");
                        
                        if (cfg_.planning.dwa_method)
                        {
                            // get the latest min distance and velocity direction
                            float min_scan_dist = min_scan_dist_;
                            Eigen::Vector2f v_dir = v_dir_;
                            // Eigen::Vector2f goal_pos = gap->getGoal()->getTermGoalPos(); 
                            Eigen::Vector2f goal_pos = gap->getGoal()->getOrigGoalPos();
                            Eigen::Vector2f p0(0.0f, 0.0f);
                            Eigen::Vector2f p2 = projectOntoCircle(goal_pos, min_scan_dist);
                            // --- Rotation-based goal seed generation (multi-trajectory per gap) ---
                            float robot_radius = cfg_.rbt.r_inscr; 
                            float gap_insurance = 0.6f * robot_radius;  // offset scaling
                            float r = p2.norm();
                            float theta_off = gap_insurance / std::max(r, 1e-3f);

                            // Number of samples on each side
                            int theta_samples = 1; // e.g. left/right + center
                            // dwa_trajs.resize(theta_samples * 2 + 1);  // create real elements
                            std::vector<Eigen::Vector2f> curve; 

                            for (int k = -theta_samples; k <= theta_samples; ++k)
                            {
                                curve.clear(); 
                                float theta = k * theta_off;
                                Eigen::Vector2f p2_rot = rotatePoint2D(p2, theta); // IMPORTANT TODO:  i don't like this 
                                Eigen::Vector2f goal_rot = rotatePoint2D(goal_pos, theta);

                                // --- Generate Bézier for this rotated goal ---
                                curve = compositeBezier(
                                    p0, p2_rot, goal_rot, min_scan_dist, v_dir, 11);

                                // bezierVisualizer_->drawCurve(curve);
                                //  bezierVisualizer_->drawP2(p2);

                                dwa_Trajectory dwa_traj;


                            // std::vector<Eigen::Vector2f> curve = compositeBezier(p0, p2, goal_pos, min_scan_dist, v_dir, 11); //todo I might be able to add more points than just 11 since now i travel at a constant vel and just pick out poses along curve
                            //  bezierVisualizer_->drawP2(p2);
                            // bezierVisualizer_->drawCurve(curve);    

                            
float v0       = current_linear_velocity;
float v_max    = cfg_.rbt.vx_absmax;
float w_max    = cfg_.rbt.vang_absmax;
float a_max    =  cfg_.rbt.vang_absmax; //todo: update this value. I just set it to 1 for now

const int   num_points   = 11;           // total points along the trajectory
const int   num_segments = num_points - 1;
const float dt           = cfg_.traj.integrate_stept; // 0.5 sec
// const float total_time   = dt * num_segments;   

// --- Step 1: desired forward speed after one step of acceleration ---
float v_des = std::min(v0 + a_max * dt, v_max);


// --- Step 2: estimate curvature at the start of the Bézier (using sampled points) ---
Eigen::Vector2f curve_point0 = curve[0];
Eigen::Vector2f curve_point1 = curve[1];
Eigen::Vector2f curve_point2 = curve[2];

Eigen::Vector2f v1 = curve_point1 - curve_point0;
Eigen::Vector2f v2 = curve_point2 - curve_point1;

// z-component of the cross product between consecutive direction vectors
float cross = v1.x() * v2.y() - v1.y() * v2.x();
float curvature = cross / powf(v1.norm(), 3.0f);   // discrete curvature estimate


// --- Step 3: desired yaw rate from curvature ---
float w_des = v_des * curvature;


// --- Step 4: compute λ scaling factor (re-parameterization of curve timing) ---
float lambda = std::min(1.0f, v_des / v1.norm());


// --- Step 5: apply angular-velocity clipping (σ term) ---
float sigma = std::min(w_max / std::fabs(lambda * w_des), 1.0f);


// --- Step 6: final commanded velocities ---
float v_cmd = sigma * lambda * v_des;
float w_cmd = sigma * lambda * w_des;

dwa_traj.v_cmd = v_cmd;
dwa_traj.w_cmd = w_cmd;


// --- Assume constant speed along Bézier arc ---
float total_time = dt * (num_points - 1);
float total_length = 0.0f;

//compute arc length of the Bézier (approx. sum of straight-line segments)
for (int i = 1; i < curve.size(); ++i)
    total_length += (curve[i] - curve[i - 1]).norm();

//compute how far the robot travels per timestep at v_cmd
float step_distance = (v_cmd * total_time) / (num_points - 1);

//extract equally spaced points along the curve by accumulated distance

float dist_accum = 0.0f;
float next_target = 0.0f;
int current_seg = 0;

dwa_traj.positions.reserve(num_points);
dwa_traj.yaws.reserve(num_points);
dwa_traj.times.reserve(num_points);

// always include starting point
dwa_traj.positions.push_back(curve.front());
dwa_traj.yaws.push_back(atan2((curve[1] - curve[0]).y(), (curve[1] - curve[0]).x()));
dwa_traj.times.push_back(0.0f);

// iterate through segments and pick positions spaced by step_distance
for (int i = 1; i < num_points; ++i)
{
    next_target += step_distance;

    // walk along the Bézier until accumulated distance exceeds next_target
    while (current_seg + 1 < curve.size() &&
           (dist_accum + (curve[current_seg + 1] - curve[current_seg]).norm()) < next_target)
    {
        dist_accum += (curve[current_seg + 1] - curve[current_seg]).norm();
        current_seg++;
    }

    // if (current_seg + 1 >= curve.size())
    //     break;

    float seg_len = (curve[current_seg + 1] - curve[current_seg]).norm();
    float remaining = next_target - dist_accum;
    float t_seg = remaining / std::max(seg_len, 1e-6f);

    // linear interpolation between curve points
    Eigen::Vector2f pt = curve[current_seg] + t_seg * (curve[current_seg + 1] - curve[current_seg]);
    Eigen::Vector2f dir = (curve[current_seg + 1] - curve[current_seg]).normalized();

    dwa_traj.positions.push_back(pt); //I suspect this may be causing the issue - 1028
    // ROS_ERROR_STREAM_NAMED("planner debug", "[before toPoseArray] dwa_traj.positions.size(): )" <<  dwa_traj.positions.size());
    dwa_traj.yaws.push_back(atan2(dir.y(), dir.x()));
    dwa_traj.times.push_back(i * dt);
} 
//publish as PoseArray


// --- Convert positioin evaluator path.size()ns to PoseArray ---
dwa_traj.pose_array = dwa_traj.toPoseArray(cfg_.sensor_frame_id);//I suspect this may be causing the issue - 1028
    
// --- Evaluate each pose using your evaluator --- // going to use evaluateTraj instead 
// int counter = 0; 
// for (const auto& pose : pose_array.poses)
// {
//     float cost = trajEvaluator_->dwa_evaluatePose(pose, futureScans.at(scanIdx+counter));
//     ROS_ERROR_STREAM_NAMED("dwa_evaluatePose output", "pose: " << counter << "dwa_evaluatePose (not evalTraj) output: " << cost);
//     counter++; 
// }

//  std::vector<float> dwa_goToGoalPoseCosts, dwa_PoseCosts;
//                     float dwa_goToGoalTerminalPoseCost, dwa_TerminalPoseCost;
//                     float dwa_goToGoalCost, dwa_PoseCost;

std::vector<geometry_msgs::PoseStamped> visiblePlan =
    globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
// float totalTrajCost = 0;
float speedCost = 0; 
trajEvaluator_->dwa_evaluateTrajectory(totalTrajCost, dwa_traj.pose_array, dwa_PoseCosts, dwa_PathCosts, dwa_TerminalPoseCost, futureScans, scanIdx, visiblePlan);
speedCost = trajEvaluator_->calcSpeedCost(v_cmd, v_max); 
totalTrajCost += speedCost * cfg_.traj.w_speed;
// ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "totalTrajCost: " << totalTrajCost);
float pose_cost_sum = 0.0f;
float path_cost_sum = 0.0f;

// if (!dwa_PoseCosts.empty())
// {
//     pose_cost_sum = std::accumulate(dwa_PoseCosts.begin(), dwa_PoseCosts.end(), 0.0f) / dwa_PoseCosts.size();
//     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "PoseCosts (size = " << dwa_PoseCosts.size() << "):");
//     for (size_t i = 0; i < dwa_PoseCosts.size(); ++i)
//     {
//         ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "    [" << i << "] = " << dwa_PoseCosts[i]);
//     }
// }

// if (!dwa_PathCosts.empty())
// {
//     path_cost_sum = std::accumulate(dwa_PathCosts.begin(), dwa_PathCosts.end(), 0.0f) / dwa_PathCosts.size();
//     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "PathCosts (size = " << dwa_PathCosts.size() << "):");
//     for (size_t i = 0; i < dwa_PathCosts.size(); ++i)
//     {
//         ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "    [" << i << "] = " << dwa_PathCosts[i]);
//     }
// }

// ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "PoseCosts avg: " << pose_cost_sum);
// ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "PathCosts avg: " << path_cost_sum);

// traj_pub_.publish(dwa_traj.toPoseArray(cfg_.sensor_frame_id));
traj_pub_.publish(dwa_traj.pose_array);
dwa_traj.PoseCosts = dwa_PoseCosts; 
dwa_traj.PathCosts = dwa_PathCosts; 
dwa_traj.TerminalPoseCost = dwa_TerminalPoseCost; 
dwa_traj.totalTrajCost = totalTrajCost; 
dwa_trajs.push_back(dwa_traj);  // append new trajectory

bool visualize_curves_and_costs = false; 
if (visualize_curves_and_costs)
{
// Evaluate trajectory
float dwa_visual_duration = .5; 
// trajEvaluator_->dwa_evaluateTrajectory(
//     pose_array, dwa_PoseCosts, dwa_TerminalPoseCost, futureScans, scanIdx);

static ros::Publisher traj_cost_viz_pub =
    nh_.advertise<visualization_msgs::MarkerArray>("dwa_traj_cost_viz", 1, true);

//Clear previous markers if you want a clean refresh each cycle
visualization_msgs::MarkerArray clear;
clear.markers.clear();
traj_cost_viz_pub.publish(clear);


static int global_marker_id = 0;


// --- for visualization combine pose, path, and terminal costs into total per-step costs ---
std::vector<float> dwa_TotalCostsVector;
dwa_TotalCostsVector.reserve(dwa_PoseCosts.size());

for (size_t i = 0; i < dwa_PoseCosts.size(); ++i)
{
    float total_cost = dwa_PoseCosts[i] + dwa_PathCosts[i];

    // add terminal cost only at the last time step
    if (i == dwa_PoseCosts.size() - 1)
        total_cost += dwa_TerminalPoseCost;

    dwa_TotalCostsVector.push_back(total_cost);
}

visualization_msgs::MarkerArray traj_markers;
std::vector<float> costPrintout; 
costPrintout = dwa_TotalCostsVector; // adjust this line for the rviz cost print out. options: dwa_PoseCosts, dwa_PathCosts, dwa_TotalCostsVector

// ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "dwa_PoseCosts" << dwa_PoseCosts);

// --- line strip for this trajectory ---
visualization_msgs::Marker line;
line.header.frame_id = cfg_.sensor_frame_id;
line.header.stamp = ros::Time::now();
line.ns = "dwa_traj_lines";
line.id = global_marker_id++;
line.type = visualization_msgs::Marker::LINE_STRIP;
line.action = visualization_msgs::Marker::ADD;
line.scale.x = 0.03;
line.color.r = 0.0;
line.color.g = 1.0;
line.color.b = 0.0;
line.color.a = 1.0;
line.lifetime = ros::Duration(dwa_visual_duration);  // keep indefinitely

for (const auto& pose : dwa_traj.pose_array.poses)
{
    geometry_msgs::Point p;
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = 0.02;
    line.points.push_back(p);
}
traj_markers.markers.push_back(line);

// --- per-pose cost text ---
for (size_t i = 0; i < dwa_traj.pose_array.poses.size(); ++i)
{
    visualization_msgs::Marker text;
    text.header = line.header;
    text.ns = "dwa_pose_costs";
    text.id = global_marker_id++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose = dwa_traj.pose_array.poses[i];
    text.pose.position.z += 0.15;
    text.scale.z = 0.15;
    text.color.r = 0.0;
    text.color.g = 0.0;
    text.color.b = 0.0;
    text.color.a = 1.0;
    text.lifetime = ros::Duration(dwa_visual_duration);

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << costPrintout[i];
    text.text = ss.str();

    traj_markers.markers.push_back(text);
}

// Publish all markers for this trajectory
traj_cost_viz_pub.publish(traj_markers);
}





// ------------------------------------------------------------
// Visualization-only rollout for RViz
// ------------------------------------------------------------
bool visualize_dwa_rollout = false; 
if (visualize_dwa_rollout)
{
    // initial state (robot frame)
    float vis_x = 0.0f;
    float vis_y = 0.0f;
    float vis_yaw = 0.0f;
    float vis_t = 0.0f;

    // storage
    std::vector<Eigen::Vector2f> vis_positions;
    std::vector<float> vis_yaws;
    std::vector<float> vis_times;

    vis_positions.reserve(num_points);
    vis_yaws.reserve(num_points);
    vis_times.reserve(num_points);

    // store initial pose
    vis_positions.push_back(Eigen::Vector2f(vis_x, vis_y));
    vis_yaws.push_back(vis_yaw);
    vis_times.push_back(vis_t);

    // rollout integration (unicycle model)
    for (int vi = 1; vi < num_points; ++vi)
    {
        vis_yaw += w_cmd * dt;
        vis_x   += v_cmd * std::cos(vis_yaw) * dt;
        vis_y   += v_cmd * std::sin(vis_yaw) * dt;
        vis_t   += dt;

        vis_positions.push_back(Eigen::Vector2f(vis_x, vis_y));
        vis_yaws.push_back(vis_yaw);
        vis_times.push_back(vis_t);
    }

    // ------------------------------------------------------------
    // Convert to ROS PoseArray
    // ------------------------------------------------------------
    geometry_msgs::PoseArray vis_traj_path;
    vis_traj_path.header.frame_id = cfg_.sensor_frame_id;
    vis_traj_path.header.stamp = ros::Time::now();

    for (int vi = 0; vi < num_points; ++vi)
    {
        geometry_msgs::Pose vis_pose;
        vis_pose.position.x = vis_positions[vi].x();
        vis_pose.position.y = vis_positions[vi].y();
        vis_pose.position.z = 0.0;
        vis_pose.orientation = tf::createQuaternionMsgFromYaw(vis_yaws[vi]);
        vis_traj_path.poses.push_back(vis_pose);
    }
}

}

}
// pick the cheapest dwa traj
if (!dwa_trajs.empty())
{
    cheapest_dwa = dwa_trajs[0];
    for (size_t i = 1; i < dwa_trajs.size(); ++i)
    {
        if (dwa_trajs[i].totalTrajCost < cheapest_dwa.totalTrajCost)
            cheapest_dwa = dwa_trajs[i];
    }
    // ROS_ERROR_STREAM("[CHEAPEST_DWA] cost=" << cheapest_dwa.totalTrajCost);
}
else
{
    ROS_WARN_STREAM("[CHEAPEST_DWA] no valid DWA trajectories found");
}


bool visualize_all_dwa_trajs = true;
if (visualize_all_dwa_trajs && !dwa_trajs.empty())
{
    static ros::Publisher traj_cost_viz_pub =
        nh_.advertise<visualization_msgs::MarkerArray>("dwa_traj_cost_viz", 1, true);

    // clear previous markers
    visualization_msgs::MarkerArray clear;
    traj_cost_viz_pub.publish(clear);

    visualization_msgs::MarkerArray all_markers;
    static int global_marker_id = 0;
    float text_scale = 0.12f;
    float line_width = 0.03f;
    float lifetime = 0.25f;

    for (size_t t = 0; t < dwa_trajs.size(); ++t)
    {
        const auto &traj = dwa_trajs[t];
        // geometry_msgs::PoseArray poses = traj.toPoseArray(cfg_.sensor_frame_id);
        const geometry_msgs::PoseArray& poses = traj.pose_array;

        // --- compute total cost per pose (pose + path + speed + terminal) ---
        std::vector<float> total_costs;
        total_costs.reserve(poses.poses.size());
        for (size_t i = 0; i < poses.poses.size(); ++i)
        {
            float pose_cost = (i < traj.PoseCosts.size()) ? traj.PoseCosts[i] : 0.0f;
            float path_cost = (i < traj.PathCosts.size()) ? traj.PathCosts[i] : 0.0f;
            float speed_cost = 0.0f;
            float term_cost = 0.0f;

            // normalize per length (same as evaluator)
            float len_norm = std::max(1.0f, float(poses.poses.size()));
            pose_cost /= len_norm;
            path_cost /= len_norm;

            if (i == poses.poses.size() - 1)
            {
                speed_cost = trajEvaluator_->calcSpeedCost(traj.v_cmd, cfg_.rbt.vx_absmax);
                term_cost  = traj.TerminalPoseCost;
            }

            float total = cfg_.traj.w_obs  * pose_cost +
                          cfg_.traj.w_path * path_cost +
                          cfg_.traj.w_speed * speed_cost +
                          cfg_.traj.w_goal  * term_cost;
            total_costs.push_back(total);
        }

        // --- draw trajectory line ---
        visualization_msgs::Marker line;
        line.header.frame_id = cfg_.sensor_frame_id;
        line.header.stamp = ros::Time::now();
        line.ns = "dwa_traj_lines";
        line.id = global_marker_id++;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.scale.x = line_width;
        line.color.r = 0.0;
        line.color.g = 0.8;
        line.color.b = 1.0;
        line.color.a = 1.0;
        line.lifetime = ros::Duration(lifetime);

        for (size_t i = 0; i < poses.poses.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = poses.poses[i].position.x;
            p.y = poses.poses[i].position.y;
            p.z = 0.05;
            line.points.push_back(p);
        }
        all_markers.markers.push_back(line);

        // --- per-pose cost text ---
        for (size_t i = 0; i < poses.poses.size(); ++i)
        {
            visualization_msgs::Marker text;
            text.header = line.header;
            text.ns = "dwa_pose_costs";
            text.id = global_marker_id++;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::Marker::ADD;
            text.pose = poses.poses[i];
            text.pose.position.z += 0.15;
            text.scale.z = text_scale;
            text.color.r = 0.0;
            text.color.g = 0.0;
            text.color.b = 0.0;
            text.color.a = 1.0;
            text.lifetime = ros::Duration(lifetime);

            std::ostringstream ss;
            ss << std::fixed << std::setprecision(2) << total_costs[i];
            text.text = ss.str();

            all_markers.markers.push_back(text);
        }

    traj_cost_viz_pub.publish(all_markers);
    
}





// traj_pub_.publish(vis_traj_path); // commented out because i'm being lazy and want to use the same publisher to publish what's above

//delete this: 
                            // pursuitGuidanceTraj = gapTrajGenerator_->generateTrajectoryV2(gap, 
                            //                                                                 currPose, 
                            //                                                                 // currVel, 
                            //                                                                 globalGoalRobotFrame_);
                            // pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj); 

                            
                            // pursuitGuidanceTraj = bezierTraj;
                            // pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj); 


                            //delete this: 
                            //  pursuitGuidanceTraj = gapTrajGenerator_->generateTrajectoryV2(gap, 
                            //                                                                 currPose, 
                            //                                                                 // currVel, 
                            //                                                                 globalGoalRobotFrame_);
                            // pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj); 
                            //                                                             // currPose,
                            //                                                             // currVel,
                            //                                                         // false);
                        }
                        else
                        {
                            pursuitGuidanceTraj = gapTrajGenerator_->generateTrajectoryV2(gap, 
                                                                                            currPose, 
                                                                                            // currVel, 
                                                                                            globalGoalRobotFrame_);
                            pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj); 
                                                                                        // currPose,
                                                                                        // currVel,
                                                                                    // false);
                        }
                        
                        // if (j == (gapTube->size() - 1)) //commented out while debugging dwa mismatch 
                        // {
                        //     // prune trajectory
                        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        pruning pursuit guidance (available) traj");
                        //     pursuitGuidanceTraj = gapTrajGenerator_->pruneTrajectory(pursuitGuidanceTraj);
                        // }futureScans, scanIdx);futureScans, scanIdx);
                        
    // ROS_ERROR_STREAM("before evaluator: tube=" << i 
    //              << " gap=" << j 
    //              << " traj path.size()=" 
    //              << pursuitGuidanceTraj.getPathRbtFrame().poses.size());


                        // ROS_ERROR_STREAM("before evaluator: tube=" << i 
                        // << " gap=" << j);
                        // IMPORTANT todo: put an if statement here for when dwa is jused since I already evaluateTraj() above

                        if (!cfg_.planning.dwa_method)
                        {
                            trajEvaluator_->evaluateTrajectory(pursuitGuidanceTraj, pursuitGuidancePoseCosts, pursuitGuidanceTerminalPoseCost, futureScans, scanIdx);
                            pursuitGuidancePoseCost = pursuitGuidanceTerminalPoseCost + std::accumulate(pursuitGuidancePoseCosts.begin(), pursuitGuidancePoseCosts.end(), float(0)) / pursuitGuidancePoseCosts.size();
                            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        pursuitGuidancePoseCost: " << pursuitGuidancePoseCost);
                        }
                        else
                        { 
                            //package dwa traj into the Trajectory class since that's what the later code expects 
                            // --- Package DWA-generated path (pose_array) into a Trajectory object ---
                            Trajectory dwaTrajectory;

                            // The Trajectory class typically stores PoseArray and timing
                            dwaTrajectory.setPathRbtFrame(cheapest_dwa.pose_array);         // robot-frame path

                            if (!cheapest_dwa.times.empty())
                            {
                                dwaTrajectory.setPathTiming(cheapest_dwa.times); 
                                // ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "times size: " << dwaTrajectory.getPathTiming().size());

                            }

                            else
                            {
                                ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "cheapest_dwa.times is empty, cannot use the dwaTrajectory.setPathTiming() function");
                            } 
                            // ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "poses size: " << dwaTrajectory.getPathRbtFrame().poses.size());


                            dwaTrajectory.setVcmd(cheapest_dwa.v_cmd);
                            dwaTrajectory.setWcmd(cheapest_dwa.w_cmd);
                            // ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "inside generateGapTraj v_cmd = " << dwaTrajectory.getVcmd() << " w_cmd = " << dwaTrajectory.getWcmd());

                            ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2",
                            "PACKAGED DWA traj: rbt=" << dwaTrajectory.getPathRbtFrame().poses.size()
                            << " timing=" << dwaTrajectory.getPathTiming().size()
                            << " v=" << dwaTrajectory.getVcmd()
                            << " w=" << dwaTrajectory.getWcmd());


                            // Important! I'm being lazy and brute forceing and redirect pursuitGuidanceTraj to use this new trajectory
                            pursuitGuidanceTraj = dwaTrajectory;
       
                            pursuitGuidancePoseCost = cheapest_dwa.totalTrajCost; // this total traj does the same thing, just look in TrajectoryEvaluator
                            pursuitGuidancePoseCosts = cheapest_dwa.PoseCosts;
                            pursuitGuidanceTerminalPoseCost = cheapest_dwa.TerminalPoseCost;
                            // ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "TerminalPoseCost: " << cheapest_dwa.TerminalPoseCost);

                

                        for (size_t i = 0; i < pursuitGuidancePoseCosts.size(); ++i)
                        {
                            ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2",
                                " inside generateGap pursuitGuidancePoseCosts[" << i << "] = " << pursuitGuidancePoseCosts[i]);
                        }


                        }

                    } else
                    {
                        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running pursuit guidance (not available)");
                        pursuitGuidanceTraj = gapTrajGenerator_->generateIdlingTrajectoryV2(gap, 
                                                                                            currPose,
                                                                                            runningTraj);
                                                                                            // rbtPoseInOdomFrame_);
                        // pursuitGuidanceTraj = gapTrajGenerator_->processIdlingTrajectory(pursuitGuidanceTraj,
                        //                                                                     runningTraj);
                        pursuitGuidanceTraj = gapTrajGenerator_->processTrajectory(pursuitGuidanceTraj); 
                                                                                    // currPose,
                                                                                    // currVel,
                                                                                    // false);
                        // if (j == (gapTube->size() - 1))
                        // {
                        //     // prune trajectory
                        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        pruning pursuit guidance (unavailable) traj");
                        //     pursuitGuidanceTraj = gapTrajGenerator_->pruneTrajectory(pursuitGuidanceTraj);
                        // }

                        trajEvaluator_->evaluateTrajectory(pursuitGuidanceTraj, pursuitGuidancePoseCosts, pursuitGuidanceTerminalPoseCost, futureScans, scanIdx);
                        pursuitGuidancePoseCost = pursuitGuidanceTerminalPoseCost + std::accumulate(pursuitGuidancePoseCosts.begin(), pursuitGuidancePoseCosts.end(), float(0)) / pursuitGuidancePoseCosts.size();
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

                    // dumpSizes("POST-SELECT", i, j, scanIdx, traj, runningTraj, poseCosts, runningPoseCosts, futureScans);
                    // ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2",
                    //     "POST-SELECT: selected poses=" << traj.getPathRbtFrame().poses.size()
                    // << " timing=" << traj.getPathTiming().size()
                    // << " poseCosts=" << poseCosts.size()
                    // << " terminal=" << terminalPoseCost);

        
                    // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                    traj.setPathOdomFrame(gapTrajGenerator_->transformPath(traj.getPathRbtFrame(), rbt2odom_));

//                     ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2",
//     "PRE-BACK CHECK | selected traj sizes: poses="
//     << traj.getPathRbtFrame().poses.size()
//     << " timing=" << traj.getPathTiming().size()
//     << " poseCosts=" << poseCosts.size());

if (traj.getPathRbtFrame().poses.empty()) {
    ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2",
        "ERROR: selected traj has empty poses (cannot use back()).");
}

if (traj.getPathTiming().empty()) {
    ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2",
        "ERROR: selected traj has empty timing (cannot use back/front).");
}

                    // update current pose and time
                    currPose.pose = traj.getPathRbtFrame().poses.back();

                    float trajDuration = traj.getPathTiming().back() - traj.getPathTiming().front();
                    currPose.header.stamp = traj.getPathRbtFrame().header.stamp += ros::Duration(trajDuration);

                    // append traj
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        appending trajectory piece...");
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        traj size: " << traj.getPathRbtFrame().poses.size());
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running traj size: " << runningTraj.getPathRbtFrame().poses.size());
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running pose costs size: " << poseCosts.size());
                    ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running pose costs size: " << runningPoseCosts.size());
                    if (j == 0)
                    {
                        runningTraj = traj;
                        runningPoseCosts = poseCosts;
                        runningTerminalPoseCost = terminalPoseCost;
                    } else
                    {
                        runningTraj.appendTraj(traj);
                        // append costs
                        runningPoseCosts.insert(runningPoseCosts.end(), poseCosts.begin() + 1, poseCosts.end());
                        runningTerminalPoseCost = terminalPoseCost; //  += terminalPoseCost;
                    }

                    scanIdx = runningTraj.getPathRbtFrame().poses.size() - 1;
                }

                gapTubeTrajs.at(i) = runningTraj;
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
        } catch (const std::out_of_range & e)
        {
            ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generateGapTrajsV2 out of range exception: " << e.what());
        } catch (const std::exception & e)
        {
            ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generateGapTrajsV2 exception: " << e.what());
        }

        return;
    }

    void Planner::generateUngapTrajs(std::vector<Ungap *> & ungaps, 
                                        std::vector<Trajectory> & ungapTrajs,
                                        std::vector<std::vector<float>> & ungapTrajPoseCosts,
                                        std::vector<float> & ungapTrajTerminalPoseCosts,
                                        const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        boost::mutex::scoped_lock gapset(gapMutex_);

        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "[generateUngapTrajs()]");
     
        try
        {

            ungapTrajPoseCosts = std::vector<std::vector<float>>(ungaps.size());
            ungapTrajTerminalPoseCosts = std::vector<float>(ungaps.size());

            for (size_t i = 0; i < ungaps.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generating traj for un-gap: " << i);
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                
                // Run go to goal behavior

                Trajectory traj, pursuitGuidanceTraj;
                std::vector<float> pursuitGuidancePoseCosts;
                float pursuitGuidanceTerminalPoseCost;
                float pursuitGuidancePoseCost;

                // Run pursuit guidance behavior
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        running pursuit guidance");
                pursuitGuidanceTraj = ungapTrajGenerator_->generateTrajectory(ungaps.at(i), 
                                                                                rbtPoseInSensorFrame_);

                pursuitGuidanceTraj = ungapTrajGenerator_->processTrajectory(pursuitGuidanceTraj, 
                                                                                rbtPoseInSensorFrame_,
                                                                                true);
                trajEvaluator_->evaluateTrajectory(pursuitGuidanceTraj, 
                                                    pursuitGuidancePoseCosts, 
                                                    pursuitGuidanceTerminalPoseCost, 
                                                    futureScans,
                                                    0);
                pursuitGuidancePoseCost = pursuitGuidanceTerminalPoseCost + std::accumulate(pursuitGuidancePoseCosts.begin(), pursuitGuidancePoseCosts.end(), float(0)) / pursuitGuidancePoseCosts.size();
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        pursuitGuidancePoseCost: " << pursuitGuidancePoseCost);

                traj = pursuitGuidanceTraj;
                ungapTrajPoseCosts.at(i) = pursuitGuidancePoseCosts;
                ungapTrajTerminalPoseCosts.at(i) = pursuitGuidanceTerminalPoseCost;

                // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                traj.setPathOdomFrame(ungapTrajGenerator_->transformPath(traj.getPathRbtFrame(), rbt2odom_));
                ungapTrajs.push_back(traj);
            }

            // trajVisualizer_->drawGapTrajectoryPoseScores(ungapTrajs, ungapTrajPoseCosts);
            trajVisualizer_->drawUngapTrajectories(ungapTrajs);        
        } catch (const std::out_of_range & e)
        {
            ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generateUngapTrajs out of range exception: " << e.what());
        } catch (const std::exception & e)
        {
            ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generateUngapTrajs exception: " << e.what());
        }

        return;
    }

    // void Planner::generateIdlingTraj(std::vector<Trajectory> & idlingTrajs,
    //                                     std::vector<std::vector<float>> & idlingTrajPoseCosts,
    //                                     std::vector<float> & idlingTrajTerminalPoseCosts,
    //                                     const std::vector<sensor_msgs::LaserScan> & futureScans) 
    // {
    //     try
    //     {
    //         boost::mutex::scoped_lock gapset(gapMutex_);

    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "[generateIdlingTraj()]");

    //         Gap * nullGap = nullptr;
    //         Trajectory idlingTrajectory;
    //         idlingTrajectory = gapTrajGenerator_->generateIdlingTrajectoryV2(nullGap, 
    //                                                                             rbtPoseInSensorFrame_);

    //         std::vector<float> idlingPoseCosts;
    //         float idlingTerminalPoseCost, idlingCost;

    //         trajEvaluator_->evaluateTrajectory(idlingTrajectory, idlingPoseCosts, idlingTerminalPoseCost, futureScans, 0);

    //         idlingCost = idlingTerminalPoseCost + std::accumulate(idlingPoseCosts.begin(), idlingPoseCosts.end(), float(0)) / idlingPoseCosts.size();

    //         idlingTrajPoseCosts.push_back(idlingPoseCosts);
    //         idlingTrajTerminalPoseCosts.push_back(idlingTerminalPoseCost);

    //         idlingTrajectory.setPathOdomFrame(gapTrajGenerator_->transformPath(idlingTrajectory.getPathRbtFrame(), rbt2odom_));
    //         idlingTrajs.push_back(idlingTrajectory);    

    //         // // push back idling trajectory as another option

    //         // idlingTrajectory = gapTrajGenerator_->generateIdlingTrajectory(rbtPoseInOdomFrame_);
            
    //         // 
    //         // idlingCost = idlingTerminalPoseCost + std::accumulate(idlingPoseCosts.begin(), idlingPoseCosts.end(), float(0));
    //         // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        idlingCost: " << idlingCost);
    //     } catch (const std::out_of_range & e)
    //     {
    //         ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generateIdlingTraj out of range exception: " << e.what());
    //     } catch (const std::exception & e)
    //     {
    //         ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "    generateIdlingTraj exception: " << e.what());
    //     }
    

    //     return;
    // }

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

        try
        {

            ROS_INFO_STREAM_NAMED("Planner", "[pickTraj()]");
            
            // int lowestCostTrajIdx = -1;        
            
            int numUngapTrajs = ungapTrajs.size();
            int numIdlingTrajs = idlingTrajs.size();
            int numGapTrajs = gapTrajs.size();
            int numTrajs = numGapTrajs + numUngapTrajs + numIdlingTrajs;
            if (numTrajs == 0)
            {
                ROS_WARN_STREAM_NAMED("Planner", "No trajectories synthesized");
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
            std::vector<float> pathCosts(numGapTrajs + numUngapTrajs + numIdlingTrajs);

            // evaluate gap trajectories
            int runningTrajIdx = 0;
            Trajectory currentTraj;
            std::vector<float> currentTrajPoseCosts;
            float currentTrajTerminalPoseCost;
            for (size_t i = 0; i < gapTrajs.size(); i++) 
            {
                currentTraj = gapTrajs.at(i);
                ROS_ERROR_STREAM("inside picktraj gapTrajs.at(i).getPathRbtFrame().poses.size(): " << gapTrajs.at(i).getPathRbtFrame().poses.size());

                currentTrajPoseCosts = gapTrajPoseCosts.at(i);
                currentTrajTerminalPoseCost = gapTrajTerminalPoseCosts.at(i);
                // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());

                // if (gapTrajs.at(i).getPathRbtFrame().poses.size() < 2)
                // {
                //     ROS_WARN_STREAM_NAMED("Planner", "gap traj " << i << " has < 2 poses");
                // }

                pathCosts.at(runningTrajIdx) = gapTrajTerminalPoseCosts.at(i) + std::accumulate(currentTrajPoseCosts.begin(), currentTrajPoseCosts.end(), float(0)) / currentTrajPoseCosts.size();
                
                pathCosts.at(runningTrajIdx) = currentTraj.getPathRbtFrame().poses.size() < 2 ? std::numeric_limits<float>::infinity() : 
                                                                                                    pathCosts.at(runningTrajIdx);
                
                ROS_INFO_STREAM_NAMED("Planner", "    for gap tube " << i << " (length: " << gapTrajs.at(i).getPathRbtFrame().poses.size() << "), returning cost of " << pathCosts.at(runningTrajIdx));
            
                runningTrajIdx++;
            }

            // evaluate ungap trajectories
            for (size_t i = 0; i < ungapTrajs.size(); i++) 
            {
                // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());

                if (ungapTrajs.at(i).getPathRbtFrame().poses.size() < 2)
                {
                    ROS_WARN_STREAM_NAMED("Planner", "ungap traj " << i << " has < 2 poses");
                }

                pathCosts.at(runningTrajIdx) = ungapTrajTerminalPoseCosts.at(i) + std::accumulate(ungapTrajPoseCosts.at(i).begin(), ungapTrajPoseCosts.at(i).end(), float(0)) / ungapTrajPoseCosts.at(i).size();
                
                pathCosts.at(runningTrajIdx) = ungapTrajs.at(i).getPathRbtFrame().poses.size() < 2 ? std::numeric_limits<float>::infinity() : 
                                                                                                        pathCosts.at(runningTrajIdx);
                
                ROS_INFO_STREAM_NAMED("Planner", "    for ungap " << i << " (length: " << ungapTrajs.at(i).getPathRbtFrame().poses.size() << "), returning cost of " << pathCosts.at(runningTrajIdx));
            
                runningTrajIdx++;
            }

            // evaluate idling trajectory
            for (size_t i = 0; i < idlingTrajs.size(); i++) 
            {
                // ROS_WARN_STREAM("paths(" << i << "): size " << paths.at(i).poses.size());

                if (idlingTrajs.at(i).getPathRbtFrame().poses.size() < 2)
                {
                    ROS_WARN_STREAM_NAMED("Planner", "idling traj " << i << " has < 2 poses");
                }

                pathCosts.at(runningTrajIdx) = idlingTrajTerminalPoseCosts.at(i) + std::accumulate(idlingTrajPoseCosts.at(i).begin(), idlingTrajPoseCosts.at(i).end(), float(0)) / idlingTrajPoseCosts.at(i).size();
                
                pathCosts.at(runningTrajIdx) = idlingTrajs.at(i).getPathRbtFrame().poses.size() < 2 ? std::numeric_limits<float>::infinity() : 
                                                                                                        pathCosts.at(runningTrajIdx);
                
                ROS_INFO_STREAM_NAMED("Planner", "    for idling " << i << " (length: " << idlingTrajs.at(i).getPathRbtFrame().poses.size() << "), returning cost of " << pathCosts.at(runningTrajIdx));
            
                runningTrajIdx++;
            }

            // selecting best trajectory
            auto lowestCostTrajIterIter = std::min_element(pathCosts.begin(), pathCosts.end());
            int candidateLowestCostTrajIdx = std::distance(pathCosts.begin(), lowestCostTrajIterIter);

            // if (pathCosts.at(candidateLowestCostTrajIdx) == std::numeric_limits<float>::infinity()) 
            // {    
            //     ROS_INFO_STREAM_NAMED("Planner", "    all infinity");
            //     ROS_INFO_STREAM_NAMED("Planner", "No executable trajectory, values: ");
            //     for (float pathCost : pathCosts) 
            //     {
            //         ROS_INFO_STREAM_NAMED("Planner", "Cost: " << pathCost);
            //     }
            //     ROS_INFO_STREAM_NAMED("Planner", "------------------");

            //     trajFlag = NONE;
            //     // do not update candidateLowestCostTrajIdx
            //     return;
            // }

            if (candidateLowestCostTrajIdx >= 0 && candidateLowestCostTrajIdx < numGapTrajs)
            {
                lowestCostTrajIdx = candidateLowestCostTrajIdx;
                trajFlag = GAP;
                ROS_INFO_STREAM_NAMED("Planner", "    picking gap traj: " << lowestCostTrajIdx);
                // ROS_ERROR_STREAM_NAMED("Planner", "--------- picked GAP traj");
                /////////////// just for debugging printouts: 
                // --- Extract cost data for this gap ---
                const std::vector<float>& currentTrajPoseCosts = gapTrajPoseCosts.at(lowestCostTrajIdx);
                float currentTrajTerminalPoseCost = gapTrajTerminalPoseCosts.at(lowestCostTrajIdx);

                // --- Print pose-wise costs ---
                // ROS_ERROR_STREAM_NAMED("Planner", "[GAP_COST] Pose-wise breakdown (traj " << lowestCostTrajIdx << "):");
                for (size_t k = 0; k < currentTrajPoseCosts.size(); ++k)
                {
                    float cost = currentTrajPoseCosts[k];
                    if (k == currentTrajPoseCosts.size() - 1)
                        cost += currentTrajTerminalPoseCost;

                    ROS_ERROR_STREAM_NAMED("Planner", "   pose[" << k << "] cost=" << cost);
                }

            } else if (candidateLowestCostTrajIdx >= numGapTrajs && candidateLowestCostTrajIdx < (numGapTrajs + numUngapTrajs))
            {
                lowestCostTrajIdx = candidateLowestCostTrajIdx - numGapTrajs;
                trajFlag = UNGAP;
                ROS_ERROR_STREAM_NAMED("Planner", "          picked UPGAP traj");
                ROS_INFO_STREAM_NAMED("Planner", "    picking ungap traj: " << lowestCostTrajIdx);
            } else if (candidateLowestCostTrajIdx >= (numGapTrajs + numUngapTrajs) && candidateLowestCostTrajIdx < (numGapTrajs + numUngapTrajs + numIdlingTrajs))
            {
                lowestCostTrajIdx = candidateLowestCostTrajIdx - (numGapTrajs + numUngapTrajs);
                trajFlag = IDLING;
                ROS_ERROR_STREAM_NAMED("Planner", "          picked IDLING traj");
                ROS_INFO_STREAM_NAMED("Planner", "    picking idling traj: " << lowestCostTrajIdx);
            } else
            {
                ROS_WARN_STREAM_NAMED("Planner", "    trajFlag not set");
            }
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "    pickTraj out of range exception: " << e.what());
        } catch (const std::exception & e)
        {
            ROS_WARN_STREAM_NAMED("Planner", "    pickTraj exception: " << e.what());
        }

        return;
    }

    Trajectory Planner::changeTrajectoryHelper(const Trajectory & incomingTraj, 
                                                const bool & switchToIncoming,
                                                const int & trajFlag,
                                                Gap * incomingGap) 
    {
        // publishToMpc_ = true;
                
        if (switchToIncoming) 
        {
            setCurrentTraj(incomingTraj);
            setCurrentTrajTrackingStartTime(ros::Time::now());
            setCurrentTrajLifespan(ros::Duration(incomingTraj.getPathTiming().back() - incomingTraj.getPathTiming().front()));

            if (trajFlag == GAP && incomingGap) // gap trajectory
            {
                setCurrentLeftGapPtModelID(incomingGap->getLeftGapPt()->getModel());
                setCurrentRightGapPtModelID(incomingGap->getRightGapPt()->getModel());
            } else // ungap or idling trajectory
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
            setCurrentTrajTrackingStartTime(ros::Time::now());
            setCurrentTrajLifespan(ros::Duration(0.0));

            setCurrentLeftGapPtModelID(nullptr);
            setCurrentRightGapPtModelID(nullptr);

            trajVisualizer_->drawCurrentTrajectory(emptyTraj);
            trajVisualizer_->drawTrajectorySwitchCount(trajectoryChangeCount_, emptyTraj);
            trajectoryChangeCount_++;            

            return emptyTraj;
        }        
    }

    Trajectory Planner::compareToCurrentTraj(const Trajectory & incomingTraj,
                                                const std::vector<sensor_msgs::LaserScan> & futureScans,
                                                const int & trajFlag,
                                                Gap * incomingGap,
                                                const bool & isCurrentGapFeasible) // bool isIncomingGapAssociated,
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "[compareToCurrentTraj()]");
        boost::mutex::scoped_lock gapset(gapMutex_);
        
        // Gap * incomingGap = (trajFlag == 0 ? feasibleGaps.at(lowestCostTrajIdx) : nullptr);
        // Trajectory incomingTraj = trajs.at(lowestCostTrajIdx);
        Trajectory currentTraj = getCurrentTraj();

        if (trajFlag != GAP && trajFlag != UNGAP && trajFlag != IDLING)
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "    compareToCurrentTraj: invalid trajFlag: " << trajFlag);
            return currentTraj;
        }

        try
        {
            // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapPtModelID() << ", current right gap index: " << getCurrentRightGapPtModelID());

            // ROS_INFO_STREAM("current gap indices: " << getCurrentLeftGapPtModelID() << ", " << getCurrentRightGapPtModelID());
            
            //////////////////////////////////////////////////////////////////////////////
            // Transform into the current robot frame to score against the current scan //
            //////////////////////////////////////////////////////////////////////////////
            geometry_msgs::PoseArray incomingPathRobotFrame = gapTrajGenerator_->transformPath(incomingTraj.getPathOdomFrame(), odom2rbt_);

            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    evaluating incoming trajectory");
            std::vector<float> incomingPathPoseCosts;
            float incomingPathTerminalPoseCost;
            trajEvaluator_->evaluateTrajectory(incomingTraj, incomingPathPoseCosts, incomingPathTerminalPoseCost, futureScans, 0);

            float incomingPathCost = incomingPathTerminalPoseCost + std::accumulate(incomingPathPoseCosts.begin(), incomingPathPoseCosts.end(), float(0)) / incomingPathPoseCosts.size();
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    incoming trajectory received a cost of: " << incomingPathCost);
            

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
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        trajectory change " << trajectoryChangeCount_ <<  
                                                                ": current path is of length zero, " << incomingPathStatus);                
                return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath, trajFlag, incomingGap);
            } 

            ///////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the currently executing gap is not feasible //
            ///////////////////////////////////////////////////////////////////////////////
            if (!isCurrentGapFeasible) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        trajectory change " << trajectoryChangeCount_ <<  
                                                                ": current gap is not feasible, " << incomingPathStatus);                
                return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath, trajFlag, incomingGap);
            } 

            // Transform current traj into most recent robot frame to score against the current scan
            geometry_msgs::PoseArray updatedCurrentPathRobotFrame = gapTrajGenerator_->transformPath(currentTraj.getPathOdomFrame(), odom2rbt_);
            std::vector<float> updatedCurrentPathTiming = currentTraj.getPathTiming();
            Trajectory updatedCurrentTraj(updatedCurrentPathRobotFrame, updatedCurrentPathTiming);
            updatedCurrentTraj.setPathOdomFrame(currentTraj.getPathOdomFrame()); // odom frame traj will not change

            //  FIX: preserve velocity commands
            updatedCurrentTraj.setVcmd(currentTraj.getVcmd());
            updatedCurrentTraj.setWcmd(currentTraj.getWcmd());

            visualizePoseArray(updatedCurrentPathRobotFrame, "updated_current_traj");


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
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        trajectory change " << trajectoryChangeCount_ <<  
                                                                ": old path length less than 2, " << incomingPathStatus);

                return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath, trajFlag, incomingGap);
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Enact a trajectory switch if the we have been tracking the currently executing path for its entire lifespan  //
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////   
            ros::Duration currentTrajTrackingTime = ros::Time::now() - currentTrajTrackingStartTime_;
            
            ////////////////// unused max tracking time //////////////////
             double lifespan_sec = currentTrajLifespan_.toSec();
            // lifespan_sec = std::max(0.0, lifespan_sec - 4.0);
            double tracking_sec = currentTrajTrackingTime.toSec();
            // ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "tracking_sec: " << tracking_sec); 
            // // if (currentTrajTrackingTime > currentTrajLifespan_)
            // if (tracking_sec + 4 > lifespan_sec)
            double max_track_time = 0.1; 
            // if (tracking_sec >= max_track_time)
            ////////////////// unused max tracking time //////////////////


            // if (currentTrajTrackingTime > currentTrajLifespan_)
            if (tracking_sec >= max_track_time)
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        trajectory change " << trajectoryChangeCount_ <<  
                                                                ": current path has been tracked for its entire lifespan, " << incomingPathStatus);
                return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath, trajFlag, incomingGap);
            }

            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //  Compare the costs of the incoming trajectory with the cost of the current trajectory to see if we need to switch //
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    evaluating current trajectory");            
            
            Trajectory reducedCurrentTraj(reducedCurrentPathRobotFrame, reducedCurrentPathTiming);
              //FIX: preserve velocity commands
            reducedCurrentTraj.setVcmd(currentTraj.getVcmd());
            reducedCurrentTraj.setWcmd(currentTraj.getWcmd());

            std::vector<float> reducedCurrentPathPoseCosts;
            float reducedCurrentPathTerminalPoseCost;
            trajEvaluator_->evaluateTrajectory(reducedCurrentTraj, reducedCurrentPathPoseCosts, reducedCurrentPathTerminalPoseCost, futureScans, updatedCurrentPathPoseIdx);
            float reducedCurrentPathSubCost = reducedCurrentPathTerminalPoseCost + std::accumulate(reducedCurrentPathPoseCosts.begin(), reducedCurrentPathPoseCosts.end(), float(0)) / reducedCurrentPathPoseCosts.size();
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "    reduced current trajectory (length: " << reducedCurrentPathRobotFrame.poses.size() << " received a subcost of: " << reducedCurrentPathSubCost);

            // if (reducedCurrentPathSubCost == std::numeric_limits<float>::infinity()) 
            // {
            //     ROS_ERROR_STREAM_NAMED("GapTrajectoryGeneratorV2", "        trajectory change " << trajectoryChangeCount_ << 
            //                                                 ": current trajectory is of cost infinity," << incomingPathStatus);
            //     return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath, trajFlag, incomingGap);
            // }
            
            // std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();

            // float timeTakenInMilliseconds = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - prevTrajSwitchTime_).count();
            // float timeTakenInSeconds = timeTakenInMilliseconds * 1.0e-6;

            // if (timeTakenInSeconds > currentTraj.getPathTiming().back()) 
            // {
            //     ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        trajectory change " << trajectoryChangeCount_ << 
            //                                                 ": current trajectory has been tracked for " << timeTakenInSeconds << " seconds, " << incomingPathStatus);
            //     return changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath, trajFlag, incomingGap);
            // }

            // if planner has been idling for longer than maxt and incoming score is better

            // if (incomingPathCost < reducedCurrentPathSubCost) 
            // {
            //     ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        trajectory change " << trajectoryChangeCount_ << 
            //                                                 ": incoming trajectory is lower score");
            //     changeTrajectoryHelper(incomingTraj, ableToSwitchToIncomingPath, trajFlag, incomingGap);
            // }
            
            
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        trajectory maintain");
            // currentTrajectoryPublisher_.publish(currentTraj.getPathRbtFrame());
            trajVisualizer_->drawCurrentTrajectory(reducedCurrentTraj);

            return updatedCurrentTraj;

        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "        compareToCurrentTraj() out of range exception: " << e.what());
            return currentTraj;
        } catch (const std::exception & e)
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "        compareToCurrentTraj() exception: " << e.what());
            return currentTraj;
        }
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
        ROS_INFO_STREAM_NAMED("Planner", "[runPlanningLoop()]: count " << timeKeeper_->getPlanningLoopCalls());

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

        timeKeeper_->startTimer(PLAN);

        trajVisualizer_->drawPlanningLoopIdx(timeKeeper_->getPlanningLoopCalls());

        isGoalReached();

        std::vector<Gap *> copiedRawGaps = deepCopyCurrentRawGaps();
        std::vector<Gap *> planningGaps = deepCopyCurrentSimplifiedGaps();
        std::vector<Ungap *> ungaps, recedingUngaps;

        // Gap Tubes
        std::vector<GapTube *> gapTubes; /**< Set of gap tubes */

        int gapCount = planningGaps.size();

        if (gapCount == 0)
        {
            ROS_WARN_STREAM_NAMED("Planner", "No gaps to plan for");
            chosenTraj = Trajectory();
            trajFlag = NONE;
            return;
        }

        if (cfg_.planning.gap_prop == 1)
        {
            //////////////////////////////////////////////////////////////////////////////////////
            //                                      ATTACH UNGAP IDS                            //
            //////////////////////////////////////////////////////////////////////////////////////

            attachUngapIDs(planningGaps, ungaps);
            
            //////////////////////////////////////////////////////////////////////////////////////
            //                              PRUNE APPROACHING UNGAPS                            //
            //////////////////////////////////////////////////////////////////////////////////////

            recedingUngaps = pruneApproachingUngaps(ungaps);
        }

        //////////////////////////////////////////////////////////////////////////////////////
        //                                  GAP MANIPULATION                                //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(GAP_MANIP);
        std::vector<Gap *> manipulatedGaps = manipulateGaps(planningGaps);
        timeKeeper_->stopTimer(GAP_MANIP);

        bool isCurrentGapFeasible = false;

        if (cfg_.planning.gap_prop == 1)
        {
            //////////////////////////////////////////////////////////////////////////////////////
            //                            GAP POINT PROPAGATION (v2)                            //
            //////////////////////////////////////////////////////////////////////////////////////

            timeKeeper_->startTimer(GAP_PROP);
            propagateGapPointsV2(manipulatedGaps, gapTubes);
            timeKeeper_->stopTimer(GAP_PROP);

            //////////////////////////////////////////////////////////////////////////////////////
            //                              GAP MANIPULATION                                    //
            //////////////////////////////////////////////////////////////////////////////////////

            gapGoalPlacementV2(gapTubes);

            ungapGoalPlacement(recedingUngaps);

            //////////////////////////////////////////////////////////////////////////////////////
            //                           GAP FEASIBILITY CHECK (v2)                             //
            //////////////////////////////////////////////////////////////////////////////////////

            timeKeeper_->startTimer(GAP_FEAS);
            gapSetFeasibilityCheckV2(gapTubes, isCurrentGapFeasible);
            timeKeeper_->stopTimer(GAP_FEAS);

            //////////////////////////////////////////////////////////////////////////////////////
            //                         UNGAP FEASIBILITY CHECK                                  //
            //////////////////////////////////////////////////////////////////////////////////////

            ungapSetFeasibilityCheck(recedingUngaps);
        }

        // Have to run here because terminal gap goals are set during feasibility check
        // gapVisualizer_->drawManipGaps(manipulatedGaps, std::string("manip"));
        gapVisualizer_->drawGapTubes(gapTubes);
        gapVisualizer_->drawManipGapModels(manipulatedGaps, std::string("manip"));
        goalVisualizer_->drawGapTubeGoals(gapTubes);

        // gapCount = feasibleGaps.size();

        //////////////////////////////////////////////////////////////////////////////////////
        //                              FUTURE SCAN PROPAGATION                             //
        //////////////////////////////////////////////////////////////////////////////////////
        std::vector<sensor_msgs::LaserScan> futureScans;
        timeKeeper_->startTimer(SCAN_PROP);
        if (cfg_.planning.future_scan_propagation)
        {
            if (cfg_.planning.egocircle_prop_cheat)
            {
                throw std::runtime_error("cheat not implemented"); // futureScans = dynamicScanPropagator_->propagateCurrentLaserScanCheat(currentTrueAgentPoses_, currentTrueAgentVels_);
            }
                else
            {
                futureScans = dynamicScanPropagator_->propagateCurrentLaserScan(copiedRawGaps);        
                publishFutureScans(futureScans);
       
            }
        }
            else 
        {
            sensor_msgs::LaserScan currentScan = *scan_.get();
            futureScans = std::vector<sensor_msgs::LaserScan>(int(cfg_.traj.integrate_maxt/cfg_.traj.integrate_stept) + 1, currentScan);
        }
        timeKeeper_->stopTimer(SCAN_PROP);

        std::vector<Trajectory> gapTrajs;
        std::vector<std::vector<float>> gapTrajPoseCosts; 
        std::vector<float> gapTrajTerminalPoseCosts;

        std::vector<Trajectory> ungapTrajs;
        std::vector<std::vector<float>> ungapTrajPoseCosts; 
        std::vector<float> ungapTrajTerminalPoseCosts; 

        std::vector<Trajectory> idlingTrajs;
        std::vector<std::vector<float>> idlingPathPoseCosts; 
        std::vector<float> idlingPathTerminalPoseCosts; 

        if (cfg_.planning.gap_prop == 1)
        {
            //////////////////////////////////////////////////////////////////////////////////////
            //                        GAP TRAJECTORY GENERATION AND SCORING (V2)                //
            //////////////////////////////////////////////////////////////////////////////////////

            timeKeeper_->startTimer(GAP_TRAJ_GEN);
            generateGapTrajsV2(gapTubes, gapTrajs, gapTrajPoseCosts, gapTrajTerminalPoseCosts, futureScans);
            timeKeeper_->stopTimer(GAP_TRAJ_GEN);

            //////////////////////////////////////////////////////////////////////////////////////
            //                         UNGAP TRAJECTORY GENERATION AND SCORING                  //
            //////////////////////////////////////////////////////////////////////////////////////

            // timeKeeper_->startTimer(UNGAP_TRAJ_GEN);
            // generateUngapTrajs(recedingUngaps, ungapTrajs, ungapTrajPoseCosts, ungapTrajTerminalPoseCosts, futureScans);
            // timeKeeper_->stopTimer(UNGAP_TRAJ_GEN);

            //////////////////////////////////////////////////////////////////////////////////////
            //                        IDLING TRAJECTORY GENERATION AND SCORING                  //
            //////////////////////////////////////////////////////////////////////////////////////

            // timeKeeper_->startTimer(IDLING_TRAJ_GEN);
            // generateIdlingTraj(idlingTrajs, idlingPathPoseCosts, idlingPathTerminalPoseCosts, futureScans);         
            // timeKeeper_->stopTimer(IDLING_TRAJ_GEN);
        } 

        ROS_INFO_STREAM_NAMED("Planner", "       ungap trajs size: " << ungapTrajs.size());
        ROS_INFO_STREAM_NAMED("Planner", "       gap trajs size: " << gapTrajs.size());
        ROS_INFO_STREAM_NAMED("Planner", "       idling gap trajs size: " << idlingTrajs.size());

        //////////////////////////////////////////////////////////////////////////////////////
        //                               GAP TRAJECTORY SELECTION                           //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(TRAJ_PICK);
        int lowestCostTrajIdx = -1;
        trajFlag = NONE;
        
        pickTraj(trajFlag, lowestCostTrajIdx, 
                    gapTrajs, gapTrajPoseCosts, gapTrajTerminalPoseCosts,
                    ungapTrajs, ungapTrajPoseCosts, ungapTrajTerminalPoseCosts,
                    idlingTrajs, idlingPathPoseCosts, idlingPathTerminalPoseCosts);
        timeKeeper_->stopTimer(TRAJ_PICK);

        //////////////////////////////////////////////////////////////////////////////////////
        //                              GAP TRAJECTORY COMPARISON                           //
        //////////////////////////////////////////////////////////////////////////////////////

        timeKeeper_->startTimer(TRAJ_COMP);

        Gap * incomingGap = nullptr;
        Ungap * incomingUngap = nullptr;
        Trajectory incomingTraj;

        if (lowestCostTrajIdx < 0)
        {
            ROS_WARN_STREAM_NAMED("Planner", "       no trajectory selected, lowestCostTrajIdx: " << lowestCostTrajIdx);
            chosenTraj = Trajectory();
            trajFlag = NONE;
            return;
        } else if (trajFlag == NONE)
        {
            ROS_INFO_STREAM_NAMED("Planner", "       comparing to none trajectory");
            chosenTraj = Trajectory();
            trajFlag = NONE;
            return;
        } else if (trajFlag == GAP) // comparing to traj within one of the gaps
        {
            ROS_INFO_STREAM_NAMED("Planner", "       comparing to actual trajectory");

            if (lowestCostTrajIdx >= gapTrajs.size())
            {
                ROS_WARN_STREAM_NAMED("Planner", "       lowest cost gap trajectory index out of bounds for gapTrajs");
                chosenTraj = Trajectory();
                trajFlag = NONE;
                return;
            }

            incomingGap = gapTubes.at(lowestCostTrajIdx)->at(0);
            incomingTraj = gapTrajs.at(lowestCostTrajIdx);
        } else if (trajFlag == UNGAP) // comparing to un-gap traj
        {
            ROS_INFO_STREAM_NAMED("Planner", "       comparing to un-gap trajectory");

            if (lowestCostTrajIdx >= ungapTrajs.size())
            {
                ROS_WARN_STREAM_NAMED("Planner", "       lowest cost ungap trajectory index out of bounds");
                chosenTraj = Trajectory();
                trajFlag = NONE;
                return;
            }
            incomingUngap = recedingUngaps.at(lowestCostTrajIdx);
            ungapRbtSpeed_ = incomingUngap->getRbtSpeed();
            incomingTraj = ungapTrajs.at(lowestCostTrajIdx);
        } else if (trajFlag == IDLING)
        {
            ROS_INFO_STREAM_NAMED("Planner", "       comparing to idling trajectory");
            if (lowestCostTrajIdx >= idlingTrajs.size())
            {
                ROS_WARN_STREAM_NAMED("Planner", "       lowest cost idling trajectory index out of bounds");
                chosenTraj = Trajectory();
                trajFlag = NONE;
                return;
            }

            incomingTraj = idlingTrajs.at(lowestCostTrajIdx);
        } else
        {
            ROS_WARN_STREAM_NAMED("Planner", "       unknown trajectory flag");

            chosenTraj = Trajectory();
            trajFlag = NONE;
            return;            
        }

        chosenTraj = compareToCurrentTraj(incomingTraj,           
                                            futureScans,
                                            trajFlag,
                                            incomingGap,
                                            isCurrentGapFeasible); // incomingGap, isCurrentGapFeasible,

        timeKeeper_->stopTimer(TRAJ_COMP);

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

        timeKeeper_->stopTimer(PLAN);
        timeKeeper_->computeAverageNumberGaps(gapCount);

        return;
    }

    void Planner::attachUngapIDs(const std::vector<Gap *> & planningGaps,
                                    std::vector<Ungap *> & ungaps)
    {
        ROS_INFO_STREAM_NAMED("Planner", "[attachUngapIDs()]");

        try
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
                        ROS_INFO_STREAM_NAMED("Planner", "  attaching ungap id: " << ungapID << " to");
                        ROS_INFO_STREAM_NAMED("Planner", "    gap" << (i / 2) << " right point: " << planningGaps.at(i / 2)->getLeftGapPt()->getModel()->getGapState().transpose());
                        ROS_INFO_STREAM_NAMED("Planner", "    gap" << (nextIdx / 2) << " left point: " << planningGaps.at(nextIdx / 2)->getRightGapPt()->getModel()->getGapState().transpose());

                        planningGaps.at(i / 2)->getRightGapPt()->setUngapID(ungapID);
                        planningGaps.at(i / 2)->getRightGapPt()->getModel()->setUngap();

                        planningGaps.at(nextIdx / 2)->getLeftGapPt()->setUngapID(ungapID);
                        planningGaps.at(nextIdx / 2)->getLeftGapPt()->getModel()->setUngap();

                        ungaps.push_back(new Ungap(planningGaps.at(i / 2)->getRightGapPt(), 
                                                    planningGaps.at(nextIdx / 2)->getLeftGapPt(),
                                                    ungapID));
                    } else
                    {
                        ROS_INFO_STREAM_NAMED("Planner", "  attaching ungap id: " << ungapID << " to");
                        ROS_INFO_STREAM_NAMED("Planner", "    gap" << (i / 2) << " left point: " << planningGaps.at(i / 2)->getLeftGapPt()->getModel()->getGapState().transpose());
                        ROS_INFO_STREAM_NAMED("Planner", "    gap" << (nextIdx / 2) << " right point: " << planningGaps.at(nextIdx / 2)->getRightGapPt()->getModel()->getGapState().transpose());

                        planningGaps.at(i / 2)->getLeftGapPt()->setUngapID(ungapID);
                        planningGaps.at(i / 2)->getLeftGapPt()->getModel()->setUngap();

                        planningGaps.at(nextIdx / 2)->getRightGapPt()->setUngapID(ungapID);
                        planningGaps.at(nextIdx / 2)->getRightGapPt()->getModel()->setUngap();
                        
                        ungaps.push_back(new Ungap(planningGaps.at(nextIdx / 2)->getRightGapPt(), 
                                                    planningGaps.at(i / 2)->getLeftGapPt(),
                                                    ungapID));
                    }
                }
            }
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "    attachUngapIDs() out of range exception: " << e.what());
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

        return (distCheck && speedCheck && angleCheck); // distCheck; // 
    }

    std::vector<Ungap *> Planner::pruneApproachingUngaps(const std::vector<Ungap *> & ungaps)
    {
        ROS_INFO_STREAM_NAMED("Planner", "[pruneApproachingUngaps()]");

        std::vector<Ungap *> recedingUngaps;

        try
        {

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
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "    pruneApproachingUngaps() out of range exception: " << e.what());
        }

        return recedingUngaps;
    }

geometry_msgs::Twist Planner::ctrlGeneration(const geometry_msgs::PoseArray & localTrajectory,
                                             int & trajFlag,
                                             float v_cmd,
                                             float w_cmd)
    {
        ROS_INFO_STREAM_NAMED("Controller", "[ctrlGeneration()]");
        timeKeeper_->startTimer(CONTROL);
        
        geometry_msgs::Twist rawCmdVel = geometry_msgs::Twist();
        geometry_msgs::Twist cmdVel = rawCmdVel;

        try
        {

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
                ROS_INFO_STREAM_NAMED("Planner", "trajFlag is NONE, obstacle avoidance control chosen.");
                if (cfg_.planning.holonomic)
                {
                    rawCmdVel = trajController_->obstacleAvoidanceControlLaw();
                } else
                {
                    rawCmdVel = trajController_->obstacleAvoidanceControlLawNonHolonomic();
                }

                return rawCmdVel;
            } else if (localTrajectory.poses.size() == 0) 
            {
                ROS_WARN_STREAM_NAMED("Controller", "Available Execution Traj length: " << localTrajectory.poses.size() << " == 0, obstacle avoidance control chosen.");
                
                if (cfg_.planning.holonomic)
                {
                    rawCmdVel = trajController_->obstacleAvoidanceControlLaw();
                } else
                {
                    rawCmdVel = trajController_->obstacleAvoidanceControlLawNonHolonomic();
                }
                
                return rawCmdVel;
            } else if (cfg_.ctrl.man_ctrl)  // MANUAL CONTROL 
            {
                ROS_INFO_STREAM_NAMED("Controller", "Manual control chosen.");
                // rawCmdVel = trajController_->manualControlLawKeyboard();
                // rawCmdVel = trajController_->manualControlLawReconfig();
                // rawCmdVel = trajController_->manualControlLawPrescribed(currPoseOdomFrame);
            // } else if (cfg_.ctrl.mpc_ctrl) // MPC CONTROL
            // { 
            //     rawCmdVel = mpcTwist_;
            } else if (cfg_.ctrl.feedback_ctrl && (trajFlag == UNGAP || trajFlag == GAP)) // FEEDBACK CONTROL 
            {
                ROS_INFO_STREAM_NAMED("Controller", "Trajectory tracking control chosen.");

                // obtain current robot pose in odom frame

                // traj in odom frame here

                // get point along trajectory to target/move towards
                targetTrajectoryPoseIdx_ = trajController_->extractTargetPoseIdx(currPoseOdomFrame, localTrajectory);

                geometry_msgs::Pose targetTrajectoryPose = localTrajectory.poses.at(targetTrajectoryPoseIdx_);

                float trackingSpeed = (trajFlag == UNGAP) ? ungapRbtSpeed_ : cfg_.rbt.vx_absmax;

                timeKeeper_->startTimer(FEEBDACK);
                // ROS_ERROR_STREAM_NAMED("Controller", "v_cmd: " << v_cmd << " w_cmd: " << w_cmd);

                if (!std::isnan(v_cmd) && !std::isnan(w_cmd))
                {
                    ROS_INFO_STREAM_NAMED("Controller", "Using external DWA command v=" << v_cmd << ", w=" << w_cmd);

                    rawCmdVel.linear.x  = v_cmd;
                    rawCmdVel.linear.y  = 0.0;
                    rawCmdVel.angular.z = w_cmd;

                    cmdVel = trajController_->processCmdVelNonHolonomic(
                                currPoseOdomFrame,
                                targetTrajectoryPose,
                                rawCmdVel,
                                rbtPoseInSensorFrame_);
                }
                else
                {

                    if (cfg_.planning.holonomic)
                    {
                        rawCmdVel = trajController_->constantVelocityControlLaw(currPoseOdomFrame, targetTrajectoryPose, trackingSpeed);
                    } else
                    {
                        rawCmdVel = trajController_->constantVelocityControlLawNonHolonomicLookahead(currPoseOdomFrame, targetTrajectoryPose, trackingSpeed);
                    
                        cmdVel = trajController_->processCmdVelNonHolonomic(currPoseOdomFrame,
                                                                            targetTrajectoryPose,
                                                                            rawCmdVel,
                                                                            rbtPoseInSensorFrame_); 
                    }
                }

                timeKeeper_->stopTimer(FEEBDACK);
            } else
            {
                ROS_ERROR_STREAM_NAMED("Controller", "No control method selected");
            }

            timeKeeper_->startTimer(PO);

            if (cfg_.planning.holonomic)
            {
                cmdVel = trajController_->processCmdVel(rawCmdVel,
                                                        rbtPoseInSensorFrame_); 
            } else
            {
                // running inside if statement
            }

            timeKeeper_->stopTimer(PO);

            timeKeeper_->stopTimer(CONTROL);
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("Planner", "    ctrlGeneration() out of range exception: " << e.what());
        } catch (const std::exception & e)
        {
            ROS_WARN_STREAM_NAMED("Planner", "    ctrlGeneration() exception: " << e.what());
        }

        return cmdVel;
    }

    void Planner::setCurrentLeftGapPtModelID(Estimator * leftModel) 
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

    void Planner::setCurrentRightGapPtModelID(Estimator * rightModel) 
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
        setCurrentTraj(Trajectory());

        // changeTrajectoryHelper(Trajectory(), 
        //                         false,
        //                         NONE,
        //                         nullptr);

        // hasGlobalGoal_ = false; 
        // hasLaserScan_ = false;

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

    void Planner::publishFutureScans(const std::vector<sensor_msgs::LaserScan>& futureScans)
    {
    static std::vector<ros::Publisher> futureScanPubs;

    // Create publishers only once (latched)
    if (futureScanPubs.empty())
    {
        futureScanPubs.reserve(futureScans.size());
        for (size_t k = 0; k < futureScans.size(); ++k)
        {
            std::string topic = "/future_scan_" + std::to_string(k);
            futureScanPubs.push_back(nh_.advertise<sensor_msgs::LaserScan>(topic, 1, true));
            ROS_INFO_STREAM_NAMED("Planner", "Advertising future scan topic: " << topic);
        }
    }

    // Publish each future scan
    for (size_t k = 0; k < futureScans.size(); ++k)
    {
        futureScanPubs[k].publish(futureScans[k]);
    }
    }

    void Planner::visualizePoseArray(const geometry_msgs::PoseArray & path,
                                 const std::string & ns)
{
    if (path.poses.empty())
        return;

    visualization_msgs::Marker line;
    line.header = path.header;
    line.ns = ns;
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;

    // --- THICK LINE ---
    line.scale.x = 0.06;   // cm line width

    // --- YELLOW COLOR ---
    line.color.r = 1.0;
    line.color.g = 1.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    for (const auto &pose : path.poses)
    {
        geometry_msgs::Point p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = 0.05;
        line.points.push_back(p);
    }

    transformed_path_pub_.publish(line);
}

}