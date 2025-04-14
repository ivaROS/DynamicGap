#include <dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h>


namespace dynamic_gap 
{
    TrajectoryEvaluator::TrajectoryEvaluator(const DynamicGapConfig& cfg)
    {
        cfg_ = & cfg;
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

    void TrajectoryEvaluator::evaluateTrajectory(const Trajectory & traj,
                                                std::vector<float> & posewiseCosts,
                                                float & terminalPoseCost,
                                                const std::vector<sensor_msgs::LaserScan> & futureScans, 
                                                dynamic_gap::Gap* gap) 
    {    
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
        // Requires LOCAL FRAME

        Eigen::Vector2f leftGapRelVel(0, 0);
        geometry_msgs::TwistStamped RbtVelMsg;
        Eigen::Vector2f RbtVel(0, 0);
        Eigen::Vector2f leftGapRelPos(0, 0);
        Eigen::Vector2f rightGapRelVel(0, 0);
        Eigen::Vector2f rightGapRelPos(0, 0); 
        bool leftGapPtIsDynamic = false; 
        bool rightGapPtIsDynamic = false; 
        
        if(cfg_->planning.social_cost_function == 1)
        {
        if(gap)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());

        leftGapPtIsDynamic = gap->getLeftGapPt()->getUngapID()>=0; 
        if(leftGapPtIsDynamic)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());
         gap->getLeftGapPt()->getModel()->isolateGapDynamics();
        //  gap->leftGapPt__model_->isolateGapDynamics();
         leftGapRelVel = gap->getLeftGapPt()->getModel()->getGapVelocity();
         RbtVelMsg = gap->getLeftGapPt()->getModel()->getRobotVel();
         RbtVel << RbtVelMsg.twist.linear.x, RbtVelMsg.twist.linear.y;
         // ROS_ERROR_STREAM_NAMED("evalTraj", "leftGapRelVel: ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftGapRelVel);  
         leftGapRelPos = gap->getLeftGapPt()->getModel()->getState().head<2>(); //distance from robot to gap.
        // ROS_ERROR_STREAM_NAMED("evalTraj", "gap->leftGapPtModel_->getState(): ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftGapRelPos);  

        }

        rightGapPtIsDynamic = gap->getRightGapPt()->getUngapID()>=0; 
        if(rightGapPtIsDynamic)
        {
            // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
            // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());
         gap->getRightGapPt()->getModel()->isolateGapDynamics();
        //  gap->rightGapPt__model_->isolateGapDynamics();
         rightGapRelVel = gap->getRightGapPt()->getModel()->getGapVelocity();
        
         // ROS_ERROR_STREAM_NAMED("evalTraj", "rightGapRelVel: ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", rightGapRelVel);  
         rightGapRelPos = gap->getRightGapPt()->getModel()->getState().head<2>(); //distance from robot to gap.
        // ROS_ERROR_STREAM_NAMED("evalTraj", "gap->rightGapPtModel_->getState(): ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", rightGapRelPos);  

        }


        
    //     int i = 0; //ATODO DELETE
    //     for(i = 0; i < 1; i++)
    // {
        
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
    //     INFO("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());
        
    // }
        }
    }

        geometry_msgs::PoseArray path = traj.getPathRbtFrame();
        std::vector<float> pathTiming = traj.getPathTiming();
        
        posewiseCosts = std::vector<float>(path.poses.size());

        float leftGapPtCost = 0; 
        float rightGapPtCost = 0; 
        float weight = cfg_->planning.social_cost_weight; 


        for (int i = 0; i < posewiseCosts.size(); i++) 
        {
            if (leftGapPtIsDynamic){ // if(leftGapPtIsDynamic){
                geometry_msgs::Point posUncoverted = path.poses.at(i).position;
                Eigen::Vector2f Pos;
                Pos.x() = posUncoverted.x; 
                Pos.y() = posUncoverted.y;
                
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "leftGapRelPos (which is distance of robot to gap): "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftGapRelPos);

                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "Pos: "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", Pos);

                leftGapPtCost = relativeVelocityCost(leftGapRelVel, leftGapRelPos, Pos, RbtVel);
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(leftGapRelVel, leftGapRelPos, RbtVel)"); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftGapPtCost); 
            }
            

            if(rightGapPtIsDynamic){ //(rightGapPtIsDynamic){
                geometry_msgs::Point posUncoverted = path.poses.at(i).position;
                Eigen::Vector2f Pos;
                Pos.x() = posUncoverted.x; 
                Pos.y() = posUncoverted.y;
                
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "rightGapRelPos: "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightGapRelPos);

                rightGapPtCost = relativeVelocityCost(rightGapRelVel, rightGapRelPos, Pos, RbtVel);
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(rightGapRelVel, rightGapRelPos, RbtVel)"); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightGapPtCost); 
            }

            // std::cout << "regular range at " << i << ": ";
            posewiseCosts.at(i) = evaluatePose(path.poses.at(i), futureScans.at(i)) + weight * leftGapPtCost + weight * rightGapPtCost; ; //  / posewiseCosts.size()
            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " score: " << posewiseCosts.at(i));

            // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " score: " << posewiseCosts.at(i));
            // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", " social score: " <<  weight * leftGapPtCost + weight * rightGapPtCost);


        }
        float totalTrajCost = std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0));
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "             pose-wise cost: " << totalTrajCost);

        if (posewiseCosts.size() > 0) 
        {
            // obtain terminalGoalCost, scale by Q
            terminalPoseCost = cfg_->traj.Q_f * terminalGoalCost(*std::prev(path.poses.end()));

            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            terminal cost: " << terminalPoseCost);
        }
        
        // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "evaluateTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
        return;
    }


    void TrajectoryEvaluator::evaluateUngapTrajectory(const Trajectory & traj,
        std::vector<float> & posewiseCosts,
        float & terminalPoseCost,
        const std::vector<sensor_msgs::LaserScan> & futureScans, 
        dynamic_gap::Ungap* ungap) 
{    
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
        // Requires LOCAL FRAME

        Eigen::Vector2f leftUngapRelVel(0, 0);
        geometry_msgs::TwistStamped RbtVelMsg;
        Eigen::Vector2f RbtVel(0, 0);
        Eigen::Vector2f leftUngapRelPos(0, 0);
        Eigen::Vector2f rightUngapRelVel(0, 0);
        Eigen::Vector2f rightUngapRelPos(0, 0); 
        bool leftUngapPtIsDynamic = false; 
        bool rightUngapPtIsDynamic = false; 

        if(cfg_->planning.social_cost_function == 1)
        {
        if(ungap)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "ungap->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator",ungap->getUngapID());

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator","cfg_.planning.social_cost_function");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator",cfg_->planning.social_cost_function);


        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightUngapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator",ungap->getRightUngapPt()->getUngapID());
 
        // leftUngapPtIsDynamic =ungap->getLeftUngapPt()->getUngapID()>=0;  //ATODO UNCOMMENTED THIS

        if(leftUngapPtIsDynamic)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftUngapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator",ungap->getLeftUngapPt()->getUngapID());
       ungap->getLeftUngapPt()->getModel()->isolateGapDynamics();
        // ungap->leftGapPt__model_->isolateGapDynamics();
        leftUngapRelVel =ungap->getLeftUngapPt()->getModel()->getGapVelocity();
        RbtVelMsg =ungap->getLeftUngapPt()->getModel()->getRobotVel();
        RbtVel << RbtVelMsg.twist.linear.x, RbtVelMsg.twist.linear.y;
        // ROS_ERROR_STREAM_NAMED("evalTraj", "leftUngapRelVel: ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftUngapRelVel);  
        leftUngapRelPos =ungap->getLeftUngapPt()->getModel()->getState().head<2>(); //distance from robot toungap.
        // ROS_ERROR_STREAM_NAMED("evalTraj", "gap->leftUngapPtModel_->getState(): ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftUngapRelPos);  

        }

        // rightUngapPtIsDynamic =ungap->getRightUngapPt()->getUngapID()>=0;  //ATODO UNCOMMENTED THIS
        if(rightUngapPtIsDynamic)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightUngapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator",ungap->getRightUngapPt()->getUngapID());
       ungap->getRightUngapPt()->getModel()->isolateGapDynamics();
        // ungap->rightUngapPt__model_->isolateGapDynamics();
        rightUngapRelVel =ungap->getRightUngapPt()->getModel()->getGapVelocity();

        // ROS_ERROR_STREAM_NAMED("evalTraj", "rightUngapRelVel: ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", rightUngapRelVel);  
        rightUngapRelPos =ungap->getRightUngapPt()->getModel()->getState().head<2>(); //distance from robot toungap.
        // ROS_ERROR_STREAM_NAMED("evalTraj", "gap->rightUngapPtModel_->getState(): ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", rightUngapRelPos);  

        }



        //     int i = 0; //ATODO DELETE
        //     for(i = 0; i < 1; i++)
        // {

        //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightUngapPt()->getUngapID()");
        //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator",ungap->getRightUngapPt()->getUngapID());
        //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftUngapPt()->getUngapID()");
        //     INFO("TrajectoryEvaluator",ungap->getLeftUngapPt()->getUngapID());

        // }
        }
    }

        geometry_msgs::PoseArray path = traj.getPathRbtFrame();
        std::vector<float> pathTiming = traj.getPathTiming();

        posewiseCosts = std::vector<float>(path.poses.size());

        float leftUngapPtCost = 0; 
        float rightUngapPtCost = 0; 
        float weight = cfg_->planning.social_cost_weight; 


        for (int i = 0; i < posewiseCosts.size(); i++) 
        {
        if (leftUngapPtIsDynamic){ // if(leftUngapPtIsDynamic){
        geometry_msgs::Point posUncoverted = path.poses.at(i).position;
        Eigen::Vector2f Pos;
        Pos.x() = posUncoverted.x; 
        Pos.y() = posUncoverted.y;

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "leftUngapRelPos (which is distance of robot toungap): "); 
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftUngapRelPos);

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "Pos: "); 
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", Pos);

        leftUngapPtCost = relativeVelocityCost(leftUngapRelVel, leftUngapRelPos, Pos, RbtVel);
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(leftUngapRelVel, leftUngapRelPos, RbtVel)"); 
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftUngapPtCost); 
        }


        if(rightUngapPtIsDynamic){ //(rightUngapPtIsDynamic){
        geometry_msgs::Point posUncoverted = path.poses.at(i).position;
        Eigen::Vector2f Pos;
        Pos.x() = posUncoverted.x; 
        Pos.y() = posUncoverted.y;

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "rightUngapRelPos: "); 
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightUngapRelPos);

        rightUngapPtCost = relativeVelocityCost(rightUngapRelVel, rightUngapRelPos, Pos, RbtVel);
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(rightUngapRelVel, rightUngapRelPos, RbtVel)"); 
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightUngapPtCost); 
        }

        // std::cout << "regular range at " << i << ": ";
        posewiseCosts.at(i) = evaluatePose(path.poses.at(i), futureScans.at(i)) + weight * leftUngapPtCost + weight * rightUngapPtCost; ; //  / posewiseCosts.size()
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " score: " << posewiseCosts.at(i));

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " score: " << posewiseCosts.at(i));
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", " ungap social score: " <<  weight * leftUngapPtCost + weight * rightUngapPtCost);


        }
        float totalTrajCost = std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0));
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "             pose-wise cost: " << totalTrajCost);

        if (posewiseCosts.size() > 0) 
        {
        // obtain terminalGoalCost, scale by Q
        terminalPoseCost = cfg_->traj.Q_f * terminalGoalCost(*std::prev(path.poses.end()));

        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            terminal cost: " << terminalPoseCost);
        }

        // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "evaluateTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
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

    float TrajectoryEvaluator::evaluatePose(const geometry_msgs::Pose & pose,
                                      const sensor_msgs::LaserScan scan_k) 
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
                                                        ", closest scan point: " << range * std::cos(theta) << ", " << range * std::sin(theta) << ", static cost: " << cost);
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


    float TrajectoryEvaluator::relativeVelocityCost(Eigen::Vector2f relativeVel,
        Eigen::Vector2f relativeGapPos,
        Eigen::Vector2f trajPos,
        Eigen::Vector2f robotVel){
Eigen::Vector2f relVel = -relativeVel; // so velocity and position vector point in the same direction for the dot product

// ROS_ERROR_STREAM_NAMED("relvel cost: ", relVel << ", relativeGapPos: " << relativeGapPos << ", robotVel: " << robotVel);

// ROS_ERROR_STREAM_NAMED("relvel cost", "relativeGapPos: ");
// ROS_ERROR_STREAM_NAMED("relvel cost", relativeGapPos);

Eigen::Vector2f posToGapPtDist = relativeGapPos - trajPos;// distance between the current pos we're looking at and the gap point (which represents the dynamic obstacle)
// ROS_ERROR_STREAM_NAMED("relvel cost", "posToGapPtDist: ");
// ROS_ERROR_STREAM_NAMED("relvel cost", posToGapPtDist);




float cost = (std::max(relVel.dot(relativeGapPos), 0.0f) + robotVel.norm()) / posToGapPtDist.norm();

// ROS_ERROR_STREAM_NAMED("relvel cost", "std::max(relVel.dot(relativeGapPos), 0.0f");
// ROS_ERROR_STREAM_NAMED("relvel cost", std::max(relVel.dot(relativeGapPos), 0.0f));

// ROS_ERROR_STREAM_NAMED("relvel cost", "relVel.dot(relativeGapPos)");
// ROS_ERROR_STREAM_NAMED("relvel cost", relVel.dot(relativeGapPos));

// ROS_ERROR_STREAM_NAMED("GapTrajectoryGenerator", "relativeVelocityCost() !!UNWEIGHTED!! cost: ");
// ROS_ERROR_STREAM_NAMED("GapTrajectoryGenerator", cost);

return cost;
}
}