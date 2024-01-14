#include <dynamic_gap/goal_management/GoalSelector.h>

namespace dynamic_gap 
{    
    GoalSelector::GoalSelector(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) 
    {
        cfg_ = &cfg;
    }

    void GoalSelector::updateGlobalPathMapFrame(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame) 
    {
        // Incoming plan is in map frame
        boost::mutex::scoped_lock lock(goalSelectMutex_);
        boost::mutex::scoped_lock gplock(globalPlanMutex_);

        globalPlanMapFrame_ = globalPlanMapFrame;
    }

    void GoalSelector::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scanPtr_ = scan;
    }

    void GoalSelector::generateGlobalPathLocalWaypoint(const geometry_msgs::TransformStamped & map2rbt) 
    {
        boost::mutex::scoped_lock glock(goalSelectMutex_);
        boost::mutex::scoped_lock llock(scanMutex_);
        
        if (globalPlanMapFrame_.size() < 2) // No Global Path
            return;

        // ROS_INFO_STREAM("running generateGlobalPathLocalWaypoint");
        // getting snippet of global trajectory in robot frame (snippet is whatever part of global trajectory is within laser scan)
        std::vector<geometry_msgs::PoseStamped> globalPlanSnipperRobotFrame = getVisibleGlobalPlanSnippetRobotFrame(map2rbt);
        
        if (globalPlanSnipperRobotFrame.size() < 1) // relevant global plan snippet
            return;

        globalPathLocalWaypointRobotFrame_ = globalPlanSnipperRobotFrame.back();
    }

    std::vector<geometry_msgs::PoseStamped> GoalSelector::getVisibleGlobalPlanSnippetRobotFrame(const geometry_msgs::TransformStamped & map2rbt) 
    {
        // ROS_INFO_STREAM("getVisibleGlobalPlanSnippetRobotFrame");
        boost::mutex::scoped_lock gplock(globalPlanMutex_);
        std::vector<geometry_msgs::PoseStamped> globalPlan = globalPlanMapFrame_;
        // where is globalPlanMapFrame_ coming from?
        // globalPlan = globalPlanMapFrame_;

        // if (globalPlan.size() == 0) {
        //     ROS_FATAL_STREAM("Global Plan Length = 0");
        // }

        // transforming plan into robot frame
        for (int i = 0; i < globalPlan.size(); i++)
            tf2::doTransform(globalPlan.at(i), globalPlan.at(i), map2rbt);

        // ROS_INFO_STREAM("mod plan size: " << globalPlan.size());
        std::vector<float> planPoseNorms(globalPlan.size());
        std::vector<float> scanDistsAtPlanIndices(globalPlan.size());
        std::vector<float> scanMinusPlanPoseNormDiffs(globalPlan.size());

        for (int i = 0; i < planPoseNorms.size(); i++) 
        {
            planPoseNorms.at(i) = poseNorm(globalPlan.at(i)); // calculating distance to robot at each step of plan
            scanDistsAtPlanIndices.at(i) = calculateScanDistsAtPlanIndices(globalPlan.at(i));
            scanMinusPlanPoseNormDiffs.at(i) = (scanDistsAtPlanIndices.at(i) - (cfg_->rbt.r_inscr / 2.0)) - planPoseNorms.at(i);
        }

        // Find closest pose to robot to start the global plan snippet
        auto closestPlanPose = std::min_element(planPoseNorms.begin(), planPoseNorms.end());
        int closestPlanPoseIdx = std::distance(planPoseNorms.begin(), closestPlanPose);
        // ROS_INFO_STREAM("closestPlanPoseIdx: " << closestPlanPoseIdx);

        // find_if returns iterator for which the predicate (second input) is true within range.

        // firstNotVisibleGlobalPlanPose is the first point within the global plan that lies beyond the current scan.
        auto firstNotVisibleGlobalPlanPose = std::find_if(scanMinusPlanPoseNormDiffs.begin() + closestPlanPoseIdx, 
                                                          scanMinusPlanPoseNormDiffs.end(),
                                                          std::bind1st(std::mem_fun(&GoalSelector::isNotWithin), this));

        if (closestPlanPose == scanMinusPlanPoseNormDiffs.end()) 
        {
            ROS_ERROR_STREAM("No Global Plan pose within Robot scan");
            return std::vector<geometry_msgs::PoseStamped>(0);
        }

        int firstNotVisibleGlobalPlanPoseIdx = std::distance(scanMinusPlanPoseNormDiffs.begin(), firstNotVisibleGlobalPlanPose);

        std::vector<geometry_msgs::PoseStamped> visibleGlobalPlanSnippetRobotFrame(globalPlan.begin() + closestPlanPoseIdx, 
                                                                                   globalPlan.begin() + firstNotVisibleGlobalPlanPoseIdx);
        return visibleGlobalPlanSnippetRobotFrame;
    }

    int GoalSelector::poseIdxInScan(const geometry_msgs::PoseStamped & pose) 
    {
        float orientation = getPoseOrientation(pose);
        int index = theta2idx(orientation); // float(orientation + M_PI) / (scan_.get()->angle_increment);
        return index;
    }

    float GoalSelector::getPoseOrientation(const geometry_msgs::PoseStamped & pose) 
    {
        return std::atan2(pose.pose.position.y + 1e-3, pose.pose.position.x + 1e-3);
    }

    float GoalSelector::poseNorm(const geometry_msgs::PoseStamped & pose) 
    {
        return sqrt(pow(pose.pose.position.x, 2) + pow(pose.pose.position.y, 2));
    }

    float GoalSelector::calculateScanDistsAtPlanIndices(const geometry_msgs::PoseStamped & pose) 
    {
        sensor_msgs::LaserScan scan = *scanPtr_.get();

        int poseIdx = poseIdxInScan(pose);

        float scanRangeAtPoseIdx = scan.ranges.at(poseIdx);

        return scanRangeAtPoseIdx;
    }

    bool GoalSelector::isNotWithin(const float dist) 
    {
        return dist <= 0.0;
    }

    // This should return something in odom frame
    geometry_msgs::PoseStamped GoalSelector::getGlobalPathLocalWaypointOdomFrame(const geometry_msgs::TransformStamped & rbt2odom) 
    {
        geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame;
        tf2::doTransform(globalPathLocalWaypointRobotFrame_, globalPathLocalWaypointOdomFrame, rbt2odom);
        
        return globalPathLocalWaypointOdomFrame;
    }

    std::vector<geometry_msgs::PoseStamped> GoalSelector::getGlobalPathOdomFrame() 
    {
        return globalPlanMapFrame_;
    }
}