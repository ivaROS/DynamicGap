#pragma once

#include <ros/ros.h>

#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numeric>
#include <iostream>
#include <chrono>

#include <math.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>

#include "dynamic_gap/TrajPlan.h"
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/gap_estimation/GapAssociator.h>
// #include <dynamic_gap/helper.h>
// #include <dynamic_gap/trajectory_follower.h>
#include <dynamic_gap/gap_detection/GapDetector.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/scan_separation/StaticScanSeparator.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/visualization/GapVisualizer.h>
#include <dynamic_gap/visualization/GoalVisualizer.h>
#include <dynamic_gap/visualization/TrajectoryVisualizer.h>
#include <dynamic_gap/goal_management/GoalSelector.h>
#include <dynamic_gap/trajectory_scoring/TrajectoryScorer.h>
#include <dynamic_gap/trajectory_generation/GapManipulator.h>
#include <dynamic_gap/trajectory_tracking/TrajectoryController.h>
#include <dynamic_gap/gap_feasibility/GapFeasibilityChecker.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <omp.h>

// #include <dynamic_reconfigure/server.h>
// #include <dynamic_gap/dgConfig.h>

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

namespace dynamic_gap
{
    class Planner
    {
    private:
        geometry_msgs::TransformStamped map2rbt_; /**< Transformation from map frame to robot frame */
        geometry_msgs::TransformStamped odom2rbt_; /**< Transformation from odometry frame to robot frame */
        geometry_msgs::TransformStamped rbt2odom_; /**< Transformation from robot frame to odometry frame */
        geometry_msgs::TransformStamped map2odom_;
        geometry_msgs::TransformStamped cam2odom_;
        geometry_msgs::TransformStamped rbt2cam_;

        geometry_msgs::PoseStamped rbtPoseInRbtFrame_;
        geometry_msgs::PoseStamped rbtPoseInSensorFrame_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener * tfListener_ = NULL;

        ros::NodeHandle nh;
        ros::Publisher currentTrajectoryPublisher_;
        ros::Publisher staticScanPublisher_;

        std::vector<int> rawAssocation_, simpAssociation_;
        std::vector<std::vector<float>> rawDistMatrix_, simpDistMatrix_;

        // Goals and stuff
        geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame_;  
        geometry_msgs::PoseStamped globalGoalOdomFrame_, globalGoalRobotFrame_;

        // Gaps:
        std::vector<dynamic_gap::Gap *> currRawGaps_, currSimplifiedGaps_;

        std::vector<dynamic_gap::Gap *> prevRawGaps_, prevSimplifiedGaps_;

        // std::vector<dynamic_gap::Gap *> associatedRawGaps_;
        // std::vector<dynamic_gap::Gap *> associatedSimplifiedGaps_;
        
        dynamic_gap::GapDetector * gapDetector_ = NULL;
        dynamic_gap::StaticScanSeparator * staticScanSeparator_ = NULL;
        dynamic_gap::GapVisualizer * gapVisualizer_ = NULL;
        dynamic_gap::GoalSelector * goalSelector_ = NULL;
        dynamic_gap::TrajectoryVisualizer * trajVisualizer_ = NULL;
        dynamic_gap::GoalVisualizer * goalVisualizer_ = NULL;
        dynamic_gap::TrajectoryScorer * trajScorer_ = NULL;
        dynamic_gap::GapTrajectoryGenerator * gapTrajGenerator_ = NULL;
        dynamic_gap::GapManipulator * gapManipulator_ = NULL;
        dynamic_gap::TrajectoryController * trajController_ = NULL;
        dynamic_gap::GapAssociator * gapAssociator_ = NULL;
        dynamic_gap::GapFeasibilityChecker * gapFeasibilityChecker_ = NULL;

        // Status
        bool hasGlobalGoal_ = false;
        bool initialized_ = false;

        int targetTrajectoryPoseIdx_ = 0;

        // Robot pose
        geometry_msgs::PoseStamped robotPoseOdomFrame_;

        sensor_msgs::LaserScan staticScan_;
        boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
        // boost::shared_ptr<sensor_msgs::LaserScan const> inflatedScan_;
        std::vector<sensor_msgs::LaserScan> futureScans_;

        dynamic_gap::DynamicGapConfig cfg_;

        boost::mutex gapsetMutex;

        dynamic_gap::Gap * currentGap_ = NULL;
        geometry_msgs::PoseArray currentPath_;
        std::vector<float> currentPathTiming_;

        boost::circular_buffer<float> cmdVelBuffer_;

        ros::Time tPreviousFilterUpdate_;

        geometry_msgs::TwistStamped currentRbtVel_, currentRbtAcc_;

        std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_, intermediateRbtAccs_;

        int currentModelIdx_;

        dynamic_gap::Estimator * currLeftGapPtModel_ = NULL;
        dynamic_gap::Estimator * currRightGapPtModel_ = NULL;
        
        geometry_msgs::TwistStamped currentPeakSplineVel_;

        // float curr_peakVelX_, curr_peakVelY_;

        int currentAgentCount_;

        std::vector<geometry_msgs::Pose> currentTrueAgentPoses_;
        std::vector<geometry_msgs::Vector3Stamped> currentTrueAgentVels_;

        std::vector<Eigen::Vector4f> currentEstimatedAgentStates_;

        int trajectoryChangeCount_;

        bool readyToPlan = false;

    public:
        Planner();

        ~Planner();

        /**
         * Set ros Buffer and etc.
         * 
         * @param None
         * @return initialization success / failure
         */
        bool initialize(const ros::NodeHandle&);

        /**
         * Check if reached goal using euclidean dist
         * @param None, internally stored goal location and robot position
         * @return bool reached
         */
        bool isGoalReached();

        /**
         * call back function to laserscan, externally linked
         * @param msg laser scan msg
         * @return None, laser scan msg stored locally
         */
        void laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

        void staticLaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

        /**
         * call back function to pose, pose information obtained here only used when a new goal is used
         * @param msg pose msg
         * @return None
         */
        void poseCB(const nav_msgs::Odometry::ConstPtr& msg);

        void jointPoseAccCB(const nav_msgs::Odometry::ConstPtr& odom_msg, const geometry_msgs::TwistStamped::ConstPtr& accel_msg);

        /**
         * Interface function for receiving global plan
         * @param plan, vector of PoseStamped
         * @return boolean type on whether successfully registered goal
         */
        bool setGoal(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame);

        /**
         * update all tf transform at the beginning of every planning cycle
         * @param None, all tf received via TF
         * @return None, all registered via internal variables in TransformStamped
         */
        void updateTF();

        /**
         * Setter and Getter of Current Gap, this is performed in the compareToCurrentTraj function
         */
        void setCurrentGap(dynamic_gap::Gap * currentGap) { delete currentGap_; currentGap_ = currentGap; return; }

        dynamic_gap::Gap * getCurrentGap() { return currentGap_; }

        /**
         * Setter and Getter of Current Trajectory, this is performed in the compareToCurrentTraj function
         */
        void setCurrentPath(const geometry_msgs::PoseArray & currentPath) { currentPath_ = currentPath; return; }

        geometry_msgs::PoseArray getCurrentPath() { return currentPath_;}

        void setCurrentPathTiming(const std::vector<float> & currentPathTiming) { currentPathTiming_ = currentPathTiming; return; }
        
        std::vector<float> getCurrentPathTiming() { return currentPathTiming_; }

        int getCurrentAgentCount() { return currentAgentCount_; }

        int initialized() { return initialized_; } 

        /**
         * Generate ctrl command to a target pose
         * TODO: fix vector pop and get rid of pose_counter
         * @param pose_arr_odom
         * @return command velocity by assigning to pass by reference
         */
        geometry_msgs::Twist ctrlGeneration(const geometry_msgs::PoseArray & localTrajectory);
        
        /**
         * Take current observed gaps and perform gap conversion
         * @param None, directly taken from private variable space
         * @return gap_set, simplfied radial prioritized gaps
         */
        std::vector<dynamic_gap::Gap *> gapManipulate(const std::vector<dynamic_gap::Gap *> & _observed_gaps);

        /**
         * 
         *
         */
        std::vector<std::vector<float>> generateGapTrajs(std::vector<dynamic_gap::Gap *>& vec, std::vector<geometry_msgs::PoseArray>& res, std::vector<std::vector<float>>& res_time_traj);

        /**
         * Callback function to config object
         * @param incoming config
         * @param level Level of incoming config
         */
        // void rcfgCallback(dynamic_gap::dgConfig &config, uint32_t level);

        /**
         * Pick the best trajectory from the current set
         * @param Vector of PoseArray
         * @param Vector of corresponding trajectory scores
         * @return the best trajectory
         */
        int pickTraj(const std::vector<geometry_msgs::PoseArray> & paths, 
                     const std::vector<std::vector<float>> & pathTimings);

        /**
         * Compare to the old trajectory and pick the best one
         * @param incoming trajectory
         * @return the best trajectory  
         */
        geometry_msgs::PoseArray compareToCurrentTraj(dynamic_gap::Gap * chosenGap, 
                                                  const geometry_msgs::PoseArray & chosenPath,                                                        
                                                  const std::vector<float> & chosenPathTiming,
                                                  const std::vector<dynamic_gap::Gap *> & feasibleGaps,
                                                  const bool & curr_exec_gap_feas);

        int getCurrentRightGapPtModelID();
        int getCurrentLeftGapPtModelID();

        /**
         * Conglomeration of getting a plan Trajectory
         * @return the trajectory
         */
        geometry_msgs::PoseArray runPlanningLoop();        

        /**
         * Gets the current position along the currently executing Trajectory
         */
        int egoTrajPosition(const geometry_msgs::PoseArray & curr);

        /**
         * Reset Planner, clears current observedSet
         */
        void reset();

        /**
         * Check if the robot has been stuck
         * @param command velocity
         * @return False if robot has been stuck for the past cfg.planning.halt_size iterations
         */
        bool recordAndCheckVel(const geometry_msgs::Twist & cmdVel);
    
        void updateModels(std::vector<dynamic_gap::Gap *> & _observed_gaps, 
                            const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                            const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                            const ros::Time & t_kf_update);

        void updateModel(const int & i, 
                            std::vector<dynamic_gap::Gap *>& _observed_gaps, 
                            const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                            const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                            const ros::Time & t_kf_update);

        void setCurrentLeftModel(dynamic_gap::Estimator * left_model);
        void setCurrentRightModel(dynamic_gap::Estimator * right_model);

        void setCurrentGapPeakVelocities(const float & peakVelX, const float & peakVelY);

        void printGapModels(const std::vector<dynamic_gap::Gap *> & gaps);
        void printGapAssociations(const std::vector<dynamic_gap::Gap *> & currentGaps, 
                                  const std::vector<dynamic_gap::Gap *> & previousGaps, 
                                  const std::vector<int> & association);

        std::vector<dynamic_gap::Gap *> gapSetFeasibilityCheck(bool & isCurrentGapFeasible); // bool & isCurrentGapAssociated, 
                                                             

        void agentOdomCB(const nav_msgs::Odometry::ConstPtr& msg);
        void visualizeComponents(const std::vector<dynamic_gap::Gap *> & manip_gap_set);

        void getFutureScans();        

        geometry_msgs::PoseArray changeTrajectoryHelper(dynamic_gap::Gap * chosenGap, 
                                                        const geometry_msgs::PoseArray & chosenPath, 
                                                        const std::vector<float> & chosenPathTiming, 
                                                        const bool & switchToIncoming);
    };
}