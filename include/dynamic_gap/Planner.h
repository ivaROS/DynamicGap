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
        geometry_msgs::TransformStamped map2rbt_;        // Transform
        geometry_msgs::TransformStamped odom2rbt_;
        geometry_msgs::TransformStamped rbt2odom_;
        geometry_msgs::TransformStamped map2odom_;
        geometry_msgs::TransformStamped cam2odom_;
        geometry_msgs::TransformStamped rbt2cam_;

        geometry_msgs::PoseStamped rbt_in_rbt;
        geometry_msgs::PoseStamped rbt_in_cam;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener *tfListener_;

        ros::NodeHandle nh;
        ros::Publisher currentTrajectoryPublisher_;
        ros::Publisher staticScanPublisher_;

        std::vector<int> rawAssocation_, simpAssociation_;
        std::vector<std::vector<float>> rawDistMatrix_, simpDistMatrix_;

        // Goals and stuff
        geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame_;  
        geometry_msgs::PoseStamped globalGoalOdomFrame_, globalGoalRobotFrame_;

        // Gaps:
        std::vector<dynamic_gap::Gap> rawGaps_, simplifiedGaps_;

        std::vector<dynamic_gap::Gap> previousRawGaps_, previousSimplifiedGaps_;

        std::vector<dynamic_gap::Gap> associatedRawGaps_;
        std::vector<dynamic_gap::Gap> associatedSimplifiedGaps_;
        
        dynamic_gap::GapDetector * gapDetector_;
        dynamic_gap::StaticScanSeparator * staticScanSeparator_;
        dynamic_gap::GapVisualizer * gapVisualizer_;
        dynamic_gap::GoalSelector * goalSelector_;
        dynamic_gap::TrajectoryVisualizer * trajVisualizer_;
        dynamic_gap::GoalVisualizer * goalvisualizer;
        dynamic_gap::TrajectoryScorer * trajScorer_;
        dynamic_gap::GapTrajectoryGenerator * gapTrajGenerator_;
        dynamic_gap::GapManipulator * gapManipulator_;
        dynamic_gap::TrajectoryController * trajController_;
        dynamic_gap::GapAssociator * gapAssociator_;
        dynamic_gap::GapFeasibilityChecker * gapFeasibilityChecker_;

        // Status
        bool hasGlobalGoal_ = false;
        bool initialized_ = false;

        int targetTrajectoryPoseIdx_ = 0;

        // Robot pose
        geometry_msgs::PoseStamped robotPoseOdomFrame_;

        sensor_msgs::LaserScan staticScan_;
        boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
        boost::shared_ptr<sensor_msgs::LaserScan const> inflatedScan_;
        std::vector<sensor_msgs::LaserScan> futureScans_;

        dynamic_gap::DynamicGapConfig cfg_;

        boost::mutex gapset_mutex;

        geometry_msgs::PoseArray currentTrajectory_;
        std::vector<float> currentTrajectoryTiming_;

        boost::circular_buffer<float> log_vel_comp;

        ros::Time tPreviousFilterUpdate_;

        geometry_msgs::TwistStamped currentRbtVel_, currentRbtAcc_;

        std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_, intermediateRbtAccs_;

        int currentModelIdx_;

        dynamic_gap::Estimator * currLeftGapPtModel_;
        dynamic_gap::Estimator * currRightGapPtModel_;
        
        geometry_msgs::TwistStamped currentPeakSplineVel_;

        // float curr_peak_velocity_x, curr_peak_velocity_y;

        int currentAgentCount_;

        std::vector<geometry_msgs::Pose> currentTrueAgentPoses_;
        std::vector<geometry_msgs::Vector3Stamped> currentTrueAgentVels_;

        std::vector<Eigen::Matrix<float, 4, 1> > currentEstimatedAgentStates_;

        int trajectoryChangeCount_;

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
        bool setGoal(const std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * update all tf transform at the beginning of every planning cycle
         * @param None, all tf received via TF
         * @return None, all registered via internal variables in TransformStamped
         */
        void updateTF();

        /**
         * Setter and Getter of Current Trajectory, this is performed in the compareToOldTraj function
         */
        void setCurrentTraj(geometry_msgs::PoseArray curr_traj) { currentTrajectory_ = curr_traj; return; }

        geometry_msgs::PoseArray getCurrentTraj() { return currentTrajectory_;}

        void setCurrentTimeArr(std::vector<float> curr_time_arr) { currentTrajectoryTiming_ = curr_time_arr; return; }
        
        std::vector<float> getCurrentTimeArr() { return currentTrajectoryTiming_; }

        int getCurrentAgentCount() { return currentAgentCount_; }

        int initialized() { return initialized_; } 

        /**
         * Generate ctrl command to a target pose
         * TODO: fix vector pop and get rid of pose_counter
         * @param pose_arr_odom
         * @return cmd_vel by assigning to pass by reference
         */
        geometry_msgs::Twist ctrlGeneration(geometry_msgs::PoseArray traj);
        
        /**
         * Take current observed gaps and perform gap conversion
         * @param None, directly taken from private variable space
         * @return gap_set, simplfied radial prioritized gaps
         */
        std::vector<dynamic_gap::Gap> gapManipulate(const std::vector<dynamic_gap::Gap> & _observed_gaps);

        /**
         * 
         *
         */
        std::vector<std::vector<float>> initialTrajGen(std::vector<dynamic_gap::Gap>& vec, std::vector<geometry_msgs::PoseArray>& res, std::vector<std::vector<float>>& res_time_traj);

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
        int pickTraj(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<float>> score);

        /**
         * Compare to the old trajectory and pick the best one
         * @param incoming trajectory
         * @return the best trajectory  
         */
        geometry_msgs::PoseArray compareToOldTraj(geometry_msgs::PoseArray incoming, 
                                                  dynamic_gap::Gap incoming_gap, 
                                                  std::vector<dynamic_gap::Gap> feasible_gaps, 
                                                  std::vector<float> time_arr,
                                                  bool curr_exec_gap_assoc, 
                                                  bool curr_exec_gap_feas);

        int getCurrentRightGapIndex();
        int getCurrentLeftGapIndex();

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
        bool recordAndCheckVel(geometry_msgs::Twist cmd_vel);
    
        std::vector<dynamic_gap::Gap> update_models(const std::vector<dynamic_gap::Gap> & _observed_gaps, 
                                                    const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied,
                                                    const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied,
                                                    const ros::Time & t_kf_update,
                                                    bool print);

        void update_model(int i, std::vector<dynamic_gap::Gap>& _observed_gaps, 
                                    const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied,
                                    const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied,
                                    const ros::Time & t_kf_update,
                                    bool print);

        void setCurrentLeftModel(dynamic_gap::Estimator * _left_model);
        void setCurrentRightModel(dynamic_gap::Estimator * _right_model);

        void setCurrentGapPeakVelocities(float _peak_velocity_x, float _peak_velocity_y);

        void printGapModels(std::vector<dynamic_gap::Gap> gaps);
        void printGapAssociations(const std::vector<dynamic_gap::Gap> & current_gaps, 
                                  const std::vector<dynamic_gap::Gap> & previous_gaps, 
                                  const std::vector<int> & association);

        std::vector<dynamic_gap::Gap> gapSetFeasibilityCheck(bool &, bool &);

        void agentOdomCB(const nav_msgs::Odometry::ConstPtr& msg);
        void visualizeComponents(std::vector<dynamic_gap::Gap> manip_gap_set);

        void getFutureScans();        

        geometry_msgs::PoseArray changeTrajectoryHelper(dynamic_gap::Gap incoming_gap, 
                                    geometry_msgs::PoseArray incoming, 
                                    std::vector<float> time_arr, 
                                    bool switching_to_empty);
    };
}