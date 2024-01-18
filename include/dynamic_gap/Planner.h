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
#include <dynamic_gap/utils/Trajectory.h>
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
    /**
    * Class responsible for the core planning loop of dynamic gap.
    */
    class Planner
    {

        public:
            Planner();

            ~Planner();

            /**
            * \brief initialize Planner class
            * 
            * \param nh ROS node handle 
            * \return initialization success / failure
            */
            bool initialize(const ros::NodeHandle& nh);

            /**
            * \brief Check if global goal has been reached by robot
            * \return boolean for if global goal has been reached or not
            */
            bool isGoalReached();

            /**
            * \brief Call back function to robot laser scan
            * \param scan incoming laser scan msg
            */
            void laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            /**
            * \brief Function for updating the gap models
            * \param gaps set of gaps whose models we are updating
            * \param intermediateRbtVels intermediate robot velocity values between last model update and current model update
            * \param intermediateRbtAccs intermediate robot acceleration values between last model update and current model update
            * \param tCurrentFilterUpdate time step for current estimator update
            */
            void updateModels(std::vector<dynamic_gap::Gap *> & gaps, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                                const ros::Time & tCurrentFilterUpdate);

            /**
            * \brief Function for updating a single gap's models
            * \param idx index of gap whose models we must update
            * \param gaps set of gaps whose models we are updating
            * \param intermediateRbtVels intermediate robot velocity values between last model update and current model update
            * \param intermediateRbtAccs intermediate robot acceleration values between last model update and current model update
            * \param tCurrentFilterUpdate time step for current estimator update
            */
            void updateModel(const int & idx, 
                                std::vector<dynamic_gap::Gap *> & gaps, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                                const ros::Time & tCurrentFilterUpdate);

            /**
            * \brief Joint call back function for robot pose (position + velocity) and robot acceleration messages
            * \param rbtOdomMsg incoming robot odometry message
            * \param rbtAccelMsg incoming robot acceleration message
            */
            void jointPoseAccCB(const nav_msgs::Odometry::ConstPtr & rbtOdomMsg, 
                                const geometry_msgs::TwistStamped::ConstPtr & rbtAccelMsg);

            /**
            * \brief Call back function for other agent odometry messages
            * \param agentOdomMsg incoming agent odometry message
            */
            void agentOdomCB(const nav_msgs::Odometry::ConstPtr& agentOdomMsg);

            /**
            * \brief Interface function for receiving incoming global plan and updating
            * the global plan local waypoint
            * \param globalPlanMapFrame incoming global plan in map frame
            * \return boolean type on whether planner successfully registered goal
            */
            bool setGoal(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame);

            /**
            * \brief Function for updating all tf transform at the beginning of every planning cycle
            */
            void updateTF();

            /**
            * \brief Function for all member objects updating their current egocircles
            */
            void updateEgoCircle();

            /**
            * \brief Function for performing gap manipulation steps
            * \return manipulated set of gaps
            */
            std::vector<dynamic_gap::Gap *> gapManipulate(const std::vector<dynamic_gap::Gap *> & feasibleGaps);

            /**
            * \brief Function for generating candidate trajectories through the current set of gaps
            * \param gaps incoming set of gaps through which we want to generate trajectories
            * \param generatedTrajs set of generated trajectories
            * \return Vector of pose-wise scores for the generated trajectories
            */
            std::vector<std::vector<float>> generateGapTrajs(std::vector<dynamic_gap::Gap *> & gaps, 
                                                             std::vector<dynamic_gap::Trajectory> & generatedTrajs);

            /**
            * \brief Function for selecting the best trajectory out of the set of recently generated trajectories
            * \param trajs set of recently generated trajectories
            * \param pathPoseScores vector of pose-wise scores for the generated trajectories
            * \return index of the highest score trajectory
            */
            int pickTraj(const std::vector<dynamic_gap::Trajectory> & trajs, 
                        const std::vector<std::vector<float>> & pathPoseScores);

            /**
            * \brief Helper function for switching to a new trajectory
            * \param incomingGap incoming gap to switch to
            * \param incomingTraj incoming trajectory to switch to
            * \param switchToIncoming boolean for if planner is to switch to the incoming trajectory
            * \return trajectory that planner will switch to
            */
            dynamic_gap::Trajectory changeTrajectoryHelper(dynamic_gap::Gap * incomingGap, 
                                                            const dynamic_gap::Trajectory & incomingTraj, 
                                                            const bool & switchToIncoming);

            /**
            * \brief Compare incoming highest scoring trajectory to the trajectory that the
            * robot is currently following to determine if robot should switch to the incoming trajectory
            * \param incomingGap incoming gap to switch to
            * \param incomingTraj incoming trajectory to switch to
            * \param feasibleGaps set of feasible gaps which we are manipulating
            * \param isIncomingGapFeasible boolean for if the incoming gap is feasible 
            * \return the trajectory that the robot will track
            */
            dynamic_gap::Trajectory compareToCurrentTraj(dynamic_gap::Gap * incomingGap, 
                                                            const dynamic_gap::Trajectory & incomingTraj,                                                        
                                                            const std::vector<dynamic_gap::Gap *> & feasibleGaps, 
                                                            const bool & isIncomingGapFeasible);

           /**
            * Gets the current position along the currently executing Trajectory
            */
            int egoTrajPosition(const geometry_msgs::PoseArray & curr);


            /**
            * Setter and Getter of Current Gap, this is performed in the compareToCurrentTraj function
            */
            void setCurrentGap(dynamic_gap::Gap * currentGap) { delete currentGap_; currentGap_ = currentGap; return; }

            dynamic_gap::Gap * getCurrentGap() { return currentGap_; }

            /**
            * Setter and Getter of Current Trajectory, this is performed in the compareToCurrentTraj function
            */

            void setCurrentTraj(const dynamic_gap::Trajectory & currentTraj) { currentTraj_ = currentTraj; return; }

            dynamic_gap::Trajectory getCurrentTraj() { return currentTraj_; }


            // void setCurrentPathTiming(const std::vector<float> & currentPathTiming) { currentPathTiming_ = currentPathTiming; return; }
            
            // std::vector<float> getCurrentPathTiming() { return currentPathTiming_; }

            int getCurrentAgentCount() { return currentAgentCount_; }

            int initialized() { return initialized_; } 

            /**
            * Generate ctrl command to a target pose
            * TODO: fix vector pop and get rid of pose_counter
            * @param pose_arr_odom
            * @return command velocity by assigning to pass by reference
            */
            geometry_msgs::Twist ctrlGeneration(const geometry_msgs::PoseArray & localTrajectory);

            int getCurrentRightGapPtModelID();
            int getCurrentLeftGapPtModelID();

            /**
            * Conglomeration of getting a plan Trajectory
            * @return the trajectory (in odom frame)
            */
            dynamic_gap::Trajectory runPlanningLoop();        

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

            void setCurrentLeftModel(dynamic_gap::Estimator * left_model);
            void setCurrentRightModel(dynamic_gap::Estimator * right_model);

            void setCurrentGapPeakVelocities(const float & peakVelX, const float & peakVelY);

            void printGapModels(const std::vector<dynamic_gap::Gap *> & gaps);
            void printGapAssociations(const std::vector<dynamic_gap::Gap *> & currentGaps, 
                                    const std::vector<dynamic_gap::Gap *> & previousGaps, 
                                    const std::vector<int> & association);

            std::vector<dynamic_gap::Gap *> gapSetFeasibilityCheck(bool & isCurrentGapFeasible); // bool & isCurrentGapAssociated, 
                                                                
            void visualizeComponents(const std::vector<dynamic_gap::Gap *> & manip_gap_set);

            void getFutureScans();        

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

        ros::NodeHandle nh_;
        ros::Publisher currentTrajectoryPublisher_;
        ros::Publisher staticScanPublisher_;

        std::vector<int> rawAssocation_, simpAssociation_;
        std::vector<std::vector<float>> rawDistMatrix_, simpDistMatrix_;

        // Goals and stuff
        geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame_;  
        geometry_msgs::PoseStamped globalGoalOdomFrame_;
        geometry_msgs::PoseStamped globalGoalRobotFrame_;

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
        dynamic_gap::Trajectory currentTraj_;
        // std::vector<float> currentPathTiming_;

        boost::circular_buffer<float> cmdVelBuffer_;

        ros::Time tPreviousFilterUpdate_;

        geometry_msgs::TwistStamped currentRbtVel_, currentRbtAcc_;

        std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_, intermediateRbtAccs_;

        int currentModelIdx_;

        dynamic_gap::Estimator * currLeftGapPtModel_ = NULL;
        dynamic_gap::Estimator * currRightGapPtModel_ = NULL;
        
        geometry_msgs::TwistStamped currentPeakSplineVel_;

        int currentAgentCount_;

        std::vector<geometry_msgs::Pose> currentTrueAgentPoses_;
        std::vector<geometry_msgs::Vector3Stamped> currentTrueAgentVels_;

        std::vector<Eigen::Vector4f> currentEstimatedAgentStates_;

        int trajectoryChangeCount_;

        bool readyToPlan = false;
    };
}