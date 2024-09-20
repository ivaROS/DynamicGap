#pragma once

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>

// #include <traj_generator.h>
// #include <turtlebot_trajectory_generator/near_identity.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Trajectory.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/trajectory_generation/TrajectorySynthesisMethods.h>

// #include <vector>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// #include "tf/transform_datatypes.h"
// #include <sensor_msgs/LaserScan.h>
#include "OsqpEigen/OsqpEigen.h"

namespace dynamic_gap 
{
    /**
    * \brief class responsible for generating local collision-free trajectories through gap
    */    
    class GapTrajectoryGenerator
    {
        public:
            GapTrajectoryGenerator(const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; };
            
            /**
            * \brief generate local collision-free trajectory through gap
            * \param selectedGap gap through which trajectory will be generated
            * \param currPose current robot pose
            * \param currVel current robot velocity
            * \param runGoToGoal boolean for if go to goal trajectory method should be run
            * \return trajectory through gap
            */
            dynamic_gap::Trajectory generateTrajectory(dynamic_gap::Gap * selectedGap, 
                                                        const geometry_msgs::PoseStamped & currPose, 
                                                        const geometry_msgs::TwistStamped & currVel,
                                                        const geometry_msgs::PoseStamped & globalGoalRobotFrame,
                                                        const bool & runGoToGoal);

            dynamic_gap::Trajectory generateIdlingTrajectory(const geometry_msgs::PoseStamped & rbtPoseInOdomFrame);

            /**
            * \brief helper function for transforming trajectory from source frame to destination frame
            * \param path path that is to be transformed
            * \param transform transform to apply to path
            * \return transformed path
            */
            geometry_msgs::PoseArray transformPath(const geometry_msgs::PoseArray & path,
                                                    const geometry_msgs::TransformStamped & transform);

            /**
            * \brief helper function for post-processing the resulting trajectory which includes
            * removing intermediate poses that are sufficiently close together and
            * setting intermediate pose orientations to follow the path
            * \param traj incoming trajectory to be processed
            * \return post-processed trajectory
            */                       
            dynamic_gap::Trajectory processTrajectory(const dynamic_gap::Trajectory & traj,
                                                        const bool & prune);

        private: 

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
    };
}