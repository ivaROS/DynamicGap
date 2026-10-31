#pragma once

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>

#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Trajectory.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/trajectory_generation/TrajectorySynthesisMethods.h>

#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>


namespace dynamic_gap 
{
    /**
    * \brief class responsible for generating local collision-free trajectories through gap
    */    
    class GapTrajectoryGenerator
    {
        public:
            GapTrajectoryGenerator(const DynamicGapConfig& cfg) { cfg_ = &cfg; };

            Trajectory generateGoToGoalTrajectoryV2(Gap * selectedGap, 
                                                    const geometry_msgs::PoseStamped & currPose, 
                                                    // const geometry_msgs::TwistStamped & currVel,
                                                    const geometry_msgs::PoseStamped & globalGoalRobotFrame);

            /**
            * \brief generate local collision-free trajectory through gap
            * \param selectedGap gap through which trajectory will be generated
            * \param currPose current robot pose
            * \param currVel current robot velocity
            * \param globalGoalRobotFrame global goal in the robot frame
            * \param runGoToGoal boolean for if go to goal trajectory method should be run
            * \return trajectory through gap
            */
            Trajectory generateTrajectoryV2(Gap * selectedGap, 
                                            const geometry_msgs::PoseStamped & currPose, 
                                            // const geometry_msgs::TwistStamped & currVel,
                                            const geometry_msgs::PoseStamped & globalGoalRobotFrame);

            // /**
            // * \brief generate local collision-free trajectory through gap
            // * \param selectedGap gap through which trajectory will be generated
            // * \param currPose current robot pose
            // * \param currVel current robot velocity
            // * \param globalGoalRobotFrame global goal in the robot frame
            // * \param runGoToGoal boolean for if go to goal trajectory method should be run
            // * \return trajectory through gap
            // */
            // Trajectory generateTrajectory(Gap * selectedGap, 
            //                                 const geometry_msgs::PoseStamped & currPose, 
            //                                 const geometry_msgs::TwistStamped & currVel,
            //                                 const geometry_msgs::PoseStamped & globalGoalRobotFrame,
            //                                 const bool & runGoToGoal);

            // /**
            // * \brief generate trajectory for idling in place
            // * \param rbtPoseInOdomFrame robot pose in odometry frame
            // * \return trajectory through gap
            // */
            // Trajectory generateIdlingTrajectory(const geometry_msgs::PoseStamped & rbtPoseInOdomFrame);

            /**
            * \brief generate trajectory for idling in place
            * \param rbtPoseInOdomFrame robot pose in odometry frame
            * \return trajectory through gap
            */
            Trajectory generateIdlingTrajectoryV2(Gap * gap,
                                                    const geometry_msgs::PoseStamped & currPose,
                                                    const Trajectory & runningTraj);
                                                    // const geometry_msgs::PoseStamped & rbtPoseInOdomFrame);

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
            * \param prune flag for if you want to prune poses off of trajectory 
            * \return post-processed trajectory
            */                       
            Trajectory processTrajectory(const Trajectory & traj);
                                            // const geometry_msgs::PoseStamped & currPose, 
                                            // const geometry_msgs::TwistStamped & currVel,                
                                            // const bool & prune);

            Trajectory pruneTrajectory(const Trajectory & traj);

            // Trajectory processIdlingTrajectory(const Trajectory & traj,
            //                                     const Trajectory & runningTraj);

        private: 

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
    };
}