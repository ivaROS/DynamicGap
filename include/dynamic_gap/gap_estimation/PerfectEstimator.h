#pragma once

#include <ros/ros.h>

#include <dynamic_gap/config/DynamicGapConfig.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <dynamic_gap/gap_estimation/Estimator.h>


namespace dynamic_gap 
{
    /**
    * \brief Perfector model estimator that represents gaps attached to the static environment with a velocity of zero
    * and gaps attached to the dynamic environment with a velocity of its corresponding agent 
    */
    class PerfectEstimator : public Estimator 
    {
        public:
            PerfectEstimator();

            void initialize(const std::string & side, const int & modelID, 
                            const float & gapPtX, const float & gapPtY,
                            const ros::Time & t_update, const geometry_msgs::TwistStamped & lastRbtVel,
                            const geometry_msgs::TwistStamped & lastRbtAcc);

            void transfer(const Estimator & incomingModel);

            void transferFromPropagatedGapPoint(const Estimator & incomingModel);

            void update(const Eigen::Vector2f & measurement, 
                        const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                        const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                        const std::map<std::string, geometry_msgs::Pose> & agentPoses,
                        const std::map<std::string, geometry_msgs::Vector3Stamped> & agentVels,
                        const ros::Time & tUpdate);
 
            /**
            * \brief Getter function for relative estimator state
            * \return relative (gap-robot) estimator state
            */     
            Eigen::Vector4f getState();

            /**
            * \brief use ground truth information on agent poses and velocities to perfectly update model state

            * \return updated model state
            */
            Eigen::Vector4f updateStateFromEnv();

            float min_dist_thresh = 1.5;

        private:            
            std::map<std::string, geometry_msgs::Pose> agentPoses_; /**< poses of all agents in environment (ground truth information used for certain estimator classes */
            std::map<std::string, geometry_msgs::Vector3Stamped> agentVels_; /**< velocities of all agents in environment (ground truth information used for certain estimator classes) */
    };
}