#pragma once

#include <geometry_msgs/PoseArray.h>
#include <vector>

namespace dynamic_gap
{
    /**
    * \brief Wrapper class for candidate local trajectories that planner produces
    */
    class Trajectory
    {
        public:
            Trajectory()
            {
                pathRbtFrame_ = geometry_msgs::PoseArray();
            }

            Trajectory(const geometry_msgs::PoseArray & pathRbtFrame, 
                       const std::vector<float> & pathTiming)
            {
                pathRbtFrame_ = pathRbtFrame;
                pathTiming_ = pathTiming;
            }

            /**
            * \brief Setter for trajectory path in robot frame
            * \param pathRbtFrame trajectory path in robot frame
            */
            void setPathRbtFrame(const geometry_msgs::PoseArray & pathRbtFrame) { pathRbtFrame_ = pathRbtFrame; }
            
            /**
            * \brief Getter for trajectory path in robot frame
            * \return trajectory path in robot frame
            */
            geometry_msgs::PoseArray getPathRbtFrame() const { return pathRbtFrame_; }

            /**
            * \brief Setter for trajectory path in odom frame
            * \param pathOdomFrame trajectory path in odom frame
            */
            void setPathOdomFrame(const geometry_msgs::PoseArray & pathOdomFrame) { pathOdomFrame_ = pathOdomFrame; }
            
            /**
            * \brief Getter for trajectory path in odom frame
            * \return trajectory path in odom frame
            */            
            geometry_msgs::PoseArray getPathOdomFrame() const { return pathOdomFrame_; }

            /**
            * \brief Setter for trajectory path timing
            * \param pathTiming trajectory path timing
            */
            void setPathTiming(const std::vector<float> & pathTiming) { pathTiming_ = pathTiming; }

            /**
            * \brief Getter for trajectory path timing
            * \return trajectory path timing
            */
            std::vector<float> getPathTiming() const { return pathTiming_; }

        private:
            geometry_msgs::PoseArray pathRbtFrame_; /**< trajectory path in robot frame */
            geometry_msgs::PoseArray pathOdomFrame_; /**< trajectory path in odom frame */
            std::vector<float> pathTiming_; /**< trajectory path timing */
    };
}