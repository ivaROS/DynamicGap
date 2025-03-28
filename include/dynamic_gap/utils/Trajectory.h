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

                if (pathRbtFrame.poses.empty())
                {
                    ROS_WARN_STREAM("Trajectory path in robot frame is empty");
                }

                if (pathTiming.empty())
                {
                    ROS_WARN_STREAM("Trajectory path timing is empty");
                }

                if (pathRbtFrame.poses.size() != pathTiming.size())
                {
                    ROS_WARN_STREAM("Trajectory path and timing size mismatch");
                }

                if (pathRbtFrame_.header.frame_id.empty())
                {
                    ROS_WARN_STREAM("Trajectory path frame id is empty");
                }
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

            void appendTraj(const Trajectory & otherTraj)
            {
                ROS_INFO_STREAM_NAMED("Trajectory", "Appending trajectory");

                geometry_msgs::PoseArray otherPathRbtFrame = otherTraj.getPathRbtFrame();
                geometry_msgs::PoseArray otherPathOdomFrame = otherTraj.getPathOdomFrame();
                std::vector<float> otherPathTiming = otherTraj.getPathTiming();

                ROS_INFO_STREAM_NAMED("Trajectory", "other path rbt frame");
                for (int i = 0; i < otherPathRbtFrame.poses.size(); i++)
                {
                    ROS_INFO_STREAM_NAMED("Trajectory", "time: " << otherPathTiming[i] << ", pose " << i << ": " << otherPathRbtFrame.poses[i].position.x << ", " << otherPathRbtFrame.poses[i].position.y);
                }

                if (otherPathRbtFrame.poses.size() > 1)
                {
                    // start one in to not repeat last of previous trajectory
                    pathRbtFrame_.poses.insert(pathRbtFrame_.poses.end(), otherPathRbtFrame.poses.begin() + 1, otherPathRbtFrame.poses.end());
                    pathOdomFrame_.poses.insert(pathOdomFrame_.poses.end(), otherPathOdomFrame.poses.begin() + 1, otherPathOdomFrame.poses.end());
                    
                    // append path timing
                    float lastTime = pathTiming_.back();
                    for (int i = 0; i < otherPathTiming.size(); i++)
                        otherPathTiming[i] += lastTime;

                    pathTiming_.insert(pathTiming_.end(), otherPathTiming.begin() + 1, otherPathTiming.end());
                }
            }

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