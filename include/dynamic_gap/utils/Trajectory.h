#pragma once

#include <geometry_msgs/PoseArray.h>
#include <vector>

namespace dynamic_gap
{
    class Trajectory
    {
        public:
            Trajectory()
            {
                path_ = geometry_msgs::PoseArray();
            }

            Trajectory(const geometry_msgs::PoseArray & path, const std::vector<float> & pathTiming)
            {
                path_ = path;
                pathTiming_ = pathTiming;
            }

            void setPath(const geometry_msgs::PoseArray & path) { path_ = path; }
            void setPathOdomFrame(const geometry_msgs::PoseArray & pathOdomFrame) { pathOdomFrame_ = pathOdomFrame; }
            void setPathTiming(const std::vector<float> & pathTiming) { pathTiming_ = pathTiming; }

            geometry_msgs::PoseArray getPath() const { return path_; }
            geometry_msgs::PoseArray getPathOdomFrame() const { return pathOdomFrame_; }
            std::vector<float> getPathTiming() const { return pathTiming_; }

        private:
            geometry_msgs::PoseArray path_; // in robot frame
            geometry_msgs::PoseArray pathOdomFrame_; // in odom frame
            std::vector<float> pathTiming_;
    };
}