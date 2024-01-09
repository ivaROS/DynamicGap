#pragma once
#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <vector>
#include <map>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>

#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap
{
    class Visualizer {
        public: 
            Visualizer() {};
            ~Visualizer() {};

            Visualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            // Visualizer& operator=(Visualizer other) {cfg_ = other.cfg_; return *this; };
            // Visualizer(const Visualizer &t) {cfg_ = t.cfg_;};

        protected:
            const DynamicGapConfig* cfg_;
    };
}