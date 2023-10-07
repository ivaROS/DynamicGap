#pragma once
#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/gap.h>
#include <dynamic_gap/dynamicgap_config.h>
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

#include <dynamic_gap/dynamicgap_config.h>

namespace dynamic_gap
{
    class Visualizer {
        public: 
            Visualizer() {};
            ~Visualizer() {};

            Visualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            Visualizer& operator=(Visualizer other) {cfg_ = other.cfg_; return *this; };
            Visualizer(const Visualizer &t) {cfg_ = t.cfg_;};

        protected:
            const DynamicGapConfig* cfg_;
    };
}