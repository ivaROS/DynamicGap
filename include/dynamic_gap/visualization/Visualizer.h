#pragma once
#include <ros/ros.h>
#include <math.h>

#include <vector>
#include <numeric>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Trajectory.h>
#include <dynamic_gap/utils/Utils.h>

namespace dynamic_gap
{
    class Visualizer 
    {
        public: 
            Visualizer() {};

            Visualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);

        protected:
            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
    };
}