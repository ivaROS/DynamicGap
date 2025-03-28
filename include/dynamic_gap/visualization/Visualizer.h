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

            Visualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg);

        protected:
            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */

            void clearMarkerPublisher(const ros::Publisher & publisher)
            {
                visualization_msgs::Marker clearMarker;
                clearMarker.id = 0;
                clearMarker.ns =  "clear";
                clearMarker.action = visualization_msgs::Marker::DELETEALL;
                publisher.publish(clearMarker);
            }

            void clearMarkerArrayPublisher(const ros::Publisher & publisher)
            {
                visualization_msgs::MarkerArray clearMarkerArray;
                visualization_msgs::Marker clearMarker;
                clearMarker.id = 0;
                clearMarker.ns =  "clear";
                clearMarker.action = visualization_msgs::Marker::DELETEALL;
                clearMarkerArray.markers.push_back(clearMarker);
                publisher.publish(clearMarkerArray);
            }
    };
}