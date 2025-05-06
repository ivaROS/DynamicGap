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
#include <dynamic_gap/utils/GapTube.h>
#include <dynamic_gap/utils/Trajectory.h>
#include <dynamic_gap/utils/Utils.h>

namespace dynamic_gap
{
    class Visualizer 
    {
        public: 
            Visualizer() { initColors();};

            Visualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg) { initColors();}

        protected:
            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */

            std::vector<std_msgs::ColorRGBA> visionColors_;

            void initColors()
            {
                // vision colors
                std_msgs::ColorRGBA visionColor;

                visionColor.r = 24.0/255.0; visionColor.g = 217/255.0; visionColor.b = 242.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);
                visionColor.r = 204.0/255.0; visionColor.g = 18/255.0; visionColor.b = 255.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);
                visionColor.r = 25.0/255.0; visionColor.g = 83.0/255.0; visionColor.b = 255.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);
                visionColor.r = 220.0/255.0; visionColor.g = 138.0/255.0; visionColor.b = 255.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);       
                visionColor.r = 25.0/255.0; visionColor.g = 156.0/255.0; visionColor.b = 255.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);         
                visionColor.r = 39.0/255.0; visionColor.g = 24.0/255.0; visionColor.b = 242.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);          
                visionColor.r = 24.0/255.0; visionColor.g = 242.0/255.0; visionColor.b = 196.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);                     
                visionColor.r = 7.0/255.0; visionColor.g = 93.0/255.0; visionColor.b = 94.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);     
                visionColor.r = 105.0/255.0; visionColor.g = 183.0/255.0; visionColor.b = 242.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);          
                visionColor.r = 44.0/255.0; visionColor.g = 115.0/255.0; visionColor.b = 133.0/255.0; visionColor.a = 1.0;
                visionColors_.push_back(visionColor);          
            }

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