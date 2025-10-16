#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <vector>
#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap
{
    class BezierVisualizer
    {
    public:
        BezierVisualizer() = default;
        BezierVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg)
        {
            cfg_ = &cfg;
            p2Publisher_ = nh.advertise<visualization_msgs::Marker>("bezier_p2", 1);
            curvePublisher_ = nh.advertise<visualization_msgs::Marker>("bezier_curve", 1);
        }

        void drawP2(const Eigen::Vector2f& p2)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = cfg_->sensor_frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "bezier_debug";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = p2.x();
            marker.pose.position.y = p2.y();
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            p2Publisher_.publish(marker);
        }

        void drawCurve(const std::vector<Eigen::Vector2f>& curve)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = cfg_->sensor_frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "bezier_debug";
            marker.id = 1;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.02;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            for (const auto& pt : curve)
            {
                geometry_msgs::Point p;
                p.x = pt.x();
                p.y = pt.y();
                p.z = 0.0;
                marker.points.push_back(p);
            }
            curvePublisher_.publish(marker);
        }

    private:
        const DynamicGapConfig* cfg_ = nullptr;
        ros::Publisher p2Publisher_;
        ros::Publisher curvePublisher_;
    };
}
