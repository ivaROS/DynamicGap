#pragma once

#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <ros/ros.h>

struct dwa_Trajectory
{
    // --- Core geometry ---
    std::vector<Eigen::Vector2f> positions;   // sampled x,y positions (robot frame)s
    std::vector<float> yaws;                  // heading angles at each step
    std::vector<float> times;                 // timestamps (s)

    // --- Commanded velocities used to generate rollout ---
    float v_cmd = 0.0f;  // linear velocity
    float w_cmd = 0.0f;  // angular velocity
    float H_left = 0.0f;  // h for the CBF

    // --- Cost terms (for later DWA scoring) ---
    float obs_cost = 0.0f;
    float goal_cost = 0.0f;
    float speed_cost = 0.0f;
    float path_cost = 0.0f;
    float total_cost = 0.0f;
    float totalTrajCost = 0.0f; 
    geometry_msgs::PoseArray pose_array;
     std::vector<float> PoseCosts;// IMPORTANT NOTE: dwa_PoseCosts is only the distance-related cost not other costs like path cost 
     std::vector<float> PathCosts; 
     std::vector<float> RelVelPoseCosts; //relative velocity cost applied to each pose 
     float TerminalPoseCost= 0.0f; 

    // --- Convert to ROS PoseArray for visualization ---
    geometry_msgs::PoseArray toPoseArray(const std::string& frame_id) const
    {
        geometry_msgs::PoseArray msg;
        msg.header.frame_id = frame_id;
        msg.header.stamp = ros::Time::now();

        for (size_t i = 0; i < positions.size(); ++i)
        {
            geometry_msgs::Pose pose;
            pose.position.x = positions[i].x();
            pose.position.y = positions[i].y();
            pose.position.z = 0.0;
            pose.orientation = tf::createQuaternionMsgFromYaw(yaws[i]);
            msg.poses.push_back(pose);
        }
        return msg;
    }
};
