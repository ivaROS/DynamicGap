#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

namespace dynamic_gap 
{
    class TrajectoryVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 
            TrajectoryVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void drawEntireGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & plan);
            // void trajScore(geometry_msgs::PoseArray, std::vector<double>);
            void pubAllTraj(std::vector<geometry_msgs::PoseArray> prr);
            void pubAllScore(std::vector<geometry_msgs::PoseArray>, std::vector<std::vector<double>>);
            void drawRelevantGlobalPlanSnippet(std::vector<geometry_msgs::PoseStamped> traj);
            void drawTrajectorySwitchCount(int switch_index, geometry_msgs::PoseArray switch_traj);

        private: 
            ros::Publisher entire_global_plan_pub;
            ros::Publisher trajectory_score;
            ros::Publisher all_traj_viz;
            ros::Publisher relevant_global_plan_snippet_pub;
            ros::Publisher trajectory_switch_pub;
            double prev_num_trajs;           
    };
}