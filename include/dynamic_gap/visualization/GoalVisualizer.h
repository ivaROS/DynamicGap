
#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

namespace dynamic_gap {
    class GoalVisualizer : public Visualizer
    {
        public: 
            using Visualizer::Visualizer;
            GoalVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void localGoal(geometry_msgs::PoseStamped);
            void drawGapGoal(visualization_msgs::MarkerArray&, dynamic_gap::Gap, bool initial);
            void drawGapGoals(std::vector<dynamic_gap::Gap>);
        private: 
            ros::Publisher goal_pub;
            ros::Publisher gapwp_pub;
            std_msgs::ColorRGBA gapwp_color;
            std_msgs::ColorRGBA terminal_gapwp_color;
            std_msgs::ColorRGBA localGoal_color;
            double prev_num_goals;
    };
}