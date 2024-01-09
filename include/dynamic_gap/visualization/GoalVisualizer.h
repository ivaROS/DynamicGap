
#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

namespace dynamic_gap 
{
    class GoalVisualizer : public Visualizer
    {
        public: 
            using Visualizer::Visualizer;

            GoalVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void drawGlobalPathLocalWaypoint(const geometry_msgs::PoseStamped & globalPathLocalWaypoint);
            void drawGapGoals(const std::vector<dynamic_gap::Gap *> & gaps);
        
        private: 
            void drawGapGoal(visualization_msgs::MarkerArray&, dynamic_gap::Gap *, bool initial);

            ros::Publisher globalPathLocalWaypointPublisher;
            ros::Publisher gapGoalsPublisher;
            std_msgs::ColorRGBA gapGoalsColor;
            std_msgs::ColorRGBA terminalGapGoalsColor;
            std_msgs::ColorRGBA globalPathLocalWaypointColor;
            // float prev_num_goals;
    };
}