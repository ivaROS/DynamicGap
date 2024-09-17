
#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

namespace dynamic_gap 
{
    /**
    * \brief Class for visualizing goal-related objects
    */
    class GoalVisualizer : public Visualizer
    {
        using Visualizer::Visualizer;

        public: 
            GoalVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);

            /**
            * \brief Visualize set of gap goals
            * \param gaps set of gaps whose goals we want to visualize
            */
            void drawGapGoals(const std::vector<dynamic_gap::Gap *> & gaps);

            /**
            * \brief Visualize global path local waypoint
            * \param globalPathLocalWaypoint global path local waypoint to visualize
            */
            void drawGlobalPathLocalWaypoint(const geometry_msgs::PoseStamped & globalPathLocalWaypoint);
        
            void drawGlobalGoal(const geometry_msgs::PoseStamped & globalGoalOdomFrame);

        private: 
            // /**
            // * \brief Visualize a single gap goal 
            // * \param gapGoalsMarkerArray marker array to add gap goal marker to
            // * \param gap gap whose goal we want to visualize
            // * \param initial boolean for if we want to visualize the gap's initial goal or terminal goal
            // */        
            // void drawGapGoal(visualization_msgs::MarkerArray & gapGoalsMarkerArray, 
            //                  dynamic_gap::Gap * gap, 
            //                  const bool & initial);

            void drawGapGoals(visualization_msgs::MarkerArray & marker, 
                                const std::vector<dynamic_gap::Gap *> & gaps, 
                                const bool & initial);

            std_msgs::ColorRGBA gapGoalsColor; /**< Color to visualize gap goals with */
            std_msgs::ColorRGBA terminalGapGoalsColor; /**< Color to visualize terminal gap goals with */
            std_msgs::ColorRGBA globalPathLocalWaypointColor; /**< Color to visualize global path local waypoint with */
            std_msgs::ColorRGBA globalGoalColor; /**< Color to visualize global goal with */

            ros::Publisher globalPathLocalWaypointPublisher; /**< Publisher for global path local waypoint */
            ros::Publisher gapGoalsPublisher; /**< Publisher for gap goals */
            ros::Publisher gapGoalsFigPublisher;
            ros::Publisher globalGoalPublisher; /**< Publisher for global goal */
    };
}