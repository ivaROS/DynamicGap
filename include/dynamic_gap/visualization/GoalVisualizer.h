
#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

#include <geometry_msgs/PoseStamped.h>

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
        
            /**
            * \brief Visualize global goal
            * \param globalGoalOdomFrame global goal in odom frame
            */
            void drawGlobalGoal(const geometry_msgs::PoseStamped & globalGoalOdomFrame);

        private: 
            /**
            * \brief Visualize set of gap goals
            * \param marker marker to visualize
            * \param gaps set of gaps to visualize
            * \param initial flag for visualizing initial or terminal gap goals
            */
            void drawGapGoals(visualization_msgs::Marker & marker, 
                                const std::vector<dynamic_gap::Gap *> & gaps, 
                                const bool & initial);

            std_msgs::ColorRGBA gapGoalsColor; /**< Color to visualize gap goals with */
            std_msgs::ColorRGBA terminalGapGoalsColor; /**< Color to visualize terminal gap goals with */
            std_msgs::ColorRGBA globalPathLocalWaypointColor; /**< Color to visualize global path local waypoint with */
            std_msgs::ColorRGBA globalGoalColor; /**< Color to visualize global goal with */

            ros::Publisher globalPathLocalWaypointPublisher; /**< Publisher for global path local waypoint */
            ros::Publisher gapGoalsPublisher; /**< Publisher for gap goals */
            ros::Publisher globalGoalPublisher; /**< Publisher for global goal */
    };
}