
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
        // using Visualizer::Visualizer;

        public: 
            GoalVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg);

            /**
            * \brief Visualize set of gap goals
            * \param gaps set of gaps whose goals we want to visualize
            */
            void drawGapTubeGoals(const std::vector<GapTube *> & gapTubes);

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
            // void drawGapTubeGoals(visualization_msgs::MarkerArray & gapGoalsMarkerArray, GapTube * gapTube);

            void drawGapGoalPosition(visualization_msgs::Marker & goalMarker, Gap * gap, int & id);
            void drawGapGoalVelocity(visualization_msgs::Marker & goalMarker, Gap * gap, int & id);

            std_msgs::ColorRGBA gapGoalsColor; /**< Color to visualize gap goals with */
            
            std_msgs::ColorRGBA terminalGapGoalsColor; /**< Color to visualize terminal gap goals with */
            std_msgs::ColorRGBA globalPathLocalWaypointColor; /**< Color to visualize global path local waypoint with */
            std_msgs::ColorRGBA globalGoalColor; /**< Color to visualize global goal with */

            ros::Publisher globalPathLocalWaypointPublisher; /**< Publisher for global path local waypoint */
            ros::Publisher gapGoalPositionsPublisher; /**< Publisher for gap goal positions */
            ros::Publisher gapGoalVelocitiesPublisher; /**< Publisher for gap goal velocities */
            ros::Publisher globalGoalPublisher; /**< Publisher for global goal */
    };
}