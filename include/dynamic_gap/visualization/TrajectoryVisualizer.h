#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

namespace dynamic_gap 
{
    /**
    * \brief Class for visualizing candidate trajectories through gaps
    */
    class TrajectoryVisualizer : public Visualizer
    {
        using Visualizer::Visualizer;
        
        public: 
            TrajectoryVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);

            /**
            * \brief Visualize set of candidate trajectories through gaps
            * \param trajs set of trajectories to visualize
            */
            void drawGapTrajectories(const std::vector<dynamic_gap::Trajectory> & trajs);

            /**
            * \brief Visualize pose-wise scores along candidate trajectories
            * \param trajs set of trajectories whose pose-wise scores we want to visualize
            * \param trajPoseScores pose-wise scores to visualize
            */
            void drawGapTrajectoryPoseScores(const std::vector<dynamic_gap::Trajectory> & trajs,
                                                const std::vector<std::vector<float>> & trajPoseScores);

            /**
            * \brief Visualize counter for planning loop
            * \param planningLoopIdx counter for planning loop
            */
            void drawPlanningLoopIdx(const int & planningLoopIdx);

            /**
            * \brief Visualize occurrence of a trajectory switch for planner
            * \param trajSwitchIndex trajectory switch count
            * \param chosenTraj new trajectory that planner is switching to
            */
            void drawTrajectorySwitchCount(const int & trajSwitchIndex, 
                                            const dynamic_gap::Trajectory & chosenTraj);

            /**
            * \brief Visualize global plan we are using within local planner
            * \param globalPlan global plan to visualize
            */
            void drawGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlan);

            /**
            * \brief Visualize snippet of global plan that is within current robot view
            * \param globalPlanSnippet visible snippet of global plan
            */
            void drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet);

        private: 

            ros::Publisher gapTrajectoriesPublisher; /**< Publisher for gap trajectories */
            ros::Publisher trajPoseScoresPublisher; /**< Publisher for gap trajectory pose-wise scores */
            ros::Publisher trajSwitchIdxPublisher; /**< Publisher for planner trajectory switch count */
            ros::Publisher globalPlanPublisher; /**< Publisher for global plan */
            ros::Publisher globalPlanSnippetPublisher; /**< Publisher for visible snippet of global plan */
            ros::Publisher planLoopIdxPublisher; /**< Publisher for planning loop idx */
    };
}