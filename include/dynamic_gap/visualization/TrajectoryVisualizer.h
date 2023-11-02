#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

namespace dynamic_gap 
{
    class TrajectoryVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 
            TrajectoryVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);

            void drawTrajectorySwitchCount(int trajSwitchIndex, const geometry_msgs::PoseArray & chosenTraj);
            void drawGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & plan);

            void drawGapTrajectories(const std::vector<geometry_msgs::PoseArray> & trajs);
            void drawGapTrajectoryPoseScores(const std::vector<geometry_msgs::PoseArray> & trajs, 
                             const std::vector<std::vector<float>> & trajPoseScores);
            void drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::PoseStamped> & traj);

        private: 
            ros::Publisher trajSwitchIdxPublisher;
            ros::Publisher globalPlanPublisher;
            ros::Publisher gapTrajectoriesPublisher;
            ros::Publisher trajPoseScoresPublisher;
            
            ros::Publisher globalPlanSnippetPublisher;
            // float prev_num_trajs;           
    };
}