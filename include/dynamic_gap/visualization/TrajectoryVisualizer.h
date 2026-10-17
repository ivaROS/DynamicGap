#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <set>
#include <utility>

namespace dynamic_gap 
{
    struct MppiGapVisualization
    {
        int gap_index = 0;
        int left_id = -1;
        int right_id = -1;
        std::vector<geometry_msgs::PoseArray> samples;
        std::vector<bool> sample_feasible;
        geometry_msgs::PoseArray nominal;
        float cost = std::numeric_limits<float>::infinity();
        std::size_t total_samples = 0;
        std::size_t feasible_samples = 0;
        std::size_t progress_samples = 0;
    };

    using MppiMarkerKeys = std::set<std::pair<std::string, int>>;

    // Build a complete planning frame without ROS publisher side effects. The key
    // set tracks only this display's markers, allowing scoped stale-marker removal.
    visualization_msgs::MarkerArray buildMppiMarkers(
        const std::vector<MppiGapVisualization>& gaps,
        const std::vector<std_msgs::ColorRGBA>& gap_colors,
        const ros::Time& stamp,
        MppiMarkerKeys& previous_keys,
        std::size_t max_samples_per_gap = 0,
        std::size_t point_stride = 1);

    /**
    * \brief Class for visualizing candidate trajectories through gaps
    */
    class TrajectoryVisualizer : public Visualizer
    {
        // using Visualizer::Visualizer;
        
        public: 
            TrajectoryVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg);

            void drawCurrentTrajectory(const Trajectory & traj);

            /**
            * \brief Visualize set of candidate trajectories through gaps
            * \param trajs set of trajectories to visualize
            */
            void drawGapTrajectories(const std::vector<Trajectory> & trajs);

            void drawGapTubeTrajectories(const std::vector<Trajectory> & trajs);

            // Publish every gap together. Zero max_samples_per_gap draws all samples.
            void drawMppiTrajectories(const std::vector<MppiGapVisualization>& gaps,
                                      std::size_t max_samples_per_gap = 0,
                                      std::size_t point_stride = 1);

            void drawUngapTrajectories(const std::vector<Trajectory> & trajs);

            /**
            * \brief Visualize pose-wise scores along candidate trajectories
            * \param trajs set of trajectories whose pose-wise scores we want to visualize
            * \param trajPoseScores pose-wise scores to visualize
            */
            void drawGapTrajectoryPoseScores(const std::vector<Trajectory> & trajs,
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
                                            const Trajectory & chosenTraj);

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

            ros::Publisher currentTrajectoryPublisher_; /**< ROS publisher for currently tracked trajectory */
            ros::Publisher gapTrajectoriesPublisher; /**< Publisher for gap trajectories */
            ros::Publisher gapTubeTrajectoriesPublisher; /**< Publisher for gap tube trajectories */
            ros::Publisher mppiTrajectoriesPublisher_;
            MppiMarkerKeys mppiMarkerKeys_;
            ros::Publisher ungapTrajectoriesPublisher; /**< Publisher for un-gap trajectories */
            ros::Publisher trajPoseScoresPublisher; /**< Publisher for gap trajectory pose-wise scores */
            ros::Publisher trajSwitchIdxPublisher; /**< Publisher for planner trajectory switch count */
            ros::Publisher globalPlanPublisher; /**< Publisher for global plan */
            ros::Publisher globalPlanSnippetPublisher; /**< Publisher for visible snippet of global plan */
            ros::Publisher planLoopIdxPublisher; /**< Publisher for planning loop idx */
    };
}
