#ifndef VIS_H
#define VIS_H

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/gap.h>
#include <dynamic_gap/dynamicgap_config.h>
#include <vector>
#include <map>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

namespace dynamic_gap
{
    class Visualizer {
        public: 
            Visualizer() {};
            ~Visualizer() {};

            Visualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            Visualizer& operator=(Visualizer other) {cfg_ = other.cfg_;};
            Visualizer(const Visualizer &t) {cfg_ = t.cfg_;};

        protected:
            const DynamicGapConfig* cfg_;
    };

    class GapVisualizer : public Visualizer{
            using Visualizer::Visualizer;
        public: 

            GapVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void initialize(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void drawGap(visualization_msgs::MarkerArray &, dynamic_gap::Gap g, std::string ns, std::string color = "Default");
            void drawGaps(std::vector<dynamic_gap::Gap> g, std::string ns, std::string color = "Default");
            void drawManipGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g, bool & circle);
            void drawManipGaps(std::vector<dynamic_gap::Gap> vec);
            void drawGapsModels(std::vector<dynamic_gap::Gap> g);
            void drawGapModels(visualization_msgs::MarkerArray & model_arr,dynamic_gap::Gap g, std::string ns);

        private:
            std::map<std::string, std::vector<std_msgs::ColorRGBA>> colormap;
            ros::Publisher gaparc_publisher;
            ros::Publisher gapside_publisher;
            ros::Publisher gapgoal_publisher;
            ros::Publisher gapmodel_publisher;
    };

    class TrajectoryVisualizer : public Visualizer{
            using Visualizer::Visualizer;
        public: 
            TrajectoryVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void globalPlanRbtFrame(const std::vector<geometry_msgs::PoseStamped> & );
            void trajScore(geometry_msgs::PoseArray, std::vector<double>);
            void pubAllTraj(std::vector<geometry_msgs::PoseArray> prr);
            void pubAllScore(std::vector<geometry_msgs::PoseArray>, std::vector<std::vector<double>>);
        private: 
            ros::Publisher goal_selector_traj_vis;
            ros::Publisher trajectory_score;
            ros::Publisher all_traj_viz;
    };

    class GoalVisualizer : public Visualizer{
        public: 
            using Visualizer::Visualizer;
            GoalVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void localGoal(geometry_msgs::PoseStamped);
            void drawGapGoal(visualization_msgs::MarkerArray&, dynamic_gap::Gap);
            void drawGapGoals(std::vector<dynamic_gap::Gap>);
        private: 
            ros::Publisher goal_pub;
            ros::Publisher gapwp_pub;
            std_msgs::ColorRGBA gapwp_color;
            std_msgs::ColorRGBA localGoal_color;
    };
}

#endif