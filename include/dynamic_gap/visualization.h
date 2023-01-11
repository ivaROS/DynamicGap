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
#include <tf2/LinearMath/Quaternion.h>


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
            void drawGap(visualization_msgs::MarkerArray &, dynamic_gap::Gap g, std::string ns, bool initial);
            void drawGaps(std::vector<dynamic_gap::Gap> g, std::string ns);
            void drawManipGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g, bool & circle, std::string ns, bool initial);
            void drawManipGaps(std::vector<dynamic_gap::Gap> vec, std::string ns);
            void drawGapsModels(std::vector<dynamic_gap::Gap> g);
            void drawGapModels(visualization_msgs::MarkerArray & gap_vel_arr, dynamic_gap::Gap g, std::string ns);
            void drawGapGroundTruthModels(visualization_msgs::MarkerArray & gap_vel_arr, dynamic_gap::Gap g, std::string ns);
            void drawReachableGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g);
            void drawReachableGaps(std::vector<dynamic_gap::Gap> g);
            void drawReachableGapsCenters(std::vector<dynamic_gap::Gap> g);
            void drawReachableGapCenters(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g);
            void drawReachableGapNoRGE(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g);
            void drawGapSplines(std::vector<dynamic_gap::Gap> g);
            void drawGapSpline(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g);
            void draw_model_pt_head(visualization_msgs::Marker & model_vel_pt, dynamic_gap::Gap g, bool left, int & model_id, std::string ns, bool ground_truth);




        private:
            std::map<std::string, std::vector<std_msgs::ColorRGBA>> colormap;
            ros::Publisher raw_gap_publisher;
            ros::Publisher simp_gap_publisher;
            ros::Publisher gapside_publisher;
            ros::Publisher gapgoal_publisher;
            ros::Publisher gapmodel_pos_publisher;
            ros::Publisher gapmodel_vel_publisher;
            ros::Publisher gapmodel_pos_GT_publisher;
            ros::Publisher gapmodel_vel_GT_publisher;
            ros::Publisher reachable_gap_publisher;     
            ros::Publisher reachable_gap_centers_publisher;
            ros::Publisher reachable_gap_no_RGE_publisher;
            ros::Publisher gap_spline_publisher;

            int prev_num_gaps;  
            int prev_num_manip_gaps;
            int prev_num_models;
            int prev_num_reachable_gaps;
    };

    class TrajectoryVisualizer : public Visualizer{
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

    class GoalVisualizer : public Visualizer{
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

#endif