#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

namespace dynamic_gap
{
    class GapVisualizer : public Visualizer
    {
            using Visualizer::Visualizer;
        public: 

            GapVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void initialize(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void drawGap(visualization_msgs::MarkerArray &, dynamic_gap::Gap g, std::string ns, bool initial);
            void drawGaps(std::vector<dynamic_gap::Gap> g, std::string ns);
            void drawManipGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g, bool & circle, std::string ns, bool initial);
            void drawManipGaps(std::vector<dynamic_gap::Gap> vec, std::string ns);
            void drawGapsModels(std::vector<dynamic_gap::Gap> g);
            void drawGapModels(visualization_msgs::MarkerArray & gap_vel_arr, visualization_msgs::MarkerArray & gap_vel_error_arr, dynamic_gap::Gap g, std::string ns);
            void drawGapGroundTruthModels(visualization_msgs::MarkerArray & gap_vel_arr, dynamic_gap::Gap g, std::string ns);
            void drawGapVelocityError(visualization_msgs::MarkerArray & gap_vel_arr, dynamic_gap::Gap g, std::string ns);
            void drawReachableGap(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g);
            void drawReachableGaps(std::vector<dynamic_gap::Gap> g);
            void drawReachableGapsCenters(std::vector<dynamic_gap::Gap> g);
            void drawReachableGapCenters(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g);
            void drawReachableGapNoRGE(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g);
            void drawGapSplines(std::vector<dynamic_gap::Gap> g);
            void drawGapSpline(visualization_msgs::MarkerArray & vis_arr, dynamic_gap::Gap g);
            void draw_model_pt_head(visualization_msgs::Marker & model_vel_pt, dynamic_gap::Gap g, bool left, int & model_id, std::string ns, bool ground_truth);
            void draw_model_vel_error(visualization_msgs::Marker & model_vel_error_pt, visualization_msgs::Marker model_vel_pt, dynamic_gap::Gap g, bool left, std::string ns);




        private:
            std::map<std::string, std::vector<std_msgs::ColorRGBA>> colormap;
            ros::Publisher raw_gap_publisher;
            ros::Publisher simp_gap_publisher;
            ros::Publisher gapside_publisher;
            ros::Publisher gapgoal_publisher;
            ros::Publisher gapmodel_pos_publisher;
            ros::Publisher gapmodel_vel_publisher;
            ros::Publisher gapmodel_vel_error_publisher;
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
}