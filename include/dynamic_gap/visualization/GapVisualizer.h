#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

namespace dynamic_gap
{
    class GapVisualizer : public Visualizer
    {
        using Visualizer::Visualizer;
        public: 
            GapVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            void drawGaps(const std::vector<dynamic_gap::Gap *> & gaps, const std::string & ns);
            void drawManipGaps(const std::vector<dynamic_gap::Gap *> & gaps, const std::string & ns);
            void drawGapsModels(const std::vector<dynamic_gap::Gap *> & gaps);
            void drawReachableGaps(const std::vector<dynamic_gap::Gap *> & gaps);
            // void drawGapSplines(const std::vector<dynamic_gap::Gap *> & gaps);

        private:
            void initialize(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);

            void drawGap(visualization_msgs::MarkerArray &, dynamic_gap::Gap * gap, const std::string & ns, const bool & initial);

            void drawManipGap(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap, const std::string & ns, const bool & initial);

            void drawGapModels(visualization_msgs::MarkerArray & gapModelMarkerArray, 
                               dynamic_gap::Gap * gap, const std::string & ns); // visualization_msgs::MarkerArray & gap_vel_error_arr, 
            // void drawGapGroundTruthModels(visualization_msgs::MarkerArray & gapModelMarkerArray,   
            //                                 dynamic_gap::Gap * gap, std::string ns);

            void drawModel(visualization_msgs::Marker & modelMarker, 
                            dynamic_gap::Gap * gap, const bool & left, int & id, const std::string & ns, const bool & ground_truth);
            // void draw_model_vel_error(visualization_msgs::Marker & model_vel_error_pt, visualization_msgs::Marker modelMarker, 
            //                             dynamic_gap::Gap * gap, bool left, std::string ns);

            void drawReachableGap(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap);
            void drawReachableGapsCenters(const std::vector<dynamic_gap::Gap *> & gap);
            void drawReachableGapCenters(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap);
            // void drawReachableGapNoRGE(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap);
            

            // void drawGapSpline(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap);

            std::map<std::string, std_msgs::ColorRGBA> colorMap;
            ros::Publisher rawGapsPublisher;
            ros::Publisher simpGapsPublisher;
            ros::Publisher manipGapsPublisher;
            ros::Publisher reachableGapsPublisher;     

            ros::Publisher gapModelPosPublisher;
            ros::Publisher gapModelVelPublisher;
            // ros::Publisher gapmodel_vel_error_publisher;
            // ros::Publisher gapmodel_pos_GT_publisher;
            // ros::Publisher gapmodel_vel_GT_publisher;
            ros::Publisher ahpfCentersPublisher;
            // ros::Publisher reachable_gap_no_RGE_publisher;
            ros::Publisher gapSplinesPublisher;

            // int prev_num_gaps;  
            // int prev_num_manip_gaps;
            // int prev_num_models;
            // int prev_num_reachable_gaps;
    };
}