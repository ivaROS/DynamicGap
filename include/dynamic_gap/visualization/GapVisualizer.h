#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

namespace dynamic_gap
{
    /**
    * \brief Class for visualizing gap-related objects
    */
    class GapVisualizer : public Visualizer
    {
        using Visualizer::Visualizer;
        public: 
            GapVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);

            /**
            * \brief Visualize set of gaps
            * \param gaps set of gaps to visualize
            * \param ns namespace of gaps to visualize
            */
            void drawGaps(const std::vector<dynamic_gap::Gap *> & gaps, const std::string & ns);
            
            /**
            * \brief Visualize set of manipulated gaps
            * \param gaps set of manipulated gaps to visualize
            * \param ns namespace of manipulated gaps to visualize
            */            
            void drawManipGaps(const std::vector<dynamic_gap::Gap *> & gaps, const std::string & ns);
            
            /**
            * \brief Visualize set of gap models
            * \param gaps set of gaps whose models we want to visualize
            */
            void drawGapsModels(const std::vector<dynamic_gap::Gap *> & gaps);
            
            /**
            * \brief Visualize set of reachable gaps
            * \param gaps set of gaps whose reachable regions we want to visualize
            */            
            void drawReachableGaps(const std::vector<dynamic_gap::Gap *> & gaps,
                                    const int & highestScoreTrajIdx);

            // void drawGapSplines(const std::vector<dynamic_gap::Gap *> & gaps);

        private:
            /**
            * \brief initialize gap visualizer
            * \param nh ROS node handle for gap visualizer
            * \param cfg Planner hyperparameter config list
            */
            void initialize(ros::NodeHandle& nh, 
                            const dynamic_gap::DynamicGapConfig& cfg);

            // /**
            // * \brief Helper function for visualizing single gap
            // * \param markerArray marker array to add gap marker to
            // * \param gap gap to visualize
            // * \param ns namespace of gap to visualize
            // * \param initial boolean for if we are visualizing initial gap or terminal gap
            // */
            // void drawGap(visualization_msgs::MarkerArray & markerArray, 
            //              dynamic_gap::Gap * gap, 
            //              const std::string & ns, 
            //              const bool & initial);

            void drawGap(visualization_msgs::Marker & marker, const std::vector<dynamic_gap::Gap *> & gaps, 
                            const std::string & ns, const bool & initial);

            // /**
            // * \brief Helper function for visualizing single manipulated gap
            // * \param markerArray marker array to add gap marker to
            // * \param gap manipulated gap to visualize
            // * \param ns namespace of manipulated gap to visualize
            // * \param initial boolean for if we are visualizing initial gap or terminal gap
            // */
            // void drawManipGap(visualization_msgs::MarkerArray & markerArray, 
            //                   dynamic_gap::Gap * gap, 
            //                   const std::string & ns, 
            //                   const bool & initial);

            void drawManipGap(visualization_msgs::Marker & marker, const std::vector<dynamic_gap::Gap *> & gaps, 
                                        const std::string & ns, const bool & initial);

            /**
            * \brief Helper function for visualizing a single gap's left and right point models
            * \param gapModelMarkerArray marker array to add gap point models to
            * \param gap gap whose models we want to visualize
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawGapModels(visualization_msgs::MarkerArray & gapModelMarkerArray, 
                               dynamic_gap::Gap * gap, 
                               const std::string & ns);

            /**
            * \brief Helper function for visualizing a single gap point model
            * \param modelMarker marker for gap point model
            * \param gap gap whose models we want to visualize
            * \param left boolean for if we are visualizing left gap point model or right gap point model
            * \param id ID for model marker
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawModel(visualization_msgs::Marker & modelMarker, 
                            dynamic_gap::Gap * gap, 
                            const bool & left, 
                            int & id, 
                            const std::string & ns);

            // void drawReachableGapsCenters(const std::vector<dynamic_gap::Gap *> & gap);

            // void drawReachableGapCenters(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap);

            void drawReachableGap(visualization_msgs::Marker & marker, 
                                    const std::vector<dynamic_gap::Gap *> & gaps,
                                    const int & highestScoreTrajIdx);
            // void drawReachableGap(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap);
            // void drawReachableGapNoRGE(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap);
            
            // void drawGapSpline(visualization_msgs::MarkerArray & markerArray, dynamic_gap::Gap * gap);

            std::map<std::string, std_msgs::ColorRGBA> colorMap; /**< Map from gap namespace to color for visualization */

            ros::Publisher rawGapsPublisher; /**< Publisher for raw gaps */
            ros::Publisher simpGapsPublisher; /**< Publisher for simplified gaps */
            ros::Publisher manipGapsPublisher; /**< Publisher for manipulated gaps */
            ros::Publisher reachableGapsPublisher; /**< Publisher for reachable gaps */
            ros::Publisher gapModelsPublisher; /**< Publisher for gap models */
            ros::Publisher ahpfCentersPublisher; /**< Publisher for AHPF harmonic term center positions */
            ros::Publisher gapSplinesPublisher; /**< Publisher for approximate spline trajectories through gaps */

    };
}