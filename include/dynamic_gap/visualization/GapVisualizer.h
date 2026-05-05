#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <fstream>
#include <string>
#include <std_msgs/Int16.h>

namespace dynamic_gap
{
    /**
    * \brief Class for visualizing gap-related objects
    */
    class GapVisualizer : public Visualizer
    {
        // using Visualizer::Visualizer;
        public: 
            GapVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg);

            /**
            * \brief Visualize set of gaps
            * \param gaps set of gaps to visualize
            * \param ns namespace of gaps to visualize
            */
            void drawGaps(const std::vector<Gap *> & gaps, const std::string & ns);
            
            /**
            * \brief Visualize set of gap models
            * \param gaps set of gaps whose models we want to visualize
            */
            void drawGapModels(const std::vector<Gap *> & gaps, const std::string & ns);

            /**
            * \brief Visualize set of manipulated gaps
            * \param gaps set of manipulated gaps to visualize
            * \param ns namespace of manipulated gaps to visualize
            */            
            void drawManipGaps(const std::vector<Gap *> & gaps, const std::string & ns);
            
            /**
            * \brief Visualize set of manipulated gap models
            * \param gaps set of manipulated gap models to visualize
            * \param ns namespace of manipulated gap models to visualize
            */            
            void drawManipGapModels(const std::vector<Gap *> & gaps, const std::string & ns);

            /**
            * \brief Visualize set of gap tubes
            * \param gapTubes set of gap tubes to visualize
            */
            void drawGapTubes(const std::vector<GapTube *> & gapTubes);            

        private:
            /**
             * \brief Subscriber for Arena task reset messages.
             *
             * The reset topic gives the current Arena run/episode index. This is written
             * to the CSV so the logged samples can be traced back to their run.
             */
            ros::Subscriber scenarioResetSub_;

            /**
             * \brief Current Arena run/episode ID.
             *
             * Updated from the scenario reset topic and written into every CSV row.
             */
            int currentRunId_ = 0;

            /**
             * \brief Output file stream for simplified gap velocity training data.
             */
            std::ofstream gapCsvFile_;

            /**
             * \brief Enables or disables simplified gap CSV logging.
             */
            bool gapCsvLoggingEnabled_ = true;

            /**
             * \brief Full path to the active CSV log file.
             */
            std::string gapCsvPath_;

            /**
             * \brief Unique session ID for the current logging file.
             *
             * Usually generated from launch time, for example:
             * dgap_26-05-05_08-29-26
             */
            std::string gapCsvSessionId_;

            /**
             * \brief Callback for Arena task reset messages.
             * \param msg Reset message containing the current run/episode index.
             */
            void scenarioResetCallback(const std_msgs::Int16::ConstPtr& msg);

            /**
             * \brief Initializes the simplified gap CSV logger.
             * \param nh ROS node handle used to read parameters if needed.
             *
             * Creates a timestamped CSV file and writes the header row.
             */
            void initializeGapCsvLogger(ros::NodeHandle& nh);

            /**
             * \brief Writes one simplified gap training sample to the CSV file.
             * \param stamp ROS timestamp for the sample.
             * \param run_id Current Arena run/episode ID.
             * \param gap_id Gap model ID.
             * \param side Gap side, usually "left" or "right".
             * \param gapState Simplified gap position state [x, y].
             * \param gapVel Gap velocity label [vx, vy].
             * \param ns Visualization namespace, used to confirm simplified/raw type.
             */
            void logSimplifiedGapCsvRow(
                const ros::Time& stamp,
                const int& run_id,
                const int& gap_id,
                const std::string& side,
                const Eigen::Vector2f& gapState,
                const Eigen::Vector2f& gapVel,
                const std::string& ns
            );
            /**
            * \brief initialize gap visualizer
            * \param nh ROS node handle for gap visualizer
            * \param cfg Planner hyperparameter config list
            */
            void initialize(ros::NodeHandle& nh, 
                            const DynamicGapConfig& cfg);

            /**
            * \brief Helper function for visualizing single gap
            * \param marker marker to add gap marker to
            * \param gaps gaps to visualize
            * \param ns namespace of gap to visualize
            */
            void drawGap(visualization_msgs::Marker & marker, 
                            const std::vector<Gap *> & gaps, 
                            const std::string & ns);


            /**
            * \brief Helper function for visualizing a single gap's left and right point models
            * \param gapModelMarkerArray marker array to add gap point models to
            * \param gap gap whose models we want to visualize
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawGapModelPositions(visualization_msgs::MarkerArray & gapModelMarkerArray, 
                                        Gap * gap, 
                                        const std::string & ns);

            /**
            * \brief Helper function for visualizing a single gap's left and right point models
            * \param gapModelMarkerArray marker array to add gap point models to
            * \param gap gap whose models we want to visualize
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawGapModelVelocities(visualization_msgs::MarkerArray & gapModelMarkerArray, 
                                        Gap * gap, 
                                        const std::string & ns);

            /**
            * \brief Helper function for visualizing a single gap point model
            * \param modelMarker marker for gap point model
            * \param gap gap whose models we want to visualize
            * \param left boolean for if we are visualizing left gap point model or right gap point model
            * \param id ID for model marker
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawModelPosition(visualization_msgs::Marker & modelMarker, 
                                    Gap * gap, 
                                    const bool & left, 
                                    int & id, 
                                    const std::string & ns);

            /**
            * \brief Helper function for visualizing a single gap point model
            * \param modelMarker marker for gap point model
            * \param gap gap whose models we want to visualize
            * \param left boolean for if we are visualizing left gap point model or right gap point model
            * \param id ID for model marker
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawModelVelocity(visualization_msgs::Marker & modelMarker, 
                                    Gap * gap, 
                                    const bool & left, 
                                    int & id, 
                                    const std::string & ns);

            /**
            * \brief Helper function for visualizing single manipulated gap
            * \param marker marker to add gap marker to
            * \param gaps manipulated gap to visualize
            * \param ns namespace of manipulated gap to visualize
            */
            void drawManipGap(visualization_msgs::Marker & marker, 
                                const std::vector<Gap *> & gaps, 
                                const std::string & ns);

            /**
            * \brief Helper function for visualizing a single gap's left and right point models
            * \param gapModelMarkerArray marker array to add gap point models to
            * \param gap gap whose models we want to visualize
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawManipGapModelPositions(visualization_msgs::MarkerArray & gapModelMarkerArray, 
                                            const int & gapIdx,
                                            Gap * gap, 
                                            const std::string & ns);

            /**
            * \brief Helper function for visualizing a single gap's left and right point models
            * \param gapModelMarkerArray marker array to add gap point models to
            * \param gap gap whose models we want to visualize
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawManipGapModelVelocities(visualization_msgs::MarkerArray & gapModelMarkerArray,
                                                const int & gapIdx,
                                                Gap * gap, 
                                                const std::string & ns);

            /**
            * \brief Helper function for visualizing a single gap point model
            * \param modelMarker marker for gap point model
            * \param gap gap whose models we want to visualize
            * \param left boolean for if we are visualizing left gap point model or right gap point model
            * \param id ID for model marker
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawManipModelPosition(visualization_msgs::Marker & modelMarker, 
                                        const int & gapIdx,
                                        Gap * gap, 
                                        const bool & left, 
                                        int & id, 
                                        const std::string & ns);

            /**
            * \brief Helper function for visualizing a single gap point model
            * \param modelMarker marker for gap point model
            * \param gap gap whose models we want to visualize
            * \param left boolean for if we are visualizing left gap point model or right gap point model
            * \param id ID for model marker
            * \param ns namespace of gap whose models we want to visualize
            */
            void drawManipModelVelocity(visualization_msgs::Marker & modelMarker, 
                                        const int & gapIdx,
                                        Gap * gap, 
                                        const bool & left, 
                                        int & id, 
                                        const std::string & ns);

            std::map<std::string, std_msgs::ColorRGBA> colorMap; /**< Map from gap namespace to color for visualization */

            ros::Publisher rawGapsPublisher; /**< Publisher for raw gaps */
            ros::Publisher simpGapsPublisher; /**< Publisher for simplified gaps */
            ros::Publisher manipGapsPublisher; /**< Publisher for manipulated gaps */
            ros::Publisher navigableGapsPublisher; /**< Publisher for navigable gaps */

            ros::Publisher rawGapModelPositionsPublisher; /**< Publisher for raw gap model positions */
            ros::Publisher simpGapModelPositionsPublisher; /**< Publisher for simp gap model positions */
            ros::Publisher manipGapModelPositionsPublisher; /**< Publisher for simp gap model positions */

            ros::Publisher rawGapModelVelocitiesPublisher; /**< Publisher for raw gap model velocities */
            ros::Publisher simpGapModelVelocitiesPublisher; /**< Publisher for simp gap model velocities */
            ros::Publisher manipGapModelVelocitiesPublisher; /**< Publisher for simp gap model velocities */

            ros::Publisher gapTubePublisher; /**< Publisher for gap tubes */

            int gapSpanResoln = 2;
            float invGapSpanResoln = 0.5;
    };
}