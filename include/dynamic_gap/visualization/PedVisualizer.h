
#pragma once

#include <dynamic_gap/visualization/Visualizer.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace dynamic_gap 
{
    /**
    * \brief Class for visualizing goal-related objects
    */
    class PedVisualizer : public Visualizer
    {
        // using Visualizer::Visualizer;

        public: 
            PedVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg);

            /**
            * \brief Draw pedestrian histories
            * \param currentTrueAgentPoses set of current true agent poses
            * \param currentTrueAgentVels set of current true agent velocities
            */
            void drawPedestrianHistories(const std::map<std::string, geometry_msgs::Pose> & currentTrueAgentPoses,
                                         const std::map<std::string, geometry_msgs::Vector3Stamped> & currentTrueAgentVels,
                                         const std::string & robot_frame);

        private: 

            std_msgs::ColorRGBA pedColors; /**< Color of pedestrians */

            ros::Publisher pedHistoryPublisher; /**< Publisher for agent histories */

    };
}