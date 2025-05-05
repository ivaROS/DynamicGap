#include <dynamic_gap/visualization/PedVisualizer.h>

namespace dynamic_gap
{   

PedVisualizer::PedVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg)
{
    pedHistoryPublisher = nh.advertise<visualization_msgs::MarkerArray>("pedestrian_histories", 10);

    pedColors = std_msgs::ColorRGBA();
    pedColors.r = 0.50f;
    pedColors.g = 0.50f;
    pedColors.b = 0.50f;
    pedColors.a = 1.0f;

}

void PedVisualizer::drawPedestrianHistories(const std::map<std::string, geometry_msgs::Pose> & currentTrueAgentPoses,
                                            const std::map<std::string, geometry_msgs::Vector3Stamped> & currentTrueAgentVels,
                                            const std::string & robot_frame)
{
    ROS_INFO_STREAM_NAMED("PedVisualizer", "[drawPedestrianHistories]");

    visualization_msgs::MarkerArray pedHistoryMarkerArray;

    int counter = 0;
    int numHistories = 5;
    for (const auto & agent : currentTrueAgentPoses)
    {
        const std::string & agentID = agent.first;
        const geometry_msgs::Pose & agentPose = agent.second;

        ROS_INFO_STREAM_NAMED("PedVisualizer", "Agent ID: " << agentID);  
        ROS_INFO_STREAM_NAMED("PedVisualizer", "Agent Pose: " << agentPose.position.x << ", " << agentPose.position.y << ", " << agentPose.position.z);
        ROS_INFO_STREAM_NAMED("PedVisualizer", "Agent Orientation: " << agentPose.orientation.x << ", " << agentPose.orientation.y << ", " << agentPose.orientation.z << ", " << agentPose.orientation.w);

        visualization_msgs::Marker pedHistoryMarker;
        pedHistoryMarker.header.frame_id = robot_frame;
        pedHistoryMarker.header.stamp = ros::Time::now();
        pedHistoryMarker.ns = "pedestrian_histories";
        pedHistoryMarker.id = pedHistoryMarkerArray.markers.size();
        pedHistoryMarker.type = visualization_msgs::Marker::CYLINDER;
        pedHistoryMarker.action = visualization_msgs::Marker::ADD;
        pedHistoryMarker.pose = agentPose;
        pedHistoryMarker.scale.x = 0.45;
        pedHistoryMarker.scale.y = 0.45;
        pedHistoryMarker.scale.z = 0.00001; // Height of the cylinder
        pedHistoryMarker.color = pedColors;

        // Set the lifetime of the marker
        // pedHistoryMarker.lifetime = ros::Duration(0.1); // 0.1 seconds

        ROS_INFO_STREAM_NAMED("PedVisualizer", "Pedestrian Marker: ");
        ROS_INFO_STREAM_NAMED("PedVisualizer", "        " << pedHistoryMarker);

        // Add the marker to the array
        pedHistoryMarkerArray.markers.push_back(pedHistoryMarker);

        for (int i = 0; i < numHistories; i++)
        {
            geometry_msgs::Vector3Stamped agentVel = currentTrueAgentVels.at(agentID);

            pedHistoryMarker.id = pedHistoryMarkerArray.markers.size();

            pedHistoryMarker.pose.position.x += agentVel.vector.x;
            pedHistoryMarker.pose.position.y += agentVel.vector.y;
            pedHistoryMarker.color.a -= (1.0f / numHistories);

            // Add the marker to the array
            pedHistoryMarkerArray.markers.push_back(pedHistoryMarker);
        }

    }

    // Publish the marker array
    pedHistoryPublisher.publish(pedHistoryMarkerArray);

    return;
}

}