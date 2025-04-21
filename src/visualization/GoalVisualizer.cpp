#include <dynamic_gap/visualization/GoalVisualizer.h>

namespace dynamic_gap
{
    GoalVisualizer::GoalVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        globalGoalPublisher = nh.advertise<visualization_msgs::Marker>("global_goal", 10);
        globalPathLocalWaypointPublisher = nh.advertise<visualization_msgs::Marker>("global_path_local_waypoint", 10);
        gapGoalsPublisher = nh.advertise<visualization_msgs::MarkerArray>("gap_goals", 10);

        gapGoalsColor.r = 1.0;
        gapGoalsColor.g = 0.0;
        gapGoalsColor.b = 0.0;
        gapGoalsColor.a = 1;

        terminalGapGoalsColor.r = 0.5;
        terminalGapGoalsColor.g = 1;
        terminalGapGoalsColor.b = 0.5;
        terminalGapGoalsColor.a = 0.5;

        globalPathLocalWaypointColor.r = 0;
        globalPathLocalWaypointColor.g = 1;
        globalPathLocalWaypointColor.b = 0;
        globalPathLocalWaypointColor.a = 1;

        globalGoalColor.r = 1.;
        globalGoalColor.g = 1.;
        globalGoalColor.b = 0.;
        globalGoalColor.a = 1;        
    }

    void GoalVisualizer::drawGlobalGoal(const geometry_msgs::PoseStamped & globalGoalOdomFrame)
    {
        // First, clearing topic.
        clearMarkerPublisher(globalGoalPublisher);

        visualization_msgs::Marker globalGoalMarker;

        if (globalGoalOdomFrame.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGlobalGoal] Global goal frame_id is empty");
            return;
        }

        globalGoalMarker.header.frame_id = globalGoalOdomFrame.header.frame_id;
        globalGoalMarker.header.stamp = globalGoalOdomFrame.header.stamp;
        globalGoalMarker.ns = "global_goal";
        globalGoalMarker.id = 0;
        globalGoalMarker.type = visualization_msgs::Marker::SPHERE;
        globalGoalMarker.action = visualization_msgs::Marker::ADD;
        globalGoalMarker.pose.position.x = globalGoalOdomFrame.pose.position.x;
        globalGoalMarker.pose.position.y = globalGoalOdomFrame.pose.position.y;
        globalGoalMarker.pose.position.z = 0.0005;
        globalGoalMarker.pose.orientation.w = 1;
        globalGoalMarker.scale.x = 0.1;
        globalGoalMarker.scale.y = 0.1;
        globalGoalMarker.scale.z = 0.1;
        globalGoalMarker.color = globalGoalColor;
        globalGoalPublisher.publish(globalGoalMarker);        
    }

    void GoalVisualizer::drawGlobalPathLocalWaypoint(const geometry_msgs::PoseStamped & globalPathLocalWaypoint)
    {
        // First, clearing topic.
        clearMarkerPublisher(globalPathLocalWaypointPublisher);

        visualization_msgs::Marker globalPathLocalWaypointMarker;

        if (globalPathLocalWaypoint.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGlobalPathLocalWaypoint] Global path local waypoint frame_id is empty");
            return;
        }

        globalPathLocalWaypointMarker.header.frame_id = globalPathLocalWaypoint.header.frame_id;
        globalPathLocalWaypointMarker.header.stamp = globalPathLocalWaypoint.header.stamp;
        globalPathLocalWaypointMarker.ns = "local_goal";
        globalPathLocalWaypointMarker.id = 0;
        globalPathLocalWaypointMarker.type = visualization_msgs::Marker::SPHERE;
        globalPathLocalWaypointMarker.action = visualization_msgs::Marker::ADD;
        globalPathLocalWaypointMarker.pose.position.x = globalPathLocalWaypoint.pose.position.x;
        globalPathLocalWaypointMarker.pose.position.y = globalPathLocalWaypoint.pose.position.y;
        globalPathLocalWaypointMarker.pose.position.z = 0.0005;
        globalPathLocalWaypointMarker.pose.orientation.w = 1;
        globalPathLocalWaypointMarker.scale.x = 0.1;
        globalPathLocalWaypointMarker.scale.y = 0.1;
        globalPathLocalWaypointMarker.scale.z = 0.1;
        globalPathLocalWaypointMarker.color = globalPathLocalWaypointColor;
        globalPathLocalWaypointPublisher.publish(globalPathLocalWaypointMarker);
    }

    void GoalVisualizer::drawGapTubeGoals(const std::vector<GapTube *> & gapTubes) 
    {
        // First, clearing topic.
        clearMarkerPublisher(gapGoalsPublisher);

        // visualization_msgs::MarkerArray gapGoalsMarkerArray;
        visualization_msgs::MarkerArray gapGoalsMarkerArray;

        for (int i = 0; i < gapTubes.size(); i++)
        {
            GapTube * gapTube = gapTubes.at(i);

            drawGapTubeGoals(gapGoalsMarkerArray, gapTube);
        }

        gapGoalsPublisher.publish(gapGoalsMarkerArray);

        return;
    }

    void GoalVisualizer::drawGapTubeGoals(visualization_msgs::MarkerArray & gapGoalsMarkerArray, GapTube * gapTube)
    {
        int id = gapGoalsMarkerArray.markers.size();

        visualization_msgs::Marker goalMarker;

        drawModel(goalMarker, gapTube, id);
        gapGoalsMarkerArray.markers.push_back(goalMarker);
    }


    // void GoalVisualizer::drawGapTubeGoals(visualization_msgs::Marker & marker, 
    //                                         const std::vector<GapTube *> & gapTubes) 
    // {
    //     ROS_INFO_STREAM_NAMED("GoalVisualizer", "[drawGapGoals()]");

    //     if (gapTubes.size() == 0)
    //     {
    //         ROS_WARN_STREAM_NAMED("GoalVisualizer", "[drawGapGoals] No gap tube goals to visualize");
    //         return;
    //     }

    //     // if (gaps[0]->getFrame().empty())
    //     // {
    //     //     ROS_WARN_STREAM_NAMED("GoalVisualizer", "[drawGapGoals] Gap frame is empty");
    //     //     return;
    //     // }

    //     marker.header.stamp = ros::Time();
    //     marker.ns = "gap_goal";
    //     // marker.header.frame_id = gaps[0]->getFrame();

    //     marker.id = 0;

    //     marker.type = visualization_msgs::Marker::SPHERE_LIST;
    //     marker.action = visualization_msgs::Marker::ADD;     

    //     marker.pose.position.x = 0.0;
    //     marker.pose.position.y = 0.0;
    //     marker.pose.position.z = 0.0;
    //     marker.pose.orientation.x = 0;
    //     marker.pose.orientation.y = 0;
    //     marker.pose.orientation.z = 0;
    //     marker.pose.orientation.w = 1;

    //     marker.scale.x = 0.1;
    //     marker.scale.y = 0.1;
    //     marker.scale.z = 0.1;

    //     // marker.color = gapGoalsColor;

    //     for (int i = 0; i < gapTubes.size(); i++)
    //     {
    //         ROS_INFO_STREAM_NAMED("GoalVisualizer", "    tube " << i);

    //         GapTube * gapTube = gapTubes.at(i);
    //         for (int j = 0; j < gapTube->size(); j++)
    //         {
    //             ROS_INFO_STREAM_NAMED("GoalVisualizer", "       gap " << j);

    //             Gap * gap = gapTube->at(j);

    //             marker.header.frame_id = gap->getFrame();
                
    //             geometry_msgs::Point p;
    //             if (initial) 
    //             {
    //                 p.x = gap->getGoal()->getOrigGoalPosX(); // gap->goal.x_;
    //                 p.y = gap->getGoal()->getOrigGoalPosY(); // gap->goal.y_;
    //                 ROS_INFO_STREAM_NAMED("GoalVisualizer", "visualizing initial goal: " << p.x << ", " << p.y);
    //             } else 
    //             {
    //                 p.x = gap->getGoal()->getTermGoalPosX(); // gap->terminalGoal.x_;
    //                 p.y = gap->getGoal()->getTermGoalPosY(); // gap->terminalGoal.y_; 
    //                 ROS_INFO_STREAM_NAMED("GoalVisualizer", "visualizing terminal goal: " << p.x << ", " << p.y);
    //             }

    //             p.z = 0.0;
    //             marker.points.push_back(p);
    //             marker.colors.push_back(gapGoalsColor);
    //         }
    //     }

    //     // ROS_INFO_STREAM_NAMED("Visualizer", "marker: " << marker);
    // }

    void GoalVisualizer::drawModel(visualization_msgs::Marker & goalMarker, GapTube * gapTube, int & id) 
    {
    //     if (gap->getFrame().empty())
    //     {
    //         ROS_WARN_STREAM("[drawModel] Gap frame is empty");
    //         return;
    //     }
        
    //     std::string fullNamespace = ns;
    //     fullNamespace.append("_initial");

    //     auto colorIter = colorMap.find(fullNamespace);
    //     if (colorIter == colorMap.end()) 
    //     {
    //         ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
    //         return;
    //     }

    //     modelMarker.color = colorIter->second;

    //     // ROS_INFO_STREAM("[drawModel()]");
    //     modelMarker.header.frame_id = gap->getFrame();
    //     modelMarker.header.stamp = ros::Time();
    //     modelMarker.ns = ns;
    //     modelMarker.id = id++;
    //     modelMarker.type = visualization_msgs::Marker::ARROW;
    //     modelMarker.action = visualization_msgs::Marker::ADD;
        
    //     gap->getLeftGapPt()->getModel()->isolateGapDynamics();
    //     gap->getRightGapPt()->getModel()->isolateGapDynamics();

    //     Eigen::Vector4f leftModelState = gap->getLeftGapPt()->getModel()->getGapState();
    //     Eigen::Vector4f rightModelState = gap->getRightGapPt()->getModel()->getGapState();

    //     // ROS_INFO_STREAM("   leftModelState: " << leftModelState.transpose());
    //     // ROS_INFO_STREAM("   rightModelState: " << rightModelState.transpose());

    //     // ROS_INFO_STREAM("   gap->getLeftGapPt()->getModel()->getRobotVel(): " << gap->getLeftGapPt()->getModel()->getRobotVel());
    //     // ROS_INFO_STREAM("   gap->getRightGapPt()->getModel()->getRobotVel(): " << gap->getRightGapPt()->getModel()->getRobotVel());

    //     Eigen::Vector2f gapVel(0.0, 0.0);
    //     if (left)
    //     {
    //         modelMarker.pose.position.x = leftModelState[0];
    //         modelMarker.pose.position.y = leftModelState[1];
    //         gapVel = leftModelState.tail(2);
    //     } else
    //     {
    //         modelMarker.pose.position.x = rightModelState[0];
    //         modelMarker.pose.position.y = rightModelState[1];            
    //         gapVel << rightModelState.tail(2); 
    //     }
    //     modelMarker.pose.position.z = 0.01;

    //     float gapVelTheta;
    //     if (gapVel.norm() < std::numeric_limits<float>::epsilon())
    //     {
    //         gapVelTheta = 0.0;
    //     } else
    //     {
    //         gapVelTheta = std::atan2(gapVel[1], gapVel[0]);

    //     }

    //     tf2::Quaternion quat;
    //     quat.setRPY(0.0, 0.0, gapVelTheta);
        
    //     modelMarker.pose.orientation.x = quat.getX();
    //     modelMarker.pose.orientation.y = quat.getY();
    //     modelMarker.pose.orientation.z = quat.getZ();
    //     modelMarker.pose.orientation.w = quat.getW();

    //     modelMarker.scale.x = gapVel.norm() + 0.000001;
    //     modelMarker.scale.y = 0.1;
    //     modelMarker.scale.z = 0.000001;

    //     // modelMarker.color.a = 1.0;
    //     // modelMarker.color.r = 1.0;
    //     // modelMarker.color.b = 1.0;
    //     modelMarker.lifetime = ros::Duration(0);
    }    
}