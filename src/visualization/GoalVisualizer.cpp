#include <dynamic_gap/visualization/GoalVisualizer.h>

namespace dynamic_gap
{
    GoalVisualizer::GoalVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        globalPathLocalWaypointPublisher = nh.advertise<visualization_msgs::Marker>("goals", 10);
        gapGoalsPublisher = nh.advertise<visualization_msgs::MarkerArray>("gap_goals", 10);

        gapGoalsColor.r = 1.0;
        gapGoalsColor.g = 0.666;
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
    }

    void GoalVisualizer::drawGlobalPathLocalWaypoint(const geometry_msgs::PoseStamped & globalPathLocalWaypoint)
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        visualization_msgs::Marker globalPathLocalWaypointMarker;
        globalPathLocalWaypointMarker.header.frame_id = globalPathLocalWaypoint.header.frame_id;
        globalPathLocalWaypointMarker.header.stamp = ros::Time::now();
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

    void GoalVisualizer::drawGapGoals(const std::vector<dynamic_gap::Gap *> & gaps) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns = "gap_goal";
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);
        gapGoalsPublisher.publish(clearMarkerArray);

        visualization_msgs::MarkerArray gapGoalsMarkerArray;
        for (dynamic_gap::Gap * gap : gaps) 
        {
            //drawGapGoal(gapGoalsMarkerArray, gap, true);
            drawGapGoal(gapGoalsMarkerArray, gap, false);
        }
        gapGoalsPublisher.publish(gapGoalsMarkerArray);
        return;
    }

    void GoalVisualizer::drawGapGoal(visualization_msgs::MarkerArray & gapGoalsMarkerArray, 
                                     dynamic_gap::Gap * gap, const bool & initial) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        visualization_msgs::Marker gapGoalMarker;
        gapGoalMarker.header.frame_id = gap->frame_;
        // std::cout << "g frame in draw gap goal: " << gap->frame_ << std::endl;
        gapGoalMarker.header.stamp = ros::Time::now();
        gapGoalMarker.ns = "gap_goal";
        gapGoalMarker.id = int (gapGoalsMarkerArray.markers.size());
        gapGoalMarker.type = visualization_msgs::Marker::CYLINDER;
        gapGoalMarker.action = visualization_msgs::Marker::ADD;
        if (initial) 
        {
            gapGoalMarker.pose.position.x = gap->goal.x_;
            gapGoalMarker.pose.position.y = gap->goal.y_;
            gapGoalMarker.color = gapGoalsColor;
            //ROS_INFO_STREAM("visualizing initial goal: " << gap->goal.x << ", " << gap->goal.y);
        } else 
        {
            gapGoalMarker.pose.position.x = gap->terminalGoal.x_;
            gapGoalMarker.pose.position.y = gap->terminalGoal.y_; 
            gapGoalMarker.color = gapGoalsColor;
            // ROS_INFO_STREAM("visualizing terminal goal: " << gap->terminalGoal.x << ", " << gap->terminalGoal.y);
        }
        gapGoalMarker.pose.position.z = 0.0001;
        gapGoalMarker.pose.orientation.w = 1;
        gapGoalMarker.scale.x = 0.1;
        gapGoalMarker.scale.y = 0.1;
        gapGoalMarker.scale.z = 0.000001;
        gapGoalMarker.lifetime = ros::Duration(0);
        gapGoalsMarkerArray.markers.push_back(gapGoalMarker);
    }
}