#include <dynamic_gap/visualization/GoalVisualizer.h>

namespace dynamic_gap
{
    GoalVisualizer::GoalVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        globalGoalPublisher = nh.advertise<visualization_msgs::Marker>("global_goal", 10);
        globalPathLocalWaypointPublisher = nh.advertise<visualization_msgs::Marker>("goals", 10);
        gapGoalsPublisher = nh.advertise<visualization_msgs::Marker>("gap_goals", 10);

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

    void GoalVisualizer::drawGapGoals(const std::vector<Gap *> & gaps) 
    {
        // First, clearing topic.
        clearMarkerPublisher(gapGoalsPublisher);

        // visualization_msgs::MarkerArray gapGoalsMarkerArray;
        visualization_msgs::Marker gapGoalsMarker;
        drawGapGoals(gapGoalsMarker, gaps, false);

        gapGoalsPublisher.publish(gapGoalsMarker);

        return;
    }

    void GoalVisualizer::drawGapGoals(visualization_msgs::Marker & marker, 
                                        const std::vector<Gap *> & gaps, 
                                        const bool & initial) 
    {
        ROS_INFO_STREAM_NAMED("Visualizer", "[drawGapGoals()]");

        if (gaps.size() == 0)
            return;

        if (gaps[0]->frame_.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGapGoals] Gap frame is empty");
            return;
        }

        marker.header.stamp = ros::Time();
        marker.ns = "gap_goal";
        marker.header.frame_id = gaps[0]->frame_;

        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;     

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color = gapGoalsColor;

        for (Gap * gap : gaps) 
        {
            // marker.header.frame_id = gap->frame_;

            geometry_msgs::Point p;
            if (initial) 
            {
                p.x = gap->goal.x_;
                p.y = gap->goal.y_;
                ROS_INFO_STREAM_NAMED("Visualizer", "visualizing initial goal: " << gap->goal.x_ << ", " << gap->goal.y_);
            } else 
            {
                p.x = gap->terminalGoal.x_;
                p.y = gap->terminalGoal.y_; 
                ROS_INFO_STREAM_NAMED("Visualizer", "visualizing terminal goal: " << gap->terminalGoal.x_ << ", " << gap->terminalGoal.y_);
            }

            marker.points.push_back(p);
        }
    }
}