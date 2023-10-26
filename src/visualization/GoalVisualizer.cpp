#include <dynamic_gap/visualization/GoalVisualizer.h>

namespace dynamic_gap
{
    GoalVisualizer::GoalVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        goal_pub = nh.advertise<visualization_msgs::Marker>("goals", 10);
        gapwp_pub = nh.advertise<visualization_msgs::MarkerArray>("gap_goals", 10);

        gapwp_color.r = 1.0;
        gapwp_color.g = 0.666;
        gapwp_color.b = 0.0;
        gapwp_color.a = 1;

        terminal_gapwp_color.r = 0.5;
        terminal_gapwp_color.g = 1;
        terminal_gapwp_color.b = 0.5;
        terminal_gapwp_color.a = 0.5;

        localGoal_color.r = 0;
        localGoal_color.g = 1;
        localGoal_color.b = 0;
        localGoal_color.a = 1;
    }

    void GoalVisualizer::localGoal(geometry_msgs::PoseStamped localGoal)
    {
        if (!cfg_->gap_viz.debug_viz) return;

        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = localGoal.header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "local_goal";
        lg_marker.id = 0;
        lg_marker.type = visualization_msgs::Marker::SPHERE;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.position.x = localGoal.pose.position.x;
        lg_marker.pose.position.y = localGoal.pose.position.y;
        lg_marker.pose.position.z = 0.0005;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.1;
        lg_marker.color = localGoal_color;
        goal_pub.publish(lg_marker);
    }

    void GoalVisualizer::drawGapGoal(visualization_msgs::MarkerArray& vis_arr, 
                                     const dynamic_gap::Gap & gap, bool initial) 
    {
        if (!cfg_->gap_viz.debug_viz) return;

        visualization_msgs::Marker lg_marker;
        lg_marker.header.frame_id = gap.frame_;
        // std::cout << "g frame in draw gap goal: " << gap.frame_ << std::endl;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "gap_goal";
        lg_marker.id = int (vis_arr.markers.size());
        lg_marker.type = visualization_msgs::Marker::CYLINDER;
        lg_marker.action = visualization_msgs::Marker::ADD;
        if (initial) {
            lg_marker.pose.position.x = gap.goal.x_;
            lg_marker.pose.position.y = gap.goal.y_;
            lg_marker.color = gapwp_color;
            //ROS_INFO_STREAM("visualizing initial goal: " << gap.goal.x << ", " << gap.goal.y);
        } else {
            lg_marker.pose.position.x = gap.terminalGoal.x_;
            lg_marker.pose.position.y = gap.terminalGoal.y_; 
            lg_marker.color = gapwp_color;
            // ROS_INFO_STREAM("visualizing terminal goal: " << gap.terminalGoal.x << ", " << gap.terminalGoal.y);
        }
        lg_marker.pose.position.z = 0.0001;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.000001;
        lg_marker.lifetime = ros::Duration(100.0);
        vis_arr.markers.push_back(lg_marker);
    }

    void GoalVisualizer::drawGapGoals(const std::vector<dynamic_gap::Gap> & gaps) 
    {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = "gap_goal";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        gapwp_pub.publish(clear_arr);


        visualization_msgs::MarkerArray vis_arr;
        for (const dynamic_gap::Gap & gap : gaps) 
        {
            //drawGapGoal(vis_arr, gap, true);
            drawGapGoal(vis_arr, gap, false);
        }
        gapwp_pub.publish(vis_arr);
        return;
    }
}