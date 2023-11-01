
#include <dynamic_gap/visualization/TrajectoryVisualizer.h>

namespace dynamic_gap
{
    TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        entire_global_plan_pub = nh.advertise<geometry_msgs::PoseArray>("entire_global_plan", 10);
        trajectory_score = nh.advertise<visualization_msgs::MarkerArray>("traj_score", 10);
        all_traj_viz = nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 10);
        relevant_global_plan_snippet_pub = nh.advertise<geometry_msgs::PoseArray>("relevant_global_plan_snippet", 10);
        trajectory_switch_pub = nh.advertise<visualization_msgs::Marker>("trajectory_switch", 10);
    }

    void TrajectoryVisualizer::drawTrajectorySwitchCount(int switch_index, const geometry_msgs::PoseArray & switch_traj) 
    {
        geometry_msgs::Pose last_pose;
        if (switch_traj.poses.size() > 0)
            last_pose = switch_traj.poses[switch_traj.poses.size() - 1];
        else 
            last_pose = geometry_msgs::Pose();

        visualization_msgs::Marker marker;
        marker.header = switch_traj.header;
        marker.ns = "traj_switch_count";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = last_pose.position;
        marker.pose.orientation = last_pose.orientation;;
        marker.scale.z = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.text = "SWITCH: " + std::to_string(switch_index);
        trajectory_switch_pub.publish(marker);
    }

    void TrajectoryVisualizer::drawEntireGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlan) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;
        if (globalPlan.size() < 1) 
            ROS_WARN_STREAM("Goal Selector Returned Trajectory Size " << globalPlan.size() << " < 1");

        geometry_msgs::PoseArray poseArray;
        poseArray.header = globalPlan.at(0).header;
        for (const geometry_msgs::PoseStamped & pose : globalPlan) 
            poseArray.poses.push_back(pose.pose);

        entire_global_plan_pub.publish(poseArray);
    }

    void TrajectoryVisualizer::pubAllScore(const std::vector<geometry_msgs::PoseArray> & prr, 
                                            const std::vector<std::vector<float>> & cost) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray score_arr;
        visualization_msgs::Marker lg_marker;
        if (prr.size() == 0)
        {
            ROS_WARN_STREAM("traj count length 0");
            return;
        }

        // The above ensures this is safe
        lg_marker.header.frame_id = prr.at(0).header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "trajScore";
        lg_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.pose.orientation.w = 1;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.1;
        lg_marker.scale.z = 0.05;

        lg_marker.color.a = 1;
        lg_marker.color.r = 1;
        lg_marker.color.g = 1;
        lg_marker.color.b = 1;
        lg_marker.lifetime = ros::Duration(100.0);


        ROS_FATAL_STREAM_COND(!prr.size() == cost.size(), "pubAllScore size mismatch, prr: "
            << prr.size() << ", cost: " << cost.size());

        for (int i = 0; i < prr.size(); i++) 
        {
            ROS_FATAL_STREAM_COND(!prr.at(i).poses.size() == cost.at(i).size(), "pubAllScore size mismatch," << i << "th "
                << prr.at(i).poses.size() << ", cost: " << cost.at(i).size());
            
            for (int j = 0; j < prr.at(i).poses.size(); j++) 
            {
                lg_marker.id = int (score_arr.markers.size());
                lg_marker.pose = prr.at(i).poses.at(j);

                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << cost.at(i).at(j);
                lg_marker.text = stream.str();

                score_arr.markers.push_back(lg_marker);
            }
        }
        trajectory_score.publish(score_arr);
    }

    void TrajectoryVisualizer::pubAllTraj(const std::vector<geometry_msgs::PoseArray> & trajectories) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns =  "allTraj";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        all_traj_viz.publish(clear_arr);

        
        visualization_msgs::MarkerArray vis_traj_arr;
        visualization_msgs::Marker lg_marker;
        if (trajectories.size() == 0)
        {
            ROS_WARN_STREAM("traj count length 0");
            return;
        }

        // The above makes this safe
        lg_marker.header.frame_id = trajectories.at(0).header.frame_id;
        lg_marker.header.stamp = ros::Time::now();
        lg_marker.ns = "allTraj";
        lg_marker.type = visualization_msgs::Marker::ARROW;
        lg_marker.action = visualization_msgs::Marker::ADD;
        lg_marker.scale.x = 0.1;
        lg_marker.scale.y = 0.04;// 0.01;
        lg_marker.scale.z = 0.0001;
        lg_marker.color.a = 1;
        lg_marker.color.b = 1.0;
        lg_marker.color.g = 1.0;
        lg_marker.lifetime = ros::Duration(100.0);

        for (const geometry_msgs::PoseArray & traj : trajectories) 
        {
            for (const geometry_msgs::Pose & pose : traj.poses) 
            {
                lg_marker.id = int (vis_traj_arr.markers.size());
                lg_marker.pose = pose;
                vis_traj_arr.markers.push_back(lg_marker);
            }
        }
        all_traj_viz.publish(vis_traj_arr);
    }

    void TrajectoryVisualizer::drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet) 
    {
        try 
        { 
            geometry_msgs::PoseArray pub_traj;
            if (globalPlanSnippet.size() > 0) 
            {
                // Should be safe with this check
                pub_traj.header = globalPlanSnippet.at(0).header;
            }

            for (const geometry_msgs::PoseStamped & pose : globalPlanSnippet) 
            {
                pub_traj.poses.push_back(pose.pose);
            }
            relevant_global_plan_snippet_pub.publish(pub_traj);
        } catch (...) {
            ROS_FATAL_STREAM("getVisibleGlobalPlanSnippetRobotFrame");
        }
    }
}