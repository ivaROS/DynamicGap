
#include <dynamic_gap/visualization/TrajectoryVisualizer.h>

namespace dynamic_gap
{
    TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        trajSwitchIdxPublisher = nh.advertise<visualization_msgs::Marker>("trajectory_switch", 10);
        globalPlanPublisher = nh.advertise<geometry_msgs::PoseArray>("entire_global_plan", 10);
        trajPoseScoresPublisher = nh.advertise<visualization_msgs::MarkerArray>("traj_score", 10);
        gapTrajectoriesPublisher = nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 10);
        globalPlanSnippetPublisher = nh.advertise<geometry_msgs::PoseArray>("relevant_global_plan_snippet", 10);
    }

    void TrajectoryVisualizer::drawTrajectorySwitchCount(const int & trajSwitchIndex, const dynamic_gap::Trajectory & chosenTraj) 
    {
        geometry_msgs::PoseArray path = chosenTraj.getPath();
        geometry_msgs::Pose lastTrajPose = (path.poses.size() > 0) ? path.poses.back() : geometry_msgs::Pose();

        visualization_msgs::Marker trajSwitchIdxMarker;
        trajSwitchIdxMarker.header = path.header;
        trajSwitchIdxMarker.ns = "traj_switch_count";
        trajSwitchIdxMarker.id = 0;
        trajSwitchIdxMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trajSwitchIdxMarker.action = visualization_msgs::Marker::ADD;
        trajSwitchIdxMarker.pose.position = lastTrajPose.position;
        trajSwitchIdxMarker.pose.orientation = lastTrajPose.orientation;;
        trajSwitchIdxMarker.scale.z = 0.3;
        trajSwitchIdxMarker.color.a = 1.0; // Don't forget to set the alpha!
        trajSwitchIdxMarker.color.r = 0.0;
        trajSwitchIdxMarker.color.g = 0.0;
        trajSwitchIdxMarker.color.b = 0.0;
        trajSwitchIdxMarker.text = "SWITCH: " + std::to_string(trajSwitchIndex);
        trajSwitchIdxPublisher.publish(trajSwitchIdxMarker);
    }

    void TrajectoryVisualizer::drawGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlan) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;
        if (globalPlan.size() < 1) 
            ROS_WARN_STREAM("Goal Selector Returned Trajectory Size " << globalPlan.size() << " < 1");

        geometry_msgs::PoseArray globalPlanPoseArray;
        globalPlanPoseArray.header = globalPlan.at(0).header;
        for (const geometry_msgs::PoseStamped & pose : globalPlan) 
            globalPlanPoseArray.poses.push_back(pose.pose);

        globalPlanPublisher.publish(globalPlanPoseArray);
    }

    void TrajectoryVisualizer::drawGapTrajectories(const std::vector<dynamic_gap::Trajectory> & trajs) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;
        clearMarker.id = 0;
        clearMarker.ns =  "clear";
        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);
        gapTrajectoriesPublisher.publish(clearMarkerArray);

        if (trajs.size() == 0)
        {
            ROS_WARN_STREAM("no trajectories to visualize");
            return;
        }
        
        visualization_msgs::MarkerArray gapTrajMarkerArray;
        visualization_msgs::Marker gapTrajMarker;

        dynamic_gap::Trajectory traj = trajs.at(0);

        // The above makes this safe
        gapTrajMarker.header.frame_id = traj.getPath().header.frame_id;
        gapTrajMarker.header.stamp = ros::Time::now();
        gapTrajMarker.ns = "allTraj";
        gapTrajMarker.type = visualization_msgs::Marker::ARROW;
        gapTrajMarker.action = visualization_msgs::Marker::ADD;
        gapTrajMarker.scale.x = 0.1;
        gapTrajMarker.scale.y = 0.04; // 0.01;
        gapTrajMarker.scale.z = 0.0001;
        gapTrajMarker.color.a = 1;
        gapTrajMarker.color.b = 1.0;
        gapTrajMarker.color.g = 1.0;
        gapTrajMarker.lifetime = ros::Duration(0);

        for (const dynamic_gap::Trajectory & traj : trajs) 
        {
            geometry_msgs::PoseArray path = traj.getPath();
            for (const geometry_msgs::Pose & pose : path.poses) 
            {
                gapTrajMarker.id = int (gapTrajMarkerArray.markers.size());
                gapTrajMarker.pose = pose;
                gapTrajMarkerArray.markers.push_back(gapTrajMarker);
            }
        }
        gapTrajectoriesPublisher.publish(gapTrajMarkerArray);
    }

    void TrajectoryVisualizer::drawGapTrajectoryPoseScores(const std::vector<dynamic_gap::Trajectory> & trajs, 
                                                           const std::vector<std::vector<float>> & trajPoseScores) 
    {
        // if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray trajPoseScoresMarkerArray;
        visualization_msgs::Marker trajPoseScoresMarker;
        if (trajs.size() == 0)
        {
            ROS_WARN_STREAM("no trajectories to visualize");
            return;
        }

        // The above ensures this is safe
        trajPoseScoresMarker.header.frame_id = trajs.at(0).getPath().header.frame_id;
        trajPoseScoresMarker.header.stamp = ros::Time::now();
        trajPoseScoresMarker.ns = "trajScore";
        trajPoseScoresMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trajPoseScoresMarker.action = visualization_msgs::Marker::ADD;
        trajPoseScoresMarker.pose.orientation.w = 1;
        trajPoseScoresMarker.scale.x = 0.1;
        trajPoseScoresMarker.scale.y = 0.1;
        trajPoseScoresMarker.scale.z = 0.05;

        trajPoseScoresMarker.color.a = 1;
        trajPoseScoresMarker.color.r = 1;
        trajPoseScoresMarker.color.g = 1;
        trajPoseScoresMarker.color.b = 1;
        trajPoseScoresMarker.lifetime = ros::Duration(0);

        // ROS_FATAL_STREAM_COND(!trajs.size() == trajPoseScores.size(), "drawGapTrajectoryPoseScores size mismatch, trajs: "
        //                                         << trajs.size() << ", trajPoseScores: " << trajPoseScores.size());

        for (int i = 0; i < trajs.size(); i++) 
        {
            // ROS_FATAL_STREAM_COND(!trajs.at(i).poses.size() == trajPoseScores.at(i).size(), "drawGapTrajectoryPoseScores size mismatch," << i << "th "
            //     << trajs.at(i).poses.size() << ", trajPoseScores: " << trajPoseScores.at(i).size());
            
            for (int j = 0; j < trajs.at(i).getPath().poses.size(); j++) 
            {
                trajPoseScoresMarker.id = int (trajPoseScoresMarkerArray.markers.size());
                trajPoseScoresMarker.pose = trajs.at(i).getPath().poses.at(j);

                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << trajPoseScores.at(i).at(j);
                trajPoseScoresMarker.text = stream.str();

                trajPoseScoresMarkerArray.markers.push_back(trajPoseScoresMarker);
            }
        }
        trajPoseScoresPublisher.publish(trajPoseScoresMarkerArray);
    }

    void TrajectoryVisualizer::drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet) 
    {
        // try 
        // { 

        geometry_msgs::PoseArray globalPlanSnippetPoseArray;
        if (globalPlanSnippet.size() > 0)             // Should be safe with this check
            globalPlanSnippetPoseArray.header = globalPlanSnippet.at(0).header;

        for (const geometry_msgs::PoseStamped & pose : globalPlanSnippet) 
            globalPlanSnippetPoseArray.poses.push_back(pose.pose);

        globalPlanSnippetPublisher.publish(globalPlanSnippetPoseArray);

        // } catch (...) {
        //     ROS_FATAL_STREAM("getVisibleGlobalPlanSnippetRobotFrame");
        // }
    }
}