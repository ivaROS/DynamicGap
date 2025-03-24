
#include <dynamic_gap/visualization/TrajectoryVisualizer.h>

namespace dynamic_gap
{
    TrajectoryVisualizer::TrajectoryVisualizer(ros::NodeHandle& nh, const DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;
        trajSwitchIdxPublisher = nh.advertise<visualization_msgs::Marker>("trajectory_switch", 10);
        planLoopIdxPublisher = nh.advertise<visualization_msgs::Marker>("planning_loop_idx", 10);

        currentTrajectoryPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("curr_exec_dg_traj", 1);

        // globalPlanPublisher = nh.advertise<geometry_msgs::PoseArray>("entire_global_plan", 10);
        globalPlanPublisher = nh.advertise<visualization_msgs::MarkerArray>("entire_global_plan", 10);

        trajPoseScoresPublisher = nh.advertise<visualization_msgs::MarkerArray>("traj_score", 10);
        gapTrajectoriesPublisher = nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 10);
        ungapTrajectoriesPublisher = nh.advertise<visualization_msgs::MarkerArray>("candidate_ungap_trajectories", 10);

        // globalPlanSnippetPublisher = nh.advertise<geometry_msgs::PoseArray>("relevant_global_plan_snippet", 10);
        globalPlanSnippetPublisher = nh.advertise<visualization_msgs::MarkerArray>("relevant_global_plan_snippet", 10);
    }

    void TrajectoryVisualizer::drawCurrentTrajectory(const Trajectory & traj)
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(currentTrajectoryPublisher_);

        visualization_msgs::MarkerArray trajMarkerArray;
        visualization_msgs::Marker trajMarker;

        if (traj.getPathRbtFrame().header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawCurrentTrajectory] Trajectory frame_id is empty");
            return;
        }

        trajMarker.header.frame_id = traj.getPathRbtFrame().header.frame_id;
        trajMarker.header.stamp = traj.getPathRbtFrame().header.stamp;
        trajMarker.ns = "currentTraj";
        trajMarker.type = visualization_msgs::Marker::ARROW;
        trajMarker.action = visualization_msgs::Marker::ADD;
        trajMarker.scale.x = 0.1;
        trajMarker.scale.y = 0.08; // 0.01;
        trajMarker.scale.z = 0.0001;
        trajMarker.color.a = 1;
        trajMarker.color.r = 1.0;
        trajMarker.color.g = 0.0;
        trajMarker.color.b = 0.0;

        trajMarker.lifetime = ros::Duration(0);     
        
        geometry_msgs::PoseArray path = traj.getPathRbtFrame();
        for (const geometry_msgs::Pose & pose : path.poses) 
        {
            trajMarker.id = int (trajMarkerArray.markers.size());
            trajMarker.pose = pose;
            trajMarkerArray.markers.push_back(trajMarker);
        }
    
        currentTrajectoryPublisher_.publish(trajMarkerArray);
    }

    void TrajectoryVisualizer::drawPlanningLoopIdx(const int & planningLoopIdx) 
    {
        // First, clearing topic.
        clearMarkerPublisher(planLoopIdxPublisher);

        visualization_msgs::Marker trajSwitchIdxMarker;

        if (cfg_->robot_frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawPlanningLoopIdx] Trajectory frame_id is empty");
            return; 
        }

        trajSwitchIdxMarker.header.frame_id = cfg_->robot_frame_id;
        trajSwitchIdxMarker.header.stamp = ros::Time::now();

        trajSwitchIdxMarker.ns = "planning_loop_idx";
        trajSwitchIdxMarker.id = 0;
        trajSwitchIdxMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trajSwitchIdxMarker.action = visualization_msgs::Marker::ADD;
        trajSwitchIdxMarker.pose.position.x = 0.0;
        trajSwitchIdxMarker.pose.position.y = 0.0;
        trajSwitchIdxMarker.pose.position.z = 0.05;
        trajSwitchIdxMarker.pose.orientation.w = 1.0;
        trajSwitchIdxMarker.pose.orientation.x = 0.0;
        trajSwitchIdxMarker.pose.orientation.y = 0.0;
        trajSwitchIdxMarker.pose.orientation.z = 0.0;

        trajSwitchIdxMarker.scale.z = 0.3;
        trajSwitchIdxMarker.color.a = 1.0; // Don't forget to set the alpha!
        trajSwitchIdxMarker.color.r = 0.0;
        trajSwitchIdxMarker.color.g = 0.0;
        trajSwitchIdxMarker.color.b = 0.0;
        trajSwitchIdxMarker.text = "PLAN: " + std::to_string(planningLoopIdx);
        planLoopIdxPublisher.publish(trajSwitchIdxMarker);
    }

    void TrajectoryVisualizer::drawTrajectorySwitchCount(const int & trajSwitchIndex, const Trajectory & chosenTraj) 
    {
        // First, clearing topic.
        clearMarkerPublisher(trajSwitchIdxPublisher);

        geometry_msgs::PoseArray path = chosenTraj.getPathRbtFrame();
        geometry_msgs::Pose lastTrajPose = (path.poses.size() > 0) ? path.poses.back() : geometry_msgs::Pose();

        if (path.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawTrajectorySwitchCount] Trajectory frame_id is empty");
            return; 
        }

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
        // First, clearing topic.
        clearMarkerArrayPublisher(globalPlanPublisher);

        if (globalPlan.empty()) 
            ROS_WARN_STREAM_NAMED("Visualizer", "Goal Selector Returned Trajectory Size " << globalPlan.size() << " < 1");

        if (globalPlan.at(0).header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGlobalPlan] Trajectory frame_id is empty");
            return;
        }

        visualization_msgs::MarkerArray globalPlanMarkerArray;
        visualization_msgs::Marker globalPlanMarker;

        globalPlanMarker.header.frame_id = globalPlan.at(0).header.frame_id;
        globalPlanMarker.header.stamp = globalPlan.at(0).header.stamp;
        globalPlanMarker.ns = "globalPlan";
        globalPlanMarker.type = visualization_msgs::Marker::ARROW;
        globalPlanMarker.action = visualization_msgs::Marker::ADD;
        globalPlanMarker.scale.x = 0.1;
        globalPlanMarker.scale.y = 0.04; // 0.01;
        globalPlanMarker.scale.z = 0.0001;
        globalPlanMarker.color.a = 1;
        globalPlanMarker.color.r = 1.0;
        globalPlanMarker.color.g = 0.0;
        globalPlanMarker.color.b = 0.0;

        globalPlanMarker.lifetime = ros::Duration(0);     
        
        for (const geometry_msgs::PoseStamped & poseStamped : globalPlan) 
        {
            globalPlanMarker.id = int (globalPlanMarkerArray.markers.size());
            globalPlanMarker.pose = poseStamped.pose;
            globalPlanMarkerArray.markers.push_back(globalPlanMarker);
        }

        // geometry_msgs::PoseArray globalPlanPoseArray;
        // globalPlanPoseArray.header = globalPlan.at(0).header;
        // for (const geometry_msgs::PoseStamped & pose : globalPlan) 
            // globalPlanPoseArray.poses.push_back(pose.pose);

        globalPlanPublisher.publish(globalPlanMarkerArray);
    }

    void TrajectoryVisualizer::drawGapTrajectories(const std::vector<Trajectory> & trajs) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(gapTrajectoriesPublisher);

        if (trajs.size() == 0)
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "no trajectories to visualize");
            return;
        }
        
        visualization_msgs::MarkerArray gapTrajMarkerArray;
        visualization_msgs::Marker gapTrajMarker;

        Trajectory traj = trajs.at(0);

        if (traj.getPathRbtFrame().header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGapTrajectories] Trajectory frame_id is empty");
            return;
        }

        // The above makes this safe
        gapTrajMarker.header.frame_id = traj.getPathRbtFrame().header.frame_id;
        gapTrajMarker.header.stamp = traj.getPathRbtFrame().header.stamp;
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

        for (const Trajectory & traj : trajs) 
        {
            geometry_msgs::PoseArray path = traj.getPathRbtFrame();
            for (const geometry_msgs::Pose & pose : path.poses) 
            {
                gapTrajMarker.id = int (gapTrajMarkerArray.markers.size());
                gapTrajMarker.pose = pose;
                gapTrajMarkerArray.markers.push_back(gapTrajMarker);
            }
        }
        gapTrajectoriesPublisher.publish(gapTrajMarkerArray);
    }

    void TrajectoryVisualizer::drawGapTrajectoryPoseScores(const std::vector<Trajectory> & trajs, 
                                                           const std::vector<std::vector<float>> & trajPoseScores) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(trajPoseScoresPublisher);

        if (trajs.size() == 0)
            return;

        // if (!cfg_->gap_viz.debug_viz) return;
        visualization_msgs::MarkerArray trajPoseScoresMarkerArray;
        visualization_msgs::Marker trajPoseScoresMarker;

        // The above ensures this is safe
        trajPoseScoresMarker.ns = "trajScore";
        trajPoseScoresMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        trajPoseScoresMarker.action = visualization_msgs::Marker::ADD;
        // trajPoseScoresMarker.scale.x = 0.1;
        // trajPoseScoresMarker.scale.y = 0.1;
        trajPoseScoresMarker.scale.z = 0.3;

        trajPoseScoresMarker.color.a = 1.0;
        trajPoseScoresMarker.color.r = 0.0;
        trajPoseScoresMarker.color.g = 0.0;
        trajPoseScoresMarker.color.b = 0.0;
        trajPoseScoresMarker.lifetime = ros::Duration(0);

        // ROS_FATAL_STREAM_COND(!trajs.size() == trajPoseScores.size(), "drawGapTrajectoryPoseScores size mismatch, trajs: "
        //                                         << trajs.size() << ", trajPoseScores: " << trajPoseScores.size());

        for (int i = 0; i < trajs.size(); i++) 
        {
            geometry_msgs::PoseArray path = trajs.at(i).getPathRbtFrame();
            geometry_msgs::Pose lastTrajPose = (path.poses.size() > 0) ? path.poses.back() : geometry_msgs::Pose();
                        
            if (path.header.frame_id.empty())
            {
                ROS_WARN_STREAM_NAMED("Visualizer", "[drawGapTrajectoryPoseScores] Trajectory frame_id is empty");
                return;
            }

            trajPoseScoresMarker.header = path.header;
            trajPoseScoresMarker.id = i;
            trajPoseScoresMarker.pose.position = lastTrajPose.position;
            trajPoseScoresMarker.pose.orientation = lastTrajPose.orientation;

            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << "SCORE: " << std::accumulate(trajPoseScores.at(i).begin(), trajPoseScores.at(i).end(), float(0));

            trajPoseScoresMarker.text = stream.str();

            trajPoseScoresMarkerArray.markers.push_back(trajPoseScoresMarker);

            // ROS_FATAL_STREAM_COND(!trajs.at(i).poses.size() == trajPoseScores.at(i).size(), "drawGapTrajectoryPoseScores size mismatch," << i << "th "
            //     << trajs.at(i).poses.size() << ", trajPoseScores: " << trajPoseScores.at(i).size());
            
            /*
            for (int j = 0; j < trajs.at(i).getPathRbtFrame().poses.size(); j++) 
            {
                trajPoseScoresMarker.id = int (trajPoseScoresMarkerArray.markers.size());
                trajPoseScoresMarker.pose = trajs.at(i).getPathRbtFrame().poses.at(j);

                std::stringstream stream;
                stream << std::fixed << std::setprecision(2) << trajPoseScores.at(i).at(j);
                trajPoseScoresMarker.text = stream.str();

                trajPoseScoresMarkerArray.markers.push_back(trajPoseScoresMarker);
            }
            */
        }
        trajPoseScoresPublisher.publish(trajPoseScoresMarkerArray);
    }

    void TrajectoryVisualizer::drawUngapTrajectories(const std::vector<Trajectory> & trajs) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(ungapTrajectoriesPublisher);

        if (trajs.size() == 0)
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "no trajectories to visualize");
            return;
        }
        
        visualization_msgs::MarkerArray ungapTrajMarkerArray;
        visualization_msgs::Marker ungapTrajMarker;

        Trajectory traj = trajs.at(0);

        if (traj.getPathRbtFrame().header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGapTrajectories] Trajectory frame_id is empty");
            return;
        }

        // The above makes this safe
        ungapTrajMarker.header.frame_id = traj.getPathRbtFrame().header.frame_id;
        ungapTrajMarker.header.stamp = traj.getPathRbtFrame().header.stamp;
        ungapTrajMarker.ns = "allTraj";
        ungapTrajMarker.type = visualization_msgs::Marker::ARROW;
        ungapTrajMarker.action = visualization_msgs::Marker::ADD;
        ungapTrajMarker.scale.x = 0.1;
        ungapTrajMarker.scale.y = 0.04; // 0.01;
        ungapTrajMarker.scale.z = 0.0001;
        ungapTrajMarker.color.a = 1;
        ungapTrajMarker.color.b = 0.75;
        ungapTrajMarker.color.g = 0.75;
        ungapTrajMarker.lifetime = ros::Duration(0);

        for (const Trajectory & traj : trajs) 
        {
            geometry_msgs::PoseArray path = traj.getPathRbtFrame();
            for (const geometry_msgs::Pose & pose : path.poses) 
            {
                ungapTrajMarker.id = int (ungapTrajMarkerArray.markers.size());
                ungapTrajMarker.pose = pose;
                ungapTrajMarkerArray.markers.push_back(ungapTrajMarker);
            }
        }
        ungapTrajectoriesPublisher.publish(ungapTrajMarkerArray);
    }


    void TrajectoryVisualizer::drawRelevantGlobalPlanSnippet(const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(globalPlanSnippetPublisher);

        if (globalPlanSnippet.empty())             // Should be safe with this check
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "Goal Selector Returned Trajectory Size " << globalPlanSnippet.size() << " < 1");
            return;
        }    
        
        if (globalPlanSnippet.at(0).header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawRelevantGlobalPlanSnippet] Trajectory frame_id is empty");
            return;
        }

        visualization_msgs::MarkerArray globalPlanSnippetMarkerArray;
        visualization_msgs::Marker globalPlanSnippetMarker;

        // The above makes this safe
        globalPlanSnippetMarker.header.frame_id = globalPlanSnippet.at(0).header.frame_id;
        globalPlanSnippetMarker.header.stamp = globalPlanSnippet.at(0).header.stamp;
        globalPlanSnippetMarker.ns = "globalPlanSnippet";
        globalPlanSnippetMarker.type = visualization_msgs::Marker::ARROW;
        globalPlanSnippetMarker.action = visualization_msgs::Marker::ADD;
        globalPlanSnippetMarker.scale.x = 0.1;
        globalPlanSnippetMarker.scale.y = 0.04; // 0.01;
        globalPlanSnippetMarker.scale.z = 0.0001;
        globalPlanSnippetMarker.color.a = 1;
        globalPlanSnippetMarker.color.r = 1.0;
        globalPlanSnippetMarker.lifetime = ros::Duration(0);

        for (const geometry_msgs::PoseStamped & poseStamped : globalPlanSnippet) 
        {
            globalPlanSnippetMarker.id = int (globalPlanSnippetMarkerArray.markers.size());
            globalPlanSnippetMarker.pose = poseStamped.pose;
            globalPlanSnippetMarkerArray.markers.push_back(globalPlanSnippetMarker);
        }

        globalPlanSnippetPublisher.publish(globalPlanSnippetMarkerArray);

        // geometry_msgs::PoseArray globalPlanSnippetPoseArray;

        // globalPlanSnippetPoseArray.header = globalPlanSnippet.at(0).header;

        // for (const geometry_msgs::PoseStamped & pose : globalPlanSnippet) 
        //     globalPlanSnippetPoseArray.poses.push_back(pose.pose);

        // globalPlanSnippetPublisher.publish(globalPlanSnippetPoseArray);
    }
}