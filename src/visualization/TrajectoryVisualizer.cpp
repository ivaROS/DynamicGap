
#include <dynamic_gap/visualization/TrajectoryVisualizer.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace dynamic_gap
{
    visualization_msgs::MarkerArray buildMppiMarkers(
        const std::vector<MppiGapVisualization>& gaps,
        const std::vector<std_msgs::ColorRGBA>& gap_colors,
        const ros::Time& stamp,
        MppiMarkerKeys& previous_keys,
        std::size_t max_samples_per_gap,
        std::size_t point_stride)
    {
        visualization_msgs::MarkerArray result;
        MppiMarkerKeys current_keys;
        point_stride = std::max<std::size_t>(1, point_stride);

        // Rollouts are supplied in the planner's common sensor frame. Never mix
        // frames in one planning snapshot without a transform.
        std::string frame;
        for (const auto& gap : gaps)
        {
            if (!gap.nominal.poses.empty() && !gap.nominal.header.frame_id.empty())
                frame = gap.nominal.header.frame_id;
            if (frame.empty())
                for (const auto& sample : gap.samples)
                    if (!sample.poses.empty() && !sample.header.frame_id.empty())
                    {
                        frame = sample.header.frame_id;
                        break;
                    }
            if (!frame.empty()) break;
        }

        auto append_line = [&](const geometry_msgs::PoseArray& path,
                               const std::string& ns, int id,
                               const std_msgs::ColorRGBA& color,
                               double width, double height) {
            if (path.poses.empty() || frame.empty() || path.header.frame_id != frame)
                return false;

            visualization_msgs::Marker marker;
            marker.header.frame_id = frame;
            marker.header.stamp = stamp;
            marker.ns = ns;
            marker.id = id;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = width;
            marker.color = color;
            marker.lifetime = ros::Duration(2.0);

            for (std::size_t i = 0; i < path.poses.size(); ++i)
            {
                const auto& p = path.poses[i].position;
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                    return false;
                if (i % point_stride == 0 || i + 1 == path.poses.size())
                {
                    geometry_msgs::Point point = p;
                    point.z += height;
                    marker.points.push_back(point);
                }
            }
            current_keys.emplace(marker.ns, marker.id);
            result.markers.push_back(std::move(marker));
            return true;
        };

        for (const auto& gap : gaps)
        {
            const std::string ns = "mppi_gap_" + std::to_string(gap.gap_index) +
                "_" + std::to_string(gap.left_id) + "_" + std::to_string(gap.right_id);
            std_msgs::ColorRGBA color;
            if (!gap_colors.empty())
                color = gap_colors[static_cast<std::size_t>(std::max(0, gap.gap_index)) % gap_colors.size()];
            else
            {
                color.r = 0.2;
                color.g = 0.8;
                color.b = 1.0;
            }
            color.a = 1.0;

            const std::size_t draw_count = max_samples_per_gap == 0 ? gap.samples.size() :
                std::min(max_samples_per_gap, gap.samples.size());
            std::size_t drawn = 0;
            for (std::size_t i = 0; i < draw_count; ++i)
            {
                // Deterministically spread a requested visual subset across the
                // population; sampling and feasibility statistics are untouched.
                const std::size_t sample_index = i * gap.samples.size() / draw_count;
                const bool feasible = sample_index >= gap.sample_feasible.size() ||
                    gap.sample_feasible[sample_index];
                std_msgs::ColorRGBA sample_color = color;
                sample_color.a = feasible ? 0.4 : 0.12;
                if (append_line(gap.samples[sample_index], ns + "/samples",
                                static_cast<int>(sample_index), sample_color,
                                feasible ? 0.012 : 0.006, 0.035))
                    ++drawn;
            }

            const bool have_nominal = append_line(gap.nominal, ns + "/nominal", 0,
                                                  color, 0.065, 0.07);
            const geometry_msgs::PoseArray* label_path = have_nominal ? &gap.nominal : nullptr;
            if (!label_path)
                for (const auto& sample : gap.samples)
                    if (!sample.poses.empty() && sample.header.frame_id == frame &&
                        std::isfinite(sample.poses.back().position.x) &&
                        std::isfinite(sample.poses.back().position.y))
                    {
                        label_path = &sample;
                        break;
                    }
            if (!label_path) continue;

            visualization_msgs::Marker label;
            label.header.frame_id = frame;
            label.header.stamp = stamp;
            label.ns = ns + "/cost";
            label.id = 0;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.position = label_path->poses.back().position;
            // Optimizers can converge to nearby endpoints; keep their labels
            // readable even when the nominal paths overlap.
            label.pose.position.y += 0.5 * std::max(0, gap.gap_index);
            label.pose.position.z = 0.25;
            label.pose.orientation.w = 1.0;
            label.scale.z = 0.12;
            label.color = color;
            label.lifetime = ros::Duration(2.0);
            std::ostringstream text;
            text << "gap " << gap.gap_index << " [" << gap.left_id << "," << gap.right_id
                 << "] J=" << std::fixed << std::setprecision(2) << gap.cost
                 << "\nsamples=" << gap.total_samples << " drawn=" << drawn
                 << " feasible=" << gap.feasible_samples << " progress=" << gap.progress_samples;
            label.text = text.str();
            current_keys.emplace(label.ns, label.id);
            result.markers.push_back(std::move(label));
        }

        // A dropped gap or reduced sample budget must disappear in this same
        // message. No DELETEALL can erase another Dynamic Gap visualization.
        for (const auto& key : previous_keys)
            if (current_keys.count(key) == 0)
            {
                visualization_msgs::Marker remove;
                remove.header.frame_id = frame;
                remove.header.stamp = stamp;
                remove.ns = key.first;
                remove.id = key.second;
                remove.action = visualization_msgs::Marker::DELETE;
                result.markers.push_back(std::move(remove));
            }
        previous_keys = std::move(current_keys);
        return result;
    }

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
        gapTubeTrajectoriesPublisher = nh.advertise<visualization_msgs::MarkerArray>("candidate_tube_trajectories", 10);
        mppiTrajectoriesPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("mppi_trajectories", 1);
        ungapTrajectoriesPublisher = nh.advertise<visualization_msgs::MarkerArray>("candidate_ungap_trajectories", 10);

        // globalPlanSnippetPublisher = nh.advertise<geometry_msgs::PoseArray>("relevant_global_plan_snippet", 10);
        globalPlanSnippetPublisher = nh.advertise<visualization_msgs::MarkerArray>("relevant_global_plan_snippet", 10);
    }

    void TrajectoryVisualizer::drawMppiTrajectories(
        const std::vector<MppiGapVisualization>& gaps,
        std::size_t max_samples_per_gap,
        std::size_t point_stride)
    {
        mppiTrajectoriesPublisher_.publish(buildMppiMarkers(
            gaps, visionColors_, ros::Time::now(), mppiMarkerKeys_,
            max_samples_per_gap, point_stride));
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
            if (cfg_->mppi.enabled)
                trajMarker.pose.position.z += 0.15; // selected arrows above the sample cloud
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
            ROS_WARN_STREAM_NAMED("Visualizer", "Goal Selector Returned Trajectory Size 0");

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
            // ROS_WARN_STREAM_NAMED("Visualizer", "no trajectories to visualize");
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

    void TrajectoryVisualizer::drawGapTubeTrajectories(const std::vector<Trajectory> & trajs) 
    {
        // First, clearing topic.
        clearMarkerArrayPublisher(gapTubeTrajectoriesPublisher);

        if (trajs.size() == 0)
        {
            // ROS_WARN_STREAM_NAMED("Visualizer", "no trajectories to visualize");
            return;
        }
        
        visualization_msgs::MarkerArray gapTrajMarkerArray;
        visualization_msgs::Marker gapTrajMarker;

        Trajectory traj = trajs.at(0);

        if (traj.getPathRbtFrame().header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Visualizer", "[drawGapTubeTrajectories] Trajectory frame_id is empty");
            return;
        }

        // The above makes this safe
        gapTrajMarker.header.frame_id = traj.getPathRbtFrame().header.frame_id;
        gapTrajMarker.header.stamp = traj.getPathRbtFrame().header.stamp;
        gapTrajMarker.ns = "allTraj";
        gapTrajMarker.type = visualization_msgs::Marker::ARROW;
        gapTrajMarker.action = visualization_msgs::Marker::ADD;
        gapTrajMarker.scale.x = 0.2; // 0.1;
        gapTrajMarker.scale.y = 0.08; // 0.04; // 0.01;
        gapTrajMarker.scale.z = 0.0001;
        // gapTrajMarker.color.a = 1;
        // gapTrajMarker.color.b = 0.5;
        // gapTrajMarker.color.g = 1.0;
        
        gapTrajMarker.lifetime = ros::Duration(0);

        for (int i = 0; i < trajs.size(); i++)
        {
            Trajectory traj = trajs.at(i);
            geometry_msgs::PoseArray path = traj.getPathRbtFrame();

            gapTrajMarker.color = visionColors_[i % visionColors_.size()];

            for (const geometry_msgs::Pose & pose : path.poses) 
            {
                gapTrajMarker.id = int (gapTrajMarkerArray.markers.size());
                gapTrajMarker.pose = pose;
                gapTrajMarkerArray.markers.push_back(gapTrajMarker);
            }
        }
        gapTubeTrajectoriesPublisher.publish(gapTrajMarkerArray);
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

            float score = std::accumulate(trajPoseScores.at(i).begin(), trajPoseScores.at(i).end(), float(0)) / trajPoseScores.at(i).size();

            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << "SCORE: " << score;

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
            // ROS_WARN_STREAM_NAMED("Visualizer", "no trajectories to visualize");
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
        ungapTrajMarker.scale.x = 0.2; // 0.1;
        ungapTrajMarker.scale.y = 0.08; // 0.04; // 0.01;
        ungapTrajMarker.scale.z = 0.0001;
        ungapTrajMarker.color.a = 1;
        ungapTrajMarker.color.r = 0.50;
        ungapTrajMarker.color.b = 0.50;
        ungapTrajMarker.color.g = 0.50;
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
