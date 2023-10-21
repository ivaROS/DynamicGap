#include <dynamic_gap/Planner.h>

// #include "tf/transform_datatypes.h"
// #include <tf/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>

namespace dynamic_gap
{   
    Planner::Planner()
    {
        // Do something? maybe set names
        ros::NodeHandle nh("planner_node");
    }

    Planner::~Planner() {}

    bool Planner::initialize(const ros::NodeHandle& unh)
    {
        // ROS_INFO_STREAM("starting initialize");
        if (initialized())
        {
            ROS_WARN("DynamicGap Planner already initalized");
            return true;
        }

        // Config Setup
        cfg.loadRosParamFromNodeHandle(unh);

        // Visualization Setup
        currentTrajectoryPublisher_ = nh.advertise<geometry_msgs::PoseArray>("curr_exec_dg_traj", 1);
        staticScanPublisher_ = nh.advertise<sensor_msgs::LaserScan>("static_scan", 1);

        // TF Lookup setup
        tfListener = new tf2_ros::TransformListener(tfBuffer);
        initialized_ = true;

        gapDetector = new dynamic_gap::GapDetector(cfg);
        finder = new dynamic_gap::StaticScanSeparator(cfg);
        gapvisualizer = new dynamic_gap::GapVisualizer(nh, cfg);
        goalselector = new dynamic_gap::GoalSelector(nh, cfg);
        trajvisualizer = new dynamic_gap::TrajectoryVisualizer(nh, cfg);
        trajArbiter = new dynamic_gap::TrajectoryScorer(nh, cfg);
        gapTrajSyn = new dynamic_gap::GapTrajectoryGenerator(nh, cfg);
        goalvisualizer = new dynamic_gap::GoalVisualizer(nh, cfg);
        gapManip = new dynamic_gap::GapManipulator(nh, cfg);
        trajController = new dynamic_gap::TrajectoryController(nh, cfg);
        gapassociator = new dynamic_gap::GapAssociator(nh, cfg);
        gapFeasibilityChecker = new dynamic_gap::GapFeasibilityChecker(nh, cfg);

        // MAP FRAME ID SHOULD BE: known_map
        // ODOM FRAME ID SHOULD BE: map_static

        map2rbt.transform.rotation.w = 1;
        odom2rbt.transform.rotation.w = 1;
        rbt2odom.transform.rotation.w = 1;
        rbt_in_rbt.pose.orientation.w = 1;
        rbt_in_rbt.header.frame_id = cfg.robot_frame_id;

        log_vel_comp.set_capacity(cfg.planning.halt_size);

        current_rbt_vel = geometry_msgs::TwistStamped();
        current_rbt_acc = geometry_msgs::TwistStamped();
        
        init_val = 0;
        model_idx = &init_val;

        curr_left_model = NULL;
        curr_right_model = NULL;

        robotPoseOdomFrame_ = geometry_msgs::Pose();

        final_goal_rbt = geometry_msgs::PoseStamped();
        num_obsts = cfg.rbt.num_obsts;

        agent_odoms = std::vector<geometry_msgs::Pose>(num_obsts);
        agent_vels = std::vector<geometry_msgs::Vector3Stamped>(num_obsts);

        sensor_msgs::LaserScan tmp_scan = sensor_msgs::LaserScan();
        future_scans.push_back(tmp_scan);
        for (float t_iplus1 = cfg.traj.integrate_stept; t_iplus1 <= cfg.traj.integrate_maxt; t_iplus1 += cfg.traj.integrate_stept) 
        {
            future_scans.push_back(tmp_scan);
        }

        static_scan = sensor_msgs::LaserScan();
        // ROS_INFO_STREAM("future_scans size: " << future_scans.size());
        // ROS_INFO_STREAM("done initializing");
        switch_index = 0;

        intermediate_vels.clear();
        intermediate_accs.clear();

        hasGoal = false;

        t_last_kf_update = ros::Time::now();


        return true;
    }

    bool Planner::initialized()
    {
        return initialized_;
    }

    int Planner::get_num_obsts() {
        return num_obsts;
    }

    bool Planner::isGoalReached()
    {
        float dx = final_goal_odom.pose.position.x - robotPoseOdomFrame_.position.x;
        float dy = final_goal_odom.pose.position.y - robotPoseOdomFrame_.position.y;
        bool result = sqrt(pow(dx, 2) + pow(dy, 2)) < cfg.goal.goal_tolerance;
        
        if (result)
        {
            ROS_INFO_STREAM("[Reset] Goal Reached");
            return true;
        }

        float waydx = local_waypoint_odom.pose.position.x - robotPoseOdomFrame_.position.x;
        float waydy = local_waypoint_odom.pose.position.y - robotPoseOdomFrame_.position.y;
        bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) < cfg.goal.waypoint_tolerance;
        if (wayres) {
            ROS_INFO_STREAM("[Reset] Waypoint reached, getting new one");
        }
        return false;
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        ros::Time curr_time = msg->header.stamp;

        scan_ = msg;

        if (hasGoal)
        {
            std::vector<geometry_msgs::TwistStamped> ego_rbt_vels_copied = intermediate_vels;
            std::vector<geometry_msgs::TwistStamped> ego_rbt_accs_copied = intermediate_accs;

            t_kf_update = msg->header.stamp;

            // std::chrono::steady_clock::time_point gap_detection_start_time = std::chrono::steady_clock::now();
            // ROS_INFO_STREAM("Time elapsed before raw gaps processing: " << (ros::WallTime::now().toSec() - start_time));
            raw_gaps = gapDetector->gapDetection(msg, final_goal_rbt);
            // float gap_detection_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_detection_start_time).count() / 1.0e6;
            // ROS_INFO_STREAM("gapDetection: " << gap_detection_time << " seconds");

            if (cfg.debug.raw_gaps_debug_log) ROS_INFO_STREAM("RAW GAP ASSOCIATING");    
            rawDistMatrix_ = gapassociator->obtainDistMatrix(raw_gaps, previous_raw_gaps);
            rawAssocation_ = gapassociator->associateGaps(rawDistMatrix_);         // ASSOCIATE GAPS PASSES BY REFERENCE
            gapassociator->assignModels(rawAssocation_, rawDistMatrix_, 
                                        raw_gaps, previous_raw_gaps, 
                                        model_idx, t_kf_update,
                                        ego_rbt_vels_copied, ego_rbt_accs_copied,
                                        cfg.debug.raw_gaps_debug_log);
            
            associated_raw_gaps = update_models(raw_gaps, ego_rbt_vels_copied, 
                                                ego_rbt_accs_copied, t_kf_update,
                                                cfg.debug.raw_gaps_debug_log);
            // ROS_INFO_STREAM("Time elapsed after raw gaps processing: " << (ros::WallTime::now().toSec() - start_time));

            static_scan = finder->staticDynamicScanSeparation(associated_raw_gaps, msg, cfg.debug.static_scan_separation_debug_log);
            staticScanPublisher_.publish(static_scan);
            trajArbiter->updateStaticEgoCircle(static_scan);
            gapManip->updateStaticEgoCircle(static_scan);
            curr_agents = finder->getCurrAgents();

            // std::chrono::steady_clock::time_point gap_simplification_start_time = std::chrono::steady_clock::now();
            observed_gaps = gapDetector->gapSimplification(raw_gaps);
            // float gap_simplification_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_simplification_start_time).count() / 1.0e6;
            // ROS_INFO_STREAM("gapSimplification: " << gap_simplification_time << " seconds");

            if (cfg.debug.simplified_gaps_debug_log) ROS_INFO_STREAM("SIMPLIFIED GAP ASSOCIATING");    
            simpDistMatrix_ = gapassociator->obtainDistMatrix(observed_gaps, previous_gaps);
            simpAssociation_ = gapassociator->associateGaps(simpDistMatrix_); // must finish this and therefore change the association
            gapassociator->assignModels(simpAssociation_, simpDistMatrix_, 
                                        observed_gaps, previous_gaps, 
                                        model_idx, t_kf_update,
                                        ego_rbt_vels_copied, ego_rbt_accs_copied,
                                        cfg.debug.simplified_gaps_debug_log);
            associated_observed_gaps = update_models(observed_gaps, ego_rbt_vels_copied, 
                                                        ego_rbt_accs_copied, t_kf_update,
                                                        cfg.debug.simplified_gaps_debug_log);
            // ROS_INFO_STREAM("Time elapsed after observed gaps processing: " << (ros::WallTime::now().toSec() - start_time));

            gapvisualizer->drawGaps(associated_raw_gaps, std::string("raw"));
            // gapvisualizer->drawGaps(associated_observed_gaps, std::string("simp"));
            gapvisualizer->drawGapsModels(associated_raw_gaps);
        }

        geometry_msgs::PoseStamped local_goal;
        {
            goalselector->updateEgoCircle(scan_);
            goalselector->updateLocalGoal(map2rbt);
            local_goal = goalselector->transformLocalGoalToOdomFrame(rbt2odom);
            goalvisualizer->localGoal(local_goal);
        }
        // ROS_INFO_STREAM("Time elapsed after updating goal selector: " << (ros::WallTime::now().toSec() - start_time));

        trajArbiter->updateEgoCircle(scan_);
        trajArbiter->updateLocalGoal(local_goal, odom2rbt);

        // ROS_INFO_STREAM("Time elapsed after updating arbiter: " << (ros::WallTime::now().toSec() - start_time));

        gapManip->updateEgoCircle(scan_);
        trajController->updateEgoCircle(scan_);
        gapFeasibilityChecker->updateEgoCircle(scan_);
        // ROS_INFO_STREAM("Time elapsed after updating rest: " << (ros::WallTime::now().toSec() - start_time));

        // ROS_INFO_STREAM("laserscan time elapsed: " << ros::WallTime::now().toSec() - start_time);

        previous_raw_gaps = associated_raw_gaps;
        previous_gaps = associated_observed_gaps;
        t_last_kf_update = t_kf_update;
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    std::vector<dynamic_gap::Gap> Planner::update_models(const std::vector<dynamic_gap::Gap> _observed_gaps, 
                                                         const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied,
                                                         const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied,
                                                         const ros::Time & t_kf_update,
                                                         bool print) 
    {
        if (print) ROS_INFO_STREAM("[update_models()]");
        std::vector<dynamic_gap::Gap> associated_observed_gaps = _observed_gaps;
        
        // double start_time = ros::WallTime::now().toSec();
        for (int i = 0; i < 2*associated_observed_gaps.size(); i++) 
        {
            if (print) ROS_INFO_STREAM("    update gap model " << i << " of " << 2*associated_observed_gaps.size());
            update_model(i, associated_observed_gaps, ego_rbt_vels_copied, ego_rbt_accs_copied, t_kf_update, print);
            if (print) ROS_INFO_STREAM("");
		}

        //ROS_INFO_STREAM("update_models time elapsed: " << ros::WallTime::now().toSec() - start_time);
        return associated_observed_gaps;
    }

    void Planner::update_model(int i, std::vector<dynamic_gap::Gap>& _observed_gaps, 
                               const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied,
                               const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied,
                               const ros::Time & t_kf_update,
                               bool print) 
    {
		dynamic_gap::Gap g = _observed_gaps[int(std::floor(i / 2.0))];
 
        float beta_tilde, range_tilde;
		if (i % 2 == 0) 
        {
			beta_tilde = idx2theta(g.RIdx()); // float(g.RIdx() - g.half_scan) / g.half_scan * M_PI;
            range_tilde = g.RDist();
		} else 
        {
            beta_tilde = idx2theta(g.LIdx()); // float(g.LIdx() - g.half_scan) / g.half_scan * M_PI;
            range_tilde = g.LDist();
		}

		Eigen::Matrix<float, 2, 1> laserscan_measurement(range_tilde, beta_tilde);

        if (i % 2 == 0) 
        {
            //std::cout << "entering left model update" << std::endl;
            try {
                g.right_model->update(laserscan_measurement, 
                                        ego_rbt_vels_copied, ego_rbt_accs_copied, 
                                        print, agent_odoms, 
                                        agent_vels,
                                        t_kf_update);
            } catch (...) {
                ROS_INFO_STREAM("kf_update_loop fails");
            }
        } else {
            //std::cout << "entering right model update" << std::endl;
            try {
                g.left_model->update(laserscan_measurement, 
                                        ego_rbt_vels_copied, ego_rbt_accs_copied, 
                                        print, agent_odoms, 
                                        agent_vels,
                                        t_kf_update);
            } catch (...) {
                ROS_INFO_STREAM("kf_update_loop fails");
            }
        }
    }

    void Planner::jointPoseAccCB(const nav_msgs::Odometry::ConstPtr &odom_msg, 
                                 const geometry_msgs::TwistStamped::ConstPtr &accel_msg)
    {
        // ROS_INFO_STREAM("joint pose acc cb");

        // ROS_INFO_STREAM("accel time stamp: " << accel_msg->header.stamp.toSec());
        current_rbt_acc = *accel_msg;
        intermediate_accs.push_back(current_rbt_acc);

        // deleting old sensor measurements already used in an update
        for (int i = 0; i < intermediate_accs.size(); i++)
        {
            if (intermediate_accs[i].header.stamp <= t_last_kf_update)
            {
                intermediate_accs.erase(intermediate_accs.begin() + i);
                i--;
            }
        }    

        // ROS_INFO_STREAM("odom time stamp: " << odom_msg->header.stamp.toSec());

        // ROS_INFO_STREAM("acc - odom time difference: " << (accel_msg->header.stamp - odom_msg->header.stamp).toSec());

        updateTF();

        // Transform the msg to odom frame
        if(odom_msg->header.frame_id != cfg.odom_frame_id)
        {
            //std::cout << "odom msg is not in odom frame" << std::endl;
            geometry_msgs::TransformStamped robot_pose_odom_trans = tfBuffer.lookupTransform(cfg.odom_frame_id, odom_msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = odom_msg->header;
            in_pose.pose = odom_msg->pose.pose;

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;

            tf2::doTransform(in_pose, out_pose, robot_pose_odom_trans);
            robotPoseOdomFrame_ = out_pose.pose;
        }
        else
        {
            robotPoseOdomFrame_ = odom_msg->pose.pose;
        }

        //--------------- VELOCITY -------------------//
        // velocity always comes in wrt robot frame in STDR
        geometry_msgs::TwistStamped ego_rbt_vel;
        ego_rbt_vel.header = odom_msg->header;
        ego_rbt_vel.header.frame_id = accel_msg->header.frame_id;
        ego_rbt_vel.twist = odom_msg->twist.twist;

        current_rbt_vel = ego_rbt_vel;
        intermediate_vels.push_back(current_rbt_vel);

        // deleting old sensor measurements already used in an update
        for (int i = 0; i < intermediate_vels.size(); i++)
        {
            if (intermediate_vels[i].header.stamp <= t_last_kf_update)
            {
                intermediate_vels.erase(intermediate_vels.begin() + i);
                i--;
            }
        }

    }
    
    void Planner::agentOdomCB(const nav_msgs::Odometry::ConstPtr& msg) 
    {
        std::string robot_namespace = msg->child_frame_id;
        // ROS_INFO_STREAM("robot_namespace: " << robot_namespace);
        robot_namespace.erase(0,5); // removing "robot"
        char *robot_name_char = strdup(robot_namespace.c_str());
        int robot_id = std::atoi(robot_name_char);
        // ROS_INFO_STREAM("robot_id: " << robot_id);
        // I need BOTH odom and vel in robot2 frame
        //std::cout << "odom msg is not in odom frame" << std::endl;
        try 
        {
            // transforming Odometry message from map_static to robotN
            geometry_msgs::TransformStamped agent_to_robot_odom_trans = tfBuffer.lookupTransform(cfg.robot_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;
            // ROS_INFO_STREAM("updating inpose " << robot_namespace << " to: (" << in_pose.pose.position.x << ", " << in_pose.pose.position.y << ")");

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;
            tf2::doTransform(in_pose, out_pose, agent_to_robot_odom_trans);
            
            // ROS_INFO_STREAM("updating " << robot_namespace << " odom from " << agent_odom_vects[robot_id][0] << ", " << agent_odom_vects[robot_id][1] << " to " << odom_vect[0] << ", " << odom_vect[1]);
            agent_odoms[robot_id] = out_pose.pose;
        } catch (tf2::TransformException &ex) {
            ROS_INFO_STREAM("Odometry transform failed for " << robot_namespace);
        }
        
        try 
        {
            std::string source_frame = msg->child_frame_id; 
            // std::cout << "in agentOdomCB" << std::endl;
            // std::cout << "transforming from " << source_frame << " to " << cfg.robot_frame_id << std::endl;
            geometry_msgs::TransformStamped agent_to_robot_trans = tfBuffer.lookupTransform(cfg.robot_frame_id, source_frame, ros::Time(0));
            geometry_msgs::Vector3Stamped in_vel, out_vel;
            in_vel.header = msg->header;
            in_vel.header.frame_id = source_frame;
            in_vel.vector = msg->twist.twist.linear;
            // std::cout << "incoming vector: " << in_vel.vector.x << ", " << in_vel.vector.y << std::endl;
            tf2::doTransform(in_vel, out_vel, agent_to_robot_trans);
            // std::cout << "outcoming vector: " << out_vel.vector.x << ", " << out_vel.vector.y << std::endl;

            agent_vels[robot_id] = out_vel;
        } catch (tf2::TransformException &ex) {
            ROS_INFO_STREAM("Velocity transform failed for " << robot_namespace);
        }            
    }

    bool Planner::setGoal(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (plan.size() == 0) return true;
        final_goal_odom = *std::prev(plan.end());
        tf2::doTransform(final_goal_odom, final_goal_odom, map2odom);
        tf2::doTransform(final_goal_odom, final_goal_rbt, odom2rbt);
        // Store New Global Plan to Goal Selector
        goalselector->setGoal(plan);
        
        // Obtaining Local Goal by using global plan
        goalselector->updateLocalGoal(map2rbt);
        // return local goal (odom) frame
        auto new_local_waypoint = goalselector->transformLocalGoalToOdomFrame(rbt2odom);

        {
            // Plan New
            float waydx = local_waypoint_odom.pose.position.x - new_local_waypoint.pose.position.x;
            float waydy = local_waypoint_odom.pose.position.y - new_local_waypoint.pose.position.y;
            bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) > cfg.goal.waypoint_tolerance;
            if (wayres) {
                local_waypoint_odom = new_local_waypoint;
            }
        }

        // Set new local goal to trajectory arbiter
        trajArbiter->updateLocalGoal(local_waypoint_odom, odom2rbt);

        // Visualization only
        std::vector<geometry_msgs::PoseStamped> traj = goalselector->getRelevantGlobalPlan(map2rbt);
        trajvisualizer->drawRelevantGlobalPlanSnippet(traj);
        // trajvisualizer->drawEntireGlobalPlan(goalselector->getOdomGlobalPlan());

        hasGoal = true;
        return true;
    }

    void Planner::updateTF()
    {
        try 
        {
            map2rbt  = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.map_frame_id, ros::Time(0));
            odom2rbt = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.odom_frame_id, ros::Time(0));
            rbt2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.robot_frame_id, ros::Time(0));
            cam2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.sensor_frame_id, ros::Time(0));
            map2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2cam = tfBuffer.lookupTransform(cfg.sensor_frame_id, cfg.robot_frame_id, ros::Time(0));

            tf2::doTransform(rbt_in_rbt, rbt_in_cam, rbt2cam);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            return;
        }
    }

    std::vector<dynamic_gap::Gap> Planner::gapManipulate(const std::vector<dynamic_gap::Gap> & _observed_gaps) 
    {
        if (cfg.debug.manipulation_debug_log) ROS_INFO_STREAM("[gapManipulate()]");

        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<dynamic_gap::Gap> manip_set = _observed_gaps;
        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;

        for (size_t i = 0; i < manip_set.size(); i++)
        {
            if (cfg.debug.manipulation_debug_log) ROS_INFO_STREAM("    manipulating initial gap " << i);
            // MANIPULATE POINTS AT T=0
            manip_set.at(i).initManipIndices();
            
            // gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true);
            gapManip->convertRadialGap(manip_set.at(i), true); 
            gapManip->inflateGapSides(manip_set.at(i), true);
            gapManip->radialExtendGap(manip_set.at(i), true);
            gapManip->setGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true);
            
            
            // MANIPULATE POINTS AT T=1
            if (cfg.debug.manipulation_debug_log) ROS_INFO_STREAM("    manipulating terminal gap " << i);
            gapManip->updateDynamicEgoCircle(manip_set.at(i), future_scans);
            if ((!manip_set.at(i).gap_crossed && !manip_set.at(i).gap_closed) || (manip_set.at(i).gap_crossed_behind)) 
            {
                // gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), false);
                gapManip->convertRadialGap(manip_set.at(i), false);
            }
            gapManip->inflateGapSides(manip_set.at(i), false);
            gapManip->radialExtendGap(manip_set.at(i), false);
            gapManip->setTerminalGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal());
        }

        return manip_set;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<float>> Planner::initialTrajGen(std::vector<dynamic_gap::Gap>& vec, 
                                                            std::vector<geometry_msgs::PoseArray>& res, 
                                                            std::vector<std::vector<float>>& res_time_traj) 
    {
        if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("[initialTrajGen()]");
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<geometry_msgs::PoseArray> ret_traj(vec.size());
        std::vector<std::vector<float>> ret_time_traj(vec.size());
        std::vector<std::vector<float>> ret_traj_scores(vec.size());
        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam; // lc as local copy

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;
        try {
            for (size_t i = 0; i < vec.size(); i++) 
            {
                if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    generating traj for gap: " << i);
                // std::cout << "starting generate trajectory with rbt_in_cam_lc: " << rbt_in_cam_lc.pose.position.x << ", " << rbt_in_cam_lc.pose.position.y << std::endl;
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                std::tuple<geometry_msgs::PoseArray, std::vector<float>> return_tuple;
                
                // TRAJECTORY GENERATED IN RBT FRAME
                bool run_g2g = true; // (vec.at(i).goal.goalwithin || vec.at(i).artificial);
                if (run_g2g) 
                {
                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        running g2g");
                    std::tuple<geometry_msgs::PoseArray, std::vector<float>> g2g_tuple;
                    g2g_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, run_g2g);
                    g2g_tuple = gapTrajSyn->forwardPassTrajectory(g2g_tuple);
                    std::vector<float> g2g_score_vec = trajArbiter->scoreTrajectory(std::get<0>(g2g_tuple), std::get<1>(g2g_tuple), curr_raw_gaps, 
                                                                                     agent_odoms, agent_vels, future_scans, false, false);
                    float g2g_score = std::accumulate(g2g_score_vec.begin(), g2g_score_vec.end(), float(0));
                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        g2g_score: " << g2g_score);

                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        running ahpf");
                    std::tuple<geometry_msgs::PoseArray, std::vector<float>> ahpf_tuple;
                    ahpf_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, !run_g2g);
                    ahpf_tuple = gapTrajSyn->forwardPassTrajectory(ahpf_tuple);
                    std::vector<float> ahpf_score_vec = trajArbiter->scoreTrajectory(std::get<0>(ahpf_tuple), std::get<1>(ahpf_tuple), curr_raw_gaps, 
                                                                                        agent_odoms, agent_vels, future_scans, false, false);
                    float ahpf_score = std::accumulate(ahpf_score_vec.begin(), ahpf_score_vec.end(), float(0));
                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        ahpf_score: " << ahpf_score);

                    return_tuple = (g2g_score > ahpf_score) ? g2g_tuple : ahpf_tuple;
                    ret_traj_scores.at(i) = (g2g_score > ahpf_score) ? g2g_score_vec : ahpf_score_vec;
                } else {
                    return_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, run_g2g);
                    return_tuple = gapTrajSyn->forwardPassTrajectory(return_tuple);

                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    scoring trajectory for gap: " << i);
                    ret_traj_scores.at(i) = trajArbiter->scoreTrajectory(std::get<0>(return_tuple), std::get<1>(return_tuple), curr_raw_gaps, 
                                                                         agent_odoms, agent_vels, future_scans, false, false);  
                    // ROS_INFO_STREAM("done with scoreTrajectory");
                }

                // TRAJECTORY TRANSFORMED BACK TO ODOM FRAME
                ret_traj.at(i) = gapTrajSyn->transformBackTrajectory(std::get<0>(return_tuple), cam2odom);
                ret_time_traj.at(i) = std::get<1>(return_tuple);
            }

        } catch (...) {
            ROS_FATAL_STREAM("initialTrajGen");
        }

        trajvisualizer->pubAllScore(ret_traj, ret_traj_scores);
        trajvisualizer->pubAllTraj(ret_traj);
        res = ret_traj;
        res_time_traj = ret_time_traj;
        return ret_traj_scores;
    }

    int Planner::pickTraj(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<float>> score) 
    {
        if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("[pickTraj()]");
        // ROS_INFO_STREAM_NAMED("pg_trajCount", "pg_trajCount, " << prr.size());
        if (prr.size() == 0) {
            ROS_WARN_STREAM("No traj synthesized");
            return -1;
        }

        if (prr.size() != score.size()) {
            ROS_FATAL_STREAM("pickTraj size mismatch: prr = " << prr.size() << " != score =" << score.size());
            return -1;
        }

        // poses here are in odom frame 
        std::vector<float> result_score(prr.size());
        int counts;
        try {
            // if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < result_score.size(); i++) {
                // ROS_WARN_STREAM("prr(" << i << "): size " << prr.at(i).poses.size());
                counts = std::min(cfg.planning.num_feasi_check, int(score.at(i).size()));

                result_score.at(i) = std::accumulate(score.at(i).begin(), score.at(i).begin() + counts, float(0));
                result_score.at(i) = prr.at(i).poses.size() == 0 ? -std::numeric_limits<float>::infinity() : result_score.at(i);
                if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    for gap " << i << " (length: " << prr.at(i).poses.size() << "), returning score of " << result_score.at(i));
                /*
                if (result_score.at(i) == -std::numeric_limits<float>::infinity()) {
                    for (size_t j = 0; j < counts; j++) {
                        if (score.at(i).at(j) == -std::numeric_limits<float>::infinity()) {
                            std::cout << "-inf score at idx " << j << " of " << counts << std::endl;
                        }
                    }
                }
                */
            }
        } catch (...) {
            ROS_FATAL_STREAM("pickTraj");
        }

        auto iter = std::max_element(result_score.begin(), result_score.end());
        int idx = std::distance(result_score.begin(), iter);

        if (result_score.at(idx) == -std::numeric_limits<float>::infinity()) {
            
            ROS_INFO_STREAM("    all -infinity");
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (auto val : result_score) {
                ROS_INFO_STREAM("Score: " << val);
            }
            ROS_INFO_STREAM("------------------");
        }

        if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    picking gap: " << idx);
        
        return idx;
    }

    geometry_msgs::PoseArray Planner::changeTrajectoryHelper(dynamic_gap::Gap incoming_gap, geometry_msgs::PoseArray incoming, std::vector<float> time_arr, bool switching_to_empty) {
        
        if (switching_to_empty) {
            geometry_msgs::PoseArray empty_traj = geometry_msgs::PoseArray();
            empty_traj.header = incoming.header;
            std::vector<float> empty_time_arr;
            setCurrentTraj(empty_traj);
            setCurrentTimeArr(empty_time_arr);
            setCurrentLeftModel(NULL);
            setCurrentRightModel(NULL);
            setCurrentGapPeakVelocities(0.0, 0.0);
            currentTrajectoryPublisher_.publish(empty_traj);
            trajvisualizer->drawTrajectorySwitchCount(switch_index++, empty_traj);
            return empty_traj;
        } else {
            setCurrentTraj(incoming);
            setCurrentTimeArr(time_arr);
            setCurrentLeftModel(incoming_gap.left_model);
            setCurrentRightModel(incoming_gap.right_model);
            setCurrentGapPeakVelocities(incoming_gap.peak_velocity_x, incoming_gap.peak_velocity_y);
            currentTrajectoryPublisher_.publish(incoming);          
            trajvisualizer->drawTrajectorySwitchCount(switch_index++, incoming);

            return incoming;  
        }                       
    }

    geometry_msgs::PoseArray Planner::compareToOldTraj(geometry_msgs::PoseArray incoming, 
                                                       dynamic_gap::Gap incoming_gap, 
                                                       std::vector<dynamic_gap::Gap> feasible_gaps, 
                                                       std::vector<float> time_arr,
                                                       bool curr_exec_gap_assoc,
                                                       bool curr_exec_gap_feas) 
    {
        if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("[compareToOldTraj()]");
        boost::mutex::scoped_lock gapset(gapset_mutex);
        auto curr_traj = getCurrentTraj();
        auto curr_time_arr = getCurrentTimeArr();

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;

        try {
            float curr_time = ros::WallTime::now().toSec();
            
            // FORCING OFF CURRENT TRAJ IF NO LONGER FEASIBLE
            // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapIndex() << ", current right gap index: " << getCurrentRightGapIndex());
            // bool curr_gap_feasible = (curr_exec_gap_assoc && curr_exec_gap_feas);
            bool curr_gap_feasible = true;
            for (dynamic_gap::Gap g : feasible_gaps) 
            {
                // ROS_INFO_STREAM("feasible left gap index: " << g.left_model->get_index() << ", feasible right gap index: " << g.right_model->get_index());
                if (g.left_model->get_index() == getCurrentLeftGapIndex() &&
                    g.right_model->get_index() == getCurrentRightGapIndex()) {
                    setCurrentGapPeakVelocities(g.peak_velocity_x, g.peak_velocity_y);
                    break;
                }
            }

            // std::cout << "current traj length: " << curr_traj.poses.size() << std::endl;
            // ROS_INFO_STREAM("current gap indices: " << getCurrentLeftGapIndex() << ", " << getCurrentRightGapIndex());
            //std::cout << "current time length: " << curr_time_arr.size() << std::endl;
            //std::cout << "incoming traj length: " << incoming.poses.size() << std::endl;
            //std::cout << "incoming time length: " << time_arr.size() << std::endl;
            
            // Both Args are in Odom frame
            auto incom_rbt = gapTrajSyn->transformBackTrajectory(incoming, odom2rbt);
            incom_rbt.header.frame_id = cfg.robot_frame_id;
            // why do we have to rescore here?

            if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    scoring incoming trajectory");
            auto incom_score = trajArbiter->scoreTrajectory(incom_rbt, time_arr, curr_raw_gaps, 
                                                            agent_odoms, agent_vels, future_scans, false, true);
            // int counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));

            int counts = std::min(cfg.planning.num_feasi_check, (int) incom_score.size());
            auto incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, float(0));

            if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    incoming trajectory subscore: " << incom_subscore);
            bool curr_traj_length_zero = curr_traj.poses.size() == 0;
            bool curr_gap_not_feasible = !curr_gap_feasible;

            // CURRENT TRAJECTORY LENGTH ZERO OR IS NOT FEASIBLE
            if (curr_traj_length_zero || curr_gap_not_feasible) {
                bool valid_incoming_traj = incoming.poses.size() > 0 && incom_subscore > -std::numeric_limits<float>::infinity();
                
                std::string curr_traj_status = (curr_traj_length_zero) ? "curr traj length 0" : "curr traj is not feasible";
                if (!curr_exec_gap_assoc) {
                    curr_traj_status = "curr exec gap is not associated (left: " + std::to_string(getCurrentLeftGapIndex()) + ", right: " + std::to_string(getCurrentRightGapIndex()) + ")";
                } else if (!curr_exec_gap_feas) {
                    curr_traj_status = "curr exec gap is deemed infeasible";
                }
                
                if (valid_incoming_traj) 
                {
                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << switch_index << " to incoming: " << curr_traj_status << ", incoming score finite");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
                } else  {
                    std::string incoming_traj_status = (incoming.poses.size() > 0) ? "incoming traj length 0" : "incoming score infinite";

                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << switch_index <<  " to empty: " << curr_traj_status << ", " << incoming_traj_status);
                    
                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, true);
                }
            } 

            // CURRENT TRAJECTORY HAS ENDED
            auto curr_rbt = gapTrajSyn->transformBackTrajectory(curr_traj, odom2rbt);
            curr_rbt.header.frame_id = cfg.robot_frame_id;
            int start_position = egoTrajPosition(curr_rbt);
            geometry_msgs::PoseArray reduced_curr_rbt = curr_rbt;
            std::vector<float> reduced_curr_time_arr = curr_time_arr;
            reduced_curr_rbt.poses = std::vector<geometry_msgs::Pose>(curr_rbt.poses.begin() + start_position, curr_rbt.poses.end());
            reduced_curr_time_arr = std::vector<float>(curr_time_arr.begin() + start_position, curr_time_arr.end());
            if (reduced_curr_rbt.poses.size() < 2) {
                if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << switch_index <<  " to incoming: old traj length less than 2");

                return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
            }

            counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incoming.poses.size(), reduced_curr_rbt.poses.size()));
            incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, float(0));
            if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    re-scored incoming trajectory subscore: " << incom_subscore);
            if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    scoring current trajectory");            
            auto curr_score = trajArbiter->scoreTrajectory(reduced_curr_rbt, reduced_curr_time_arr, curr_raw_gaps, 
                                                           agent_odoms, agent_vels, future_scans, false, false);
            auto curr_subscore = std::accumulate(curr_score.begin(), curr_score.begin() + counts, float(0));
            if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("    current trajectory subscore: " << curr_subscore);

            std::vector<std::vector<float>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incom_score;
            ret_traj_scores.at(1) = curr_score;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incom_rbt;
            viz_traj.at(1) = reduced_curr_rbt;
            trajvisualizer->pubAllScore(viz_traj, ret_traj_scores);

            if (curr_subscore == -std::numeric_limits<float>::infinity()) 
            {
                if (incom_subscore == -std::numeric_limits<float>::infinity()) 
                {
                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << switch_index <<  " to empty: both -infinity");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, true);
                } else {
                    if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory change " << switch_index << " to incoming: swapping trajectory due to collision");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
                }
            }

            /*
            if (incom_subscore > curr_subscore) {
                if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: higher score");
                changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
            }
            */
          
            if (cfg.debug.traj_debug_log) ROS_INFO_STREAM("        trajectory maintain");
            currentTrajectoryPublisher_.publish(curr_traj);
        } catch (...) {
            ROS_FATAL_STREAM("compareToOldTraj");
        }
        return curr_traj;
    }

    int Planner::egoTrajPosition(const geometry_msgs::PoseArray & curr) 
    {
        std::vector<float> pose_diff(curr.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            pose_diff[i] = sqrt(pow(curr.poses.at(i).position.x, 2) + 
                                pow(curr.poses.at(i).position.y, 2));
        }

        auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
        int closest_pose = std::distance(pose_diff.begin(), min_element_iter) + 1;
        return std::min(closest_pose, int(curr.poses.size() - 1));
    }

    void Planner::setCurrentRightModel(dynamic_gap::Estimator * _right_model) {
        curr_right_model = _right_model;
    }

    void Planner::setCurrentLeftModel(dynamic_gap::Estimator * _left_model) {
        curr_left_model = _left_model;
    }

    void Planner::setCurrentGapPeakVelocities(float _peak_velocity_x, float _peak_velocity_y) {
        curr_peak_velocity_x = _peak_velocity_x;
        curr_peak_velocity_y = _peak_velocity_y;
    }


    int Planner::getCurrentRightGapIndex() {
        // std::cout << "get current left" << std::endl;
        if (curr_right_model != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return curr_right_model->get_index();
        } else {
            // std::cout << "model is null" << std::endl;
            return -1;
        }
    }
    
    int Planner::getCurrentLeftGapIndex() {
        // std::cout << "get current right" << std::endl;
        if (curr_left_model != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return curr_left_model->get_index();
        } else {
            // std::cout << "model is null" << std::endl;
            return -1;
        }    
    }

    void Planner::setCurrentTraj(geometry_msgs::PoseArray curr_traj) {
        curr_executing_traj = curr_traj;
        return;
    }

    geometry_msgs::PoseArray Planner::getCurrentTraj() {
        return curr_executing_traj;
    }

    void Planner::setCurrentTimeArr(std::vector<float> curr_time_arr) {
        curr_executing_time_arr = curr_time_arr;
        return;
    }
    
    std::vector<float> Planner::getCurrentTimeArr() {
        return curr_executing_time_arr;
    }

    void Planner::reset()
    {
        observed_gaps.clear();
        setCurrentTraj(geometry_msgs::PoseArray());
        current_rbt_vel = geometry_msgs::TwistStamped();
        current_rbt_acc = geometry_msgs::TwistStamped();
        ROS_INFO_STREAM("log_vel_comp size: " << log_vel_comp.size());
        log_vel_comp.clear();
        ROS_INFO_STREAM("log_vel_comp size after clear: " << log_vel_comp.size() << ", is full: " << log_vel_comp.capacity());
        return;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(geometry_msgs::PoseArray traj) 
    {
        if (cfg.debug.control_debug_log) ROS_INFO_STREAM("[ctrlGeneration()]");
        geometry_msgs::Twist raw_cmd_vel = geometry_msgs::Twist();

        if (cfg.man.man_ctrl) { // MANUAL CONTROL
            raw_cmd_vel = trajController->manualControlLaw();
        } else if (traj.poses.size() < 2) { // OBSTACLE AVOIDANCE CONTROL
            sensor_msgs::LaserScan stored_scan_msgs;

            stored_scan_msgs = *scan_.get();

            ROS_INFO_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 2");
            raw_cmd_vel = trajController->obstacleAvoidanceControlLaw(stored_scan_msgs);
            return raw_cmd_vel;
        } else { // FEEDBACK CONTROL
            // Know Current Pose
            geometry_msgs::PoseStamped curr_pose_local;
            curr_pose_local.header.frame_id = cfg.robot_frame_id;
            curr_pose_local.pose.orientation.w = 1;
            geometry_msgs::PoseStamped curr_pose_odom;
            curr_pose_odom.header.frame_id = cfg.odom_frame_id;
            tf2::doTransform(curr_pose_local, curr_pose_odom, rbt2odom);
            geometry_msgs::Pose curr_pose = curr_pose_odom.pose;

            // obtain current robot pose in odom frame

            // traj in odom frame here
            // returns a TrajPlan (poses and velocities, velocities are zero here)
            dynamic_gap::TrajPlan orig_ref = trajController->trajGen(traj);
            
            // get point along trajectory to target/move towards
            ctrl_idx = trajController->targetPoseIdx(curr_pose, orig_ref);
            nav_msgs::Odometry ctrl_target_pose;
            ctrl_target_pose.header = orig_ref.header;
            ctrl_target_pose.pose.pose = orig_ref.poses.at(ctrl_idx);
            ctrl_target_pose.twist.twist = orig_ref.twist.at(ctrl_idx);
            
            geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;
            // sensor_msgs::LaserScan static_scan = *static_scan_ptr.get();
            
            raw_cmd_vel = trajController->controlLaw(curr_pose, ctrl_target_pose, 
                                                    static_scan, curr_peak_velocity_x, curr_peak_velocity_y);
        }
        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;

        geometry_msgs::Twist cmd_vel = trajController->processCmdVel(raw_cmd_vel,
                        static_scan, rbt_in_cam_lc, 
                        curr_right_model, curr_left_model,
                        current_rbt_vel, current_rbt_acc); 

        return cmd_vel;
    }

    /*
    void Planner::rcfgCallback(dynamic_gap::dgConfig &config, uint32_t level)
    {
        cfg.reconfigure(config);
        
        // set_capacity destroys everything if different from original size, 
        // resize only if the new size is greater
        log_vel_comp.clear();
        log_vel_comp.set_capacity(cfg.planning.halt_size);
    }
    */

    std::vector<dynamic_gap::Gap> Planner::gapSetFeasibilityCheck(bool & curr_exec_gap_assoc, 
                                                                  bool & curr_exec_gap_feas) 
    {
        if (cfg.debug.feasibility_debug_log) ROS_INFO_STREAM("[gapSetFeasibilityCheck()]");

        boost::mutex::scoped_lock gapset(gapset_mutex);
        //std::cout << "PULLING MODELS TO ACT ON" << std::endl;
        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;
        std::vector<dynamic_gap::Gap> curr_observed_gaps = associated_observed_gaps;
        
        std::vector<dynamic_gap::Gap> prev_raw_gaps = previous_raw_gaps;
        std::vector<dynamic_gap::Gap> prev_observed_gaps = previous_gaps;  

        //std::cout << "curr_raw_gaps size: " << curr_raw_gaps.size() << std::endl;
        //std::cout << "curr_observed_gaps size: " << curr_observed_gaps.size() << std::endl;

        //std::cout << "prev_raw_gaps size: " << prev_raw_gaps.size() << std::endl;
        //std::cout << "prev_observed_gaps size: " << prev_observed_gaps.size() << std::endl;
        //std::cout << "current robot velocity. Linear: " << current_rbt_vel.linear.x << ", " << current_rbt_vel.linear.y << ", angular: " << current_rbt_vel.angular.z << std::endl;
        //std::cout << "current raw gaps:" << std::endl;
        //printGapModels(curr_raw_gaps);

        //std::cout << "pulled current simplified associations:" << std::endl;
        //printGapAssociations(curr_observed_gaps, prev_observed_gaps, _simpAssociation_);
        
        /*
        ROS_INFO_STREAM("current raw gaps:");
        printGapModels(curr_raw_gaps);
        */
        
        int curr_left_idx = getCurrentLeftGapIndex();
        int curr_right_idx = getCurrentRightGapIndex();

        if (cfg.debug.feasibility_debug_log) 
        {
            ROS_INFO_STREAM("    current simplified gaps:");
            printGapModels(curr_observed_gaps);
            ROS_INFO_STREAM("    current left/right indices: " << curr_left_idx << ", " << curr_right_idx);
        }
        curr_exec_gap_assoc = false;
        curr_exec_gap_feas = false;

        bool gap_i_feasible;
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        for (size_t i = 0; i < curr_observed_gaps.size(); i++) 
        {
            // obtain crossing point

            if (cfg.debug.feasibility_debug_log) ROS_INFO_STREAM("    feasibility check for gap " << i);
            gap_i_feasible = gapFeasibilityChecker->indivGapFeasibilityCheck(curr_observed_gaps.at(i));
            gap_i_feasible = true;

            if (gap_i_feasible) {
                curr_observed_gaps.at(i).addTerminalRightInformation();
                feasible_gap_set.push_back(curr_observed_gaps.at(i));
                // ROS_INFO_STREAM("Pushing back gap with peak velocity of : " << curr_observed_gaps.at(i).peak_velocity_x << ", " << curr_observed_gaps.at(i).peak_velocity_y);
            }

            if (curr_observed_gaps.at(i).left_model->get_index() == curr_left_idx && 
                curr_observed_gaps.at(i).right_model->get_index() == curr_right_idx) {
                curr_exec_gap_assoc = true;
                curr_exec_gap_feas = true;
            }
        }

        /*
        if (gap_associated) {
            ROS_INFO_STREAM("currently executing gap associated");
        } else {
            ROS_INFO_STREAM("currently executing gap NOT associated");
        }
        */

        return feasible_gap_set;
    }

    void Planner::getFutureScans(std::vector<geometry_msgs::Pose> _agent_odoms,
                                 std::vector<geometry_msgs::Vector3Stamped> _agent_vels,
                                 bool print) 
    {
        // boost::mutex::scoped_lock gapset(gapset_mutex);
        if (print) ROS_INFO_STREAM("[getFutureScans()]");
        sensor_msgs::LaserScan stored_scan = *scan_.get();
        sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();
        dynamic_laser_scan.header = stored_scan.header;
        dynamic_laser_scan.angle_min = stored_scan.angle_min;
        dynamic_laser_scan.angle_max = stored_scan.angle_max;
        dynamic_laser_scan.angle_increment = stored_scan.angle_increment;
        dynamic_laser_scan.time_increment = stored_scan.time_increment;
        dynamic_laser_scan.scan_time = stored_scan.scan_time;
        dynamic_laser_scan.range_min = stored_scan.range_min;
        dynamic_laser_scan.range_max = stored_scan.range_max;
        dynamic_laser_scan.ranges = stored_scan.ranges;

        float t_i = 0.0;
        future_scans[0] = dynamic_laser_scan; // at t = 0.0

        std::vector<geometry_msgs::Pose> agent_odoms_lc = _agent_odoms;
        std::vector<geometry_msgs::Vector3Stamped> agent_vels_lc = _agent_vels;
        std::vector<Eigen::Matrix<float, 4, 1> > curr_agents_lc;
        
        if (cfg.planning.egocircle_prop_cheat) 
        {
            Eigen::Matrix<float, 4, 1> agent_i_state;
            curr_agents_lc.clear();
            for (int i = 0; i < num_obsts; i++) 
            {
                agent_i_state << agent_odoms_lc[i].position.x, agent_odoms_lc[i].position.y, agent_vels_lc[i].vector.x, agent_vels_lc[i].vector.y;
                curr_agents_lc.push_back(agent_i_state);
            }
        } else {
            curr_agents_lc = curr_agents;
        }

        if (print) 
        {
            ROS_INFO_STREAM("    detected agents: ");
            for (int i = 0; i < curr_agents_lc.size(); i++)
                ROS_INFO_STREAM("        agent" << i << " position: " << curr_agents_lc[i][0] << ", " << curr_agents_lc[i][1] << ", velocity: " << curr_agents_lc[i][2] << ", " << curr_agents_lc[i][3]);
        }
        
        int future_scan_idx;
        for (float t_iplus1 = cfg.traj.integrate_stept; t_iplus1 <= cfg.traj.integrate_maxt; t_iplus1 += cfg.traj.integrate_stept) 
        {
            dynamic_laser_scan.ranges = stored_scan.ranges;

            //trajArbiter->recoverDynamicEgocircleCheat(t_i, t_iplus1, agent_odoms_lc, agent_vels_lc, dynamic_laser_scan, print);
            trajArbiter->recoverDynamicEgoCircle(t_i, t_iplus1, curr_agents_lc, dynamic_laser_scan, print);
            
            future_scan_idx = (int) (t_iplus1 / cfg.traj.integrate_stept);
            // ROS_INFO_STREAM("adding scan from " << t_i << " to " << t_iplus1 << " at idx: " << future_scan_idx);
            future_scans[future_scan_idx] = dynamic_laser_scan;

            t_i = t_iplus1;
        }
    }

    geometry_msgs::PoseArray Planner::getPlanTrajectory() 
    {
        // float getPlan_start_time = ros::WallTime::now().toSec();

        // float start_time = ros::WallTime::now().toSec();      
        // std::chrono::steady_clock::time_point start_time_c;

        bool curr_exec_gap_assoc, curr_exec_gap_feas;
        
        std::vector<dynamic_gap::Gap> curr_observed_gaps = associated_observed_gaps;
        // int gaps_size = curr_observed_gaps.size();

        std::chrono::steady_clock::time_point plan_loop_start_time = std::chrono::steady_clock::now();

        ///////////////////////////
        // GAP FEASIBILITY CHECK //
        ///////////////////////////
        std::chrono::steady_clock::time_point feasibility_start_time = std::chrono::steady_clock::now();
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        try 
        { 
            feasible_gap_set = gapSetFeasibilityCheck(curr_exec_gap_assoc, curr_exec_gap_feas);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in gapSetFeasibilityCheck");
        }
        int gaps_size = feasible_gap_set.size();

        float feasibility_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - feasibility_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[gapSetFeasibilityCheck() for " << gaps_size << " gaps: " << feasibility_time << " seconds]");
        
        /////////////////////////////
        // FUTURE SCAN PROPAGATION //
        /////////////////////////////
        std::chrono::steady_clock::time_point future_scans_start_time = std::chrono::steady_clock::now();
        try 
        {
            getFutureScans(agent_odoms, agent_vels, cfg.debug.future_scan_propagation_debug_log);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in getFutureScans");
        }

        float future_scans_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - future_scans_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[getFutureScans() for " << gaps_size << " gaps: " << future_scans_time << " seconds]");
        
        //////////////////////
        // GAP MANIPULATION //
        //////////////////////
        std::chrono::steady_clock::time_point gap_manipulation_start_time = std::chrono::steady_clock::now();
        std::vector<dynamic_gap::Gap> manip_gap_set;
        try 
        {
            manip_gap_set = gapManipulate(feasible_gap_set);
        } catch (std::out_of_range) 
        {
            ROS_INFO_STREAM("out of range in gapManipulate");
        }

        float gap_manipulation_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_manipulation_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[gapManipulate() for " << gaps_size << " gaps: " << gap_manipulation_time << " seconds]");

        ///////////////////////////////////////////
        // GAP TRAJECTORY GENERATION AND SCORING //
        ///////////////////////////////////////////
        std::chrono::steady_clock::time_point gap_trajectory_generation_start_time = std::chrono::steady_clock::now();
        std::vector<geometry_msgs::PoseArray> traj_set;
        std::vector<std::vector<float>> time_set;
        std::vector<std::vector<float>> score_set; 
        try 
        {
            score_set = initialTrajGen(manip_gap_set, traj_set, time_set);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in initialTrajGen");
        }

        float gap_trajectory_generation_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_trajectory_generation_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[initialTrajGen() for " << gaps_size << " gaps: " << gap_trajectory_generation_time << " seconds]");

        visualizeComponents(manip_gap_set); // need to run after initialTrajGen to see what weights for reachable gap are

        //////////////////////////////
        // GAP TRAJECTORY SELECTION //
        //////////////////////////////
        std::chrono::steady_clock::time_point gap_trajectory_selection_start_time = std::chrono::steady_clock::now();
        int traj_idx;
        try 
        {
            traj_idx = pickTraj(traj_set, score_set);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in pickTraj");
        }

        float gap_trajectory_selection_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_trajectory_selection_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[pickTraj() for " << gaps_size << " gaps: " << gap_trajectory_selection_time << " seconds]");

        geometry_msgs::PoseArray chosen_traj;
        std::vector<float> chosen_time_arr;
        dynamic_gap::Gap chosen_gap;
        if (traj_idx >= 0) 
        {
            chosen_traj = traj_set[traj_idx];
            chosen_time_arr = time_set[traj_idx];
            chosen_gap = manip_gap_set[traj_idx];
        } else {
            chosen_traj = geometry_msgs::PoseArray();
            chosen_gap = dynamic_gap::Gap();
        }

        ///////////////////////////////
        // GAP TRAJECTORY COMPARISON //
        ///////////////////////////////
        std::chrono::steady_clock::time_point gap_trajectory_comparison_start_time = std::chrono::steady_clock::now();

        geometry_msgs::PoseArray final_traj;
        try 
        {
            final_traj = compareToOldTraj(chosen_traj, chosen_gap, manip_gap_set, chosen_time_arr, curr_exec_gap_assoc, curr_exec_gap_feas);
        } catch (std::out_of_range) 
        {
            ROS_FATAL_STREAM("out of range in compareToOldTraj");
        }

        float gap_trajectory_comparison_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - gap_trajectory_comparison_start_time).count() / 1.0e6;
        ROS_INFO_STREAM("[compareToOldTraj() for " << gaps_size << " gaps: "  << gap_trajectory_comparison_time << " seconds]");

        float plan_loop_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - plan_loop_start_time).count() / 1.0e6;

        ROS_INFO_STREAM("planning loop for " << gaps_size << " gaps: "  << plan_loop_time << " seconds");
        
        // geometry_msgs::PoseArray final_traj;

        return final_traj;
    }

    void Planner::visualizeComponents(std::vector<dynamic_gap::Gap> manip_gap_set) {
        boost::mutex::scoped_lock gapset(gapset_mutex);

        gapvisualizer->drawManipGaps(manip_gap_set, std::string("manip"));
        // gapvisualizer->drawReachableGaps(manip_gap_set);        
        // gapvisualizer->drawReachableGapsCenters(manip_gap_set); 

        // gapvisualizer->drawGapSplines(manip_gap_set);
        // goalvisualizer->drawGapGoals(manip_gap_set);
    }

    void Planner::printGapAssociations(std::vector<dynamic_gap::Gap> current_gaps, std::vector<dynamic_gap::Gap> previous_gaps, std::vector<int> association) {
        std::cout << "current simplified associations" << std::endl;
        std::cout << "number of gaps: " << current_gaps.size() << ", number of previous gaps: " << previous_gaps.size() << std::endl;
        std::cout << "association size: " << association.size() << std::endl;
        for (int i = 0; i < association.size(); i++) {
            std::cout << association[i] << ", ";
        }
        std::cout << "" << std::endl;

        float curr_x, curr_y, prev_x, prev_y;
        for (int i = 0; i < association.size(); i++) {
            std::vector<int> pair{i, association[i]};
            std::cout << "pair (" << i << ", " << association[i] << "). ";
            int current_gap_idx = int(std::floor(pair[0] / 2.0));
            int previous_gap_idx = int(std::floor(pair[1] / 2.0));

            if (pair[0] % 2 == 0) {  // curr left
                    current_gaps.at(current_gap_idx).getSimplifiedRCartesian(curr_x, curr_y);
                } else { // curr right
                    current_gaps.at(current_gap_idx).getSimplifiedLCartesian(curr_x, curr_y);
            }
            
            if (i >= 0 && association[i] >= 0) {
                if (pair[1] % 2 == 0) { // prev left
                    previous_gaps.at(previous_gap_idx).getSimplifiedRCartesian(prev_x, prev_y);
                } else { // prev right
                    previous_gaps.at(previous_gap_idx).getSimplifiedLCartesian(prev_x, prev_y);
                }
                std::cout << "From (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << simpDistMatrix_[pair[0]][pair[1]] << std::endl;
            } else {
                std::cout << "From NULL to (" << curr_x << ", " <<  curr_y << ")" << std::endl;
            }
        }
    }

    void Planner::printGapModels(std::vector<dynamic_gap::Gap> gaps) 
    {
        // THIS IS NOT FOR MANIPULATED GAPS
        float x,y;
        for (size_t i = 0; i < gaps.size(); i++)
        {
            dynamic_gap::Gap g = gaps.at(i);
            ROS_INFO_STREAM("    gap " << i << ", indices: " << g.RIdx() << " to "  << g.LIdx() << ", left model: " << g.left_model->get_index() << ", right_model: " << g.right_model->get_index());
            Eigen::Matrix<float, 4, 1> left_state = g.left_model->get_cartesian_state();
            g.getLCartesian(x, y);            
            ROS_INFO_STREAM("        left point: (" << x << ", " << y << "), left model: (" << left_state[0] << ", " << left_state[1] << ", " << left_state[2] << ", " << left_state[3] << ")");
            Eigen::Matrix<float, 4, 1> right_state = g.right_model->get_cartesian_state();
            g.getRCartesian(x, y);
            ROS_INFO_STREAM("        right point: (" << x << ", " << y << "), right model: (" << right_state[0] << ", " << right_state[1] << ", " << right_state[2] << ", " << right_state[3] << ")");
           
        }
    }

    bool Planner::recordAndCheckVel(geometry_msgs::Twist cmd_vel) {
        float val = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.linear.y) + std::abs(cmd_vel.angular.z);
        log_vel_comp.push_back(val);
        float cum_vel_sum = std::accumulate(log_vel_comp.begin(), log_vel_comp.end(), float(0));
        bool ret_val = cum_vel_sum > 1.0 || !log_vel_comp.full();
        if (!ret_val && !cfg.man.man_ctrl) {
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            ROS_INFO_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        return ret_val || cfg.man.man_ctrl;
    }

}