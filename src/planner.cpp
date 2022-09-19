#include <dynamic_gap/planner.h>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>

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
        ROS_INFO_STREAM("starting initialize");
        if (initialized())
        {
            ROS_WARN("DynamicGap Planner already initalized");
            return true;
        }

        // Config Setup
        cfg.loadRosParamFromNodeHandle(unh);

        // Visualization Setup
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("curr_exec_dg_traj", 1);
        // dyn_egocircle_pub = nh.advertise<sensor_msgs::LaserScan>("dyn_egocircle", 1);

        // TF Lookup setup
        tfListener = new tf2_ros::TransformListener(tfBuffer);
        _initialized = true;

        finder = new dynamic_gap::GapUtils(cfg);
        gapvisualizer = new dynamic_gap::GapVisualizer(nh, cfg);
        goalselector = new dynamic_gap::GoalSelector(nh, cfg);
        trajvisualizer = new dynamic_gap::TrajectoryVisualizer(nh, cfg);
        trajArbiter = new dynamic_gap::TrajectoryArbiter(nh, cfg);
        gapTrajSyn = new dynamic_gap::GapTrajGenerator(nh, cfg);
        goalvisualizer = new dynamic_gap::GoalVisualizer(nh, cfg);
        gapManip = new dynamic_gap::GapManipulator(nh, cfg);
        trajController = new dynamic_gap::TrajectoryController(nh, cfg);
        gapassociator = new dynamic_gap::GapAssociator(nh, cfg);
        gapFeasibilityChecker = new dynamic_gap::GapFeasibilityChecker(nh, cfg);

        map2rbt.transform.rotation.w = 1;
        rbt2map.transform.rotation.w = 1;
        odom2rbt.transform.rotation.w = 1;
        rbt2odom.transform.rotation.w = 1;
        rbt_in_rbt.pose.orientation.w = 1;
        rbt_in_rbt.header.frame_id = cfg.robot_frame_id;

        log_vel_comp.set_capacity(cfg.planning.halt_size);

        current_rbt_vel = geometry_msgs::Twist();

        rbt_accel = geometry_msgs::TwistStamped();
        
        init_val = 0;
        model_idx = &init_val;

        curr_left_model = NULL;
        curr_right_model = NULL;

        sharedPtr_pose = geometry_msgs::Pose();
        prev_sharedPtr_pose = geometry_msgs::Pose();

        final_goal_rbt = geometry_msgs::PoseStamped();
        num_obsts = cfg.rbt.num_obsts;

        agent_odoms = std::vector<geometry_msgs::Pose>(num_obsts);
        agent_vels = std::vector<geometry_msgs::Vector3Stamped>(num_obsts);

        sensor_msgs::LaserScan tmp_scan = sensor_msgs::LaserScan();
        future_scans.push_back(tmp_scan);
        for (double t_iplus1 = cfg.traj.integrate_stept; t_iplus1 <= cfg.traj.integrate_maxt; t_iplus1 += cfg.traj.integrate_stept) {
            future_scans.push_back(tmp_scan);
        }

        prev_pose_msg_time = ros::Time::now();
        prev_acc_msg_time = ros::Time::now();
        prev_scan_msg_time = ros::Time::now();
        // ROS_INFO_STREAM("future_scans size: " << future_scans.size());
        // ROS_INFO_STREAM("done initializing");
        return true;
    }

    bool Planner::initialized()
    {
        return _initialized;
    }

    int Planner::get_num_obsts() {
        return num_obsts;
    }

    bool Planner::isGoalReached()
    {
        current_pose_ = sharedPtr_pose;
        double dx = final_goal_odom.pose.position.x - current_pose_.position.x;
        double dy = final_goal_odom.pose.position.y - current_pose_.position.y;
        bool result = sqrt(pow(dx, 2) + pow(dy, 2)) < cfg.goal.goal_tolerance;
        if (result)
        {
            ROS_INFO_STREAM("[Reset] Goal Reached");
            return true;
        }

        double waydx = local_waypoint_odom.pose.position.x - current_pose_.position.x;
        double waydy = local_waypoint_odom.pose.position.y - current_pose_.position.y;
        bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) < cfg.goal.waypoint_tolerance;
        if (wayres) {
            ROS_INFO_STREAM("[Reset] Waypoint reached, getting new one");
            // global_plan_location += global_plan_lookup_increment;
        }
        return false;
    }
    
    void Planner::staticLaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg) {
        static_scan_ptr = msg;
        trajArbiter->updateStaticEgoCircle(msg);
        gapManip->updateStaticEgoCircle(msg);
    }

    void Planner::inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sharedPtr_inflatedlaser = msg;
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock gapset(gapset_mutex); // this is where time lag happens (~0.1 to 0.2 seconds)
        ros::Time curr_time = msg->header.stamp;
        double scan_dt = (curr_time - prev_scan_msg_time).toSec();
        // ROS_INFO_STREAM("time since last scanCB: " << scan_dt);

        sharedPtr_laser = msg;
        // planning_inflated is a 0
        if (cfg.planning.planning_inflated && sharedPtr_inflatedlaser) {
            msg = sharedPtr_inflatedlaser;
        }

        Matrix<double, 1, 3> v_ego(current_rbt_vel.linear.x, current_rbt_vel.linear.y, current_rbt_vel.angular.z);
        // Matrix<double, 1, 3> a_ego(rbt_accel.linear.x, rbt_accel.linear.y, rbt_accel.angular.z);
        // Matrix<double, 1, 3> v_ego(rbt_vel_min1.linear.x, rbt_vel_min1.linear.y, rbt_vel_min1.angular.z);
        // Matrix<double, 1, 3> a_ego(rbt_accel_min1.linear.x, rbt_accel_min1.linear.y, rbt_accel_min1.angular.z);

        // ROS_INFO_STREAM("RAW GAP ASSOCIATING");
        // ROS_INFO_STREAM("Time elapsed before raw gaps processing: " << (ros::WallTime::now().toSec() - start_time));

        previous_raw_gaps = associated_raw_gaps;
        raw_gaps = finder->hybridScanGap(msg, final_goal_rbt);
        
        raw_distMatrix = gapassociator->obtainDistMatrix(raw_gaps, previous_raw_gaps, "raw");
        raw_association = gapassociator->associateGaps(raw_distMatrix);         // ASSOCIATE GAPS PASSES BY REFERENCE
        gapassociator->assignModels(raw_association, raw_distMatrix, raw_gaps, previous_raw_gaps, v_ego, model_idx);
        associated_raw_gaps = update_models(raw_gaps, intermediate_vels, intermediate_accs, scan_dt, false);
        // ROS_INFO_STREAM("Time elapsed after raw gaps processing: " << (ros::WallTime::now().toSec() - start_time));

        // double observed_gaps_start_time = ros::WallTime::now().toSec();
        previous_gaps = associated_observed_gaps;
        observed_gaps = finder->mergeGapsOneGo(msg, raw_gaps);
              
        simp_distMatrix = gapassociator->obtainDistMatrix(observed_gaps, previous_gaps, "simplified"); // finishes
        simp_association = gapassociator->associateGaps(simp_distMatrix); // must finish this and therefore change the association
        gapassociator->assignModels(simp_association, simp_distMatrix, observed_gaps, previous_gaps, v_ego, model_idx);
        associated_observed_gaps = update_models(observed_gaps, intermediate_vels, intermediate_accs, scan_dt, false);
        // ROS_INFO_STREAM("Time elapsed after observed gaps processing: " << (ros::WallTime::now().toSec() - start_time));

        finder->staticDynamicScanSeparation(associated_observed_gaps, msg);

        intermediate_vels.clear();
        intermediate_accs.clear();

        // gapvisualizer->drawGaps(associated_raw_gaps, std::string("raw"));
        gapvisualizer->drawGaps(associated_observed_gaps, std::string("simp"));
        gapvisualizer->drawGapsModels(associated_observed_gaps);

        boost::shared_ptr<sensor_msgs::LaserScan const> tmp;
        if (sharedPtr_inflatedlaser) {
            tmp = sharedPtr_inflatedlaser;
        } else {
            tmp = msg;
        }

        geometry_msgs::PoseStamped local_goal;
        {
            goalselector->updateEgoCircle(tmp);
            goalselector->updateLocalGoal(map2rbt);
            local_goal = goalselector->transformLocalGoalToOdomFrame(rbt2odom);
            goalvisualizer->localGoal(local_goal);
        }
        // ROS_INFO_STREAM("Time elapsed after updating goal selector: " << (ros::WallTime::now().toSec() - start_time));

        trajArbiter->updateEgoCircle(msg);
        trajArbiter->updateLocalGoal(local_goal, odom2rbt);

        // ROS_INFO_STREAM("Time elapsed after updating arbiter: " << (ros::WallTime::now().toSec() - start_time));

        gapManip->updateEgoCircle(msg);
        trajController->updateEgoCircle(msg);
        gapFeasibilityChecker->updateEgoCircle(msg);
        // ROS_INFO_STREAM("Time elapsed after updating rest: " << (ros::WallTime::now().toSec() - start_time));

        // ROS_INFO_STREAM("laserscan time elapsed: " << ros::WallTime::now().toSec() - start_time);

        prev_scan_msg_time = curr_time;

    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    std::vector<dynamic_gap::Gap> Planner::update_models(std::vector<dynamic_gap::Gap> _observed_gaps, 
                                                         std::vector<geometry_msgs::Twist> intermediate_vels, 
                                                         std::vector<geometry_msgs::TwistStamped> intermediate_accs, 
                                                         double scan_dt,
                                                         bool print) {
        std::vector<dynamic_gap::Gap> associated_observed_gaps = _observed_gaps;
        
        // double start_time = ros::WallTime::now().toSec();
        for (int i = 0; i < 2*associated_observed_gaps.size(); i++) {
            //std::cout << "update gap model: " << i << std::endl;
            update_model(i, associated_observed_gaps, intermediate_vels, intermediate_accs, scan_dt, print);
            //std::cgout << "" << std::endl;
		}

        //ROS_INFO_STREAM("update_models time elapsed: " << ros::WallTime::now().toSec() - start_time);
        return associated_observed_gaps;
    }

    void Planner::update_model(int i, std::vector<dynamic_gap::Gap>& _observed_gaps, 
                                std::vector<geometry_msgs::Twist> intermediate_vels, 
                                std::vector<geometry_msgs::TwistStamped> intermediate_accs, double scan_dt, bool print) {
		dynamic_gap::Gap g = _observed_gaps[int(std::floor(i / 2.0))];
 
        double beta_tilde, range_tilde;
		if (i % 2 == 0) {
			beta_tilde = float(g.RIdx() - g.half_scan) / g.half_scan * M_PI;
            range_tilde = g.RDist();
		} else {
            beta_tilde = float(g.LIdx() - g.half_scan) / g.half_scan * M_PI;
            range_tilde = g.LDist();
		}

		Matrix<double, 2, 1> laserscan_measurement(range_tilde, beta_tilde);

        if (i % 2 == 0) {
            //std::cout << "entering left model update" << std::endl;
            try {
                g.right_model->kf_update_loop(laserscan_measurement, intermediate_accs, intermediate_vels, print, agent_odoms, agent_vels, scan_dt);
            } catch (...) {
                ROS_FATAL_STREAM("kf_update loop fails");
            }
        } else {
            //std::cout << "entering right model update" << std::endl;
            try {
                g.left_model->kf_update_loop(laserscan_measurement, intermediate_accs, intermediate_vels, print, agent_odoms, agent_vels, scan_dt);
            } catch (...) {
                ROS_FATAL_STREAM("kf_update loop fails");
            }
        }
    }

    /*
    Acceleration message comes in in robot frame 
    */
    void Planner::robotAccCB(boost::shared_ptr<geometry_msgs::TwistStamped const> msg)
    {
        ros::Time curr_time = msg->header.stamp;
        // ROS_INFO_STREAM("time since last accCB: " << (curr_time - prev_acc_msg_time).toSec());
        prev_acc_msg_time = curr_time;

        rbt_accel = *msg;
        intermediate_accs.push_back(rbt_accel);
    }
    
    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ros::Time curr_time = msg->header.stamp;
        // ROS_INFO_STREAM("time since last poseCB: " << (curr_time - prev_pose_msg_time).toSec());
        prev_pose_msg_time = curr_time;
        updateTF();

        // Transform the msg to odom frame
        if(msg->header.frame_id != cfg.odom_frame_id)
        {
            //std::cout << "odom msg is not in odom frame" << std::endl;
            geometry_msgs::TransformStamped robot_pose_odom_trans = tfBuffer.lookupTransform(cfg.odom_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;

            tf2::doTransform(in_pose, out_pose, robot_pose_odom_trans);
            sharedPtr_pose = out_pose.pose;
        }
        else
        {
            sharedPtr_pose = msg->pose.pose;
        }

        // velocity always comes in wrt robot frame in STDR
        current_rbt_vel = msg->twist.twist;
        intermediate_vels.push_back(current_rbt_vel);

        // ros::Time end_time = ros::Time::now();
        // ROS_INFO_STREAM("poseCB time taken: " << (end_time - curr_time).toSec());
        
    }
    
    void Planner::agentOdomCB(const nav_msgs::Odometry::ConstPtr& msg) {

        std::string robot_namespace = msg->child_frame_id;
        // ROS_INFO_STREAM("robot_namespace: " << robot_namespace);
        robot_namespace.erase(0,5); // removing "robot"
        char *robot_name_char = strdup(robot_namespace.c_str());
        int robot_id = std::atoi(robot_name_char);
        // ROS_INFO_STREAM("robot_id: " << robot_id);
        // I need BOTH odom and vel in robot2 frame
        //std::cout << "odom msg is not in odom frame" << std::endl;
        try {
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
        
        try {
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
            double waydx = local_waypoint_odom.pose.position.x - new_local_waypoint.pose.position.x;
            double waydy = local_waypoint_odom.pose.position.y - new_local_waypoint.pose.position.y;
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

        return true;
    }

    void Planner::updateTF()
    {
        try {
            map2rbt  = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2map  = tfBuffer.lookupTransform(cfg.map_frame_id, cfg.robot_frame_id, ros::Time(0));
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

    std::vector<dynamic_gap::Gap> Planner::gapManipulate(std::vector<dynamic_gap::Gap> _observed_gaps) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<dynamic_gap::Gap> manip_set = _observed_gaps;
        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;

        for (size_t i = 0; i < manip_set.size(); i++)
        {
            if (cfg.gap_manip.debug_log) ROS_INFO_STREAM("MANIPULATING INITIAL GAP " << i);
            // MANIPULATE POINTS AT T=0
            manip_set.at(i).initManipIndices();
            
            // gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true);
            gapManip->convertAxialGap(manip_set.at(i), true); 
            gapManip->inflateGapSides(manip_set.at(i), true);
            gapManip->radialExtendGap(manip_set.at(i), true);
            gapManip->setGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true);
            
            // MANIPULATE POINTS AT T=1
            if (cfg.gap_manip.debug_log) ROS_INFO_STREAM("MANIPULATING TERMINAL GAP " << i);
            gapManip->updateDynamicEgoCircle(manip_set.at(i), future_scans);
            if ((!manip_set.at(i).gap_crossed && !manip_set.at(i).gap_closed) || (manip_set.at(i).gap_crossed_behind)) {
                // gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), false);
                gapManip->convertAxialGap(manip_set.at(i), false);
            }
            gapManip->inflateGapSides(manip_set.at(i), false);
            gapManip->radialExtendGap(manip_set.at(i), false);
            gapManip->setTerminalGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal());
        }

        return manip_set;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<double>> Planner::initialTrajGen(std::vector<dynamic_gap::Gap>& vec, std::vector<geometry_msgs::PoseArray>& res, std::vector<std::vector<double>>& res_time_traj) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<geometry_msgs::PoseArray> ret_traj(vec.size());
        std::vector<std::vector<double>> ret_time_traj(vec.size());
        std::vector<std::vector<double>> ret_traj_scores(vec.size());
        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam; // lc as local copy

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;
        try {
            for (size_t i = 0; i < vec.size(); i++) {
                if (cfg.traj.debug_log) ROS_INFO_STREAM("generating traj for gap: " << i);
                // std::cout << "starting generate trajectory with rbt_in_cam_lc: " << rbt_in_cam_lc.pose.position.x << ", " << rbt_in_cam_lc.pose.position.y << std::endl;
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple;
                
                // TRAJECTORY GENERATED IN RBT FRAME
                bool run_g2g = (vec.at(i).goal.goalwithin || vec.at(i).artificial);
                if (run_g2g) {
                    std::tuple<geometry_msgs::PoseArray, std::vector<double>> g2g_tuple;
                    g2g_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, run_g2g);
                    g2g_tuple = gapTrajSyn->forwardPassTrajectory(g2g_tuple);
                    std::vector<double> g2g_score_vec = trajArbiter->scoreTrajectory(std::get<0>(g2g_tuple), std::get<1>(g2g_tuple), curr_raw_gaps, 
                                                                                     agent_odoms, agent_vels, future_scans, false, false);
                    double g2g_score = std::accumulate(g2g_score_vec.begin(), g2g_score_vec.end(), double(0));

                    std::tuple<geometry_msgs::PoseArray, std::vector<double>> ahpf_tuple;
                    ahpf_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, !run_g2g);
                    ahpf_tuple = gapTrajSyn->forwardPassTrajectory(ahpf_tuple);
                    std::vector<double> ahpf_score_vec = trajArbiter->scoreTrajectory(std::get<0>(ahpf_tuple), std::get<1>(ahpf_tuple), curr_raw_gaps, 
                                                                                        agent_odoms, agent_vels, future_scans, false, false);
                    double ahpf_score = std::accumulate(ahpf_score_vec.begin(), ahpf_score_vec.end(), double(0));

                    if (cfg.gap_manip.debug_log) {
                        ROS_INFO_STREAM("running g2g and ahpf");
                        ROS_INFO_STREAM("g2g_score: " << g2g_score);
                        ROS_INFO_STREAM("ahpf_score: " << ahpf_score);
                    }

                    return_tuple = (g2g_score > ahpf_score) ? g2g_tuple : ahpf_tuple;
                    ret_traj_scores.at(i) = (g2g_score > ahpf_score) ? g2g_score_vec : ahpf_score_vec;
                } else {
                    return_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel, run_g2g);
                    return_tuple = gapTrajSyn->forwardPassTrajectory(return_tuple);

                    if (cfg.gap_manip.debug_log) ROS_INFO_STREAM("scoring trajectory for gap: " << i);
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

    int Planner::pickTraj(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<double>> score) {
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
        std::vector<double> result_score(prr.size());
        int counts;
        try {
            if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < result_score.size(); i++) {
                // ROS_WARN_STREAM("prr(" << i << "): size " << prr.at(i).poses.size());
                counts = std::min(cfg.planning.num_feasi_check, int(score.at(i).size()));

                result_score.at(i) = std::accumulate(score.at(i).begin(), score.at(i).begin() + counts, double(0));
                result_score.at(i) = prr.at(i).poses.size() == 0 ? -std::numeric_limits<double>::infinity() : result_score.at(i);
                if (cfg.traj.debug_log) ROS_INFO_STREAM("for gap " << i << " (length: " << prr.at(i).poses.size() << "), returning score of " << result_score.at(i));
                /*
                if (result_score.at(i) == -std::numeric_limits<double>::infinity()) {
                    for (size_t j = 0; j < counts; j++) {
                        if (score.at(i).at(j) == -std::numeric_limits<double>::infinity()) {
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

        if (result_score.at(idx) == -std::numeric_limits<double>::infinity()) {
            
            ROS_INFO_STREAM("all -infinity");
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (auto val : result_score) {
                ROS_INFO_STREAM("Score: " << val);
            }
            ROS_INFO_STREAM("------------------");
        }

        if (cfg.traj.debug_log) ROS_INFO_STREAM("picking gap: " << idx);
        
        return idx;
    }

    geometry_msgs::PoseArray Planner::changeTrajectoryHelper(dynamic_gap::Gap incoming_gap, geometry_msgs::PoseArray incoming, std::vector<double> time_arr, bool switching_to_empty) {
        
        if (switching_to_empty) {
            geometry_msgs::PoseArray empty_traj = geometry_msgs::PoseArray();
            std::vector<double> empty_time_arr;
            setCurrentTraj(empty_traj);
            setCurrentTimeArr(empty_time_arr);
            setCurrentLeftModel(NULL);
            setCurrentRightModel(NULL);
            setCurrentGapPeakVelocities(0.0, 0.0);
            trajectory_pub.publish(empty_traj);

            return empty_traj;
        } else {
            setCurrentTraj(incoming);
            setCurrentTimeArr(time_arr);
            setCurrentLeftModel(incoming_gap.left_model);
            setCurrentRightModel(incoming_gap.right_model);
            setCurrentGapPeakVelocities(incoming_gap.peak_velocity_x, incoming_gap.peak_velocity_y);
            trajectory_pub.publish(incoming);          

            return incoming;  
        }                       
    }

    geometry_msgs::PoseArray Planner::compareToOldTraj(geometry_msgs::PoseArray incoming, dynamic_gap::Gap incoming_gap, std::vector<dynamic_gap::Gap> feasible_gaps, std::vector<double> time_arr) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        auto curr_traj = getCurrentTraj();
        auto curr_time_arr = getCurrentTimeArr();

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;

        try {
            double curr_time = ros::WallTime::now().toSec();
            
            // FORCING OFF CURRENT TRAJ IF NO LONGER FEASIBLE
            // ROS_INFO_STREAM("current left gap index: " << getCurrentLeftGapIndex() << ", current right gap index: " << getCurrentRightGapIndex());
            bool curr_gap_feasible = false;
            for (dynamic_gap::Gap g : feasible_gaps) {
                // ROS_INFO_STREAM("feasible left gap index: " << g.left_model->get_index() << ", feasible right gap index: " << g.right_model->get_index());
                if (g.right_model->get_index() == getCurrentRightGapIndex() && g.left_model->get_index() == getCurrentLeftGapIndex()) {
                    curr_gap_feasible = true;
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
            auto incom_score = trajArbiter->scoreTrajectory(incom_rbt, time_arr, curr_raw_gaps, 
                                                            agent_odoms, agent_vels, future_scans, false, true);
            // int counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));

            int counts = std::min(cfg.planning.num_feasi_check, (int) incom_score.size());
            auto incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));

            if (cfg.traj.debug_log) ROS_INFO_STREAM("incoming trajectory subscore: " << incom_subscore);
            bool curr_traj_length_zero = curr_traj.poses.size() == 0;
            bool curr_gap_not_feasible = !curr_gap_feasible;


            // CURRENT TRAJECTORY LENGTH ZERO OR IS NOT FEASIBLE
            if (curr_traj_length_zero || curr_gap_not_feasible) {
                bool valid_incoming_traj = incoming.poses.size() > 0 && incom_subscore > -std::numeric_limits<double>::infinity();
                
                std::string curr_traj_status = (curr_traj_length_zero) ? "curr traj length 0" : "curr traj is not feasible";
                if (valid_incoming_traj) {
                    if (cfg.traj.debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: " << curr_traj_status << ", incoming score finite");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
                } else  {
                    std::string incoming_traj_status = (incoming.poses.size() > 0) ? "incoming traj length 0" : "incoming score infinite";

                    if (cfg.traj.debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO EMPTY: " << curr_traj_status << ", " << incoming_traj_status);
                    
                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, true);
                }
            } 

            // CURRENT TRAJECTORY HAS ENDED
            auto curr_rbt = gapTrajSyn->transformBackTrajectory(curr_traj, odom2rbt);
            curr_rbt.header.frame_id = cfg.robot_frame_id;
            int start_position = egoTrajPosition(curr_rbt);
            geometry_msgs::PoseArray reduced_curr_rbt = curr_rbt;
            std::vector<double> reduced_curr_time_arr = curr_time_arr;
            reduced_curr_rbt.poses = std::vector<geometry_msgs::Pose>(curr_rbt.poses.begin() + start_position, curr_rbt.poses.end());
            reduced_curr_time_arr = std::vector<double>(curr_time_arr.begin() + start_position, curr_time_arr.end());
            if (reduced_curr_rbt.poses.size() < 2) {
                if (cfg.traj.debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: old traj length less than 2");

                return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
            }

            counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incoming.poses.size(), reduced_curr_rbt.poses.size()));
            incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));
            if (cfg.traj.debug_log) ROS_INFO_STREAM("re-scored incoming trajectory subscore: " << incom_subscore);
            auto curr_score = trajArbiter->scoreTrajectory(reduced_curr_rbt, reduced_curr_time_arr, curr_raw_gaps, 
                                                           agent_odoms, agent_vels, future_scans, false, false);
            auto curr_subscore = std::accumulate(curr_score.begin(), curr_score.begin() + counts, double(0));
            if (cfg.traj.debug_log) ROS_INFO_STREAM("current trajectory subscore: " << curr_subscore);

            std::vector<std::vector<double>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incom_score;
            ret_traj_scores.at(1) = curr_score;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incom_rbt;
            viz_traj.at(1) = reduced_curr_rbt;
            trajvisualizer->pubAllScore(viz_traj, ret_traj_scores);

            if (curr_subscore == -std::numeric_limits<double>::infinity()) {
                if (incom_subscore == -std::numeric_limits<double>::infinity()) {
                    if (cfg.traj.debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO EMPTY: both -infinity");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, true);
                } else {
                    if (cfg.traj.debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: swapping trajectory due to collision");

                    return changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
                }
            }

            /*
            if (incom_subscore > curr_subscore) {
                if (cfg.traj.debug_log) ROS_INFO_STREAM("TRAJECTORY CHANGE TO INCOMING: higher score");
                changeTrajectoryHelper(incoming_gap, incoming, time_arr, false);
            }
            */
          
            if (cfg.traj.debug_log) ROS_INFO_STREAM("TRAJECTORY MAINTAIN");
            trajectory_pub.publish(curr_traj);
        } catch (...) {
            ROS_FATAL_STREAM("compareToOldTraj");
        }
        return curr_traj;
    }

    int Planner::egoTrajPosition(geometry_msgs::PoseArray curr) {
        std::vector<double> pose_diff(curr.poses.size());
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

    void Planner::setCurrentRightModel(dynamic_gap::cart_model * _right_model) {
        curr_right_model = _right_model;
    }

    void Planner::setCurrentLeftModel(dynamic_gap::cart_model * _left_model) {
        curr_left_model = _left_model;
    }

    void Planner::setCurrentGapPeakVelocities(double _peak_velocity_x, double _peak_velocity_y) {
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

    void Planner::setCurrentTimeArr(std::vector<double> curr_time_arr) {
        curr_executing_time_arr = curr_time_arr;
        return;
    }
    
    std::vector<double> Planner::getCurrentTimeArr() {
        return curr_executing_time_arr;
    }

    void Planner::reset()
    {
        observed_gaps.clear();
        setCurrentTraj(geometry_msgs::PoseArray());
        rbt_accel = geometry_msgs::TwistStamped();
        ROS_INFO_STREAM("log_vel_comp size: " << log_vel_comp.size());
        log_vel_comp.clear();
        ROS_INFO_STREAM("log_vel_comp size after clear: " << log_vel_comp.size() << ", is full: " << log_vel_comp.capacity());
        return;
    }


    bool Planner::isReplan() {
        return replan;
    }

    void Planner::setReplan() {
        replan = false;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(geometry_msgs::PoseArray traj) {
        sensor_msgs::LaserScan stored_scan_msgs;
        if (cfg.planning.projection_inflated) {
            stored_scan_msgs = *sharedPtr_inflatedlaser.get();
        } else {
            stored_scan_msgs = *sharedPtr_laser.get();
        }

        geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
        if (traj.poses.size() < 2) {
            ROS_INFO_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 2");
            cmd_vel = trajController->obstacleAvoidanceControlLaw(stored_scan_msgs);
            return cmd_vel;
        }

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
        auto orig_ref = trajController->trajGen(traj);
        
        // get point along trajectory to target/move towards
        ctrl_idx = trajController->targetPoseIdx(curr_pose, orig_ref);
        nav_msgs::Odometry ctrl_target_pose;
        ctrl_target_pose.header = orig_ref.header;
        ctrl_target_pose.pose.pose = orig_ref.poses.at(ctrl_idx);
        ctrl_target_pose.twist.twist = orig_ref.twist.at(ctrl_idx);

        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;
        sensor_msgs::LaserScan static_scan = *static_scan_ptr.get();
        cmd_vel = trajController->controlLaw(curr_pose, ctrl_target_pose, 
                                             static_scan, rbt_in_cam_lc,
                                             current_rbt_vel, rbt_accel,
                                             curr_right_model, curr_left_model,
                                             curr_peak_velocity_x, curr_peak_velocity_y);
        //geometry_msgs::Twist cmd_vel;
        //cmd_vel.linear.x = 0.25;
        return cmd_vel;
    }

    void Planner::rcfgCallback(dynamic_gap::dgConfig &config, uint32_t level)
    {
        cfg.reconfigure(config);
        
        // set_capacity destroys everything if different from original size, 
        // resize only if the new size is greater
        log_vel_comp.clear();
        log_vel_comp.set_capacity(cfg.planning.halt_size);
    }

    // should return gaps with initialized models, no attachements to anything else
    std::vector<dynamic_gap::Gap> Planner::get_curr_raw_gaps() {
        return raw_gaps;
    }

    std::vector<dynamic_gap::Gap> Planner::get_curr_observed_gaps() {
        return observed_gaps;
    }

    std::vector<int> Planner::get_raw_associations() {
        return raw_association;
    }

    std::vector<int> Planner::get_simplified_associations() {
        return simp_association;
    }

    std::vector<dynamic_gap::Gap> Planner::gapSetFeasibilityCheck() {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        //std::cout << "PULLING MODELS TO ACT ON" << std::endl;
        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;
        std::vector<dynamic_gap::Gap> curr_observed_gaps = associated_observed_gaps;
        
        std::vector<dynamic_gap::Gap> prev_raw_gaps = previous_raw_gaps;
        std::vector<dynamic_gap::Gap> prev_observed_gaps = previous_gaps;  

        //std::vector<int> _raw_association = raw_association;
        //std::vector<int> _simp_association = simp_association;

        //std::cout << "curr_raw_gaps size: " << curr_raw_gaps.size() << std::endl;
        //std::cout << "curr_observed_gaps size: " << curr_observed_gaps.size() << std::endl;

        //std::cout << "prev_raw_gaps size: " << prev_raw_gaps.size() << std::endl;
        //std::cout << "prev_observed_gaps size: " << prev_observed_gaps.size() << std::endl;

        //std::cout << "_raw_association size: " << _raw_association.size() << std::endl;
        //std::cout << "_simp_association size: " << _simp_association.size() << std::endl;

        //std::cout << "current robot velocity. Linear: " << current_rbt_vel.linear.x << ", " << current_rbt_vel.linear.y << ", angular: " << current_rbt_vel.angular.z << std::endl;
        //std::cout << "current raw gaps:" << std::endl;
        //printGapModels(curr_raw_gaps);

        //std::cout << "pulled current simplified associations:" << std::endl;
        //printGapAssociations(curr_observed_gaps, prev_observed_gaps, _simp_association);
        
        /*
        ROS_INFO_STREAM("current raw gaps:");
        printGapModels(curr_raw_gaps);
        */
        
        if (cfg.gap_feas.debug_log) {
            ROS_INFO_STREAM("current simplified gaps:");
            printGapModels(curr_observed_gaps);
        }

        /*
        int curr_left_idx = getCurrentLeftGapIndex();
        int curr_right_idx = getCurrentRightGapIndex();
        ROS_INFO_STREAM("current left/right indices: " << curr_left_idx << ", " << curr_right_idx);
        bool gap_associated = false;
        
        */

        bool gap_i_feasible;
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        for (size_t i = 0; i < curr_observed_gaps.size(); i++) {
            // obtain crossing point
            /*
            if ( curr_observed_gaps.at(i).left_model->get_index() == curr_left_idx && curr_observed_gaps.at(i).right_model->get_index() == curr_right_idx) {
                gap_associated = true;
            }
            //  << ", left index: " << curr_observed_gaps.at(i).left_model->get_index() << ", right index: " << curr_observed_gaps.at(i).right_model->get_index()
            */

            if (cfg.gap_feas.debug_log) ROS_INFO_STREAM("feasibility check for gap " << i);
            gap_i_feasible = gapFeasibilityChecker->indivGapFeasibilityCheck(curr_observed_gaps.at(i));
            
            if (gap_i_feasible) {
                curr_observed_gaps.at(i).addTerminalRightInformation();
                feasible_gap_set.push_back(curr_observed_gaps.at(i));
                // ROS_INFO_STREAM("Pushing back gap with peak velocity of : " << curr_observed_gaps.at(i).peak_velocity_x << ", " << curr_observed_gaps.at(i).peak_velocity_y);
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
                                 bool print) {
        // boost::mutex::scoped_lock gapset(gapset_mutex);
        // ROS_INFO_STREAM("running getFutureScans");
        sensor_msgs::LaserScan stored_scan = *sharedPtr_laser.get();
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
        std::vector<float> scan_intensities(stored_scan.ranges.size(), 0.5);
        dynamic_laser_scan.intensities = scan_intensities;

        double t_i = 0.0;
        future_scans[0] = dynamic_laser_scan;

        std::vector<geometry_msgs::Pose> agent_odoms_lc = _agent_odoms;
        std::vector<geometry_msgs::Vector3Stamped> agent_vels_lc = _agent_vels;

        int future_scan_idx;
        for (double t_iplus1 = cfg.traj.integrate_stept; t_iplus1 <= cfg.traj.integrate_maxt; t_iplus1 += cfg.traj.integrate_stept) {
            dynamic_laser_scan.ranges = stored_scan.ranges;

            trajArbiter->recoverDynamicEgocircleCheat(t_i, t_iplus1, agent_odoms_lc, agent_vels_lc, dynamic_laser_scan, print);

            future_scan_idx = (int) (t_iplus1 / cfg.traj.integrate_stept);
            // ROS_INFO_STREAM("adding scan from " << t_i << " to " << t_iplus1 << " at idx: " << future_scan_idx);
            future_scans[future_scan_idx] = dynamic_laser_scan;

            t_i = t_iplus1;
        }
    }

    geometry_msgs::PoseArray Planner::getPlanTrajectory() {
        double getPlan_start_time = ros::WallTime::now().toSec();

        double start_time = ros::WallTime::now().toSec();      
        
        
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        try { 
            feasible_gap_set = gapSetFeasibilityCheck();
        } catch (std::out_of_range) {
            ROS_FATAL_STREAM("out of range in gapSetFeasibilityCheck");
        }
        int gaps_size = feasible_gap_set.size();
        ROS_INFO_STREAM("DGap gapSetFeasibilityCheck time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));

        
        start_time = ros::WallTime::now().toSec();
        try {
            getFutureScans(agent_odoms, agent_vels, false);
        } catch (std::out_of_range) {
            ROS_FATAL_STREAM("out of range in getFutureScans");
        }
        ROS_INFO_STREAM("DGap getFutureScans time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));
        
        start_time = ros::WallTime::now().toSec();
        std::vector<dynamic_gap::Gap> manip_gap_set;
        try {
            manip_gap_set = gapManipulate(feasible_gap_set);
        } catch (std::out_of_range) {
            ROS_FATAL_STREAM("out of range in gapManipulate");
        }
        ROS_INFO_STREAM("DGap gapManipulate time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));

        start_time = ros::WallTime::now().toSec();
        std::vector<geometry_msgs::PoseArray> traj_set;
        std::vector<std::vector<double>> time_set;

        std::vector<std::vector<double>> score_set; 
        try {
            score_set = initialTrajGen(manip_gap_set, traj_set, time_set);
        } catch (std::out_of_range) {
            ROS_FATAL_STREAM("out of range in initialTrajGen");
        }
        ROS_INFO_STREAM("DGap initialTrajGen time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));

        visualizeComponents(manip_gap_set); // need to run after initialTrajGen to see what weights for reachable gap are

        start_time = ros::WallTime::now().toSec();
        int traj_idx;
        try {
            traj_idx = pickTraj(traj_set, score_set);
        } catch (std::out_of_range) {
            ROS_FATAL_STREAM("out of range in pickTraj");
        }
        ROS_INFO_STREAM("DGap pickTraj time taken for " << gaps_size << " gaps: " << (ros::WallTime::now().toSec() - start_time));

        geometry_msgs::PoseArray chosen_traj;
        std::vector<double> chosen_time_arr;
        dynamic_gap::Gap chosen_gap;
        if (traj_idx >= 0) {
            chosen_traj = traj_set[traj_idx];
            chosen_time_arr = time_set[traj_idx];
            chosen_gap = manip_gap_set[traj_idx];
        } else {
            chosen_traj = geometry_msgs::PoseArray();
            chosen_gap = dynamic_gap::Gap();
        }

        start_time = ros::WallTime::now().toSec();
        geometry_msgs::PoseArray final_traj;
        
        try {
            final_traj = compareToOldTraj(chosen_traj, chosen_gap, feasible_gap_set, chosen_time_arr);
        } catch (std::out_of_range) {
            ROS_FATAL_STREAM("out of range in compareToOldTraj");
        }
        ROS_INFO_STREAM("DGap compareToOldTraj time taken for " << gaps_size << " gaps: "  << (ros::WallTime::now().toSec() - start_time));
        
        ROS_INFO_STREAM("DGap getPlanTrajectory time taken for " << gaps_size << " gaps: "  << (ros::WallTime::now().toSec() - getPlan_start_time));
        return final_traj;
    }

    void Planner::visualizeComponents(std::vector<dynamic_gap::Gap> manip_gap_set) {
        boost::mutex::scoped_lock gapset(gapset_mutex);

        gapvisualizer->drawManipGaps(manip_gap_set, std::string("manip"));
        gapvisualizer->drawReachableGaps(manip_gap_set);        
        gapvisualizer->drawReachableGapsCenters(manip_gap_set); 

        gapvisualizer->drawGapSplines(manip_gap_set);
        goalvisualizer->drawGapGoals(manip_gap_set);
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
                std::cout << "From (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << simp_distMatrix[pair[0]][pair[1]] << std::endl;
            } else {
                std::cout << "From NULL to (" << curr_x << ", " <<  curr_y << ")" << std::endl;
            }
        }
    }

    void Planner::printGapModels(std::vector<dynamic_gap::Gap> gaps) {
        // THIS IS NOT FOR MANIPULATED GAPS
        float x,y;
        for (size_t i = 0; i < gaps.size(); i++)
        {
            dynamic_gap::Gap g = gaps.at(i);
            ROS_INFO_STREAM("gap " << i << ", indices: " << g.RIdx() << " to "  << g.LIdx());
            Matrix<double, 4, 1> left_state = g.left_model->get_cartesian_state();
            g.getLCartesian(x, y);            
            ROS_INFO_STREAM("left point: (" << x << ", " << y << "), left model: (" << left_state[0] << ", " << left_state[1] << ", " << left_state[2] << ", " << left_state[3] << ")");
            Matrix<double, 4, 1> right_state = g.right_model->get_cartesian_state();
            g.getRCartesian(x, y);
            ROS_INFO_STREAM("right point: (" << x << ", " << y << "), right model: (" << right_state[0] << ", " << right_state[1] << ", " << right_state[2] << ", " << right_state[3] << ")");
           
        }
    }

    bool Planner::recordAndCheckVel(geometry_msgs::Twist cmd_vel) {
        double val = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.linear.y) + std::abs(cmd_vel.angular.z);
        log_vel_comp.push_back(val);
        double cum_vel_sum = std::accumulate(log_vel_comp.begin(), log_vel_comp.end(), double(0));
        bool ret_val = cum_vel_sum > 1.0 || !log_vel_comp.full();
        if (!ret_val && !cfg.man.man_ctrl) {
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            ROS_INFO_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        return ret_val || cfg.man.man_ctrl;
    }

}