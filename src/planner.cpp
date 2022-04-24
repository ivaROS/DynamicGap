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
        if (initialized())
        {
            ROS_WARN("DynamicGap Planner already initalized");
            return true;
        }

        std::vector<int> association;

        // Config Setup
        cfg.loadRosParamFromNodeHandle(unh);

        // Visualization Setup
        // Fix this later
        local_traj_pub = nh.advertise<geometry_msgs::PoseArray>("relevant_traj", 500);
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("pg_traj", 10);
        gap_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("gaps", 1);
        selected_gap_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("sel_gaps", 1);
        ni_traj_pub = nh.advertise<geometry_msgs::PoseArray>("ni_traj", 10);
        ni_traj_pub_other = nh.advertise<visualization_msgs::MarkerArray>("other_ni_traj", 5);
        rbt_accel_sub = nh.subscribe(cfg.robot_frame_id + "/imu", 100, &Planner::robotImuCB, this);

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

        map2rbt.transform.rotation.w = 1;
        rbt2map.transform.rotation.w = 1;
        odom2rbt.transform.rotation.w = 1;
        rbt2odom.transform.rotation.w = 1;
        rbt_in_rbt.pose.orientation.w = 1;
        rbt_in_rbt.header.frame_id = cfg.robot_frame_id;

        log_vel_comp.set_capacity(cfg.planning.halt_size);

        previous_cmd_vel = geometry_msgs::Twist();
        current_cmd_vel = geometry_msgs::Twist();

        a << 0.0, 0.0;
        init_val = 0;
        model_idx = &init_val;
        prev_traj_switch_time = ros::Time::now().toSec();
        init_time = ros::Time::now().toSec(); 
        return true;
    }

    bool Planner::initialized()
    {
        return _initialized;
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

    void Planner::inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sharedPtr_inflatedlaser = msg;
    }

    void Planner::robotImuCB(boost::shared_ptr<sensor_msgs::Imu const> msg)
    {
        geometry_msgs::Vector3Stamped rbt_accel_rbt_frame;

        rbt_accel_rbt_frame.vector.x = msg->linear_acceleration.x;
        rbt_accel_rbt_frame.vector.y = msg->linear_acceleration.y;

        a[0] = rbt_accel_rbt_frame.vector.x;
        a[1] = rbt_accel_rbt_frame.vector.y;
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sharedPtr_laser = msg;

        if (cfg.planning.planning_inflated && sharedPtr_inflatedlaser) {
            msg = sharedPtr_inflatedlaser;
        }

        // ROS_INFO_STREAM(msg.get()->ranges.size());

        try {
            boost::mutex::scoped_lock gapset(gapset_mutex);
            // getting raw gaps
            raw_gaps = finder->hybridScanGap(msg);
            frame = msg.get()->header.frame_id;
            angle_min = msg.get()->angle_min;
            angle_increment = msg.get()->angle_increment;
            min_safe_dist = *std::min_element(msg.get()->ranges.begin(), msg.get()->ranges.end());

            gapvisualizer->drawGaps(raw_gaps, std::string("raw"));
            observed_gaps = finder->mergeGapsOneGo(msg, raw_gaps);

            // do we need to merge every time?
            // ROS_INFO_STREAM("observed_gaps count:" << observed_gaps.size());
        } catch (...) {
            ROS_FATAL_STREAM("mergeGapsOneGo");
        }

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
            local_goal = goalselector->getCurrentLocalGoal(rbt2odom);
            goalvisualizer->localGoal(local_goal);
        }
        
        trajArbiter->updateEgoCircle(msg);
        trajArbiter->updateLocalGoal(local_goal, odom2rbt);

        gapManip->updateEgoCircle(msg);
        trajController->updateEgoCircle(msg);

    }
    
    void Planner::update_model(int i, std::vector<dynamic_gap::Gap>& _observed_gaps) {
        // UPDATING MODELS
        //std::cout << "obtaining gap g" << std::endl;
		geometry_msgs::Vector3Stamped gap_pt_vector_rbt_frame;
        geometry_msgs::Vector3Stamped range_vector_rbt_frame;
        gap_pt_vector_rbt_frame.header.frame_id = cfg.robot_frame_id;
		dynamic_gap::Gap g = _observed_gaps[int(std::floor(i / 2.0))];
        //std::cout << "obtaining gap pt vector" << std::endl;
        // convex is for simplified gaps, straight ldist is for raw gaps, may need separate one for both
		// THIS VECTOR IS IN THE ROBOT FRAME
		if (i % 2 == 0) {
			gap_pt_vector_rbt_frame.vector.x = (g.convex.convex_ldist) * cos(-((float) g.half_scan - g.convex.convex_lidx) / g.half_scan * M_PI);
			gap_pt_vector_rbt_frame.vector.y = (g.convex.convex_ldist) * sin(-((float) g.half_scan - g.convex.convex_lidx) / g.half_scan * M_PI);
		} else {
			gap_pt_vector_rbt_frame.vector.x = (g.convex.convex_rdist) * cos(-((float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
			gap_pt_vector_rbt_frame.vector.y = (g.convex.convex_rdist) * sin(-((float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
		}

        range_vector_rbt_frame.vector.x = gap_pt_vector_rbt_frame.vector.x - rbt_in_cam.pose.position.x;
        range_vector_rbt_frame.vector.y = gap_pt_vector_rbt_frame.vector.y - rbt_in_cam.pose.position.y;
        //std::cout << "gap_pt_vector_rbt_frame: " << gap_pt_vector_rbt_frame.vector.x << ", " << gap_pt_vector_rbt_frame.vector.y << std::endl;
		
        // pretty sure rbt in cam is always 0,0
        //std::cout << "rbt in cam pose: " << rbt_in_cam.pose.position.x << ", " << rbt_in_cam.pose.position.x << std::endl;
        //std::cout << "range vector rbt frame: " << range_vector_rbt_frame.vector.x << ", " << range_vector_rbt_frame.vector.x << std::endl;

        double beta_tilde = std::atan2(range_vector_rbt_frame.vector.y, range_vector_rbt_frame.vector.x);
		double norm = std::sqrt(std::pow(range_vector_rbt_frame.vector.x, 2) + pow(range_vector_rbt_frame.vector.y, 2));
		
		Matrix<double, 3, 1> y_tilde;
		y_tilde << 1.0 / norm, 
				   std::sin(beta_tilde),
				   std::cos(beta_tilde);
        // std::cout << "y_tilde: " << y_tilde << std::endl;

        Matrix<double, 1, 2> v_ego(current_cmd_vel.linear.x, current_cmd_vel.linear.y);
		if (i % 2 == 0) {
            //std::cout << "entering left model update" << std::endl;
			g.left_model->kf_update_loop(y_tilde, a, v_ego);
		} else {
            //std::cout << "entering right model update" << std::endl;
			g.right_model->kf_update_loop(y_tilde, a, v_ego);
		}
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    void Planner::update_models(std::vector<dynamic_gap::Gap>& _observed_gaps) {
        for (int i = 0; i < 2*_observed_gaps.size(); i++) {
            // std::cout << "update gap model: " << i << std::endl;
            update_model(i, _observed_gaps);
            //std::cout << "" << std::endl;
		}
    }
    
    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Transform the msg to odom frame
        if(msg->header.frame_id != cfg.odom_frame_id)
        {
            geometry_msgs::TransformStamped robot_pose_odom_trans = tfBuffer.lookupTransform(cfg.odom_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;

            tf2::doTransform(in_pose, out_pose, robot_pose_odom_trans);
            sharedPtr_pose = out_pose.pose;
        }
        else
        {
            sharedPtr_pose = msg->pose.pose;
        }
    }

    bool Planner::setGoal(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (plan.size() == 0) return true;
        final_goal_odom = *std::prev(plan.end());
        tf2::doTransform(final_goal_odom, final_goal_odom, map2odom);

        // Store New Global Plan to Goal Selector
        goalselector->setGoal(plan);
        
        trajvisualizer->globalPlanRbtFrame(goalselector->getOdomGlobalPlan());

        // Find Local Goal
        goalselector->updateLocalGoal(map2rbt);
        // return local goal (odom) frame
        auto new_local_waypoint = goalselector->getCurrentLocalGoal(rbt2odom);

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
        try { 
            auto traj = goalselector->getRelevantGlobalPlan(map2rbt);
            geometry_msgs::PoseArray pub_traj;
            if (traj.size() > 0) {
                // Should be safe with this check
                pub_traj.header = traj.at(0).header;
            }
            for (auto trajpose : traj) {
                pub_traj.poses.push_back(trajpose.pose);
            }
            local_traj_pub.publish(pub_traj);
        } catch (...) {
            ROS_FATAL_STREAM("getRelevantGlobalPlan");
        }

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

    [[deprecated("Use Proper trajectory scoring instead")]]
    void Planner::vectorSelectGap(dynamic_gap::Gap & selected_gap)
    {
        dynamic_gap::Gap result = trajArbiter->returnAndScoreGaps();
        selected_gap = result;
        return;
    }

    /*
    // CHANGING SO NOT PASSING BY REFERENCE HERE JUST TO SEE WHAT CHANGES DO, FIX LATER
    std::vector<dynamic_gap::Gap> Planner::gapManipulateByCategory(std::vector<dynamic_gap::Gap> _observed_gaps, Matrix<double, 1, 2> v_ego) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<dynamic_gap::Gap> manip_set;
        manip_set = _observed_gaps;

        // we want to change the models in here

        try {
            for (size_t i = 0; i < manip_set.size(); i++) {
                std::cout << "MANIPULATING GAP " << i << std::endl;
                if (manip_set.at(i).getCategory() == "expanding") { 
                    std::cout << "manipulating an expanding gap" << std::endl;
                    // reduceGap, don't really need if we are not using models?

                    // AGC, set new model to static
                    gapManip->convertAxialGap(manip_set.at(i), v_ego); // swing axial inwards

                    // radial extend: don't really need if we are not using models?
                    // (how would we do radial extend if gap is non-convex?)
                } else if (manip_set.at(i).getCategory() == "closing") {
                    std::cout << "manipulating a static/closing gap" << std::endl;
                    // AGC (TODO: what to do about model of pivot point?)
                    // radial extend ( and extend models)
                } else { // translating
                    std::cout << "manipulating a translating gap" << std::endl;
                    // AGC (TODO: what to do about model of pivot point?)
                    // radial extend ( and extend models)
                }
            }    
        } catch(...) {
            ROS_FATAL_STREAM("gapManipulateByCategory");
        }   

        gapvisualizer->drawManipGaps(manip_set, std::string("manip"));
        return manip_set;
    }
    */

    std::vector<dynamic_gap::Gap> Planner::gapManipulate(std::vector<dynamic_gap::Gap> _observed_gaps, Matrix<double, 1, 2> v_ego) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<dynamic_gap::Gap> manip_set;
        manip_set = _observed_gaps;

        // we want to change the models in here

        try {
            for (size_t i = 0; i < manip_set.size(); i++)
            {
                // a copied pointer still points to same piece of memory, so we need to copy the models
                manip_set.at(i).left_model = new dynamic_gap::MP_model(*_observed_gaps.at(i).left_model);
                manip_set.at(i).right_model = new dynamic_gap::MP_model(*_observed_gaps.at(i).right_model);

                std::cout << "MANIPULATING GAP " << i << std::endl;
                gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal()); // cut down from non convex 
                gapManip->convertAxialGap(manip_set.at(i), v_ego); // swing axial inwards
                gapManip->radialExtendGap(manip_set.at(i)); // extend behind robot
            }
        } catch(...) {
            ROS_FATAL_STREAM("gapManipulate");
        }

        gapvisualizer->drawManipGaps(manip_set, std::string("manip"));
        return manip_set;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<double>> Planner::initialTrajGen(std::vector<dynamic_gap::Gap>& vec, std::vector<geometry_msgs::PoseArray>& res, std::vector<dynamic_gap::Gap>& current_raw_gaps, std::vector<std::vector<double>>& res_time_traj) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<geometry_msgs::PoseArray> ret_traj(vec.size());
        std::vector<std::vector<double>> ret_time_traj(vec.size());
        std::vector<std::vector<double>> ret_traj_scores(vec.size());
        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam; // lc as local copy
        try {
            for (size_t i = 0; i < vec.size(); i++) {
                std::cout << "generating traj for gap: " << i << std::endl;
                // std::cout << "starting generate trajectory with rbt_in_cam_lc: " << rbt_in_cam_lc.pose.position.x << ", " << rbt_in_cam_lc.pose.position.y << std::endl;
                // std::cout << "goal of: " << vec.at(i).goal.x << ", " << vec.at(i).goal.y << std::endl;
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple;
                return_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_cmd_vel);
                return_tuple = gapTrajSyn->forwardPassTrajectory(return_tuple);

                std::cout << "scoring trajectory for gap: " << i << std::endl;
                ret_traj_scores.at(i) = trajArbiter->scoreTrajectory(std::get<0>(return_tuple), std::get<1>(return_tuple), current_raw_gaps);
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

        std::vector<double> result_score(prr.size());
        /*
                        std::cout "iterating through traj scores" << std::endl;
                for (size_t j = 0; j < counts; j++) {
                    std::cout << score.at(i).at(j) << ", " << std::endl;
                }
                std::cout << "" << std::endl;
        */
        try {
            if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < result_score.size(); i++) {
                // ROS_WARN_STREAM("prr(" << i << "): size " << prr.at(i).poses.size());
                int counts = std::min(cfg.planning.num_feasi_check, int(score.at(i).size()));

                result_score.at(i) = std::accumulate(score.at(i).begin(), score.at(i).begin() + counts, double(0));
                result_score.at(i) = prr.at(i).poses.size() == 0 ? -std::numeric_limits<double>::infinity() : result_score.at(i);
                std::cout << "for gap " << i << ", returning score of " << result_score.at(i) << " of length: " << counts << std::endl;
            }
        } catch (...) {
            ROS_FATAL_STREAM("pickTraj");
        }

        auto iter = std::max_element(result_score.begin(), result_score.end());
        int idx = std::distance(result_score.begin(), iter);

        if (result_score.at(idx) == -std::numeric_limits<double>::infinity()) {
            std::cout << "all -infinity" << std::endl;
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (auto val : result_score) {
                ROS_INFO_STREAM("Score: " << val);
            }
            ROS_INFO_STREAM("------------------");
        }

        std::cout << "picking gap: " << idx << std::endl;

        return idx;
    }

    geometry_msgs::PoseArray Planner::compareToOldTraj(geometry_msgs::PoseArray incoming, dynamic_gap::Gap incoming_gap, std::vector<int> feasible_gap_model_indices, std::vector<dynamic_gap::Gap>& current_raw_gaps, std::vector<double> time_arr) {
        auto curr_traj = getCurrentTraj();
        auto curr_time_arr = getCurrentTimeArr();


        try {
            double curr_time = ros::Time::now().toSec();
            std::cout << "current traj length: " << curr_traj.poses.size() << std::endl;
            std::cout << "current gap indices: " << getCurrentLeftGapIndex() << ", " << getCurrentRightGapIndex() << std::endl;
            //std::cout << "current time length: " << curr_time_arr.size() << std::endl;
            std::cout << "incoming traj length: " << incoming.poses.size() << std::endl;
            //std::cout << "incoming time length: " << time_arr.size() << std::endl;
            if (curr_left_model != NULL) {
                Matrix<double,4,1> curr_left_cart_state = curr_left_model->get_cartesian_state(); 
                std::cout << "current left point: " << curr_left_cart_state[0] << ", " << curr_left_cart_state[1] << std::endl;
            } else {
                std::cout << "current left point: NULL" << std::endl;
            }
            if (curr_right_model != NULL) {
                Matrix<double,4,1> curr_right_cart_state = curr_right_model->get_cartesian_state(); 
                std::cout << "current right point: " << curr_right_cart_state[0] << ", " << curr_right_cart_state[1] << std::endl;
            } else {
                std::cout << "current right point: NULL" << std::endl;
            }
            // Both Args are in Odom frame
            auto incom_rbt = gapTrajSyn->transformBackTrajectory(incoming, odom2rbt);
            incom_rbt.header.frame_id = cfg.robot_frame_id;
            // why do we have to rescore here?
            std::cout << "scoring incoming trajectory" << std::endl;
            auto incom_score = trajArbiter->scoreTrajectory(incom_rbt, time_arr, current_raw_gaps);
            // int counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));

            int counts = std::min(cfg.planning.num_feasi_check, (int) incom_score.size());
            auto incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));

            std::cout << "incom_subscore: " << incom_subscore << std::endl;
            if (curr_traj.poses.size() == 0) {
                if (incom_subscore == -std::numeric_limits<double>::infinity()) {
                    std::cout << "TRAJECTORY CHANGE TO EMPTY: curr traj length 0, incoming score of -infinity" << std::endl;
                    ROS_WARN_STREAM("Incoming score of negative infinity");
                    auto empty_traj = geometry_msgs::PoseArray();
                    std::vector<double> empty_time_arr;
                    setCurrentTraj(empty_traj);
                    setCurrentTimeArr(empty_time_arr);
                    setCurrentLeftModel(incoming_gap.left_model);
                    setCurrentRightModel(incoming_gap.right_model);
                    return empty_traj;
                } else if (incoming.poses.size() == 0) {
                    std::cout << "TRAJECTORY CHANGE TO EMPTY: curr traj length 0, incoming traj length of 0" << std::endl;        
                    ROS_WARN_STREAM("Incoming traj length 0");
                    auto empty_traj = geometry_msgs::PoseArray();
                    std::vector<double> empty_time_arr;
                    setCurrentTraj(empty_traj);
                    setCurrentTimeArr(empty_time_arr);
                    setCurrentLeftModel(NULL);
                    setCurrentRightModel(NULL);
                    return empty_traj;
                } else {
                    std::cout << "TRAJECTORY CHANGE TO INCOMING: curr traj length 0, incoming score finite" << std::endl;
                    setCurrentTraj(incoming);
                    setCurrentTimeArr(time_arr);
                    setCurrentLeftModel(incoming_gap.left_model);
                    setCurrentRightModel(incoming_gap.right_model);
                    trajectory_pub.publish(incoming);
                    ROS_WARN_STREAM("Old Traj length 0");
                    prev_traj_switch_time = curr_time;
                    return incoming;
                }
            } 

            auto curr_rbt = gapTrajSyn->transformBackTrajectory(curr_traj, odom2rbt);
            curr_rbt.header.frame_id = cfg.robot_frame_id;
            int start_position = egoTrajPosition(curr_rbt);
            geometry_msgs::PoseArray reduced_curr_rbt = curr_rbt;
            std::vector<double> reduced_curr_time_arr = curr_time_arr;
            reduced_curr_rbt.poses = std::vector<geometry_msgs::Pose>(curr_rbt.poses.begin() + start_position, curr_rbt.poses.end());
            reduced_curr_time_arr = std::vector<double>(curr_time_arr.begin() + start_position, curr_time_arr.end());
            if (reduced_curr_rbt.poses.size() < 2) {
                std::cout << "TRAJECTORY CHANGE TO INCOMING: old traj length less than 2" << std::endl;
                ROS_WARN_STREAM("Old Traj short");
                setCurrentTraj(incoming);
                setCurrentTimeArr(time_arr);
                setCurrentLeftModel(incoming_gap.left_model);
                setCurrentRightModel(incoming_gap.right_model);
                prev_traj_switch_time = curr_time;
                return incoming;
            }

            counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incoming.poses.size(), reduced_curr_rbt.poses.size()));
            // std::cout << "counts: " << counts << std::endl;

            incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));
            std::cout << "incoming subscore: " << incom_subscore << std::endl;

            std::cout << "scoring reduced current trajectory" << std::endl;
            auto curr_score = trajArbiter->scoreTrajectory(reduced_curr_rbt, reduced_curr_time_arr, current_raw_gaps);
            auto curr_subscore = std::accumulate(curr_score.begin(), curr_score.begin() + counts, double(0));
            std::cout << "reduced current subscore: " << curr_subscore << std::endl;

            std::vector<std::vector<double>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incom_score;
            ret_traj_scores.at(1) = curr_score;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incom_rbt;
            viz_traj.at(1) = reduced_curr_rbt;
            trajvisualizer->pubAllScore(viz_traj, ret_traj_scores);

            ROS_DEBUG_STREAM("Curr Score: " << curr_subscore << ", incom Score:" << incom_subscore);


            if (curr_subscore == -std::numeric_limits<double>::infinity() && incom_subscore == -std::numeric_limits<double>::infinity()) {
                std::cout << "TRAJECTORY CHANGE TO EMPTY: both -infinity" << std::endl;
                ROS_WARN_STREAM("Both Failed");
                auto empty_traj = geometry_msgs::PoseArray();
                std::vector<double> empty_time_arr;
                setCurrentTraj(empty_traj);
                setCurrentTimeArr(empty_time_arr);
                setCurrentLeftModel(NULL);
                setCurrentRightModel(NULL);
                return empty_traj;
            }

            double oscillation_pen = counts; // * std::exp(-(curr_time - prev_traj_switch_time)/5.0);
            if (incom_subscore > (curr_subscore + oscillation_pen)) {
                std::cout << "TRAJECTORY CHANGE TO INCOMING: swapping trajectory" << std::endl;
                ROS_WARN_STREAM("Swap to new for better score: " << incom_subscore << " > " << curr_subscore << " + " << oscillation_pen);
                setCurrentTraj(incoming);
                setCurrentTimeArr(time_arr);
                setCurrentLeftModel(incoming_gap.left_model);
                setCurrentRightModel(incoming_gap.right_model);
                trajectory_pub.publish(incoming);
                prev_traj_switch_time = curr_time;
                return incoming;
            }

            bool left_index_count = std::count(feasible_gap_model_indices.begin(), feasible_gap_model_indices.end(), getCurrentLeftGapIndex());
            bool right_index_count = std::count(feasible_gap_model_indices.begin(), feasible_gap_model_indices.end(), getCurrentRightGapIndex());
            // std::cout << "left index count: " << left_index_count << ", right index count: " << right_index_count << std::endl; 
            // FORCING OFF CURRENT TRAJ IF NO LONGER FEASIBLE
            
            if ((left_index_count == 0) || (right_index_count == 0)) {
                if (incoming.poses.size() > 0) {
                    std::cout << "TRAJECTORY CHANGE TO INCOMING: curr exec gap no longer feasible. left: " <<  left_index_count << ", right: " << right_index_count << std::endl;
                    ROS_WARN_STREAM("Swap to incoming, current trajectory no longer through feasible gap");
                    setCurrentTraj(incoming);
                    setCurrentTimeArr(time_arr);
                    setCurrentLeftModel(incoming_gap.left_model);
                    setCurrentRightModel(incoming_gap.right_model);
                    trajectory_pub.publish(incoming);
                    prev_traj_switch_time = curr_time;
                    return incoming;   
                } else {
                    auto empty_traj = geometry_msgs::PoseArray();
                    std::vector<double> empty_time_arr;
                    setCurrentTraj(empty_traj);
                    setCurrentTimeArr(empty_time_arr);
                    setCurrentLeftModel(NULL);
                    setCurrentRightModel(NULL);
                    return empty_traj; 
                }    
            }
            
            
            
            std::cout << "keeping current trajectory" << std::endl;

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

    void Planner::setCurrentGapIndices(int _left_idx, int _right_idx) {
        curr_exec_left_idx = _left_idx;
        curr_exec_right_idx = _right_idx;
        return;
    }

    void Planner::setCurrentLeftModel(dynamic_gap::MP_model * _left_model) {
        curr_left_model = _left_model;
    }

    void Planner::setCurrentRightModel(dynamic_gap::MP_model * _right_model) {
        curr_right_model = _right_model;
    }

    int Planner::getCurrentLeftGapIndex() {
        if (curr_left_model != NULL) {
            return curr_left_model->get_index();
        } else {
            return -1;
        }
    }
    
    int Planner::getCurrentRightGapIndex() {
        if (curr_right_model != NULL) {
            return curr_right_model->get_index();
        } else {
            return -1;
        }    }

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
        a << 0.0, 0.0;
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
        if (traj.poses.size() < 2){
            ROS_WARN_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 3");
            return geometry_msgs::Twist();
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

        // returns a TrajPlan (poses and velocities, velocities are zero here)
        auto orig_ref = trajController->trajGen(traj);
        
        // get point along trajectory to target/move towards
        ctrl_idx = trajController->targetPoseIdx(curr_pose, orig_ref);
        nav_msgs::Odometry ctrl_target_pose;
        ctrl_target_pose.header = orig_ref.header;
        ctrl_target_pose.pose.pose = orig_ref.poses.at(ctrl_idx);
        ctrl_target_pose.twist.twist = orig_ref.twist.at(ctrl_idx);

        sensor_msgs::LaserScan stored_scan_msgs;
        if (cfg.planning.projection_inflated) {
            stored_scan_msgs = *sharedPtr_inflatedlaser.get();
        } else {
            stored_scan_msgs = *sharedPtr_laser.get();
        }

        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;
        auto cmd_vel = trajController->controlLaw(curr_pose, ctrl_target_pose, stored_scan_msgs, rbt_in_cam_lc);
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

    geometry_msgs::PoseArray Planner::getPlanTrajectory() {
        // double begin_time = ros::Time::now().toSec();
        updateTF();
        Matrix<double, 1, 2> v_ego(current_cmd_vel.linear.x, current_cmd_vel.linear.y);

        std::cout << "RAW GAP ASSOCIATION AND UPDATING" << std::endl;
        std::vector<dynamic_gap::Gap> curr_raw_gaps = get_curr_raw_gaps();
        raw_association = gapassociator->associateGaps(curr_raw_gaps, previous_raw_gaps, model_idx, "raw", v_ego);
        update_models(curr_raw_gaps);
        std::cout << "FINISHED RAW GAP ASSOCIATION AND UPDATING" << std::endl;

        std::cout << "SIMPLIFIED GAP ASSOCIATION AND UPDATING" << std::endl;
        std::vector<dynamic_gap::Gap> curr_observed_gaps = get_curr_observed_gaps();
        association = gapassociator->associateGaps(curr_observed_gaps, previous_gaps, model_idx, "simplified", v_ego);
        update_models(curr_observed_gaps);
        gapvisualizer->drawGapsModels(curr_observed_gaps);
        std::cout << "FINISHED SIMPLIFIED GAP ASSOCIATION AND UPDATING" << std::endl;

        std::cout << "GAP FEASIBILITY CHECK" << std::endl;
        std::vector<dynamic_gap::Gap> feasible_gap_set = gapManip->gapSetFeasibilityCheck(curr_observed_gaps);
        std::cout << "FINISHED GAP FEASIBILITY CHECK" << std::endl;

        // need to have here for models
        gapvisualizer->drawGaps(curr_observed_gaps, std::string("simp"));

        std::cout << "STARTING GAP MANIPULATE" << std::endl;
        auto manip_gap_set = gapManipulate(feasible_gap_set, v_ego);
        std::cout << "FINISHED GAP MANIPULATE" << std::endl;

        std::cout << "SET GAP GOAL" << std::endl;
        for (size_t i = 0; i < manip_gap_set.size(); i++) {
            std::cout << "setting goal for gap: " << i << std::endl;
            gapManip->setGapWaypoint(manip_gap_set.at(i), goalselector->rbtFrameLocalGoal()); // incorporating dynamic gap types
        }
        goalvisualizer->drawGapGoals(manip_gap_set);
        std::cout << "FINISHED SET GAP GOAL" << std::endl;
        
        std::cout << "INITIAL TRAJ GEN/SCORING" << std::endl;
        std::vector<geometry_msgs::PoseArray> traj_set;
        std::vector<std::vector<double>> time_set;
        auto score_set = initialTrajGen(manip_gap_set, traj_set, curr_raw_gaps, time_set);
        std::cout << "FINISHED INITIAL TRAJ GEN/SCORING" << std::endl;

        std::cout << "PICK TRAJ" << std::endl;
        auto traj_idx = pickTraj(traj_set, score_set);
        std::cout << "PICK TRAJ" << std::endl;

        std::cout << "COMPARE TO OLD TRAJ" << std::endl;
        geometry_msgs::PoseArray chosen_traj;
        std::vector<double> chosen_time_arr;
        dynamic_gap::Gap chosen_gap;
        if (traj_idx >= 0) {
            chosen_traj = traj_set[traj_idx];
            chosen_time_arr = time_set[traj_idx];
            chosen_gap = feasible_gap_set[traj_idx];
        } else {
            chosen_traj = geometry_msgs::PoseArray();
            chosen_gap = dynamic_gap::Gap();
        }

        std::vector<int> feasible_gap_model_indices;
        std::cout << "feasible gap indices: ";
        for (size_t i = 0; i < manip_gap_set.size(); i++) {
            std::cout << feasible_gap_set.at(i).left_model->get_index() << ", " << feasible_gap_set.at(i).right_model->get_index() << ", ";
            feasible_gap_model_indices.push_back(manip_gap_set.at(i).left_model->get_index());
            feasible_gap_model_indices.push_back(manip_gap_set.at(i).right_model->get_index());
        }
        std::cout << "" << std::endl;

        auto final_traj = compareToOldTraj(chosen_traj, chosen_gap, feasible_gap_model_indices, curr_raw_gaps, chosen_time_arr);
        std::cout << "FINISHED COMPARE TO OLD TRAJ" << std::endl;

        previous_gaps = curr_observed_gaps;
        previous_raw_gaps = curr_raw_gaps;

        // ROS_WARN_STREAM("getPlanTrajectory time: " << ros::Time::now().toSec() - begin_time);
        return final_traj;
    }

    bool Planner::recordAndCheckVel(geometry_msgs::Twist cmd_vel) {
        double val = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.linear.y) + std::abs(cmd_vel.angular.z);
        log_vel_comp.push_back(val);
        double cum_vel_sum = std::accumulate(log_vel_comp.begin(), log_vel_comp.end(), double(0));
        bool ret_val = cum_vel_sum > 1.0 || !log_vel_comp.full();
        if (!ret_val && !cfg.man.man_ctrl) {
            std::cout << "-------- planning failed -------" << std::endl;
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        previous_cmd_vel = current_cmd_vel;
        current_cmd_vel = cmd_vel;
        return ret_val || cfg.man.man_ctrl;
    }

}