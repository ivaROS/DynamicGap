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

        // std::vector<int> association;

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
        dyn_egocircle_pub = nh.advertise<sensor_msgs::LaserScan>("dyn_egocircle", 5);

        rbt_accel_sub = nh.subscribe(cfg.robot_frame_id + "/acc", 100, &Planner::robotAccCB, this);
        // rbt_vel_sub = nh.subscribe(cfg.robot_frame_id + "/current_vel", 100, &Planner::robotVelCB, this);

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

        current_rbt_vel = geometry_msgs::Twist();
        rbt_vel_min1 = geometry_msgs::Twist();

        rbt_accel = geometry_msgs::Twist();
        rbt_accel_min1 = geometry_msgs::Twist();
        
        init_val = 0;
        model_idx = &init_val;
        prev_traj_switch_time = ros::Time::now().toSec();
        init_time = ros::Time::now().toSec(); 

        curr_left_model = NULL;
        curr_right_model = NULL;
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

    void Planner::robotAccCB(boost::shared_ptr<geometry_msgs::Twist const> msg)
    {
        rbt_accel = *msg;
        /*
        geometry_msgs::Vector3Stamped rbt_accel_rbt_frame;

        rbt_accel_rbt_frame.vector.x = msg->linear_acceleration.x;
        rbt_accel_rbt_frame.vector.y = msg->linear_acceleration.y;
        rbt_accel_rbt_frame.vector.z = msg->angular_velocity.z; //in the ROS msg its omega, but this value is alpha

        rbt_accel[0] = rbt_accel_rbt_frame.vector.x;
        rbt_accel[1] = rbt_accel_rbt_frame.vector.y;
        rbt_accel[2] = rbt_accel_rbt_frame.vector.z;
        */
    }

    /*
    void Planner::robotVelCB(boost::shared_ptr<geometry_msgs::Twist const> msg) {
        current_rbt_vel = *msg;
        // std::cout << "from STDR, vel in robot frame: " << msg->linear.x << ", " << msg->linear.y << std::endl;
    }
    */

    // running at 30 Hz
    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sharedPtr_laser = msg;

        if (cfg.planning.planning_inflated && sharedPtr_inflatedlaser) {
            msg = sharedPtr_inflatedlaser;
        }

        // ROS_INFO_STREAM(msg.get()->ranges.size());

        try {
            boost::mutex::scoped_lock gapset(gapset_mutex);
            //Matrix<double, 1, 3> rbt_vel_t(current_rbt_vel.linear.x, current_rbt_vel.linear.y, current_rbt_vel.angular.z);
            //Matrix<double, 1, 3> rbt_acc_t(rbt_accel.linear.x, rbt_accel.linear.y, rbt_accel.angular.z);

            Matrix<double, 1, 3> rbt_vel_tmin1(rbt_vel_min1.linear.x, rbt_vel_min1.linear.y, rbt_vel_min1.angular.z);
            Matrix<double, 1, 3> rbt_acc_tmin1(rbt_accel_min1.linear.x, rbt_accel_min1.linear.y, rbt_accel_min1.angular.z);

            Matrix<double, 1, 3> v_ego = rbt_vel_tmin1;
            Matrix<double, 1, 3> a_ego = rbt_acc_tmin1;

            // getting raw gaps
            raw_gaps = finder->hybridScanGap(msg);
            gapvisualizer->drawGaps(raw_gaps, std::string("raw"));

            //std::cout << "robot pose: " << std::endl;
            //std::cout << "x,y: " << sharedPtr_pose.position.x << ", " << sharedPtr_pose.position.y;
            //std::cout << ", quat: " << sharedPtr_pose.orientation.x << ", " << sharedPtr_pose.orientation.y << ", " << sharedPtr_pose.orientation.z << ", " << sharedPtr_pose.orientation.w << std::endl;
            //std::cout << "RAW GAP ASSOCIATING" << std::endl;
            // ASSOCIATE GAPS PASSES BY REFERENCE
            raw_distMatrix = gapassociator->associateGaps(raw_association, raw_gaps, previous_raw_gaps, model_idx, "raw", v_ego);
            //std::cout << "RAW GAP UPDATING" << std::endl;
            associated_raw_gaps = update_models(raw_gaps, v_ego, a_ego);

            observed_gaps = finder->mergeGapsOneGo(msg, raw_gaps);

            //std::cout << "SIMPLIFIED GAP ASSOCIATING" << std::endl;
            simp_distMatrix = gapassociator->associateGaps(simp_association, observed_gaps, previous_gaps, model_idx, "simplified", v_ego);
            //std::cout << "SIMPLIFIED GAP UPDATING" << std::endl;
            associated_observed_gaps = update_models(observed_gaps, v_ego, a_ego);

            if (print_associations) {
                printGapAssociations(observed_gaps, previous_gaps, simp_association);
                print_associations = false;
            }

            previous_gaps = associated_observed_gaps;
            previous_raw_gaps = associated_raw_gaps;



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

        rbt_vel_min1 = current_rbt_vel;
        rbt_accel_min1 = rbt_accel;

    }
    
    void Planner::update_model(int i, std::vector<dynamic_gap::Gap>& _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego) {
        // boost::mutex::scoped_lock gapset(gapset_mutex);

        // UPDATING MODELS
        //std::cout << "obtaining gap g" << std::endl;
		geometry_msgs::Vector3Stamped gap_pt_vector_rbt_frame;
        geometry_msgs::Vector3Stamped range_vector_rbt_frame;
        gap_pt_vector_rbt_frame.header.frame_id = cfg.robot_frame_id;
		dynamic_gap::Gap g = _observed_gaps[int(std::floor(i / 2.0))];
        //std::cout << "obtaining gap pt vector" << std::endl;
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
		double range_tilde = std::sqrt(std::pow(range_vector_rbt_frame.vector.x, 2) + pow(range_vector_rbt_frame.vector.y, 2));
		
		Matrix<double, 2, 1> laserscan_measurement;
		laserscan_measurement << range_tilde, 
				                 beta_tilde;
        // std::cout << "y_tilde: " << y_tilde << std::endl;

        // Matrix<double, 1, 3> v_ego(current_rbt_vel.linear.x, current_rbt_vel.linear.y, current_rbt_vel.angular.z);
        if (i % 2 == 0) {
            //std::cout << "entering left model update" << std::endl;
            g.left_model->kf_update_loop(laserscan_measurement, _a_ego, _v_ego);
        } else {
            //std::cout << "entering right model update" << std::endl;
            g.right_model->kf_update_loop(laserscan_measurement, _a_ego, _v_ego);
        }
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    std::vector<dynamic_gap::Gap> Planner::update_models(std::vector<dynamic_gap::Gap> _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego) {
        std::vector<dynamic_gap::Gap> associated_observed_gaps = _observed_gaps;

        for (int i = 0; i < 2*associated_observed_gaps.size(); i++) {
            //std::cout << "update gap model: " << i << std::endl;
            update_model(i, associated_observed_gaps, _v_ego, _a_ego);
            //std::cout << "" << std::endl;
		}

        return associated_observed_gaps;
    }
    
    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
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
            // this is what happens
            //std::cout << "odom msg is in odom frame" << std::endl;
            // doing odom to rbt transform to get velocity in rbt frame
            /*
            geometry_msgs::TransformStamped odom_vel_robot_trans = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.odom_frame_id, ros::Time(0));

            geometry_msgs::Vector3Stamped in_vel, out_vel;
            in_vel.header = msg->header;
            in_vel.vector = msg->twist.twist.linear;
            tf2::doTransform(in_vel, out_vel, odom_vel_robot_trans);
            //std::cout << "rbt vel: " << out_vel.vector.x << ", " << out_vel.vector.y << std::endl;
            */
            sharedPtr_pose = msg->pose.pose;
        }

        current_rbt_vel = msg->twist.twist;
        
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

    std::vector<dynamic_gap::Gap> Planner::gapManipulate(std::vector<dynamic_gap::Gap> _observed_gaps, Matrix<double, 1, 2> v_ego, std::vector<dynamic_gap::Gap> curr_raw_gaps) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<dynamic_gap::Gap> manip_set;
        manip_set = _observed_gaps;

        // we want to change the models in here

        try {
            for (size_t i = 0; i < manip_set.size(); i++)
            {
                // a copied pointer still points to same piece of memory, so we need to copy the models
                // if we want to have a separate model for the manipulated gap
                
                //manip_set.at(i).left_model = new dynamic_gap::cart_model(*_observed_gaps.at(i).left_model);
                //manip_set.at(i).right_model = new dynamic_gap::cart_model(*_observed_gaps.at(i).right_model);

                /*
                sensor_msgs::LaserScan stored_scan = *sharedPtr_laser.get();
                sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();
                dynamic_laser_scan.angle_increment = stored_scan.angle_increment;
                dynamic_laser_scan.header = stored_scan.header;
                std::vector<float> dynamic_ranges(stored_scan.ranges.size());
                dynamic_laser_scan.ranges = dynamic_ranges;
                */

                std::cout << "MANIPULATING INITIAL GAP " << i << std::endl;
                // MANIPULATE POINTS AT T=0
                gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true); // cut down from non convex 
                gapManip->convertAxialGap(manip_set.at(i), v_ego, true); // swing axial inwards
                gapManip->radialExtendGap(manip_set.at(i), true); // extend behind robot

                gapManip->setGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal(), true); // incorporating dynamic gap types
                /*
                if (curr_raw_gaps.size() > 0) {
                    trajArbiter->recoverDynamicEgoCircle(0.0, manip_set.at(i).gap_lifespan, curr_raw_gaps, dynamic_laser_scan);
                }
                dynamic_laser_scan.range_min = *std::min_element(dynamic_laser_scan.ranges.begin(), dynamic_laser_scan.ranges.end());
                */
                std::cout << "MANIPULATING TERMINAL GAP " << i << std::endl;
                //if (!manip_set.at(i).gap_crossed && !manip_set.at(i).gap_closed) {
                gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal(), false); // cut down from non convex 
                gapManip->convertAxialGap(manip_set.at(i), v_ego, false); // swing axial inwards
                //}
                gapManip->radialExtendGap(manip_set.at(i), false); // extend behind robot
                // gapManip->clipGapByLaserScan(manip_set.at(i));
                gapManip->setTerminalGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal()); // incorporating dynamic gap type
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
                return_tuple = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc, current_rbt_vel);
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

        // poses here are in odom frame 
        std::vector<double> result_score(prr.size());
        try {
            if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < result_score.size(); i++) {
                // ROS_WARN_STREAM("prr(" << i << "): size " << prr.at(i).poses.size());
                int counts = std::min(cfg.planning.num_feasi_check, int(score.at(i).size()));

                result_score.at(i) = std::accumulate(score.at(i).begin(), score.at(i).begin() + counts, double(0));
                result_score.at(i) = prr.at(i).poses.size() == 0 ? -std::numeric_limits<double>::infinity() : result_score.at(i);
                std::cout << "for gap " << i << ", returning score of " << result_score.at(i) << std::endl;
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
            
            // Both Args are in Odom frame
            auto incom_rbt = gapTrajSyn->transformBackTrajectory(incoming, odom2rbt);
            incom_rbt.header.frame_id = cfg.robot_frame_id;
            // why do we have to rescore here?
            std::cout << "~~scoring incoming trajectory~~" << std::endl;
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
            std::cout << "~~comparing incoming with current~~" << std::endl; 
            std::cout << "re-scoring incoming trajectory" << std::endl;
            incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));
            std::cout << "incoming subscore: " << incom_subscore << std::endl;

            std::cout << "scoring current trajectory" << std::endl;
            auto curr_score = trajArbiter->scoreTrajectory(reduced_curr_rbt, reduced_curr_time_arr, current_raw_gaps);
            auto curr_subscore = std::accumulate(curr_score.begin(), curr_score.begin() + counts, double(0));
            std::cout << "current subscore: " << curr_subscore << std::endl;

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

            //bool left_index_count = std::count(feasible_gap_model_indices.begin(), feasible_gap_model_indices.end(), getCurrentLeftGapIndex());
            //bool right_index_count = std::count(feasible_gap_model_indices.begin(), feasible_gap_model_indices.end(), getCurrentRightGapIndex());
            // std::cout << "left index count: " << left_index_count << ", right index count: " << right_index_count << std::endl; 
            // FORCING OFF CURRENT TRAJ IF NO LONGER FEASIBLE
            if (getCurrentLeftGapIndex() != incoming_gap.left_model->get_index() || getCurrentRightGapIndex() != incoming_gap.right_model->get_index()) {
                if (incoming.poses.size() > 0) {
                    std::cout << "TRAJECTORY CHANGE TO INCOMING: curr exec gap no longer feasible. current left: " <<  getCurrentLeftGapIndex() << ", incoming left: " << incoming_gap.left_model->get_index() << ", current right: " << getCurrentRightGapIndex() << ", incoming right: " << incoming_gap.right_model->get_index() << std::endl;
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

    void Planner::setCurrentLeftModel(dynamic_gap::cart_model * _left_model) {
        curr_left_model = _left_model;
    }

    void Planner::setCurrentRightModel(dynamic_gap::cart_model * _right_model) {
        curr_right_model = _right_model;
    }

    int Planner::getCurrentLeftGapIndex() {
        // std::cout << "get current left" << std::endl;
        if (curr_left_model != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return curr_left_model->get_index();
        } else {
            // std::cout << "model is null" << std::endl;
            return -1;
        }
    }
    
    int Planner::getCurrentRightGapIndex() {
        // std::cout << "get current right" << std::endl;
        if (curr_right_model != NULL) {
            // std::cout << "model is not  null" << std::endl;
            return curr_right_model->get_index();
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
        rbt_accel = geometry_msgs::Twist();
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
        Matrix<double, 1, 2> v_ego(current_rbt_vel.linear.x, current_rbt_vel.linear.y);

        print_associations = true;

        std::vector<dynamic_gap::Gap> curr_raw_gaps = associated_raw_gaps;
        std::vector<dynamic_gap::Gap> curr_observed_gaps = associated_observed_gaps;
        
        std::vector<dynamic_gap::Gap> prev_raw_gaps = previous_raw_gaps;
        std::vector<dynamic_gap::Gap> prev_observed_gaps = previous_gaps;  

        std::vector<int> curr_simp_association = simp_association;   
        
        std::cout << "current robot velocity. Linear: " << current_rbt_vel.linear.x << ", " << current_rbt_vel.linear.y << ", angular: " << current_rbt_vel.angular.z << std::endl;
        // std::cout << "curr_raw_gaps:" << std::endl;
        //printGapModels(curr_raw_gaps);
        


        std::cout << "current simplified gaps:" << std::endl;
        printGapModels(curr_observed_gaps);

        std::cout << "drawing models" << std::endl;
        gapvisualizer->drawGapsModels(curr_observed_gaps);

        std::cout << "GAP FEASIBILITY CHECK" << std::endl;
        std::vector<dynamic_gap::Gap> feasible_gap_set = gapManip->gapSetFeasibilityCheck(curr_observed_gaps);
        std::cout << "FINISHED GAP FEASIBILITY CHECK" << std::endl;

        // need to have here for models
        gapvisualizer->drawGaps(curr_observed_gaps, std::string("simp"));

        std::cout << "STARTING GAP MANIPULATE" << std::endl;
        auto manip_gap_set = gapManipulate(feasible_gap_set, v_ego, curr_raw_gaps);
        std::cout << "FINISHED GAP MANIPULATE" << std::endl;

        // pruning overlapping gaps?
        for (size_t i = 0; i < (manip_gap_set.size() - 1); i++)
        {
            dynamic_gap::Gap current_gap = manip_gap_set.at(i);
            int prev_gap_idx = (i == 0) ? manip_gap_set.size() - 1 : i-1;
            dynamic_gap::Gap prev_gap = manip_gap_set.at(prev_gap_idx);
            int next_gap_idx = (i == (manip_gap_set.size() - 1)) ? 0 :i+1;
            dynamic_gap::Gap next_gap = manip_gap_set.at(next_gap_idx);

            std::cout << "previous: (" << prev_gap.convex.convex_lidx << ", " << prev_gap.convex.convex_ldist << "), (" << prev_gap.convex.convex_ridx << ", " << prev_gap.convex.convex_rdist << ")" << std::endl;
            std::cout << "current: (" << current_gap.convex.convex_lidx << ", " << current_gap.convex.convex_ldist << "), (" << current_gap.convex.convex_ridx << ", " << current_gap.convex.convex_rdist << ")" << std::endl;
            std::cout << "next: (" << next_gap.convex.convex_lidx << ", " << next_gap.convex.convex_ldist << "), (" << next_gap.convex.convex_ridx << ", " << next_gap.convex.convex_rdist << ")" << std::endl;

            if (prev_gap.convex.convex_lidx < current_gap.convex.convex_lidx && prev_gap.convex.convex_ridx > current_gap.convex.convex_lidx) {
                std::cout << "manipulated gap overlap, previous gap right: " << prev_gap.convex.convex_ridx << ", current gap left: " << current_gap.convex.convex_lidx << std::endl;
                if (prev_gap.convex.convex_rdist > current_gap.convex.convex_ldist) {
                    manip_gap_set.erase(manip_gap_set.begin() + prev_gap_idx);
                } else {
                    manip_gap_set.erase(manip_gap_set.begin() + i);
                }
                i = -1; // to restart for loop
            } 
            if (next_gap.convex.convex_lidx > current_gap.convex.convex_lidx && next_gap.convex.convex_lidx < current_gap.convex.convex_ridx) {
                std::cout << "manipulated gap overlap, current gap right: " << current_gap.convex.convex_ridx << ", next gap left: " << next_gap.convex.convex_lidx << std::endl;
                if (next_gap.convex.convex_ldist > current_gap.convex.convex_rdist) {
                    manip_gap_set.erase(manip_gap_set.begin() + next_gap_idx);
                } else {
                    manip_gap_set.erase(manip_gap_set.begin() + i);
                }
                i = -1; // to restart for loop
            }
        }
        std::cout << "SIMPLIFIED INITIAL AND TERMINAL POINTS FOR FEASIBLE GAPS" << std::endl;
        double x1, x2, y1, y2;

        for (size_t i = 0; i < feasible_gap_set.size(); i++)
        {
            std::cout << "gap " << i << std::endl;
            dynamic_gap::Gap g = feasible_gap_set.at(i);
            x1 = (g._ldist) * cos(-((float) g.half_scan - g._left_idx) / g.half_scan * M_PI);
            y1 = (g._ldist) * sin(-((float) g.half_scan - g._left_idx) / g.half_scan * M_PI);
            x2 = (g._rdist) * cos(-((float) g.half_scan - g._right_idx) / g.half_scan * M_PI);
            y2 = (g._rdist) * sin(-((float) g.half_scan - g._right_idx) / g.half_scan * M_PI);
            std::cout << "initial, polar L/R: (" << g._left_idx << ", " << g._ldist << "), (" << g._right_idx << ", " << g._rdist << "), cart L/R: (" << x1 << ", " << y1 << "), (" << x2 << ", " << y2 << ")" << std::endl; 
            x1 = (g.terminal_ldist) * cos(-((float) g.half_scan - g.terminal_lidx) / g.half_scan * M_PI);
            y1 = (g.terminal_ldist) * sin(-((float) g.half_scan - g.terminal_lidx) / g.half_scan * M_PI);
            x2 = (g.terminal_rdist) * cos(-((float) g.half_scan - g.terminal_ridx) / g.half_scan * M_PI);
            y2 = (g.terminal_rdist) * sin(-((float) g.half_scan - g.terminal_ridx) / g.half_scan * M_PI);
            std::cout << "terminal, polar L/R: (" << g.terminal_lidx << ", " << g.terminal_ldist << "), (" << g.terminal_ridx << ", " << g.terminal_rdist << "), cart L/R: (" << x1 << ", " << y1 << "), (" << x2 << ", " << y2 << ")" << std::endl; 
            std::cout << "~~~" << std::endl;
        }

        std::cout << "MANIPULATED INITIAL AND TERMINAL POINTS FOR FEASIBLE GAPS" << std::endl;
        for (size_t i = 0; i < manip_gap_set.size(); i++)
        {
            std::cout << "gap " << i << std::endl;
            dynamic_gap::Gap g = manip_gap_set.at(i);
            x1 = (g.convex.convex_ldist) * cos(-((float) g.half_scan - g.convex.convex_lidx) / g.half_scan * M_PI);
            y1 = (g.convex.convex_ldist) * sin(-((float) g.half_scan - g.convex.convex_lidx) / g.half_scan * M_PI);
            x2 = (g.convex.convex_rdist) * cos(-((float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
            y2 = (g.convex.convex_rdist) * sin(-((float) g.half_scan - g.convex.convex_ridx) / g.half_scan * M_PI);
            std::cout << "initial, polar L/R: (" << g.convex.convex_lidx << ", " << g.convex.convex_ldist << "), (" << g.convex.convex_ridx << ", " << g.convex.convex_rdist << "), cart L/R: (" << x1 << ", " << y1 << "), (" << x2 << ", " << y2 << ")" << std::endl; 
            std::cout << "initial goal: (" << g.goal.x << ", " << g.goal.y << ")" << std::endl;
            x1 = (g.convex.terminal_ldist) * cos(-((float) g.half_scan - g.convex.terminal_lidx) / g.half_scan * M_PI);
            y1 = (g.convex.terminal_ldist) * sin(-((float) g.half_scan - g.convex.terminal_lidx) / g.half_scan * M_PI);
            x2 = (g.convex.terminal_rdist) * cos(-((float) g.half_scan - g.convex.terminal_ridx) / g.half_scan * M_PI);
            y2 = (g.convex.terminal_rdist) * sin(-((float) g.half_scan - g.convex.terminal_ridx) / g.half_scan * M_PI);
            std::cout << "terminal, polar L/R: (" << g.convex.terminal_lidx << ", " << g.convex.terminal_ldist << "), (" << g.convex.terminal_ridx << ", " << g.convex.terminal_rdist << "), cart L/R: (" << x1 << ", " << y1 << "), (" << x2 << ", " << y2 << ")" << std::endl; 
            std::cout << "terminal goal: (" << g.terminal_goal.x << ", " << g.terminal_goal.y << ")" << std::endl;
            std::cout << "~~~" << std::endl;
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

        // ROS_WARN_STREAM("getPlanTrajectory time: " << ros::Time::now().toSec() - begin_time);
        return final_traj;
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
            if (i >= 0 && association[i] >= 0) {
                int current_gap_idx = int(std::floor(pair[0] / 2.0));
                int previous_gap_idx = int(std::floor(pair[1] / 2.0));
                if (pair[0] % 2 == 0) {  // curr left
                    current_gaps.at(current_gap_idx).getSimplifiedLCartesian(curr_x, curr_y);
                } else { // curr right
                    current_gaps.at(current_gap_idx).getSimplifiedRCartesian(curr_x, curr_y);
                }
                if (pair[1] % 2 == 0) { // prev left
                    previous_gaps.at(previous_gap_idx).getSimplifiedLCartesian(prev_x, prev_y);
                } else { // prev right
                    previous_gaps.at(previous_gap_idx).getSimplifiedRCartesian(prev_x, prev_y);
                }
                std::cout << "From (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << simp_distMatrix[pair[0]][pair[1]] << std::endl;
            } else {
                std::cout << "From NULL to (" << curr_x << ", " <<  curr_y << ")";
            }
        }
    }
    void Planner::printGapModels(std::vector<dynamic_gap::Gap> gaps) {
        for (size_t i = 0; i < gaps.size(); i++)
        {
            std::cout << "gap " << i << std::endl;
            dynamic_gap::Gap g = gaps.at(i);
            Matrix<double, 4, 1> left_state = g.left_model->get_state();
            Matrix<double, 4, 1> right_state = g.right_model->get_state();
            std::cout << "left model: (" << left_state[0] << ", " << left_state[1] << ", " << left_state[2] << ", " << left_state[3] << ")" << std::endl;
            std::cout << "right model: (" << right_state[0] << ", " << right_state[1] << ", " << right_state[2] << ", " << right_state[3] << ")" << std::endl;
        }
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
        return ret_val || cfg.man.man_ctrl;
    }

}