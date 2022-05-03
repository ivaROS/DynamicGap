#include <ros/ros.h>
#include <dynamic_gap/gap.h>
#include <dynamic_gap/gap_associator.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include "nav_msgs/Odometry.h"
#include "dynamic_gap/TrajPlan.h"
#include <dynamic_gap/helper.h>
#include <dynamic_gap/gap.h>
// #include <dynamic_gap/trajectory_follower.h>
#include <dynamic_gap/gap_utils.h>

#include <dynamic_gap/dynamicgap_config.h>
#include <dynamic_gap/visualization.h>
#include <dynamic_gap/goal_selector.h>
#include <dynamic_gap/trajectory_scoring.h>
#include <dynamic_gap/gap_manip.h>
#include <dynamic_gap/trajectory_controller.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <omp.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_gap/dgConfig.h>

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#ifndef PLANNER_H
#define PLANNER_H

namespace dynamic_gap
{
    class Planner
    {
    private:
        geometry_msgs::TransformStamped map2rbt;        // Transform
        geometry_msgs::TransformStamped rbt2map;
        geometry_msgs::TransformStamped odom2rbt;
        geometry_msgs::TransformStamped rbt2odom;
        geometry_msgs::TransformStamped map2odom;
        geometry_msgs::TransformStamped cam2odom;
        geometry_msgs::TransformStamped rbt2cam;

        geometry_msgs::PoseStamped goal_rbt_frame;
        geometry_msgs::PoseStamped curr_pose_odom;
        geometry_msgs::PoseStamped rbt_in_rbt;
        geometry_msgs::PoseStamped rbt_in_cam;
        geometry_msgs::Twist rbt_vel_in_rbt;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener *tfListener;
        tf2_ros::TransformBroadcaster goal_br;

        ros::NodeHandle nh;
        ros::Publisher local_traj_pub;
        ros::Publisher trajectory_pub;
        ros::Publisher gap_vis_pub;
        ros::Publisher selected_gap_vis_pub;
        ros::Publisher ni_traj_pub;
        ros::Publisher ni_traj_pub_other;

        ros::Subscriber rbt_accel_sub;
        ros::Subscriber rbt_vel_sub;

        std::vector<int> association;
        std::vector<int> raw_association;


        // Goals and stuff
        // double goal_orientation;
        geometry_msgs::Pose current_pose_;
        geometry_msgs::PoseStamped local_waypoint_odom; // local_waypoint, 
        geometry_msgs::PoseStamped final_goal_odom;

        // Gaps:
        std::vector<dynamic_gap::Gap> raw_gaps;
        std::vector<dynamic_gap::Gap> observed_gaps;

        std::vector<dynamic_gap::Gap> previous_gaps;
        std::vector<dynamic_gap::Gap> previous_raw_gaps;

        std::vector<dynamic_gap::Gap> associated_raw_gaps;
        std::vector<dynamic_gap::Gap> associated_observed_gaps;
        
        std::vector<dynamic_gap::Gap> merged_gaps;
        std::vector<dynamic_gap::Gap> selected_gap_set;
        std::vector<dynamic_gap::Gap> ftg_gaps;
        std::vector<dynamic_gap::Gap> safe_gaps_left;
        std::vector<dynamic_gap::Gap> safe_gaps_right;
        std::vector<dynamic_gap::Gap> safe_gaps_central;
        std::vector<dynamic_gap::Gap> safe_gaps;

        dynamic_gap::GapUtils *finder;
        dynamic_gap::GapVisualizer *gapvisualizer;
        dynamic_gap::GoalSelector *goalselector;
        dynamic_gap::TrajectoryVisualizer *trajvisualizer;
        dynamic_gap::GoalVisualizer *goalvisualizer;
        dynamic_gap::TrajectoryArbiter *trajArbiter;
        dynamic_gap::GapTrajGenerator *gapTrajSyn;
        dynamic_gap::GapManipulator *gapManip;
        dynamic_gap::TrajectoryController *trajController;
        dynamic_gap::GapAssociator *gapassociator;

        // Status
        bool hasGoal = false;
        bool _initialized = false;

        geometry_msgs::PoseArray pose_arr;
        geometry_msgs::PoseArray pose_arr_odom;

        // std::vector<turtlebot_trajectory_generator::ni_state> ctrl;
        int ctrl_idx = 0;

        geometry_msgs::Pose sharedPtr_pose;
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser;
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_inflatedlaser;

        ros::WallTime last_time;
        dynamic_gap::TrajPlan ni_ref, orig_ref;

        // Dynamic Reconfigure
        boost::shared_ptr<dynamic_reconfigure::Server<dynamic_gap::dgConfig> > dynamic_recfg_server;
        dynamic_reconfigure::Server<dynamic_gap::dgConfig>::CallbackType f;

        bool replan = true;
        
        dynamic_gap::DynamicGapConfig cfg;

        boost::mutex gapset_mutex;

        geometry_msgs::PoseArray curr_executing_traj;
        std::vector<double> curr_executing_time_arr;
        int curr_exec_left_idx;
        int curr_exec_right_idx;

        boost::circular_buffer<double> log_vel_comp;

        geometry_msgs::Twist previous_cmd_vel;
        geometry_msgs::Twist current_cmd_vel;
        geometry_msgs::Twist current_rbt_vel;
        Matrix<double, 1, 2> a;

        std::string frame;
        double angle_increment;
        double angle_min;
        double half_scan;
        double min_safe_dist;

        int init_val;
        int * model_idx;
        double prev_traj_switch_time;
        double init_time;

        dynamic_gap::MP_model * curr_left_model;
        dynamic_gap::MP_model * curr_right_model;

    public:
        Planner();

        ~Planner();

        /**
         * Set ros Buffer and etc.
         * 
         * @param None
         * @return initialization success / failure
         */
        bool initialize(const ros::NodeHandle&);

        /**
         * Return initialization status
         * @param None
         * @return bool initialization status
         */
        bool initialized();

        /**
         * Check if reached goal using euclidean dist
         * @param None, internally stored goal location and robot position
         * @return bool reached
         */
        bool isGoalReached();

        /**
         * call back function to laserscan, externally linked
         * @param msg laser scan msg
         * @return None, laser scan msg stored locally
         */
        void laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
        void inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

        void robotImuCB(boost::shared_ptr<sensor_msgs::Imu const> msg);
        void robotVelCB(boost::shared_ptr<geometry_msgs::Twist const> msg);

        /**
         * call back function to pose, pose information obtained here only used when a new goal is used
         * @param msg pose msg
         * @return None
         */
        void poseCB(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * Interface function for receiving global plan
         * @param plan, vector of PoseStamped
         * @return boolean type on whether successfully registered goal
         */
        bool setGoal(const std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * update all tf transform at the beginning of every planning cycle
         * @param None, all tf received via TF
         * @return None, all registered via internal variables in TransformStamped
         */
        void updateTF();

        /**
         * select the gap to pass through based on where the goal is
         * TODO: make this polymorphism so more than one strategy can be adopted
         * @param selected_gap that will be returned by the same variable
         * @return selected_gap via the passed in variable
         */
        void vectorSelectGap(dynamic_gap::Gap & selected_gap);

        /**
         * Generate ctrl command to a target pose
         * TODO: fix vector pop and get rid of pose_counter
         * @param pose_arr_odom
         * @return cmd_vel by assigning to pass by reference
         */
        geometry_msgs::Twist ctrlGeneration(geometry_msgs::PoseArray traj);
        
        /**
         * Take current observed gaps and perform gap conversion
         * @param None, directly taken from private variable space
         * @return gap_set, simplfied radial prioritized gaps
         */
        std::vector<dynamic_gap::Gap> gapManipulate(std::vector<dynamic_gap::Gap> _observed_gaps, Matrix<double, 1, 2> v_ego, std::vector<dynamic_gap::Gap> curr_raw_gaps);

        /**
         * 
         *
         */
        std::vector<std::vector<double>> initialTrajGen(std::vector<dynamic_gap::Gap>& vec, std::vector<geometry_msgs::PoseArray>& res, std::vector<dynamic_gap::Gap>& current_raw_gaps, std::vector<std::vector<double>>& res_time_traj);

        /**
         * Callback function to config object
         * @param incoming config
         * @param level Level of incoming config
         */
        void rcfgCallback(dynamic_gap::dgConfig &config, uint32_t level);

        /**
         * Pick the best trajectory from the current set
         * @param Vector of PoseArray
         * @param Vector of corresponding trajectory scores
         * @return the best trajectory
         */
        int pickTraj(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<double>> score);

        /**
         * Compare to the old trajectory and pick the best one
         * @param incoming trajectory
         * @return the best trajectory  
         */
        geometry_msgs::PoseArray compareToOldTraj(geometry_msgs::PoseArray incoming, dynamic_gap::Gap incoming_gap, std::vector<int> feasible_gap_model_indices, std::vector<dynamic_gap::Gap>& current_raw_gaps, std::vector<double> time_arr);

        /**
         * Setter and Getter of Current Trajectory, this is performed in the compareToOldTraj function
         */
        void setCurrentTraj(geometry_msgs::PoseArray);        
        geometry_msgs::PoseArray getCurrentTraj();

        void setCurrentTimeArr(std::vector<double>);
        std::vector<double> getCurrentTimeArr();

        void setCurrentGapIndices(int _left_idx, int _right_idx);
        int getCurrentLeftGapIndex();
        int getCurrentRightGapIndex();


        /**
         * Conglomeration of getting a plan Trajectory
         * @return the trajectory
         */
        geometry_msgs::PoseArray getPlanTrajectory();        

        /**
         * Gets the current position along the currently executing Trajectory
         */
        int egoTrajPosition(geometry_msgs::PoseArray curr);


        /**
         * Reset Planner, clears current observedSet
         */
        void reset();
        bool isReplan();
        void setReplan();

        /**
         * Check if the robot has been stuck
         * @param command velocity
         * @return False if robot has been stuck for the past cfg.planning.halt_size iterations
         */
        bool recordAndCheckVel(geometry_msgs::Twist cmd_vel);
    
        void update_model(int i, std::vector<dynamic_gap::Gap>&);
        std::vector<dynamic_gap::Gap> update_models(std::vector<dynamic_gap::Gap>);
        std::vector<dynamic_gap::Gap> get_curr_raw_gaps();
        std::vector<dynamic_gap::Gap> get_curr_observed_gaps();

        void setCurrentLeftModel(dynamic_gap::MP_model * _left_model);
        void setCurrentRightModel(dynamic_gap::MP_model * _right_model);

        std::vector<dynamic_gap::Gap> gapManipulateByCategory(std::vector<dynamic_gap::Gap> _observed_gaps, Matrix<double, 1, 2> v_ego);

    };
}

#endif