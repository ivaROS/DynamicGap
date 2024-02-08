#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap 
{
    /**
    * Hyperparameter list for dynamic_gap planner
    */
    class DynamicGapConfig 
    {
        // DEFAULT VALUES, CAN BE OVERRIDEN THROUGH ROS PARAMETERS
        public:
            std::string map_frame_id = "map"; /**< Map frame ID */
            std::string odom_frame_id = "odom"; /**< Odometry frame ID */
            std::string robot_frame_id = "robot0"; /**< Robot frame ID */
            std::string sensor_frame_id = "camera_link"; /**< Sensor frame ID */

            /**
            * \brief Hyperparameters for gap visualization
            */
            struct GapVisualization 
            {
                int min_resoln = 2; /**< Frequency of scan indices to visualize (plot one every min_resoln scan point) */
            } gap_viz;
            
            /**
            * \brief Hyperparameters for ego-robot
            */
            struct Robot 
            {
                float r_inscr = 0.2; /**< Inscribed radius of robot */
            } rbt;

            /**
            * \brief Hyperparameters for planning environment
            */
            struct Environment
            {
                int num_agents = 0; /**< Total number of agents in environment */
            } env;

            /**
            * \brief Hyperparameters for laser scan
            */
            struct Scan
            {
                float angle_min = -M_PI; /**< minimum angle value in scan */
                float angle_max = M_PI; /**< maximum angle value in scan */
                int half_scan = 256; /**< Half of total rays in scan (integer) */
                float half_scan_f = 256.; /**< Half of total rays in scan (float) */
                int full_scan = 512; /**< Total ray count in scan (integer) */
                float full_scan_f = 512.; /**< Total ray count in scan (float) */
                float angle_increment = (2 * M_PI) / (full_scan_f - 1); /**< Angular increment between consecutive scan indices */
                float range_min = 0.0; /**< Minimum detectable range in scan */
                float range_max = 4.99; /**< Maximum detectable range in scan */
            } scan;

            /**
            * \brief Hyperparameters for planning mode
            */
            struct PlanningMode 
            {
                bool holonomic = true; /**< Boolean for if robot is holonomic or not */
                bool projection_operator = false; /**< Boolean for if planner should apply projection operator */
                int halt_size = 5; /**< Size of command velocity buffer */
                bool gap_feasibility_check = true; /**< Flag for enacting gap feasibility checking */
                bool future_scan_propagation = true; /**< Flag for enacting future scan propagation */
                bool egocircle_prop_cheat = true; /**< Flag for enacting future scan propagation through cheating */
            } planning;            

            /**
            * \brief Hyperparameters for manual teleoperation control
            */
            struct ManualControl 
            {
                bool man_ctrl = false; /**< Flag for enacting manual teleoperation control */
            } man;

            /**
            * \brief Hyperparameters for navigation goal
            */
            struct Goal 
            {
                float goal_tolerance = 0.2; /**< Distance threshold for global goal */
                float waypoint_tolerance = 0.1; /**< Distance threshold for global path local waypoint */
            } goal;

            /**
            * \brief Hyperparameters for gap detection
            */
            struct GapDetection
            {
                int max_idx_diff = 256; /**< Maximum size in scan indices of a merged gap during gap simplification */
            } gap_det;

            /**
            * \brief Hyperparameters for gap association
            */
            struct GapAssociation 
            {
                float assoc_thresh = 0.15; /**< Distance threshold for gap association */
            } gap_assoc;           

            /**
            * \brief Hyperparameters for gap manipulation
            */
            struct GapManipulation 
            {
                float epsilon1 = 0.20; /**< Denominator for setting radial gap pivot angle */
                float epsilon2 = 0.30; /**< Numerator for setting radial gap pivot angle */
                float reduction_threshold = M_PI; /**< Maximum allowable size of gap */
                bool radial_extend = true; /**< Flag for if gap manipulator should apply radial extension */
                bool radial_convert = true; /**< Flag for if gap manipulator should apply radial gap conversion */
            } gap_manip;

            /**
            * \brief Hyperparameters for trajectory generation
            */
            struct Trajectory 
            {
                bool synthesized_frame = true; /**< Flag for if trajectory should be represented in sensor frame */
                float integrate_maxt = 5.0; /**< Trajectory generation time horizon (in seconds) */
                float integrate_stept = 0.5; /**< Trajectory generation time step (in seconds) */
                float max_pose_pen_dist = 0.5; /**< Minimum robot to environment distance for which we should penalize in trajectory scoring */
                float cobs = -1.0; /**< Scaling hyperparameter for trajectory pose-wise cost */
                float pose_exp_weight = 5.0; /**< Standard deviation hyperparameter in exponential term of trajectory pose-wise cost */
                float inf_ratio = 1.21; /**< Inflation ratio for planner */
                float terminal_weight = 10.0; /**< Scaling hyperparamter for terminal pose cost based on distance from global plan local waypoint */
                int num_curve_points = 50; /**< Number of points to use to discretize gap boundary */
                int num_extended_gap_origin_points = 6; /**< Number of points to use to discretize radial gap extension */
            } traj;            

            /**
            * \brief Hyperparameters for trajectory tracking
            */
            struct ControlParams 
            {
                float k_fb_x = 0.5; /**< Proportional feedback gain in x-direction */
                float k_fb_y = 0.5; /**< Proportional feedback gain in y-direction */
                float k_fb_theta = 0.5; /**< Proportional feedback for robot yaw */
                int ctrl_ahead_pose = 2; /**< Number of poses ahead of closest pose in current trajectory to track */
                float vx_absmax = 0.5; /**< Maximum linear speed in x-direction for robot */
                float vy_absmax = 0.5; /**< Maximum linear speed in y-direction for robot */
                float vang_absmax = 1.5; /**< Maximum angular speed for robot */
                float ax_absmax = 3.0; /**< Maximum linear acceleration in x-direction for robot */
                float ay_absmax = 3.0; /**< Maximum linear acceleration in y-direction for robot */
                float aang_absmax = 3.0; /**< Maximum angular acceleration for robot */
            } control;
            
            /**
            * \brief Hyperparameters for projection operator
            */
            struct ProjectionParam 
            {
                float k_po_x = 1.0; /**< Proportional gain in x-direction for projection operator */
                float k_po_theta = 1.0; /**< Proportional gain for yaw for projection operator */

                float r_unity = 0.35; /**< Robot to environment distance at which projection operator takes on a value of 1 */
                float r_zero = 1.0; /**< Robot to environment distance at which projection operator takes on a value of 0 */

                float cbf_param = 0.1; /**< Scaling hyperparamter for CBF */            
                float k_CBF = 1.0; /**< Proportional gain for CBF */
            } projection;

        /**
        * \brief Load in planner hyperparameters from node handle (specified in launch file and yamls)
        */
        void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

        /**
        * \brief Load in hyperparameters from current laser scan
        */
        void updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr);
    };
}