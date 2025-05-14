#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <dynamic_gap/utils/Utils.h>    

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
            std::string odom_frame_id = "TBD"; /**< Odometry frame ID */
            std::string robot_frame_id = "TBD"; /**< Robot frame ID */
            std::string sensor_frame_id = "TBD"; /**< Sensor frame ID */
            std::string odom_topic = "TBD"; /**< Odometry ROS topic */
            std::string acc_topic = "TBD"; /**< IMU ROS topic */
            std::string scan_topic = "TBD"; /**< Laser scan ROS topic */
            std::string ped_topic = "/pedsim_simulator/simulated_agents"; /**< Topic for pedestrian states (cheat mode only) */

            /**
            * \brief Hyperparameters for ego-robot
            */
            struct Robot 
            {
                float r_inscr = 0.2; /**< Inscribed radius of robot */
                float vx_absmax = 1.0; /**< Maximum linear speed in x-direction for robot */
                float vy_absmax = 1.0; /**< Maximum linear speed in y-direction for robot */
                float vang_absmax = 1.0; /**< Maximum angular speed for robot */            
            } rbt;

            /**
            * \brief Hyperparameters for laser scan
            */
            struct Scan
            {
                // will get overriden in updateParamFromScan
                float angle_min = -M_PI; /**< minimum angle value in scan */
                float angle_max = M_PI; /**< maximum angle value in scan */
                int half_scan = 256; /**< Half of total rays in scan (integer) */
                float half_scan_f = 256.; /**< Half of total rays in scan (float) */
                int full_scan = 512; /**< Total ray count in scan (integer) */
                float full_scan_f = 512.; /**< Total ray count in scan (float) */
                float angle_increment = (2 * M_PI) / (full_scan_f - 1); /**< Angular increment between consecutive scan indices */
                float range_min = 0.03; /**< Minimum detectable range in scan */
                float range_max = -1e10; /**< Maximum detectable range in scan */
            } scan;

            /**
            * \brief Hyperparameters for planning mode
            */
            struct PlanningMode 
            {
                int gap_prop = 1; /**< 0 - old, 1 - new */
                int pursuit_guidance_method = 1; /**< 0 - pure pursuit, 1 - parallel navigation */
                // bool heading = false; /**< Boolean for if robot tracks path headings or not */
                bool holonomic = false; /**< Boolean for if robot is holonomic or not */
                bool future_scan_propagation = true; /**< Flag for enacting future scan propagation */
                bool egocircle_prop_cheat = false; /**< Flag for enacting future scan propagation through cheating */
                bool projection_operator = true; /**< Boolean for if planner should apply projection operator */
                bool gap_feasibility_check = true; /**< Flag for enacting gap feasibility checking */
                bool perfect_gap_models = false; /**< Flag for using perfect gap models */
                bool social_cost_function = true; /**< false - old without cost function, true - new */
                float social_cost_weight = .1; /**< weight for social cost during pose evaluation for traj selection */
            } planning;            

            /**
            * \brief Hyperparameters for manual teleoperation control
            */
            struct Control 
            {
                // only ONE of these should be true
                bool man_ctrl = false; /**< Flag for enacting manual teleoperation control */
                bool mpc_ctrl = false; /**< Flag for enacting MPC control */
                bool feedback_ctrl = true; /**< Flag for enacting feedback control */
                int ctrl_ahead_pose = 2; /**< Number of poses ahead of closest pose in current trajectory to track */
            } ctrl;

            /**
            * \brief Hyperparameters for navigation goal
            */
            struct Goal 
            {
                float xy_global_goal_tolerance = 0.2; /**< Distance threshold for global goal */
                float yaw_global_goal_tolerance = M_PI; /**< Angular distance threshold for global goal */
                float xy_waypoint_tolerance = 0.1; /**< Distance threshold for global path local waypoint */
            } goal;

            /**
            * \brief Hyperparameters for gap association
            */
            struct GapAssociation 
            {
                float assoc_thresh = 1.0; /**< Distance threshold for gap association */
            } gap_assoc;           

            /**
            * \brief Hyperparameters for gap manipulation
            */
            struct GapManipulation 
            {
                float rgc_angle = 1.0; /**< Rotation amount (radians) for RGC step  */              
                // bool radial_extend = true; /**< Flag for if gap manipulator should apply radial extension */
            } gap_manip;

            /**
            * \brief Hyperparameters for trajectory generation
            */
            struct Trajectory 
            {
                float integrate_maxt = 10.0; /**< Trajectory generation time horizon (in seconds) */
                float integrate_stept = 0.5; /**< Trajectory generation time step (in seconds) */
                float max_pose_to_scan_dist = 0.5; /**< Minimum robot to environment distance for which we should penalize in trajectory scoring */
                float Q = 1.0; /**< Scaling hyperparameter for trajectory pose-wise cost */
                float pen_exp_weight = 5.0; /**< Standard deviation hyperparameter in exponential term of trajectory pose-wise cost */
                float inf_ratio = 1.5; /**< Inflation ratio for planner */
                float Q_f = 1.0; /**< Scaling hyperparamter for terminal pose cost based on distance from global plan local waypoint */
            } traj;            
            
            /**
            * \brief Hyperparameters for projection operator
            */
            struct ProjectionParam 
            {
                float k_po_x = 1.0; /**< Proportional gain in x-direction for projection operator */
                float r_unity = 0.5; /**< Robot to environment distance at which projection operator takes on a value of 1 */
                float r_zero = 1.0; /**< Robot to environment distance at which projection operator takes on a value of 0 */
            } projection;

            /**
            * \brief Load in planner hyperparameters from node handle (specified in launch file and yamls)
            */
            void loadRosParamFromNodeHandle(const std::string & name);

            /**
            * \brief Load in hyperparameters from current laser scan
            */
            void updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr);
    };
}