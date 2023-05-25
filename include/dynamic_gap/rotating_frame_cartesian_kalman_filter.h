#ifndef ROTATING_FRAME_CARTESIAN_KALMAN_FILTER_H 
#define ROTATING_FRAME_CARTESIAN_KALMAN_FILTER_H

#include <ros/console.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions> // for matrix exponential

#include <geometry_msgs/TwistStamped.h>

namespace dynamic_gap 
{
    class rotatingFrameCartesianKalmanFilter
    {
        public:
            rotatingFrameCartesianKalmanFilter(const int & filter_index, 
                                            const double & initial_range, 
                                            const double & initial_bearing,
                                            const ros::Time & t_kf_update,
                                            const geometry_msgs::TwistStamped & last_ego_rbt_vel,
                                            const geometry_msgs::TwistStamped & last_ego_rbt_acc);
            void update(Eigen::Matrix<double, 2, 1> laserscan_measurement, 
                        const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied, 
                        const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied, 
                        const ros::Time & t_kf_update,
                        bool print);
            
            Eigen::Vector4d get_model_state();
            geometry_msgs::TwistStamped get_v_ego();
            int get_index();

            void freeze_robot_vel();
            Eigen::Vector4d get_frozen_cartesian_state();
            Eigen::Vector2d get_x_tilde(); 

            void set_side(std::string _side); 
            std::string get_side(); 

            void frozen_state_propagate(double froz_dt);
            void rewind_propagate(double rew_dt);

            Eigen::Vector4d get_frozen_modified_polar_state();
            void set_rewind_state();
            Eigen::Vector4d get_rewind_cartesian_state();
            Eigen::Vector4d get_rewind_modified_polar_state();


        private:
            void processEgoRobotVelsAndAccs(const ros::Time & t_update);
            Eigen::Vector4d integrate();
            void linearize(int idx);
            void discretizeQ(int idx); 

            bool print;

            int filter_index;

            Eigen::Matrix<double, 2, 4> H; // observation matrix
            Eigen::Matrix<double, 4, 2> H_transpose;

            double R_scalar; // scalar for measurement noise matrix
            Eigen::Matrix2d R_k; // measurement noise matrix
            Eigen::Matrix4d Q_k, Q_1, Q_2, Q_3; // process noise matrix
            Eigen::Matrix4d dQ; // discretized process noise matrix
            Eigen::Matrix4d Q_tmp; // discretized process noise matrix

            Eigen::Matrix4d P_kmin1_plus, P_k_minus, P_k_plus, P_intermediate, P_tmp; // covariance matrix
            Eigen::Matrix<double, 4, 2> G_k; // kalman gain
            Eigen::Vector2d x_tilde; // measurement
            Eigen::Vector2d innovation; 
            Eigen::Vector2d residual;

            Eigen::Matrix2d tmp_mat;
            Eigen::Matrix4d eyes;

            Eigen::Matrix4d A, STM; // state dynamics matrix and state transition matrix

            Eigen::Vector4d x_hat_kmin1_plus, x_hat_k_minus, x_hat_k_plus, frozen_x, rewind_x;

            ros::Time t_last_update;
            std::vector<geometry_msgs::TwistStamped> ego_rbt_vels;
            std::vector<geometry_msgs::TwistStamped> ego_rbt_accs;        
            geometry_msgs::TwistStamped last_ego_rbt_vel;
            geometry_msgs::TwistStamped last_ego_rbt_acc;

            std::string side;
    };
} // namespace dynamic_gap

#endif