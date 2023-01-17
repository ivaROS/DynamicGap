#ifndef ODE_H
#define ODE_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TransformStamped.h>
// #include "geometry_msgs/Twist.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <geometry_msgs/Vector3Stamped.h>

// #include <tf/LinearMath/Matrix3x3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "tf/transform_datatypes.h"

namespace dynamic_gap {
    typedef boost::array<double, 8> state_type;
    //typedef state_type::index_range range;

    struct reachable_gap_APF {
        Eigen::Vector2d rel_left_vel, rel_right_vel, 
                        goal_pt_0, goal_pt_1;

        double v_lin_max, a_lin_max, rg, 
               theta_right, theta_left, thetax, thetag, new_theta, 
               a_x_rbt, a_y_rbt, a_x_rel, a_y_rel, v_nom,
               theta, eps, K_att; 
        bool _radial, past_gap_points, past_goal, past_left_point, past_right_point, pass_gap;
        Eigen::Vector2d init_rbt_pos, rbt, rel_right_pos, rel_left_pos, abs_left_pos, abs_right_pos, 
                        abs_goal_pos, rel_goal_pos, c_left, c_right, sub_goal_vec, v_des, v_cmd, v_raw, 
                        a_des, a_actual, nom_acc, nonrel_left_vel, nonrel_right_vel, nonrel_goal_vel;
        Eigen::Vector4d abs_left_state, abs_right_state, goal_state;

        Eigen::MatrixXd weights, all_centers, all_inward_norms, gradient_of_pti_wrt_rbt, centers_to_rbt;
        Eigen::VectorXd rowwise_sq_norms;

        reachable_gap_APF(Eigen::Vector2d init_rbt_pos, Eigen::Vector2d goal_pt_1,
                          double v_lin_max, Eigen::Vector2d nom_acc, Eigen::MatrixXd all_centers, Eigen::MatrixXd all_inward_norms, Eigen::MatrixXd weights,
                          Eigen::Vector2d nonrel_left_vel, Eigen::Vector2d nonrel_right_vel, Eigen::Vector2d nonrel_goal_vel) 
                          : init_rbt_pos(init_rbt_pos), goal_pt_1(goal_pt_1),
                            v_lin_max(v_lin_max), nom_acc(nom_acc), all_centers(all_centers), all_inward_norms(all_inward_norms), weights(weights),
                            nonrel_left_vel(nonrel_left_vel), nonrel_right_vel(nonrel_right_vel), nonrel_goal_vel(nonrel_goal_vel)
                        { 
                            eps = 0.0000001;              
                        }
        
        state_type adjust_state(const state_type &x) {
            // clipping velocities
            // taking norm of sin/cos norm vector
            state_type new_x = x;
            Eigen::Vector2d rbt_vel = clip_velocities(new_x[2], new_x[3], v_lin_max);
            new_x[2] = rbt_vel[0];
            new_x[3] = rbt_vel[1];

            return new_x;
        }

        Eigen::Vector2d clip_velocities(double x_vel, double y_vel, double x_lim) {
            // std::cout << "in clip_velocities with " << x_vel << ", " << y_vel << std::endl;
            Eigen::Vector2d original_vel(x_vel, y_vel);
            double abs_x_vel = std::abs(x_vel);
            double abs_y_vel = std::abs(y_vel);
            if (abs_x_vel <= x_lim && abs_y_vel <= x_lim) {
                // std::cout << "not clipping" << std::endl;
                return original_vel;
            } else {
                // std::cout << "max: " << vx_absmax << ", norm: " << original_vel.norm() << std::endl;
                Eigen::Vector2d clipped_vel = x_lim * original_vel / std::max(abs_x_vel, abs_y_vel);
                return clipped_vel;
            }
        }
        void operator()(const state_type &x, state_type &dxdt, const double t)
        { 
            // state_type new_x = adjust_state(x);
               
            // Just use the same state to be able to make these past checks
            rbt << x[0], x[1];
            abs_left_pos << x[2], x[3];
            abs_right_pos << x[4], x[5];
            abs_goal_pos << x[6], x[7];

            rel_left_pos = abs_left_pos - rbt;
            rel_right_pos = abs_right_pos - rbt;
            rel_goal_pos = abs_goal_pos - rbt;

            past_goal = abs_goal_pos.dot(rel_goal_pos) < 0; 
            past_left_point = abs_left_pos.dot(rel_left_pos) < 0;
            past_right_point = abs_right_pos.dot(rel_right_pos) < 0;
            
            if (_radial) {
                past_gap_points = past_left_point || past_right_point;
            } else {
                past_gap_points = past_left_point && past_right_point;
            }
            
            pass_gap = past_gap_points || past_goal;

            // ROS_INFO_STREAM("t: " << t);
            // ROS_INFO_STREAM("rbt state: " << x[0] << ", " << x[1]);
            // ROS_INFO_STREAM("left state: " << x[2] << ", " << x[3]);
            // ROS_INFO_STREAM("right state: " << x[4] << ", " << x[5]);
            // ROS_INFO_STREAM("goal state: " << x[6] << ", " << x[7]);

            if (pass_gap) {
                // ROS_INFO_STREAM("past gap");
                dxdt[0] = 0; dxdt[1] = 0; dxdt[2] = 0; dxdt[3] = 0; 
                dxdt[4] = 0; dxdt[5] = 0; dxdt[6] = 0; dxdt[7] = 0;
                return;
            } 

            // ROS_INFO_STREAM("left_pos: " << abs_left_pos[0] << ", " << abs_left_pos[1]);            
            // ROS_INFO_STREAM("right_pos: " << abs_right_pos[0] << ", " << abs_right_pos[1]);
            // ROS_INFO_STREAM("goal_pos: " << abs_goal_pos[0] << ", " << abs_goal_pos[1]);            

            // APF

            rg = rel_goal_pos.norm();

            K_att = - pow(rg, 2);
            // ROS_INFO_STREAM("attractive term: " << K_att);

            // Eigen::MatrixXd gradient_of_pti_wrt_centers(Kplus1, 2); // (2, Kplus1); //other one used is Kplus1, 2
   
            // ROS_INFO_STREAM("all_centers size: " << all_centers.rows() << ", " << all_centers.cols());
            // ROS_INFO_STREAM("all_inward_norms size: " << all_inward_norms.rows() << ", " << all_inward_norms.cols());

            centers_to_rbt = (-all_centers).rowwise() + rbt.transpose(); // size (r: Kplus1, c: 2)
            // ROS_INFO_STREAM("centers_to_rbt size: " << centers_to_rbt.rows() << ", " << centers_to_rbt.cols());

            Eigen::Index maxIndex;
            float maxNorm = centers_to_rbt.rowwise().norm().minCoeff(&maxIndex);
            /*
            for (int i = 0; i < centers_to_rbt.rows(); i++) {
                ROS_INFO_STREAM("row " << i << ": " << centers_to_rbt.row(i));
            }
            */
            if (maxIndex > 0) {
                // ROS_INFO_STREAM("min norm index: " << maxIndex);
                // ROS_INFO_STREAM("min norm center to rbt dir: " << centers_to_rbt.row(maxIndex));
                Eigen::Vector2d min_norm_center_to_rbt = centers_to_rbt.row(maxIndex);
                // ROS_INFO_STREAM("min norm center: " << min_norm_center);
                Eigen::Vector2d min_norm_inward_norm = all_inward_norms.row(maxIndex - 1);
                // ROS_INFO_STREAM("min norm inward dir: " << min_norm_inward_norm);
                if (min_norm_center_to_rbt.dot(min_norm_inward_norm) < 0) {
                    // ROS_INFO_STREAM("out of gap");
                    dxdt[0] = 0; dxdt[1] = 0; dxdt[2] = 0; dxdt[3] = 0; 
                    dxdt[4] = 0; dxdt[5] = 0; dxdt[6] = 0; dxdt[7] = 0;
                    return;
                }
            }

            rowwise_sq_norms = centers_to_rbt.rowwise().squaredNorm();
            // ROS_INFO_STREAM("rowwise_sq_norms size: " << rowwise_sq_norms.rows() << ", " << rowwise_sq_norms.cols());

            gradient_of_pti_wrt_rbt = centers_to_rbt.array().colwise() / (rowwise_sq_norms.array() + eps);          
            // ROS_INFO_STREAM("gradient_of_pti_wrt_rbt size: " << gradient_of_pti_wrt_rbt.rows() << ", " << gradient_of_pti_wrt_rbt.cols());

            // output of AHPF
            v_raw = K_att * gradient_of_pti_wrt_rbt.transpose() * weights;

            // Normalizing raw output and scaling by velocity limit
            v_des = v_lin_max * (v_raw / v_raw.norm());

            // CLIPPING DESIRED VELOCITIES
            // v_cmd = clip_velocities(v_des[0], v_des[1], v_lin_max);
            // ROS_INFO_STREAM("v_cmd: " << v_cmd[0] << ", " << v_cmd[1]);
            // set desired acceleration based on desired velocity

            dxdt[0] = v_des[0]; // rbt_x
            dxdt[1] = v_des[1]; // rbt_y

            dxdt[2] = nonrel_left_vel[0]; // left point r_x (absolute)
            dxdt[3] = nonrel_left_vel[1]; // left point r_y (absolute)

            dxdt[4] = nonrel_right_vel[0]; // right point r_x (absolute)
            dxdt[5] = nonrel_right_vel[1]; // right point r_y (absolute)

            dxdt[6] = nonrel_goal_vel[0]; // goal point r_x (absolute)
            dxdt[7] = nonrel_goal_vel[1]; // goal point r_y (absolute)
        }
    };
    
    struct g2g {
        double goal_vel_x, goal_vel_y, v_lin_max;
        bool goal_reached;
        g2g(double initial_goal_x, double initial_goal_y, 
            double terminal_goal_x, double terminal_goal_y, 
            double gap_lifespan, double v_lin_max)
        : goal_vel_x(goal_vel_x), goal_vel_y(goal_vel_y), v_lin_max(v_lin_max) 
        {
            goal_vel_x = (terminal_goal_x - initial_goal_x) / gap_lifespan; // absolute velocity (not relative to robot)
            goal_vel_y = (terminal_goal_y - initial_goal_y) / gap_lifespan;
        }

        Eigen::Vector2d clip_velocities(double x_vel, double y_vel, double x_lim) {
            // std::cout << "in clip_velocities with " << x_vel << ", " << y_vel << std::endl;
            Eigen::Vector2d original_vel(x_vel, y_vel);
            double abs_x_vel = std::abs(x_vel);
            double abs_y_vel = std::abs(y_vel);
            if (abs_x_vel <= x_lim && abs_y_vel <= x_lim) {
                // std::cout << "not clipping" << std::endl;
                return original_vel;
            } else {
                // std::cout << "max: " << vx_absmax << ", norm: " << original_vel.norm() << std::endl;
                Eigen::Vector2d clipped_vel = x_lim * original_vel / std::max(abs_x_vel, abs_y_vel);
                return clipped_vel;
            }
        }

        void operator() ( const state_type &x , state_type &dxdt , const double  t  )
        {
            // std::cout << "x: " << std::endl;
            double goal_norm = sqrt(pow(x[6] - x[0], 2) + pow(x[7] - x[1], 2));
            Eigen::Vector2d v_des(0.0, 0.0);

            // ROS_INFO_STREAM("t: " << t << ", x: " << x[0] << ", " << x[1] << ", goal: " << x[12] << ", " << x[13] << ", goal_norm: " << goal_norm);

            if (goal_norm < 0.1) {  // we want to make it so once robot reaches gap, trajectory ends, even if goal keeps moving
                // ROS_INFO_STREAM("t: " << t << ", stopping at x: " << x[0] << ", " << x[1] << ", goal: " << x[12] << ", " << x[13]);
                v_des(0) = 0;
                v_des(1) = 0;

                dxdt[0] = 0.0;
                dxdt[1] = 0.0;
                dxdt[6] = 0.0;
                dxdt[7] = 0.0;
                return;
            }


            v_des[0] = (x[6] - x[0]);
            v_des[1] = (x[7] - x[1]);
            
            Eigen::Vector2d v_cmd = clip_velocities(v_des[0], v_des[1], v_lin_max);

            dxdt[0] = v_cmd[0];
            dxdt[1] = v_cmd[1];
            dxdt[6] = goal_vel_x;
            dxdt[7] = goal_vel_y;
            return;
        }
    };

    // this is an observer?
    struct write_trajectory
    {
        geometry_msgs::PoseArray& _posearr;
        std::string _frame_id;
        double _scale;
        std::vector<double>& _timearr;

        write_trajectory(geometry_msgs::PoseArray& posearr, std::string frame_id, 
                         double scale, std::vector<double>& timearr): 
                         _posearr(posearr), _frame_id(frame_id), _scale(scale), _timearr(timearr) {}

        void operator()( const state_type &x , double t )
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = _frame_id;
            // if this _coefs is not 1.0, will cause jump between initial and next poses
            pose.pose.position.x = x[0] / _scale;
            pose.pose.position.y = x[1] / _scale;
            pose.pose.position.z = 0;

            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            _posearr.poses.push_back(pose.pose);

            _timearr.push_back(t);
        }
    };

}

#endif
