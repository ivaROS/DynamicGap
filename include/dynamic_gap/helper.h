#ifndef ODE_H
#define ODE_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Twist.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_datatypes.h"

//#include "osqp.h"
//#include "/home/masselmeier3/osqp-cpp/include/osqp++.h"
#include "OsqpEigen/OsqpEigen.h"

namespace dynamic_gap {
    typedef boost::array<double, 14> state_type;
    //typedef state_type::index_range range;

    
    struct polar_gap_field{

        double x1, x2, y1, y2, gx, gy;
        double close_pt_x, close_pt_y, far_pt_x, far_pt_y, far_vec_x, far_vec_y, rbt_vec_x, rbt_vec_y, angle_gap;
        double dir_vec_x, dir_vec_y;
        double _sigma;
        bool mode_agc, pivoted_left, _axial;
        double rbt_x_0, rbt_y_0;    
        double K_acc;

        polar_gap_field(double x1, double x2, double y1, double y2, double gx, double gy, bool mode_agc, bool pivoted_left, bool axial, double sigma, double rbt_x_0, double rbt_y_0, double K_acc)
            : x1(x1), x2(x2), y1(y1), y2(y2), gx(gx), gy(gy), mode_agc(mode_agc), pivoted_left(pivoted_left), _axial(axial), _sigma(sigma), rbt_x_0(rbt_x_0), rbt_y_0(rbt_y_0), K_acc(K_acc) {}

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            // IN HERE X1,Y1 IS RIGHT FROM ROBOT POV, X2,Y2 IS LEFT FROM ROBOT POV
            if (atan2(y1, x1) > atan2(y2, x2)) {
                std::swap(y1, y2);
                std::swap(x1, x2);
            }
            
            Eigen::Vector2d rbt(x[0], x[1]);
            Eigen::Vector2d p1(x1, y1);
            Eigen::Vector2d p2(x2, y2);

            Eigen::Vector2d vec_1 = p1 - rbt;
            Eigen::Vector2d vec_2 = p2 - rbt;

            Eigen::Matrix2d r_pi2;
            double rot_angle = M_PI / 2;
            r_pi2 << std::cos(rot_angle), -std::sin(rot_angle), std::sin(rot_angle), std::cos(rot_angle);
            Eigen::Matrix2d neg_r_pi2;
            neg_r_pi2 << std::cos(-rot_angle), -std::sin(-rot_angle), std::sin(-rot_angle), std::cos(-rot_angle);

            Eigen::Vector2d goal_pt(gx, gy);
            Eigen::Vector2d goal_vec = goal_pt - rbt;

            double rg = goal_vec.norm();
            double theta1 = atan2(y1, x1);
            double theta2 = atan2(y2, x2);
            double thetax = atan2(x[1], x[0]);
            double thetag = atan2(goal_vec(1), goal_vec(0));

            double new_theta = std::min(std::max(thetag, theta1), theta2);

            double ang_diff_1 = std::abs(thetax - theta1);
            double ang_diff_2 = std::abs(theta2 - thetax);
            //double new_sigma = 0.05;

            Eigen::Vector2d c1 = r_pi2     * (vec_1 / vec_1.norm()) * exp(- ang_diff_1 / _sigma); // on robot's right
            Eigen::Vector2d c2 = neg_r_pi2 * (vec_2 / vec_2.norm()) * exp(- ang_diff_2 / _sigma); // on robot's left

            // Since local goal will definitely be within the range of the gap, this limit poses no difference
            Eigen::Vector2d sub_goal_vec(rg * cos(new_theta), rg * sin(new_theta));

            Eigen::Vector2d init_to_left_vect(x2 - rbt_x_0, y2 - rbt_y_0);
            Eigen::Vector2d init_to_right_vect(x1 - rbt_x_0, y1 - rbt_y_0);
            Eigen::Vector2d curr_to_left_vect(x2 - rbt[0], y2 - rbt[1]);  
            Eigen::Vector2d curr_to_right_vect(x1 - rbt[0], y1 - rbt[1]);
            Eigen::Vector2d init_to_goal_vect(gx - rbt_x_0, gy - rbt_y_0);
            Eigen::Vector2d curr_to_goal_vect(gx - rbt[0], gy - rbt[1]);

            bool past_gap_points;
            bool past_goal = (init_to_goal_vect.dot(curr_to_goal_vect) < 0);
            bool past_left_point = init_to_left_vect.dot(curr_to_left_vect) < 0;
            bool past_right_point = init_to_right_vect.dot(curr_to_right_vect) < 0;
            
            //bool pass_gap;
            if (_axial) {
                //pass_gap = (rbt.norm() > std::min(p_right.norm(), p_left.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
                past_gap_points = past_left_point || past_right_point;
            } else {
                // pass_gap = (rbt.norm() > std::max(p_right.norm(), p_left.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
                past_gap_points = past_left_point && past_right_point;
            }
            
            bool pass_gap = past_gap_points || past_goal;

            Eigen::Vector2d vel_des(0, 0); 

            Eigen::Vector2d final_goal_vec(0,0);

            if (!pass_gap)
            {
                double coeffs = past_gap_points ? 0.0 : 1.0;

                double vec_1_norm = vec_1.norm();
                double vec_2_norm = vec_2.norm();
                double w1 = vec_2_norm / sqrt(pow(vec_1_norm, 2) + pow(vec_2_norm,2));
                double w2 = vec_1_norm / sqrt(pow(vec_1_norm, 2) + pow(vec_2_norm,2));
                Eigen::Vector2d weighted_circulation_sum = w1*c1 + w2*c2;
                Eigen::Vector2d circulation_field = coeffs * weighted_circulation_sum / weighted_circulation_sum.norm();
                Eigen::Vector2d attraction_field = 0.5 * sub_goal_vec / sub_goal_vec.norm();
                //std::cout << "inte_t: " << t << std::endl;
                //std::cout << "robot to left: (" << vec_2[0] << ", " << vec_2[1] << "), robot to right: (" << vec_1[0] << ", " << vec_1[1] << ")" << std::endl;
                //std::cout << "angular difference to left: " << ang_diff_2 << ", angular difference to right: " << ang_diff_1 << std::endl;
                //std::cout << "left weight: " << w2 << ", left circulation component: (" << c2[0] << ", " << c2[1] << "), right weight: " << w1 << ", right circulation component: (" << c1[0] << ", " << c1[1] << ")" << std::endl;  
                //std::cout << "circulation: (" << circulation_field[0] << ", " << circulation_field[1] << ")" << std::endl;
                //std::cout << "robot to goal: (" << goal_vec[0] << ", " << goal_vec[1] << ")" << std::endl;
                //std::cout << "attraction: (" << attraction_field[0] << ", " << attraction_field[1] << ")" << std::endl;
                vel_des = circulation_field + attraction_field;
            }

            Eigen::Vector2d acc(K_acc*(vel_des(0) - x[2]), K_acc*(vel_des(1) - x[3]));

            dxdt[0] = x[2];
            dxdt[1] = x[3];
            dxdt[2] = acc[0];
            dxdt[3] = acc[1];
            return;
        }
    };


    struct APF_CBF{
        double x_right, x_left, y_right, y_left, _sigma, rbt_x_0, rbt_y_0, K_acc, rot_angle, 
               v_lin_max, a_lin_max, cbf_left_const, cbf_right_const, cbf_param, goal_vel_x, goal_vel_y,
               rg, theta_right, theta_left, thetax, thetag, new_theta, ang_diff_right, ang_diff_left, 
               coeffs, rel_right_pos_norm, rel_left_pos_norm, w_left, w_right, a_x_rbt, a_y_rbt, a_x_rel, a_y_rel, v_nom,
               r_reach, theta, r_inscr; 
        bool mode_agc, pivoted_left, _axial, past_gap_points, past_goal, past_left_point, past_right_point, pass_gap;
        Eigen::Matrix2d r_pi2, neg_r_pi2;
        Eigen::Vector2d rbt, rel_right_pos, rel_left_pos, abs_left_pos, abs_right_pos, 
                        abs_goal_pos, rel_goal_pos, c_left, c_right, sub_goal_vec, v_des, 
                        weighted_circulation_sum, circulation_field, attraction_field, a_des, a_actual;
        Eigen::Vector4d cart_left_state, cart_right_state;

        APF_CBF(double x_right, double x_left, double y_right, double y_left, 
                bool axial, double sigma, 
                double rbt_x_0, double rbt_y_0, double K_acc, double cbf_left_const, double cbf_right_const, 
                double cbf_param, double goal_vel_x, double goal_vel_y, double v_lin_max, double a_lin_max)
            : x_right(x_right), x_left(x_left), y_right(y_right), y_left(y_left), 
              mode_agc(mode_agc), pivoted_left(pivoted_left), _axial(axial), _sigma(sigma), 
              rbt_x_0(rbt_x_0), rbt_y_0(rbt_y_0), K_acc(K_acc), rot_angle(M_PI/2), cbf_left_const(cbf_left_const), cbf_right_const(cbf_right_const), cbf_param(cbf_param),
              goal_vel_x(goal_vel_x), goal_vel_y(goal_vel_y), v_lin_max(v_lin_max), a_lin_max(a_lin_max), v_nom(v_lin_max), r_inscr(0.2)
              {
                r_pi2 << std::cos(rot_angle), -std::sin(rot_angle), std::sin(rot_angle), std::cos(rot_angle);
                neg_r_pi2 << std::cos(-rot_angle), -std::sin(-rot_angle), std::sin(-rot_angle), std::cos(-rot_angle);
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

        state_type adjust_state(const state_type &x) {
            // clipping velocities
            // taking norm of sin/cos norm vector
            state_type new_x = x;
            Eigen::Vector2d rbt_vel = clip_velocities(new_x[2], new_x[3], v_lin_max);
            new_x[2] = rbt_vel[0];
            new_x[3] = rbt_vel[1];

            return new_x;
        }


        double cbf_left(const state_type &x, Eigen::Vector2d left_rel_pos_rbt_frame, Eigen::Vector2d left_rel_vel_rbt_frame) {
            // current design: h_left = r*betadot
            double r = sqrt(pow(left_rel_pos_rbt_frame(0), 2) + pow(left_rel_pos_rbt_frame(1), 2));
            double betadot = (left_rel_pos_rbt_frame(0)*left_rel_vel_rbt_frame(1) - left_rel_pos_rbt_frame(1)*left_rel_vel_rbt_frame(0))/pow(r, 2);

            double h_left = betadot - cbf_left_const;
            /*
            if (betadot >= 0) {
                h_left = betadot;
            } else {
                h_left = betadot - betadot_L_0;
            }
            */

            return h_left;
        }

        Eigen::Vector4d cbf_partials_left(const state_type &x, Eigen::Vector2d left_rel_pos_rbt_frame, Eigen::Vector2d left_rel_vel_rbt_frame) {
            // current design: h_left = r*betadot
            Eigen::Vector4d d_h_left_dx;
            double r = sqrt(pow(left_rel_pos_rbt_frame(0), 2) + pow(left_rel_pos_rbt_frame(1), 2));
            double r_v_cross_prod = left_rel_pos_rbt_frame(0)*left_rel_vel_rbt_frame(1) - left_rel_pos_rbt_frame(1)*left_rel_vel_rbt_frame(0);

            // derivative with respect to r_ox, r_oy, v_ox, v_oy
            d_h_left_dx(0) = -left_rel_vel_rbt_frame(1)/pow(r,2) + 2*left_rel_pos_rbt_frame(0)*r_v_cross_prod/pow(r, 4);
            d_h_left_dx(1) =  left_rel_vel_rbt_frame(0)/pow(r,2) + 2*left_rel_pos_rbt_frame(1)*r_v_cross_prod/pow(r, 4);
            d_h_left_dx(2) =  left_rel_pos_rbt_frame(1) / pow(r,2);
            d_h_left_dx(3) = -left_rel_pos_rbt_frame(0) / pow(r,2);
            return d_h_left_dx;
        }

        double cbf_right(const state_type &x, Eigen::Vector2d right_rel_pos_rbt_frame, Eigen::Vector2d right_rel_vel_rbt_frame) {
            // current design: h_right = -r*betadot
            double r = sqrt(pow(right_rel_pos_rbt_frame(0), 2) + pow(right_rel_pos_rbt_frame(1), 2));
            double betadot = (right_rel_pos_rbt_frame(0)*right_rel_vel_rbt_frame(1) - right_rel_pos_rbt_frame(1)*right_rel_vel_rbt_frame(0))/pow(r,2);
            
            double h_right = cbf_right_const - betadot;
            /*
            if (betadot <= 0) {
                h_right = -betadot;
            } else {
                h_right = betadot_R_0 - betadot;
            }
            */
            
            return h_right;
        }

        Eigen::Vector4d cbf_partials_right(const state_type &x, Eigen::Vector2d right_rel_pos_rbt_frame, Eigen::Vector2d right_rel_vel_rbt_frame) {
            // current design: h_right = -r * betadot
            Eigen::Vector4d d_h_right_dx;
            
            double r = sqrt(pow(right_rel_pos_rbt_frame(0), 2) + pow(right_rel_pos_rbt_frame(1), 2));
            double r_v_cross_prod = right_rel_pos_rbt_frame(0)*right_rel_vel_rbt_frame(1) - right_rel_pos_rbt_frame(1)*right_rel_vel_rbt_frame(0);
            // derivative with respect to r_ox, r_oy, v_ox, v_oy
            
            d_h_right_dx(0) =  right_rel_vel_rbt_frame(1)/pow(r,2) - 2*right_rel_pos_rbt_frame(0)*r_v_cross_prod/pow(r,4);
            d_h_right_dx(1) = -right_rel_vel_rbt_frame(0)/pow(r,2) - 2*right_rel_pos_rbt_frame(1)*r_v_cross_prod/pow(r,4);
            d_h_right_dx(2) = -right_rel_pos_rbt_frame(1) / pow(r,2);
            d_h_right_dx(3) =  right_rel_pos_rbt_frame(0) / pow(r,2);

            return d_h_right_dx;
        }

        void enforce_reachability(Eigen::Vector4d &state, const double t) {
            r_reach = std::min(r_inscr + t * v_nom, sqrt(pow(state[0], 2) + pow(state[1], 2)));
            theta = std::atan2(state[1], state[0]);
            state[0] = r_reach*std::cos(theta);
            state[1] = r_reach*std::sin(theta);
        }

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            // IN HERE X1,Y1 IS RIGHT FROM ROBOT POV, X2,Y2 IS LEFT FROM ROBOT POV
            // clip state, extract left/right points
            state_type new_x = adjust_state(x);
            cart_left_state << new_x[4], new_x[5], new_x[6], new_x[7];
            cart_right_state << new_x[8], new_x[9], new_x[10], new_x[11];
            enforce_reachability(cart_left_state, t);
            enforce_reachability(cart_right_state, t);


            rbt << new_x[0], new_x[1];
            rel_right_pos << cart_right_state[0], cart_right_state[1];
            rel_left_pos << cart_left_state[0], cart_left_state[1];
            abs_goal_pos << new_x[12], new_x[13];
            abs_left_pos = rel_left_pos + rbt;
            abs_right_pos = rel_right_pos + rbt;
            rel_goal_pos = abs_goal_pos - rbt;

            past_goal = abs_goal_pos.dot(rel_goal_pos) < 0;
            past_left_point = abs_left_pos.dot(rel_left_pos) < 0;
            past_right_point = abs_right_pos.dot(rel_right_pos) < 0;
            
            if (_axial) {
                past_gap_points = past_left_point || past_right_point;
            } else {
                past_gap_points = past_left_point && past_right_point;
            }
            
            pass_gap = past_gap_points || past_goal;

            if (pass_gap) {
                dxdt[0] = 0; dxdt[1] = 0; dxdt[2] = 0; dxdt[3] = 0; dxdt[4] = 0; dxdt[5] = 0; dxdt[6] = 0; 
                dxdt[7] = 0; dxdt[8] = 0; dxdt[9] = 0; dxdt[10] = 0; dxdt[11] = 0; dxdt[12] = 0; dxdt[13] = 0;
                return;
            }
            
            // APF
            rg = rel_goal_pos.norm();
            theta_right = atan2(abs_right_pos[1], abs_right_pos[0]);
            theta_left = atan2(abs_left_pos[1], abs_left_pos[0]);
            thetax = atan2(new_x[1], new_x[0]);
            thetag = atan2(rel_goal_pos[1], rel_goal_pos[0]);

            new_theta = std::min(std::max(thetag, theta_right), theta_left);

            ang_diff_right = std::abs(thetax - theta_right);
            ang_diff_left = std::abs(theta_left - thetax);

            c_left = neg_r_pi2 * (rel_left_pos / rel_left_pos.norm()) * exp(- ang_diff_left / _sigma);
            c_right = r_pi2 * (rel_right_pos / rel_right_pos.norm()) * exp(- ang_diff_right / _sigma);

            // Since local goal will definitely be within the range of the gap, this limit poses no difference
            sub_goal_vec << rg * cos(new_theta), rg * sin(new_theta);

            coeffs = (!past_gap_points);

            rel_right_pos_norm = rel_right_pos.norm();
            rel_left_pos_norm = rel_left_pos.norm();
            w_left = rel_right_pos_norm / sqrt(pow(rel_right_pos_norm, 2) + pow(rel_left_pos_norm,2));
            w_right = rel_left_pos_norm / sqrt(pow(rel_right_pos_norm, 2) + pow(rel_left_pos_norm,2));
            weighted_circulation_sum = w_left*c_left + w_right*c_right; //  
            circulation_field = coeffs * weighted_circulation_sum / weighted_circulation_sum.norm(); // / 
            attraction_field = 0.5 * sub_goal_vec / sub_goal_vec.norm(); // 
            /*
            ROS_INFO_STREAM("inte_t: " << t);
            ROS_INFO_STREAM("robot position: " << new_x[0] << ", " << new_x[1] << ", robot velocity: " << new_x[2] << ", " << new_x[3]);
            ROS_INFO_STREAM("robot to left: (" << rel_left_pos[0] << ", " << rel_left_pos[1] << "), robot to right: (" << rel_right_pos[0] << ", " << rel_right_pos[1] << ")");
            ROS_INFO_STREAM("angular difference to left: " << ang_diff_left << ", angular difference to right: " << ang_diff_right);
            ROS_INFO_STREAM("left weight: " << w_left << ", left circulation component: (" << c_left[0] << ", " << c_left[1] << "), right weight: " << w_right << ", right circulation component: (" << c_right[0] << ", " << c_right[1] << ")");  
            ROS_INFO_STREAM("circulation: (" << circulation_field[0] << ", " << circulation_field[1] << ")");
            ROS_INFO_STREAM("robot to goal: (" << rel_goal_pos[0] << ", " << rel_goal_pos[1] << ")");
            ROS_INFO_STREAM("attraction: (" << attraction_field[0] << ", " << attraction_field[1] << ")");
            */
            v_des = (circulation_field + attraction_field);

            // CLIPPING DESIRED VELOCITIES
            v_des = clip_velocities(v_des[0], v_des[1], v_lin_max);

            // set desired acceleration based on desired velocity
            a_des << K_acc*(v_des[0] - new_x[2]), K_acc*(v_des[1] - new_x[3]);
            a_des = clip_velocities(a_des[0], a_des[1], a_lin_max);
            
            // ROS_INFO_STREAM("v_des: " << v_des(0) << ", " << v_des(1)  << ", a_des: " << a_des(0) << ", " << a_des(1));

            a_actual = a_des;
            
            // check for convexity of gap
            if (rg > 0.1 && !past_gap_points) {
                // std::cout << "adding CBF" << std::endl;
                double h_dyn = 0.0;
                Eigen::Vector4d d_h_dyn_dx(0.0, 0.0, 0.0, 0.0);

                Eigen::Vector2d left_rel_pos_rbt_frame(cart_left_state(0), cart_left_state(1));
                Eigen::Vector2d left_rel_vel_rbt_frame(cart_left_state(2), cart_left_state(3));
                Eigen::Vector2d right_rel_pos_rbt_frame(cart_right_state(0), cart_right_state(1));
                Eigen::Vector2d right_rel_vel_rbt_frame(cart_right_state(2), cart_right_state(3));

                double h_dyn_left = past_left_point ? std::numeric_limits<double>::infinity() : cbf_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                double h_dyn_right = past_right_point ? std::numeric_limits<double>::infinity() : cbf_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                
                // ROS_INFO_STREAM("left CBF value is: " << h_dyn_left << ", right CBF value is: " << h_dyn_right);
               
                if (h_dyn_left <= h_dyn_right) { // left less than or equal to right
                    h_dyn = h_dyn_left;
                    d_h_dyn_dx = cbf_partials_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                } else { // right less than left
                    h_dyn = h_dyn_right;
                    d_h_dyn_dx = cbf_partials_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                }

                // calculate Psi
                Eigen::Vector4d d_x_dt(new_x[2], new_x[3], a_des[0], a_des[1]);
                double Psi = d_h_dyn_dx.dot(d_x_dt) + cbf_param * h_dyn;
                
                // ROS_INFO_STREAM("h_dyn: " << h_dyn << ", Psi: " << Psi);

                Eigen::Vector2d Lg_h(d_h_dyn_dx[2], d_h_dyn_dx[3]); // Lie derivative of h wrt x
                a_actual = a_des + -(Lg_h * std::min(Psi, 0.0)) / (Lg_h.dot(Lg_h));
            } 
            
            a_actual = clip_velocities(a_actual[0], a_actual[1], a_lin_max);
            
            //ROS_INFO_STREAM("a_actual: " << a_actual(0) << ", " << a_actual(1));

            double a_x_rbt = a_actual(0); // -K_acc*(x[2] - result(0)); // 
            double a_y_rbt = a_actual(1); // -K_acc*(x[3] - result(1)); // 

            double a_x_rel = 0 - a_x_rbt;
            double a_y_rel = 0 - a_y_rbt;

            dxdt[0] = new_x[2]; // rbt_x
            dxdt[1] = new_x[3]; // rbt_y
            dxdt[2] = a_x_rbt; // rbt_v_x
            dxdt[3] = a_y_rbt; // rbt_v_y

            dxdt[4] = new_x[6]; // r_x left
            dxdt[5] = new_x[7]; // r_y left
            dxdt[6] = a_x_rel; // v_x left
            dxdt[7] = a_y_rel; // v_y left

            dxdt[8] = new_x[10]; // r_x right
            dxdt[9] = new_x[11]; // r_y right
            dxdt[10] = a_x_rel; // v_x right
            dxdt[11] = a_y_rel; // v_y right
            dxdt[12] = goal_vel_x;
            dxdt[13] = goal_vel_y;
            return;
        }
    };

    struct reachable_gap_APF {
        Eigen::Vector2d left_pt_0, left_pt_1, right_pt_0, right_pt_1, nonrel_left_vel, nonrel_right_vel, rel_left_vel, rel_right_vel, nom_vel, 
                        goal_pt_0, goal_pt_1;
        std::vector<std::vector <double>> left_curve, right_curve;

        int num_curve_points, N, Kplus1, half_num_scan;
        double _sigma, rot_angle, 
               v_lin_max, a_lin_max, K_acc,
               rg, theta_right, theta_left, thetax, thetag, new_theta, ang_diff_right, ang_diff_left, 
               coeffs, rel_right_pos_norm, rel_left_pos_norm, w_left, w_right, a_x_rbt, a_y_rbt, a_x_rel, a_y_rel, v_nom,
               r_reach, theta, r_inscr; 
        bool mode_agc, pivoted_left, _axial, past_gap_points, past_goal, past_left_point, past_right_point, pass_gap;
        Eigen::Matrix2d r_pi2, neg_r_pi2;
        Eigen::Vector2d gap_origin, rbt, rel_right_pos, rel_left_pos, abs_left_pos, abs_right_pos, 
                        abs_goal_pos, rel_goal_pos, c_left, c_right, sub_goal_vec, v_des, 
                        weighted_circulation_sum, circulation_field, attraction_field, a_des, a_actual;
        Eigen::Vector4d cart_left_state, cart_right_state;

        Eigen::MatrixXd weights, all_centers;


        reachable_gap_APF(Eigen::Vector2d gap_origin, Eigen::Vector2d left_pt_0, Eigen::Vector2d left_pt_1,
                          Eigen::Vector2d right_pt_0, Eigen::Vector2d right_pt_1,
                          Eigen::Vector2d nonrel_left_vel, Eigen::Vector2d nonrel_right_vel,
                          Eigen::Vector2d rel_left_vel, Eigen::Vector2d rel_right_vel,
                          Eigen::Vector2d nom_vel, Eigen::Vector2d goal_pt_1,
                          double _sigma, double K_acc,
                          double v_lin_max, double a_lin_max) 
                          : gap_origin(gap_origin), left_pt_0(left_pt_0), left_pt_1(left_pt_1), right_pt_0(right_pt_0), right_pt_1(right_pt_1), 
                            nonrel_left_vel(nonrel_left_vel), nonrel_right_vel(nonrel_right_vel), rel_left_vel(rel_left_vel), rel_right_vel(rel_right_vel), nom_vel(nom_vel), goal_pt_1(goal_pt_1), _sigma(_sigma),
                            K_acc(K_acc), rot_angle(M_PI/2.), num_curve_points(25), v_lin_max(v_lin_max), a_lin_max(a_lin_max)
                        {
                            double start_time = ros::WallTime::now().toSec();
                            half_num_scan = 256;
                            N = 2*num_curve_points;
                            Kplus1 = 2*num_curve_points + 1;

                            Eigen::Matrix<double, 51, 51> A; // (Kplus1, N+1);
                            setConstraintMatrix(A, N, Kplus1);
                            // ROS_INFO_STREAM("A: " << A);

                            Eigen::Matrix<double, 51, 1> b = Eigen::MatrixXd::Zero(Kplus1, 1);

                            Eigen::SparseMatrix<double> hessian;
                            hessian.resize(Kplus1, Kplus1);

                            Eigen::VectorXd gradient;
                            gradient = Eigen::VectorXd::Zero(Kplus1, 1);

                            Eigen::SparseMatrix<double> linearMatrix; //Kplus1, Kplus1);
                            linearMatrix.resize(Kplus1, Kplus1);

                            Eigen::VectorXd lowerBound; //(Kplus1, 1);
                            lowerBound = Eigen::MatrixXd::Zero(Kplus1, 1);

                            Eigen::VectorXd upperBound; //(Kplus1, 1);
                            upperBound = Eigen::MatrixXd::Zero(Kplus1, 1);

                            // Eigen::Matrix<double, 51, 1> w_0;
                            // w_0 = Eigen::MatrixXd::Constant(Kplus1, 1, 1.0);
                            
                            for (int i = 0; i < Kplus1; i++) {
                                lowerBound(i, 0) = -OsqpEigen::INFTY;
                                upperBound(i, 0) = -0.0000001; // this leads to non-zero weights. Closer to zero this number goes, closer to zero the weights go. This makes sense
                            }

                            for (int i = 0; i < Kplus1; i++) {
                                for (int j = 0; j < Kplus1; j++) {
                                    if (i == j) {
                                        hessian.insert(i, j) = 1.0;
                                    }

                                    // need to transpose A vector, just doing here
                                    linearMatrix.insert(i, j) = A.coeff(j, i);
                                }
                            }

                            //ROS_INFO_STREAM("Hessian: " << hessian);
                            //ROS_INFO_STREAM("Gradient: " << gradient);
                            //ROS_INFO_STREAM("linearMatrix: " << linearMatrix);
                            //ROS_INFO_STREAM("lowerBound: " << lowerBound);
                            //ROS_INFO_STREAM("upperBound: " << upperBound);

                            OsqpEigen::Solver solver;
                            solver.settings()->setVerbosity(false);
                            solver.settings()->setWarmStart(false);        
                            solver.data()->setNumberOfVariables(Kplus1);
                            solver.data()->setNumberOfConstraints(Kplus1);
                            if(!solver.data()->setHessianMatrix(hessian)) return; // H ?
                            if(!solver.data()->setGradient(gradient)) return; // f ?
                            if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
                            if(!solver.data()->setLowerBound(lowerBound)) return;
                            if(!solver.data()->setUpperBound(upperBound)) return;


                            if(!solver.initSolver()) return;
                            // if(!solver.setPrimalVariable(w_0)) return;
                            
                            // solve the QP problem
                            if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return;

                            // get the controller input
                            Eigen::MatrixXd raw_weights = solver.getSolution();

                            weights = raw_weights / raw_weights.norm();                                

                            // if(!solver.setPrimalVariable(w_0)) return;
                            
                            // solve the QP problem

                            /*
                            ROS_INFO_STREAM("current solution: "); 
                            
                            std::string weights_string;
                            for (int i = 0; i < Kplus1; i++) {
                               ROS_INFO_STREAM(weights.coeff(i, 0)); 
                            }
                            
                            ROS_INFO_STREAM("optimization time elapsed: " << ros::WallTime::now().toSec() - start_time);
                            */
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

        void setConstraintMatrix(Eigen::Matrix<double, 51, 51> &A, int N, int Kplus1) {
            //ROS_INFO_STREAM("left_pt_0: " << left_pt_0[0] << ", " << left_pt_0[1]);
            //ROS_INFO_STREAM("left_pt_1: " << left_pt_1[0] << ", " << left_pt_1[1]);
            //ROS_INFO_STREAM("left_vel: " << left_vel[0] << ", " << left_vel[1]);

            //ROS_INFO_STREAM("right_pt_0: " << right_pt_0[0] << ", " << right_pt_0[1]);
            //ROS_INFO_STREAM("right_pt_1: " << right_pt_1[0] << ", " << right_pt_1[1]);
            //ROS_INFO_STREAM("right_vel: " << right_vel[0] << ", " << right_vel[1]);

            //ROS_INFO_STREAM("nom_vel: " << nom_vel[0] << ", " << nom_vel[1]);

            Eigen::Matrix<double, 25, 2> left_curve; // (num_curve_points, 2);
            Eigen::Matrix<double, 25, 2> right_curve; // (num_curve_points, 2);
            Eigen::Matrix<double, 50, 2> all_curve_pts; // (2*num_curve_points, 2);

            Eigen::Matrix<double, 25, 2> left_curve_vel; //(num_curve_points, 2);
            Eigen::Matrix<double, 25, 2> right_curve_vel; // (num_curve_points, 2);

            Eigen::Matrix<double, 25, 2> left_curve_inward_norm; // (num_curve_points, 2);
            Eigen::Matrix<double, 25, 2> right_curve_inward_norm; // (num_curve_points, 2);
            Eigen::Matrix<double, 50, 2> all_inward_norms; //(2*num_curve_points, 2);

            Eigen::Matrix<double, 50, 2> left_right_centers; //(2*num_curve_points, 2);
            all_centers.resize(2*num_curve_points + 1, 2);
                                    
            r_pi2 << std::cos(rot_angle), -std::sin(rot_angle), std::sin(rot_angle), std::cos(rot_angle);
            neg_r_pi2 << std::cos(-rot_angle), -std::sin(-rot_angle), std::sin(-rot_angle), std::cos(-rot_angle);

            double left_weight = nonrel_left_vel.norm() / nom_vel.norm();
            double right_weight = nonrel_right_vel.norm() / nom_vel.norm();
            double eps = 0.0000001;
            double offset = 0.5;

            // model gives: left_pt - rbt.
            Eigen::Vector2d weighted_left_pt = left_weight * left_pt_0;
            Eigen::Vector2d weighted_right_pt = right_weight * right_pt_0;

            Eigen::Matrix<double, 1, 2> goal; // (1, 2);
            goal << goal_pt_1[0],
                    goal_pt_1[1];

            //ROS_INFO_STREAM("weighted_left_pt: " << weighted_left_pt[0] << ", " << weighted_left_pt[1]);
            //ROS_INFO_STREAM("weighted_right_pt: " << weighted_right_pt[0] << ", " << weighted_right_pt[1]);
 
            // THIS IS FINE
            // populating the quadratic weighted bezier
            double s;    
            for (double i = 0; i < num_curve_points; i++) {
                s = (i + 1.0) / num_curve_points;
                //ROS_INFO_STREAM("i: " << i << ", s: " << s);
                double pos_val0 = (1 - s) * (1 - s);
                double pos_val1 = 2*(1 - s)*s;
                double pos_val2 = s*s;

                double vel_val0 = (1 - 2*s);
                double vel_val1 = (2 - 4*s);
                double vel_val2 = 2*s;
                Eigen::Vector2d curr_left_pt = pos_val0 * gap_origin + pos_val1*weighted_left_pt + pos_val2*left_pt_1;
                Eigen::Vector2d curr_left_vel = vel_val0 * gap_origin + vel_val1 * weighted_left_pt + vel_val2 * left_pt_1;
                Eigen::Vector2d rotated_curr_left_vel = neg_r_pi2 * curr_left_vel;
                Eigen::Vector2d left_inward_norm = rotated_curr_left_vel / (rotated_curr_left_vel.norm() + eps);
                left_curve.row(i) = curr_left_pt;
                left_curve_vel.row(i) = curr_left_vel;
                left_curve_inward_norm.row(i) = left_inward_norm;

                //ROS_INFO_STREAM("left_pt: " << left_pt[0] << ", " << left_pt[1] << ", right_pt: " << right_pt[0] << ", " << right_pt[1]);
                //ROS_INFO_STREAM("left_fin_pt: "<< left_fin_pt[0] << ", " << left_fin_pt[1]);

                Eigen::Vector2d curr_right_pt = pos_val0 * gap_origin + pos_val1*weighted_right_pt + pos_val2*right_pt_1;
                Eigen::Vector2d curr_right_vel = vel_val0 * gap_origin + vel_val1 * weighted_right_pt + vel_val2*right_pt_1;
                Eigen::Vector2d rotated_curr_right_vel = r_pi2 * curr_right_vel;
                Eigen::Vector2d right_inward_norm = rotated_curr_right_vel / (rotated_curr_right_vel.norm() + eps);
                right_curve.row(i) = curr_right_pt;
                right_curve_vel.row(i) = curr_right_vel;
                right_curve_inward_norm.row(i) = right_inward_norm;
            }

            all_curve_pts << left_curve, right_curve;
            all_inward_norms << left_curve_inward_norm, right_curve_inward_norm;

            left_right_centers = all_curve_pts - all_inward_norms*offset;
            all_centers << goal, left_right_centers;
            // ROS_INFO_STREAM("all_curve_pts: " << all_curve_pts);
            // ROS_INFO_STREAM("all_inward_norms: " << all_inward_norms);
            // ROS_INFO_STREAM("all_centers: " << all_centers);

            Eigen::Matrix<double, 1, 2> inward_norm_vector; //(Kplus1, 2);
            Eigen::Matrix<double, 2, 51> gradient_of_pti_wrt_centers; // (Kplus1, 2);
            Eigen::Matrix<double, 51, 50> A_N; //(Kplus1, N);
            Eigen::Matrix<double, 51, 1> A_S = Eigen::MatrixXd::Zero(Kplus1, 1);
            Eigen::Matrix<double, 51, 1> w_0 = Eigen::MatrixXd::Zero(Kplus1, 1);
            Eigen::VectorXd neg_one_vect(1, 1);
            neg_one_vect << -1.0;

            for (int i = 0; i < N; i++) {
                Eigen::Vector2d boundary_pt_i = all_curve_pts.row(i);
                inward_norm_vector = all_inward_norms.row(i);

                for (int j = 0; j < Kplus1; j++) {                    
                    Eigen::Vector2d center_j = all_centers.row(j);
                    // ROS_INFO_STREAM("boundary_pt_i: " << boundary_pt_i);
                    // ROS_INFO_STREAM("center_j: " << center_j);
                    Eigen::Vector2d diff = boundary_pt_i - center_j;
                    // ROS_INFO_STREAM("diff: " << diff);
                    Eigen::Vector2d gradient = diff / pow(diff.norm() + eps, 2);
                    gradient_of_pti_wrt_centers.col(j) = gradient;
                }
                //ROS_INFO_STREAM("inward_norm_vector: " << inward_norm_vector);
                //ROS_INFO_STREAM("gradient_of_pti: " << gradient_of_pti_wrt_centers);

                Eigen::Matrix<double, 1, 51> A_pi;
                A_pi = inward_norm_vector * gradient_of_pti_wrt_centers;
                // ROS_INFO_STREAM("A_pi: " << A_pi);
                A_N.col(i) = A_pi;
                // dotting inward norm (Kplus1 rows, 2 columns) with gradient of pti (Kplus1 rows, 2 columns) to get (Kplus1 rows, 1 column)
            }

            A_S.row(0) = neg_one_vect;

            A << A_N, A_S;
        }

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {             
            state_type new_x = adjust_state(x);
            cart_left_state << new_x[4], new_x[5], new_x[6], new_x[7];  
            cart_right_state << new_x[8], new_x[9], new_x[10], new_x[11];
               
            // Just use the same state to be able to make these past checks
            rbt << new_x[0], new_x[1];
            rel_right_pos << cart_right_state[0], cart_right_state[1];
            rel_left_pos << cart_left_state[0], cart_left_state[1];
            abs_goal_pos = goal_pt_1;
            abs_left_pos = rel_left_pos + rbt;
            abs_right_pos = rel_right_pos + rbt;
            rel_goal_pos = goal_pt_1 - rbt;

            past_goal = abs_goal_pos.dot(rel_goal_pos) < 0;
            past_left_point = abs_left_pos.dot(rel_left_pos) < 0;
            past_right_point = abs_right_pos.dot(rel_right_pos) < 0;
            
            if (_axial) {
                past_gap_points = past_left_point || past_right_point;
            } else {
                past_gap_points = past_left_point && past_right_point;
            }
            
            pass_gap = past_gap_points || past_goal;

            if (pass_gap) {
                dxdt[0] = 0; dxdt[1] = 0; dxdt[2] = 0; dxdt[3] = 0; dxdt[4] = 0; dxdt[5] = 0; dxdt[6] = 0; 
                dxdt[7] = 0; dxdt[8] = 0; dxdt[9] = 0; dxdt[10] = 0; dxdt[11] = 0; dxdt[12] = 0; dxdt[13] = 0;
                return;
            } 

            // ROS_INFO_STREAM("t: " << t);
            // ROS_INFO_STREAM("rbt state: " << new_x[0] << ", " << new_x[1] << ", " << new_x[2] << ", " << new_x[3]);
            // ROS_INFO_STREAM("left_pos: " << abs_left_pos[0] << ", " << abs_left_pos[1]);            
            // ROS_INFO_STREAM("right_pos: " << abs_right_pos[0] << ", " << abs_right_pos[1]);
            // ROS_INFO_STREAM("goal_pos: " << abs_goal_pos[0] << ", " << abs_goal_pos[1]);            
            
            int left_idx = (int) half_num_scan*std::atan2(abs_left_pos[1], abs_left_pos[0]) / M_PI + half_num_scan;
            int right_idx = (int) half_num_scan*std::atan2(abs_right_pos[1], abs_right_pos[0]) / M_PI + half_num_scan;
            int rbt_idx = (int) half_num_scan*std::atan2(rbt[1], rbt[0]) / M_PI + half_num_scan;
            if (left_idx < rbt_idx || right_idx > rbt_idx) {
                ROS_INFO_STREAM("potential exit at " << t);
                ROS_INFO_STREAM("rbt_idx: " << rbt_idx << ", left_idx: " << left_idx << ", right_idx: " << right_idx);
            }
            
            // APF
            rg = rel_goal_pos.norm();
            theta_right = atan2(abs_right_pos[1], abs_right_pos[0]);
            theta_left = atan2(abs_left_pos[1], abs_left_pos[0]);
            thetax = atan2(new_x[1], new_x[0]);
            thetag = atan2(rel_goal_pos[1], rel_goal_pos[0]);

            new_theta = std::min(std::max(thetag, theta_right), theta_left);

            double attractive_term = - pow(rel_goal_pos.norm(), 2);
            // ROS_INFO_STREAM("attractive term: " << attractive_term);

            double eps = 0.0000001;
            Eigen::Matrix<double, 2, 51> gradient_of_pti_wrt_centers; // (Kplus1, 2);
            Eigen::Vector2d rbt_pos(new_x[0], new_x[1]);
            for (int i = 0; i < Kplus1; i++) {

                Eigen::Vector2d center_i = all_centers.row(i);
                Eigen::Vector2d diff = rbt_pos - center_i;

                Eigen::Vector2d gradient = diff / pow(diff.norm(), 2);
                gradient_of_pti_wrt_centers.col(i) = gradient;
                /*
                if (i == 0) {
                    ROS_INFO_STREAM("goal point: (" << center_i[0] << ", " << center_i[1] << "), gradient: (" << gradient[0] << ", " << gradient[1] << "), weight: " << weights.coeff(i,0) << ", term: (" << attractive_term * weights.coeff(i,0) * gradient[0] << ", " << attractive_term * weights.coeff(i,0) * gradient[1] << ")");
                } else if (i >= 1 && i <= 25) {
                    ROS_INFO_STREAM("left point: (" << center_i[0] << ", " << center_i[1] << "), gradient: (" << gradient[0] << ", " << gradient[1] <<  "), weight: " << weights.coeff(i,0) << ", term: (" << attractive_term * weights.coeff(i,0) * gradient[0] << ", " << attractive_term * weights.coeff(i,0) * gradient[1] << ")");
                } else {
                    ROS_INFO_STREAM("right point: (" << center_i[0] << ", " << center_i[1] << "), gradient: (" << gradient[0] << ", " << gradient[1] <<  "), weight: " << weights.coeff(i,0) << ", term: (" << attractive_term * weights.coeff(i,0) * gradient[0] << ", " << attractive_term * weights.coeff(i,0) * gradient[1] << ")");
                }
                */
            }

            Eigen::Vector2d weighted_goal_term = attractive_term * gradient_of_pti_wrt_centers.block(0, 0, 2, 1) * weights.coeff(0, 0);
            Eigen::Vector2d weighted_left_term = attractive_term * gradient_of_pti_wrt_centers.block(0, 1, 2, 25) * weights.block(1, 0, 25, 1);
            Eigen::Vector2d weighted_right_term = attractive_term * gradient_of_pti_wrt_centers.block(0, 26, 2, 25) * weights.block(26, 0, 25, 1);

            // ROS_INFO_STREAM("weighted_goal_term: " << weighted_goal_term[0] << ", " << weighted_goal_term[1]);
            // ROS_INFO_STREAM("weighted_left_term: " << weighted_left_term[0] << ", " << weighted_left_term[1]);
            // ROS_INFO_STREAM("weighted_right_term: " << weighted_right_term[0] << ", " << weighted_right_term[1]);

            Eigen::Vector2d v_des = weighted_goal_term + weighted_left_term + weighted_right_term;
            // ROS_INFO_STREAM("v_des: " << v_des[0] << ", " << v_des[1]);

            // Eigen::Vector2d v_cmd = v_des / v_des.norm();
            // CLIPPING DESIRED VELOCITIES
            Eigen::Vector2d v_cmd = clip_velocities(v_des[0], v_des[1], v_lin_max);
            // ROS_INFO_STREAM("v_cmd: " << v_cmd[0] << ", " << v_cmd[1]);
            // set desired acceleration based on desired velocity
            a_des << K_acc*(v_cmd[0] - new_x[2]), K_acc*(v_cmd[1] - new_x[3]);
            a_des = clip_velocities(a_des[0], a_des[1], a_lin_max);
            // ROS_INFO_STREAM("a_des: " << a_des[0] << ", " << a_des[1]);
            double a_x_rbt = a_des(0); // -K_acc*(x[2] - result(0)); // 
            double a_y_rbt = a_des(1); // -K_acc*(x[3] - result(1)); // 

            double a_x_rel = 0 - a_x_rbt;
            double a_y_rel = 0 - a_y_rbt;

            dxdt[0] = new_x[2]; // rbt_x
            dxdt[1] = new_x[3]; // rbt_y
            dxdt[2] = a_x_rbt; // rbt_v_x
            dxdt[3] = a_y_rbt; // rbt_v_y

            dxdt[4] = new_x[6]; // r_x left
            dxdt[5] = new_x[7]; // r_y left
            dxdt[6] = a_x_rel; // v_x left
            dxdt[7] = a_y_rel; // v_y left

            dxdt[8] = new_x[10]; // r_x right
            dxdt[9] = new_x[11]; // r_y right
            dxdt[10] = a_x_rel; // v_x right
            dxdt[11] = a_y_rel; // v_y right
            dxdt[12] = 0.0;
            dxdt[13] = 0.0;
        }
    };
    
    struct g2g {
        double goal_vel_x, goal_vel_y, v_lin_max;
        g2g(double goal_vel_x, double goal_vel_y, double v_lin_max)
        : goal_vel_x(goal_vel_x), goal_vel_y(goal_vel_y), v_lin_max(v_lin_max) {}

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
            // std::cout << x[0] << ", " << x[1] << std::endl;
            double goal_norm = sqrt(pow(x[12] - x[0], 2) + pow(x[13] - x[1], 2));
            Eigen::Vector2d v_des(0.0, 0.0);

            if (goal_norm < 0.1) {
                v_des(0) = 0;
                v_des(1) = 0;
            } else {
                v_des(0) = (x[12] - x[0]);
                v_des(1) = (x[13] - x[1]);
            }

            Eigen::Vector2d v_cmd = clip_velocities(v_des[0], v_des[1], v_lin_max);

            // set desired acceleration based on desired velocity
            // Eigen::Vector2d a_des(-K_acc*(x[2] - v_des(0)), -K_acc*(x[3] - v_des(1)));
            // std::cout << "v_des: " << v_des(0) << ", " << v_des(1)  << ". a_des: " << a_des(0) << ", " << a_des(1) << std::endl;
        
            dxdt[0] = v_cmd[0]; // rbt_x
            dxdt[1] = v_cmd[1]; // rbt_y
            dxdt[12] = goal_vel_x;
            dxdt[13] = goal_vel_y;
            return;
        }
    };

    
    struct clf_cbf {
        double K_des;
        double K_acc;
        double cbf_param;
        bool _axial;
        double local_goal_dist;
        double forwards_gap;
        Eigen::Vector2d p1;
        Eigen::Vector2d p2;
        double gap_angle;
        double T_h;
        double betadot_L_0, betadot_R_0;
        double vx_absmax, vy_absmax;
        double init_rbt_x, init_rbt_y;
        double gx, gy;
        double goal_vel_x, goal_vel_y;
        bool gap_crossed;
        double ax_absmax, ay_absmax;
        clf_cbf(bool axial, double K_des, double cbf_param, double K_acc, double gx, 
                double gy, double betadot_L_0, double betadot_R_0, double vx_absmax, 
                double vy_absmax, double init_rbt_x, double init_rbt_y, 
                double goal_vel_x, double goal_vel_y, bool gap_crossed)
        : _axial(axial), K_des(K_des), cbf_param(cbf_param), K_acc(K_acc), gx(gx), 
            gy(gy), betadot_L_0(betadot_L_0), betadot_R_0(betadot_R_0), 
            vx_absmax(vx_absmax), vy_absmax(vy_absmax), init_rbt_x(init_rbt_x), 
            init_rbt_y(init_rbt_y), goal_vel_x(goal_vel_x), goal_vel_y(goal_vel_y), 
            gap_crossed(gap_crossed), ax_absmax(1.5), ay_absmax(1.5) {}
        // state: 
        // x[0]: rbt_x
        // x[1]: rbt_y
        // x[2]: rbt_v_x
        // x[3]: rbt_v_y
        // x[4]: 1 / r_left
        // x[5]: sin(beta_left)
        // x[6]: cos(beta_left)
        // x[7]: rdot_left / r_left
        // x[8]: betadot_left
        // x[9]: 1 / r_right
        // x[10]: sin(beta_right)
        // x[11]: cos(beta_right)
        // x[12]: rdot_right / r_right
        // x[13]: betadot_right
        // x[14]: sin(beta_goal)
        // x[15]: cos(beta_goal)

        Eigen::Vector4d mod_pol_to_cart(Eigen::VectorXd y) 
        {
            // x state:
            // [r_x, r_y, v_x, v_y]
            Eigen::Vector4d x(0.0, 0.0, 0.0, 0.0);
            x(0) = (1 / y(0)) * y(2);
            x(1) = (1 / y(0)) * y(1);
            x(2) = (1 / y(0)) * (y(3) * y(2) - y(4)*y(1));
            x(3) = (1 / y(0)) * (y(4) * y(2) + y(3)*y(1));
            return x;
        }

        double cbf_left(const state_type &x, Eigen::Vector2d left_rel_pos_rbt_frame, Eigen::Vector2d left_rel_vel_rbt_frame) {
            // current design: h_left = r*betadot
            double r = sqrt(pow(left_rel_pos_rbt_frame(0), 2) + pow(left_rel_pos_rbt_frame(1), 2));
            double betadot = (left_rel_pos_rbt_frame(0)*left_rel_vel_rbt_frame(1) - left_rel_pos_rbt_frame(1)*left_rel_vel_rbt_frame(0))/pow(r, 2);

            double h_left;
            if (betadot >= 0) {
                h_left = betadot;
            } else {
                h_left = betadot - betadot_L_0;
            }

            return h_left;
        }

        Eigen::Vector4d cbf_partials_left(const state_type &x, Eigen::Vector2d left_rel_pos_rbt_frame, Eigen::Vector2d left_rel_vel_rbt_frame) {
            // current design: h_left = r*betadot
            Eigen::Vector4d d_h_left_dx(0.0, 0.0, 0.0, 0.0);
            double r = sqrt(pow(left_rel_pos_rbt_frame(0), 2) + pow(left_rel_pos_rbt_frame(1), 2));
            double r_v_cross_prod = left_rel_pos_rbt_frame(0)*left_rel_vel_rbt_frame(1) - left_rel_pos_rbt_frame(1)*left_rel_vel_rbt_frame(0);

            // derivative with respect to r_ox, r_oy, v_ox, v_oy
            d_h_left_dx(0) = -left_rel_vel_rbt_frame(1)/pow(r,2) + 2*left_rel_pos_rbt_frame(0)*r_v_cross_prod/pow(r, 4);
            d_h_left_dx(1) =  left_rel_vel_rbt_frame(0)/pow(r,2) + 2*left_rel_pos_rbt_frame(1)*r_v_cross_prod/pow(r, 4);
            d_h_left_dx(2) =  left_rel_pos_rbt_frame(1) / pow(r,2);
            d_h_left_dx(3) = -left_rel_pos_rbt_frame(0) / pow(r,2);
            return d_h_left_dx;
        }

        double cbf_right(const state_type &x, Eigen::Vector2d right_rel_pos_rbt_frame, Eigen::Vector2d right_rel_vel_rbt_frame) {
            // current design: h_right = -r*betadot
            double r = sqrt(pow(right_rel_pos_rbt_frame(0), 2) + pow(right_rel_pos_rbt_frame(1), 2));
            double betadot = (right_rel_pos_rbt_frame(0)*right_rel_vel_rbt_frame(1) - right_rel_pos_rbt_frame(1)*right_rel_vel_rbt_frame(0))/pow(r,2);
            
            double h_right;
            if (betadot <= 0) {
                h_right = -betadot;
            } else {
                h_right = betadot_R_0 - betadot;
            }
            
            return h_right;
        }

        Eigen::Vector4d cbf_partials_right(const state_type &x, Eigen::Vector2d right_rel_pos_rbt_frame, Eigen::Vector2d right_rel_vel_rbt_frame) {
            // current design: h_right = -r * betadot
            Eigen::Vector4d d_h_right_dx(0.0, 0.0, 0.0, 0.0);
            
            double r = sqrt(pow(right_rel_pos_rbt_frame(0), 2) + pow(right_rel_pos_rbt_frame(1), 2));
            double r_v_cross_prod = right_rel_pos_rbt_frame(0)*right_rel_vel_rbt_frame(1) - right_rel_pos_rbt_frame(1)*right_rel_vel_rbt_frame(0);
            // derivative with respect to r_ox, r_oy, v_ox, v_oy
            
            d_h_right_dx(0) =  right_rel_vel_rbt_frame(1)/pow(r,2) - 2*right_rel_pos_rbt_frame(0)*r_v_cross_prod/pow(r,4);
            d_h_right_dx(1) = -right_rel_vel_rbt_frame(0)/pow(r,2) - 2*right_rel_pos_rbt_frame(1)*r_v_cross_prod/pow(r,4);
            d_h_right_dx(2) = -right_rel_pos_rbt_frame(1) / pow(r,2);
            d_h_right_dx(3) =  right_rel_pos_rbt_frame(0) / pow(r,2);

            return d_h_right_dx;
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

        state_type adjust_state(const state_type &x) {
            // clipping velocities
            // taking norm of sin/cos norm vector
            state_type new_x = x;
            // std::cout << "original state: " << new_x[0] << ", " << new_x[1] << ", " << new_x[2] << ", " << new_x[3] << std::endl;
            Eigen::Vector2d rbt_vel = clip_velocities(new_x[2], new_x[3], vx_absmax);
            new_x[2] = rbt_vel[0];
            new_x[3] = rbt_vel[1];
            // std::cout << "new state with clipped vels: " << new_x[0] << ", " << new_x[1] << ", " << new_x[2] << ", " << new_x[3] << std::endl;

            //new_x[5] /= std::sqrt(pow(new_x[5], 2) + pow(new_x[6], 2));
            //new_x[6] /= std::sqrt(pow(new_x[5], 2) + pow(new_x[6], 2));

            //new_x[10] /= std::sqrt(pow(new_x[10], 2) + pow(new_x[11], 2));
            //new_x[11] /= std::sqrt(pow(new_x[10], 2) + pow(new_x[11], 2));
            // std::cout << "new state with normalized left and right vectors: " << new_x[5] << ", " << new_x[6] << ", " << new_x[10] << ", " << new_x[11] << std::endl;

            return new_x;
        }

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            Eigen::Vector2d goal(x[12], x[13]);
            Eigen::Vector2d init_to_goal_vect(goal[0] - init_rbt_x, goal[1] - init_rbt_y);
            Eigen::Vector2d curr_to_goal_vect(goal[0] - x[0], goal[1] - x[1]);

            bool past_goal = (init_to_goal_vect.dot(curr_to_goal_vect) < 0);

            //Eigen::Vector2d goal_pt(gx - init_rbt_x, gy - init_rbt_y);
            //Eigen::Vector2d rbt_traveled(x[0] - init_rbt_x, x[1] - init_rbt_y);
            //bool past_goal = rbt_traveled.norm() > 0.9*goal_pt.norm();
            // std::cout << "rbt traveled norm: " << rbt_traveled.norm() << ", goal_pt norm: " << goal_pt.norm() << std::endl;
            
            if (past_goal) {   
                // std::cout << "past gap: " << pass_gap << ", closed gap: " << closed_gap << std::endl;
                dxdt[0] = 0; dxdt[1] = 0; dxdt[2] = 0; dxdt[3] = 0; dxdt[4] = 0;
                dxdt[5] = 0; dxdt[6] = 0; dxdt[7] = 0; dxdt[8] = 0; dxdt[9] = 0; dxdt[10] = 0;
                dxdt[11] = 0; dxdt[12] = 0; dxdt[13] = 0;
                return;
            }

            state_type new_x = adjust_state(x);

            // NEED TO TURN OFF L/R IF CROSSES BUT DOES NOT CLOSE

            Eigen::Vector2d rel_goal(goal[0] - new_x[0], goal[1] - new_x[1]);

            double V = pow(rel_goal[0], 2) + pow(rel_goal[1], 2);
            Eigen::Vector2d rbt(new_x[0], new_x[1]);
            Eigen::Vector2d rbt_vel(new_x[2], new_x[3]);

            // std::cout << "inte_t: " << t << ", x: " << new_x[0] << ", " << new_x[1] << ", " << new_x[2] << ", " << new_x[3] << std::endl;
            // std::cout << "V: " << V << ", local goal: " << goal[0] << ", " << goal[1] << ", " << std::endl;
            /*
            // set left/right models
            Eigen::VectorXd y_left(5);
            y_left << new_x[4], new_x[5], new_x[6], new_x[7], new_x[8];
            Eigen::VectorXd y_right(5);
            y_right << new_x[9], new_x[10], new_x[11], new_x[12], new_x[13];
            */
            // obtain cartesian/rbt frame info from models
            Eigen::Vector4d x_left(4); // = mod_pol_to_cart(y_left);
            x_left << new_x[4], new_x[5], new_x[6], new_x[7];
            Eigen::Vector4d x_right(4); // = mod_pol_to_cart(y_right);
            x_right << new_x[8], new_x[9], new_x[10], new_x[11];
            // std::cout << "y_left: " << y_left(0) << ", " << std::atan2(y_left(1),y_left(2)) << ", " << y_left(3) << ", " << y_left(4) << ", y_right: " << y_right(0) << ", " << std::atan2(y_right(1), y_right(2)) << ", " << y_right(3) << ", " << y_right(4) << std::endl;
            // std::cout << "x_left: " << x_left(0) << ", " << x_left(1) << ", " << x_left(2) << ", " << x_left(3) << ", x_right: " << x_right(0) << ", " << x_right(1) << ", " << x_right(2) << ", " << x_right(3) << std::endl;

            /*
            if (rel_goal.dot(rbt_vel) < 0) {
                std::cout << "moving away from goal" << std::endl;
            }
            */


            Eigen::Vector2d v_des(0.0, 0.0);
            // If V sufficiently large, set desired velocity
            if (V > 0.1) {
                v_des(0) = -K_des*(new_x[0] - goal[0]);
                v_des(1) = -K_des*(new_x[1] - goal[1]);
            }

            // CLIPPING DESIRED VELOCITIES
            v_des = clip_velocities(v_des[0], v_des[1], vx_absmax);

            // set desired acceleration based on desired velocity
            Eigen::Vector2d a_des(-K_acc*(rbt_vel[0] - v_des(0)), -K_acc*(rbt_vel[1] - v_des(1)));
            // std::cout << "v_des: " << v_des(0) << ", " << v_des(1)  << ", a_des: " << a_des(0) << ", " << a_des(1) << std::endl;

            a_des = clip_velocities(a_des[0], a_des[1], ax_absmax);

            // 1: sin, 2: cos
            double left_norm = std::sqrt(pow(x_left[0], 2) + pow(x_left[1], 2));
            double right_norm = std::sqrt(pow(x_right[0], 2) + pow(x_right[1], 2));
            Eigen::Vector2d left_unit_norm(x_left[0] / left_norm, x_left[1] / left_norm);
            Eigen::Vector2d right_unit_norm(x_right[0] / right_norm, x_right[1] / right_norm);
            double det = left_unit_norm[0]*right_unit_norm[1] - left_unit_norm[1]*right_unit_norm[0];      
            double dot = left_unit_norm[0]*right_unit_norm[0] + left_unit_norm[1]*right_unit_norm[1];

            Eigen::Vector2d rbt_curr_to_init(x[0] - init_rbt_x, x[1] - init_rbt_y);
            Eigen::Vector2d curr_to_left_vect(x_left[0], x_left[1]);  
            Eigen::Vector2d curr_to_right_vect(x_right[0], x_right[1]);

            bool past_left_point = rbt_curr_to_init.dot(curr_to_left_vect) < 0;
            bool past_right_point = rbt_curr_to_init.dot(curr_to_right_vect) < 0;
        
            //std::cout << "rbt_curr_to_init: " << rbt_curr_to_init[0] << ", " << rbt_curr_to_init[1] << std::endl;
            //std::cout << "curr_to_left_vect: " << curr_to_left_vect[0] << ", " << curr_to_left_vect[1] << std::endl;
            //std::cout << "curr_to_right_vect: " << curr_to_right_vect[0] << ", " << curr_to_right_vect[1] << std::endl;
                        
            bool past_left_and_right = past_left_point && past_right_point; //-std::atan2(det, dot);
            // std::cout << "past left: " << past_left_point << ", past right: " << past_right_point << std::endl;
            Eigen::Vector2d a_actual = a_des;
            // check for convexity of gap
            if (V > 0.1 && !past_left_and_right) {
                // std::cout << "adding CBF" << std::endl;
                double h_dyn = 0.0;
                Eigen::Vector4d d_h_dyn_dx(0.0, 0.0, 0.0, 0.0);

                Eigen::Vector2d left_rel_pos_rbt_frame(x_left(0), x_left(1));
                Eigen::Vector2d left_rel_vel_rbt_frame(x_left(2), x_left(3));
                Eigen::Vector2d right_rel_pos_rbt_frame(x_right(0), x_right(1));
                Eigen::Vector2d right_rel_vel_rbt_frame(x_right(2), x_right(3));

                double h_dyn_left = past_left_point ? std::numeric_limits<double>::infinity() : cbf_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                double h_dyn_right = past_right_point ? std::numeric_limits<double>::infinity() : cbf_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                
                if (gap_crossed) {
                    // if r_left < r_right (left is closer)
                    if (left_norm < right_norm) {
                        h_dyn_right = std::numeric_limits<double>::infinity();
                    } else {
                        h_dyn_left = std::numeric_limits<double>::infinity();
                    }
                }
                
                //Eigen::Vector4d d_h_dyn_left_dx = cbf_partials_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                //Eigen::Vector4d d_h_dyn_right_dx = cbf_partials_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                // std::cout << "left CBF value is: " << h_dyn_left << ", right CBF value is: " << h_dyn_right << std::endl;
                // " with partials: " << d_h_dyn_left_dx(0) << ", " << d_h_dyn_left_dx(1) << ", " << d_h_dyn_left_dx(2) << ", " << d_h_dyn_left_dx(3) << std::endl;
                // std::cout << << " with partials: " << d_h_dyn_right_dx(0) << ", " << d_h_dyn_right_dx(1) << ", " << d_h_dyn_right_dx(2) << ", " << d_h_dyn_right_dx(3) << std::endl;
                
                if (h_dyn_left <= h_dyn_right) { // left less than or equal to right
                    h_dyn = h_dyn_left;
                    d_h_dyn_dx = cbf_partials_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                } else { // right less than left
                    h_dyn = h_dyn_right;
                    d_h_dyn_dx = cbf_partials_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                }

                // calculate Psi
                Eigen::Vector4d d_x_dt(rbt_vel[0], rbt_vel[1], a_des[0], a_des[1]);
                double Psi = d_h_dyn_dx.dot(d_x_dt) + cbf_param * h_dyn;
                // std::cout << "h_dyn: " << h_dyn << ", Psi: " << Psi << std::endl;

                if (Psi < 0.0) {
                    Eigen::Vector2d Lg_h(d_h_dyn_dx[2], d_h_dyn_dx[3]); // Lie derivative of h wrt x
                    a_actual = a_des + -(Lg_h * Psi) / (Lg_h.dot(Lg_h));
                }
            } 

            // std::cout << "a_actual: " << a_actual(0) << ", " << a_actual(1) << std::endl;
            a_actual = clip_velocities(a_actual[0], a_actual[1], ax_absmax);

            double a_x_rbt = a_actual(0); // -K_acc*(x[2] - result(0)); // 
            double a_y_rbt = a_actual(1); // -K_acc*(x[3] - result(1)); // 

            double a_x_rel = 0 - a_x_rbt;
            double a_y_rel = 0 - a_y_rbt;

            dxdt[0] = rbt_vel[0]; // rbt_x
            dxdt[1] = rbt_vel[1]; // rbt_y
            dxdt[2] = a_x_rbt; // rbt_v_x
            dxdt[3] = a_y_rbt; // rbt_v_y

            dxdt[4] = new_x[6]; // r_x left
            dxdt[5] = new_x[7]; // r_y left
            dxdt[6] = a_x_rel; // v_x left
            dxdt[7] = a_y_rel; // v_y left

            dxdt[8] = new_x[10]; // r_x right
            dxdt[9] = new_x[11]; // r_y right
            dxdt[10] = a_x_rel; // v_x right
            dxdt[11] = a_y_rel; // v_y right
            dxdt[12] = goal_vel_x;
            dxdt[13] = goal_vel_y;
            return;
        }
    };
    

    // this is an observer?
    struct write_trajectory
    {
        geometry_msgs::PoseArray& _posearr;
        std::string _frame_id;
        double _coefs;
        std::vector<double>& _timearr;

        write_trajectory(geometry_msgs::PoseArray& posearr, std::string frame_id, double coefs, std::vector<double>& timearr)
        : _posearr(posearr), _frame_id(frame_id), _coefs(coefs), _timearr(timearr) {}

        void operator()( const state_type &x , double t )
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = _frame_id;
            // if this _coefs is not 1.0, will cause jump between initial and next poses
            pose.pose.position.x = x[0] / _coefs;
            pose.pose.position.y = x[1] / _coefs;
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
