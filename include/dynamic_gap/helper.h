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

namespace dynamic_gap {
    typedef boost::array<double, 16> state_type;

    struct polar_gap_field{

        double x_right, x_left, y_right, y_left, gx, gy;
        double close_pt_x, close_pt_y, far_pt_x, far_pt_y, far_vec_x, far_vec_y, rbt_vec_x, rbt_vec_y, angle_gap;
        double dir_vec_x, dir_vec_y;
        double _sigma;
        bool _l, _r, _axial;
        double rbt_x_0, rbt_y_0;
        double thetag_revised;
        double theta_left, theta_right;
        double thetag;
        double w_des;
        double w_circ;

        polar_gap_field(double x_right, double x_left, double y_right, double y_left, double gx, double gy, bool l, bool r, bool axial, double sigma, double rbt_x_0, double rbt_y_0)
            : x_right(x_right), x_left(x_left), y_right(y_right), y_left(y_left), gx(gx), gy(gy), _l(l), _r(r), _axial(axial), _sigma(sigma), rbt_x_0(rbt_x_0), rbt_y_0(rbt_y_0) {
                theta_left = atan2(y_left, x_left);
                theta_right = atan2(y_right, x_right);
                thetag = atan2(gy, gx);
                if (theta_left > theta_right) {
                    thetag_revised = std::min(theta_left, std::max(thetag, theta_right));
                } else {
                    if (thetag > 0) {
                        thetag_revised = std::max(theta_right, thetag);
                    } else {
                        thetag_revised = std::min(theta_left, thetag);
                    }
                }

                w_des = 1.0;
                w_circ = 1.25;
            }

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            // AS OF RIGHT NOW, P2 IS THE LEFT POINT FROM ROBOT POV
            // double begin_time = ros::Time::now().toSec();

            //std::cout << "x: " << std::endl;
            Eigen::Vector2d rbt(x[0], x[1]);
            Eigen::Vector2d p_right(x_right, y_right);
            Eigen::Vector2d p_left(x_left, y_left);
            // vectors to left and right from rbt
            Eigen::Vector2d vec_right_rbt = p_right - rbt;
            Eigen::Vector2d vec_left_rbt = p_left - rbt;

            // rotation matrices for pi/2 and -pi/2
            double rot_angle = M_PI / 2;
            Eigen::Matrix2d r_pi2;
            r_pi2 << std::cos(rot_angle), -std::sin(rot_angle), std::sin(rot_angle), std::cos(rot_angle);
            Eigen::Matrix2d neg_r_pi2;
            neg_r_pi2 << std::cos(-rot_angle), -std::sin(-rot_angle), std::sin(-rot_angle), std::cos(-rot_angle);

            Eigen::Vector2d goal_pt(gx, gy);
            Eigen::Vector2d goal_vec = goal_pt - rbt;

            double r_right = sqrt(pow(x_right, 2) + pow(y_right, 2));
            double r_left = sqrt(pow(x_left, 2) + pow(y_left, 2));
            double rx = sqrt(pow(x[0], 2) + pow(x[1], 2));
            double rg = goal_vec.norm();

            double thetax = atan2(x[1], x[0]);

            // double theta_test = std::min(std::max(thetax, theta_right), theta_left);

            Eigen::Vector2d left_vec(x_left, y_left);
            Eigen::Vector2d right_vec(x_right, y_right);
            Eigen::Vector2d rbt_vec(x[0], x[1]);
            double left_delta_theta = std::acos(left_vec.dot(rbt_vec) / (left_vec.norm() * rbt_vec.norm()));
            double right_delta_theta = std::acos(right_vec.dot(rbt_vec) / (right_vec.norm() * rbt_vec.norm()));

            
            // ROTATIONAL TERM FOR LEFT (from robot pov)
            Eigen::Vector2d c_left = neg_r_pi2 * (vec_left_rbt / vec_left_rbt.norm()) * exp(-left_delta_theta / _sigma);

            // ROTATIONAL TERM FOR RIGHT (from robot pov)
            Eigen::Vector2d c_right = r_pi2 * (vec_right_rbt / vec_right_rbt.norm()) * exp(-right_delta_theta / _sigma);
            
            // ATTRACTIVE TERM FOR GOAL
            // Since local goal will definitely be within the range of the gap, this limit poses no difference
            Eigen::Vector2d sub_goal_vec = goal_vec;                //(rg * cos(thetag_revised), rg * sin(thetag_revised));

            // bool left = r2 > r1;

            // checking to see if robot has passed the gap
                        
            Eigen::Vector2d init_to_left_vect(x_left - rbt_x_0, y_left - rbt_y_0);
            Eigen::Vector2d init_to_right_vect(x_right - rbt_x_0, y_right - rbt_y_0);
            Eigen::Vector2d curr_to_left_vect(x_left - rbt[0], y_left - rbt[1]);  
            Eigen::Vector2d curr_to_right_vect(x_right - rbt[0], y_right - rbt[1]);
            Eigen::Vector2d init_to_goal_vect(gx - rbt_x_0, gy - rbt_y_0);
            Eigen::Vector2d curr_to_goal_vect(gx - rbt[0], gy - rbt[1]);

            bool past_gap_points;
            bool past_goal = (init_to_goal_vect.dot(curr_to_goal_vect) < 0);
            bool past_left_point = init_to_left_vect.dot(curr_to_left_vect) < 0;
            bool past_right_point = init_to_right_vect.dot(curr_to_right_vect) < 0;
            
            //std::cout << "checking past_gap_points: " << std::endl;
            //std::cout << "init to left: (" << init_to_left_vect[0] << ", " << init_to_left_vect[1] << "), curr_to_left_vect: (" << curr_to_left_vect[0] << ", " << curr_to_left_vect[1] << ")" << std::endl;
            //std::cout << "init to right: (" << init_to_right_vect[0] << ", " << init_to_right_vect[1] << "), curr_to_right_vect: (" << curr_to_right_vect[0] << ", " << curr_to_right_vect[1] << ")" << std::endl;
            if (_axial) {
                //pass_gap = (rbt.norm() > std::min(p_right.norm(), p_left.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
                past_gap_points = past_left_point || past_right_point;
            } else {
                // pass_gap = (rbt.norm() > std::max(p_right.norm(), p_left.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
                past_gap_points = past_left_point && past_right_point;
            }
            
            bool pass_gap = past_gap_points || past_goal;

            //std::cout << "pass_gap: " << pass_gap << std::endl;



            // Max: commenting out, not sure if used
            // Eigen::Vector2d polar_vec = rbt.norm() < 1e-3 || pass_gap ? Eigen::Vector2d(0, 0) : rbt / (rbt.norm());

            Eigen::Vector2d result(0, 0); 

            if (!pass_gap) {
                double coeffs = past_gap_points ? 0.0 : 1.0;
                Eigen::Vector2d v_des = sub_goal_vec / sub_goal_vec.norm();
                Eigen::Vector2d v_circ = (c_left + c_right) * coeffs; 
                // Eigen::Vector2d norm_v_circ = v_circ / v_circ.norm();
                
                result = v_des + v_circ;
                // result += v_des;
                std::cout << "t: " << t << ", rbt: (" << rbt[0] << ", " << rbt[1] << "), p_left: (" << p_left[0] << ", " << p_left[1] << "), p_right: (" << p_right[0] << ", " << p_right[1] << ")" << std::endl;
                // std::cout << "start pt: (" << rbt_x_0 << ", " << rbt_y_0 << "), goal_pt: (" << goal_pt[0] << ", " << goal_pt[1] << ")" << std::endl;
                std::cout << "left delta theta: " << left_delta_theta << ", right delta theta: " << right_delta_theta << std::endl; 
                std::cout << "c_left: (" << c_left[0] << ", " << c_left[1] << "), c_right: (" << c_right[0] << ", " << c_right[1] << ")" << std::endl;
                std::cout << "v_des: (" << v_des[0] << ", " << v_des[1] << "), v_circ: (" << v_circ[0] << ", " << v_circ[1] << ")" << std::endl;   
                std::cout << "---" << std::endl;
                //std::cout << "init to left vect: (" << init_to_left_vect[0] << ", " << init_to_left_vect[1] << "), curr to left vect: (" << curr_to_left_vect[0] << ", " << curr_to_left_vect[1] << ")" << std::endl;
                //std::cout << "init to right vect: (" << init_to_right_vect[0] << ", " << init_to_right_vect[1] << "), curr to right vect: (" << curr_to_right_vect[0] << ", " << curr_to_right_vect[1] << ")" << std::endl;
                //std::cout << "left dot: " << (init_to_left_vect.dot(curr_to_left_vect) < 0) << ", right dot: " << (init_to_right_vect.dot(curr_to_right_vect) < 0) << ", goal dot: " << (init_to_goal_vect.dot(curr_to_goal_vect) < 0)  << std::endl;

                // std::cout << "revised_pass_gap: " << pass_ << std::endl;
                //std::cout << "unit vec left_rbt: (" << vec_left_rbt[0]/vec_left_rbt.norm() << ", " << vec_left_rbt[1]/vec_left_rbt.norm() << "), left angular distance: " << left_delta_theta << std::endl;
                //std::cout << "unit vec right_rbt: (" << vec_right_rbt[0]/vec_right_rbt.norm() << ", " << vec_right_rbt[1]/vec_right_rbt.norm() << "), right angular distance: " << right_delta_theta << std::endl;
            
                //std::cout << "v_des: <" << v_des[0] << ", " << v_des[1] << ">, c_left: <" << c_left[0] << ", " << c_left[1] << ">, c_right: <" << c_right[0] << ", " << c_right[1] << ">, result: " << result[0] << ", " << result[1] << std::endl;
            }

            //double K_acc = 3.0;

            // calculate acceleration
            // these need to be in world frame?
            //double a_x_rbt = -K_acc*(x[2] - result(0));
            //double a_y_rbt = -K_acc*(x[3] - result(1));
            dxdt[0] = result[0];
            dxdt[1] = result[1];
            //dxdt[2] = a_x_rbt; // rbt_v_x
            //dxdt[3] = a_y_rbt; // rbt_v_y
            // double end_time = ros::Time::now().toSec();
            // std::cout << "length: " << ros::Time::now().toSec() - begin_time << std::endl;
            return;
        }
    };

    struct g2g {
        double gx, gy, K_des, K_acc;
        g2g(double gx, double gy, double K_des, double K_acc)
        : gx(gx), gy(gy), K_des(K_des), K_acc(K_acc) {}

        void operator() ( const state_type &x , state_type &dxdt , const double  t  )
        {
            // std::cout << "x: " << std::endl;
            // std::cout << x[0] << ", " << x[1] << std::endl;
            double goal_norm = sqrt(pow(gx - x[0], 2) + pow(gy - x[1], 2));
            Eigen::Vector2d v_des(0.0, 0.0);

            if (goal_norm < 0.1) {
                v_des(0) = 0;
                v_des(1) = 0;
            } else {
                v_des(0) = K_des*(gx - x[0]);
                v_des(1) = K_des*(gy - x[1]);
            }

            // set desired acceleration based on desired velocity
            // Eigen::Vector2d a_des(-K_acc*(x[2] - v_des(0)), -K_acc*(x[3] - v_des(1)));
            // std::cout << "v_des: " << v_des(0) << ", " << v_des(1)  << ". a_des: " << a_des(0) << ", " << a_des(1) << std::endl;
        
            dxdt[0] = v_des[0]; // rbt_x
            dxdt[1] = v_des[1]; // rbt_y
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
        clf_cbf(bool axial, double K_des, double cbf_param, double K_acc, double gx, double gy, double betadot_L_0, double betadot_R_0, double vx_absmax, double vy_absmax, double init_rbt_x, double init_rbt_y, double goal_vel_x, double goal_vel_y)
        : _axial(axial), K_des(K_des), cbf_param(cbf_param), K_acc(K_acc), gx(gx), gy(gy), betadot_L_0(betadot_L_0), betadot_R_0(betadot_R_0), vx_absmax(vx_absmax), vy_absmax(vy_absmax), init_rbt_x(init_rbt_x), init_rbt_y(init_rbt_y), goal_vel_x(goal_vel_x), goal_vel_y(goal_vel_y) {}
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
            //double h_left = r * betadot;

            double h_left;
            // revised design: h_left = betadot OR betadot - betadot_0

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
            // partials wrt r_ox, r_oy, v_ox, v_oy:
            //d_h_left_dx(0) = -left_rel_vel_rbt_frame(1)/r + left_rel_pos_rbt_frame(0)*r_v_cross_prod/pow(r, 3);
            //d_h_left_dx(1) =  left_rel_vel_rbt_frame(0)/r + left_rel_pos_rbt_frame(1)*r_v_cross_prod/pow(r, 3);
            //d_h_left_dx(2) = left_rel_pos_rbt_frame(1) / r;
            //d_h_left_dx(3) = -left_rel_pos_rbt_frame(0) / r;

            // revised design: h_left = betadot OR betadot - betadot_0
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
            //double h_right = - r * betadot;
            double h_right;
            // revised design: h_right = -betadot OR betadot_0 - betadot

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
            
            //d_h_right_dx(0) =  right_rel_vel_rbt_frame(1)/r - right_rel_pos_rbt_frame(0)*r_v_cross_prod/pow(r,3);
            //d_h_right_dx(1) = -right_rel_vel_rbt_frame(0)/r - right_rel_pos_rbt_frame(1)*r_v_cross_prod/pow(r,3);
            //d_h_right_dx(2) = -right_rel_pos_rbt_frame(1) / r;
            //d_h_right_dx(3) = right_rel_pos_rbt_frame(0) / r;

            // revised design: h_right = -betadot OR betadot_0 - betadot
            d_h_right_dx(0) =  right_rel_vel_rbt_frame(1)/pow(r,2) - 2*right_rel_pos_rbt_frame(0)*r_v_cross_prod/pow(r,4);
            d_h_right_dx(1) = -right_rel_vel_rbt_frame(0)/pow(r,2) - 2*right_rel_pos_rbt_frame(1)*r_v_cross_prod/pow(r,4);
            d_h_right_dx(2) = -right_rel_pos_rbt_frame(1) / pow(r,2);
            d_h_right_dx(3) =  right_rel_pos_rbt_frame(0) / pow(r,2);

            return d_h_right_dx;
        }

        double time_to_pass_CBF(const state_type &x, Eigen::Vector2d left_rel_pos_rbt_frame, Eigen::Vector2d left_rel_vel_rbt_frame,
                                                     Eigen::Vector2d right_rel_pos_rbt_frame, Eigen::Vector2d right_rel_vel_rbt_frame) {
            double r_left = sqrt(pow(left_rel_pos_rbt_frame(0), 2) + pow(left_rel_pos_rbt_frame(1), 2));
            double r_right = sqrt(pow(right_rel_pos_rbt_frame(0), 2) + pow(right_rel_pos_rbt_frame(1), 2));
            double left_r_v_cross_prod = left_rel_pos_rbt_frame(0)*left_rel_vel_rbt_frame(1) - left_rel_pos_rbt_frame(1)*left_rel_vel_rbt_frame(0);
            double right_r_v_cross_prod = right_rel_pos_rbt_frame(0)*right_rel_vel_rbt_frame(1) - right_rel_pos_rbt_frame(1)*right_rel_vel_rbt_frame(0);
            double cbf = gap_angle + T_h*(left_r_v_cross_prod/pow(r_left,2) - right_r_v_cross_prod/pow(r_right,2));
            return cbf;
        }

        Eigen::Vector4d time_to_pass_CBF_partials(const state_type &x, Eigen::Vector2d left_rel_pos_rbt_frame, Eigen::Vector2d left_rel_vel_rbt_frame,
                                                              Eigen::Vector2d right_rel_pos_rbt_frame, Eigen::Vector2d right_rel_vel_rbt_frame) {
            Eigen::Vector4d d_h_dx(0.0, 0.0, 0.0, 0.0);         
            double r_left = sqrt(pow(left_rel_pos_rbt_frame(0), 2) + pow(left_rel_pos_rbt_frame(1), 2));
            double r_right = sqrt(pow(right_rel_pos_rbt_frame(0), 2) + pow(right_rel_pos_rbt_frame(1), 2));
            double left_r_v_cross_prod = left_rel_pos_rbt_frame(0)*left_rel_vel_rbt_frame(1) - left_rel_pos_rbt_frame(1)*left_rel_vel_rbt_frame(0);
            double right_r_v_cross_prod = right_rel_pos_rbt_frame(0)*right_rel_vel_rbt_frame(1) - right_rel_pos_rbt_frame(1)*right_rel_vel_rbt_frame(0);
            d_h_dx(0) = T_h*(-left_rel_vel_rbt_frame(1)/pow(r_left, 2) + right_rel_vel_rbt_frame(1)/pow(r_right, 2) + (2*left_rel_pos_rbt_frame(0)*left_r_v_cross_prod)/pow(r_left,4) + (-2*right_rel_pos_rbt_frame(0)*right_r_v_cross_prod)/pow(r_right, 4));
            d_h_dx(1) = T_h*(left_rel_vel_rbt_frame(0)/pow(r_left, 2) - right_rel_vel_rbt_frame(0)/pow(r_right, 2) +  (2*left_rel_pos_rbt_frame(1)*left_r_v_cross_prod)/pow(r_left,4) + (-2*right_rel_pos_rbt_frame(1)*right_r_v_cross_prod)/pow(r_right, 4));
            d_h_dx(2) = T_h*(left_rel_pos_rbt_frame(1)/pow(r_left, 2) - right_rel_pos_rbt_frame(1)/pow(r_right, 2));
            d_h_dx(3) = T_h*(-left_rel_pos_rbt_frame(0)/pow(r_left, 2) + right_rel_pos_rbt_frame(0)/pow(r_right, 2));

            return d_h_dx;           
        }

        Eigen::Vector2d clip_velocities(double x_vel, double y_vel) {
            // std::cout << "in clip_velocities with " << x_vel << ", " << y_vel << std::endl;
            Eigen::Vector2d original_vel(x_vel, y_vel);
            if (std::abs(x_vel) <= vx_absmax && std::abs(y_vel) <= vy_absmax) {
                // std::cout << "not clipping" << std::endl;
                return original_vel;
            } else {
                // std::cout << "max: " << vx_absmax << ", norm: " << original_vel.norm() << std::endl;
                Eigen::Vector2d clipped_vel = vx_absmax * original_vel / std::max(std::abs(x_vel), std::abs(y_vel));
                return clipped_vel;
            }
        }

        /*
        double get_beta_center(Eigen::VectorXd y_left, Eigen::VectorXd y_right) {
            Eigen::Vector2d left_bearing_vect(y_left[2], y_left[1]);
            Eigen::Vector2d right_bearing_vect(y_right[2], y_right[1]);

            double det = y_left[2]*y_right[1] - y_left[1]*y_right[2];      
            double dot = left_bearing_vect.dot(right_bearing_vect);

            double swept_check = -std::atan2(det, dot);     
            double L_to_R_angle = swept_check;

            double curr_beta_center;
            if (L_to_R_angle > 0) { // convex
                curr_beta_center = beta_left - (L_to_R_angle / 2.0);
            } else {    // non-convex
                L_to_R_angle += 2*M_PI; 
            }

            double beta_left = std::atan2(y_left[1], y_left[2]);
            double beta_right = std::atan2(y_right[1], y_right[2]);
            if (curr_beta_center > 0 && curr_beta_center > M_PI) {
                curr_beta_center -= 2*M_PI;
            } else if (curr_beta_center < 0 && curr_beta_center < -M_PI) {
                curr_beta_center += 2*M_PI;
            }
            std::cout << "beta left: " << beta_left << ", beta right: " << beta_right << std::endl;
            std::cout << "L_to_R_angle: " << L_to_R_angle << ", beta center: " << curr_beta_center << std::endl;
            return curr_beta_center;
        }
        */

        state_type adjust_state(const state_type &x) {
            // clipping velocities
            // taking norm of sin/cos norm vector
            state_type new_x = x;
            // std::cout << "original state: " << new_x[0] << ", " << new_x[1] << ", " << new_x[2] << ", " << new_x[3] << std::endl;
            Eigen::Vector2d rbt_vel = clip_velocities(new_x[2], new_x[3]);
            new_x[2] = rbt_vel[0];
            new_x[3] = rbt_vel[1];
            // std::cout << "new state with clipped vels: " << new_x[0] << ", " << new_x[1] << ", " << new_x[2] << ", " << new_x[3] << std::endl;

            new_x[5] /= std::sqrt(pow(new_x[5], 2) + pow(new_x[6], 2));
            new_x[6] /= std::sqrt(pow(new_x[5], 2) + pow(new_x[6], 2));

            new_x[10] /= std::sqrt(pow(new_x[10], 2) + pow(new_x[11], 2));
            new_x[11] /= std::sqrt(pow(new_x[10], 2) + pow(new_x[11], 2));
            // std::cout << "new state with normalized left and right vectors: " << new_x[5] << ", " << new_x[6] << ", " << new_x[10] << ", " << new_x[11] << std::endl;

            return new_x;
        }

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            Eigen::Vector2d goal(x[14], x[15]);
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
                dxdt[11] = 0; dxdt[12] = 0; dxdt[13] = 0; dxdt[14] = 0; dxdt[15] = 0;
                return;
            }

            state_type new_x = adjust_state(x);

            // NEED TO TURN OFF L/R IF CROSSES BUT DOES NOT CLOSE

            Eigen::Vector2d rel_goal(goal[0] - new_x[0], goal[1] - new_x[1]);

            double V = pow(rel_goal[0], 2) + pow(rel_goal[1], 2);
            Eigen::Vector2d rbt(new_x[0], new_x[1]);
            Eigen::Vector2d rbt_vel(new_x[2], new_x[3]);

            std::cout << "inte_t: " << t << ", x: " << new_x[0] << ", " << new_x[1] << ", " << new_x[2] << ", " << new_x[3] << std::endl;
            std::cout << "V: " << V << ", local goal: " << goal[0] << ", " << goal[1] << ", " << std::endl;
            
            // set left/right models
            Eigen::VectorXd y_left(5);
            y_left << new_x[4], new_x[5], new_x[6], new_x[7], new_x[8];
            Eigen::VectorXd y_right(5);
            y_right << new_x[9], new_x[10], new_x[11], new_x[12], new_x[13];

            // obtain cartesian/rbt frame info from models
            Eigen::Vector4d x_left = mod_pol_to_cart(y_left);
            Eigen::Vector4d x_right = mod_pol_to_cart(y_right);

            std::cout << "y_left: " << y_left(0) << ", " << std::atan2(y_left(1),y_left(2)) << ", " << y_left(3) << ", " << y_left(4) << ", y_right: " << y_right(0) << ", " << std::atan2(y_right(1), y_right(2)) << ", " << y_right(3) << ", " << y_right(4) << std::endl;
            std::cout << "x_left: " << x_left(0) << ", " << x_left(1) << ", " << x_left(2) << ", " << x_left(3) << ", x_right: " << x_right(0) << ", " << x_right(1) << ", " << x_right(2) << ", " << x_right(3) << std::endl;

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
            v_des = clip_velocities(v_des[0], v_des[1]);

            // set desired acceleration based on desired velocity
            Eigen::Vector2d a_des(-K_acc*(rbt_vel[0] - v_des(0)), -K_acc*(rbt_vel[1] - v_des(1)));
            std::cout << "v_des: " << v_des(0) << ", " << v_des(1)  << ", a_des: " << a_des(0) << ", " << a_des(1) << std::endl;
            
            double det = y_left[2]*y_right[1] - y_left[1]*y_right[2];      
            double dot = y_left[2]*y_right[2] + y_left[1]*y_right[1];

            Eigen::Vector2d rbt_curr_to_init(x[0] - init_rbt_x, x[1] - init_rbt_y);
            Eigen::Vector2d curr_to_left_vect(x_left[0], x_left[1]);  
            Eigen::Vector2d curr_to_right_vect(x_right[0], x_right[1]);

            bool past_left_point = rbt_curr_to_init.dot(curr_to_left_vect) < 0;
            bool past_right_point = rbt_curr_to_init.dot(curr_to_right_vect) < 0;
        
            std::cout << "rbt_curr_to_init: " << rbt_curr_to_init[0] << ", " << rbt_curr_to_init[1] << std::endl;
            std::cout << "curr_to_left_vect: " << curr_to_left_vect[0] << ", " << curr_to_left_vect[1] << std::endl;
            std::cout << "curr_to_right_vect: " << curr_to_right_vect[0] << ", " << curr_to_right_vect[1] << std::endl;
            
            std::cout << "swept check: " << swept_check << ", past left: " << past_left_point << ", past right: " << past_right_point << std::endl;
            
            bool swept_check = past_left_point && past_right_point; //-std::atan2(det, dot);
            
            // TODO: IGNORE MODEL IF IT IS THE FURTHER ONE AND GAP IS CROSSING?
            Eigen::Vector2d a_actual = a_des;
            // check for convexity of gap
            if (V > 0.1 && !swept_check) {
                // std::cout << "adding CBF" << std::endl;
                double h_dyn = 0.0;
                Eigen::Vector4d d_h_dyn_dx(0.0, 0.0, 0.0, 0.0);

                Eigen::Vector2d left_rel_pos_rbt_frame(x_left(0), x_left(1));
                Eigen::Vector2d left_rel_vel_rbt_frame(x_left(2), x_left(3));
                Eigen::Vector2d right_rel_pos_rbt_frame(x_right(0), x_right(1));
                Eigen::Vector2d right_rel_vel_rbt_frame(x_right(2), x_right(3));

                double h_dyn_left = past_left_point ? 0.0 : cbf_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                double h_dyn_right = past_right_point ? 0.0 : cbf_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                Eigen::Vector4d d_h_dyn_left_dx = cbf_partials_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                Eigen::Vector4d d_h_dyn_right_dx = cbf_partials_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                std::cout << "left CBF value is: " << h_dyn_left << ", right CBF value is: " << h_dyn_right << std::endl;
                // " with partials: " << d_h_dyn_left_dx(0) << ", " << d_h_dyn_left_dx(1) << ", " << d_h_dyn_left_dx(2) << ", " << d_h_dyn_left_dx(3) << std::endl;
                // std::cout << << " with partials: " << d_h_dyn_right_dx(0) << ", " << d_h_dyn_right_dx(1) << ", " << d_h_dyn_right_dx(2) << ", " << d_h_dyn_right_dx(3) << std::endl;
                
                if (h_dyn_left < h_dyn_right) {
                    h_dyn = h_dyn_left;
                    d_h_dyn_dx = d_h_dyn_left_dx;
                } else if (h_dyn_right < h_dyn_left) {
                    h_dyn = h_dyn_right;
                    d_h_dyn_dx = d_h_dyn_right_dx;
                }

                // calculate Psi
                Eigen::Vector4d d_x_dt(rbt_vel[0], rbt_vel[1], a_des[0], a_des[1]);
                double Psi = d_h_dyn_dx.dot(d_x_dt) + cbf_param * h_dyn;
                std::cout << "h_dyn: " << h_dyn << ", d_h_dyn_dx: (" << d_h_dyn_dx[0] <<  ", " << d_h_dyn_dx[1] << ", " << d_h_dyn_dx[2] << ", " << d_h_dyn_dx[3] << "), Psi is " << Psi << std::endl;

                if (Psi < 0.0) {
                    Eigen::Vector2d Lg_h(d_h_dyn_dx[2], d_h_dyn_dx[3]); // Lie derivative of h wrt x
                    a_actual = a_des + -(Lg_h * Psi) / (Lg_h.dot(Lg_h));
                }
            } 

            std::cout << "a_actual: " << a_actual(0) << ", " << a_actual(1) << std::endl;
            
            double a_x_rbt = a_actual(0); // -K_acc*(x[2] - result(0)); // 
            double a_y_rbt = a_actual(1); // -K_acc*(x[3] - result(1)); // 

            double a_x_rel = 0 - a_x_rbt;
            double a_y_rel = 0 - a_y_rbt;

            dxdt[0] = rbt_vel[0]; // rbt_x
            dxdt[1] = rbt_vel[1]; // rbt_y
            dxdt[2] = a_x_rbt; // rbt_v_x
            dxdt[3] = a_y_rbt; // rbt_v_y

            dxdt[4] = - new_x[7] * new_x[4]; // 1 / r_left
            dxdt[5] = new_x[6] * new_x[8]; // sin(beta_left)
            dxdt[6] = -new_x[5] * new_x[8]; // cos(beta_left)
            dxdt[7] = new_x[8]*new_x[8] - new_x[7]*new_x[7] + new_x[4]* (a_x_rel * new_x[6] + a_y_rel * new_x[5]); // rdot_left / r_left
            dxdt[8] = - 2 * new_x[7] * new_x[8] + new_x[4] * (-a_x_rel*new_x[5] + a_y_rel*new_x[6]); // betadot_left

            dxdt[9] = - new_x[12] * new_x[9]; // 1 / r_right
            dxdt[10] = new_x[11] * new_x[13]; // sin(beta_right)
            dxdt[11] = - new_x[10] * new_x[13]; // cos(beta_right)
            dxdt[12] = new_x[13]*new_x[13] - new_x[12]*new_x[12] + new_x[9] * (a_x_rel*new_x[11] + a_y_rel * new_x[10]); // rdot_right / r_right
            dxdt[13] = - 2 * new_x[12] * new_x[13] + new_x[9] * (-a_x_rel * new_x[10] + a_y_rel * new_x[11]); // betadot_right
            dxdt[14] = goal_vel_x;
            dxdt[15] = goal_vel_y;
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
