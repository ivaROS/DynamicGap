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
    typedef boost::array<double, 14> state_type;

    struct polar_gap_field{

        double x1, x2, y1, y2, gx, gy;
        double close_pt_x, close_pt_y, far_pt_x, far_pt_y, far_vec_x, far_vec_y, rbt_vec_x, rbt_vec_y, angle_gap;
        double dir_vec_x, dir_vec_y;
        double _sigma;
        bool _l, _r, _axial;

        polar_gap_field(double x1, double x2, double y1, double y2, double gx, double gy, bool l, bool r, bool axial, double sigma)
            : x1(x1), x2(x2), y1(y1), y2(y2), gx(gx), gy(gy), _l(l), _r(r), _axial(axial), _sigma(sigma) {}

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            // making sure that point 1 is left point and point 2 is right point
            double begin_time = ros::Time::now().toSec();

            if (atan2(y1, x1) > atan2(y2, x2)) {
                std::swap(y1, y2);
                std::swap(x1, x2);
            }
            //std::cout << "x: " << std::endl;
            //std::cout << x[0] << ", " << x[1] << std::endl;
            Eigen::Vector2d rbt(x[0], x[1]);
            Eigen::Vector2d p1(x1, y1);
            Eigen::Vector2d p2(x2, y2);
            // vectors to left and right from rbt
            Eigen::Vector2d vec_1 = p1 - rbt;
            Eigen::Vector2d vec_2 = p2 - rbt;

            // rotation matrices for pi/2 and -pi/2
            Eigen::Matrix2d r_pi2;
            double rot_angle = M_PI / 2;
            r_pi2 << std::cos(rot_angle), -std::sin(rot_angle), std::sin(rot_angle), std::cos(rot_angle);
            Eigen::Matrix2d neg_r_pi2;
            neg_r_pi2 << std::cos(-rot_angle), -std::sin(-rot_angle), std::sin(-rot_angle), std::cos(-rot_angle);

            Eigen::Vector2d goal_pt(gx, gy);
            Eigen::Vector2d goal_vec = goal_pt - rbt;

            double r1 = sqrt(pow(x1, 2) + pow(y1, 2));
            double r2 = sqrt(pow(x2, 2) + pow(y2, 2));
            double rx = sqrt(pow(x[0], 2) + pow(x[1], 2));
            double rg = goal_vec.norm();
            double theta1 = atan2(y1, x1);
            double theta2 = atan2(y2, x2);
            double thetax = atan2(x[1], x[0]);
            double thetag = atan2(goal_vec(1), goal_vec(0));

            double new_theta = std::min(std::max(thetag, theta1), theta2);
            double theta_test = std::min(std::max(thetax, theta1), theta2);

            // ROTATIONAL TERM FOR LEFT
            Eigen::Vector2d c1 = r_pi2     * (vec_1 / vec_1.norm()) * exp(-std::abs(thetax - theta1) / _sigma);
            
            // ROTATIONAL TERM FOR RIGHT
            Eigen::Vector2d c2 = neg_r_pi2 * (vec_2 / vec_2.norm()) * exp(-std::abs(theta2 - thetax) / _sigma);

            // ATTRACTIVE TERM FOR GOAL
            // Since local goal will definitely be within the range of the gap, this limit poses no difference
            Eigen::Vector2d sub_goal_vec(rg * cos(new_theta), rg * sin(new_theta));

            bool left = r2 > r1;

            // checking to see if robot has passed the gap
            bool pass_gap;
            if (_axial) {
                pass_gap = (rbt.norm() > std::min(p1.norm(), p2.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
            } else {
                pass_gap = (rbt.norm() > std::max(p1.norm(), p2.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
            }


            Eigen::Vector2d v1 = p1 - p2;
            Eigen::Vector2d v2 = p1 - rbt;

            // Max: commenting out, not sure if used
            // Eigen::Vector2d polar_vec = rbt.norm() < 1e-3 || pass_gap ? Eigen::Vector2d(0, 0) : rbt / (rbt.norm());

            Eigen::Vector2d result(0, 0); 
            double coeffs = pass_gap ? 0.0 : 1.0;

            Eigen::Vector2d final_goal_vec(0,0);

            if (pass_gap)
            {
                result = Eigen::Vector2d(0, 0);
            } else {
                result = (c1 + c2) * coeffs;
                result += sub_goal_vec / sub_goal_vec.norm();
            }

            dxdt[0] = result(0);
            dxdt[1] = result(1);
            // double end_time = ros::Time::now().toSec();
            std::cout << "length: " << ros::Time::now().toSec() - begin_time << std::endl;
            return;
        }
    };

    struct g2g {
        double gx, gy;
        g2g(double gx, double gy)
        : gx(gx), gy(gy) {}

        void operator() ( const state_type &x , state_type &dxdt , const double  t  )
        {
            // std::cout << "x: " << std::endl;
            // std::cout << x[0] << ", " << x[1] << std::endl;
            double goal_norm = sqrt(pow(gx - x[0], 2) + pow(gy - x[1], 2));
            Eigen::Vector2d u_desired(0.0, 0.0);

            if (goal_norm < 0.1) {
                u_desired(0) = 0;
                u_desired(1) = 0;
            } else {
                u_desired(0) = (gx - x[0]);
                u_desired(1) = (gy - x[1]);
            }

            double K_acc = 3.0;

            // calculate acceleration
            // may need prior 2 instead?
            // these need to be in world frame?
            double a_x_rbt = -K_acc*(x[2] - u_desired(0));
            double a_y_rbt = -K_acc*(x[3] - u_desired(1));

            dxdt[0] = x[2]; // rbt_x
            dxdt[1] = x[3]; // rbt_y
            dxdt[2] = a_x_rbt; // rbt_v_x
            dxdt[3] = a_y_rbt;
        }
    };

    
    struct clf_cbf {
        double gx, gy;
        double K_des;
        double cbf_param;
        double r_min;
        bool _axial;
        double x1, x2, y1, y2;
        geometry_msgs::TransformStamped rbt2odom, odom2rbt;
        clf_cbf(double x1, double x2, double y1, double y2, bool axial, double gx, double gy, double K_des, double cbf_param, double r_min, geometry_msgs::TransformStamped rbt2odom, geometry_msgs::TransformStamped odom2rbt)
        : x1(x1), x2(x2), y1(y1), y2(y2), _axial(axial), gx(gx), gy(gy), K_des(K_des), cbf_param(cbf_param), r_min(r_min), rbt2odom(rbt2odom), odom2rbt(odom2rbt) {}
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

        Eigen::Vector4d mod_pol_to_cart(Eigen::VectorXd y) 
        {
            // x state:
            // [r_x, r_y, v_x, v_y]
            Eigen::Vector4d x(0.0, 0.0, 0.0, 0.0);
            x(0) = -(1 / y(0)) * y(1);
            x(1) = (1 / y(0)) * y(2);
            x(2) = (1 / y(0)) * (-y(3) * y(1) - y(4)*y(2));
            x(3) = (1 / y(0)) * (y(3) * y(2) - y(4)*y(1));
            return x;
        }

        double cbf_right(const state_type &x, geometry_msgs::Vector3Stamped right_rel_pos_rbt_frame, geometry_msgs::Vector3Stamped right_rel_vel_rbt_frame) {
            // current design: h_right = (r - r_min)*-betadot
            // these x and y need to be in the world frame (they are currently in rbt frame)
            double r_xo = x[0];
            double r_yo = x[1];
            double v_xo = x[2];
            double v_yo = x[3];
            double r_xt_r = r_xo + right_rel_pos_rbt_frame.vector.x;
            double r_yt_r = r_yo + right_rel_pos_rbt_frame.vector.y;
            double v_xt_r = v_xo + right_rel_vel_rbt_frame.vector.x;
            double v_yt_r = v_yo + right_rel_vel_rbt_frame.vector.y;

            double r = pow(pow(r_xo - r_xt_r, 2) + pow(r_yo - r_yt_r, 2), 0.5);
            double betadot = (((r_xo - r_xt_r)*(v_yo - v_yt_r) - (r_yo - r_yt_r)*(v_xo - v_xt_r)))/(pow(r_xo - r_xt_r, 2) + pow(r_yo - r_yt_r, 2));
            double h_right = (r - r_min) * -betadot;
            return h_right;
        }

        double cbf_left(const state_type &x, geometry_msgs::Vector3Stamped left_rel_pos_rbt_frame, geometry_msgs::Vector3Stamped left_rel_vel_rbt_frame) {
            // current design: h_left = (r - r_min)*betadot
            double r_xo = x[0];
            double r_yo = x[1];
            double v_xo = x[2];
            double v_yo = x[3];
            double r_xt_l = r_xo + left_rel_pos_rbt_frame.vector.x;
            double r_yt_l = r_yo + left_rel_pos_rbt_frame.vector.y;
            double v_xt_l = v_xo + left_rel_vel_rbt_frame.vector.x;
            double v_yt_l = v_yo + left_rel_vel_rbt_frame.vector.y;
            
            double r = pow(pow(r_xo - r_xt_l, 2) + pow(r_yo - r_yt_l, 2), 0.5);
            double betadot = (((r_xo - r_xt_l)*(v_yo - v_yt_l) - (r_yo - r_yt_l)*(v_xo - v_xt_l)))/(pow(r_xo - r_xt_l, 2) + pow(r_yo - r_yt_l, 2));
            double h_left = (r - r_min) * betadot;
            return h_left;
        }

        Eigen::Vector2d cbf_partials_left(const state_type &x, geometry_msgs::Vector3Stamped left_rel_pos_rbt_frame, geometry_msgs::Vector3Stamped left_rel_vel_rbt_frame) {
            // current design: h_left = (r - r_min)*betadot
            Eigen::Vector2d d_h_left_dx(0.0, 0.0);
            double r_xo = x[0];
            double r_yo = x[1];
            double v_xo = x[2];
            double v_yo = x[3];
            double r_xt_l = r_xo + left_rel_pos_rbt_frame.vector.x;
            double r_yt_l = r_yo + left_rel_pos_rbt_frame.vector.y;
            double v_xt_l = v_xo + left_rel_vel_rbt_frame.vector.x;
            double v_yt_l = v_yo + left_rel_vel_rbt_frame.vector.y;
            double r = pow(pow(r_xo - r_xt_l, 2) + pow(r_yo - r_yt_l, 2), 0.5);
            d_h_left_dx(0) = ((2*r_xo - 2*r_xt_l)*((r_xo - r_xt_l)*(v_yo - v_yt_l) - (r_yo - r_yt_l)*(v_xo - v_xt_l)))/(2*pow(pow(r_xo - r_xt_l,2) + pow(r_yo - r_yt_l,2),1.5)) - ((v_yo - v_yt_l)*(r_min - r))/(pow(r_xo - r_xt_l,2) + pow(r_yo - r_yt_l,2)) + ((2*r_xo - 2*r_xt_l)*((r_xo - r_xt_l)*(v_yo - v_yt_l) - (r_yo - r_yt_l)*(v_xo - v_xt_l))*(r_min - r))/pow(pow(r_xo - r_xt_l,2) + pow(r_yo - r_yt_l,2),2);
            d_h_left_dx(1) = ((v_xo - v_xt_l)*(r_min - r))/(pow(r_xo - r_xt_l,2) + pow(r_yo - r_yt_l,2)) + ((2*r_yo - 2*r_yt_l)*((r_xo - r_xt_l)*(v_yo - v_yt_l) - (r_yo - r_yt_l)*(v_xo - v_xt_l)))/(2*pow(pow(r_xo - r_xt_l,2) + pow(r_yo - r_yt_l,2), 1.5)) + ((2*r_yo - 2*r_yt_l)*((r_xo - r_xt_l)*(v_yo - v_yt_l) - (r_yo - r_yt_l)*(v_xo - v_xt_l))*(r_min - r))/pow(pow(r_xo - r_xt_l,2) + pow(r_yo - r_yt_l,2),2);
            return d_h_left_dx;
        }

        Eigen::Vector2d cbf_partials_right(const state_type &x, geometry_msgs::Vector3Stamped right_rel_pos_rbt_frame, geometry_msgs::Vector3Stamped right_rel_vel_rbt_frame) {
            // current design: h_right = (r - r_min)*-betadot
            Eigen::Vector2d d_h_right_dx(0.0, 0.0);
            double r_xo = x[0];
            double r_yo = x[1];
            double v_xo = x[2];
            double v_yo = x[3];
            double r_xt_r = r_xo + right_rel_pos_rbt_frame.vector.x;
            double r_yt_r = r_yo + right_rel_pos_rbt_frame.vector.y;
            double v_xt_r = v_xo + right_rel_vel_rbt_frame.vector.x;
            double v_yt_r = v_yo + right_rel_vel_rbt_frame.vector.y;
            double r = pow(pow(r_xo - r_xt_r, 2) + pow(r_yo - r_yt_r, 2), 0.5);
            d_h_right_dx(0) = ((v_yo - v_yt_r)*(r_min - r))/(pow(r_xo - r_xt_r,2) + pow(r_yo - r_yt_r,2)) - ((2*r_xo - 2*r_xt_r)*((r_xo - r_xt_r)*(v_yo - v_yt_r) - (r_yo - r_yt_r)*(v_xo - v_xt_r)))/(2*pow(pow(r_xo - r_xt_r,2) + pow(r_yo - r_yt_r,2), 1.5)) - ((2*r_xo - 2*r_xt_r)*((r_xo - r_xt_r)*(v_yo - v_yt_r) - (r_yo - r_yt_r)*(v_xo - v_xt_r))*(r_min - r))/pow(pow(r_xo - r_xt_r,2) + pow(r_yo - r_yt_r,2),2);
            d_h_right_dx(1) = -((v_xo - v_xt_r)*(r_min - r))/(pow(r_xo - r_xt_r,2) + pow(r_yo - r_yt_r,2)) - ((2*r_yo - 2*r_yt_r)*((r_xo - r_xt_r)*(v_yo - v_yt_r) - (r_yo - r_yt_r)*(v_xo - v_xt_r)))/(2*pow(pow(r_xo - r_xt_r,2) + pow(r_yo - r_yt_r,2),1.5)) - ((2*r_yo - 2*r_yt_r)*((r_xo - r_xt_r)*(v_yo - v_yt_r) - (r_yo - r_yt_r)*(v_xo - v_xt_r))*(r_min - r))/pow(pow(r_xo - r_xt_r,2) + pow(r_yo - r_yt_r,2),2);
            return d_h_right_dx;
        }

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            // std::cout << "x " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << ", " << x[4] << ", " << x[5] << ", " << x[6] << ", "  << x[7] << ", " << x[8] << ", " << x[9] << ", " << x[10] << ", " << x[11] << ", " << x[12] << ", " << x[13] << std::endl;
            // state:
            // [rbt_x, rbt_y, rbt_v_x, rbt_v_y,
            //  1 / r_left, sin(beta_left), cos(beta_left), rdot_left / r_left, betadot_left
            //  1 / r_right, sin(beta_right), cos(beta_right), rdot_right / r_right, betadot_right]
            // synthesize desired control
            // std::cout << "calculating u_des" << std::endl;
            double begin_time = ros::Time::now().toSec();
            double V = pow(gx - x[0], 2) + pow(gy - x[1], 2);
            // std::cout << "V: " << V << std::endl;
            Eigen::Vector2d v_des(0.0, 0.0);
            if (V < 0.1) {
                v_des(0) = 0;
                v_des(1) = 0;
            } else {
                v_des(0) = -K_des*(x[0] - gx);
                v_des(1) = -K_des*(x[1] - gy);
            }
            // std::cout << "v_des: " << v_des[0] << ", " << v_des[1] << std::endl;
            
            Eigen::VectorXd y_left(5);
            y_left << x[4], x[5], x[6], x[7], x[8];
            // std::cout << "y_left: " << y_left << std::endl;
            Eigen::VectorXd y_right(5);
            y_right << x[9], x[10], x[11], x[12], x[13];
            // std::cout << "y_right: " << y_right << std::endl;

            //std::cout << "modifying polar to cartesian" << std::endl;
            // the x_left and x_right returned here are in world frame
            Eigen::Vector4d x_left = mod_pol_to_cart(y_left);
            Eigen::Vector4d x_right = mod_pol_to_cart(y_right);
            //std::cout << "y_left: " << y_left(0) << ", " << y_left(1) << ", " << y_left(2) << ", " << y_left(3) << ", " << y_left(4) << ", y_right: " << y_right(0) << ", " << y_right(1) << ", " << y_right(2) << ", " << y_right(3) << ", " << y_right(4) <<std::endl;
            //std::cout << "x_left: " << x_left(0) << ", " << x_left(1) << ", " << x_left(2) << ", " << x_left(3) << ", x_right: " << x_right(0) << ", " << x_right(1) << ", " << x_right(2) << ", " << x_right(3) << std::endl;
            
            geometry_msgs::Vector3Stamped left_rel_pos_odom_frame;
            geometry_msgs::Vector3Stamped left_rel_vel_odom_frame;
            geometry_msgs::Vector3Stamped right_rel_pos_odom_frame;
            geometry_msgs::Vector3Stamped right_rel_vel_odom_frame;

            left_rel_pos_odom_frame.vector.x = x_left(0);
            left_rel_pos_odom_frame.vector.y = x_left(1);
            left_rel_vel_odom_frame.vector.x = x_left(2);
            left_rel_vel_odom_frame.vector.y = x_left(3);
            right_rel_pos_odom_frame.vector.x = x_right(0);
            right_rel_pos_odom_frame.vector.y = x_right(1);
            right_rel_vel_odom_frame.vector.x = x_right(2);
            right_rel_vel_odom_frame.vector.y = x_right(3);

            geometry_msgs::Vector3Stamped left_rel_pos_rbt_frame;
            geometry_msgs::Vector3Stamped left_rel_vel_rbt_frame;
            geometry_msgs::Vector3Stamped right_rel_pos_rbt_frame;
            geometry_msgs::Vector3Stamped right_rel_vel_rbt_frame;

            // in, out, transform
            tf2::doTransform(left_rel_pos_odom_frame, left_rel_pos_rbt_frame, odom2rbt);
            tf2::doTransform(left_rel_vel_odom_frame, left_rel_vel_rbt_frame, odom2rbt);
            tf2::doTransform(right_rel_pos_odom_frame, right_rel_pos_rbt_frame, odom2rbt);
            tf2::doTransform(right_rel_vel_odom_frame, right_rel_vel_rbt_frame, odom2rbt);

            double h_dyn;
            Eigen::Vector2d d_h_dyn_dx;
            // calculate left and right CBF values
            // TRANSFORM X_LEFT AND X_RIGHT TO RBT FRAME
            double h_dyn_left = cbf_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
            double h_dyn_right = cbf_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
            Eigen::Vector2d d_h_dyn_left_dx = cbf_partials_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
            Eigen::Vector2d d_h_dyn_right_dx = cbf_partials_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
            //std::cout << "left CBF value is: " << h_dyn_left << " with partials: " << d_h_dyn_left_dx(0) << ", " << d_h_dyn_left_dx(1);
            //std::cout << ",         right CBF value is: " << h_dyn_right << " with partials: " << d_h_dyn_right_dx(0) << ", " << d_h_dyn_right_dx(1) << std::endl;

            if (h_dyn_left < h_dyn_right) {
                h_dyn = h_dyn_left;
                d_h_dyn_dx = d_h_dyn_left_dx;
            } else {
                h_dyn = h_dyn_right;
                d_h_dyn_dx = d_h_dyn_right_dx;
            }

            // calculate Psi
            double Psi = v_des.dot(d_h_dyn_dx) + cbf_param * h_dyn;
            //std::cout << "Psi is " << Psi << std::endl;
            Eigen::Vector2d v_actual(0.0, 0.0);

            // calculate actual control input
            if (Psi < 0) {
                // std::cout << "adding CBF" << std::endl;
                Eigen::Vector2d Lg_h = d_h_dyn_dx; // Lie derivative of h wrt x
                v_actual = v_des + -(Lg_h * Psi) / (Lg_h.dot(Lg_h));
            } else {
                //std::cout << "not adding CBF" << std::endl;
                v_actual = v_des;
            }

            // check to see if robot has passed gap

            // std::cout << "updating derivatives" << std::endl;
            
            Eigen::Vector2d result(0, 0);
            Eigen::Vector2d rbt(x[0], x[1]);
            Eigen::Vector2d p1(x1, y1);
            Eigen::Vector2d p2(x2, y2);
            Eigen::Vector2d goal_pt(gx, gy);

            bool pass_gap;
            if (_axial) {
                pass_gap = (rbt.norm() > std::min(p1.norm(), p2.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
            } else {
                pass_gap = (rbt.norm() > std::max(p1.norm(), p2.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
            }
            
            if (pass_gap) {
                //std::cout << "gap passed" << std::endl;
                result = Eigen::Vector2d(0.0, 0.0);
            } else {
                result = v_actual;
            }
            
            //std::cout << "result: " << result(0) << ", " << result(1) << std::endl;

            double K_acc = 3.0;

            // calculate acceleration
            // may need prior 2 instead?
            // these need to be in world frame?
            double a_x_rbt = -K_acc*(x[2] - result(0));
            double a_y_rbt = -K_acc*(x[3] - result(1));
            
            geometry_msgs::Vector3Stamped rbt_accel_rbt_frame;

            rbt_accel_rbt_frame.vector.x = a_x_rbt;
            rbt_accel_rbt_frame.vector.y = a_y_rbt;

            geometry_msgs::Vector3Stamped rbt_accel_odom_frame;
            tf2::doTransform(rbt_accel_rbt_frame, rbt_accel_odom_frame, rbt2odom);
            double a_x_world = rbt_accel_odom_frame.vector.x;
            double a_y_world = rbt_accel_odom_frame.vector.y;
            // need a_x, a_y for world frame
            //std::cout << "a_x_rbt: " << a_x_rbt << ", a_y_rbt: " << a_y_rbt << std::endl;

            dxdt[0] = x[2]; // rbt_x
            dxdt[1] = x[3]; // rbt_y
            dxdt[2] = a_x_rbt; // rbt_v_x
            dxdt[3] = a_y_rbt; // rbt_v_y

            dxdt[4] = - x[7] * x[4]; // 1 / r_left
            dxdt[5] = x[6] * x[8]; // sin(beta_left)
            dxdt[6] = -x[5] * x[8]; // cos(beta_left)
            dxdt[7] = x[8]*x[8] - x[7]*x[7] - x[4]* (a_y_world * x[6] - a_x_world * x[5]); // rdot_left / r_left
            dxdt[8] = -2 * x[7] * x[8] - x[4] * (-a_x_world*x[6] - a_y_world*x[5]); // betadot_left

            dxdt[9] = - x[12] * x[9]; // 1 / r_right
            dxdt[10] = x[11] * x[13]; // sin(beta_right)
            dxdt[11] = - x[10] * x[13]; // cos(beta_right)
            dxdt[12] = x[13]*x[13] - x[12]*x[12] - x[9] * (a_y_world * x[11] - a_x_world*x[10]); // rdot_right / r_right
            dxdt[13] = -2 * x[12] * x[13] - x[9] * (-a_x_world * x[11] - a_y_world * x[10]); // betadot_right
            std::cout << "length: " << ros::Time::now().toSec() - begin_time << std::endl;
            //std::cout << "done" << std::endl;
            return;
        }
    };
    

    // this is an observer?
    struct write_trajectory
    {
        geometry_msgs::PoseArray& _posearr;
        std::string _frame_id;
        double _coefs;

        write_trajectory(geometry_msgs::PoseArray& posearr, std::string frame_id, double coefs)
        : _posearr(posearr), _frame_id(frame_id), _coefs(coefs) { }

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
        }
    };

}

#endif
