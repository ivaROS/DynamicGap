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

            // ROTATIONAL TERM FOR RIGHT
            Eigen::Vector2d c1 = r_pi2     * (vec_1 / vec_1.norm()) * exp(-std::abs(thetax - theta1) / _sigma);
            
            // ROTATIONAL TERM FOR LEFT
            Eigen::Vector2d c2 = neg_r_pi2 * (vec_2 / vec_2.norm()) * exp(-std::abs(theta2 - thetax) / _sigma);

            // ATTRACTIVE TERM FOR GOAL
            // Since local goal will definitely be within the range of the gap, this limit poses no difference
            Eigen::Vector2d sub_goal_vec(rg * cos(new_theta), rg * sin(new_theta));

            // bool left = r2 > r1;

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

            double K_acc = 3.0;

            // calculate acceleration
            // these need to be in world frame?
            double a_x_rbt = -K_acc*(x[2] - result(0));
            double a_y_rbt = -K_acc*(x[3] - result(1));
            dxdt[0] = x[2];
            dxdt[1] = x[3];
            dxdt[2] = a_x_rbt; // rbt_v_x
            dxdt[3] = a_y_rbt; // rbt_v_y
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
            Eigen::Vector2d a_des(-K_acc*(x[2] - v_des(0)), -K_acc*(x[3] - v_des(1)));
            // std::cout << "v_des: " << v_des(0) << ", " << v_des(1)  << ". a_des: " << a_des(0) << ", " << a_des(1) << std::endl;
        
            dxdt[0] = x[2]; // rbt_x
            dxdt[1] = x[3]; // rbt_y
            dxdt[2] = a_des(0);
            dxdt[3] = a_des(1);
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
        clf_cbf(bool axial, double K_des, double cbf_param, double K_acc, double local_goal_dist, double betadot_L_0, double betadot_R_0, double vx_absmax, double vy_absmax, double init_rbt_x, double init_rbt_y)
        : _axial(axial), K_des(K_des), cbf_param(cbf_param), K_acc(K_acc), local_goal_dist(local_goal_dist), betadot_L_0(betadot_L_0), betadot_R_0(betadot_R_0), vx_absmax(vx_absmax), vy_absmax(vy_absmax), init_rbt_x(init_rbt_x), init_rbt_y(init_rbt_y) {}
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
            if (x_vel <= vx_absmax && y_vel <= vy_absmax) {
                // std::cout << "not clipping" << std::endl;
                return original_vel;
            } else {
                // std::cout << "max: " << vx_absmax << ", norm: " << original_vel.norm() << std::endl;
                Eigen::Vector2d clipped_vel = vx_absmax * original_vel / std::max(x_vel, y_vel);
                return clipped_vel;
            }
        }

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            // std::cout << "t: " << t << std::endl;
            // state:

            // synthesize desired control
            // double begin_time = ros::Time::now().toSec();
            
            double gx = local_goal_dist*x[15];
            double gy = local_goal_dist*x[14];

            Eigen::Vector2d rel_goal(gx - x[0], gy - x[1]);
            Eigen::Vector2d rbt_vel = clip_velocities(x[2], x[3]);

            if (rel_goal.dot(rbt_vel) < 0) {
                std::cout << "moving away from goal" << std::endl;
            }

            // set left/right models
            Eigen::VectorXd y_left(5);
            y_left << x[4], x[5], x[6], x[7], x[8];
            Eigen::VectorXd y_right(5);
            y_right << x[9], x[10], x[11], x[12], x[13];

            // obtain cartesian/rbt frame info from models
            Eigen::Vector4d x_left = mod_pol_to_cart(y_left);
            Eigen::Vector4d x_right = mod_pol_to_cart(y_right);

            Eigen::Vector2d left_rel_pos_rbt_frame(x_left(0), x_left(1));
            Eigen::Vector2d left_rel_vel_rbt_frame(x_left(2), x_left(3));
            Eigen::Vector2d right_rel_pos_rbt_frame(x_right(0), x_right(1));
            Eigen::Vector2d right_rel_vel_rbt_frame(x_right(2), x_right(3));

            Eigen::Vector2d goal_rel_pos_rbt_frame(gx, gy);

            double V = pow(rel_goal[0], 2) + pow(rel_goal[1], 2);
            bool pass_gap = false;
            Eigen::Vector2d rbt(x[0], x[1]);

            //p1 = rbt + right_rel_pos_rbt_frame;
            //p2 = rbt + left_rel_pos_rbt_frame;
            Eigen::Vector2d goal_pt(gx - init_rbt_x, gy - init_rbt_y);
            Eigen::Vector2d rbt_traveled(x[0] - init_rbt_x, x[1] - init_rbt_y);
            
        
            //std::cout << "p1: " << p1[0] << ", " << p1[1] << std::endl;
            //std::cout << "p2: " << p2[0] << ", " << p2[1] << std::endl;
            

            pass_gap = rbt_traveled.norm() > goal_pt.norm();
            /*
            if (_axial) {
                pass_gap = (rbt.norm() > std::min(p1.norm(), p2.norm()) + 0.18 || rbt.norm() > goal_pt.norm());
            } else {
                pass_gap = (rbt.norm() > std::max(p1.norm(), p2.norm()) + 0.18 || rbt.norm() > goal_pt.norm());
            }
            */

            // WRONG
            // need to know what side the gap points START on
            // then we can know if it's crossed over
            
            // getting angle swept from L to R. If result is positive, gap is convex. If result is negative, gap is non-cvx.
            // treating left point as p1, right point as p2
            double dot = left_rel_pos_rbt_frame.dot(right_rel_pos_rbt_frame);
            double det = left_rel_pos_rbt_frame(0)*right_rel_pos_rbt_frame(1) - right_rel_pos_rbt_frame(0)*left_rel_pos_rbt_frame(1);

            double gap_angle = -atan2(det, dot);

            if (gap_angle < 0) {
                gap_angle += 2*M_PI;
            }

            double beta_left = atan2(y_left[1], y_left[2]);
            double beta_right = atan2(y_right[1], y_right[2]);

            bool closed_gap = false;
            // Now, we ONLY look at the convex gaps and see if they have closed. If a non-convex gap has closed, it happened behind us and we do not care
            if (0 < gap_angle && gap_angle < M_PI) {
                if ((beta_left > 0 && beta_right > 0) || (beta_left < 0 && beta_right < 0)) {
                    closed_gap = beta_left < beta_right;
                }
            }

            // how to determine gap angle (to tell if gap has become non-convex)
            // how to tell if gap has closed
            // add piece for closed gap?
            if (pass_gap || closed_gap) {
                std::cout << "past gap: " << pass_gap << ", closed gap: " << closed_gap << std::endl;
                dxdt[0] = 0; dxdt[1] = 0; dxdt[2] = 0; dxdt[3] = 0; dxdt[4] = 0;
                dxdt[5] = 0; dxdt[6] = 0; dxdt[7] = 0; dxdt[8] = 0; dxdt[9] = 0; dxdt[10] = 0;
                dxdt[11] = 0; dxdt[12] = 0; dxdt[13] = 0; dxdt[14] = 0; dxdt[15] = 0;
                return;
            }

            // CLIPPING VELOCITIES AFTER INTEGRATION
            //Eigen::Vector2d vels = clip_velocities(dxdt[0], dxdt[1]);
            //dxdt[0] = vels[0];
            //dxdt[1] = vels[1]; 


            std::cout << "t: " << t << ", x: " << x[0] << ", " << x[1] << ", " << rbt_vel[0] << ", " << rbt_vel[1] << std::endl;
            std::cout << "V: " << V << ", local goal: " << gx << ", " << gy << ", " << std::endl;
            
            std::cout << "y_left: " << y_left(0) << ", " << std::atan2(y_left(1),y_left(2)) << ", " << y_left(3) << ", " << y_left(4) << ", y_right: " << y_right(0) << ", " << std::atan2(y_right(1), y_right(2)) << ", " << y_right(3) << ", " << y_right(4) << std::endl;
            std::cout << "x_left: " << x_left(0) << ", " << x_left(1) << ", " << x_left(2) << ", " << x_left(3) << ", x_right: " << x_right(0) << ", " << x_right(1) << ", " << x_right(2) << ", " << x_right(3) << std::endl;

            Eigen::Vector2d v_des(0.0, 0.0);
            // If V sufficiently large, set desired velocity
            if (V > 0.1) {
                v_des(0) = -K_des*(x[0] - gx);
                v_des(1) = -K_des*(x[1] - gy);
            }

            // CLIPPING DESIRED VELOCITIES
            v_des = clip_velocities(v_des[0], v_des[1]);

            // set desired acceleration based on desired velocity
            Eigen::Vector2d a_des(-K_acc*(rbt_vel[0] - v_des(0)), -K_acc*(rbt_vel[1] - v_des(1)));
            std::cout << "v_des: " << v_des(0) << ", " << v_des(1)  << ", a_des: " << a_des(0) << ", " << a_des(1) << std::endl;
            
            Eigen::Vector2d a_actual = a_des;
            if (V > 0.1) {
                // std::cout << "adding CBF" << std::endl;
                double h_dyn = 0.0;
                Eigen::Vector4d d_h_dyn_dx(0.0, 0.0, 0.0, 0.0);
                // calculate left and right CBF values
                
                double h_dyn_left = cbf_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                double h_dyn_right = cbf_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                Eigen::Vector4d d_h_dyn_left_dx = cbf_partials_left(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame);
                Eigen::Vector4d d_h_dyn_right_dx = cbf_partials_right(x, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                std::cout << "left CBF value is: " << h_dyn_left << ", right CBF value is: " << h_dyn_right << std::endl;
                // " with partials: " << d_h_dyn_left_dx(0) << ", " << d_h_dyn_left_dx(1) << ", " << d_h_dyn_left_dx(2) << ", " << d_h_dyn_left_dx(3) << std::endl;
                // std::cout << << " with partials: " << d_h_dyn_right_dx(0) << ", " << d_h_dyn_right_dx(1) << ", " << d_h_dyn_right_dx(2) << ", " << d_h_dyn_right_dx(3) << std::endl;
                
                // check for convexity of gap
                std::cout << "gap angle: " << gap_angle << std::endl;
                if (0 < gap_angle && gap_angle < M_PI) {
                    //h_dyn = time_to_pass_CBF(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                    //d_h_dyn_dx = time_to_pass_CBF_partials(x, left_rel_pos_rbt_frame, left_rel_vel_rbt_frame, right_rel_pos_rbt_frame, right_rel_vel_rbt_frame);
                    if (h_dyn_left < h_dyn_right) {
                        h_dyn = h_dyn_left;
                        d_h_dyn_dx = d_h_dyn_left_dx;
                    } else if (h_dyn_right < h_dyn_left) {
                        h_dyn = h_dyn_right;
                        d_h_dyn_dx = d_h_dyn_right_dx;
                    }
                }

                std::cout << "h_dyn: " << h_dyn << ", d_h_dyn_dx: " << d_h_dyn_dx[0] <<  ", " << d_h_dyn_dx[1] << ", " << d_h_dyn_dx[2] << ", " << d_h_dyn_dx[3] << std::endl;
                // calculate Psi
                Eigen::Vector4d d_x_dt(rbt_vel[0], rbt_vel[1], a_des[0], a_des[1]);
                double Psi = d_h_dyn_dx.dot(d_x_dt) + cbf_param * h_dyn;
                std::cout << "Psi is " << Psi << std::endl;
                
                if (Psi < 0.0) {
                    Eigen::Vector2d Lg_h(d_h_dyn_dx[2], d_h_dyn_dx[3]); // Lie derivative of h wrt x
                    a_actual = a_des + -(Lg_h * Psi) / (Lg_h.dot(Lg_h));
                }
            } 
            
            /*
            else {
                //std::cout << "not adding CBF" << std::endl;
                a_actual = a_des;
            }
            */

            std::cout << "a_actual: " << a_actual(0) << ", " << a_actual(1) << std::endl;
            
            double a_x_rbt = a_actual(0); // -K_acc*(x[2] - result(0)); // 
            double a_y_rbt = a_actual(1); // -K_acc*(x[3] - result(1)); // 

            double a_x_rel = 0 - a_x_rbt;
            double a_y_rel = 0 - a_y_rbt;

            dxdt[0] = rbt_vel[0]; // rbt_x
            dxdt[1] = rbt_vel[1]; // rbt_y
            dxdt[2] = a_x_rbt; // rbt_v_x
            dxdt[3] = a_y_rbt; // rbt_v_y

            dxdt[4] = - x[7] * x[4]; // 1 / r_left
            dxdt[5] = x[6] * x[8]; // sin(beta_left)
            dxdt[6] = -x[5] * x[8]; // cos(beta_left)
            dxdt[7] = x[8]*x[8] - x[7]*x[7] + x[4]* (a_x_rel * x[6] + a_y_rel * x[5]); // rdot_left / r_left
            dxdt[8] = - 2 * x[7] * x[8] + x[4] * (-a_x_rel*x[5] + a_y_rel*x[6]); // betadot_left

            dxdt[9] = - x[12] * x[9]; // 1 / r_right
            dxdt[10] = x[11] * x[13]; // sin(beta_right)
            dxdt[11] = - x[10] * x[13]; // cos(beta_right)
            dxdt[12] = x[13]*x[13] - x[12]*x[12] + x[9] * (a_x_rel*x[11] + a_y_rel * x[10]); // rdot_right / r_right
            dxdt[13] = - 2 * x[12] * x[13] + x[9] * (-a_x_rel * x[10] + a_y_rel * x[11]); // betadot_right

            dxdt[14] = 0;
            dxdt[15] = 0;
            /*
            // do we need to move goal
            // should I put a thing for V < 0.1? May be good to not, pushes goal away from oncoming gap point?
            double goal_gap_pt_thresh = 0.25; // distance of 0.1
            double goal_left_distance = sqrt(pow(rel_goal[0] - x_left[0], 2) + pow(rel_goal[1] - x_left[1], 2));
            double goal_right_distance = sqrt(pow(rel_goal[0] - x_right[0], 2) + pow(rel_goal[1] - x_right[1], 2));
            // should not just compare thetas here, need to compare distances I think
            if (V > 0.1 && goal_left_distance < goal_gap_pt_thresh && x[8] < 0) {
                //std::cout << "adjusting goal from left" << std::endl;
                //std::cout << "goal left distance" << goal_left_distance << std::endl;
                //std::cout << "left beta: " << std::atan2(x[5], x[6]) << ", local goal beta: " << std::atan2(x[14], x[15]) << std::endl;
                dxdt[14] = x[15] * x[8];
                dxdt[15] = -x[14] * x[8];
            } else if (V > 0.1 && goal_right_distance < goal_gap_pt_thresh && x[13] > 0) {
                //std::cout << "adjusting goal from right" << std::endl;
                //std::cout << "goal right distance" << goal_right_distance << std::endl;
                //std::cout << "right beta: " << std::atan2(x[10], x[11]) << ", local goal beta: " << std::atan2(x[14], x[15]) << std::endl;
                dxdt[14] = x[15] * x[13];
                dxdt[15] = -x[14] * x[13];
            }
            */
            //std::cout << "length: " << ros::Time::now().toSec() - begin_time << std::endl;
            // std::cout << "done" << std::endl;
            std::cout << "~~~~~~~~~~~~" << std::endl;
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
