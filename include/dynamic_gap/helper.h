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
    typedef boost::array<double, 16> state_type;
    //typedef state_type::index_range range;

    
    struct polar_gap_field{

        double x1, x2, y1, y2, gx, gy;
        double close_pt_x, close_pt_y, far_pt_x, far_pt_y, far_vec_x, far_vec_y, rbt_vec_x, rbt_vec_y, angle_gap;
        double dir_vec_x, dir_vec_y;
        double _sigma;
        bool mode_agc, pivoted_left, _axial;
        double rbt_x_0, rbt_y_0;    
        double K_acc;
        double v_lin_max, a_lin_max;

        polar_gap_field(double x1, double x2, double y1, double y2, double gx, 
                        double gy, bool mode_agc, bool pivoted_left, bool axial, 
                        double sigma, double rbt_x_0, double rbt_y_0, double K_acc,
                        double v_lin_max, double a_lin_max)
            : x1(x1), x2(x2), y1(y1), y2(y2), gx(gx), gy(gy), mode_agc(mode_agc), 
              pivoted_left(pivoted_left), _axial(axial), _sigma(sigma), rbt_x_0(rbt_x_0), 
              rbt_y_0(rbt_y_0), K_acc(K_acc), v_lin_max(v_lin_max), a_lin_max(a_lin_max) {}

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

                //double vec_1_norm = vec_1.norm();
                //double vec_2_norm = vec_2.norm();
                //double w1 = vec_2_norm / sqrt(pow(vec_1_norm, 2) + pow(vec_2_norm,2));
                //double w2 = vec_1_norm / sqrt(pow(vec_1_norm, 2) + pow(vec_2_norm,2));
                Eigen::Vector2d weighted_circulation_sum = c1 + c2;
                Eigen::Vector2d circulation_field = coeffs * weighted_circulation_sum / weighted_circulation_sum.norm();
                Eigen::Vector2d attraction_field = sub_goal_vec / sub_goal_vec.norm();
                // std::cout << "inte_t: " << t << std::endl;
                //std::cout << "robot to left: (" << vec_2[0] << ", " << vec_2[1] << "), robot to right: (" << vec_1[0] << ", " << vec_1[1] << ")" << std::endl;
                //std::cout << "angular difference to left: " << ang_diff_2 << ", angular difference to right: " << ang_diff_1 << std::endl;
                //std::cout << "left weight: " << w2 << ", left circulation component: (" << c2[0] << ", " << c2[1] << "), right weight: " << w1 << ", right circulation component: (" << c1[0] << ", " << c1[1] << ")" << std::endl;  
                // std::cout << "circulation: (" << circulation_field[0] << ", " << circulation_field[1] << ")" << std::endl;
                //std::cout << "robot to goal: (" << goal_vec[0] << ", " << goal_vec[1] << ")" << std::endl;
                // std::cout << "attraction: (" << attraction_field[0] << ", " << attraction_field[1] << ")" << std::endl;
                vel_des = circulation_field + attraction_field;
            }
            vel_des = clip_velocities(vel_des[0], vel_des[1], v_lin_max);
            Eigen::Vector2d acc(K_acc*(vel_des(0) - x[2]), K_acc*(vel_des(1) - x[3]));
            acc = clip_velocities(acc[0], acc[1], a_lin_max);

            dxdt[0] = x[2];
            dxdt[1] = x[3];
            dxdt[2] = acc[0];
            dxdt[3] = acc[1];
            return;
        }
    };

    struct reachable_gap_APF {
        Eigen::Vector2d rel_left_vel, rel_right_vel, 
                        goal_pt_0, goal_pt_1;
        std::vector<std::vector <double>> left_curve, right_curve;

        int num_curve_points, N, Kplus1, half_num_scan;
        double v_lin_max, a_lin_max, K_acc,
               rg, theta_right, theta_left, thetax, thetag, new_theta, ang_diff_right, ang_diff_left, 
               coeffs, rel_right_pos_norm, rel_left_pos_norm, w_left, w_right, a_x_rbt, a_y_rbt, a_x_rel, a_y_rel, v_nom,
               r_reach, theta, r_inscr, left_weight, right_weight; 
        bool mode_agc, pivoted_left, _axial, past_gap_points, past_goal, past_left_point, past_right_point, pass_gap;
        Eigen::Vector2d init_rbt_pos, rbt, rel_right_pos, rel_left_pos, abs_left_pos, abs_right_pos, 
                        abs_goal_pos, rel_goal_pos, c_left, c_right, sub_goal_vec, v_des, 
                        weighted_circulation_sum, circulation_field, attraction_field, a_des, a_actual;
        Eigen::Vector4d abs_left_state, abs_right_state, goal_state;

        Eigen::MatrixXd weights, all_centers, all_curve_pts, all_inward_norms;


        reachable_gap_APF(Eigen::Vector2d init_rbt_pos, Eigen::Vector2d goal_pt_1, double K_acc,
                          double v_lin_max, double a_lin_max, int num_curve_points, 
                          Eigen::MatrixXd all_curve_pts,  Eigen::MatrixXd all_centers, Eigen::MatrixXd all_inward_norms,
                          double left_weight, double right_weight) 
                          : init_rbt_pos(init_rbt_pos), goal_pt_1(goal_pt_1), K_acc(K_acc), 
                            v_lin_max(v_lin_max), a_lin_max(a_lin_max), num_curve_points(num_curve_points), 
                            all_curve_pts(all_curve_pts), all_centers(all_centers), all_inward_norms(all_inward_norms), 
                            left_weight(left_weight), right_weight(right_weight)
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
            double eps = 0.0000001;

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
            abs_left_state << new_x[4], new_x[5], new_x[6], new_x[7];  
            abs_right_state << new_x[8], new_x[9], new_x[10], new_x[11];
            goal_state << new_x[12], new_x[13], new_x[14], new_x[15];
               
            // Just use the same state to be able to make these past checks
            rbt << new_x[0], new_x[1];
            abs_left_pos << abs_left_state[0], abs_left_state[1];
            abs_right_pos << abs_right_state[0], abs_right_state[1];
            abs_goal_pos << goal_state[0], goal_state[1];

            rel_left_pos = abs_left_pos - rbt;
            rel_right_pos = abs_right_pos - rbt;
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

            // ROS_INFO_STREAM("t: " << t);
            // ROS_INFO_STREAM("rbt state: " << new_x[0] << ", " << new_x[1] << ", " << new_x[2] << ", " << new_x[3]);
            // ROS_INFO_STREAM("left_pos: " << abs_left_pos[0] << ", " << abs_left_pos[1]);            
            // ROS_INFO_STREAM("right_pos: " << abs_right_pos[0] << ", " << abs_right_pos[1]);
            // ROS_INFO_STREAM("goal_pos: " << abs_goal_pos[0] << ", " << abs_goal_pos[1]);            
            /*
            int left_idx = (int) half_num_scan*std::atan2(abs_left_pos[1], abs_left_pos[0]) / M_PI + half_num_scan;
            int right_idx = (int) half_num_scan*std::atan2(abs_right_pos[1], abs_right_pos[0]) / M_PI + half_num_scan;
            int rbt_idx = (int) half_num_scan*std::atan2(rbt[1], rbt[0]) / M_PI + half_num_scan;
            
            if (left_idx < rbt_idx || right_idx > rbt_idx) {
                ROS_INFO_STREAM("potential exit at " << t);
                ROS_INFO_STREAM("rbt_idx: " << rbt_idx << ", left_idx: " << left_idx << ", right_idx: " << right_idx);
            }
            */
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
            for (int i = 0; i < Kplus1; i++) {

                Eigen::Vector2d center_i = all_centers.row(i);
                Eigen::Vector2d diff = rbt - center_i;

                Eigen::Vector2d gradient = diff / pow(diff.norm() + eps, 2);
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
            /*
            a_des << K_acc*(v_cmd[0] - new_x[2]), K_acc*(v_cmd[1] - new_x[3]);
            a_des = clip_velocities(a_des[0], a_des[1], a_lin_max);
            // ROS_INFO_STREAM("a_des: " << a_des[0] << ", " << a_des[1]);
            double a_x_rbt = a_des(0); // -K_acc*(x[2] - result(0)); // 
            double a_y_rbt = a_des(1); // -K_acc*(x[3] - result(1)); // 

            double a_x_rel = 0 - a_x_rbt;
            double a_y_rel = 0 - a_y_rbt;
            */
            dxdt[0] = v_cmd[0]; // rbt_x
            dxdt[1] = v_cmd[1]; // rbt_y
            dxdt[2] = 0.0; // rbt_v_x
            dxdt[3] = 0.0; // rbt_v_y

            dxdt[4] = new_x[6]; // left point r_x (absolute)
            dxdt[5] = new_x[7]; // left point r_y (absolute)
            dxdt[6] = 0.0; // left point v_x (absolute) 
            dxdt[7] = 0.0; // left point v_y (absolute)

            dxdt[8] = new_x[10]; // right point r_x (absolute)
            dxdt[9] = new_x[11]; // right point r_y (absolute)
            dxdt[10] = 0.0; // right point v_x (absolute)
            dxdt[11] = 0.0; // right point v_y (absolute)
            dxdt[12] = new_x[14]; // goal point r_x (absolute)
            dxdt[13] = new_x[15]; // goal point r_y (absolute)
            dxdt[14] = 0.0; // goal point v_x (absolute)
            dxdt[15] = 0.0; // goal point v_y (absolute)
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
            double goal_norm = sqrt(pow(x[12] - x[0], 2) + pow(x[13] - x[1], 2));
            Eigen::Vector2d v_des(0.0, 0.0);

            // ROS_INFO_STREAM("t: " << t << ", x: " << x[0] << ", " << x[1] << ", goal: " << x[12] << ", " << x[13] << ", goal_norm: " << goal_norm);

            if (goal_norm < 0.1) {  // we want to make it so once robot reaches gap, trajectory ends, even if goal keeps moving
                // ROS_INFO_STREAM("t: " << t << ", stopping at x: " << x[0] << ", " << x[1] << ", goal: " << x[12] << ", " << x[13]);
                v_des(0) = 0;
                v_des(1) = 0;

                dxdt[0] = 0.0;
                dxdt[1] = 0.0;
                dxdt[12] = 0.0;
                dxdt[13] = 0.0;
                return;
            }


            v_des(0) = (x[12] - x[0]);
            v_des(1) = (x[13] - x[1]);
            
            // ROS_INFO_STREAM("v_des: " << v_des[0] << ", " << v_des[1]);
            Eigen::Vector2d v_cmd = clip_velocities(v_des[0], v_des[1], v_lin_max);
            // ROS_INFO_STREAM("v_cmd: " << v_cmd[0] << ", " << v_cmd[1]);

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
