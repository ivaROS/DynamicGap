
#include <dynamic_gap/gap_feasibility.h>

namespace dynamic_gap {

    void GapFeasibilityChecker::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
    }

    bool GapFeasibilityChecker::indivGapFeasibilityCheck(dynamic_gap::Gap& gap) {
        // std::cout << "FEASIBILITY CHECK" << std::endl;
        bool feasible;
        auto half_num_scan = gap.half_scan;
        float x1, x2, y1, y2;

        x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
       
        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        Eigen::Vector2f pg = (pl + pr) / 2.0;

        // FLIPPING MODELS HERE
        //std::cout << "FLIPPING MODELS TO GET L/R FROM ROBOT POV" << std::endl;
        dynamic_gap::cart_model* left_model = gap.right_model;
        dynamic_gap::cart_model* right_model = gap.left_model;

        // FEASIBILITY CHECK
        // std::cout << "left ori: " << left_ori << ", right_ori: " << right_ori << std::endl;
        //std::cout << "left idx: " << gap.convex.convex_lidx << ", right idx: " << gap.convex.convex_ridx << std::endl;
        double start_time = ros::Time::now().toSec();
        feasible = feasibilityCheck(gap, left_model, right_model);
        ROS_INFO_STREAM("feasibilityCheck time elapsed:" << ros::Time::now().toSec() - start_time);
        ROS_INFO_STREAM("is gap feasible: " << feasible);
        return feasible;
    }

    bool GapFeasibilityChecker::feasibilityCheck(dynamic_gap::Gap& gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model) {
        bool feasible = false;
        Matrix<double, 4, 1> left_cart_model_state = left_model->get_cartesian_state();
        Matrix<double, 4, 1> right_cart_model_state = right_model->get_cartesian_state();
       
        left_model->freeze_robot_vel();
        right_model->freeze_robot_vel();
        left_model->copy_model();
        right_model->copy_model();

        Matrix<double, 4, 1> frozen_left_model_state = left_model->get_frozen_modified_polar_state();
        Matrix<double, 4, 1> frozen_right_model_state = right_model->get_frozen_modified_polar_state();

        double left_betadot = frozen_left_model_state[3];
        double right_betadot = frozen_right_model_state[3];
        double start_time = ros::Time::now().toSec();
        double crossing_time = gapSplinecheck(gap, left_model, right_model);
        ROS_INFO_STREAM("gapSplinecheck time elapsed:" << ros::Time::now().toSec() - start_time);

        double min_betadot = std::min(left_betadot, right_betadot);
        double subtracted_left_betadot = left_betadot - min_betadot;
        double subtracted_right_betadot = right_betadot - min_betadot;

        if (subtracted_left_betadot > 0) {
            // expanding
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setCategory("expanding");
        } else {
            if (crossing_time >= 0) {
                feasible = true;
                gap.gap_lifespan = crossing_time;
            }
            gap.setCategory("closing");
        }

        return feasible;
    }
    

    double GapFeasibilityChecker::gapSplinecheck(dynamic_gap::Gap & gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model) {
        
        int lidx = gap.LIdx();
        int ridx = gap.RIdx();
        float ldist = gap.LDist();
        float rdist = gap.RDist();

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);
        y1 = (ldist) * sin(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);

        x2 = (rdist) * cos(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
        y2 = (rdist) * sin(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);

        //std::cout << "actual gap left: (" << x2 << ", " << y2 << "), actual gap right: (" << x1 << ", " << y1 << ")" << std::endl;
        Eigen::Vector2f crossing_pt(0.0, 0.0);
        double start_time = ros::Time::now().toSec();
        double crossing_time = indivGapFindCrossingPoint(gap, crossing_pt, left_model, right_model);
        ROS_INFO_STREAM("indivGapFindCrossingPoint time elapsed:" << ros::Time::now().toSec() - start_time);

        Eigen::Vector2f starting_pos(0.0, 0.0);
        Eigen::Vector2f starting_vel(left_model->get_v_ego()[0], left_model->get_v_ego()[1]);
        
        Eigen::Vector2f ending_vel(0.0, 0.0);
        if (crossing_pt.norm() > 0) {
            ending_vel << starting_vel.norm() * crossing_pt[0] / crossing_pt.norm(), starting_vel.norm() * crossing_pt[1] / crossing_pt.norm();
        } 
        //std::cout << "starting x: " << starting_pos[0] << ", " << starting_pos[1] << ", " << starting_vel[0] << ", " << starting_vel[1] << std::endl;
        //std::cout << "ending x: " << crossing_pt[0] << ", " << crossing_pt[1] << ", ending_vel: " << ending_vel[0] << ", " << ending_vel[1] << std::endl;
        
        start_time = ros::Time::now().toSec();
        Eigen::MatrixXf A_x = MatrixXf::Random(4,4);
        Eigen::VectorXf b_x = VectorXf::Random(4);
        A_x << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             1.0, crossing_time, pow(crossing_time,2), pow(crossing_time,3),
             0.0, 1.0, 2*crossing_time, 3*pow(crossing_time,2);
        //std::cout << "A_x: " << A_x << std::endl;
        b_x << starting_pos[0], starting_vel[0], crossing_pt[0], ending_vel[0];
        //std::cout << "b_x: " << b_x << std::endl;
        Eigen::Vector4f coeffs = A_x.partialPivLu().solve(b_x);
        // std::cout << "x coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        double peak_velocity_x = 3*coeffs[3]*pow(crossing_time/2.0, 2) + 2*coeffs[2]*crossing_time/2.0 + coeffs[1];
        // std::cout << "peak velocity x: " << peak_velocity_x << std::endl;
        Eigen::MatrixXf A_y = MatrixXf::Random(4,4);
        Eigen::VectorXf b_y = VectorXf::Random(4);
        A_y << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             1.0, crossing_time, pow(crossing_time,2), pow(crossing_time,3),
             0.0, 1.0, 2*crossing_time, 3*pow(crossing_time,2);
        b_y << starting_pos[1], starting_vel[1], crossing_pt[1], ending_vel[1];
        //std::cout << "A_y: " << A_y << std::endl;
        //std::cout << "b_y: " << b_y << std::endl;
        
        coeffs = A_y.partialPivLu().solve(b_y);
        //std::cout << "y coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        double peak_velocity_y = 3*coeffs[3]*pow(crossing_time/2.0, 2) + 2*coeffs[2]*crossing_time/2.0 + coeffs[1];
        //std::cout << "peak velocity: " << peak_velocity_x << ", " << peak_velocity_y << std::endl;
        ROS_INFO_STREAM("spline build time elapsed:" << ros::Time::now().toSec() - start_time);

        if (std::max(std::abs(peak_velocity_x), std::abs(peak_velocity_y)) <= cfg_->control.vx_absmax) {
            return crossing_time;
        } else {
            return -1.0;
        }
    }
 
    double GapFeasibilityChecker::indivGapFindCrossingPoint(dynamic_gap::Gap & gap, Eigen::Vector2f& gap_crossing_point, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model) {
        //std::cout << "determining crossing point" << std::endl;
        Matrix<double, 4, 1> left_frozen_state = left_model->get_frozen_modified_polar_state();        
        Matrix<double, 4, 1> right_frozen_state = right_model->get_frozen_modified_polar_state();

        double x1, x2, y1, y2;

        x1 = (gap.LDist()) * cos(-((double) gap.half_scan - gap.LIdx()) / gap.half_scan * M_PI);
        y1 = (gap.LDist()) * sin(-((double) gap.half_scan - gap.LIdx()) / gap.half_scan * M_PI);

        x2 = (gap.RDist()) * cos(-((double) gap.half_scan - gap.RIdx()) / gap.half_scan * M_PI);
        y2 = (gap.RDist()) * sin(-((double) gap.half_scan - gap.RIdx()) / gap.half_scan * M_PI);
       
        Matrix<double, 2, 1> left_bearing_vect(x2 / gap.RDist(), y2 / gap.RDist());
        Matrix<double, 2, 1> right_bearing_vect(x1 / gap.LDist(), y1 / gap.LDist());

        double det = left_bearing_vect[0]*right_bearing_vect[1] - left_bearing_vect[1]*right_bearing_vect[0];      
        double dot = left_bearing_vect[0]*right_bearing_vect[0] + left_bearing_vect[1]*right_bearing_vect[1];

        double swept_check = -std::atan2(det, dot);     
        double L_to_R_angle = swept_check;

        if (L_to_R_angle < 0) {
            L_to_R_angle += 2*M_PI; 
        }

        double beta_left = atan2(y2, x2); // std::atan2(left_frozen_state[1], left_frozen_state[2]);
        double beta_right = atan2(y1, x1); // std::atan2(right_frozen_state[1], right_frozen_state[2]);
        double beta_center = beta_left -= (L_to_R_angle / 2.0);

        Matrix<double, 2, 1> central_bearing_vect(std::cos(beta_center), std::sin(beta_center));
        
        //std::cout << "initial beta left: (" << left_bearing_vect[0] << ", " << left_bearing_vect[1] << "), initial beta right: (" << right_bearing_vect[0] << ", " << right_bearing_vect[1] << "), initial beta center: (" << central_bearing_vect[0] << ", " << central_bearing_vect[1] << ")" << std::endl;
        
        Matrix<double, 4, 1> left_frozen_cartesian_state = left_model->get_frozen_cartesian_state();
        Matrix<double, 4, 1> right_frozen_cartesian_state = right_model->get_frozen_cartesian_state();

        //std::cout << "starting left: " << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << ", " << left_frozen_cartesian_state[2] << ", " << left_frozen_cartesian_state[3] << std::endl; 
        //std::cout << "starting right: " << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1] << ", " << right_frozen_cartesian_state[2] << ", " << right_frozen_cartesian_state[3] << std::endl;

        // double beta_left, beta_right, beta_center;
       
        double left_central_dot, right_central_dot;
        bool first_cross = true;
        bool bearing_crossing_check, range_closing_check;    

        Matrix<double, 4, 1> prev_left_frozen_state = left_frozen_state;        
        Matrix<double, 4, 1> prev_right_frozen_state = right_frozen_state;        
        Matrix<double, 2, 1> prev_central_bearing_vect = central_bearing_vect;

        for (double t = cfg_->traj.integrate_stept; t < cfg_->traj.integrate_maxt; t += cfg_->traj.integrate_stept) {
            left_model->frozen_state_propagate(cfg_->traj.integrate_stept);
            right_model->frozen_state_propagate(cfg_->traj.integrate_stept);

            left_frozen_state = left_model->get_frozen_modified_polar_state();
            right_frozen_state = right_model->get_frozen_modified_polar_state();
            beta_left = left_frozen_state[1]; // std::atan2(left_frozen_state[1], left_frozen_state[2]);
            beta_right = right_frozen_state[1]; // std::atan2(right_frozen_state[1], right_frozen_state[2]);
            // beta_center = (beta_left + beta_right) / 2.0;

            left_bearing_vect << std::cos(beta_left), std::sin(beta_left);
            right_bearing_vect << std::cos(beta_right), std::sin(beta_right);
            det = left_bearing_vect[0]*right_bearing_vect[1] - left_bearing_vect[1]*right_bearing_vect[0];      
            dot = left_bearing_vect[0]*right_bearing_vect[0] + left_bearing_vect[1]*right_bearing_vect[1];
            swept_check = -std::atan2(det, dot); // this value is the angle swept clockwise from L to R (ranging from -pi to pi)
            L_to_R_angle = swept_check;

            if (L_to_R_angle < 0) {
                L_to_R_angle += 2*M_PI; 
            }
            beta_center = beta_left -= (L_to_R_angle / 2.0);

            Matrix<double, 2, 1> central_bearing_vect(std::cos(beta_center), std::sin(beta_center));
        

            left_central_dot = left_bearing_vect.dot(prev_central_bearing_vect);
            right_central_dot = right_bearing_vect.dot(prev_central_bearing_vect);


                        
            //std::cout << "left bearing vect: " << left_bearing_vect[0] << ", " << left_bearing_vect[1] << std::endl;
            //std::cout << "right bearing vect: " << right_bearing_vect[0] << ", " << right_bearing_vect[1] << std::endl;
            //std::cout << "central bearing vect: " << central_bearing_vect[0] << ", " << central_bearing_vect[1] << std::endl;
            //std::cout << "left/center dot: " << left_central_dot << ", right/center dot: " << right_central_dot << std::endl;
            //std::cout << "sweep dt: " << dt << ", swept check: " << swept_check << ". left beta: " << beta_left << ", left range: " << 1.0 / left_frozen_state[0] << ", right beta: " << beta_right << ", right range: " << 1.0 / right_frozen_state[0] << std::endl;
            if (swept_check < 0) {
                bearing_crossing_check = left_central_dot > 0.0 && right_central_dot > 0.0;
                if (bearing_crossing_check) {
                    left_frozen_cartesian_state = left_model->get_frozen_cartesian_state();
                    right_frozen_cartesian_state = right_model->get_frozen_cartesian_state();
                    
                    range_closing_check = std::abs((1.0 / left_frozen_state[0]) - (1.0 / right_frozen_state[0])) < 4*cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
                    if (range_closing_check) {
                        ROS_INFO_STREAM("gap closes at " << t << ", left point at: " << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << ", right point at " << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1]); 
                        if ((1.0 / left_frozen_state[0]) < (1.0 / right_frozen_state[0])) {
                            //std::cout << "setting right equal to cross" << std::endl;
                            //std::cout << "right state: " << right_frozen_state[0] << ", " << right_frozen_state[1] << ", " << right_frozen_state[2] << std::endl;
                            gap_crossing_point << right_frozen_cartesian_state[0], right_frozen_cartesian_state[1];
                            gap.setClosingPoint(right_frozen_cartesian_state[0], right_frozen_cartesian_state[1]);
                        } else {
                            //std::cout << "setting left equal to cross" << std::endl;
                            //std::cout << "left state: " << left_frozen_state[0] << ", " << left_frozen_state[1] << ", " << left_frozen_state[2] << std::endl;
                            gap_crossing_point << left_frozen_cartesian_state[0], left_frozen_cartesian_state[1];
                            gap.setClosingPoint(left_frozen_cartesian_state[0], left_frozen_cartesian_state[1]);
                        }
                        float prev_beta_left = prev_left_frozen_state[1]; // std::atan2(prev_left_frozen_state[1], prev_left_frozen_state[2]);
                        float prev_beta_right = prev_right_frozen_state[1]; // std::atan2(prev_right_frozen_state[1], prev_right_frozen_state[2]);
            
                        int left_idx = (int) std::ceil((prev_beta_left - msg.get()->angle_min) / msg.get()->angle_increment);
                        float left_dist = (1.0 / prev_left_frozen_state[0]);
                        int right_idx = (int) std::floor((prev_beta_right - msg.get()->angle_min) / msg.get()->angle_increment);
                        float right_dist = (1.0 / prev_right_frozen_state[0]);
                        gap.setTerminalPoints(right_idx, right_dist, left_idx, left_dist);
                        gap.gap_closed = true;
                        // gap.gap_crossed = true;
                        return t;
                    } else {
                        if (first_cross) {
                            double mid_x = (left_frozen_cartesian_state[0] + right_frozen_cartesian_state[0]) / 2;
                            double mid_y = (left_frozen_cartesian_state[1] + right_frozen_cartesian_state[1]) / 2;
                            ROS_INFO_STREAM("gap crosses but does not close at " << t << ", left point at: " << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << ", right point at " << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1]); 
                            gap.setCrossingPoint(mid_x, mid_y);
                            first_cross = false;
                            float prev_beta_left = prev_left_frozen_state[1]; // std::atan2(prev_left_frozen_state[1], prev_left_frozen_state[2]);
                            float prev_beta_right = prev_right_frozen_state[1]; // std::atan2(prev_right_frozen_state[1], prev_right_frozen_state[2]);
            
                            int left_idx = (int) std::ceil((prev_beta_left - msg.get()->angle_min) / msg.get()->angle_increment);
                            float left_dist = (1.0 / prev_left_frozen_state[0]);
                            int right_idx = (int) std::floor((prev_beta_right - msg.get()->angle_min) / msg.get()->angle_increment);
                            float right_dist = (1.0 / prev_right_frozen_state[0]);
                            gap.setTerminalPoints(right_idx, right_dist, left_idx, left_dist);
                            gap.gap_crossed = true;
                        }
                    }

                }
            }

            prev_left_frozen_state = left_frozen_state;
            prev_right_frozen_state = right_frozen_state;
            prev_central_bearing_vect = central_bearing_vect;
        }

        if (!gap.gap_crossed && !gap.gap_closed) {
            left_frozen_cartesian_state = left_model->get_frozen_cartesian_state();
            right_frozen_cartesian_state = right_model->get_frozen_cartesian_state();
            left_frozen_state = left_model->get_frozen_modified_polar_state();
            right_frozen_state = right_model->get_frozen_modified_polar_state();
            beta_left = left_frozen_state[1]; // std::atan2(left_frozen_state[1], left_frozen_state[2]);
            beta_right = right_frozen_state[1]; // std::atan2(right_frozen_state[1], right_frozen_state[2]);
            
            ROS_INFO_STREAM("no close, final swept points at: (" << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << "), (" << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1] << ")");
            int left_idx = int((beta_left - msg.get()->angle_min) / msg.get()->angle_increment);
            float left_dist = (1.0 / left_frozen_state[0]);
            int right_idx = int((beta_right - msg.get()->angle_min) / msg.get()->angle_increment);
            float right_dist = (1.0 / right_frozen_state[0]);

            gap.setTerminalPoints(right_idx, right_dist, left_idx, left_dist);
            gap.gap_crossed = false;
        }

        return cfg_->traj.integrate_maxt;
    }
}
