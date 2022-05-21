#include <dynamic_gap/gap_manip.h>

namespace dynamic_gap {
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
    }
    
    double GapManipulator::indivGapFindCrossingPoint(dynamic_gap::Gap & gap, Eigen::Vector2f& gap_crossing_point, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model) {
        std::cout << "determining crossing point" << std::endl;
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
        
        std::cout << "initial beta left: (" << left_bearing_vect[0] << ", " << left_bearing_vect[1] << "), initial beta right: (" << right_bearing_vect[0] << ", " << right_bearing_vect[1] << "), initial beta center: (" << central_bearing_vect[0] << ", " << central_bearing_vect[1] << ")" << std::endl;
        
        Matrix<double, 4, 1> left_frozen_cartesian_state = left_model->get_frozen_cartesian_state();
        Matrix<double, 4, 1> right_frozen_cartesian_state = right_model->get_frozen_cartesian_state();

        std::cout << "starting left: " << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << ", " << left_frozen_cartesian_state[2] << ", " << left_frozen_cartesian_state[3] << std::endl; 
        std::cout << "starting right: " << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1] << ", " << right_frozen_cartesian_state[2] << ", " << right_frozen_cartesian_state[3] << std::endl;

        // double beta_left, beta_right, beta_center;
       
        double left_central_dot, right_central_dot;
        bool first_cross = true;
        bool bearing_crossing_check, range_closing_check;    

        Matrix<double, 4, 1> prev_left_frozen_state = left_frozen_state;        
        Matrix<double, 4, 1> prev_right_frozen_state = right_frozen_state;        
        Matrix<double, 2, 1> prev_central_bearing_vect = central_bearing_vect;

        for (double dt = cfg_->traj.integrate_stept; dt < cfg_->traj.integrate_maxt; dt += cfg_->traj.integrate_stept) {
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
                        std::cout << "gap closes at " << dt << ", left point at: " << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << ", right point at " << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1] << std::endl; 
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
                        return dt;
                    } else {
                        if (first_cross) {
                            double mid_x = (left_frozen_cartesian_state[0] + right_frozen_cartesian_state[0]) / 2;
                            double mid_y = (left_frozen_cartesian_state[1] + right_frozen_cartesian_state[1]) / 2;
                            std::cout << "gap crosses but does not close at " << dt << ", left point at: " << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << ", right point at " << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1] << std::endl; 
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
            
            std::cout << "no close, final swept points at: (" << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << "), (" << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1] << ")" << std::endl;
            int left_idx = int((beta_left - msg.get()->angle_min) / msg.get()->angle_increment);
            float left_dist = (1.0 / left_frozen_state[0]);
            int right_idx = int((beta_right - msg.get()->angle_min) / msg.get()->angle_increment);
            float right_dist = (1.0 / right_frozen_state[0]);

            gap.setTerminalPoints(right_idx, right_dist, left_idx, left_dist);
            gap.gap_crossed = false;
        }

        return cfg_->traj.integrate_maxt;
    }


    /*
    // CHANGING SO NOT PASSING BY REFERENCE HERE JUST TO SEE WHAT CHANGES DO, FIX LATER
    void GapManipulator::setGapGoalsByCategory(dynamic_gap::Gap gap, geometry_msgs::PoseStamped localgoal) {
        
    }
    */

    bool GapManipulator::indivGapFeasibilityCheck(dynamic_gap::Gap& gap) {
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
        std::cout << "FLIPPING MODELS TO GET L/R FROM ROBOT POV" << std::endl;
        dynamic_gap::cart_model* left_model = gap.right_model;
        dynamic_gap::cart_model* right_model = gap.left_model;

        auto left_ori = gap.convex.convex_lidx * msg.get()->angle_increment + msg.get()->angle_min;
        auto right_ori = gap.convex.convex_ridx * msg.get()->angle_increment + msg.get()->angle_min;

        double gap_angle = right_ori - left_ori;
        if (gap_angle < 0) {
            gap_angle += (2*M_PI);
        }

        // FEASIBILITY CHECK
        // std::cout << "left ori: " << left_ori << ", right_ori: " << right_ori << std::endl;
        std::cout << "left idx: " << gap.convex.convex_lidx << ", right idx: " << gap.convex.convex_ridx << std::endl;
        feasible = feasibilityCheck(gap, left_model, right_model, gap_angle);

        if (feasible) {
            std::cout << "gap is feasible" << std::endl;
        } else {
            std::cout << "gap is not feasible" << std::endl;
        }
        return feasible;
    }

    double GapManipulator::gapSplinecheck(dynamic_gap::Gap & gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model) {
        
        int lidx = gap.LIdx();
        int ridx = gap.RIdx();
        float ldist = gap.LDist();
        float rdist = gap.RDist();

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);
        y1 = (ldist) * sin(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);

        x2 = (rdist) * cos(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
        y2 = (rdist) * sin(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);

        std::cout << "actual gap left: (" << x2 << ", " << y2 << "), actual gap right: (" << x1 << ", " << y1 << ")" << std::endl;
        Eigen::Vector2f crossing_pt(0.0, 0.0);
        double crossing_time = indivGapFindCrossingPoint(gap, crossing_pt, left_model, right_model);

        Eigen::Vector2f starting_pos(0.0, 0.0);
        Eigen::Vector2f starting_vel(left_model->get_v_ego()[0], left_model->get_v_ego()[1]);
        
        Eigen::Vector2f ending_vel(0.0, 0.0);
        if (crossing_pt.norm() > 0) {
            ending_vel << starting_vel.norm() * crossing_pt[0] / crossing_pt.norm(), starting_vel.norm() * crossing_pt[1] / crossing_pt.norm();
        } 
        std::cout << "starting x: " << starting_pos[0] << ", " << starting_pos[1] << ", " << starting_vel[0] << ", " << starting_vel[1] << std::endl;
        std::cout << "ending x: " << crossing_pt[0] << ", " << crossing_pt[1] << ", ending_vel: " << ending_vel[0] << ", " << ending_vel[1] << std::endl;
        
        Eigen::MatrixXf A_x = MatrixXf::Random(4,4);
        Eigen::VectorXf b_x = VectorXf::Random(4);
        A_x << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             1.0, crossing_time, pow(crossing_time,2), pow(crossing_time,3),
             0.0, 1.0, 2*crossing_time, 3*pow(crossing_time,2);
        //std::cout << "A_x: " << A_x << std::endl;
        b_x << starting_pos[0], starting_vel[0], crossing_pt[0], ending_vel[0];
        //std::cout << "b_x: " << b_x << std::endl;
        Eigen::Vector4f coeffs = A_x.bdcSvd(ComputeThinU | ComputeThinV).solve(b_x);
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
        
        coeffs = A_y.bdcSvd(ComputeThinU | ComputeThinV).solve(b_y);
        std::cout << "y coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        double peak_velocity_y = 3*coeffs[3]*pow(crossing_time/2.0, 2) + 2*coeffs[2]*crossing_time/2.0 + coeffs[1];
        std::cout << "peak velocity: " << peak_velocity_x << ", " << peak_velocity_y << std::endl;
        
        if (std::max(std::abs(peak_velocity_x), std::abs(peak_velocity_y)) <= cfg_->control.vx_absmax) {
            return crossing_time;
        } else {
            return -1.0;
        }
    }


    bool GapManipulator::feasibilityCheck(dynamic_gap::Gap& gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model, double gap_angle) {
        bool feasible = false;
        //Matrix<double, 5, 1> left_model_state = left_model->get_state();        
        //Matrix<double, 5, 1> right_model_state = right_model->get_state();
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
        double crossing_time = gapSplinecheck(gap, left_model, right_model);

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

        /*
        std::cout << "left betadot check: " << left_betadot_check << ", right betadot check: " << right_betadot_check << std::endl;
        if ((left_betadot_check >= 0  && right_betadot_check > 0) || (left_betadot_check <= 0  && right_betadot_check < 0 )) {
            // CATEGORY 1: TRANSLATING     
            std::cout << "translating gap" << std::endl;
            gap.setCategory("translating");
            if (left_betadot_check - right_betadot_check > 0) {
                feasible = true;
                // double crossing_time = gapSplinecheck(gap, left_model, right_model);
                gap.gap_lifespan = cfg_->traj.integrate_maxt;
            } else {
                // double crossing_time = gapSplinecheck(gap, left_model, right_model);
                if (crossing_time >= 0) {
                    feasible = true;
                    gap.gap_lifespan = crossing_time;
                }
            }
        } else if (left_betadot_check <= 0 && right_betadot_check >= 0)  {
            // CATEGORY 2: STATIC/CLOSING
            std::cout << "static/closing gap" << std::endl;
            gap.setCategory("closing");
            // double crossing_time = gapSplinecheck(gap, left_model, right_model);
            if (crossing_time >= 0) {
                feasible = true;
                gap.gap_lifespan = crossing_time;
            }
        } else {
            // CATEGORY 3: EXPANDING
            std::cout << "expanding gap" << std::endl;
            gap.setCategory("expanding");
            feasible = true;
            // double crossing_time = gapSplinecheck(gap, left_model, right_model);
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
        }
        */
        return feasible;
    }
    
    std::vector<dynamic_gap::Gap> GapManipulator::gapSetFeasibilityCheck(std::vector<dynamic_gap::Gap>& manip_set) {
        bool gap_i_feasible;
        int num_gaps = manip_set.size();
        std::vector<dynamic_gap::Gap> feasible_gap_set;
        for (size_t i = 0; i < num_gaps; i++) {
            // obtain crossing point
            std::cout << "feasibility check for gap " << i << std::endl; //  ", left index: " << manip_set.at(i).left_model->get_index() << ", right index: " << manip_set.at(i).right_model->get_index() 
            gap_i_feasible = indivGapFeasibilityCheck(manip_set.at(i));
            
            if (gap_i_feasible) {
                manip_set.at(i).addTerminalRightInformation();
                feasible_gap_set.push_back(manip_set.at(i));
            }
        }
        return feasible_gap_set;
    }
    /*
    // NEED TO ADD FROZEN HERE
    void GapManipulator::setGapGoal(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) {
        //std::cout << "in setGapGoal" << std::endl;
        auto half_num_scan = gap.half_scan;
        float x1, x2, y1, y2;
        int mid_idx;
        x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
       
        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        Eigen::Vector2f pg = (pl + pr) / 2.0;

        // LEFT AND RIGHT ORI USE THE LIDAR INDEX L/R WHICH IS FLIPPED FROM MODEL L/R
        // SHOULD WE JUST USE BETA VALUES HERE?

        std::cout << "starting points. x1, y1: (" << x1 << ", " << y1 << "), x2, y2: (" << x2 << ", " << y2 << ")" << std::endl; 
        std::cout << "original left idx: " << gap.convex.convex_lidx << ", original right idx: " << gap.convex.convex_ridx << std::endl;
        //std::cout << "left ori: " << left_ori << ", right_ori: " << right_ori << std::endl;

        dynamic_gap::cart_model* left_model = gap.right_model;
        dynamic_gap::cart_model* right_model = gap.left_model;

        // Matrix<double, 4, 1> left_model_state = left_model->get_state();
        // Matrix<double, 4, 1> right_model_state = right_model->get_state();
        
        
        left_model->freeze_robot_vel();
        right_model->freeze_robot_vel();
        
        //setGapGoalTimeBased(left_model, right_model, gap, localgoal);
        setGapGoalCrossingBased(left_model, right_model, gap, localgoal);
    }
    */

    void GapManipulator::setGapGoalCrossingBased(dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model, dynamic_gap::Gap& gap,  geometry_msgs::PoseStamped localgoal) {
        auto half_num_scan = gap.half_scan;
        float x1, x2, y1, y2;
        int mid_idx;
        x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);

        auto lf = (pr - pl) / (pr - pl).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pl;
        auto thetalf = car2pol(lf)(1);
        auto lr = (pl - pr) / (pl - pr).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pr;
        auto thetalr = car2pol(lr)(1);
        // get crossing point
        Eigen::Vector2f crossing_pt(0.0, 0.0);
        double crossing_time = indivGapFindCrossingPoint(gap, crossing_pt, left_model, right_model);
        
        /*
        // if cross is 0,0, just set gap goal to middle point
        if (gap.convex.convex_ridx - gap.convex.convex_lidx < 0) {
            mid_idx = int(((gap.convex.convex_lidx + gap.convex.convex_ridx) % int(2*half_num_scan)) / 2.0);
        } else {
            mid_idx = int((gap.convex.convex_lidx + gap.convex.convex_ridx) / 2.0);
        }
        std::cout << "left idx: " << gap.convex.convex_lidx << ", right idx: " << gap.convex.convex_ridx << "mid idx: " << mid_idx << std::endl;
        double theta_mid = mid_idx * msg.get()->angle_increment + msg.get()->angle_min;
        double r_mid = (gap.convex.convex_rdist - gap.convex.convex_ldist) * (theta_mid - thetalf) / (thetalr - thetalf) + gap.convex.convex_ldist;
        //std::cout << "theta_mid: " << theta_mid << ", r_mid: " << r_mid << std::endl;
        gap.goal.x = r_mid*std::cos(theta_mid);
        gap.goal.y = r_mid*std::sin(theta_mid);
        gap.goal.set = true;
        */
        
        
        if (crossing_pt[0] == 0 && crossing_pt[1] == 0) {
            int mid_idx;
            if (gap.convex.convex_ridx - gap.convex.convex_lidx < 0) {
                mid_idx = int(((gap.convex.convex_lidx + gap.convex.convex_ridx) % int(2*half_num_scan)) / 2.0);
            } else {
                mid_idx = int((gap.convex.convex_lidx + gap.convex.convex_ridx) / 2.0);
            }
            std::cout << "left idx: " << gap.convex.convex_lidx << ", right idx: " << gap.convex.convex_ridx << "mid idx: " << mid_idx << std::endl;
            double theta_mid = mid_idx * msg.get()->angle_increment + msg.get()->angle_min;
            double r_mid = (gap.convex.convex_rdist - gap.convex.convex_ldist) * (theta_mid - thetalf) / (thetalr - thetalf) + gap.convex.convex_ldist;
            //std::cout << "theta_mid: " << theta_mid << ", r_mid: " << r_mid << std::endl;
            gap.goal.x = r_mid*std::cos(theta_mid);
            gap.goal.y = r_mid*std::sin(theta_mid);
            std::cout << "no cross, goal index: " << mid_idx << std::endl; // set to: " << gap.goal.x << ", " << gap.goal.y << std::endl;
        } else {
            //std::cout << "crossing pt: " << crossing_pt[0] << ", " << crossing_pt[1] << std::endl;
            // else, set to radially extended cross point
            double theta_crossing = std::atan2(crossing_pt[1], crossing_pt[0]);
            int goal_idx = int((theta_crossing - msg.get()->angle_min) / msg.get()->angle_increment);
            double r_crossing = crossing_pt.norm();
            double r_arc_at_theta_crossing = (gap.convex.convex_rdist - gap.convex.convex_ldist) * (theta_crossing - thetalf) / (thetalr - thetalf) + gap.convex.convex_ldist;
            gap.goal.x = 1.1*std::max(r_crossing, r_arc_at_theta_crossing)*std::cos(theta_crossing);
            gap.goal.y = 1.1*std::max(r_crossing, r_arc_at_theta_crossing)*std::sin(theta_crossing);
            std::cout << "yes cross, goal index: " << goal_idx << std::endl; //set to: " << gap.goal.x << ", " << gap.goal.y << std::endl;
            // pivot it in by a robot radius?
        }
        gap.goal.set = true;
        
    }
    
    void GapManipulator::setTerminalGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) {
        if (gap.getCategory() == "expanding") { 
            std::cout << "setting goal for expanding gap" << std::endl;
            setGapWaypoint(gap, localgoal, false);
        } else if (gap.getCategory() == "closing") {
            if (gap.gap_crossed) {
                std::cout << "setting goal for crossed closing gap" << std::endl;
                // get left and right models
                gap.left_model->freeze_robot_vel();
                gap.right_model->freeze_robot_vel();
                gap.left_model->copy_model();
                gap.right_model->copy_model();
                Eigen::Vector4d left_model = gap.left_model->get_frozen_modified_polar_state();
                Eigen::Vector4d right_model = gap.right_model->get_frozen_modified_polar_state();

                std::cout << "comparing left: (" << left_model[0] << ", " << left_model[1] << ", " << left_model[2] << ", " << left_model[3] << ") ";
                std::cout << "to right: (" << right_model[0] << ", " << right_model[1] << ", " << right_model[2] << ", " << right_model[3] << ")" << std::endl;
                int lidx = gap.convex.terminal_lidx;
                int ridx = gap.convex.terminal_ridx;
                float ldist = gap.convex.terminal_ldist;
                float rdist = gap.convex.terminal_rdist;

                float x1, x2, y1, y2;
                x1 = (ldist) * cos(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);
                y1 = (ldist) * sin(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);

                x2 = (rdist) * cos(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
                y2 = (rdist) * sin(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
                Eigen::Vector2f pl(x1, y1);
                Eigen::Vector2f pr(x2, y2);
                
                auto lf = (pr - pl) / (pr - pl).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pl;
                auto lr = (pl - pr) / (pl - pr).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pr;
                Eigen::Matrix2f r_negpi2;
                r_negpi2 << 0,1,
                        -1,0;
                Eigen::Vector2f offset(0.0, 0.0);
                // which one has the smallest betadot
                // pick that manipulated side as anchor
                Eigen::Vector2f anchor(0.0, 0.0);
                if (std::abs(left_model[3]) > std::abs(right_model[3])) { // left betadot larger
                    std::cout << "left betadot is larger, setting anchor to right" << std::endl;
                    anchor << x2, y2;
                    Eigen::Vector2f norm_lr = lr / lr.norm();
                    offset = r_negpi2 * norm_lr * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
                } else { // right betadot larger
                    std::cout << "right betadot is larger, setting anchor to left" << std::endl;
                    anchor << x1, y1;
                    Eigen::Vector2f norm_lf = lf / lf.norm();
                    offset = -r_negpi2 * norm_lf * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
                }
                auto goal_pt = anchor + offset;

                // do what is normally done
                gap.terminal_goal.x = goal_pt[0];
                gap.terminal_goal.y = goal_pt[1];
                gap.terminal_goal.set = true;
            } else if (gap.gap_closed) {
                std::cout << "setting goal for closed closing gap" << std::endl;
                Eigen::Vector2f closing_pt = gap.getClosingPoint();
                float mult_factor = (closing_pt.norm() + cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) / closing_pt.norm();
                gap.terminal_goal.x = closing_pt[0] * mult_factor;
                gap.terminal_goal.y = closing_pt[1] * mult_factor;
                gap.terminal_goal.set = true;
            } else {
                std::cout << "setting goal for existent closing gap" << std::endl;
                setGapWaypoint(gap, localgoal, false);
            }
        }  
    }


    void GapManipulator::setGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan){
        std::cout << "~running setGapWaypoint" << std::endl;
        auto half_num_scan = gap.half_scan;

        int lidx = initial ? gap.convex.convex_lidx : gap.convex.terminal_lidx;
        int ridx = initial ? gap.convex.convex_ridx : gap.convex.terminal_ridx;
        float ldist = initial ? gap.convex.convex_ldist : gap.convex.terminal_ldist;
        float rdist = initial ? gap.convex.convex_rdist : gap.convex.terminal_rdist;

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) half_num_scan - lidx) / half_num_scan * M_PI);
        y1 = (ldist) * sin(-((float) half_num_scan - lidx) / half_num_scan * M_PI);

        x2 = (rdist) * cos(-((float) half_num_scan - ridx) / half_num_scan * M_PI);
        y2 = (rdist) * sin(-((float) half_num_scan - ridx) / half_num_scan * M_PI);

        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        
        std::cout << "gap index/dist, left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")" << std::endl;
        std::cout << "gap x/y, left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")" << std::endl;
        
        // if agc. then the shorter side need to be further in
        // lf: front (first one, left from laser scan)
        // lr: rear (second one, right from laser scan)
        // what do these do?
        auto lf = (pr - pl) / (pr - pl).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pl;
        auto thetalf = car2pol(lf)(1);
        auto lr = (pl - pr) / (pl - pr).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pr;
        auto thetalr = car2pol(lr)(1);
        
        auto left_ori = lidx * msg.get()->angle_increment + msg.get()->angle_min;
        auto right_ori = ridx * msg.get()->angle_increment + msg.get()->angle_min;

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        double gap_angle = right_ori - left_ori;;
        if (gap_angle < 0) {
            gap_angle += 2*M_PI;
        }

        bool gap_size_check = gap_angle < M_PI;
        float dist = 0;
        bool small_gap = false;
        if (gap_size_check && !cfg_->planning.planning_inflated) {
            // if smaller than M_PI/3
            dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
            small_gap = dist < 4 * cfg_->rbt.r_inscr;
        }

        if (thetalr < thetalf || small_gap) {
            std::cout << "Option 1: small gap" << std::endl;
            if (initial) {
                gap.goal.set = true;
                gap.goal.x = (x1 + x2) / 2;
                gap.goal.y = (y1 + y2) / 2;
                std::cout << "goal: " << gap.goal.x << ", " << gap.goal.y << std::endl;
            } else {
                gap.terminal_goal.set = true;
                gap.terminal_goal.x = (x1 + x2) / 2;
                gap.terminal_goal.y = (y1 + y2) / 2;
                std::cout << "goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y << std::endl;
            }

            return;
        }

        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_laser_scan;
        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        double local_goal_idx = std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);
        std::cout << "local goal idx: " << local_goal_idx << ", local goal x/y: (" << localgoal.pose.position.x << ", " << localgoal.pose.position.y << ")" << std::endl;
        if (checkGoalVisibility(localgoal, stored_scan_msgs) && checkGoalWithinGapAngleRange(gap, local_goal_idx, lidx, ridx)) {
            std::cout << "Option 2: local goal" << std::endl;
            if (initial) {
                gap.goal.x = localgoal.pose.position.x;
                gap.goal.y = localgoal.pose.position.y;
                gap.goal.set = true;
                gap.goal.goalwithin = true;
            } else {
                gap.terminal_goal.x = localgoal.pose.position.x;
                gap.terminal_goal.y = localgoal.pose.position.y;
                gap.terminal_goal.set = true;
                gap.terminal_goal.goalwithin = true;
            }
            return;
        }

        std::cout << "Option 3: biasing" << std::endl;

        float confined_theta = std::min(thetalr, std::max(thetalf, goal_orientation));
        double confined_idx = std::floor(confined_theta*half_num_scan/M_PI + half_num_scan);
        std::cout << "confined idx: " << confined_idx << std::endl;
        float confined_r = (rdist - ldist) * (confined_theta - thetalf) / (thetalr - thetalf) + ldist;
        float xg = confined_r * cos(confined_theta);
        float yg = confined_r * sin(confined_theta);
        Eigen::Vector2f anchor(xg, yg);
        Eigen::Matrix2f r_negpi2;
        r_negpi2 << 0,1,
                   -1,0;
        auto offset = r_negpi2 * (pr - pl);

        auto goal_pt = offset * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + anchor;
        std::cout << "anchor: (" << anchor[0] << ", " << anchor[1] << "), offset with r_ins " << cfg_->rbt.r_inscr << " and inf ratio " << cfg_->traj.inf_ratio << ", :(" << offset[0] << ", " << offset[1] << "), goal_pt: (" << goal_pt[0] << ", " << goal_pt[1] << ")" << std::endl;
        if (initial) {
            gap.goal.x = goal_pt(0);
            gap.goal.y = goal_pt(1);
            gap.goal.set = true;
        } else {
            gap.terminal_goal.x = goal_pt(0);
            gap.terminal_goal.y = goal_pt(1);
            gap.terminal_goal.set = true;
        }

    }

    bool GapManipulator::checkGoalWithinGapAngleRange(dynamic_gap::Gap& gap, double gap_goal_idx, float lidx, float ridx) { 
        return (lidx <= gap_goal_idx && gap_goal_idx <= ridx);
    }

    bool GapManipulator::checkGoalVisibility(geometry_msgs::PoseStamped localgoal, sensor_msgs::LaserScan const scan) {
        boost::mutex::scoped_lock lock(egolock);
        // with robot as 0,0 (localgoal in robot frame as well)
        double dist2goal = sqrt(pow(localgoal.pose.position.x, 2) + pow(localgoal.pose.position.y, 2));

        // auto scan = *msg.get();
        auto min_val = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr) {
            return true;
        }

        // If within closest configuration space
        if (dist2goal < min_val - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr) {
            return true;
        }

        // Should be sufficiently far, otherwise we are in trouble
        double goal_angle = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_index = (int) round((goal_angle - scan.angle_min) / scan.angle_increment);

        double half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = goal_index - index;
        int upper_bound = goal_index + index;
        float min_val_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);

        return dist2goal < min_val_round_goal;
    }

    // In place modification
    void GapManipulator::reduceGap(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        if (!msg) return; 
        // msg is from egocircle

        int lidx = initial ? gap.LIdx() : gap.terminal_lidx;
        int ridx = initial ? gap.RIdx() : gap.terminal_ridx;
        float ldist = initial ? gap.LDist() : gap.terminal_ldist;
        float rdist = initial ? gap.RDist() : gap.terminal_rdist;
        double gap_idx_size = (ridx - lidx);

        double gap_theta_size = gap_idx_size * (msg.get()->angle_increment);
        // std::cout << "gap idx size: " << gap_idx_size << std::endl;

        // threshold = pi right now
        if (gap_theta_size < cfg_->gap_manip.reduction_threshold){
            return;
        }

        std::cout << "~running reduceGap, gap_theta_size: " << gap_theta_size << "~" << std::endl;

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);
        y1 = (ldist) * sin(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);

        x2 = (rdist) * cos(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
        y2 = (rdist) * sin(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);

        std::cout << "pre-reduce gap in polar. left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")" << std::endl;
        std::cout << "pre-reduce gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")" << std::endl;
        //std::cout << "indices: " << lidx << ", " << ridx << std::endl;


        // the desired size for the reduced gap?
        // target is pi
        int target_idx_size = cfg_->gap_manip.reduction_target / msg.get()->angle_increment;
        int l_biased_r = lidx + target_idx_size;
        int r_biased_l = ridx - target_idx_size;
       
        double goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);

        int acceptable_dist = target_idx_size / 2; // distance in scan indices

        //std::cout << "goal orientation: " << goal_orientation << ", goal idx: " << goal_idx << ", acceptable distance: " << acceptable_dist << std::endl;
        int new_l_idx, new_r_idx;
        //std::cout << "right greater than left" << std::endl;
        if (goal_idx + acceptable_dist > ridx){ // r-biased Gap, checking if goal is on the right side of the gap
            //std::cout << "right biased gap" << std::endl;
            new_r_idx = ridx;
            new_l_idx = r_biased_l;
        } else if (goal_idx - acceptable_dist < lidx) { // l-biased gap, checking if goal is on left side of gap?
            //std::cout << "left biased gap" << std::endl;                
            new_r_idx = l_biased_r;
            new_l_idx = lidx;
        } else { // Lingering in center
            //std::cout << "central gap" << std::endl;
            new_l_idx = goal_idx - acceptable_dist;
            new_r_idx = goal_idx + acceptable_dist;
        }

        // removed some float casting here
        float new_ldist = float(new_l_idx - lidx) / float(ridx - lidx) * (rdist - ldist) + ldist;
        float new_rdist = float(new_r_idx - lidx) / float(ridx - lidx) * (rdist - ldist) + ldist;
        
        if (initial) {
            gap.convex.convex_lidx = new_l_idx;
            gap.convex.convex_ridx = new_r_idx;
            gap.convex.convex_ldist = new_ldist + cfg_->gap_viz.viz_jitter;
            gap.convex.convex_rdist = new_rdist + cfg_->gap_viz.viz_jitter;
            gap.mode.reduced = true;

            x1 = (gap.convex.convex_ldist) * cos(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.convex_ldist) * sin(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);

            x2 = (gap.convex.convex_rdist) * cos(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI);
            y2 = (gap.convex.convex_rdist) * sin(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI);
            std::cout << "post-reduce gap in polar. left: (" << gap.convex.convex_lidx << ", " << gap.convex.convex_ldist << "), right: (" << gap.convex.convex_ridx << ", " << gap.convex.convex_rdist << ")" << std::endl;

        } else {
            gap.convex.terminal_lidx = new_l_idx;
            gap.convex.terminal_ridx = new_r_idx;
            gap.convex.terminal_ldist = new_ldist + cfg_->gap_viz.viz_jitter;
            gap.convex.terminal_rdist = new_rdist + cfg_->gap_viz.viz_jitter;
            gap.mode.terminal_reduced = true;

            x1 = (gap.convex.terminal_ldist) * cos(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.terminal_ldist) * sin(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);

            x2 = (gap.convex.terminal_rdist) * cos(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI);
            y2 = (gap.convex.terminal_rdist) * sin(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI);
            std::cout << "post-reduce gap in polar. left: (" << gap.convex.terminal_lidx << ", " << gap.convex.terminal_ldist << "), right: (" << gap.convex.terminal_ridx << ", " << gap.convex.terminal_rdist << ")" << std::endl;
        }
        std::cout << "post-reduce in cart. left: (" << x1 << ", " << y1 << "), right: (" << x2 << ", " << y2 << ")" << std::endl;
        return;
    }

    void GapManipulator::convertAxialGap(dynamic_gap::Gap& gap, Matrix<double, 1, 2> v_ego, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        // Return if not axial gap or disabled
        if (!gap.isSwept(initial) || !cfg_->gap_manip.axial_convert) {
            return;
        }
        std::cout << "~running convertAxialGap~" << std::endl;

        int lidx = initial ? gap.LIdx() : gap.terminal_lidx;
        int ridx = initial ? gap.RIdx() : gap.terminal_ridx;
        float ldist = initial ? gap.LDist() : gap.terminal_ldist;
        float rdist = initial ? gap.RDist() : gap.terminal_rdist;

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);
        y1 = (ldist) * sin(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);

        x2 = (rdist) * cos(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
        y2 = (rdist) * sin(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);

        std::cout << "pre-AGC gap in polar. left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")" << std::endl;
        std::cout << "pre-AGC gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")" << std::endl;
        //std::cout << "indices: " << lidx << ", " << ridx << std::endl;
        
        bool left = initial ? gap.isLeftType() : gap.isTerminalLeftType();
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the visibility line
        // we are pivoting around the closer point?
        float rot_val = (float) std::atan2(cfg_->gap_manip.epsilon2 * cfg_->gap_manip.rot_ratio, cfg_->gap_manip.epsilon1);
        float theta = left ? (rot_val + 1e-3): -(rot_val + 1e-3);
        std::cout << "theta to pivot: " << theta << std::endl;
        int near_idx, far_idx;
        float near_dist, far_dist;
        dynamic_gap::cart_model * near_model;
        
        if (left) {
            near_idx = lidx;
            far_idx = ridx;
            near_dist = ldist;
            far_dist = rdist;
            gap.pivoted_left = false;
        } else {
            near_idx = ridx;
            far_idx = lidx;
            near_dist = rdist;
            far_dist = ldist;
            gap.pivoted_left = true;
        }

        std::cout << "near point in polar: " << near_idx << ", " << near_dist << ", far point in polar: " << far_idx << ", " << far_dist << std::endl;
        
        Eigen::Matrix3f rot_mat;
        // rot_mat: SE(3) matrix that represents desired rotation amount
        rot_mat << cos(theta), -sin(theta), 0,
                   sin(theta), cos(theta), 0,
                   0, 0, 1;

        auto half_num_scan = gap.half_scan;
        
        // obtaining near and far theta values from indices
        double near_theta = M_PI * (near_idx - half_num_scan) / half_num_scan;
        double far_theta = M_PI * (far_idx - half_num_scan) / half_num_scan;
        Eigen::Matrix3f near_rbt;
        // near_rbt, far_robt: SE(3) matrices that represent translation from rbt origin to far and near points
        near_rbt << 1, 0, near_dist * cos(near_theta),
                    0, 1, near_dist * sin(near_theta),
                    0, 0, 1;
        Eigen::Matrix3f far_rbt;
        far_rbt  << 1, 0, far_dist * cos(far_theta),
                    0, 1, far_dist * sin(far_theta),
                    0, 0, 1;

        // rot_rbt: my guess, transformation matrix FROM rbt origin TO desired pivot point
        Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));
        // std::cout << "rot_rbt: " << rot_rbt;

        // radius and index representing desired pivot point
        float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        int idx = int(half_num_scan * std::atan2(rot_rbt(1, 2), rot_rbt(0, 2)) / M_PI) + half_num_scan;

        std::cout << "r: " << r << ", idx: " << idx << std::endl;

        // Rotation Completed
        // Get minimum dist range val between initial gap point and pivoted gap point
        
        // offset: original right index or the pivoted left
        int init_search_idx = left ? ridx : idx;
        // upperbound: pivoted right index or original left  
        int final_search_idx = left ? idx : lidx;
        std:: cout << "init_search_idx: " << init_search_idx << ", final_search_idx: " << final_search_idx << std::endl;
        int size = final_search_idx - init_search_idx;


        if (size < 3) {
            // Arbitrary value
            std::cout << "discarding" << std::endl;
            if (initial) {
                gap.goal.discard = true;
            } else {
                gap.terminal_goal.discard = true;
            }
            return;
        }

        // For wraparound (check to see if this happens)
        init_search_idx = std::max(init_search_idx, 0);
        final_search_idx = std::min(final_search_idx, num_of_scan - 1);

        std:: cout << "wrapped init_search_idx: " << init_search_idx << ", wrapped final_search_idx: " << final_search_idx << std::endl;
        size = final_search_idx - init_search_idx;
        std::vector<float> dist(final_search_idx - init_search_idx);

        if (size == 0) {
            // This shouldn't happen
            return;
        }

        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_laser_scan;
        if (stored_scan_msgs.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect gap manip");
        }

        // using the law of cosines to find the index between init/final indices
        // that's shortest distance between near point and laser scan
        try{
            for (int i = 0; i < dist.size(); i++) {
                dist.at(i) = sqrt(pow(near_dist, 2) + pow(stored_scan_msgs.ranges.at(i + init_search_idx), 2) -
                    2 * near_dist * stored_scan_msgs.ranges.at(i + init_search_idx) * cos((i + init_search_idx - near_idx) * stored_scan_msgs.angle_increment));
            }
        } catch(...) {
            ROS_FATAL_STREAM("convertAxialGap outofBound");
        }

        //auto farside_iter = std::min_element(dist.begin(), dist.end());
        float min_dist = *std::min_element(dist.begin(), dist.end());

        std::cout << "min dist: " << min_dist << std::endl;         

        // pivoting around near point, pointing towards far point or something?
        Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;

        float translation_norm = sqrt(pow(far_near(0, 2), 2) + pow(far_near(1, 2), 2));
        // float coefs = far_near.block<2, 1>(0, 2).norm();
        // clearstd::cout << "coefs: " << coefs << ", translation norm: " << translation_norm << std::endl;
        // normalizing the pivot direction, multiplying by min dist        
        far_near(0, 2) *= min_dist / translation_norm;     
        far_near(1, 2) *= min_dist / translation_norm;
        Eigen::Matrix3f short_pt = near_rbt * (rot_mat * far_near);

        r = float(sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2)));
        float pivoted_theta = std::atan2(short_pt(1, 2), short_pt(0, 2));
        idx = int (half_num_scan * pivoted_theta / M_PI) + half_num_scan;


        // Recalculate end point location based on length
        if (initial) {
            gap.convex.convex_lidx = left ? near_idx : idx;
            gap.convex.convex_ldist = left ? near_dist : r;
            gap.convex.convex_ridx = left ? idx : near_idx;
            gap.convex.convex_rdist = left ? r : near_dist;

            x1 = (gap.convex.convex_ldist) * cos(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.convex_ldist) * sin(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);
            x2 = (gap.convex.convex_rdist) * cos(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI); 
            y2 = (gap.convex.convex_rdist) * sin(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI); 
            std::cout << "post-AGC gap in polar. left: (" << gap.convex.convex_lidx << ", " << gap.convex.convex_ldist << "), right: (" << gap.convex.convex_ridx << ", " << gap.convex.convex_rdist << ")" << std::endl;
            gap.mode.agc = true;
        } else {
            gap.convex.terminal_lidx = left ? near_idx : idx;
            gap.convex.terminal_ldist = left ? near_dist : r;
            gap.convex.terminal_ridx = left ? idx : near_idx;
            gap.convex.terminal_rdist = left ? r : near_dist;
            
            x1 = (gap.convex.terminal_ldist) * cos(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.terminal_ldist) * sin(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);
            x2 = (gap.convex.terminal_rdist) * cos(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI); 
            y2 = (gap.convex.terminal_rdist) * sin(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI); 
            std::cout << "post-AGC gap in polar. left: (" << gap.convex.terminal_lidx << ", " << gap.convex.terminal_ldist << "), right: (" << gap.convex.terminal_ridx << ", " << gap.convex.terminal_rdist << ")" << std::endl;
            gap.mode.terminal_agc = true;
        }
        std::cout << "post-AGC gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")" << std::endl;
    }

    void GapManipulator::clipGapByLaserScan(dynamic_gap::Gap& gap) {
        std::cout << "~running clipGapByLaserScan" << std::endl;
        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_laser_scan;
        double laserscan_left_dist = 0.8 * stored_scan_msgs.ranges.at(gap.convex.terminal_lidx);
        if (gap.convex.terminal_ldist > laserscan_left_dist) {
            std::cout << "clipping left dist from " << gap.convex.terminal_ldist << " to " << laserscan_left_dist << std::endl;
            gap.convex.terminal_ldist = laserscan_left_dist;
        }
        double laserscan_right_dist = 0.8 * stored_scan_msgs.ranges.at(gap.convex.terminal_ridx);
        if (gap.convex.terminal_rdist > laserscan_right_dist) {
            std::cout << "clipping right dist from " << gap.convex.terminal_rdist << " to " << laserscan_right_dist << std::endl;
            gap.convex.terminal_rdist = laserscan_right_dist;
        }
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap& gap, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        if (!cfg_->gap_manip.radial_extend) {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }

        std::cout << "running radialExtendGap" << std::endl;

        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_laser_scan;
        int half_num_scan = gap.half_scan; // changing this
        
        int lidx = initial ? gap.convex.convex_lidx : gap.convex.terminal_lidx;
        int ridx = initial ? gap.convex.convex_ridx : gap.convex.terminal_ridx;
        float ldist = initial ? gap.convex.convex_ldist : gap.convex.terminal_ldist;
        float rdist = initial ? gap.convex.convex_rdist : gap.convex.terminal_rdist;

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) half_num_scan - lidx) / half_num_scan * M_PI);
        y1 = (ldist) * sin(-((float) half_num_scan - lidx) / half_num_scan * M_PI);

        x2 = (rdist) * cos(-((float) half_num_scan - ridx) / half_num_scan * M_PI);
        y2 = (rdist) * sin(-((float) half_num_scan - ridx) / half_num_scan * M_PI);

        std::cout << "pre-RE gap in polar. left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")" << std::endl;
        std::cout << "pre-RE gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")" << std::endl;
        
        Eigen::Vector2f left_point(x1, y1);
        Eigen::Vector2f right_point(x2, y2);

        Eigen::Vector2f eL = left_point / left_point.norm();
        Eigen::Vector2f eR = right_point / right_point.norm();

        // middle of gap direction
        Eigen::Vector2f eB = (eL + eR) / 2;
        eB /= eB.norm();
        // angular size of gap
        float gap_size = std::acos(eL.dot(eR));

        // minSafeDist is the minimum distance within the laser scan 
        float s = gap.getMinSafeDist(); // initial ? gap.getMinSafeDist() : dynamic_laser_scan.range_min;
        // std::cout << "min safe dist: " << s << std::endl;
        
        // point opposite direction of middle of gap, magnitude of min safe dist
        Eigen::Vector2f qB = -s * eB;
        
        // push out left and right points
        Eigen::Vector2f qLp = left_point - qB;
        Eigen::Vector2f qRp = right_point - qB;

        // (x,y) to (r, theta) 
        Eigen::Vector2f pLp = car2pol(qLp);
        Eigen::Vector2f pRp = car2pol(qRp);

        // angular difference between right and left
        float phiB = pRp(1) - pLp(1);

        Eigen::Vector2f pB = car2pol(-qB);

        float thL = pB(1) - gap_size / 4; // left side 1/2 throgh
        float thR = pB(1) + gap_size / 4; // right side 1/2 through

        Eigen::Vector2f pLn = pTheta(thL, phiB, pLp, pRp);
        Eigen::Vector2f pRn = pTheta(thR, phiB, pLp, pRp);

        Eigen::Vector2f qLn = pol2car(pLn) + qB;
        Eigen::Vector2f qRn = pol2car(pRn) + qB;

        // Store info back to original gap;
        Eigen::Vector2f polqLn = car2pol(qLn);
        Eigen::Vector2f polqRn = car2pol(qRn);

        int new_left_idx = std::floor(polqLn(1) / M_PI * half_num_scan + half_num_scan);
        int new_right_idx = std::ceil(polqRn(1) / M_PI * half_num_scan + half_num_scan);
        if (new_left_idx == new_right_idx) {
            std::cout << "post-RE, indices are the same" << std::endl;
            new_right_idx += 1;
        }

        if (initial) {
            gap.convex.convex_lidx = new_left_idx;
            gap.convex.convex_ridx = new_right_idx;
            gap.convex.convex_ldist = polqLn(0);
            gap.convex.convex_rdist = polqRn(0);
            gap.mode.convex = true;
            x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
            y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
            x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
            y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
            std::cout << "post-RE gap in polar, left: (" << gap.convex.convex_lidx << ", " << gap.convex.convex_ldist << "), right: (" << gap.convex.convex_ridx << ", " << gap.convex.convex_rdist << ")" << std::endl;
            gap.qB = qB;
        } else {
            gap.convex.terminal_lidx = new_left_idx;
            gap.convex.terminal_ridx = new_right_idx;
            gap.convex.terminal_ldist = polqLn(0);
            gap.convex.terminal_rdist = polqRn(0);
            gap.mode.terminal_convex = true;
            x1 = (gap.convex.terminal_ldist) * cos(-((float) half_num_scan - gap.convex.terminal_lidx) / half_num_scan * M_PI);
            y1 = (gap.convex.terminal_ldist) * sin(-((float) half_num_scan - gap.convex.terminal_lidx) / half_num_scan * M_PI);
            x2 = (gap.convex.terminal_rdist) * cos(-((float) half_num_scan - gap.convex.terminal_ridx) / half_num_scan * M_PI);
            y2 = (gap.convex.terminal_rdist) * sin(-((float) half_num_scan - gap.convex.terminal_ridx) / half_num_scan * M_PI);
            std::cout << "post-RE gap in polar. left: (" << gap.convex.terminal_lidx << ", " << gap.convex.terminal_ldist << "), right: (" << gap.convex.terminal_ridx << ", " << gap.convex.terminal_rdist << ")" << std::endl;
            gap.terminal_qB = qB;
        }
        std::cout << "post-RE gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")" << std::endl;

        // std::cout << "after radial extension:" << std::endl;

        // std::cout << "qB: " << qB[0] << ", " << qB[1] << std::endl;

        // points are not actually extended here, qB is just calculated.
        // points are moved though, might as well do it here
        
        /*
        gap.left_model->inflate_model(x1, y1);
        gap.right_model->inflate_model(x2, y2);

        Matrix<double, 4, 1> left_cartesian_state = gap.left_model->get_cartesian_state();
        Matrix<double, 4, 1> right_cartesian_state = gap.right_model->get_cartesian_state();
        std::cout << "inflated models in cart, left: (" << left_cartesian_state[0] << ", " << left_cartesian_state[1] << ", " << left_cartesian_state[2] << ", " << left_cartesian_state[3] << "), ";
        std::cout << "right: (" << right_cartesian_state[0] << ", " << right_cartesian_state[1] << ", " << right_cartesian_state[2] << ", " << right_cartesian_state[3] << ")" << std::endl;
        */
        // we will see this in trajectory generator
        //gap.left_model->extend_model_origin(qB);
        //gap.right_model->extend_model_origin(qB);
        
        //ROS_DEBUG_STREAM("l: " << gap._left_idx << " to " << gap.convex_lidx << ", r: " << gap._right_idx << " to " << gap.convex_ridx);
        //ROS_DEBUG_STREAM("ldist: " << gap._ldist << " to " << gap.convex_ldist << ", rdist: " << gap._rdist << " to " << gap.convex_rdist);

        return;
    }

    Eigen::Vector2f GapManipulator::car2pol(Eigen::Vector2f a) {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2f GapManipulator::pol2car(Eigen::Vector2f a) {
        return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }

    Eigen::Vector2f GapManipulator::pTheta(
        float th, float phiB, Eigen::Vector2f pLp, Eigen::Vector2f pRp) {
        return pRp * (th - pLp(1)) / phiB + pLp * (pRp(1) - th) / phiB;
    }

}
