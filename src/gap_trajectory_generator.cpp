 #include <dynamic_gap/gap_trajectory_generator.h>

namespace dynamic_gap{

    void GapTrajGenerator::initializeSolver() {
        ROS_INFO_STREAM("initializing solver");

        int num_curve_points = cfg_->traj.num_curve_points;
        int num_qB_points = (cfg_->gap_manip.radial_extend) ? cfg_->traj.num_qB_points : 0;

        int Kplus1 = 2*(num_curve_points + num_qB_points) + 1;
        hessian.resize(Kplus1, Kplus1);

        gradient = Eigen::VectorXd::Zero(Kplus1, 1);

        linearMatrix.resize(Kplus1, Kplus1);

        lowerBound = Eigen::MatrixXd::Zero(Kplus1, 1);

        upperBound = Eigen::MatrixXd::Zero(Kplus1, 1);

        for (int i = 0; i < Kplus1; i++) {
            lowerBound(i, 0) = -OsqpEigen::INFTY;
            upperBound(i, 0) = -0.0000001; // this leads to non-zero weights. Closer to zero this number goes, closer to zero the weights go. This makes sense
        }

        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);        
        solver.data()->setNumberOfVariables(Kplus1);
        solver.data()->setNumberOfConstraints(Kplus1);

        if (!solver.data()->setHessianMatrix(hessian)) {
            ROS_FATAL_STREAM("SOLVER FAILED TO SET HESSIAN");
        } // H ?

        if(!solver.data()->setGradient(gradient)) {
            ROS_FATAL_STREAM("SOLVER FAILED TO SET GRADIENT");
        } // f ?

        if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) {
            ROS_FATAL_STREAM("SOLVER FAILED TO SET LINEAR MATRIX");
        }

        if(!solver.data()->setLowerBound(lowerBound)) {
            ROS_FATAL_STREAM("SOLVER FAILED TO SET LOWER BOUND");
        }

        if(!solver.data()->setUpperBound(upperBound)) {
            ROS_FATAL_STREAM("SOLVER FAILED TO SET UPPER BOUND");
        }

        if(!solver.initSolver()) {
            ROS_FATAL_STREAM("SOLVER FAILED TO INITIALIZE SOLVER");
        }
    }

    std::tuple<geometry_msgs::PoseArray, std::vector<double>> GapTrajGenerator::generateTrajectory(
                                                    dynamic_gap::Gap& selectedGap, 
                                                    geometry_msgs::PoseStamped curr_pose, 
                                                    geometry_msgs::Twist curr_vel,
                                                    bool run_g2g) {
        try {        
            // return geometry_msgs::PoseArray();
            geometry_msgs::PoseArray posearr;
            std::vector<double> timearr;
            double gen_traj_start_time = ros::Time::now().toSec();
            posearr.header.stamp = ros::Time::now();
            double coefs = cfg_->traj.scale;
            write_trajectory corder(posearr, cfg_->robot_frame_id, coefs, timearr);
            posearr.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;

            Eigen::Vector4d ego_x(curr_pose.pose.position.x + 1e-5, curr_pose.pose.position.y + 1e-6,
                                  0.0, 0.0); // curr_vel.linear.x, curr_vel.linear.y

            // get gap points in cartesian
            float x_left = selectedGap.cvx_LDist() * cos(((float) selectedGap.cvx_LIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float y_left = selectedGap.cvx_LDist() * sin(((float) selectedGap.cvx_LIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float x_right = selectedGap.cvx_RDist() * cos(((float) selectedGap.cvx_RIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float y_right = selectedGap.cvx_RDist() * sin(((float) selectedGap.cvx_RIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);

            float term_x_left = selectedGap.cvx_term_LDist() * cos(((float) selectedGap.cvx_term_LIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float term_y_left = selectedGap.cvx_term_LDist() * sin(((float) selectedGap.cvx_term_LIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float term_x_right = selectedGap.cvx_term_RDist() * cos(((float) selectedGap.cvx_term_RIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);
            float term_y_right = selectedGap.cvx_term_RDist() * sin(((float) selectedGap.cvx_term_RIdx() - selectedGap.half_scan) / selectedGap.half_scan * M_PI);

            if (run_g2g) { //   || selectedGap.goal.goalwithin
                state_type x = {ego_x[0], ego_x[1], ego_x[2], ego_x[3],
                                0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0,0.0,
                                selectedGap.goal.x, selectedGap.goal.y};
                // ROS_INFO_STREAM("Goal to Goal");
                g2g inte_g2g(selectedGap.goal.x, selectedGap.goal.y,
                             selectedGap.terminal_goal.x, selectedGap.terminal_goal.y,
                             selectedGap.gap_lifespan, cfg_->control.vx_absmax);
                boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                inte_g2g, x, 0.0,
                cfg_->traj.integrate_maxt,
                cfg_->traj.integrate_stept,
                corder);
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
                return return_tuple;
            }

            Eigen::Vector2f qB = selectedGap.qB;

            Eigen::Vector2d initial_goal(selectedGap.goal.x, selectedGap.goal.y);
            Eigen::Vector2d terminal_goal(selectedGap.terminal_goal.x, selectedGap.terminal_goal.y);

            double initial_goal_x = initial_goal[0];
            double initial_goal_y = initial_goal[1];
            double terminal_goal_x = terminal_goal[0];
            double terminal_goal_y = terminal_goal[1];

            double goal_vel_x = (terminal_goal_x - initial_goal_x) / selectedGap.gap_lifespan; // absolute velocity (not relative to robot)
            double goal_vel_y = (terminal_goal_y - initial_goal_y) / selectedGap.gap_lifespan;

            if (cfg_->traj.debug_log) {
                ROS_INFO_STREAM("actual initial robot pos: (" << ego_x[0] << ", " << ego_x[1] << ")");
                ROS_INFO_STREAM("actual inital robot velocity: " << ego_x[2] << ", " << ego_x[3] << ")");
                ROS_INFO_STREAM("actual initial left point: (" << x_left << ", " << y_left << "), actual initial right point: (" << x_right << ", " << y_right << ")"); 
                ROS_INFO_STREAM("actual terminal left point: (" << term_x_left << ", " << term_y_left << "), actual terminal right point: (" << term_x_right << ", " << term_y_right << ")");
                ROS_INFO_STREAM("actual initial goal: (" << initial_goal_x << ", " << initial_goal_y << ")"); 
                ROS_INFO_STREAM("actual terminal goal: (" << terminal_goal_x << ", " << terminal_goal_y << ")"); 
            }
            
            double left_vel_x = (term_x_left - x_left) / selectedGap.gap_lifespan;
            double left_vel_y = (term_y_left - y_left) / selectedGap.gap_lifespan;

            double right_vel_x = (term_x_right - x_right) / selectedGap.gap_lifespan;
            double right_vel_y = (term_y_right - y_right) / selectedGap.gap_lifespan;

            state_type x = {ego_x[0], ego_x[1], ego_x[2], ego_x[3],
                            x_left, y_left, left_vel_x, left_vel_y,
                            x_right, y_right, right_vel_x, right_vel_y,
                            initial_goal_x, initial_goal_y, goal_vel_x, goal_vel_y};
            
            // or if model is invalid?
            //bool invalid_models = left_model_state[0] < 0.01 || right_model_state[0] < 0.01;
            if (selectedGap.goal.discard || selectedGap.terminal_goal.discard && cfg_->traj.debug_log) {
                ROS_INFO_STREAM("discarding gap");
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
                return return_tuple;
            }
                      
            Eigen::Vector2d init_rbt_pos(x[0], x[1]);
            Eigen::Vector2d left_pt_0(x_left, y_left);
            Eigen::Vector2d left_pt_1(term_x_left, term_y_left);
            Eigen::Vector2d right_pt_0(x_right, y_right);
            Eigen::Vector2d right_pt_1(term_x_right, term_y_right);
            Eigen::Vector2d nonrel_left_vel(left_vel_x, left_vel_y);
            Eigen::Vector2d nonrel_right_vel(right_vel_x, right_vel_y);

            Eigen::Vector2d nom_vel(cfg_->control.vx_absmax, cfg_->control.vy_absmax);
            Eigen::Vector2d nom_acc(cfg_->control.ax_absmax, cfg_->control.ay_absmax);
            Eigen::Vector2d goal_pt_1(terminal_goal_x, terminal_goal_y);
            Eigen::Vector2d gap_radial_extension(qB[0], qB[1]);
            Eigen::Vector2d left_bezier_origin(selectedGap.left_bezier_origin[0],
                                               selectedGap.left_bezier_origin[1]);
            Eigen::Vector2d right_bezier_origin(selectedGap.right_bezer_origin[0],
                                                selectedGap.right_bezer_origin[1]);

            int num_curve_points = cfg_->traj.num_curve_points;
            int num_qB_points = (cfg_->gap_manip.radial_extend) ? cfg_->traj.num_qB_points : 0;

            Eigen::MatrixXd left_curve(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd right_curve(num_curve_points + num_qB_points, 2);            
            Eigen::MatrixXd all_curve_pts(2*(num_curve_points + num_qB_points), 2);

            Eigen::MatrixXd left_curve_vel(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd right_curve_vel(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd left_curve_inward_norm(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd right_curve_inward_norm(num_curve_points + num_qB_points, 2);
            Eigen::MatrixXd all_inward_norms(2*(num_curve_points + num_qB_points), 2);

            Eigen::MatrixXd left_right_centers(2*(num_curve_points + num_qB_points), 2);

            Eigen::MatrixXd all_centers(2*(num_curve_points + num_qB_points) + 1, 2);
            
            double left_weight, right_weight;
            // THIS IS BUILT WITH EXTENDED POINTS. 
            double start_time = ros::Time::now().toSec();
            buildBezierCurve(selectedGap, left_curve, right_curve, all_curve_pts, 
                             left_curve_vel, right_curve_vel,
                             left_curve_inward_norm, right_curve_inward_norm, 
                             all_inward_norms, left_right_centers, all_centers,
                             nonrel_left_vel, nonrel_right_vel, nom_vel, 
                             left_pt_0, left_pt_1, right_pt_0, right_pt_1, 
                             gap_radial_extension, goal_pt_1, left_weight, right_weight, num_curve_points, num_qB_points, 
                             init_rbt_pos, left_bezier_origin, right_bezier_origin);
            // ROS_INFO_STREAM("buildBezierCurve time elapsed: " << (ros::Time::now().toSec() - start_time));
            // ROS_INFO_STREAM("after buildBezierCurve, left weight: " << left_weight << ", right_weight: " << right_weight);
            selectedGap.left_weight = left_weight;
            selectedGap.right_weight = right_weight;
            selectedGap.left_right_centers = left_right_centers;
            selectedGap.all_curve_pts = all_curve_pts;

            /*
            SETTING UP SOLVER
            */
            int N = all_curve_pts.rows();
            int Kplus1 = all_centers.rows();
            // ROS_INFO_STREAM("N: " << N << ", Kplus1: " << Kplus1);

            Eigen::MatrixXd A(Kplus1, N+1);
            // double start_time = ros::Time::now().toSec();
            setConstraintMatrix(A, N, Kplus1, all_curve_pts, all_inward_norms, all_centers);
            // ROS_INFO_STREAM("setConstraintMatrix time elapsed: " << (ros::Time::now().toSec() - start_time));
            // ROS_INFO_STREAM("A: " << A);
            
            // Eigen::MatrixXd b = Eigen::MatrixXd::Zero(Kplus1, 1);

            for (int i = 0; i < Kplus1; i++) {
                for (int j = 0; j < Kplus1; j++) {
                    if (i == j) {
                        hessian.coeffRef(i, j) = 1.0;
                    }

                    // need to transpose A vector, just doing here
                    linearMatrix.coeffRef(i, j) = A.coeff(j, i);
                }
            }

            //ROS_INFO_STREAM("Hessian: " << hessian);
            //ROS_INFO_STREAM("Gradient: " << gradient);
            //ROS_INFO_STREAM("linearMatrix: " << linearMatrix);
            //ROS_INFO_STREAM("lowerBound: " << lowerBound);
            //ROS_INFO_STREAM("upperBound: " << upperBound);

            // solver.
            if (!solver.updateHessianMatrix(hessian)) {
                ROS_FATAL_STREAM("SOLVER FAILED TO UPDATE HESSIAN");
            } // H ?

            if (!solver.updateGradient(gradient)) {
                ROS_FATAL_STREAM("SOLVER FAILED TO UPDATE GRADIENT");
            } // f ?

            if (!solver.updateLinearConstraintsMatrix(linearMatrix)) {
                ROS_FATAL_STREAM("SOLVER FAILED TO UPDATE LINEAR MATRIX");
            }

            if (!solver.updateLowerBound(lowerBound)) {
                ROS_FATAL_STREAM("SOLVER FAILED TO UPDATE LOWER BOUND");
            }

            if (!solver.updateUpperBound(upperBound)) {
                ROS_FATAL_STREAM("SOLVER FAILED TO UPDATE UPPER BOUND");
            }

            // if(!solver.setPrimalVariable(w_0)) return;
            
            // solve the QP problem
            double opt_start_time = ros::WallTime::now().toSec();
            if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
                ROS_FATAL_STREAM("SOLVER FAILED TO SOLVE PROBLEM");
            }
            // get the controller input
            Eigen::MatrixXd weights = solver.getSolution();
            ROS_INFO_STREAM("optimization time taken: " << (ros::WallTime::now().toSec() - opt_start_time));

            // weights = raw_weights / raw_weights.norm();                                

            // if(!solver.setPrimalVariable(w_0)) return;
            
            // solve the QP problem
            /*
            ROS_INFO_STREAM("current solution: "); 
            
            std::string weights_string;
            for (int i = 0; i < Kplus1; i++) {
                ROS_INFO_STREAM(weights.coeff(i, 0)); 
            } 
            */  

            reachable_gap_APF reachable_gap_APF_inte(init_rbt_pos, goal_pt_1, cfg_->gap_manip.K_acc,
                                                    cfg_->control.vx_absmax, nom_acc, all_centers, weights);   
            
            start_time = ros::Time::now().toSec();
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                    reachable_gap_APF_inte, x, 0.0, selectedGap.gap_lifespan, 
                                                    cfg_->traj.integrate_stept, corder);
            // ROS_INFO_STREAM("integration time elapsed: " << (ros::Time::now().toSec() - start_time));

            std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
            ROS_INFO_STREAM("generateTrajectory time taken: " << ros::Time::now().toSec() - gen_traj_start_time);
            return return_tuple;
            
        } catch (...) {
            ROS_FATAL_STREAM("integrator");
        }

    }

    void GapTrajGenerator::setConstraintMatrix(Eigen::MatrixXd &A, int N, int Kplus1, 
                                               Eigen::MatrixXd all_curve_pts, Eigen::MatrixXd all_inward_norms, Eigen::MatrixXd all_centers) {

        Eigen::MatrixXd gradient_of_pti_wrt_centers(Kplus1, 2); // (2, Kplus1); // 
        Eigen::MatrixXd A_N(Kplus1, N);
        Eigen::MatrixXd A_S = Eigen::MatrixXd::Zero(Kplus1, 1);
        Eigen::MatrixXd neg_one_vect = Eigen::MatrixXd::Constant(1, 1, -1.0);

        double eps = 0.0000001;
        // all_centers size: (Kplus1 rows, 2 cols)
        Eigen::Vector2d boundary_pt_i, inward_norm_vector;
        Eigen::MatrixXd cent_to_boundary;
        Eigen::VectorXd rowwise_sq_norms;
        for (int i = 0; i < N; i++) {
            boundary_pt_i = all_curve_pts.row(i);
            inward_norm_vector = all_inward_norms.row(i);
            //Eigen::Matrix<double, 1, 2> inward_norm_vector = all_inward_norms.row(i);        
            
            // doing boundary - all_centers
            cent_to_boundary = (-all_centers).rowwise() + boundary_pt_i.transpose();
            // need to divide rowwise by norm of each row
            rowwise_sq_norms = cent_to_boundary.rowwise().squaredNorm();
            gradient_of_pti_wrt_centers = cent_to_boundary.array().colwise() / (rowwise_sq_norms.array() + eps);
            
            //ROS_INFO_STREAM("inward_norm_vector: " << inward_norm_vector);
            //ROS_INFO_STREAM("gradient_of_pti: " << gradient_of_pti_wrt_centers);

            A_N.col(i) = gradient_of_pti_wrt_centers * inward_norm_vector; // A_pi;

            // A_N.col(i) = gradient_of_pti_wrt_centers * inward_norm_vector;
            // ROS_INFO_STREAM("inward_norm_vector size: " << inward_norm_vector.rows() << ", " << inward_norm_vector.cols());
            // ROS_INFO_STREAM("gradient_of_pti_wrt_centers size: " << gradient_of_pti_wrt_centers.rows() << ", " << gradient_of_pti_wrt_centers.cols());
            // ROS_INFO_STREAM("A_pi size: " << A_pi.rows() << ", " << A_pi.cols());

            // ROS_INFO_STREAM("A_pi: " << A_pi);
            // dotting inward norm (Kplus1 rows, 2 columns) with gradient of pti (Kplus1 rows, 2 columns) to get (Kplus1 rows, 1 column)
        }

        // ROS_INFO_STREAM("A_N: " << A_N);
        // ROS_INFO_STREAM("A_N_new: " << A_N_new);
            
        A_S.row(0) = neg_one_vect;

        A << A_N, A_S;
    }


    Matrix<double, 5, 1> GapTrajGenerator::cartesian_to_polar(Eigen::Vector4d x) {
        Matrix<double, 5, 1> polar_y;
        polar_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        polar_y(0) = 1.0 / std::sqrt(pow(x[0], 2) + pow(x[1], 2));
        double beta = std::atan2(x[1], x[0]);
        polar_y(1) = std::sin(beta);
        polar_y(2) = std::cos(beta);
        polar_y(3) = (x[0]*x[2] + x[1]*x[3]) / (pow(x[0],2) + pow(x[1], 2)); // rdot/r
        polar_y(4) = (x[0]*x[3] - x[1]*x[2]) / (pow(x[0],2) + pow(x[1], 2)); // betadot
        return polar_y;
    }

    double num_int(Eigen::Vector2d pt_origin, 
                   Eigen::Vector2d pt_0, 
                   Eigen::Vector2d pt_1, 
                   double t_start, double t_end, double num_points) {
        double interp_dist = 0.0;
        double steps = num_points - 1;

        // ROS_INFO_STREAM("performing numerical integration between " << t_start << " and " << t_end);
        double t_i, t_iplus1, dist;
        Eigen::Vector2d pt_i, pt_iplus1;
        for (int i = 0; i < steps; i++) {
            t_i = t_start + (t_end - t_start) * double(i / steps);
            t_iplus1 = t_start + (t_end - t_start) * double((i + 1) / steps);
            // ROS_INFO_STREAM("t_i: " << t_i << ", t_iplus1: " << t_iplus1);

            pt_i = (1 - t_i)*(1 - t_i)*pt_origin + 2*(1 - t_i)*t_i*pt_0 + (t_i)*(t_i)*pt_1;
            pt_iplus1 = (1 - t_iplus1)*(1 - t_iplus1)*pt_origin + 2*(1 - t_iplus1)*t_iplus1*pt_0 + (t_iplus1)*(t_iplus1)*pt_1;
            // ROS_INFO_STREAM("pt_i: " << pt_i[0] << ", " << pt_i[1] << ", pt_iplus1: " << pt_iplus1[0] << ", " << pt_iplus1[1]);
            dist = (pt_iplus1 - pt_i).norm();
            // ROS_INFO_STREAM("adding dist:" << dist);
            interp_dist += dist;
        }

        return interp_dist;
    }

    Eigen::VectorXd GapTrajGenerator::arclength_sample_bezier(Eigen::Vector2d pt_origin, 
                                                   Eigen::Vector2d pt_0, 
                                                   Eigen::Vector2d pt_1, 
                                                   double num_curve_points,
                                                   double & des_dist_interval) {
        // ROS_INFO_STREAM("running arclength sampling");
        // ROS_INFO_STREAM("pt_origin: " << pt_origin[0] << ", " << pt_origin[1]);
        // ROS_INFO_STREAM("pt_0: " << pt_0[0] << ", " << pt_0[1]);
        // ROS_INFO_STREAM("pt_1: " << pt_1[0] << ", " << pt_1[1]);        
        double total_approx_dist = num_int(pt_origin, pt_0, pt_1, 0.0, 1.0, num_curve_points);

        double t_kmin1 = 0.0;
        double num_interp_points = 5.0;
        Eigen::VectorXd uniform_indices = Eigen::MatrixXd::Zero(int(num_curve_points), 1);
        int uniform_index_entry = 1;
        double num_sampled_points = 2 * num_curve_points;
        des_dist_interval = total_approx_dist / (num_curve_points - 1);
        double dist_thresh = des_dist_interval / 100.0;

        // ROS_INFO_STREAM("number of points: " << num_curve_points);
        // ROS_INFO_STREAM("total distance: " << total_approx_dist);
        // ROS_INFO_STREAM("desired distance interval: " << des_dist_interval);
        double t_k, current_dist, t_prev, t_interp, interp_dist, t_high, t_low;
        for (double t_k = 0.0; t_k <= 1.0; t_k += (1.0 / num_sampled_points)) {
            current_dist = num_int(pt_origin, pt_0, pt_1, t_kmin1, t_k, num_interp_points);
            // ROS_INFO_STREAM("from " << t_kmin1 << " to " << t_k << ": " << current_dist);

            if (std::abs(current_dist - des_dist_interval) < dist_thresh) {
                // ROS_INFO_STREAM("adding " << t_k);
                uniform_indices(uniform_index_entry, 0) = t_k;
                uniform_index_entry += 1;
                t_kmin1 = t_k;
            } else if (current_dist > des_dist_interval) {
                t_prev = t_k - (1.0 / num_sampled_points);
                t_interp = (t_k + t_prev) / 2.0;
                interp_dist = num_int(pt_origin, pt_0, pt_1, t_kmin1, t_interp, num_interp_points);

                // ROS_INFO_STREAM("from " << t_kmin1 << " to " << t_interp << ": " << interp_dist);

                t_high = t_k;
                t_low = t_prev;
                while ( abs(interp_dist - des_dist_interval) > dist_thresh) {
                    if (interp_dist < des_dist_interval) {
                        t_low = t_interp;
                        t_interp = (t_interp + t_high) / 2.0;
                        // ROS_INFO_STREAM("raising t_interp to: " << t_interp);
                    } else {
                        t_high = t_interp;
                        t_interp = (t_interp + t_low) / 2.0;
                        // ROS_INFO_STREAM("lowering t_interp to: " << t_interp);

                    }
                    interp_dist = num_int(pt_origin, pt_0, pt_1, t_kmin1, t_interp, num_interp_points);
                    // ROS_INFO_STREAM("from " << t_kmin1 << " to " << t_interp << ": " << interp_dist);
                }
                
                // ROS_INFO_STREAM("adding " << t_interp);
                uniform_indices(uniform_index_entry, 0) = t_interp;
                uniform_index_entry += 1;
                t_kmin1 = t_interp;
            }   
        }

        uniform_indices(int(num_curve_points) - 1, 0) = 1.0;

        return uniform_indices;
        /*
        ROS_INFO_STREAM("uniform indices: ");
        for (int i = 0; i < int(num_curve_points); i++) {
            ROS_INFO_STREAM(uniform_indices(i, 0));
        }
        */
        // ROS_INFO_STREAM("total approx dist: " << total_approx_dist);
    }
    
    double getLeftToRightAngle(Eigen::Vector2d left_norm_vect, Eigen::Vector2d right_norm_vect) {
        double determinant = left_norm_vect[1]*right_norm_vect[0] - left_norm_vect[0]*right_norm_vect[1];
        double dot_product = left_norm_vect[0]*right_norm_vect[0] + left_norm_vect[1]*right_norm_vect[1];

        double left_to_right_angle = std::atan2(determinant, dot_product);
        
        // removing this for inward_norm thing
        /*
        if (left_to_right_angle < 0) {
            left_to_right_angle += 2*M_PI; 
        }
        */

        return left_to_right_angle;
    }

    void GapTrajGenerator::buildBezierCurve(dynamic_gap::Gap& selectedGap, Eigen::MatrixXd & left_curve, Eigen::MatrixXd & right_curve, Eigen::MatrixXd & all_curve_pts,
                                            Eigen::MatrixXd & left_curve_vel, Eigen::MatrixXd & right_curve_vel,
                                            Eigen::MatrixXd & left_curve_inward_norm, Eigen::MatrixXd & right_curve_inward_norm, 
                                            Eigen::MatrixXd & all_inward_norms, Eigen::MatrixXd & left_right_centers, Eigen::MatrixXd & all_centers,
                                            Eigen::Vector2d nonrel_left_vel, Eigen::Vector2d nonrel_right_vel, Eigen::Vector2d nom_vel,
                                            Eigen::Vector2d left_pt_0, Eigen::Vector2d left_pt_1, Eigen::Vector2d right_pt_0, Eigen::Vector2d right_pt_1, 
                                            Eigen::Vector2d gap_radial_extension, Eigen::Vector2d goal_pt_1, double & left_weight, double & right_weight, 
                                            double num_curve_points, double num_qB_points, Eigen::Vector2d init_rbt_pos,
                                            Eigen::Vector2d left_bezier_origin, Eigen::Vector2d right_bezier_origin) {  
        
        
        // ROS_INFO_STREAM("building bezier curve");
        
        left_weight = nonrel_left_vel.norm() / nom_vel.norm(); // capped at 1, we can scale down towards 0 until initial constraints are met?
        right_weight = nonrel_right_vel.norm() / nom_vel.norm();

        // for a totally static gap, can get no velocity on first bezier curve point which corrupts vector field
        Eigen::Vector2d weighted_left_pt_0, weighted_right_pt_0;
        if (nonrel_left_vel.norm() > 0.0) {
            weighted_left_pt_0 = left_bezier_origin + left_weight * (left_pt_0 - left_bezier_origin);
        } else {
            weighted_left_pt_0 = (0.95 * left_bezier_origin + 0.05 * left_pt_1);
        }

        if (nonrel_right_vel.norm() >  0.0) {
            weighted_right_pt_0 = right_bezier_origin + right_weight * (right_pt_0 - right_bezier_origin);
        } else {
            weighted_right_pt_0 = (0.95 * right_bezier_origin + 0.05 * right_pt_1);
        }
        
        selectedGap.left_pt_0 = weighted_left_pt_0;
        selectedGap.left_pt_1 = left_pt_1;
        selectedGap.right_pt_0 = weighted_right_pt_0;
        selectedGap.right_pt_1 = right_pt_1;        /*
        ROS_INFO_STREAM("gap_radial_extension: " << gap_radial_extension[0] << ", " << gap_radial_extension[1]);
        ROS_INFO_STREAM("init_rbt_pos: " << init_rbt_pos[0] << ", " << init_rbt_pos[1]);

        ROS_INFO_STREAM("left_bezier_origin: " << left_bezier_origin[0] << ", " << left_bezier_origin[1]);
        ROS_INFO_STREAM("left_pt_0: " << left_pt_0[0] << ", " << left_pt_0[1]);
        ROS_INFO_STREAM("weighted_left_pt_0: " << weighted_left_pt_0[0] << ", " << weighted_left_pt_0[1]);       
        ROS_INFO_STREAM("left_pt_1: " << left_pt_1[0] << ", " << left_pt_1[1]);
        ROS_INFO_STREAM("left_vel: " << nonrel_left_vel[0] << ", " << nonrel_left_vel[1]);

        ROS_INFO_STREAM("right_bezier_origin: " << right_bezier_origin[0] << ", " << right_bezier_origin[1]);
        ROS_INFO_STREAM("right_pt_0: " << right_pt_0[0] << ", " << right_pt_0[1]);
        ROS_INFO_STREAM("weighted_right_pt_0: " << weighted_right_pt_0[0] << ", " << weighted_right_pt_0[1]);        
        ROS_INFO_STREAM("right_pt_1: " << right_pt_1[0] << ", " << right_pt_1[1]);
        ROS_INFO_STREAM("right_vel: " << nonrel_right_vel[0] << ", " << nonrel_right_vel[1]);

        ROS_INFO_STREAM("nom_vel: " << nom_vel[0] << ", " << nom_vel[1]);
        */
        Eigen::Matrix2d rpi2, neg_rpi2;
        double rot_val = M_PI/2;
        double s, s_left, s_right;
        rpi2 << std::cos(rot_val), -std::sin(rot_val), std::sin(rot_val), std::cos(rot_val);
        neg_rpi2 << std::cos(-rot_val), -std::sin(-rot_val), std::sin(-rot_val), std::cos(-rot_val);

        // ROS_INFO_STREAM("radial extensions: ");
        // ADDING DISCRETE POINTS FOR RADIAL GAP EXTENSION
        double pos_val0, pos_val1, pos_val2, vel_val0, vel_val1, vel_val2;
        Eigen::Vector2d curr_left_pt, curr_left_vel, left_inward_vect, rotated_curr_left_vel, left_inward_norm_vect,
                        curr_right_pt, curr_right_vel, right_inward_vect, rotated_curr_right_vel, right_inward_norm_vect;
        
        for (double i = 0; i < num_qB_points; i++) {
            s = (i) / (num_qB_points - 1);
            pos_val0 = (1 - s);
            pos_val1 = s;
            curr_left_pt = pos_val0 * gap_radial_extension + pos_val1 * left_bezier_origin;
            curr_left_vel = (left_bezier_origin - gap_radial_extension);
            left_inward_vect = neg_rpi2 * curr_left_vel;
            left_inward_norm_vect = left_inward_vect / left_inward_vect.norm();
            left_curve.row(i) = curr_left_pt;
            left_curve_vel.row(i) = curr_left_vel;
            left_curve_inward_norm.row(i) = left_inward_norm_vect;

            curr_right_pt = pos_val0 * gap_radial_extension + pos_val1 * right_bezier_origin;
            curr_right_vel = (right_bezier_origin - gap_radial_extension);
            right_inward_vect = rpi2 * curr_right_vel;
            right_inward_norm_vect = right_inward_vect / right_inward_vect.norm();
            right_curve.row(i) = curr_right_pt;
            right_curve_vel.row(i) = curr_right_vel;
            right_curve_inward_norm.row(i) = right_inward_norm_vect;
        }

        double eps = 0.0000001;

        // model gives: left_pt - rbt.
        // populating the quadratic weighted bezier

        double des_left_dist = 0.01;
        double des_right_dist = 0.01;
        Eigen::VectorXd left_indices = arclength_sample_bezier(left_bezier_origin, weighted_left_pt_0, left_pt_1, num_curve_points, des_left_dist);
        Eigen::VectorXd right_indices = arclength_sample_bezier(right_bezier_origin, weighted_right_pt_0, right_pt_1, num_curve_points, des_right_dist);
        Eigen::Matrix<double, 1, 2> origin, centered_origin_inward_norm;
        origin << 0.0, 0.0;
        double offset = 0.1; // (des_left_dist + des_right_dist) / 2.0;
        // ROS_INFO_STREAM("offset: " << offset);

        int counter = 0;
        for (double i = num_qB_points; i < (num_curve_points + num_qB_points); i++) {
            // s = (i - num_qB_points) / (num_curve_points - 1);
            
            s_left = left_indices(counter, 0);
            s_right = right_indices(counter, 0);
            counter++;

            // ROS_INFO_STREAM("s: " << s << ", s_left: " << s_left << ", s_right: " << s_right);
            pos_val0 = (1 - s_left) * (1 - s_left);
            pos_val1 = 2*(1 - s_left)*s_left;
            pos_val2 = s_left*s_left;

            vel_val0 = (2*s_left - 2);
            vel_val1 = (2 - 4*s_left);
            vel_val2 = 2*s_left;
            curr_left_pt = pos_val0 * left_bezier_origin + pos_val1*weighted_left_pt_0 + pos_val2*left_pt_1;
            curr_left_vel = vel_val0 * left_bezier_origin + vel_val1*weighted_left_pt_0 + vel_val2*left_pt_1;
            rotated_curr_left_vel = neg_rpi2 * curr_left_vel;
            left_inward_norm_vect = rotated_curr_left_vel / (rotated_curr_left_vel.norm() + eps);
            left_curve.row(i) = curr_left_pt;
            left_curve_vel.row(i) = curr_left_vel;
            left_curve_inward_norm.row(i) = left_inward_norm_vect;

            pos_val0 = (1 - s_right) * (1 - s_right);
            pos_val1 = 2*(1 - s_right)*s_right;
            pos_val2 = s_right*s_right;

            vel_val0 = (2*s_right - 2);
            vel_val1 = (2 - 4*s_right);
            vel_val2 = 2*s_right;
            curr_right_pt = pos_val0 * right_bezier_origin + pos_val1*weighted_right_pt_0 + pos_val2*right_pt_1;
            curr_right_vel = vel_val0 * right_bezier_origin + vel_val1*weighted_right_pt_0 + vel_val2*right_pt_1;
            rotated_curr_right_vel = rpi2 * curr_right_vel;
            right_inward_norm_vect = rotated_curr_right_vel / (rotated_curr_right_vel.norm() + eps);
            right_curve.row(i) = curr_right_pt;
            right_curve_vel.row(i) = curr_right_vel;
            right_curve_inward_norm.row(i) = right_inward_norm_vect;

            /*
            ROS_INFO_STREAM("left_pt: " << curr_left_pt[0] << ", " << curr_left_pt[1]);
            ROS_INFO_STREAM("left_vel " << i << ": " << curr_left_vel[0] << ", " << curr_left_vel[1]);
            ROS_INFO_STREAM("left_inward_norm: " << left_inward_norm_vect[0] << ", " << left_inward_norm_vect[1]);
            // ROS_INFO_STREAM("left_center: " << (curr_left_pt[0] - left_inward_norm[0]*offset) << ", " << (curr_left_pt[1] - left_inward_norm[1]*offset));
            ROS_INFO_STREAM("right_pt " << i << ": " << curr_right_pt[0] << ", " << curr_right_pt[1]);
            ROS_INFO_STREAM("right_vel " << i << ": " << curr_right_vel[0] << ", " << curr_right_vel[1]);
            ROS_INFO_STREAM("right_inward_norm " << i << ": " << right_inward_norm_vect[0] << ", " << right_inward_norm_vect[1]);
            // ROS_INFO_STREAM("curr_right_pt: " << curr_right_pt[0] << ", " << curr_right_pt[1]);
            */
        }
        
        // Eigen::Vector2d left_origin_inward_norm = left_curve_inward_norm.row(0);
        // Eigen::Vector2d right_origin_inward_norm = right_curve_inward_norm.row(0);

        // ROS_INFO_STREAM("left_origin_inward_norm: " << left_origin_inward_norm[0] << ", " << left_origin_inward_norm[1]);
        // ROS_INFO_STREAM("right_origin_inward_norm: " << right_origin_inward_norm[0] << ", " << right_origin_inward_norm[1]);
        // double beta_left_origin_inward_norm = std::atan2(left_origin_inward_norm[1], left_origin_inward_norm[0]);
        // double init_L_to_R_angle = getLeftToRightAngle(left_origin_inward_norm, right_origin_inward_norm);
        // double beta_origin_center = beta_left_origin_inward_norm - 0.5 * init_L_to_R_angle;
        // ROS_INFO_STREAM("init_L_to_R_angle: " << init_L_to_R_angle << ", beta_origin_center: " << beta_origin_center);
        // centered_origin_inward_norm << std::cos(beta_origin_center), std::sin(beta_origin_center);

        // ROS_INFO_STREAM("centered origin inward norm: " << centered_origin_inward_norm[0] << ", " << centered_origin_inward_norm[1]);
        all_curve_pts << left_curve, right_curve; // origin, 
        // ROS_INFO_STREAM("all_curve_pts worked");
        all_inward_norms << left_curve_inward_norm, right_curve_inward_norm; // centered_origin_inward_norm, 
        // ROS_INFO_STREAM("all_inward_norms worked");
        left_right_centers = all_curve_pts - all_inward_norms*offset;
        // ROS_INFO_STREAM("left_right_centers worked");
        // ROS_INFO_STREAM("left_curve points: " << left_curve);
        // ROS_INFO_STREAM("right_curve_points: " << right_curve);

        // ROS_INFO_STREAM("left_curve inward norms: " << left_curve_inward_norm);
        // ROS_INFO_STREAM("right_curve inward_norms: " << right_curve_inward_norm);

        // ROS_INFO_STREAM("all_inward_norms: " << all_inward_norms);
        // ROS_INFO_STREAM("all_centers: " << all_centers);

        Eigen::Matrix<double, 1, 2> goal; // (1, 2);
        goal << goal_pt_1[0], goal_pt_1[1];
        all_centers << goal, left_right_centers;
        // ROS_INFO_STREAM("all_centers worked");
    }

    // If i try to delete this DGap breaks
    [[deprecated("Use single trajectory generation")]]
    std::vector<geometry_msgs::PoseArray> GapTrajGenerator::generateTrajectory(std::vector<dynamic_gap::Gap> gapset) {
        std::vector<geometry_msgs::PoseArray> traj_set(gapset.size());
        return traj_set;
    }

    // Return in Odom frame (used for ctrl)
    geometry_msgs::PoseArray GapTrajGenerator::transformBackTrajectory(
        geometry_msgs::PoseArray posearr,
        geometry_msgs::TransformStamped planning2odom)
    {
        geometry_msgs::PoseArray retarr;
        geometry_msgs::PoseStamped outplaceholder;
        outplaceholder.header.frame_id = cfg_->odom_frame_id;
        geometry_msgs::PoseStamped inplaceholder;
        inplaceholder.header.frame_id = cfg_->robot_frame_id;
        for (const auto pose : posearr.poses)
        {
            inplaceholder.pose = pose;
            tf2::doTransform(inplaceholder, outplaceholder, planning2odom);
            retarr.poses.push_back(outplaceholder.pose);
        }
        retarr.header.frame_id = cfg_->odom_frame_id;
        retarr.header.stamp = ros::Time::now();
        // ROS_WARN_STREAM("leaving transform back with length: " << retarr.poses.size());
        return retarr;
    }

    std::tuple<geometry_msgs::PoseArray, std::vector<double>> GapTrajGenerator::forwardPassTrajectory(std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple)
    {
        geometry_msgs::PoseArray pose_arr = std::get<0>(return_tuple);
        std::vector<double> time_arr = std::get<1>(return_tuple);
        Eigen::Quaternionf q;
        geometry_msgs::Pose old_pose;
        old_pose.position.x = 0;
        old_pose.position.y = 0;
        old_pose.position.z = 0;
        old_pose.orientation.x = 0;
        old_pose.orientation.y = 0;
        old_pose.orientation.z = 0;
        old_pose.orientation.w = 1;
        geometry_msgs::Pose new_pose;
        double dx, dy, result;
        // std::cout << "entering at : " << pose_arr.poses.size() << std::endl;
        //std::cout << "starting pose: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << std::endl; 
        //std::cout << "final pose: " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << std::endl;
        /*
        if (pose_arr.poses.size() > 1) {
            double total_dx = pose_arr.poses[0].position.x - pose_arr.poses[pose_arr.poses.size() - 1].position.x;
            double total_dy = pose_arr.poses[0].position.y - pose_arr.poses[pose_arr.poses.size() - 1].position.y;
            double total_dist = sqrt(pow(total_dx, 2) + pow(total_dy, 2));
            ROS_WARN_STREAM("total distance: " << total_dist);
        }
        */
        std::vector<geometry_msgs::Pose> shortened;
        std::vector<double> shortened_time_arr;
        shortened.push_back(old_pose);
        shortened_time_arr.push_back(0.0);
        double threshold = 0.1;
        // ROS_INFO_STREAM("pose[0]: " << pose_arr.poses[0].position.x << ", " << pose_arr.poses[0].position.y);
        for (int i = 1; i < pose_arr.poses.size(); i++) {
            auto pose = pose_arr.poses[i];
            dx = pose.position.x - shortened.back().position.x;
            dy = pose.position.y - shortened.back().position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            if (result > threshold) {
                // ROS_INFO_STREAM("result " << i << " kept at " << result);
                shortened.push_back(pose);
                shortened_time_arr.push_back(time_arr[i]);

            } else {
                // ROS_INFO_STREAM("result " << i << " cut at " << result);
            }
        }
        // std::cout << "leaving at : " << shortened.size() << std::endl;
        pose_arr.poses = shortened;

        // Fix rotation
        for (int idx = 1; idx < pose_arr.poses.size(); idx++)
        {
            new_pose = pose_arr.poses[idx];
            old_pose = pose_arr.poses[idx - 1];
            dx = new_pose.position.x - old_pose.position.x;
            dy = new_pose.position.y - old_pose.position.y;
            result = std::atan2(dy, dx);
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(result, Eigen::Vector3f::UnitZ());
            q.normalize();
            pose_arr.poses[idx - 1].orientation.x = q.x();
            pose_arr.poses[idx - 1].orientation.y = q.y();
            pose_arr.poses[idx - 1].orientation.z = q.z();
            pose_arr.poses[idx - 1].orientation.w = q.w();
        }
        pose_arr.poses.pop_back();
        shortened_time_arr.pop_back();

        std::tuple<geometry_msgs::PoseArray, std::vector<double>> shortened_tuple(pose_arr, shortened_time_arr);
        return shortened_tuple;
    }

}