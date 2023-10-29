#include <dynamic_gap/trajectory_generation/GapTrajectoryGenerator.h>

namespace dynamic_gap 
{
    void GapTrajectoryGenerator::initializeSolver(OsqpEigen::Solver & solver, int Kplus1, 
                                                    const Eigen::MatrixXd & A) 
    {
        // ROS_INFO_STREAM("initializing solver");

        /*
         * HAVE TO KEEP THESE AS DOUBLES, OSQP EIGEN DOES NOT ACCEPT FLOATS
         */
        Eigen::SparseMatrix<double> hessian(Kplus1, Kplus1);

        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(Kplus1, 1);

        Eigen::SparseMatrix<double> linearMatrix(Kplus1, Kplus1);

        Eigen::VectorXd lowerBound = Eigen::MatrixXd::Zero(Kplus1, 1);

        Eigen::VectorXd upperBound = Eigen::MatrixXd::Zero(Kplus1, 1);

        for (int i = 0; i < Kplus1; i++) 
        {
            lowerBound(i, 0) = -OsqpEigen::INFTY;
            upperBound(i, 0) = -0.0000001; // this leads to non-zero weights. Closer to zero this number goes, closer to zero the weights go. This makes sense
        }

        for (int i = 0; i < Kplus1; i++) 
        {
            for (int j = 0; j < Kplus1; j++) 
            {
                if (i == j)
                    hessian.coeffRef(i, j) = 1.0;

                // need to transpose A vector, just doing here
                linearMatrix.coeffRef(i, j) = A.coeff(j, i);
            }
        }

        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);        
        solver.data()->setNumberOfVariables(Kplus1);
        solver.data()->setNumberOfConstraints(Kplus1);

        if (!solver.data()->setHessianMatrix(hessian)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET HESSIAN"); }

        if(!solver.data()->setGradient(gradient)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET GRADIENT"); }

        if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET LINEAR MATRIX"); }

        if(!solver.data()->setLowerBound(lowerBound)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET LOWER BOUND"); }

        if(!solver.data()->setUpperBound(upperBound)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET UPPER BOUND"); }

        if(!solver.initSolver()) { ROS_FATAL_STREAM("SOLVER FAILED TO INITIALIZE SOLVER");}
    }

    std::tuple<geometry_msgs::PoseArray, std::vector<float>> GapTrajectoryGenerator::generateTrajectory(
                                                    dynamic_gap::Gap& selectedGap, 
                                                    const geometry_msgs::PoseStamped & currPose, 
                                                    const geometry_msgs::TwistStamped & currVel,
                                                    bool runGoToGoal) 
    {
        if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("        [generateTrajectory()]");

        geometry_msgs::PoseArray path;
        std::vector<float> pathTiming;

        try 
        {        
            // return geometry_msgs::PoseArray();
            numCurvePts = cfg_->traj.num_curve_points;
            // ROS_INFO_STREAM("numCurvePts: " << numCurvePts);

            float gen_traj_start_time = ros::Time::now().toSec();
            path.header.stamp = ros::Time::now();
            TrajectoryLogger corder(path, cfg_->robot_frame_id, pathTiming);
            path.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;

            Eigen::Vector4f rbtState(currPose.pose.position.x + 1e-5, currPose.pose.position.y + 1e-6,
                                  0.0, 0.0); // currVel.linear.x, currVel.linear.y

            // get gap points in cartesian
            float xLeft, yLeft, xRight, yRight;
            selectedGap.getLCartesian(xLeft, yLeft);
            selectedGap.getRCartesian(xRight, yRight);

            // float ldist = selectedGap.cvxLeftDist();
            // float rdist = selectedGap.cvxRightDist();
            // float ltheta = idx2theta(selectedGap.cvxLeftIdx());
            // float rtheta = idx2theta(selectedGap.cvxRightIdx());
            // float xLeft = ldist * cos(ltheta);
            // float yLeft = ldist * sin(ltheta);
            // float xRight = rdist * cos(rtheta);
            // float yRight = rdist * sin(rtheta);

            float xLeftTerm, yLeftTerm, xRightTerm, yRightTerm;
            selectedGap.getLCartesian(xLeftTerm, yLeftTerm);
            selectedGap.getRCartesian(xRightTerm, yRightTerm);

            // ldist = selectedGap.cvxTermLDist();
            // rdist = selectedGap.cvxTermRDist();
            // ltheta = idx2theta(selectedGap.cvxTermLIdx());
            // rtheta = idx2theta(selectedGap.cvxTermRIdx());
            // float xLeftTerm = ldist * cos(ltheta);
            // float xLeftTerm = ldist * sin(ltheta);
            // float xRightTerm = rdist * cos(rtheta);
            // float yRightTerm = rdist * sin(rtheta);

            Eigen::Vector2f extendedGapOrigin = selectedGap.extendedGapOrigin_;

            Eigen::Vector2d initialGoal(selectedGap.goal.x_, selectedGap.goal.y_);
            Eigen::Vector2d terminalGoal(selectedGap.terminalGoal.x_, selectedGap.terminalGoal.y_);

            float initialGoalX = initialGoal[0];
            float initialGoalY = initialGoal[1];
            float terminalGoalX = terminalGoal[0];
            float terminalGoalY = terminalGoal[1];

            float goalVelX = (terminalGoalX - initialGoalX) / selectedGap.gapLifespan_; // absolute velocity (not relative to robot)
            float goalVelY = (terminalGoalY - initialGoalY) / selectedGap.gapLifespan_;

            if (cfg_->debug.traj_debug_log) 
            {
                ROS_INFO_STREAM("            actual initial robot pos: (" << rbtState[0] << ", " << rbtState[1] << ")");
                ROS_INFO_STREAM("            actual inital robot velocity: " << rbtState[2] << ", " << rbtState[3] << ")");
                ROS_INFO_STREAM("            actual initial left point: (" << xLeft << ", " << yLeft << "), actual initial right point: (" << xRight << ", " << yRight << ")"); 
                ROS_INFO_STREAM("            actual terminal left point: (" << xLeftTerm << ", " << xLeftTerm << "), actual terminal right point: (" << xRightTerm << ", " << yRightTerm << ")");
                ROS_INFO_STREAM("            actual initial goal: (" << initialGoalX << ", " << initialGoalY << ")"); 
                ROS_INFO_STREAM("            actual terminal goal: (" << terminalGoalX << ", " << terminalGoalY << ")"); 
            }
            
            float leftVelX = (xLeftTerm - xLeft) / selectedGap.gapLifespan_;
            float leftVelY = (xLeftTerm - yLeft) / selectedGap.gapLifespan_;

            float rightVelX = (xRightTerm - xRight) / selectedGap.gapLifespan_;
            float rightVelY = (yRightTerm - yRight) / selectedGap.gapLifespan_;

            state_type x = {rbtState[0], rbtState[1], xLeft, yLeft, xRight, yRight, initialGoalX, initialGoalY};
            
            // ROS_INFO_STREAM("pre-integration, x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);

            if (runGoToGoal) 
            { //   || selectedGap.goal.goalwithin
                state_type x = {rbtState[0], rbtState[1], 0.0, 0.0, 0.0, 0.0, selectedGap.goal.x_, selectedGap.goal.y_};
                // ROS_INFO_STREAM("Goal to Goal");
                GoToGoal goToGoal(selectedGap.goal.x_, selectedGap.goal.y_,
                             selectedGap.terminalGoal.x_, selectedGap.terminalGoal.y_,
                             selectedGap.gapLifespan_, cfg_->control.vx_absmax);
                boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                goToGoal, x, 0.0f, cfg_->traj.integrate_maxt, cfg_->traj.integrate_stept, corder);
                std::tuple<geometry_msgs::PoseArray, std::vector<float>> return_tuple(path, pathTiming);
                return return_tuple;
            }

            Eigen::Vector2d initRbtPos(x[0], x[1]);
            Eigen::Vector2d leftCurveInitPt(xLeft, yLeft);
            Eigen::Vector2d leftCurveTermPt(xLeftTerm, xLeftTerm);
            Eigen::Vector2d rightCurveInitPt(xRight, yRight);
            Eigen::Vector2d rightCurveTermPt(xRightTerm, yRightTerm);
            Eigen::Vector2d leftGapPtVel(leftVelX, leftVelY);
            Eigen::Vector2d rightGapPtVel(rightVelX, rightVelY);
            Eigen::Vector2d gapGoalTermPt(terminalGoalX, terminalGoalY);
            Eigen::Vector2d gapGoalVel(goalVelX, goalVelY);
            
            Eigen::Vector2d maxRbtVel(cfg_->control.vx_absmax, cfg_->control.vy_absmax);
            Eigen::Vector2d maxRbtAcc(cfg_->control.ax_absmax, cfg_->control.ay_absmax);
            Eigen::Vector2d gap_radial_extension(extendedGapOrigin[0], extendedGapOrigin[1]);
            Eigen::Vector2d leftBezierOrigin(selectedGap.leftBezierOrigin_[0],
                                               selectedGap.leftBezierOrigin_[1]);
            Eigen::Vector2d rightBezierOrigin(selectedGap.rightBezierOrigin_[0],
                                                selectedGap.rightBezierOrigin_[1]);

            Eigen::MatrixXd leftCurveVels, rightCurveVels, leftCurveInwardNorms, rightCurveInwardNorms,
                            leftCurvePosns, rightCurvePosns, gapCurvesPosns, gapCurvesInwardNorms, 
                            gapSideAHPFCenters, allAHPFCenters;
            
            float leftBezierWeight, rightBezierWeight;
            int numLeftRGEPoints, numRightRGEPoints;
            // THIS IS BUILT WITH EXTENDED POINTS. 
            double start_time = ros::Time::now().toSec();
            buildBezierCurve(selectedGap, leftCurvePosns, rightCurvePosns, gapCurvesPosns, 
                             leftCurveVels, rightCurveVels,
                             leftCurveInwardNorms, rightCurveInwardNorms, 
                             gapCurvesInwardNorms, gapSideAHPFCenters, allAHPFCenters,
                             leftGapPtVel, rightGapPtVel, maxRbtVel, 
                             leftCurveInitPt, leftCurveTermPt, rightCurveInitPt, rightCurveTermPt, 
                             gap_radial_extension, gapGoalTermPt, leftBezierWeight, rightBezierWeight, numCurvePts, 
                             numLeftRGEPoints, numRightRGEPoints,
                             initRbtPos, leftBezierOrigin, rightBezierOrigin);
            if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("            buildBezierCurve time taken: " << (ros::Time::now().toSec() - start_time));
            // ROS_INFO_STREAM("after buildBezierCurve, left weight: " << leftBezierWeight << ", rightBezierWeight: " << rightBezierWeight);
            selectedGap.leftWeight_ = leftBezierWeight;
            selectedGap.rightWeight_ = rightBezierWeight;
            selectedGap.leftRightCenters_ = gapSideAHPFCenters;
            selectedGap.allCurvePts_ = gapCurvesPosns;
            selectedGap.numLeftRGEPoints_ = numLeftRGEPoints;
            selectedGap.numRightRGEPoints_ = numRightRGEPoints;

            // add radial gap extension
            initialGoalX -= selectedGap.extendedGapOrigin_[0];
            initialGoalY -= selectedGap.extendedGapOrigin_[1];
            xLeft -= selectedGap.extendedGapOrigin_[0];
            yLeft -= selectedGap.extendedGapOrigin_[1];
            xRight -= selectedGap.extendedGapOrigin_[0];
            yRight -= selectedGap.extendedGapOrigin_[1];
            rbtState[0] -= selectedGap.extendedGapOrigin_[0];
            rbtState[1] -= selectedGap.extendedGapOrigin_[1];

            x = {rbtState[0], rbtState[1], xLeft, yLeft, xRight, yRight, initialGoalX, initialGoalY};

            /*
            PolarGapField polarGapField(xRight, xLeft, yRight, yLeft,
                                                    initialGoalX, initialGoalY,
                                                    selectedGap.mode.RGC_, selectedGap.isRadial(),
                                                    x[0], x[1],
                                                    cfg_->control.vx_absmax, cfg_->control.vx_absmax);
            */
            // float start_time = ros::Time::now().toSec();

            /*
            SETTING UP SOLVER
            */
            
            int N = gapCurvesPosns.rows();
            int Kplus1 = allAHPFCenters.rows();
            // ROS_INFO_STREAM("N: " << N << ", Kplus1: " << Kplus1);

            Eigen::MatrixXd A(Kplus1, Kplus1);
            double setConstraintMatrix_start_time = ros::WallTime::now().toSec();
            setConstraintMatrix(A, N, Kplus1, gapCurvesPosns, gapCurvesInwardNorms, allAHPFCenters);
            if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("            setConstraintMatrix time taken: " << ros::WallTime::now().toSec() - setConstraintMatrix_start_time);

            OsqpEigen::Solver solver;

            double initializeSolver_start_time = ros::WallTime::now().toSec();
            initializeSolver(solver, Kplus1, A);
            if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("            initializeSolver time taken: " << ros::WallTime::now().toSec() - initializeSolver_start_time);

            // ROS_INFO_STREAM("setConstraintMatrix time elapsed: " << (ros::Time::now().toSec() - start_time));
            // ROS_INFO_STREAM("A: " << A);
            
            // Eigen::MatrixXd b = Eigen::MatrixXd::Zero(Kplus1, 1);

            //ROS_INFO_STREAM("Hessian: " << hessian);
            //ROS_INFO_STREAM("Gradient: " << gradient);
            //ROS_INFO_STREAM("linearMatrix: " << linearMatrix);
            //ROS_INFO_STREAM("lowerBound: " << lowerBound);
            //ROS_INFO_STREAM("upperBound: " << upperBound);

            // if(!solver.setPrimalVariable(w_0)) return;
            
            // solve the QP problem
            double opt_start_time = ros::WallTime::now().toSec();
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
                ROS_FATAL_STREAM("SOLVER FAILED TO SOLVE PROBLEM");

            // get the controller input
            Eigen::MatrixXd weights = solver.getSolution();
            if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("            optimization time taken: " << (ros::WallTime::now().toSec() - opt_start_time));
            
            /*
            ROS_INFO_STREAM("current solution: "); 
            
            std::string weights_string;
            for (int i = 0; i < Kplus1; i++) {
                ROS_INFO_STREAM(weights.coeff(i, 0)); 
            } 
            */  

            
            AHPF ahpf(initRbtPos, gapGoalTermPt,
                        cfg_->control.vx_absmax, maxRbtAcc, allAHPFCenters, gapCurvesInwardNorms, weights,
                        leftGapPtVel, rightGapPtVel, gapGoalVel);   
            
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                    ahpf, x, 0.0f, selectedGap.gapLifespan_, 
                                                    cfg_->traj.integrate_stept, corder);
            start_time = ros::Time::now().toSec();
            /*
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                    polarGapField, x, 0.0f, selectedGap.gapLifespan_, 
                                                    cfg_->traj.integrate_stept, corder);
            for (auto & p : posearr.poses) 
            {
                p.position.x += selectedgap.extendedGapOrigin_[0];
                p.position.y += selectedgap.extendedGapOrigin_[1];
            }
            */
            if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("            integration time taken: " << (ros::Time::now().toSec() - start_time));

            std::tuple<geometry_msgs::PoseArray, std::vector<float>> return_tuple(path, pathTiming);
            if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("            generateTrajectory time taken: " << ros::Time::now().toSec() - gen_traj_start_time);
            return return_tuple;
            
        } catch (...) 
        {
            ROS_FATAL_STREAM("integrator");
            std::tuple<geometry_msgs::PoseArray, std::vector<float>> return_tuple(path, pathTiming);
            return return_tuple;
        }

    }

    void GapTrajectoryGenerator::setConstraintMatrix(Eigen::MatrixXd &A, int N, int Kplus1, 
                                                        const Eigen::MatrixXd & gapCurvesPosns, 
                                                        const Eigen::MatrixXd & gapCurvesInwardNorms,
                                                        const Eigen::MatrixXd & allAHPFCenters) 
    {
        Eigen::MatrixXd gradient_of_pti_wrt_centers(Kplus1, 2); // (2, Kplus1); // 
        Eigen::MatrixXd A_N(Kplus1, N);
        Eigen::MatrixXd A_S = Eigen::MatrixXd::Zero(Kplus1, 1);
        Eigen::MatrixXd neg_one_vect = Eigen::MatrixXd::Constant(1, 1, -1.0);

        float eps = 0.0000001;
        // allAHPFCenters size: (Kplus1 rows, 2 cols)
        Eigen::Vector2d boundary_pt_i, inward_norm_vector;
        Eigen::MatrixXd cent_to_boundary;
        Eigen::VectorXd rowwise_sq_norms;
        for (int i = 0; i < N; i++) 
        {
            boundary_pt_i = gapCurvesPosns.row(i);
            inward_norm_vector = gapCurvesInwardNorms.row(i);
            //Eigen::Matrix<double, 1, 2> inward_norm_vector = gapCurvesInwardNorms.row(i);        
            
            // doing boundary - allAHPFCenters
            cent_to_boundary = (-allAHPFCenters).rowwise() + boundary_pt_i.transpose();
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

    float num_int(Eigen::Vector2d pt_origin, 
                   Eigen::Vector2d pt_0, 
                   Eigen::Vector2d pt_1, 
                   float t_start, float t_end, float num_points) 
    {
        float interp_dist = 0.0;
        float steps = num_points - 1;

        // ROS_INFO_STREAM("performing numerical integration between " << t_start << " and " << t_end);
        float t_i, t_iplus1, dist;
        Eigen::Vector2d pt_i, pt_iplus1;
        for (int i = 0; i < steps; i++) {
            t_i = t_start + (t_end - t_start) * float(i / steps);
            t_iplus1 = t_start + (t_end - t_start) * float((i + 1) / steps);
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

    Eigen::VectorXd GapTrajectoryGenerator::arclength_sample_bezier(Eigen::Vector2d pt_origin, 
                                                              Eigen::Vector2d pt_0, 
                                                              Eigen::Vector2d pt_1, 
                                                              float numCurvePts,
                                                              float & des_dist_interval) 
    {
        // ROS_INFO_STREAM("running arclength sampling");
        // ROS_INFO_STREAM("pt_origin: " << pt_origin[0] << ", " << pt_origin[1]);
        // ROS_INFO_STREAM("pt_0: " << pt_0[0] << ", " << pt_0[1]);
        // ROS_INFO_STREAM("pt_1: " << pt_1[0] << ", " << pt_1[1]);        
        float total_approx_dist = num_int(pt_origin, pt_0, pt_1, 0.0, 1.0, numCurvePts);

        float t_kmin1 = 0.0;
        float num_interp_points = 5.0;
        Eigen::VectorXd uniform_indices = Eigen::MatrixXd::Zero(int(numCurvePts), 1);
        int uniform_index_entry = 1;
        float num_sampled_points = 2 * numCurvePts;
        des_dist_interval = total_approx_dist / (numCurvePts - 1);
        float dist_thresh = des_dist_interval / 100.0;

        // ROS_INFO_STREAM("number of points: " << numCurvePts);
        // ROS_INFO_STREAM("total distance: " << total_approx_dist);
        // ROS_INFO_STREAM("desired distance interval: " << des_dist_interval);
        float t_k, current_dist, t_prev, t_interp, interp_dist, t_high, t_low;
        for (float t_k = 0.0; t_k <= 1.0; t_k += (1.0 / num_sampled_points)) {
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

        uniform_indices(int(numCurvePts) - 1, 0) = 1.0;

        return uniform_indices;
        /*
        ROS_INFO_STREAM("uniform indices: ");
        for (int i = 0; i < int(numCurvePts); i++) {
            ROS_INFO_STREAM(uniform_indices(i, 0));
        }
        */
        // ROS_INFO_STREAM("total approx dist: " << total_approx_dist);
    }

    void GapTrajectoryGenerator::buildBezierCurve(dynamic_gap::Gap& selectedGap, 
                                            Eigen::MatrixXd & leftCurvePosns, Eigen::MatrixXd & rightCurvePosns, Eigen::MatrixXd & gapCurvesPosns,
                                            Eigen::MatrixXd & leftCurveVels, Eigen::MatrixXd & rightCurveVels,
                                            Eigen::MatrixXd & leftCurveInwardNorms, Eigen::MatrixXd & rightCurveInwardNorms, 
                                            Eigen::MatrixXd & gapCurvesInwardNorms, Eigen::MatrixXd & gapSideAHPFCenters, Eigen::MatrixXd & allAHPFCenters,
                                            Eigen::Vector2d leftGapPtVel, Eigen::Vector2d rightGapPtVel, Eigen::Vector2d maxRbtVel,
                                            Eigen::Vector2d leftCurveInitPt, Eigen::Vector2d leftCurveTermPt, Eigen::Vector2d rightCurveInitPt, Eigen::Vector2d rightCurveTermPt, 
                                            Eigen::Vector2d gap_radial_extension, Eigen::Vector2d gapGoalTermPt, float & leftBezierWeight, float & rightBezierWeight, 
                                            float numCurvePts, 
                                            int & numLeftRGEPoints, int & numRightRGEPoints,
                                            Eigen::Vector2d initRbtPos,
                                            Eigen::Vector2d leftBezierOrigin_, Eigen::Vector2d rightBezierOrigin) 
    {  
        // ROS_INFO_STREAM("building bezier curve");
       
        leftBezierWeight = leftGapPtVel.norm() / maxRbtVel.norm(); // capped at 1, we can scale down towards 0 until initial constraints are met?
        rightBezierWeight = rightGapPtVel.norm() / maxRbtVel.norm();

        // for a totally static gap, can get no velocity on first bezier curve point which corrupts vector field
        Eigen::Vector2d weighted_leftCurveInitPt, weighted_rightCurveInitPt;
        if (leftGapPtVel.norm() > 0.0)
            weighted_leftCurveInitPt = leftBezierOrigin_ + leftBezierWeight * (leftCurveInitPt - leftBezierOrigin_);
        else
            weighted_leftCurveInitPt = (0.95 * leftBezierOrigin_ + 0.05 * leftCurveTermPt);

        if (rightGapPtVel.norm() >  0.0)
            weighted_rightCurveInitPt = rightBezierOrigin + rightBezierWeight * (rightCurveInitPt - rightBezierOrigin);
        else
            weighted_rightCurveInitPt = (0.95 * rightBezierOrigin + 0.05 * rightCurveTermPt);
        
        selectedGap.leftPt0_ = weighted_leftCurveInitPt;
        selectedGap.leftPt1_ = leftCurveTermPt;
        selectedGap.rightPt0_ = weighted_rightCurveInitPt;
        selectedGap.rightPt1_ = rightCurveTermPt;  

        float pos_val0, pos_val1, pos_val2, vel_val0, vel_val1, vel_val2;
        Eigen::Vector2d curr_left_pt, curr_left_vel, left_inward_vect, rotated_curr_left_vel, left_inward_norm_vect,
                        curr_right_pt, curr_right_vel, right_inward_vect, rotated_curr_right_vel, right_inward_norm_vect;
        Eigen::Matrix2d rpi2, neg_rpi2;
        float rot_val = M_PI/2;
        float s, s_left, s_right;
        rpi2 << std::cos(rot_val), -std::sin(rot_val), std::sin(rot_val), std::cos(rot_val);
        neg_rpi2 << std::cos(-rot_val), -std::sin(-rot_val), std::sin(-rot_val), std::cos(-rot_val);

        
        float des_left_dist = 0.01;        
        Eigen::VectorXd left_indices = arclength_sample_bezier(leftBezierOrigin_, weighted_leftCurveInitPt, leftCurveTermPt, numCurvePts, des_left_dist);
        numLeftRGEPoints = (cfg_->gap_manip.radial_extend) ? std::max(int(std::ceil( (gap_radial_extension - leftBezierOrigin_).norm() / des_left_dist)), 2) : 0;

        int total_num_left_curve_points = numLeftRGEPoints + numCurvePts; 
        leftCurvePosns = Eigen::MatrixXd(total_num_left_curve_points, 2);
        leftCurveVels = Eigen::MatrixXd(total_num_left_curve_points, 2);
        leftCurveInwardNorms = Eigen::MatrixXd(total_num_left_curve_points, 2);

        // ROS_INFO_STREAM("true left RGE points: " << numLeftRGEPoints);
        for (float i = 0; i < numLeftRGEPoints; i++) 
        {
            s = i / numLeftRGEPoints;
            // ROS_INFO_STREAM("s_left_rge: " << s);
            pos_val0 = (1 - s);
            pos_val1 = s;
            curr_left_pt = pos_val0 * gap_radial_extension + pos_val1 * leftBezierOrigin_;
            curr_left_vel = (leftBezierOrigin_ - gap_radial_extension);
            left_inward_vect = neg_rpi2 * curr_left_vel;
            left_inward_norm_vect = left_inward_vect / left_inward_vect.norm();
            leftCurvePosns.row(i) = curr_left_pt;
            leftCurveVels.row(i) = curr_left_vel;
            leftCurveInwardNorms.row(i) = left_inward_norm_vect;
        }

        float des_right_dist = 0.01;
        Eigen::VectorXd right_indices = arclength_sample_bezier(rightBezierOrigin, weighted_rightCurveInitPt, rightCurveTermPt, numCurvePts, des_right_dist);
        numRightRGEPoints = (cfg_->gap_manip.radial_extend) ? std::max(int(std::ceil( (gap_radial_extension - rightBezierOrigin).norm() / des_right_dist)), 2) : 0;

        int total_num_right_curve_points = numRightRGEPoints + numCurvePts; 
        rightCurvePosns = Eigen::MatrixXd(total_num_right_curve_points, 2);            
        rightCurveVels = Eigen::MatrixXd(total_num_right_curve_points, 2);
        rightCurveInwardNorms = Eigen::MatrixXd(total_num_right_curve_points, 2);
        
        // ROS_INFO_STREAM("true right RGE points: " << numRightRGEPoints);
        for (float i = 0; i < numRightRGEPoints; i++) {
            s = i / numRightRGEPoints;
            // ROS_INFO_STREAM("s_right_rge: " << s);
            pos_val0 = (1 - s);
            pos_val1 = s;
            curr_right_pt = pos_val0 * gap_radial_extension + pos_val1 * rightBezierOrigin;
            curr_right_vel = (rightBezierOrigin - gap_radial_extension);
            right_inward_vect = rpi2 * curr_right_vel;
            right_inward_norm_vect = right_inward_vect / right_inward_vect.norm();
            rightCurvePosns.row(i) = curr_right_pt;
            rightCurveVels.row(i) = curr_right_vel;
            rightCurveInwardNorms.row(i) = right_inward_norm_vect;
        }          

        gapCurvesPosns = Eigen::MatrixXd(total_num_left_curve_points + total_num_right_curve_points, 2);
        gapCurvesInwardNorms = Eigen::MatrixXd(total_num_left_curve_points + total_num_right_curve_points, 2);
        gapSideAHPFCenters = Eigen::MatrixXd(total_num_left_curve_points + total_num_right_curve_points, 2);
        allAHPFCenters = Eigen::MatrixXd(total_num_left_curve_points + total_num_right_curve_points + 1, 2);
             
        /*
        ROS_INFO_STREAM("gap_radial_extension: " << gap_radial_extension[0] << ", " << gap_radial_extension[1]);
        ROS_INFO_STREAM("initRbtPos: " << initRbtPos[0] << ", " << initRbtPos[1]);

        ROS_INFO_STREAM("leftBezierOrigin_: " << leftBezierOrigin_[0] << ", " << leftBezierOrigin_[1]);
        ROS_INFO_STREAM("leftCurveInitPt: " << leftCurveInitPt[0] << ", " << leftCurveInitPt[1]);
        ROS_INFO_STREAM("weighted_leftCurveInitPt: " << weighted_leftCurveInitPt[0] << ", " << weighted_leftCurveInitPt[1]);       
        ROS_INFO_STREAM("leftCurveTermPt: " << leftCurveTermPt[0] << ", " << leftCurveTermPt[1]);
        ROS_INFO_STREAM("left_vel: " << leftGapPtVel[0] << ", " << leftGapPtVel[1]);

        ROS_INFO_STREAM("rightBezierOrigin: " << rightBezierOrigin[0] << ", " << rightBezierOrigin[1]);
        ROS_INFO_STREAM("rightCurveInitPt: " << rightCurveInitPt[0] << ", " << rightCurveInitPt[1]);
        ROS_INFO_STREAM("weighted_rightCurveInitPt: " << weighted_rightCurveInitPt[0] << ", " << weighted_rightCurveInitPt[1]);        
        ROS_INFO_STREAM("rightCurveTermPt: " << rightCurveTermPt[0] << ", " << rightCurveTermPt[1]);
        ROS_INFO_STREAM("right_vel: " << rightGapPtVel[0] << ", " << rightGapPtVel[1]);

        ROS_INFO_STREAM("maxRbtVel: " << maxRbtVel[0] << ", " << maxRbtVel[1]);
        */

        // ROS_INFO_STREAM("radial extensions: ");
        // ADDING DISCRETE POINTS FOR RADIAL GAP EXTENSION

        float eps = 0.0000001;

        // model gives: left_pt - rbt.
        // populating the quadratic weighted bezier

        Eigen::Matrix<double, 1, 2> origin, centered_origin_inward_norm;
        origin << 0.0, 0.0;
        float offset = 0.01; // (des_left_dist + des_right_dist) / 2.0;
        // ROS_INFO_STREAM("offset: " << offset);

        int counter = 0;
        for (float i = numLeftRGEPoints; i < total_num_left_curve_points; i++) {
            
            s_left = left_indices(counter, 0);
            counter++;

            // ROS_INFO_STREAM("s_left: " << s_left);
            pos_val0 = (1 - s_left) * (1 - s_left);
            pos_val1 = 2*(1 - s_left)*s_left;
            pos_val2 = s_left*s_left;

            vel_val0 = (2*s_left - 2);
            vel_val1 = (2 - 4*s_left);
            vel_val2 = 2*s_left;
            curr_left_pt = pos_val0 * leftBezierOrigin_ + pos_val1*weighted_leftCurveInitPt + pos_val2*leftCurveTermPt;
            curr_left_vel = vel_val0 * leftBezierOrigin_ + vel_val1*weighted_leftCurveInitPt + vel_val2*leftCurveTermPt;
            rotated_curr_left_vel = neg_rpi2 * curr_left_vel;
            left_inward_norm_vect = rotated_curr_left_vel / (rotated_curr_left_vel.norm() + eps);
            leftCurvePosns.row(i) = curr_left_pt;
            leftCurveVels.row(i) = curr_left_vel;
            leftCurveInwardNorms.row(i) = left_inward_norm_vect;
        }

        counter = 0;
        for (float i = numRightRGEPoints; i < total_num_right_curve_points; i++) {

            s_right = right_indices(counter, 0);
            counter++;

            // ROS_INFO_STREAM("s_right: " << s_right);

            pos_val0 = (1 - s_right) * (1 - s_right);
            pos_val1 = 2*(1 - s_right)*s_right;
            pos_val2 = s_right*s_right;

            vel_val0 = (2*s_right - 2);
            vel_val1 = (2 - 4*s_right);
            vel_val2 = 2*s_right;
            curr_right_pt = pos_val0 * rightBezierOrigin + pos_val1*weighted_rightCurveInitPt + pos_val2*rightCurveTermPt;
            curr_right_vel = vel_val0 * rightBezierOrigin + vel_val1*weighted_rightCurveInitPt + vel_val2*rightCurveTermPt;
            rotated_curr_right_vel = rpi2 * curr_right_vel;
            right_inward_norm_vect = rotated_curr_right_vel / (rotated_curr_right_vel.norm() + eps);
            rightCurvePosns.row(i) = curr_right_pt;
            rightCurveVels.row(i) = curr_right_vel;
            rightCurveInwardNorms.row(i) = right_inward_norm_vect;

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
        
        // Eigen::Vector2d left_origin_inward_norm = leftCurveInwardNorms.row(0);
        // Eigen::Vector2d right_origin_inward_norm = rightCurveInwardNorms.row(0);

        // ROS_INFO_STREAM("left_origin_inward_norm: " << left_origin_inward_norm[0] << ", " << left_origin_inward_norm[1]);
        // ROS_INFO_STREAM("right_origin_inward_norm: " << right_origin_inward_norm[0] << ", " << right_origin_inward_norm[1]);
        // double thetaLeft_origin_inward_norm = std::atan2(left_origin_inward_norm[1], left_origin_inward_norm[0]);
        // double init_leftToRightAngle = getLeftToRightAngle(left_origin_inward_norm, right_origin_inward_norm);
        // double beta_origin_center = thetaLeft_origin_inward_norm - 0.5 * init_leftToRightAngle;
        // ROS_INFO_STREAM("init_leftToRightAngle: " << init_leftToRightAngle << ", beta_origin_center: " << beta_origin_center);
        // centered_origin_inward_norm << std::cos(beta_origin_center), std::sin(beta_origin_center);

        // ROS_INFO_STREAM("centered origin inward norm: " << centered_origin_inward_norm[0] << ", " << centered_origin_inward_norm[1]);
        gapCurvesPosns << leftCurvePosns, rightCurvePosns; // origin, 
        // ROS_INFO_STREAM("gapCurvesPosns worked");
        gapCurvesInwardNorms << leftCurveInwardNorms, rightCurveInwardNorms; // centered_origin_inward_norm, 
        // ROS_INFO_STREAM("gapCurvesInwardNorms worked");
        gapSideAHPFCenters = gapCurvesPosns - gapCurvesInwardNorms*offset;
        // ROS_INFO_STREAM("gapSideAHPFCenters worked");
        // ROS_INFO_STREAM("left_curve points: " << left_curve);
        // ROS_INFO_STREAM("right_curve_points: " << right_curve);

        // ROS_INFO_STREAM("left_curve inward norms: " << leftCurveInwardNorms);
        // ROS_INFO_STREAM("right_curve inward_norms: " << rightCurveInwardNorms);

        // ROS_INFO_STREAM("gapCurvesInwardNorms: " << gapCurvesInwardNorms);
        // ROS_INFO_STREAM("allAHPFCenters: " << allAHPFCenters);

        Eigen::Matrix<double, 1, 2> goal; // (1, 2);
        goal << gapGoalTermPt[0], gapGoalTermPt[1];
        allAHPFCenters << goal, gapSideAHPFCenters;
        // ROS_INFO_STREAM("allAHPFCenters worked");
    }

    // If i try to delete this DGap breaks
    // [[deprecated("Use single trajectory generation")]]
    // std::vector<geometry_msgs::PoseArray> GapTrajectoryGenerator::generateTrajectory(std::vector<dynamic_gap::Gap> gapset) 
    // {
    //     std::vector<geometry_msgs::PoseArray> traj_set(gapset.size());
    //     return traj_set;
    // }

    // Return in Odom frame (used for ctrl)
    geometry_msgs::PoseArray GapTrajectoryGenerator::transformLocalTrajectoryToOdomFrame(const geometry_msgs::PoseArray & path,
                                                                             const geometry_msgs::TransformStamped & planning2odom)
    {
        geometry_msgs::PoseArray retarr;
        geometry_msgs::PoseStamped outplaceholder;
        outplaceholder.header.frame_id = cfg_->odom_frame_id;
        geometry_msgs::PoseStamped inplaceholder;
        inplaceholder.header.frame_id = cfg_->robot_frame_id;
        for (const auto pose : path.poses)
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

    std::tuple<geometry_msgs::PoseArray, std::vector<float>> GapTrajectoryGenerator::processTrajectory(const std::tuple<geometry_msgs::PoseArray, std::vector<float>> & return_tuple)
    {
        geometry_msgs::PoseArray pose_arr = std::get<0>(return_tuple);
        std::vector<float> time_arr = std::get<1>(return_tuple);
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
        float dx, dy, result;
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
        std::vector<float> shortened_time_arr;
        shortened.push_back(old_pose);
        shortened_time_arr.push_back(0.0);
        float threshold = 0.1;
        // ROS_INFO_STREAM("pose[0]: " << pose_arr.poses[0].position.x << ", " << pose_arr.poses[0].position.y);
        for (int i = 1; i < pose_arr.poses.size(); i++) 
        {
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

        std::tuple<geometry_msgs::PoseArray, std::vector<float>> shortened_tuple(pose_arr, shortened_time_arr);
        return shortened_tuple;
    }

}