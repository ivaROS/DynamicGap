#include <dynamic_gap/trajectory_generation/GapTrajectoryGenerator.h>

namespace dynamic_gap 
{
    void GapTrajectoryGenerator::initializeSolver(OsqpEigen::Solver & solver, int Kplus1, 
                                                    const Eigen::MatrixXd & A) 
    {
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "initializing solver");

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

    std::tuple<geometry_msgs::PoseArray, std::vector<float>> GapTrajectoryGenerator::generateTrajectory(dynamic_gap::Gap& selectedGap, 
                                                                                                        const geometry_msgs::PoseStamped & currPose, 
                                                                                                        const geometry_msgs::TwistStamped & currVel,
                                                                                                        bool runGoToGoal) 
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "        [generateTrajectory()]");

        geometry_msgs::PoseArray path;
        std::vector<float> pathTiming;

        try 
        {        
            // return geometry_msgs::PoseArray();
            int numCurvePts = cfg_->traj.num_curve_points;
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "numCurvePts: " << numCurvePts);

		    std::chrono::steady_clock::time_point generateTrajectoryStartTime = std::chrono::steady_clock::now();

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

            // ldist = selectedGap.cvxTermLeftDist();
            // rdist = selectedGap.cvxTermRightDist();
            // ltheta = idx2theta(selectedGap.cvxTermLeftIdx());
            // rtheta = idx2theta(selectedGap.cvxTermRightIdx());
            // float xLeftTerm = ldist * cos(ltheta);
            // float yLeftTerm = ldist * sin(ltheta);
            // float xRightTerm = rdist * cos(rtheta);
            // float yRightTerm = rdist * sin(rtheta);

            // Eigen::Vector2f extendedGapOrigin = selectedGap.extendedGapOrigin_;

            Eigen::Vector2d initialGoal(selectedGap.goal.x_, selectedGap.goal.y_);
            Eigen::Vector2d terminalGoal(selectedGap.terminalGoal.x_, selectedGap.terminalGoal.y_);

            float initialGoalX = initialGoal[0];
            float initialGoalY = initialGoal[1];
            float terminalGoalX = terminalGoal[0];
            float terminalGoalY = terminalGoal[1];

            float goalVelX = (terminalGoalX - initialGoalX) / selectedGap.gapLifespan_; // absolute velocity (not relative to robot)
            float goalVelY = (terminalGoalY - initialGoalY) / selectedGap.gapLifespan_;

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual initial robot pos: (" << rbtState[0] << ", " << rbtState[1] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual inital robot velocity: " << rbtState[2] << ", " << rbtState[3] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual initial left point: (" << xLeft << ", " << yLeft << "), actual initial right point: (" << xRight << ", " << yRight << ")"); 
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual terminal left point: (" << xLeftTerm << ", " << yLeftTerm << "), actual terminal right point: (" << xRightTerm << ", " << yRightTerm << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual initial goal: (" << initialGoalX << ", " << initialGoalY << ")"); 
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual terminal goal: (" << terminalGoalX << ", " << terminalGoalY << ")"); 
            
            float leftVelX = (xLeftTerm - xLeft) / selectedGap.gapLifespan_;
            float leftVelY = (yLeftTerm - yLeft) / selectedGap.gapLifespan_;

            float rightVelX = (xRightTerm - xRight) / selectedGap.gapLifespan_;
            float rightVelY = (yRightTerm - yRight) / selectedGap.gapLifespan_;

            state_type x = {rbtState[0], rbtState[1], xLeft, yLeft, xRight, yRight, initialGoalX, initialGoalY};
            
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pre-integration, x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);

            if (runGoToGoal) 
            { //   || selectedGap.goal.goalwithin
                state_type x = {rbtState[0], rbtState[1], 0.0, 0.0, 0.0, 0.0, selectedGap.goal.x_, selectedGap.goal.y_};
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal to Goal");
                GoToGoal goToGoal(selectedGap.goal.x_, selectedGap.goal.y_,
                             selectedGap.terminalGoal.x_, selectedGap.terminalGoal.y_,
                             selectedGap.gapLifespan_, cfg_->control.vx_absmax);
                boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                goToGoal, x, 0.0f, cfg_->traj.integrate_maxt, cfg_->traj.integrate_stept, corder);
                std::tuple<geometry_msgs::PoseArray, std::vector<float>> traj(path, pathTiming);
                float generateTrajectoryTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - generateTrajectoryStartTime).count() / 1.0e6;
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            generateTrajectory (g2g) time taken: " << generateTrajectoryTime << " seconds");                
                return traj;
            }

            Eigen::Vector2d initRbtPos(x[0], x[1]);
            Eigen::Vector2d leftCurveInitPt(xLeft, yLeft);
            Eigen::Vector2d leftCurveTermPt(xLeftTerm, yLeftTerm);
            Eigen::Vector2d rightCurveInitPt(xRight, yRight);
            Eigen::Vector2d rightCurveTermPt(xRightTerm, yRightTerm);
            Eigen::Vector2d leftGapPtVel(leftVelX, leftVelY);
            Eigen::Vector2d rightGapPtVel(rightVelX, rightVelY);
            Eigen::Vector2d gapGoalTermPt(terminalGoalX, terminalGoalY);
            Eigen::Vector2d gapGoalVel(goalVelX, goalVelY);
            
            Eigen::Vector2d maxRbtVel(cfg_->control.vx_absmax, cfg_->control.vy_absmax);
            Eigen::Vector2d maxRbtAcc(cfg_->control.ax_absmax, cfg_->control.ay_absmax);

            Eigen::MatrixXd gapCurvesPosns, gapCurvesInwardNorms, 
                            gapSideAHPFCenters, allAHPFCenters;
            
            float leftBezierWeight, rightBezierWeight;
            int numLeftRGEPoints, numRightRGEPoints;
            // THIS IS BUILT WITH EXTENDED POINTS. 
		    std::chrono::steady_clock::time_point buildExtendedBezierCurveStartTime = std::chrono::steady_clock::now();
            buildExtendedBezierCurve(selectedGap, gapCurvesPosns, 
                                        gapCurvesInwardNorms, gapSideAHPFCenters, allAHPFCenters,
                                        leftGapPtVel, rightGapPtVel, maxRbtVel, 
                                        leftCurveInitPt, leftCurveTermPt, rightCurveInitPt, rightCurveTermPt, 
                                        gapGoalTermPt, leftBezierWeight, rightBezierWeight, numCurvePts, 
                                        numLeftRGEPoints, numRightRGEPoints,
                                        initRbtPos);
            float buildExtendedBezierCurveTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - buildExtendedBezierCurveStartTime).count() / 1.0e6;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            buildExtendedBezierCurve time taken: " << buildExtendedBezierCurveTime << " seconds");

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "after buildExtendedBezierCurve, left weight: " << leftBezierWeight << ", rightBezierWeight: " << rightBezierWeight);
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

            /*
            SETTING UP SOLVER
            */
            
            int N = gapCurvesPosns.rows();
            int Kplus1 = allAHPFCenters.rows();
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "N: " << N << ", Kplus1: " << Kplus1);

            Eigen::MatrixXd A(Kplus1, Kplus1);
            // double setConstraintMatrix_start_time = ros::WallTime::now().toSec();
            setConstraintMatrix(A, N, Kplus1, gapCurvesPosns, gapCurvesInwardNorms, allAHPFCenters);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            setConstraintMatrix time taken: " << ros::WallTime::now().toSec() - setConstraintMatrix_start_time);

            OsqpEigen::Solver solver;
            double timeLimit = 0.5; // seconds
            solver.settings()->setTimeLimit(timeLimit);

            // double initializeSolver_start_time = ros::WallTime::now().toSec();
            initializeSolver(solver, Kplus1, A);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            initializeSolver time taken: " << ros::WallTime::now().toSec() - initializeSolver_start_time);

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "A: " << A);
            
            // Eigen::MatrixXd b = Eigen::MatrixXd::Zero(Kplus1, 1);

            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Hessian: " << hessian);
            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Gradient: " << gradient);
            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "linearMatrix: " << linearMatrix);
            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "lowerBound: " << lowerBound);
            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "upperBound: " << upperBound);

            // if(!solver.setPrimalVariable(w_0)) return;
            
            // solve the QP problem
		    std::chrono::steady_clock::time_point solveProblemStartTime = std::chrono::steady_clock::now();
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "SOLVER FAILED TO SOLVE PROBLEM");
                std::tuple<geometry_msgs::PoseArray, std::vector<float>> traj(path, pathTiming);
                return traj;
            }
            float solveProblemTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - solveProblemStartTime).count() / 1.0e6;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            solveProblem time taken: " << solveProblemTime << " seconds");

            // get the controller input
            Eigen::MatrixXd weights = solver.getSolution();
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            optimization time taken: " << (ros::WallTime::now().toSec() - optStart_time));
            
            /*
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "current solution: "); 
            
            std::string weights_string;
            for (int i = 0; i < Kplus1; i++) {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", weights.coeff(i, 0)); 
            } 
            */  


            AHPF ahpf(initRbtPos, gapGoalTermPt,
                        cfg_->control.vx_absmax, maxRbtAcc, allAHPFCenters, gapCurvesInwardNorms, weights,
                        leftGapPtVel, rightGapPtVel, gapGoalVel);   

            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                    ahpf, x, 0.0f, selectedGap.gapLifespan_, 
                                                    cfg_->traj.integrate_stept, corder);
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

            std::tuple<geometry_msgs::PoseArray, std::vector<float>> traj(path, pathTiming);
            float generateTrajectoryTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - generateTrajectoryStartTime).count() / 1.0e6;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            generateTrajectory (ahpf) time taken: " << generateTrajectoryTime << " seconds");
            return traj;
            
        } catch (...) 
        {
            ROS_FATAL_STREAM("integrator");
            std::tuple<geometry_msgs::PoseArray, std::vector<float>> traj(path, pathTiming);
            return traj;
        }

    }

    void GapTrajectoryGenerator::setConstraintMatrix(Eigen::MatrixXd &A, int N, int Kplus1, 
                                                        const Eigen::MatrixXd & gapCurvesPosns, 
                                                        const Eigen::MatrixXd & gapCurvesInwardNorms,
                                                        const Eigen::MatrixXd & allAHPFCenters) 
    {
        Eigen::MatrixXd dIthGradientPtdAhpfCenters(Kplus1, 2); // (2, Kplus1); // 
        Eigen::MatrixXd ABoundaryPts(Kplus1, N);
        Eigen::MatrixXd ANegativeOne = Eigen::MatrixXd::Zero(Kplus1, 1);
        // Eigen::MatrixXd negativeOne = 

        float eps = 0.0000001;
        // allAHPFCenters size: (Kplus1 rows, 2 cols)
        Eigen::Vector2d ithBoundaryPt, ithBoundaryInwardNorm;
        Eigen::MatrixXd ahpfCentersToBoundaryPt;
        Eigen::VectorXd rowWiseSquaredNorms;
        for (int i = 0; i < N; i++) 
        {
            ithBoundaryPt = gapCurvesPosns.row(i);
            ithBoundaryInwardNorm = gapCurvesInwardNorms.row(i);
            //Eigen::Matrix<double, 1, 2> ithBoundaryInwardNorm = gapCurvesInwardNorms.row(i);        
            
            // doing boundary - allAHPFCenters
            ahpfCentersToBoundaryPt = (-allAHPFCenters).rowwise() + ithBoundaryPt.transpose(); // must be in this order for matrices to work
            // need to divide rowwise by norm of each row
            rowWiseSquaredNorms = ahpfCentersToBoundaryPt.rowwise().squaredNorm();
            dIthGradientPtdAhpfCenters = ahpfCentersToBoundaryPt.array().colwise() / (rowWiseSquaredNorms.array() + eps);
            
            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "ithBoundaryInwardNorm: " << ithBoundaryInwardNorm);
            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "gradient_of_pti: " << dIthGradientPtdAhpfCenters);

            ABoundaryPts.col(i) = dIthGradientPtdAhpfCenters * ithBoundaryInwardNorm; // A_pi;

            // ABoundaryPts.col(i) = dIthGradientPtdAhpfCenters * ithBoundaryInwardNorm;
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "ithBoundaryInwardNorm size: " << ithBoundaryInwardNorm.rows() << ", " << ithBoundaryInwardNorm.cols());
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "dIthGradientPtdAhpfCenters size: " << dIthGradientPtdAhpfCenters.rows() << ", " << dIthGradientPtdAhpfCenters.cols());
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "A_pi size: " << A_pi.rows() << ", " << A_pi.cols());

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "A_pi: " << A_pi);
            // dotting inward norm (Kplus1 rows, 2 columns) with gradient of pti (Kplus1 rows, 2 columns) to get (Kplus1 rows, 1 column)
        }

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "ABoundaryPts: " << ABoundaryPts);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "ABoundaryPts_new: " << ABoundaryPts_new);
            
        ANegativeOne.row(0) = Eigen::MatrixXd::Constant(1, 1, -1.0); // -1

        A << ABoundaryPts, ANegativeOne;
    }

    float GapTrajectoryGenerator::calculateBezierArclengthDistance(const Eigen::Vector2d & bezierPt0, 
                                                                    const Eigen::Vector2d & bezierPt1, 
                                                                    const Eigen::Vector2d & bezierPt2, 
                                                                    float tStart, float tEnd, float numPoints) 
    {
        float arclengthDistance = 0.0;
        float steps = numPoints - 1;

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "performing numerical integration between " << tStart << " and " << tEnd);
        float t_i, t_iplus1, bezierPtToPtDist;
        Eigen::Vector2d bezierPt_i, bezierPt_iplus1;

        // Integrating along a Bezier curve from tStart to tEnd with numPoints to get an arclength distance
        for (int i = 0; i < steps; i++) 
        {
            t_i = tStart + (tEnd - tStart) * (i / steps);
            t_iplus1 = tStart + (tEnd - tStart) * ((i + 1) / steps);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "t_i: " << t_i << ", t_iplus1: " << t_iplus1);

            // applying Bezier equation
            bezierPt_i = (1 - t_i)*(1 - t_i)*bezierPt0 + 2*(1 - t_i)*t_i*bezierPt1 + (t_i)*(t_i)*bezierPt2;
            bezierPt_iplus1 = (1 - t_iplus1)*(1 - t_iplus1)*bezierPt0 + 2*(1 - t_iplus1)*t_iplus1*bezierPt1 + (t_iplus1)*(t_iplus1)*bezierPt2;
            
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "bezierPt_i: " << bezierPt_i[0] << ", " << bezierPt_i[1] << ", bezierPt_iplus1: " << bezierPt_iplus1[0] << ", " << bezierPt_iplus1[1]);
            bezierPtToPtDist = (bezierPt_iplus1 - bezierPt_i).norm();
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "adding dist:" << dist);
            arclengthDistance += bezierPtToPtDist;
        }

        return arclengthDistance;
    }

    Eigen::VectorXd GapTrajectoryGenerator::arclengthParameterizeBezier(const Eigen::Vector2d & bezierPt0, 
                                                                        const Eigen::Vector2d & bezierPt1, 
                                                                        const Eigen::Vector2d & bezierPt2, 
                                                                        float numCurvePts,
                                                                        float & desiredBezierPtToPtDistance) 
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "               [arclengthParameterizeBezier()]");
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "bezierPt0: " << bezierPt0[0] << ", " << bezierPt0[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "bezierPt1: " << bezierPt1[0] << ", " << bezierPt1[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "bezierPt2: " << bezierPt2[0] << ", " << bezierPt2[1]);        
             // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "running arclength sampling");
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "bezierPt0: " << bezierPt0[0] << ", " << bezierPt0[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "bezierPt1: " << bezierPt1[0] << ", " << bezierPt1[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "bezierPt2: " << bezierPt2[0] << ", " << bezierPt2[1]);        
        float bezierArclengthDistance = calculateBezierArclengthDistance(bezierPt0, bezierPt1, bezierPt2, 0.0, 1.0, numCurvePts);

        float numBezierPtToPtIntegrationPoints = 5.0;
        Eigen::VectorXd arclengthParameterization = Eigen::MatrixXd::Zero(int(numCurvePts), 1);
        int arclengthParameterizationIdx = 1;
        float numBezierSamplePts = 2 * numCurvePts;
        desiredBezierPtToPtDistance = bezierArclengthDistance / (numCurvePts - 1);
        float bezierPtToPtDistanceThresh = desiredBezierPtToPtDistance / 100.0;

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "number of points: " << numCurvePts);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "total distance: " << bezierArclengthDistance);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "desired distance interval: " << desiredBezierPtToPtDistance);

        int interpMaxIter = 100;
        int interpIter = 0;

        float t_interval = (1.0 / numBezierSamplePts);
        float t_k, t_kmin1, t_interp, t_interp_min1, t_lowerBound, t_upperBound;
        float currentBezierPtToPtArclengthDistance, bezierPtToPtArclengthDistance;
        for (float t_k = 0.0; t_k <= 1.0; t_k += t_interval) 
        {
            currentBezierPtToPtArclengthDistance = calculateBezierArclengthDistance(bezierPt0, bezierPt1, bezierPt2, t_kmin1, t_k, numBezierPtToPtIntegrationPoints);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "from " << t_kmin1 << " to " << t_k << ": " << currentBezierPtToPtArclengthDistance);

            if (std::abs(currentBezierPtToPtArclengthDistance - desiredBezierPtToPtDistance) < bezierPtToPtDistanceThresh) 
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "adding " << t_k);
                arclengthParameterization(arclengthParameterizationIdx, 0) = t_k;
                arclengthParameterizationIdx += 1;
                t_kmin1 = t_k;
            } else if (currentBezierPtToPtArclengthDistance > desiredBezierPtToPtDistance) 
            {
                t_interp_min1 = t_k - t_interval;
                t_interp = (t_k + t_interp_min1) / 2.0;
                bezierPtToPtArclengthDistance = calculateBezierArclengthDistance(bezierPt0, bezierPt1, bezierPt2, t_kmin1, t_interp, numBezierPtToPtIntegrationPoints);

                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "from " << t_kmin1 << " to " << t_interp << ": " << bezierPtToPtArclengthDistance);

                t_upperBound = t_k;
                t_lowerBound = t_interp_min1;
                interpIter = 0;                
                while ( abs(bezierPtToPtArclengthDistance - desiredBezierPtToPtDistance) > bezierPtToPtDistanceThresh && interpIter < interpMaxIter) 
                {
                    if (bezierPtToPtArclengthDistance < desiredBezierPtToPtDistance) 
                    {
                        t_lowerBound = t_interp;
                        t_interp = (t_interp + t_upperBound) / 2.0;
                        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "raising t_interp to: " << t_interp);
                    } else 
                    {
                        t_upperBound = t_interp;
                        t_interp = (t_interp + t_lowerBound) / 2.0;
                        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "lowering t_interp to: " << t_interp);

                    }
                    bezierPtToPtArclengthDistance = calculateBezierArclengthDistance(bezierPt0, bezierPt1, bezierPt2, t_kmin1, t_interp, numBezierPtToPtIntegrationPoints);
                    // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "from " << t_kmin1 << " to " << t_interp << ": " << bezierPtToPtArclengthDistance);
                    interpIter++;
                }
                
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "adding " << t_interp);
                arclengthParameterization(arclengthParameterizationIdx, 0) = t_interp;
                arclengthParameterizationIdx += 1;
                t_kmin1 = t_interp;
            }   
        }

        arclengthParameterization(int(numCurvePts) - 1, 0) = 1.0;

        return arclengthParameterization;
        /*
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "uniform indices: ");
        for (int i = 0; i < int(numCurvePts); i++) {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", arclengthParameterization(i, 0));
        }
        */
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "total approx dist: " << bezierArclengthDistance);
    }

    void GapTrajectoryGenerator::buildExtendedGapOrigin(const int numRGEPoints,
                                                        const Eigen::Vector2d & extendedGapOrigin,
                                                        const Eigen::Vector2d & bezierOrigin,
                                                        Eigen::MatrixXd & curvePosns,
                                                        Eigen::MatrixXd & curveVels,
                                                        Eigen::MatrixXd & curveInwardNorms,
                                                        bool left)
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "               [buildExtendedGapOrigin()]");
        Eigen::Vector2d currPt, currVel, currInwardVector, currInwardNorm;
        float s = 0.0;

        Eigen::Matrix2d rotMat;
        if (left)
            rotMat << Rnegpi2(0, 0), Rnegpi2(0, 1), Rnegpi2(1, 0), Rnegpi2(1, 1);
        else
            rotMat << Rpi2(0, 0), Rpi2(0, 1), Rpi2(1, 0), Rpi2(1, 1);
        //  = (left ? Rnegpi2 : Rpi2);
        for (float i = 0; i < numRGEPoints; i++) 
        {
            s = i / numRGEPoints;
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "s_left_rge: " << s);

            // line equation from gapExtendedOrigin to bezierOrigin
            currPt = (1 - s) * extendedGapOrigin + s * bezierOrigin;
            currVel = (bezierOrigin - extendedGapOrigin);
            currInwardVector = rotMat * currVel;
            currInwardNorm = currInwardVector / currInwardVector.norm();
            curvePosns.row(i) = currPt;
            curveVels.row(i) = currVel;
            curveInwardNorms.row(i) = currInwardNorm;
        }
    }

    void GapTrajectoryGenerator::buildBezierCurve(const int numRGEPoints,
                                                  const int totalNumCurvePts,
                                                  const Eigen::VectorXd & arclengthParameters,
                                                  const Eigen::Vector2d & bezierOrigin,
                                                  const Eigen::Vector2d & bezierInitialPt,
                                                  const Eigen::Vector2d & bezierTerminalPt,
                                                  Eigen::MatrixXd & curvePosns,
                                                  Eigen::MatrixXd & curveVels,
                                                  Eigen::MatrixXd & curveInwardNorms,
                                                  bool left)
    {
        Eigen::Vector2d currPt, currVel, currInwardVector, currInwardNorm;
        float s = 0.0;
        float eps = 0.0000001;

        Eigen::Matrix2d rotMat;
        if (left)
            rotMat << Rnegpi2(0, 0), Rnegpi2(0, 1), Rnegpi2(1, 0), Rnegpi2(1, 1);
        else
            rotMat << Rpi2(0, 0), Rpi2(0, 1), Rpi2(1, 0), Rpi2(1, 1);
        
        int counter = 0;
        for (float i = numRGEPoints; i < totalNumCurvePts; i++) 
        {
            s = arclengthParameters(counter, 0);
            counter++;

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "s_left: " << s_left);
            // pos_val0 = ;
            // pos_val1 = ;
            // pos_val2 = ;

            // vel_val0 = ;
            // vel_val1 = ;
            // vel_val2 = ;
            currPt = (1 - s) * (1 - s) * bezierOrigin + 
                        2*(1 - s)*s * bezierInitialPt + 
                        s*s * bezierTerminalPt;
            currVel = (2*s - 2) * bezierOrigin + 
                        (2 - 4*s) * bezierInitialPt + 
                        2*s * bezierTerminalPt;
            currInwardVector = rotMat * currVel;
            currInwardNorm = currInwardVector / (currInwardVector.norm() + eps);
            curvePosns.row(i) = currPt;
            curveVels.row(i) = currVel;
            curveInwardNorms.row(i) = currInwardNorm;
        }
    }

    void GapTrajectoryGenerator::buildExtendedBezierCurve(dynamic_gap::Gap & selectedGap, 
                                                            Eigen::MatrixXd & gapCurvesPosns,
                                                            Eigen::MatrixXd & gapCurvesInwardNorms, 
                                                            Eigen::MatrixXd & gapSideAHPFCenters, Eigen::MatrixXd & allAHPFCenters,
                                                            const Eigen::Vector2d & leftGapPtVel, const Eigen::Vector2d & rightGapPtVel, const Eigen::Vector2d & maxRbtVel,
                                                            const Eigen::Vector2d & leftCurveInitPt, const Eigen::Vector2d & leftCurveTermPt, 
                                                            const Eigen::Vector2d & rightCurveInitPt, const Eigen::Vector2d & rightCurveTermPt, 
                                                            const Eigen::Vector2d & gapGoalTermPt, 
                                                            float leftBezierWeight, float rightBezierWeight, 
                                                            float numCurvePts, int numLeftRGEPoints, int numRightRGEPoints,
                                                            const Eigen::Vector2d & initRbtPos) 
    {  
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            [buildExtendedBezierCurve()]");
        // Eigen::MatrixXd leftCurveVels, rightCurveVels, leftCurveInwardNorms, rightCurveInwardNorms,
        //                 leftCurvePosns, rightCurvePosns;

        // convert from 2f to 2d
        Eigen::Vector2d extendedGapOrigin(selectedGap.extendedGapOrigin_[0],
                                          selectedGap.extendedGapOrigin_[1]);
        Eigen::Vector2d leftBezierOrigin(selectedGap.leftBezierOrigin_[0],
                                            selectedGap.leftBezierOrigin_[1]);
        Eigen::Vector2d rightBezierOrigin(selectedGap.rightBezierOrigin_[0],
                                            selectedGap.rightBezierOrigin_[1]);


        leftBezierWeight = leftGapPtVel.norm() / maxRbtVel.norm(); // capped at 1, we can scale down towards 0 until initial constraints are met?
        rightBezierWeight = rightGapPtVel.norm() / maxRbtVel.norm();

        // for a totally static gap, can get no velocity on first bezier curve point which corrupts vector field
        Eigen::Vector2d weightedLeftBezierPt0, weightedRightBezierPt0;
        if (leftGapPtVel.norm() > 0.0)
            weightedLeftBezierPt0 = leftBezierOrigin + leftBezierWeight * (leftCurveInitPt - leftBezierOrigin);
        else
            weightedLeftBezierPt0 = (0.95 * leftBezierOrigin + 0.05 * leftCurveTermPt);

        if (rightGapPtVel.norm() >  0.0)
            weightedRightBezierPt0 = rightBezierOrigin + rightBezierWeight * (rightCurveInitPt - rightBezierOrigin);
        else
            weightedRightBezierPt0 = (0.95 * rightBezierOrigin + 0.05 * rightCurveTermPt);
        
        // set left and right bezier points
        selectedGap.leftPt0_ = weightedLeftBezierPt0;
        selectedGap.leftPt1_ = leftCurveTermPt;
        selectedGap.rightPt0_ = weightedRightBezierPt0;
        selectedGap.rightPt1_ = rightCurveTermPt;  

        // float pos_val0, pos_val1, pos_val2, vel_val0, vel_val1, vel_val2;
        // Eigen::Vector2d rotated_curr_left_vel, curr_right_pt, curr_right_vel, right_inward_vect, rotated_curr_right_vel, right_inward_norm_vect;
        // Eigen::Matrix2d rpi2, neg_rpi2;
        // float rot_val = M_PI/2;
        // float s, s_left, s_right;
        // rpi2 << std::cos(rot_val), -std::sin(rot_val), std::sin(rot_val), std::cos(rot_val);
        // neg_rpi2 << std::cos(-rot_val), -std::sin(-rot_val), std::sin(-rot_val), std::cos(-rot_val);
        float desiredLeftBezierPtToPtDistance = 0.01; // will be set in arclengthParameterizeBezier  
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   arclength parameterizing left curve");
        Eigen::VectorXd leftArclengthParameters = arclengthParameterizeBezier(leftBezierOrigin, weightedLeftBezierPt0, leftCurveTermPt, numCurvePts, desiredLeftBezierPtToPtDistance);        
        numLeftRGEPoints = (cfg_->gap_manip.radial_extend) ? std::max(int(std::ceil( (extendedGapOrigin - leftBezierOrigin).norm() / desiredLeftBezierPtToPtDistance)), 2) : 0;

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "numLeftRGEPoints: " << numLeftRGEPoints);

        int totalNumLeftCurvePts = numLeftRGEPoints + numCurvePts; 
        Eigen::MatrixXd leftCurvePosns = Eigen::MatrixXd(totalNumLeftCurvePts, 2);
        Eigen::MatrixXd leftCurveVels = Eigen::MatrixXd(totalNumLeftCurvePts, 2);
        Eigen::MatrixXd leftCurveInwardNorms = Eigen::MatrixXd(totalNumLeftCurvePts, 2);

        ///////////////////////////////
        // Left radial gap extension //
        ///////////////////////////////
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   building left extended gap origin");
        buildExtendedGapOrigin(numLeftRGEPoints, extendedGapOrigin, leftBezierOrigin, leftCurvePosns, leftCurveVels, leftCurveInwardNorms, true);

        ///////////////////////
        // Left Bezier curve //
        ///////////////////////
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   building left bezier curve");
        buildBezierCurve(numLeftRGEPoints, totalNumLeftCurvePts, leftArclengthParameters, 
                         leftBezierOrigin, selectedGap.leftPt0_, selectedGap.leftPt1_, leftCurvePosns, leftCurveVels, leftCurveInwardNorms, true);        


        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   leftBezierOrigin: " << leftBezierOrigin[0] << ", " << leftBezierOrigin[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   leftCurveInitPt: " << leftCurveInitPt[0] << ", " << leftCurveInitPt[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   selectedGap.leftPt0_: " << selectedGap.leftPt0_[0] << ", " << selectedGap.leftPt0_[1]);       
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   selectedGap.leftPt1_: " << selectedGap.leftPt1_[0] << ", " << selectedGap.leftPt1_[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   left_vel: " << leftGapPtVel[0] << ", " << leftGapPtVel[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   leftCurvePosns:");
        // for (int i = 0; i < leftCurvePosns.rows(); i++)
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       " << i << ": " << leftCurvePosns(i, 0) << ", " << leftCurvePosns(i, 1));
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   leftCurveVels:");
        // for (int i = 0; i < leftCurveVels.rows(); i++)
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       " << i << ": " << leftCurveVels(i, 0) << ", " << leftCurveVels(i, 1));
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   leftCurveInwardNorms:");
        // for (int i = 0; i < leftCurveInwardNorms.rows(); i++)
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       " << i << ": " << leftCurveInwardNorms(i, 0) << ", " << leftCurveInwardNorms(i, 1));


        float desiredRightBezierPtToPtDistance = 0.01;
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   arclength parameterizing right curve");
        Eigen::VectorXd rightArclengthParameters = arclengthParameterizeBezier(rightBezierOrigin, weightedRightBezierPt0, rightCurveTermPt, numCurvePts, desiredRightBezierPtToPtDistance);
        numRightRGEPoints = (cfg_->gap_manip.radial_extend) ? std::max(int(std::ceil( (extendedGapOrigin - rightBezierOrigin).norm() / desiredRightBezierPtToPtDistance)), 2) : 0;

        int totalNumRightCurvePts = numRightRGEPoints + numCurvePts; 
        Eigen::MatrixXd rightCurvePosns = Eigen::MatrixXd(totalNumRightCurvePts, 2);            
        Eigen::MatrixXd rightCurveVels = Eigen::MatrixXd(totalNumRightCurvePts, 2);
        Eigen::MatrixXd rightCurveInwardNorms = Eigen::MatrixXd(totalNumRightCurvePts, 2);
        
        ////////////////////////////////
        // Right radial gap extension //
        ////////////////////////////////
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   building right extended gap origin");
        buildExtendedGapOrigin(numRightRGEPoints, extendedGapOrigin, rightBezierOrigin, rightCurvePosns, rightCurveVels, rightCurveInwardNorms, false);
             
        ////////////////////////
        // Right Bezier curve //
        ////////////////////////
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   building right bezier curve");
        buildBezierCurve(numRightRGEPoints, totalNumRightCurvePts, rightArclengthParameters, 
                            rightBezierOrigin, selectedGap.rightPt0_, selectedGap.rightPt1_, rightCurvePosns, rightCurveVels, rightCurveInwardNorms, false);        

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rightBezierOrigin: " << rightBezierOrigin[0] << ", " << rightBezierOrigin[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rightCurveInitPt: " << rightCurveInitPt[0] << ", " << rightCurveInitPt[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   selectedGap.rightPt0_: " << selectedGap.rightPt0_[0] << ", " << selectedGap.rightPt0_[1]);       
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   selectedGap.rightPt1_: " << selectedGap.rightPt1_[0] << ", " << selectedGap.rightPt1_[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   right_vel: " << rightGapPtVel[0] << ", " << rightGapPtVel[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rightCurvePosns:");
        // for (int i = 0; i < rightCurvePosns.rows(); i++)
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       " << i << ": " << rightCurvePosns(i, 0) << ", " << rightCurvePosns(i, 1));
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rightCurveVels:");
        // for (int i = 0; i < rightCurveVels.rows(); i++)
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       " << i << ": " << rightCurveVels(i, 0) << ", " << rightCurveVels(i, 1));
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rightCurveInwardNorms:");
        // for (int i = 0; i < rightCurveInwardNorms.rows(); i++)
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "       " << i << ": " << rightCurveInwardNorms(i, 0) << ", " << rightCurveInwardNorms(i, 1));

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "radial extensions: ");
        // ADDING DISCRETE POINTS FOR RADIAL GAP EXTENSION

        // model gives: left_pt - rbt.
        // populating the quadratic weighted bezier

        // Eigen::Matrix<double, 1, 2> origin, centered_origin_inward_norm;
        // origin << 0.0, 0.0;
        float offset = 0.01; // (desiredLeftBezierPtToPtDistance + desiredRightBezierPtToPtDistance) / 2.0;
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "offset: " << offset);

        // counter = 0;
        // for (float i = numRightRGEPoints; i < totalNumRightCurvePts; i++) {

        //     s_right = rightArclengthParameters(counter, 0);
        //     counter++;

        //     // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "s_right: " << s_right);

        //     // pos_val0 = ;
        //     // pos_val1 = ;
        //     // pos_val2 = ;

        //     // vel_val0 = ;
        //     // vel_val1 = ;
        //     // vel_val2 = ;
        //     curr_right_pt = (1 - s_right) * (1 - s_right) * rightBezierOrigin + 
        //                     2 * (1 - s_right) * s_right * weightedRightBezierPt0 + 
        //                     s_right*s_right * rightCurveTermPt;
        //     curr_right_vel = (2*s_right - 2) * rightBezierOrigin + 
        //                      (2 - 4*s_right) * weightedRightBezierPt0 + 
        //                      2*s_right * rightCurveTermPt;
        //     rotated_curr_right_vel = rpi2 * curr_right_vel;
        //     right_inward_norm_vect = rotated_curr_right_vel / (rotated_curr_right_vel.norm() + eps);
        //     rightCurvePosns.row(i) = curr_right_pt;
        //     rightCurveVels.row(i) = curr_right_vel;
        //     rightCurveInwardNorms.row(i) = right_inward_norm_vect;

        //     /*
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "left_pt: " << curr_left_pt[0] << ", " << curr_left_pt[1]);
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "left_vel " << i << ": " << curr_left_vel[0] << ", " << curr_left_vel[1]);
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "left_inward_norm: " << left_inward_norm_vect[0] << ", " << left_inward_norm_vect[1]);
        //     // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "left_center: " << (curr_left_pt[0] - left_inward_norm[0]*offset) << ", " << (curr_left_pt[1] - left_inward_norm[1]*offset));
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "right_pt " << i << ": " << curr_right_pt[0] << ", " << curr_right_pt[1]);
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "right_vel " << i << ": " << curr_right_vel[0] << ", " << curr_right_vel[1]);
        //     ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "right_inward_norm " << i << ": " << right_inward_norm_vect[0] << ", " << right_inward_norm_vect[1]);
        //     // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "curr_right_pt: " << curr_right_pt[0] << ", " << curr_right_pt[1]);
        //     */
        // }
        
        // Eigen::Vector2d left_origin_inward_norm = leftCurveInwardNorms.row(0);
        // Eigen::Vector2d right_origin_inward_norm = rightCurveInwardNorms.row(0);

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "left_origin_inward_norm: " << left_origin_inward_norm[0] << ", " << left_origin_inward_norm[1]);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "right_origin_inward_norm: " << right_origin_inward_norm[0] << ", " << right_origin_inward_norm[1]);
        // double thetaLeft_origin_inward_norm = std::atan2(left_origin_inward_norm[1], left_origin_inward_norm[0]);
        // double init_leftToRightAngle = getLeftToRightAngle(left_origin_inward_norm, right_origin_inward_norm);
        // double beta_origin_center = thetaLeft_origin_inward_norm - 0.5 * init_leftToRightAngle;
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "init_leftToRightAngle: " << init_leftToRightAngle << ", beta_origin_center: " << beta_origin_center);
        // centered_origin_inward_norm << std::cos(beta_origin_center), std::sin(beta_origin_center);

        gapCurvesPosns = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
        gapCurvesInwardNorms = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
        gapSideAHPFCenters = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
        allAHPFCenters = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts + 1, 2);

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "centered origin inward norm: " << centered_origin_inward_norm[0] << ", " << centered_origin_inward_norm[1]);
        gapCurvesPosns << leftCurvePosns, rightCurvePosns; // origin, 
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "gapCurvesPosns worked");
        gapCurvesInwardNorms << leftCurveInwardNorms, rightCurveInwardNorms; // centered_origin_inward_norm, 
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "gapCurvesInwardNorms worked");
        gapSideAHPFCenters = gapCurvesPosns - gapCurvesInwardNorms*offset;
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "gapSideAHPFCenters worked");
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "left_curve points: " << left_curve);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "right_curve_points: " << right_curve);

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "left_curve inward norms: " << leftCurveInwardNorms);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "right_curve inward_norms: " << rightCurveInwardNorms);

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "gapCurvesInwardNorms: " << gapCurvesInwardNorms);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "allAHPFCenters: " << allAHPFCenters);

        Eigen::Matrix<double, 1, 2> goal; // (1, 2);
        goal << gapGoalTermPt[0], gapGoalTermPt[1];
        allAHPFCenters << goal, gapSideAHPFCenters;
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "allAHPFCenters worked");
    }

    // If i try to delete this DGap breaks
    // [[deprecated("Use single trajectory generation")]]
    // std::vector<geometry_msgs::PoseArray> GapTrajectoryGenerator::generateTrajectory(std::vector<dynamic_gap::Gap> gapset) 
    // {
    //     std::vector<geometry_msgs::PoseArray> traj_set(gapset.size());
    //     return traj_set;
    // }

    // Transform local trajectory between two frames of choice
    geometry_msgs::PoseArray GapTrajectoryGenerator::transformLocalTrajectory(const geometry_msgs::PoseArray & path,
                                                                              const geometry_msgs::TransformStamped & transform,
                                                                              const std::string & sourceFrame,
                                                                              const std::string & destFrame)
    {
        geometry_msgs::PoseArray transformedPath;

        geometry_msgs::PoseStamped sourcePose;
        sourcePose.header.frame_id = sourceFrame; // cfg_->robot_frame_id;

        geometry_msgs::PoseStamped destPose;
        destPose.header.frame_id = destFrame; // cfg_->odom_frame_id;

        for (const auto pose : path.poses)
        {
            sourcePose.pose = pose;
            tf2::doTransform(sourcePose, destPose, transform);
            transformedPath.poses.push_back(destPose.pose);
        }
        transformedPath.header.frame_id = destFrame; // cfg_->odom_frame_id;
        transformedPath.header.stamp = ros::Time::now();
        // ROS_WARN_STREAM("leaving transform back with length: " << transformedPath.poses.size());
        return transformedPath;
    }

    std::tuple<geometry_msgs::PoseArray, std::vector<float>> GapTrajectoryGenerator::processTrajectory(const std::tuple<geometry_msgs::PoseArray, std::vector<float>> & traj)
    {
        geometry_msgs::PoseArray rawPath = std::get<0>(traj);
        std::vector<float> rawPathTiming = std::get<1>(traj);
        
        geometry_msgs::Pose originPose;
        originPose.position.x = 0;
        originPose.position.y = 0;
        originPose.position.z = 0;
        originPose.orientation.x = 0;
        originPose.orientation.y = 0;
        originPose.orientation.z = 0;
        originPose.orientation.w = 1;
        // std::cout << "entering at : " << path.poses.size() << std::endl;
        //std::cout << "starting pose: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << std::endl; 
        //std::cout << "final pose: " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << std::endl;

        std::vector<geometry_msgs::Pose> processedPoses;
        std::vector<float> processedPathTiming;
        processedPoses.push_back(originPose);
        processedPathTiming.push_back(0.0);
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pose[0]: " << path.poses[0].position.x << ", " << path.poses[0].position.y);

        float poseToPoseDistThreshold = 0.1;
        float poseToPoseDiffX, poseToPoseDiffY, poseToPoseDist;
        for (int i = 1; i < rawPath.poses.size(); i++) 
        {
            geometry_msgs::Pose rawPose = rawPath.poses[i];
            poseToPoseDiffX = rawPose.position.x - processedPoses.back().position.x;
            poseToPoseDiffY = rawPose.position.y - processedPoses.back().position.y;
            poseToPoseDist = sqrt(pow(poseToPoseDiffX, 2) + pow(poseToPoseDiffY, 2));
            if (poseToPoseDist > poseToPoseDistThreshold) 
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "poseToPoseDist " << i << " kept at " << poseToPoseDist);
                processedPoses.push_back(rawPose);
                processedPathTiming.push_back(rawPathTiming[i]);
            } else 
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "poseToPoseDist " << i << " cut at " << poseToPoseDist);
            }
        }
        // std::cout << "leaving at : " << shortened.size() << std::endl;
        
        geometry_msgs::PoseArray processedPath = rawPath;        
        processedPath.poses = processedPoses;

        // Fix rotation along local trajectory
        geometry_msgs::Pose processedPose, prevProcessedPose;
        Eigen::Quaternionf q;
        float poseToPoseDiffTheta;
        for (int idx = 1; idx < processedPath.poses.size(); idx++)
        {
            processedPose = processedPath.poses[idx];
            prevProcessedPose = processedPath.poses[idx - 1];
            poseToPoseDiffX = processedPose.position.x - prevProcessedPose.position.x;
            poseToPoseDiffY = processedPose.position.y - prevProcessedPose.position.y;
            poseToPoseDiffTheta = std::atan2(poseToPoseDiffY, poseToPoseDiffX);
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(poseToPoseDiffTheta, Eigen::Vector3f::UnitZ());
            q.normalize();
            processedPath.poses[idx - 1].orientation.x = q.x();
            processedPath.poses[idx - 1].orientation.y = q.y();
            processedPath.poses[idx - 1].orientation.z = q.z();
            processedPath.poses[idx - 1].orientation.w = q.w();
        }
        processedPath.poses.pop_back();
        processedPathTiming.pop_back();

        std::tuple<geometry_msgs::PoseArray, std::vector<float>> processedTrajectory(processedPath, processedPathTiming);
        return processedTrajectory;
    }

}