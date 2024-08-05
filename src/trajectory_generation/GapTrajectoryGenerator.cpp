#include <dynamic_gap/trajectory_generation/GapTrajectoryGenerator.h>

namespace dynamic_gap 
{
    bool GapTrajectoryGenerator::initializeSolver(OsqpEigen::Solver & solver, const int & Kplus1, 
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

        bool validSolver = true;

        if (!solver.data()->setHessianMatrix(hessian)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET HESSIAN"); validSolver = false; }

        if(!solver.data()->setGradient(gradient)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET GRADIENT"); validSolver = false; }

        if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET LINEAR MATRIX"); validSolver = false; }

        if(!solver.data()->setLowerBound(lowerBound)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET LOWER BOUND"); validSolver = false; }

        if(!solver.data()->setUpperBound(upperBound)) { ROS_FATAL_STREAM("SOLVER FAILED TO SET UPPER BOUND"); validSolver = false; }

        if(!solver.initSolver()) { ROS_FATAL_STREAM("SOLVER FAILED TO INITIALIZE SOLVER"); validSolver = false; }

        return validSolver;
    }

    dynamic_gap::Trajectory GapTrajectoryGenerator::generateTrajectory(dynamic_gap::Gap * selectedGap, 
                                                                        const geometry_msgs::PoseStamped & currPose, 
                                                                        const geometry_msgs::TwistStamped & currVel,
                                                                        const bool & runGoToGoal) 
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
            path.header.frame_id = cfg_->sensor_frame_id;

            Eigen::Vector4f rbtState(currPose.pose.position.x + 1e-5, currPose.pose.position.y + 1e-6,
                                  0.0, 0.0); // currVel.linear.x, currVel.linear.y

            // get gap points in cartesian
            float xLeft = 0.0, yLeft = 0.0, xRight = 0.0, yRight = 0.0;
            selectedGap->getSimplifiedLCartesian(xLeft, yLeft);
            selectedGap->getSimplifiedRCartesian(xRight, yRight);

            float xLeftTerm = 0.0, yLeftTerm = 0.0, xRightTerm = 0.0, yRightTerm = 0.0;
            selectedGap->getSimplifiedTerminalLCartesian(xLeftTerm, yLeftTerm);
            selectedGap->getSimplifiedTerminalRCartesian(xRightTerm, yRightTerm);

            Eigen::Vector2d initialGoal(selectedGap->goal.x_, selectedGap->goal.y_);
            Eigen::Vector2d terminalGoal(selectedGap->terminalGoal.x_, selectedGap->terminalGoal.y_);

            float initialGoalX = initialGoal[0];
            float initialGoalY = initialGoal[1];
            float terminalGoalX = terminalGoal[0];
            float terminalGoalY = terminalGoal[1];

            float goalVelX = epsilonDivide(terminalGoalX - initialGoalX, selectedGap->gapLifespan_); // absolute velocity (not relative to robot)
            float goalVelY = epsilonDivide(terminalGoalY - initialGoalY, selectedGap->gapLifespan_);

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual initial robot pos: (" << rbtState[0] << ", " << rbtState[1] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual inital robot velocity: " << rbtState[2] << ", " << rbtState[3] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual initial left point: (" << xLeft << ", " << yLeft << "), actual initial right point: (" << xRight << ", " << yRight << ")"); 
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual terminal left point: (" << xLeftTerm << ", " << yLeftTerm << "), actual terminal right point: (" << xRightTerm << ", " << yRightTerm << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual initial goal: (" << initialGoalX << ", " << initialGoalY << ")"); 
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            actual terminal goal: (" << terminalGoalX << ", " << terminalGoalY << ")"); 
            
            float leftVelX = epsilonDivide(xLeftTerm - xLeft, selectedGap->gapLifespan_);
            float leftVelY = epsilonDivide(yLeftTerm - yLeft, selectedGap->gapLifespan_);

            float rightVelX = epsilonDivide(xRightTerm - xRight, selectedGap->gapLifespan_);
            float rightVelY = epsilonDivide(yRightTerm - yRight, selectedGap->gapLifespan_);

            robotAndGapState x = {rbtState[0], rbtState[1], xLeft, yLeft, xRight, yRight, initialGoalX, initialGoalY};
            
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "pre-integration, x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);

            if (runGoToGoal) 
            {
                robotAndGapState x = {rbtState[0], rbtState[1], 0.0, 0.0, 0.0, 0.0, selectedGap->goal.x_, selectedGap->goal.y_};
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "Goal to Goal");
                GoToGoal goToGoal(cfg_->rbt.vx_absmax);
                boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<robotAndGapState>(),
                goToGoal, x, 0.0f, cfg_->traj.integrate_maxt, cfg_->traj.integrate_stept, corder);
                dynamic_gap::Trajectory traj(path, pathTiming);
                float generateTrajectoryTime = timeTaken(generateTrajectoryStartTime);
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            generateTrajectory (g2g) time taken: " << generateTrajectoryTime << " seconds");                
                return traj;
            }

            // Eigen::Vector2d initRbtPos(x[0], x[1]);
            // Eigen::Vector2d leftCurveInitPt(xLeft, yLeft);
            // Eigen::Vector2d leftCurveTermPt(xLeftTerm, yLeftTerm);
            // Eigen::Vector2d rightCurveInitPt(xRight, yRight);
            // Eigen::Vector2d rightCurveTermPt(xRightTerm, yRightTerm);
            Eigen::Vector2d leftGapPtVel(leftVelX, leftVelY);
            Eigen::Vector2d rightGapPtVel(rightVelX, rightVelY);
            Eigen::Vector2d gapGoalTermPt(terminalGoalX, terminalGoalY);
            // Eigen::Vector2d gapGoalVel(goalVelX, goalVelY);
            
            Eigen::Vector2d maxRbtVel(cfg_->rbt.vx_absmax, cfg_->rbt.vy_absmax);
            // Eigen::Vector2d maxRbtAcc(cfg_->rbt.ax_absmax, cfg_->rbt.ay_absmax);

            // Eigen::MatrixXd gapCurvesPosns, gapCurvesInwardNorms, 
            //                 gapSideAHPFCenters, allAHPFCenters;
            
            // float leftBezierWeight = 0.0, rightBezierWeight = 0.0;
            // int numLeftRGEPoints = 0, numRightRGEPoints = 0;
            // THIS IS BUILT WITH EXTENDED POINTS. 
		    // std::chrono::steady_clock::time_point buildExtendedBezierCurveStartTime = std::chrono::steady_clock::now();
            // buildExtendedBezierCurve(selectedGap, 
            //                         leftGapPtVel, rightGapPtVel, maxRbtVel, 
            //                         leftCurveInitPt, leftCurveTermPt, rightCurveInitPt, rightCurveTermPt, 
            //                         gapGoalTermPt, numCurvePts); // initRbtPos
            // float buildExtendedBezierCurveTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - buildExtendedBezierCurveStartTime).count() / 1.0e6;
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            buildExtendedBezierCurve time taken: " << buildExtendedBezierCurveTime << " seconds");

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "after buildExtendedBezierCurve, left weight: " << leftBezierWeight << ", rightBezierWeight: " << rightBezierWeight);
            // selectedGap->leftBezierWeight_ = leftBezierWeight;
            // selectedGap->rightBezierWeight_ = rightBezierWeight;
            // selectedGap->leftRightCenters_ = gapSideAHPFCenters;
            // selectedGap->allCurvePts_ = gapCurvesPosns;
            // selectedGap->numLeftRGEPoints_ = numLeftRGEPoints;
            // selectedGap->numRightRGEPoints_ = numRightRGEPoints;

            // add radial gap extension
            // initialGoalX -= selectedGap->extendedGapOrigin_[0];
            // initialGoalY -= selectedGap->extendedGapOrigin_[1];
            // xLeft -= selectedGap->extendedGapOrigin_[0];
            // yLeft -= selectedGap->extendedGapOrigin_[1];
            // xRight -= selectedGap->extendedGapOrigin_[0];
            // yRight -= selectedGap->extendedGapOrigin_[1];
            // rbtState[0] -= selectedGap->extendedGapOrigin_[0];
            // rbtState[1] -= selectedGap->extendedGapOrigin_[1];

            x = {rbtState[0], rbtState[1], xLeft, yLeft, xRight, yRight, terminalGoalX, terminalGoalY};

            /*
            PolarGapField polarGapField(xRight, xLeft, yRight, yLeft,
                                                    initialGoalX, initialGoalY,
                                                    selectedGap->isRadial(),
                                                    x[0], x[1],
                                                    cfg_->rbt.vx_absmax, cfg_->rbt.vx_absmax);
            */

            /*
            SETTING UP SOLVER
            */
            
            int N = selectedGap->allCurvePts_.rows();
            int Kplus1 = selectedGap->allAHPFCenters_.rows();
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "N: " << N << ", Kplus1: " << Kplus1);

            Eigen::MatrixXd A(Kplus1, Kplus1);
            // double setConstraintMatrix_start_time = ros::WallTime::now().toSec();
            setConstraintMatrix(A, N, Kplus1, selectedGap->allCurvePts_, 
                                selectedGap->gapBoundaryInwardNorms_, selectedGap->allAHPFCenters_);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            setConstraintMatrix time taken: " << ros::WallTime::now().toSec() - setConstraintMatrix_start_time);

            OsqpEigen::Solver solver;
            double timeLimit = 0.5; // seconds
            solver.settings()->setTimeLimit(timeLimit);

            // double initializeSolver_start_time = ros::WallTime::now().toSec();
            bool validSolver = initializeSolver(solver, Kplus1, A);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            initializeSolver time taken: " << ros::WallTime::now().toSec() - initializeSolver_start_time);

            if (!validSolver)
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "SOLVER FAILED TO INITIALIZE");
                dynamic_gap::Trajectory traj(path, pathTiming);
                return traj;
            }

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "A: " << A);
            
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
                dynamic_gap::Trajectory traj(path, pathTiming);
                return traj;
            }
            float solveProblemTime = timeTaken(solveProblemStartTime);
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


            // AHPF
            AHPF ahpf(cfg_->rbt.vx_absmax, selectedGap->allAHPFCenters_, selectedGap->gapBoundaryInwardNorms_, weights,
                        leftGapPtVel, rightGapPtVel, gapGoalTermPt);   

            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<robotAndGapState>(),
                                                    ahpf, x, 0.0f, selectedGap->gapLifespan_, 
                                                    cfg_->traj.integrate_stept, corder);
            
            /*
            // POLAR GAP FIELD
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<robotAndGapState>(),
                                                    polarGapField, x, 0.0f, selectedGap->gapLifespan_, 
                                                    cfg_->traj.integrate_stept, corder);
            for (geometry_msgs::Pose & p : posearr.poses) 
            {
                p.position.x += selectedGap->extendedGapOrigin_[0];
                p.position.y += selectedGap->extendedGapOrigin_[1];
            }
            */

            dynamic_gap::Trajectory traj(path, pathTiming);
            float generateTrajectoryTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - generateTrajectoryStartTime).count() / 1.0e6;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            generateTrajectory (ahpf) time taken: " << generateTrajectoryTime << " seconds");
            return traj;
            
        } catch (...) 
        {
            ROS_FATAL_STREAM("integrator");
            dynamic_gap::Trajectory traj(path, pathTiming);
            return traj;
        }

    }

    void GapTrajectoryGenerator::setConstraintMatrix(Eigen::MatrixXd &A, const int & N, const int & Kplus1, 
                                                        const Eigen::MatrixXd & gapCurvesPosns, 
                                                        const Eigen::MatrixXd & gapCurvesInwardNorms,
                                                        const Eigen::MatrixXd & allAHPFCenters) 
    {
        Eigen::MatrixXd dIthGradientPtdAhpfCenters(Kplus1, 2); // (2, Kplus1); // 
        Eigen::MatrixXd ABoundaryPts(Kplus1, N);
        Eigen::MatrixXd ANegativeOne = Eigen::MatrixXd::Zero(Kplus1, 1);

        float eps = 0.0000001;
        // allAHPFCenters size: (Kplus1 rows, 2 cols)
        Eigen::Vector2d ithBoundaryPt, ithBoundaryInwardNorm;
        Eigen::MatrixXd ahpfCentersToBoundaryPt;
        Eigen::VectorXd rowWiseSquaredNorms;
        for (int i = 0; i < N; i++) 
        {
            ithBoundaryPt = gapCurvesPosns.row(i);
            ithBoundaryInwardNorm = gapCurvesInwardNorms.row(i);
            
            // doing boundary - allAHPFCenters
            ahpfCentersToBoundaryPt = (-allAHPFCenters).rowwise() + ithBoundaryPt.transpose(); // must be in this order for matrices to work
            // need to divide rowwise by norm of each row
            rowWiseSquaredNorms = ahpfCentersToBoundaryPt.rowwise().squaredNorm();
            dIthGradientPtdAhpfCenters = ahpfCentersToBoundaryPt.array().colwise() / (rowWiseSquaredNorms.array() + eps);
            
            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "ithBoundaryInwardNorm: " << ithBoundaryInwardNorm);
            //ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "gradient_of_pti: " << dIthGradientPtdAhpfCenters);

            ABoundaryPts.col(i) = dIthGradientPtdAhpfCenters * ithBoundaryInwardNorm; // A_pi;

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

    // Transform local trajectory between two frames of choice
    geometry_msgs::PoseArray GapTrajectoryGenerator::transformPath(const geometry_msgs::PoseArray & path,
                                                                              const geometry_msgs::TransformStamped & transform)
    {
        geometry_msgs::PoseStamped sourcePose;
        sourcePose.header.frame_id = transform.header.frame_id; // cfg_->robot_frame_id;

        geometry_msgs::PoseStamped destPose;
        destPose.header.frame_id = transform.child_frame_id; // cfg_->odom_frame_id;

        geometry_msgs::PoseArray transformedPath;
        for (const geometry_msgs::Pose pose : path.poses)
        {
            sourcePose.pose = pose;
            tf2::doTransform(sourcePose, destPose, transform);
            transformedPath.poses.push_back(destPose.pose);
        }
        transformedPath.header.frame_id = destPose.header.frame_id; // cfg_->odom_frame_id;
        transformedPath.header.stamp = ros::Time::now();
        // ROS_WARN_STREAM("leaving transform back with length: " << transformedPath.poses.size());
        return transformedPath;
    }

    dynamic_gap::Trajectory GapTrajectoryGenerator::processTrajectory(const dynamic_gap::Trajectory & traj)
    {
        geometry_msgs::PoseArray rawPath = traj.getPathRbtFrame();
        std::vector<float> rawPathTiming = traj.getPathTiming();
        
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
        float poseToPoseDiffX = 0.0, poseToPoseDiffY = 0.0, poseToPoseDist = 0.0;
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
                processedPathTiming.push_back(rawPathTiming.at(i));
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
        float poseToPoseDiffTheta = 0.0;
        for (int idx = 1; idx < processedPath.poses.size(); idx++)
        {
            processedPose = processedPath.poses.at(idx);
            prevProcessedPose = processedPath.poses.at(idx-1);
            poseToPoseDiffX = processedPose.position.x - prevProcessedPose.position.x;
            poseToPoseDiffY = processedPose.position.y - prevProcessedPose.position.y;
            poseToPoseDiffTheta = std::atan2(poseToPoseDiffY, poseToPoseDiffX);
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(poseToPoseDiffTheta, Eigen::Vector3f::UnitZ());
            q.normalize();
            processedPath.poses.at(idx-1).orientation.x = q.x();
            processedPath.poses.at(idx-1).orientation.y = q.y();
            processedPath.poses.at(idx-1).orientation.z = q.z();
            processedPath.poses.at(idx-1).orientation.w = q.w();
        }
        processedPath.poses.pop_back();
        processedPathTiming.pop_back();

        dynamic_gap::Trajectory processedTrajectory(processedPath, processedPathTiming);
        return processedTrajectory;
    }

}