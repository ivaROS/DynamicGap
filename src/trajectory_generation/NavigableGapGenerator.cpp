#include <dynamic_gap/trajectory_generation/NavigableGapGenerator.h>

namespace dynamic_gap 
{
    void NavigableGapGenerator::generateNavigableGap(dynamic_gap::Gap * selectedGap)
    {
        int numCurvePts = cfg_->traj.num_curve_points;

        // get gap points in cartesian
        float xLeft = 0.0, yLeft = 0.0, xRight = 0.0, yRight = 0.0;
        selectedGap->getLCartesian(xLeft, yLeft);
        selectedGap->getRCartesian(xRight, yRight);

        float xLeftTerm = 0.0, yLeftTerm = 0.0, xRightTerm = 0.0, yRightTerm = 0.0;
        selectedGap->getLCartesian(xLeftTerm, yLeftTerm);
        selectedGap->getRCartesian(xRightTerm, yRightTerm);

        Eigen::Vector2d initialGoal(selectedGap->goal.x_, selectedGap->goal.y_);
        Eigen::Vector2d terminalGoal(selectedGap->terminalGoal.x_, selectedGap->terminalGoal.y_);

        float initialGoalX = initialGoal[0];
        float initialGoalY = initialGoal[1];
        float terminalGoalX = terminalGoal[0];
        float terminalGoalY = terminalGoal[1];

        float goalVelX = epsilonDivide(terminalGoalX - initialGoalX, selectedGap->gapLifespan_); // absolute velocity (not relative to robot)
        float goalVelY = epsilonDivide(terminalGoalY - initialGoalY, selectedGap->gapLifespan_);

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual initial robot pos: (" << rbtState[0] << ", " << rbtState[1] << ")");
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual inital robot velocity: " << rbtState[2] << ", " << rbtState[3] << ")");
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual initial left point: (" << xLeft << ", " << yLeft << "), actual initial right point: (" << xRight << ", " << yRight << ")"); 
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual terminal left point: (" << xLeftTerm << ", " << yLeftTerm << "), actual terminal right point: (" << xRightTerm << ", " << yRightTerm << ")");
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual initial goal: (" << initialGoalX << ", " << initialGoalY << ")"); 
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual terminal goal: (" << terminalGoalX << ", " << terminalGoalY << ")"); 
        
        float leftVelX = epsilonDivide(xLeftTerm - xLeft, selectedGap->gapLifespan_);
        float leftVelY = epsilonDivide(yLeftTerm - yLeft, selectedGap->gapLifespan_);

        float rightVelX = epsilonDivide(xRightTerm - xRight, selectedGap->gapLifespan_);
        float rightVelY = epsilonDivide(yRightTerm - yRight, selectedGap->gapLifespan_);
        
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "pre-integration, x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);

        // Eigen::Vector2d initRbtPos(x[0], x[1]);
        Eigen::Vector2d leftCurveInitPt(xLeft, yLeft);
        Eigen::Vector2d leftCurveTermPt(xLeftTerm, yLeftTerm);
        Eigen::Vector2d rightCurveInitPt(xRight, yRight);
        Eigen::Vector2d rightCurveTermPt(xRightTerm, yRightTerm);
        Eigen::Vector2d leftGapPtVel(leftVelX, leftVelY);
        Eigen::Vector2d rightGapPtVel(rightVelX, rightVelY);
        Eigen::Vector2d gapGoalTermPt(terminalGoalX, terminalGoalY);
        Eigen::Vector2d gapGoalVel(goalVelX, goalVelY);
        
        Eigen::Vector2d maxRbtVel(cfg_->control.vx_absmax, cfg_->control.vy_absmax);
        // Eigen::Vector2d maxRbtAcc(cfg_->control.ax_absmax, cfg_->control.ay_absmax);

        // Eigen::MatrixXd gapCurvesPosns, gapCurvesInwardNorms, 
        //                 gapSideAHPFCenters, allAHPFCenters;
        
        // THIS IS BUILT WITH EXTENDED POINTS. 
        std::chrono::steady_clock::time_point buildExtendedBezierCurveStartTime = std::chrono::steady_clock::now();
        buildExtendedBezierCurve(selectedGap, 
                                    leftGapPtVel, rightGapPtVel, maxRbtVel, 
                                    leftCurveInitPt, leftCurveTermPt, rightCurveInitPt, rightCurveTermPt, 
                                    gapGoalTermPt, numCurvePts); // initRbtPos
        float buildExtendedBezierCurveTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - buildExtendedBezierCurveStartTime).count() / 1.0e6;
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            buildExtendedBezierCurve time taken: " << buildExtendedBezierCurveTime << " seconds");

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "after buildExtendedBezierCurve, left weight: " << leftBezierWeight << ", rightBezierWeight: " << rightBezierWeight);
    }

    float NavigableGapGenerator::approximateBezierArclength(const Eigen::Vector2d & bezierPt0, 
                                                                const Eigen::Vector2d & bezierPt1, 
                                                                const Eigen::Vector2d & bezierPt2, 
                                                                const float & tStart, 
                                                                const float & tEnd, 
                                                                const float & numPoints) 
    {
        float arclengthDistance = 0.0;
        float steps = numPoints - 1;

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "performing numerical integration between " << tStart << " and " << tEnd);
        float t_i = 0.0, t_iplus1 = 0.0, bezierPtToPtDist = 0.0;
        Eigen::Vector2d bezierPt_i(0.0, 0.0);
        Eigen::Vector2d bezierPt_iplus1(0.0, 0.0);

        // Integrating along a Bezier curve from tStart to tEnd with numPoints to get an arclength distance
        for (int i = 0; i < steps; i++) 
        {
            t_i = tStart + (tEnd - tStart) * (i / steps);
            t_iplus1 = tStart + (tEnd - tStart) * ((i + 1) / steps);
            // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "t_i: " << t_i << ", t_iplus1: " << t_iplus1);

            // applying Bezier equation
            bezierPt_i = (1 - t_i)*(1 - t_i)*bezierPt0 + 2*(1 - t_i)*t_i*bezierPt1 + (t_i)*(t_i)*bezierPt2;
            bezierPt_iplus1 = (1 - t_iplus1)*(1 - t_iplus1)*bezierPt0 + 2*(1 - t_iplus1)*t_iplus1*bezierPt1 + (t_iplus1)*(t_iplus1)*bezierPt2;
            
            // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "bezierPt_i: " << bezierPt_i[0] << ", " << bezierPt_i[1] << ", bezierPt_iplus1: " << bezierPt_iplus1[0] << ", " << bezierPt_iplus1[1]);
            bezierPtToPtDist = (bezierPt_iplus1 - bezierPt_i).norm();
            // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "adding dist:" << dist);
            arclengthDistance += bezierPtToPtDist;
        }

        return arclengthDistance;
    }

    Eigen::VectorXd NavigableGapGenerator::arclengthParameterizeBezier(const Eigen::Vector2d & bezierPt0, 
                                                                        const Eigen::Vector2d & bezierPt1, 
                                                                        const Eigen::Vector2d & bezierPt2, 
                                                                        const float & numCurvePts,
                                                                        float & desiredBezierPtToPtDistance) 
    {
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               [arclengthParameterizeBezier()]");
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   bezierPt0: " << bezierPt0[0] << ", " << bezierPt0[1]);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   bezierPt1: " << bezierPt1[0] << ", " << bezierPt1[1]);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   bezierPt2: " << bezierPt2[0] << ", " << bezierPt2[1]);        

        float bezierArclengthDistance = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                        0.0, 1.0, numCurvePts);

        float numBezierPtToPtIntegrationPoints = 5.0;
        Eigen::VectorXd arclengthParameterization = Eigen::MatrixXd::Zero(int(numCurvePts), 1);
        int arclengthParameterizationIdx = 1;
        float numBezierSamplePts = 2 * numCurvePts;
        desiredBezierPtToPtDistance = epsilonDivide(bezierArclengthDistance, (numCurvePts - 1));
        float bezierPtToPtDistanceThresh = desiredBezierPtToPtDistance / 100.0;

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   number of points: " << numCurvePts);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   total distance: " << bezierArclengthDistance);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   desired distance interval: " << desiredBezierPtToPtDistance);

        int interpMaxIter = 100;
        int interpIter = 0;

        float t_interval = epsilonDivide(1.0, numBezierSamplePts);
        float t_interp = 0.0, t_lowerBound = 0.0, t_upperBound = 0.0, 
              currentBezierPtToPtArclengthDistance = 0.0, bezierPtToPtArclengthDistance = 0.0, t_kmin1 = 0.0;
        
        for (float t_k = 0.0; t_k <= 1.0; t_k += t_interval) 
        {
            currentBezierPtToPtArclengthDistance = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                                    t_kmin1, t_k, numBezierPtToPtIntegrationPoints);
            ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   from " << t_kmin1 << " to " << t_k << ": " << currentBezierPtToPtArclengthDistance);

            if (std::abs(currentBezierPtToPtArclengthDistance - desiredBezierPtToPtDistance) < bezierPtToPtDistanceThresh) 
            {
                // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "adding " << t_k);
                if (arclengthParameterizationIdx < arclengthParameterization.rows())
                {
                    arclengthParameterization(arclengthParameterizationIdx, 0) = t_k;
                    arclengthParameterizationIdx += 1;
                }
            } else if (currentBezierPtToPtArclengthDistance > desiredBezierPtToPtDistance) 
            {
                t_interp = (t_kmin1 + t_k) / 2.0;
                bezierPtToPtArclengthDistance = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                                 t_kmin1, t_interp, numBezierPtToPtIntegrationPoints);

                // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "from " << t_kmin1 << " to " << t_interp << ": " << bezierPtToPtArclengthDistance);
                t_lowerBound = t_kmin1;
                t_upperBound = t_k;

                interpIter = 0;                
                while (abs(bezierPtToPtArclengthDistance - desiredBezierPtToPtDistance) > bezierPtToPtDistanceThresh && interpIter < interpMaxIter) 
                {
                    if (bezierPtToPtArclengthDistance < desiredBezierPtToPtDistance) 
                    {
                        t_lowerBound = t_interp;
                        t_interp = (t_interp + t_upperBound) / 2.0;
                        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "raising t_interp to: " << t_interp);
                    } else 
                    {
                        t_upperBound = t_interp;
                        t_interp = (t_interp + t_lowerBound) / 2.0;
                        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "lowering t_interp to: " << t_interp);
                    }
                    bezierPtToPtArclengthDistance = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                                     t_kmin1, t_interp, numBezierPtToPtIntegrationPoints);
                    // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "from " << t_kmin1 << " to " << t_interp << ": " << bezierPtToPtArclengthDistance);
                    interpIter++;
                }
                
                // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "adding " << t_interp);
                if (arclengthParameterizationIdx < arclengthParameterization.rows())
                {                
                    arclengthParameterization(arclengthParameterizationIdx, 0) = t_interp;
                    arclengthParameterizationIdx += 1;
                }
            }   
            t_kmin1 = arclengthParameterization(arclengthParameterizationIdx - 1, 0);
        }

        arclengthParameterization(arclengthParameterization.rows() - 1, 0) = 1.0;
        
        
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "uniform indices: ");
        for (int i = 0; i < arclengthParameterization.rows(); i++)
        {
            // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", arclengthParameterization(i, 0));
            if (i > 0)
            {
                if (arclengthParameterization(i, 0) <= arclengthParameterization(i - 1, 0))
                    ROS_WARN_STREAM_NAMED("NavigableGapGenerator", "       BAD PARAMETERIZATION");
            }
        }
         
        return arclengthParameterization;
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "total approx dist: " << bezierArclengthDistance);
    }

    void NavigableGapGenerator::buildExtendedGapOrigin(const int & numRGEPoints,
                                                        const Eigen::Vector2d & extendedGapOrigin,
                                                        const Eigen::Vector2d & bezierOrigin,
                                                        Eigen::MatrixXd & curvePosns,
                                                        Eigen::MatrixXd & curveVels,
                                                        Eigen::MatrixXd & curveInwardNorms,
                                                        const bool & left)
    {
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               [buildExtendedGapOrigin()]");
        Eigen::Vector2d currPt(0.0, 0.0);
        Eigen::Vector2d currVel(0.0, 0.0);
        Eigen::Vector2d currInwardVector(0.0, 0.0);
        Eigen::Vector2d currInwardNorm(0.0, 0.0);
        float s = 0.0;

        Eigen::Matrix2d rotMat;
        if (left)
            rotMat << Rnegpi2(0, 0), Rnegpi2(0, 1), Rnegpi2(1, 0), Rnegpi2(1, 1);
        else
            rotMat << Rpi2(0, 0), Rpi2(0, 1), Rpi2(1, 0), Rpi2(1, 1);

        for (float i = 0; i < numRGEPoints; i++) 
        {
            s = epsilonDivide(i, numRGEPoints);
            // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "s_left_rge: " << s);

            // line equation from gapExtendedOrigin to bezierOrigin
            currPt = (1 - s) * extendedGapOrigin + s * bezierOrigin;
            currVel = (bezierOrigin - extendedGapOrigin);
            currInwardVector = rotMat * currVel;
            currInwardNorm = unitNorm(currInwardVector);
            curvePosns.row(i) = currPt;
            curveVels.row(i) = currVel;
            curveInwardNorms.row(i) = currInwardNorm;
        }
    }

    void NavigableGapGenerator::buildBezierCurve(const int & numRGEPoints,
                                                  const int & totalNumCurvePts,
                                                  const Eigen::VectorXd & arclengthParameters,
                                                  const Eigen::Vector2d & bezierOrigin,
                                                  const Eigen::Vector2d & bezierInitialPt,
                                                  const Eigen::Vector2d & bezierTerminalPt,
                                                  Eigen::MatrixXd & curvePosns,
                                                  Eigen::MatrixXd & curveVels,
                                                  Eigen::MatrixXd & curveInwardNorms,
                                                  const bool & left)
    {
        Eigen::Vector2d currPt(0.0, 0.0);
        Eigen::Vector2d currVel(0.0, 0.0);
        Eigen::Vector2d currInwardVector(0.0, 0.0);
        Eigen::Vector2d currInwardNorm(0.0, 0.0);
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

            // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "s_left: " << s_left);
            currPt = (1 - s) * (1 - s) * bezierOrigin + 
                        2*(1 - s)*s * bezierInitialPt + 
                        s*s * bezierTerminalPt;
            currVel = (2*s - 2) * bezierOrigin + 
                        (2 - 4*s) * bezierInitialPt + 
                        2*s * bezierTerminalPt;
            currInwardVector = rotMat * currVel;
            currInwardNorm = unitNorm(currInwardVector);
            curvePosns.row(i) = currPt;
            curveVels.row(i) = currVel;
            curveInwardNorms.row(i) = currInwardNorm;
        }
    }

    void NavigableGapGenerator::buildExtendedBezierCurve(dynamic_gap::Gap * selectedGap, 
                                                            // Eigen::MatrixXd & gapCurvesPosns,
                                                            // Eigen::MatrixXd & gapCurvesInwardNorms, 
                                                            // Eigen::MatrixXd & gapSideAHPFCenters, Eigen::MatrixXd & allAHPFCenters,
                                                            const Eigen::Vector2d & leftGapPtVel, 
                                                            const Eigen::Vector2d & rightGapPtVel, 
                                                            const Eigen::Vector2d & maxRbtVel,
                                                            const Eigen::Vector2d & leftCurveInitPt, 
                                                            const Eigen::Vector2d & leftCurveTermPt, 
                                                            const Eigen::Vector2d & rightCurveInitPt, 
                                                            const Eigen::Vector2d & rightCurveTermPt, 
                                                            const Eigen::Vector2d & gapGoalTermPt, 
                                                            const float & numCurvePts) // const Eigen::Vector2d & initRbtPos 
    {  
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            [buildExtendedBezierCurve()]");
        // Eigen::MatrixXd leftCurveVels, rightCurveVels, leftCurveInwardNorms, rightCurveInwardNorms,
        //                 leftCurvePosns, rightCurvePosns;

        // convert from 2f to 2d
        Eigen::Vector2d extendedGapOrigin(selectedGap->extendedGapOrigin_[0],
                                          selectedGap->extendedGapOrigin_[1]);
        Eigen::Vector2d leftBezierOrigin(selectedGap->leftBezierOrigin_[0],
                                            selectedGap->leftBezierOrigin_[1]);
        Eigen::Vector2d rightBezierOrigin(selectedGap->rightBezierOrigin_[0],
                                            selectedGap->rightBezierOrigin_[1]);

        selectedGap->leftBezierWeight_ = epsilonDivide(leftGapPtVel.norm(), maxRbtVel.norm()); // capped at 1, we can scale down towards 0 until initial constraints are met?
        selectedGap->rightBezierWeight_ = epsilonDivide(rightGapPtVel.norm(), maxRbtVel.norm());

        // for a totally static gap, can get no velocity on first bezier curve point which corrupts vector field
        Eigen::Vector2d weightedLeftBezierPt0(0.0, 0.0);
        Eigen::Vector2d weightedRightBezierPt0(0.0, 0.0);
        if (leftGapPtVel.norm() > 0.0)
            weightedLeftBezierPt0 = leftBezierOrigin + selectedGap->leftBezierWeight_ * (leftCurveInitPt - leftBezierOrigin);
        else
            weightedLeftBezierPt0 = (0.95 * leftBezierOrigin + 0.05 * leftCurveTermPt);

        if (rightGapPtVel.norm() >  0.0)
            weightedRightBezierPt0 = rightBezierOrigin + selectedGap->rightBezierWeight_ * (rightCurveInitPt - rightBezierOrigin);
        else
            weightedRightBezierPt0 = (0.95 * rightBezierOrigin + 0.05 * rightCurveTermPt);
        
        // set left and right bezier points
        selectedGap->leftPt0_ = weightedLeftBezierPt0;
        selectedGap->leftPt1_ = leftCurveTermPt;
        selectedGap->rightPt0_ = weightedRightBezierPt0;
        selectedGap->rightPt1_ = rightCurveTermPt;  

        float desiredLeftBezierPtToPtDistance = 0.01; // will be set in arclengthParameterizeBezier  
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               arclength parameterizing left curve");
        Eigen::VectorXd leftArclengthParameters = arclengthParameterizeBezier(leftBezierOrigin, weightedLeftBezierPt0, leftCurveTermPt, numCurvePts, desiredLeftBezierPtToPtDistance);        
        
        selectedGap->numLeftRGEPoints_ = (cfg_->gap_manip.radial_extend) ? std::max(int(std::ceil( (extendedGapOrigin - leftBezierOrigin).norm() / desiredLeftBezierPtToPtDistance)), 2) : 0;

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "numLeftRGEPoints: " << numLeftRGEPoints);

        int totalNumLeftCurvePts = selectedGap->numLeftRGEPoints_ + numCurvePts; 
        Eigen::MatrixXd leftCurvePosns = Eigen::MatrixXd(totalNumLeftCurvePts, 2);
        Eigen::MatrixXd leftCurveVels = Eigen::MatrixXd(totalNumLeftCurvePts, 2);
        Eigen::MatrixXd leftCurveInwardNorms = Eigen::MatrixXd(totalNumLeftCurvePts, 2);

        ///////////////////////////////
        // Left radial gap extension //
        ///////////////////////////////
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "   building left extended gap origin");
        buildExtendedGapOrigin(selectedGap->numLeftRGEPoints_, extendedGapOrigin, 
                                leftBezierOrigin, leftCurvePosns, leftCurveVels, leftCurveInwardNorms, true);

        ///////////////////////
        // Left Bezier curve //
        ///////////////////////
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "   building left bezier curve");
        buildBezierCurve(selectedGap->numLeftRGEPoints_, totalNumLeftCurvePts, leftArclengthParameters, 
                         leftBezierOrigin, selectedGap->leftPt0_, selectedGap->leftPt1_, leftCurvePosns, leftCurveVels, leftCurveInwardNorms, true);        

        float desiredRightBezierPtToPtDistance = 0.01;
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               arclength parameterizing right curve");
        Eigen::VectorXd rightArclengthParameters = arclengthParameterizeBezier(rightBezierOrigin, weightedRightBezierPt0, rightCurveTermPt, numCurvePts, desiredRightBezierPtToPtDistance);

        selectedGap->numRightRGEPoints_ = (cfg_->gap_manip.radial_extend) ? std::max(int(std::ceil( (extendedGapOrigin - rightBezierOrigin).norm() / desiredRightBezierPtToPtDistance)), 2) : 0;

        int totalNumRightCurvePts = selectedGap->numRightRGEPoints_ + numCurvePts; 
        Eigen::MatrixXd rightCurvePosns = Eigen::MatrixXd(totalNumRightCurvePts, 2);            
        Eigen::MatrixXd rightCurveVels = Eigen::MatrixXd(totalNumRightCurvePts, 2);
        Eigen::MatrixXd rightCurveInwardNorms = Eigen::MatrixXd(totalNumRightCurvePts, 2);
        
        ////////////////////////////////
        // Right radial gap extension //
        ////////////////////////////////
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "   building right extended gap origin");
        buildExtendedGapOrigin(selectedGap->numRightRGEPoints_, extendedGapOrigin, 
                                rightBezierOrigin, rightCurvePosns, rightCurveVels, rightCurveInwardNorms, false);
             
        ////////////////////////
        // Right Bezier curve //
        ////////////////////////
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "   building right bezier curve");
        buildBezierCurve(selectedGap->numRightRGEPoints_, totalNumRightCurvePts, rightArclengthParameters, 
                            rightBezierOrigin, selectedGap->rightPt0_, selectedGap->rightPt1_, rightCurvePosns, rightCurveVels, rightCurveInwardNorms, false);        

        float offset = 0.01;

        selectedGap->allCurvePts_ = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
        selectedGap->gapBoundaryInwardNorms_ = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
        selectedGap->leftRightCenters_ = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
        selectedGap->allAHPFCenters_ = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts + 1, 2);

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "centered origin inward norm: " << centered_origin_inward_norm[0] << ", " << centered_origin_inward_norm[1]);
        selectedGap->allCurvePts_ << leftCurvePosns, rightCurvePosns; 
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesPosns worked");
        selectedGap->gapBoundaryInwardNorms_ << leftCurveInwardNorms, rightCurveInwardNorms; // centered_origin_inward_norm, 
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesInwardNorms worked");
        selectedGap->leftRightCenters_ = selectedGap->allCurvePts_ - selectedGap->gapBoundaryInwardNorms_*offset;
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapSideAHPFCenters worked");
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "left_curve points: " << left_curve);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "right_curve_points: " << right_curve);

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "left_curve inward norms: " << leftCurveInwardNorms);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "right_curve inward_norms: " << rightCurveInwardNorms);

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesInwardNorms: " << gapCurvesInwardNorms);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "allAHPFCenters: " << allAHPFCenters);

        Eigen::Matrix<double, 1, 2> goal;
        goal << gapGoalTermPt[0], gapGoalTermPt[1];
        selectedGap->allAHPFCenters_ << goal, selectedGap->leftRightCenters_;
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "allAHPFCenters worked");   

        // selectedGap->allAHPFCenters_ = allAHPFCenters;
        // selectedGap->leftRightCenters_ = gapSideAHPFCenters;
        // selectedGap->allCurvePts_ = gapCurvesPosns;        
    }
}