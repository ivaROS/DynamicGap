#include <dynamic_gap/trajectory_generation/NavigableGapGenerator.h>

namespace dynamic_gap 
{
    void NavigableGapGenerator::generateNavigableGap(dynamic_gap::Gap * gap)
    {
        int numCurvePts = cfg_->traj.num_curve_points;

        // get gap points in cartesian
        float xLeft = 0.0, yLeft = 0.0, xRight = 0.0, yRight = 0.0;
        gap->getLCartesian(xLeft, yLeft);
        gap->getRCartesian(xRight, yRight);

        float xLeftTerm = 0.0, yLeftTerm = 0.0, xRightTerm = 0.0, yRightTerm = 0.0;
        gap->getLCartesian(xLeftTerm, yLeftTerm);
        gap->getRCartesian(xRightTerm, yRightTerm);

        Eigen::Vector2d initialGoal(gap->goal.x_, gap->goal.y_);
        Eigen::Vector2d terminalGoal(gap->terminalGoal.x_, gap->terminalGoal.y_);

        // float initialGoalX = initialGoal[0];
        // float initialGoalY = initialGoal[1];
        float terminalGoalX = terminalGoal[0];
        float terminalGoalY = terminalGoal[1];

        // float goalVelX = epsilonDivide(terminalGoalX - initialGoalX, gap->gapLifespan_); // absolute velocity (not relative to robot)
        // float goalVelY = epsilonDivide(terminalGoalY - initialGoalY, gap->gapLifespan_);

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual initial robot pos: (" << rbtState[0] << ", " << rbtState[1] << ")");
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual inital robot velocity: " << rbtState[2] << ", " << rbtState[3] << ")");
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual initial left point: (" << xLeft << ", " << yLeft << "), actual initial right point: (" << xRight << ", " << yRight << ")"); 
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual terminal left point: (" << xLeftTerm << ", " << yLeftTerm << "), actual terminal right point: (" << xRightTerm << ", " << yRightTerm << ")");
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual initial goal: (" << initialGoalX << ", " << initialGoalY << ")"); 
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            actual terminal goal: (" << terminalGoalX << ", " << terminalGoalY << ")"); 
        
        float leftVelX = epsilonDivide(xLeftTerm - xLeft, gap->gapLifespan_);
        float leftVelY = epsilonDivide(yLeftTerm - yLeft, gap->gapLifespan_);

        float rightVelX = epsilonDivide(xRightTerm - xRight, gap->gapLifespan_);
        float rightVelY = epsilonDivide(yRightTerm - yRight, gap->gapLifespan_);
        
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "pre-integration, x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);

        // Eigen::Vector2d initRbtPos(x[0], x[1]);
        Eigen::Vector2d leftCurveInitPt(xLeft, yLeft);
        Eigen::Vector2d leftCurveTermPt(xLeftTerm, yLeftTerm);
        Eigen::Vector2d rightCurveInitPt(xRight, yRight);
        Eigen::Vector2d rightCurveTermPt(xRightTerm, yRightTerm);
        Eigen::Vector2d leftGapPtVel(leftVelX, leftVelY);
        Eigen::Vector2d rightGapPtVel(rightVelX, rightVelY);
        Eigen::Vector2d gapGoalTermPt(terminalGoalX, terminalGoalY);
        // Eigen::Vector2d gapGoalVel(goalVelX, goalVelY);
        
        Eigen::Vector2d maxRbtVel(cfg_->control.vx_absmax, cfg_->control.vy_absmax);
        // Eigen::Vector2d maxRbtAcc(cfg_->control.ax_absmax, cfg_->control.ay_absmax);

        // Eigen::MatrixXd gapCurvesPosns, gapCurvesInwardNorms, 
        //                 gapSideAHPFCenters, allAHPFCenters;
        
        // THIS IS BUILT WITH EXTENDED POINTS. 
        // std::chrono::steady_clock::time_point buildExtendedBezierCurveStartTime = std::chrono::steady_clock::now();
        // buildExtendedBezierCurve(gap, leftGapPtVel, rightGapPtVel, maxRbtVel, 
        //                             leftCurveInitPt, leftCurveTermPt, rightCurveInitPt, rightCurveTermPt, 
        //                             gapGoalTermPt, numCurvePts); // initRbtPos
        // float buildExtendedBezierCurveTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - buildExtendedBezierCurveStartTime).count() / 1.0e6;
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            buildExtendedBezierCurve time taken: " << buildExtendedBezierCurveTime << " seconds");

        // gap->leftBezierWeight_ = epsilonDivide(leftGapPtVel.norm(), maxRbtVel.norm()); // capped at 1, we can scale down towards 0 until initial constraints are met?
        // gap->rightBezierWeight_ = epsilonDivide(rightGapPtVel.norm(), maxRbtVel.norm());

        // // for a totally static gap, can get no velocity on first bezier curve point which corrupts vector field
        // Eigen::Vector2d weightedLeftBezierPt0(0.0, 0.0);
        // Eigen::Vector2d weightedRightBezierPt0(0.0, 0.0);
        // if (leftGapPtVel.norm() > 0.0)
        //     weightedLeftBezierPt0 = gap->leftBezierOrigin_ + gap->leftBezierWeight_ * (leftCurveInitPt - leftBezierOrigin);
        // else
        //     weightedLeftBezierPt0 = (0.95 * gap->leftBezierOrigin_ + 0.05 * leftCurveTermPt);

        // if (rightGapPtVel.norm() >  0.0)
        //     weightedRightBezierPt0 = gap->rightBezierOrigin_ + gap->rightBezierWeight_ * (rightCurveInitPt - rightBezierOrigin);
        // else
        //     weightedRightBezierPt0 = (0.95 * gap->rightBezierOrigin_ + 0.05 * rightCurveTermPt);
        
        // set left and right bezier points
        gap->leftPt0_ = leftCurveInitPt;
        gap->leftPt1_ = leftCurveTermPt;
        gap->rightPt0_ = rightCurveInitPt;
        gap->rightPt1_ = rightCurveTermPt;  


        Eigen::VectorXd left_indices, arc_indices, right_indices;
        float des_pt_to_pt_arclength = arclengthParameterizeBoundary(gap, left_indices, arc_indices, right_indices, gapGoalTermPt); 

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "after buildExtendedBezierCurve, left weight: " << leftBezierWeight << ", rightBezierWeight: " << rightBezierWeight);
    }

    float NavigableGapGenerator::arclengthParameterizeBoundary(dynamic_gap::Gap * gap,
                                                                Eigen::VectorXd & left_indices,
                                                                Eigen::VectorXd & arc_indices,
                                                                Eigen::VectorXd & right_indices,
                                                                const Eigen::Vector2d & gapGoalTermPt)
    {
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        [arclengthParameterizeBoundary()]: ");
        int numCurvePts = cfg_->traj.num_curve_points;

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        leftBezierOrigin_: " << gap->leftBezierOrigin_.transpose());
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        leftPt0_: " << gap->leftPt0_.transpose());
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        leftPt1_: " << gap->leftPt1_.transpose());

        // calculate left curve arclength
        float leftBezierOriginTheta = std::atan2(gap->leftBezierOrigin_[1], gap->leftBezierOrigin_[0]);
        float leftBezierArclength = approximateBezierArclength(gap->leftBezierOrigin_,
                                                                gap->leftPt0_,
                                                                gap->leftPt1_,
                                                                0.0,
                                                                1.0,
                                                                250);
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        leftBezierArclength: " << leftBezierArclength);


        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        rightBezierOrigin_: " << gap->rightBezierOrigin_.transpose());
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        rightPt0_: " << gap->rightPt0_.transpose());
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        rightPt1_: " << gap->rightPt1_.transpose());

        // calculate right curve arclength
        float rightBezierOriginTheta = std::atan2(gap->rightBezierOrigin_[1], gap->rightBezierOrigin_[0]);
        float rightBezierArclength = approximateBezierArclength(gap->rightBezierOrigin_,
                                                                gap->rightPt0_,
                                                                gap->rightPt1_,
                                                                0.0,
                                                                1.0,
                                                                250);        
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        rightBezierArclength: " << rightBezierArclength);

        // calculate extended arc arclength
        float d_safe = std::min(gap->leftBezierOrigin_.norm(), gap->rightBezierOrigin_.norm());
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        d_safe: " << d_safe);

        float arc_angle = 0.0;
        if (rightBezierOriginTheta > leftBezierOriginTheta)
            arc_angle = (rightBezierOriginTheta - leftBezierOriginTheta);
        else
            arc_angle = 2*M_PI + (rightBezierOriginTheta - leftBezierOriginTheta);

        float arc_arclength = d_safe * arc_angle;

        float boundaryArclength = leftBezierArclength + rightBezierArclength + arc_arclength;
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "        boundaryArclength: " << boundaryArclength);

        float des_boundary_pt_to_pt_arclength = boundaryArclength / (numCurvePts - 1);
        float bezierPtToPtDistanceThresh = des_boundary_pt_to_pt_arclength / 1000;

        float remaining_snippet_arclength = 0.0;

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            ALP-ing left curve:");
        left_indices = arclengthParameterizeBezier(gap->leftBezierOrigin_,
                                                    gap->leftPt0_,
                                                    gap->leftPt1_,
                                                    numCurvePts,
                                                    des_boundary_pt_to_pt_arclength,
                                                    bezierPtToPtDistanceThresh,
                                                    remaining_snippet_arclength, 
                                                    true);

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            left_indices: " << left_indices);

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            ALP-ing arc curve:");
        arc_indices = arclengthParameterizeArc(leftBezierOriginTheta,
                                                rightBezierOriginTheta,
                                                d_safe,
                                                des_boundary_pt_to_pt_arclength,
                                                remaining_snippet_arclength);

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            arc_indices: " << arc_indices);

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            ALP-ing right curve:");
        right_indices = arclengthParameterizeBezier(gap->rightBezierOrigin_,
                                                    gap->rightPt0_,
                                                    gap->rightPt1_,
                                                    numCurvePts,
                                                    des_boundary_pt_to_pt_arclength,
                                                    bezierPtToPtDistanceThresh,
                                                    remaining_snippet_arclength, 
                                                    false);        

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            right_indices: " << right_indices);

        gap->allCurvePts_ = Eigen::MatrixXd(numCurvePts, 2);
        gap->gapBoundaryInwardNorms_ = Eigen::MatrixXd(numCurvePts, 2);
        gap->leftRightCenters_ = Eigen::MatrixXd(numCurvePts, 2);
        gap->allAHPFCenters_ = Eigen::MatrixXd(numCurvePts + 1, 2);

        Eigen::MatrixXd leftCurvePosns = Eigen::MatrixXd(left_indices.rows(), 2);
        // Eigen::MatrixXd leftCurveVels = Eigen::MatrixXd(left_indices.rows(), 2);
        Eigen::MatrixXd leftCurveInwardNorms = Eigen::MatrixXd(left_indices.rows(), 2);

        buildBezierCurve(left_indices, 
                         gap->leftBezierOrigin_, gap->leftPt0_, gap->leftPt1_, 
                         leftCurvePosns, leftCurveInwardNorms, true);        

        Eigen::MatrixXd arcCurvePosns = Eigen::MatrixXd(arc_indices.rows(), 2);
        // Eigen::MatrixXd arcCurveVels = Eigen::MatrixXd(arc_indices.rows(), 2);
        Eigen::MatrixXd arcCurveInwardNorms = Eigen::MatrixXd(arc_indices.rows(), 2);

        buildArcCurve(arc_indices,
                        leftBezierOriginTheta,
                        rightBezierOriginTheta,
                        d_safe,
                        arcCurvePosns,
                        arcCurveInwardNorms);

        Eigen::MatrixXd rightCurvePosns = Eigen::MatrixXd(right_indices.rows(), 2);
        // Eigen::MatrixXd rightCurveVels = Eigen::MatrixXd(right_indices.rows(), 2);
        Eigen::MatrixXd rightCurveInwardNorms = Eigen::MatrixXd(right_indices.rows(), 2);

        buildBezierCurve(right_indices, 
                         gap->rightBezierOrigin_, gap->rightPt0_, gap->rightPt1_, 
                         rightCurvePosns, rightCurveInwardNorms, true);        

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "centered origin inward norm: " << centered_origin_inward_norm[0] << ", " << centered_origin_inward_norm[1]);
        gap->allCurvePts_ << leftCurvePosns, arcCurvePosns, rightCurvePosns; 
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesPosns worked");
        gap->gapBoundaryInwardNorms_ << leftCurveInwardNorms, arcCurveInwardNorms, rightCurveInwardNorms; // centered_origin_inward_norm, 
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesInwardNorms worked");
        
        double offset = des_boundary_pt_to_pt_arclength;
        gap->leftRightCenters_ = gap->allCurvePts_ - gap->gapBoundaryInwardNorms_*offset;
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapSideAHPFCenters worked");
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "left_curve points: " << left_curve);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "right_curve_points: " << right_curve);

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "left_curve inward norms: " << leftCurveInwardNorms);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "right_curve inward_norms: " << rightCurveInwardNorms);

        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesInwardNorms: " << gapCurvesInwardNorms);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "allAHPFCenters: " << allAHPFCenters);

        Eigen::Matrix<double, 1, 2> goal;
        goal << gapGoalTermPt[0], gapGoalTermPt[1];
        gap->allAHPFCenters_ << goal, gap->leftRightCenters_;

        return des_boundary_pt_to_pt_arclength;
    }

    Eigen::VectorXd NavigableGapGenerator::arclengthParameterizeArc(const float & leftBezierOriginTheta,
                                                                    const float & rightBezierOriginTheta,
                                                                    const float & d_safe,
                                                                    const float & desiredBezierPtToPtDistance,
                                                                    float & prior_snippet_arclength)
    {
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               [arclengthParameterizeArc()]");
        // Eigen::VectorXd arclengthParameterization;
        // int parameterizationCounter = 0;
        std::vector<double> arclengthParameterization;

        float desiredBezierPtToPtDist = desiredBezierPtToPtDistance - prior_snippet_arclength;

        float theta_des = desiredBezierPtToPtDist / d_safe;

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   desiredBezierPtToPtDist: " << desiredBezierPtToPtDist);

        float theta_curr = 0.0;

        float theta_total = 0.0;
        if (rightBezierOriginTheta > leftBezierOriginTheta)
            theta_total = (rightBezierOriginTheta - leftBezierOriginTheta);
        else
            theta_total = 2*M_PI + (rightBezierOriginTheta - leftBezierOriginTheta);
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   theta_total: " << theta_total);

        while (true)
        {
            if ((theta_curr + theta_des) <= theta_total)
            {
                arclengthParameterization.push_back( (theta_curr + theta_des) / theta_total);
                ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   adding " << (theta_curr + theta_des) / theta_total);

                theta_curr += theta_des;

                if (arclengthParameterization.size() == 1)
                {
                    desiredBezierPtToPtDist = desiredBezierPtToPtDistance; //  + prior_snippet_arclength;
                    theta_des = desiredBezierPtToPtDist / d_safe;
                }
            } else
            {
                prior_snippet_arclength = (1.0 - arclengthParameterization.at(arclengthParameterization.size() - 1)) * (theta_total * d_safe);
                ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   prior_snippet_arclength: " << prior_snippet_arclength);
                break;
            }
        }

        Eigen::VectorXd arclengthParameterizationEigen = Eigen::MatrixXd::Zero(arclengthParameterization.size(), 1);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "uniform indices: ");
        for (int i = 0; i < arclengthParameterizationEigen.rows(); i++)
        {
            arclengthParameterizationEigen(i, 0) = arclengthParameterization.at(i);
        }
        return arclengthParameterizationEigen;
    }

    Eigen::VectorXd NavigableGapGenerator::arclengthParameterizeBezier(const Eigen::Vector2d & bezierPt0, 
                                                                        const Eigen::Vector2d & bezierPt1, 
                                                                        const Eigen::Vector2d & bezierPt2, 
                                                                        const float & numCurvePts,
                                                                        const float & desiredBezierPtToPtDistance,
                                                                        const float & bezierPtToPtDistanceThresh,
                                                                        float & prior_snippet_arclength,
                                                                        const bool & left) 
    {
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               [arclengthParameterizeBezier()]");
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   bezierPt0: " << bezierPt0[0] << ", " << bezierPt0[1]);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   bezierPt1: " << bezierPt1[0] << ", " << bezierPt1[1]);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   bezierPt2: " << bezierPt2[0] << ", " << bezierPt2[1]);        

        std::vector<double> arclengthParameterization;
        // int arclengthParameterizationIdx = 0;
        if (left)
        {
            arclengthParameterization.push_back(1.0);
            // arclengthParameterization(0, 0) = 1.0;
            // arclengthParameterizationIdx += 1;
        }
        
        float desiredBezierPtToPtDist = desiredBezierPtToPtDistance - prior_snippet_arclength;

        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   number of points: " << numCurvePts);
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   desiredBezierPtToPtDist: " << desiredBezierPtToPtDist);

        int interpMaxIter = 100;
        int interpIter = 0;
        float numBezierPtToPtIntegrationPoints = 5.0;
        float numBezierSamplePts = 2 * numCurvePts;

        float t_interval = epsilonDivide(1.0, numBezierSamplePts);

        float start_val = 0.0, end_val = 0.0, inc_val = 0.0;
        if (left)
        {
            start_val = 1.0;
            end_val = 0.0;
            inc_val = -t_interval;
        } else
        {
            start_val = 0.0;
            end_val = 1.0;
            inc_val = t_interval;
        }

        float t_interp = 0.0, t_lowerBound = 0.0, t_upperBound = 0.0, 
              currentBezierPtToPtArclengthDistance = 0.0, intermediateBezierPtToPtArclengthDistance = 0.0;

        float t_kmin1 = start_val;
        
        for (float t_k = start_val; (left ? t_k >= end_val : t_k <= end_val); t_k += inc_val) 
        {
            currentBezierPtToPtArclengthDistance = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                                t_kmin1, t_k, numBezierPtToPtIntegrationPoints);
            ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   from " << t_kmin1 << " to " << t_k << ": " << currentBezierPtToPtArclengthDistance << ", desired distance: " << desiredBezierPtToPtDist);

            if (std::abs(currentBezierPtToPtArclengthDistance - desiredBezierPtToPtDist) < bezierPtToPtDistanceThresh) 
            {
                    // arclengthParameterization.resize(arclengthParameterizationIdx+1);
                    // arclengthParameterization(arclengthParameterizationIdx, 0) = t_k;
                    // arclengthParameterization << arclengthParameterization, t_k;
                    arclengthParameterization.push_back(t_k);
                    ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   adding " << t_k);
                    // arclengthParameterizationIdx += 1;

            } else if (currentBezierPtToPtArclengthDistance > desiredBezierPtToPtDist) 
            {
                t_interp = (t_kmin1 + t_k) / 2.0;
                intermediateBezierPtToPtArclengthDistance = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                                        t_kmin1, t_interp, numBezierPtToPtIntegrationPoints);

                ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   overshot, from " << t_kmin1 << " to " << t_interp << ": " << intermediateBezierPtToPtArclengthDistance << ", desired distance: " << desiredBezierPtToPtDist);
                t_lowerBound = t_kmin1;
                t_upperBound = t_k;

                interpIter = 0;                
                while (abs(intermediateBezierPtToPtArclengthDistance - desiredBezierPtToPtDist) > bezierPtToPtDistanceThresh && interpIter < interpMaxIter) 
                {
                    if (intermediateBezierPtToPtArclengthDistance < desiredBezierPtToPtDist) 
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
                    intermediateBezierPtToPtArclengthDistance = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                                            t_kmin1, t_interp, numBezierPtToPtIntegrationPoints);
                    ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   overshot, from " << t_kmin1 << " to " << t_interp << ": " << intermediateBezierPtToPtArclengthDistance << ", desired distance: " << desiredBezierPtToPtDist);
                    interpIter++;
                }
                
                // if (arclengthParameterizationIdx < arclengthParameterization.rows())
                // {               
                    // arclengthParameterization.resize(arclengthParameterizationIdx+1);
                    // arclengthParameterization(arclengthParameterizationIdx, 0) = t_interp;
                arclengthParameterization.push_back(t_interp);
                ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   adding " << t_interp);
                    // arclengthParameterizationIdx += 1;
                // }
            } else
            {
                continue;
            }

            t_kmin1 = arclengthParameterization.at(arclengthParameterization.size() - 1); // arclengthParameterization(arclengthParameterizationIdx - 1, 0);
            if (arclengthParameterization.size() == 1)
                desiredBezierPtToPtDist = desiredBezierPtToPtDistance; // + prior_snippet_arclength;
        }        

        if (left)
        {
            prior_snippet_arclength = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                    arclengthParameterization.at(arclengthParameterization.size() - 1),
                                                                    0.0,
                                                                    1000);
        } else
        {
            prior_snippet_arclength = approximateBezierArclength(bezierPt0, bezierPt1, bezierPt2, 
                                                                    arclengthParameterization.at(arclengthParameterization.size() - 1),
                                                                    1.0,
                                                                    1000);
            if (std::abs(prior_snippet_arclength - desiredBezierPtToPtDist) < 0.1 * bezierPtToPtDistanceThresh)
                arclengthParameterization.push_back(1.0);
        }
        ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "                   prior_snippet_arclength: " << prior_snippet_arclength);
        
        Eigen::VectorXd arclengthParameterizationEigen = Eigen::MatrixXd::Zero(arclengthParameterization.size(), 1);
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "uniform indices: ");
        for (int i = 0; i < arclengthParameterizationEigen.rows(); i++)
        {
            arclengthParameterizationEigen(i, 0) = arclengthParameterization.at(i);
            // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", arclengthParameterization(i, 0));
            // if (i > 0)
            // {
            //     if (arclengthParameterizationEigen(i, 0) <= arclengthParameterizationEigen(i - 1, 0))
            //         ROS_WARN_STREAM_NAMED("NavigableGapGenerator", "       BAD PARAMETERIZATION");
            // }
        }
         
        return arclengthParameterizationEigen;
        // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "total approx dist: " << bezierArclengthDistance);
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

    // void NavigableGapGenerator::buildExtendedGapOrigin(const int & numRGEPoints,
    //                                                     const Eigen::Vector2d & extendedGapOrigin,
    //                                                     const Eigen::Vector2d & bezierOrigin,
    //                                                     Eigen::MatrixXd & curvePosns,
    //                                                     Eigen::MatrixXd & curveVels,
    //                                                     Eigen::MatrixXd & curveInwardNorms,
    //                                                     const bool & left)
    // {
    //     ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               [buildExtendedGapOrigin()]");
    //     Eigen::Vector2d currPt(0.0, 0.0);
    //     Eigen::Vector2d currVel(0.0, 0.0);
    //     Eigen::Vector2d currInwardVector(0.0, 0.0);
    //     Eigen::Vector2d currInwardNorm(0.0, 0.0);
    //     float s = 0.0;

    //     Eigen::Matrix2d rotMat;
    //     if (left)
    //         rotMat << Rnegpi2(0, 0), Rnegpi2(0, 1), Rnegpi2(1, 0), Rnegpi2(1, 1);
    //     else
    //         rotMat << Rpi2(0, 0), Rpi2(0, 1), Rpi2(1, 0), Rpi2(1, 1);

    //     for (float i = 0; i < numRGEPoints; i++) 
    //     {
    //         s = epsilonDivide(i, numRGEPoints);
    //         // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "s_left_rge: " << s);

    //         // line equation from gapExtendedOrigin to bezierOrigin
    //         currPt = (1 - s) * extendedGapOrigin + s * bezierOrigin;
    //         currVel = (bezierOrigin - extendedGapOrigin);
    //         currInwardVector = rotMat * currVel;
    //         currInwardNorm = unitNorm(currInwardVector);
    //         curvePosns.row(i) = currPt;
    //         curveVels.row(i) = currVel;
    //         curveInwardNorms.row(i) = currInwardNorm;
    //     }
    // }

    void NavigableGapGenerator::buildArcCurve(const Eigen::VectorXd & arclengthParameters,
                                                const float & leftBezierOriginTheta,
                                                const float & rightBezierOriginTheta,
                                                const float & d_safe,
                                                Eigen::MatrixXd & curvePosns,
                                                // Eigen::MatrixXd & curveVels,
                                                Eigen::MatrixXd & curveInwardNorms)
    {
        Eigen::Vector2d currPt(0.0, 0.0);
        // Eigen::Vector2d currVel(0.0, 0.0);
        Eigen::Vector2d currInwardVector(0.0, 0.0);
        Eigen::Vector2d currInwardNorm(0.0, 0.0);
        float s = 0.0;

        float theta_total = 0.0;
        if (rightBezierOriginTheta > leftBezierOriginTheta)
            theta_total = (rightBezierOriginTheta - leftBezierOriginTheta);
        else
            theta_total = 2*M_PI + (rightBezierOriginTheta - leftBezierOriginTheta);

        for (int i = 0; i < arclengthParameters.size(); i++) 
        {
            s = arclengthParameters(i, 0);

            currPt = Eigen::Vector2d(d_safe * std::cos(leftBezierOriginTheta + theta_total * s),
                                     d_safe * std::sin(leftBezierOriginTheta + theta_total * s));

            currInwardVector = -currPt;
            currInwardNorm = unitNorm(currInwardVector);
            curvePosns.row(i) = currPt;
            // curveVels.row(i) = currVel;
            curveInwardNorms.row(i) = currInwardNorm;            
        }
    }

    void NavigableGapGenerator::buildBezierCurve(const Eigen::VectorXd & arclengthParameters,
                                                  const Eigen::Vector2d & bezierOrigin,
                                                  const Eigen::Vector2d & bezierInitialPt,
                                                  const Eigen::Vector2d & bezierTerminalPt,
                                                  Eigen::MatrixXd & curvePosns,
                                                //   Eigen::MatrixXd & curveVels,
                                                  Eigen::MatrixXd & curveInwardNorms,
                                                  const bool & left)
    {
        Eigen::Vector2d currPt(0.0, 0.0);
        Eigen::Vector2d currVel(0.0, 0.0);
        Eigen::Vector2d currInwardVector(0.0, 0.0);
        Eigen::Vector2d currInwardNorm(0.0, 0.0);
        float s = 0.0;
        // float eps = 0.0000001;

        Eigen::Matrix2d rotMat;
        if (left)
            rotMat << Rnegpi2(0, 0), Rnegpi2(0, 1), Rnegpi2(1, 0), Rnegpi2(1, 1);
        else
            rotMat << Rpi2(0, 0), Rpi2(0, 1), Rpi2(1, 0), Rpi2(1, 1);
        
        // int counter = 0;
        for (int i = 0; i < arclengthParameters.size(); i++) 
        {
            s = arclengthParameters(i, 0);
            // counter++;

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
            // curveVels.row(i) = currVel;
            curveInwardNorms.row(i) = currInwardNorm;
        }
    }
    
    // void NavigableGapGenerator::buildExtendedBezierCurve(dynamic_gap::Gap * gap, 
    //                                                         // Eigen::MatrixXd & gapCurvesPosns,
    //                                                         // Eigen::MatrixXd & gapCurvesInwardNorms, 
    //                                                         // Eigen::MatrixXd & gapSideAHPFCenters, Eigen::MatrixXd & allAHPFCenters,
    //                                                         const Eigen::Vector2d & leftGapPtVel, 
    //                                                         const Eigen::Vector2d & rightGapPtVel, 
    //                                                         const Eigen::Vector2d & maxRbtVel,
    //                                                         const Eigen::Vector2d & leftCurveInitPt, 
    //                                                         const Eigen::Vector2d & leftCurveTermPt, 
    //                                                         const Eigen::Vector2d & rightCurveInitPt, 
    //                                                         const Eigen::Vector2d & rightCurveTermPt, 
    //                                                         const Eigen::Vector2d & gapGoalTermPt, 
    //                                                         const float & numCurvePts) // const Eigen::Vector2d & initRbtPos 
    // {  
    //     ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "            [buildExtendedBezierCurve()]");
    //     // Eigen::MatrixXd leftCurveVels, rightCurveVels, leftCurveInwardNorms, rightCurveInwardNorms,
    //     //                 leftCurvePosns, rightCurvePosns;

    //     // convert from 2f to 2d
    //     Eigen::Vector2d extendedGapOrigin(gap->extendedGapOrigin_[0],
    //                                       gap->extendedGapOrigin_[1]);
    //     Eigen::Vector2d leftBezierOrigin(gap->leftBezierOrigin_[0],
    //                                         gap->leftBezierOrigin_[1]);
    //     Eigen::Vector2d rightBezierOrigin(gap->rightBezierOrigin_[0],
    //                                         gap->rightBezierOrigin_[1]);

    //     gap->leftBezierWeight_ = epsilonDivide(leftGapPtVel.norm(), maxRbtVel.norm()); // capped at 1, we can scale down towards 0 until initial constraints are met?
    //     gap->rightBezierWeight_ = epsilonDivide(rightGapPtVel.norm(), maxRbtVel.norm());

    //     // for a totally static gap, can get no velocity on first bezier curve point which corrupts vector field
    //     Eigen::Vector2d weightedLeftBezierPt0(0.0, 0.0);
    //     Eigen::Vector2d weightedRightBezierPt0(0.0, 0.0);
    //     if (leftGapPtVel.norm() > 0.0)
    //         weightedLeftBezierPt0 = leftBezierOrigin + gap->leftBezierWeight_ * (leftCurveInitPt - leftBezierOrigin);
    //     else
    //         weightedLeftBezierPt0 = (0.95 * leftBezierOrigin + 0.05 * leftCurveTermPt);

    //     if (rightGapPtVel.norm() >  0.0)
    //         weightedRightBezierPt0 = rightBezierOrigin + gap->rightBezierWeight_ * (rightCurveInitPt - rightBezierOrigin);
    //     else
    //         weightedRightBezierPt0 = (0.95 * rightBezierOrigin + 0.05 * rightCurveTermPt);
        
    //     // set left and right bezier points
    //     gap->leftPt0_ = weightedLeftBezierPt0;
    //     gap->leftPt1_ = leftCurveTermPt;
    //     gap->rightPt0_ = weightedRightBezierPt0;
    //     gap->rightPt1_ = rightCurveTermPt;  

    //     float placeholder = 0.0;
    //     float leftBezierArclengthDistance = approximateBezierArclength(leftBezierOrigin, weightedLeftBezierPt0, leftCurveTermPt, 
    //                                                                     0.0, 1.0, numCurvePts);
    //     float desiredLeftBezierPtToPtDistance = epsilonDivide(leftBezierArclengthDistance, (numCurvePts - 1));
    //     float leftBezierPtToPtDistanceThresh = desiredLeftBezierPtToPtDistance / 100.0;
    //     ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               arclength parameterizing left curve");      
    //     Eigen::VectorXd leftArclengthParameters = arclengthParameterizeBezier(leftBezierOrigin, weightedLeftBezierPt0, leftCurveTermPt, 
    //                                                                             numCurvePts, desiredLeftBezierPtToPtDistance,
    //                                                                             leftBezierPtToPtDistanceThresh, placeholder, false);        
        
    //     gap->numLeftRGEPoints_ = (cfg_->gap_manip.radial_extend) ? std::max(int(std::ceil( (extendedGapOrigin - leftBezierOrigin).norm() / desiredLeftBezierPtToPtDistance)), 2) : 0;

    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "numLeftRGEPoints: " << numLeftRGEPoints);

    //     int totalNumLeftCurvePts = gap->numLeftRGEPoints_ + numCurvePts; 
    //     Eigen::MatrixXd leftCurvePosns = Eigen::MatrixXd(totalNumLeftCurvePts, 2);
    //     Eigen::MatrixXd leftCurveVels = Eigen::MatrixXd(totalNumLeftCurvePts, 2);
    //     Eigen::MatrixXd leftCurveInwardNorms = Eigen::MatrixXd(totalNumLeftCurvePts, 2);

    //     ///////////////////////////////
    //     // Left radial gap extension //
    //     ///////////////////////////////
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "   building left extended gap origin");
    //     buildExtendedGapOrigin(gap->numLeftRGEPoints_, extendedGapOrigin, 
    //                             leftBezierOrigin, leftCurvePosns, leftCurveVels, leftCurveInwardNorms, true);

    //     ///////////////////////
    //     // Left Bezier curve //
    //     ///////////////////////
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "   building left bezier curve");
    //     buildBezierCurve(gap->numLeftRGEPoints_, totalNumLeftCurvePts, leftArclengthParameters, 
    //                      leftBezierOrigin, gap->leftPt0_, gap->leftPt1_, leftCurvePosns, leftCurveVels, leftCurveInwardNorms, true);        

    //     float rightBezierArclengthDistance = approximateBezierArclength(rightBezierOrigin, weightedRightBezierPt0, rightCurveTermPt, 
    //                                                                     0.0, 1.0, numCurvePts);
    //     float desiredRightBezierPtToPtDistance = epsilonDivide(rightBezierArclengthDistance, (numCurvePts - 1));
    //     float rightBezierPtToPtDistanceThresh = desiredRightBezierPtToPtDistance / 100.0;
    //     ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "               arclength parameterizing right curve");
    //     Eigen::VectorXd rightArclengthParameters = arclengthParameterizeBezier(rightBezierOrigin, weightedRightBezierPt0, rightCurveTermPt, 
    //                                                                             numCurvePts, desiredRightBezierPtToPtDistance,
    //                                                                             rightBezierPtToPtDistanceThresh, placeholder, false);

    //     gap->numRightRGEPoints_ = (cfg_->gap_manip.radial_extend) ? std::max(int(std::ceil( (extendedGapOrigin - rightBezierOrigin).norm() / desiredRightBezierPtToPtDistance)), 2) : 0;

    //     int totalNumRightCurvePts = gap->numRightRGEPoints_ + numCurvePts; 
    //     Eigen::MatrixXd rightCurvePosns = Eigen::MatrixXd(totalNumRightCurvePts, 2);            
    //     Eigen::MatrixXd rightCurveVels = Eigen::MatrixXd(totalNumRightCurvePts, 2);
    //     Eigen::MatrixXd rightCurveInwardNorms = Eigen::MatrixXd(totalNumRightCurvePts, 2);
        
    //     ////////////////////////////////
    //     // Right radial gap extension //
    //     ////////////////////////////////
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "   building right extended gap origin");
    //     buildExtendedGapOrigin(gap->numRightRGEPoints_, extendedGapOrigin, 
    //                             rightBezierOrigin, rightCurvePosns, rightCurveVels, rightCurveInwardNorms, false);
             
    //     ////////////////////////
    //     // Right Bezier curve //
    //     ////////////////////////
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "   building right bezier curve");
    //     buildBezierCurve(gap->numRightRGEPoints_, totalNumRightCurvePts, rightArclengthParameters, 
    //                         rightBezierOrigin, gap->rightPt0_, gap->rightPt1_, rightCurvePosns, rightCurveVels, rightCurveInwardNorms, false);        

    //     float offset = 0.01;

    //     gap->allCurvePts_ = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
    //     gap->gapBoundaryInwardNorms_ = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
    //     gap->leftRightCenters_ = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts, 2);
    //     gap->allAHPFCenters_ = Eigen::MatrixXd(totalNumLeftCurvePts + totalNumRightCurvePts + 1, 2);

    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "centered origin inward norm: " << centered_origin_inward_norm[0] << ", " << centered_origin_inward_norm[1]);
    //     gap->allCurvePts_ << leftCurvePosns, rightCurvePosns; 
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesPosns worked");
    //     gap->gapBoundaryInwardNorms_ << leftCurveInwardNorms, rightCurveInwardNorms; // centered_origin_inward_norm, 
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesInwardNorms worked");
    //     gap->leftRightCenters_ = gap->allCurvePts_ - gap->gapBoundaryInwardNorms_*offset;
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapSideAHPFCenters worked");
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "left_curve points: " << left_curve);
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "right_curve_points: " << right_curve);

    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "left_curve inward norms: " << leftCurveInwardNorms);
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "right_curve inward_norms: " << rightCurveInwardNorms);

    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "gapCurvesInwardNorms: " << gapCurvesInwardNorms);
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "allAHPFCenters: " << allAHPFCenters);

    //     Eigen::Matrix<double, 1, 2> goal;
    //     goal << gapGoalTermPt[0], gapGoalTermPt[1];
    //     gap->allAHPFCenters_ << goal, gap->leftRightCenters_;
    //     // ROS_INFO_STREAM_NAMED("NavigableGapGenerator", "allAHPFCenters worked");   

    //     // gap->allAHPFCenters_ = allAHPFCenters;
    //     // gap->leftRightCenters_ = gapSideAHPFCenters;
    //     // gap->allCurvePts_ = gapCurvesPosns;        
    // }

}