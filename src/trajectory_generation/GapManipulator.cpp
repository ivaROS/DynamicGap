#include <dynamic_gap/trajectory_generation/GapManipulator.h>

namespace dynamic_gap 
{
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }


    // void GapManipulator::radialExtendGap(Gap * gap) 
    // {
    //     try
    //     {
    //         if (!cfg_->gap_manip.radial_extend)
    //             return;

    //         int leftIdx = gap->manipLeftIdx();
    //         int rightIdx = gap->manipRightIdx();
    //         float leftRange = gap->manipLeftRange();
    //         float rightRange = gap->manipRightRange();

    //         float leftTheta = idx2theta(leftIdx);
    //         float rightTheta = idx2theta(rightIdx);
    //         float xLeft = leftRange * cos(leftTheta);
    //         float yLeft = leftRange * sin(leftTheta);
    //         float xRight = rightRange * cos(rightTheta);
    //         float yRight = rightRange * sin(rightTheta);
            
    //         Eigen::Vector2d leftPt(xLeft, yLeft);
    //         Eigen::Vector2d rightPt(xRight, yRight);

    //         // ROS_INFO_STREAM("    [radialExtendGap()]");
    //         // ROS_INFO_STREAM("        pre-RE gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
    //         // ROS_INFO_STREAM("        pre-RE gap in cart. left: (" << leftPt[0] << ", " << leftPt[1] << "), right: (" << rightPt[0] << ", " << rightPt[1] << ")");
            
    //         float s = std::min(gap->getMinSafeDist(), cfg_->rbt.r_inscr * cfg_->traj.inf_ratio);

    //         return;
    //     } catch (...)
    //     {
    //         ROS_ERROR_STREAM_NAMED("GapManipulator", "radialGapExtension() failed");
    //     }
    // }


    void GapManipulator::convertRadialGap(Gap * gap) 
    {
        if (!gap->isRadial()) 
            return;

        sensor_msgs::LaserScan desScan = *scan_.get();
        int leftIdx = gap->manipLeftIdx();
        int rightIdx = gap->manipRightIdx();
        float leftRange = gap->manipLeftRange();
        float rightRange = gap->manipRightRange();

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);
        float xLeft = (leftRange) * cos(leftTheta);
        float yLeft = (leftRange) * sin(leftTheta);
        float xRight = (rightRange) * cos(rightTheta);
        float yRight = (rightRange) * sin(rightTheta);

        gap->getLeftGapPt()->getModel()->isolateGapDynamics();
        gap->getRightGapPt()->getModel()->isolateGapDynamics();

        Eigen::Vector4f leftGapState = gap->getLeftGapPt()->getModel()->getGapState();
        Eigen::Vector4f rightGapState = gap->getRightGapPt()->getModel()->getGapState();

        // ROS_INFO_STREAM("    [convertRadialGap()]");            
        // ROS_INFO_STREAM("        pre-RGC gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        // // ROS_INFO_STREAM("pre-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        bool right = gap->isRightType();
        // Rotate radial gap by an amount theta so that its width is visible
        
        // start with a nominal pivot angle, we will adjust this angle to find the actual theta that we pivot by
        float nomPivotAngle = std::atan2(cfg_->gap_manip.epsilon2, cfg_->gap_manip.epsilon1) + 1e-3;
        // // ROS_INFO_STREAM("theta to pivot: " << theta);
        int nearIdx = 0, farIdx = 0;
        float nearRange = 0.0, farRange = 0.0;
        float nearTheta = 0.0, farTheta = 0.0;
        float signedNomPivotAngle = 0.0;

        // float pivotSideSpeed;
        // float pivotSideSpeedThresh = 0.25;
        if (right) 
        {
            nearIdx = rightIdx;
            nearRange = rightRange;
            farIdx = leftIdx;
            farRange = leftRange;
            signedNomPivotAngle = nomPivotAngle;
            // pivotSideSpeed = rightGapState.tail(2).norm();
            if (gap->getLeftGapPt()->getUngapID() >= 0) // if left point is part of ungap, we do not want to manipulate it 
            {
                return;
            }
        } else 
        {
            nearIdx = leftIdx;
            farIdx = rightIdx;
            nearRange = leftRange;
            farRange = rightRange;
            signedNomPivotAngle = -nomPivotAngle;
            // pivotSideSpeed = leftGapState.tail(2).norm();
            if (gap->getRightGapPt()->getUngapID() >= 0) // if right point is part of ungap, we do not want to manipulate it
            {
                return;                
            }
        }

        // ROS_INFO_STREAM("        near: (" << nearIdx << ", " << nearRange << "), far: (" << farIdx << ", " << farRange << ")");

        // ROS_INFO_STREAM("signedNomPivotAngle: " << signedNomPivotAngle);

        Eigen::Matrix3f nomPivotAngleRotationMatrix;
        // nomPivotAngleRotationMatrix: SE(3) matrix that represents desired rotation amount
        nomPivotAngleRotationMatrix << cos(signedNomPivotAngle), -sin(signedNomPivotAngle), 0,
                                        sin(signedNomPivotAngle), cos(signedNomPivotAngle), 0,
                                        0, 0, 1;
        
        // obtaining near and far theta values from indices
        nearTheta = idx2theta(nearIdx);
        farTheta = idx2theta(farIdx);   

        Eigen::Matrix3f nearPtTranslationMatrix, farPtTranslationMatrix;
        // nearPtTranslationMatrix, farPtTranslationMatrix: SE(2) matrices that represent translation from rbt origin to near and far points
        nearPtTranslationMatrix << 1., 0., nearRange * cos(nearTheta),
                                    0., 1., nearRange * sin(nearTheta),
                                    0., 0., 1.;
        farPtTranslationMatrix  << 1., 0., farRange * cos(farTheta),
                                    0., 1., farRange * sin(farTheta),
                                    0., 0., 1.;

        // pivotedPtRotationMatrix: my guess, transformation matrix to desired pivot point using existing gap points
        // inverse flips direction of the translation
        Eigen::Matrix3f nearToFarTranslationMatrix = nearPtTranslationMatrix.inverse() * farPtTranslationMatrix;
        Eigen::Matrix3f pivotedPtRotationMatrix = nearPtTranslationMatrix * 
                                                        (nomPivotAngleRotationMatrix * 
                                                            nearToFarTranslationMatrix);
        // ROS_INFO_STREAM_NAMED("GapManipulator",  "pivotedPtRotationMatrix: " << pivotedPtRotationMatrix);

        // Extracting theta and idx of pivoted point
        float nomPivotedTheta = std::atan2(pivotedPtRotationMatrix(1, 2), pivotedPtRotationMatrix(0, 2));
        int nomPivotedIdx = theta2idx(nomPivotedTheta);

        // Search along current scan to from initial gap point to pivoted gap point
        // to obtain range value to assign to the pivoted gap point
        int scanSearchStartIdx = 0, scanSearchEndIdx = 0;
        if (right)
        {   
            scanSearchStartIdx = leftIdx;
            scanSearchEndIdx = nomPivotedIdx;
        } else
        {
            scanSearchStartIdx = nomPivotedIdx;
            scanSearchEndIdx = rightIdx;
        }

        // ROS_INFO_STREAM("scanSearchStartIdx: " << scanSearchStartIdx << 
                                                // ", scanSearchEndIdx: " << scanSearchEndIdx);

        // // ROS_INFO_STREAM("wrapped scanSearchStartIdx: " << scanSearchStartIdx << ", wrapped scanSearchEndIdx: " << scanSearchEndIdx);
        int scanSearchSize = scanSearchEndIdx - scanSearchStartIdx;

        // what if search size == 0?

        if (scanSearchSize <= 0)
            scanSearchSize += cfg_->scan.full_scan; // int(2*gap->half_scan);


        int gapIdxSpan = (leftIdx - rightIdx);
        if (gapIdxSpan <= 0)
            gapIdxSpan += cfg_->scan.full_scan; // int(2*gap->half_scan);
        // // ROS_INFO_STREAM("gapIdxSpan: " << gapIdxSpan);

        // using the law of cosines to find the index between init/final indices
        // that's shortest distance between near point and laser scan
        // // ROS_INFO_STREAM("ranges size: " << stored_scan_msgs.ranges.size());
        std::vector<float> nearPtToScanDists(scanSearchSize);

        int checkIdx = 0;
        float checkRange = 0.0, checkIdxSpan = 0.0;
        for (int i = 0; i < nearPtToScanDists.size(); i++) 
        {
            checkIdx = (i + scanSearchStartIdx) % cfg_->scan.full_scan; // int(2 * gap->half_scan);
            checkRange = desScan.ranges.at(checkIdx);
            checkIdxSpan = gapIdxSpan + (scanSearchSize - i);
            nearPtToScanDists.at(i) = sqrt(pow(nearRange, 2) + pow(checkRange, 2) -
                                        2.0 * nearRange * checkRange * cos(checkIdxSpan * cfg_->scan.angle_increment));
            // // ROS_INFO_STREAM("checking idx: " << checkIdx << ", range of: " << range << ", diff in idx: " << checkIdxSpan << ", dist of " << dist.at(i));
        }

        auto minDistIter = std::min_element(nearPtToScanDists.begin(), nearPtToScanDists.end());
        int minDistIdx = (scanSearchStartIdx + std::distance(nearPtToScanDists.begin(), minDistIter)) % cfg_->scan.full_scan; // int(2*gap->half_scan);
        float minDistRange = *minDistIter;

        // ROS_INFO_STREAM("from " << scanSearchStartIdx << " to " << scanSearchEndIdx << ", min dist of " << minDistRange << " at " << minDistIdx);         

        // pivoting around near point, pointing towards far point or something?

        float nearToFarDistance = sqrt(pow(nearToFarTranslationMatrix(0, 2), 2) + pow(nearToFarTranslationMatrix(1, 2), 2));
        // ROS_INFO_STREAM("nearToFarDistance: " << nearToFarDistance);

        // // ROS_INFO_STREAM("translation norm: " << nearToFarDistance);
        
        // augmenting near to far direction by pivoted point, multiplying by min dist        
        nearToFarTranslationMatrix(0, 2) *= epsilonDivide(minDistRange, nearToFarDistance);     
        nearToFarTranslationMatrix(1, 2) *= epsilonDivide(minDistRange, nearToFarDistance);
        Eigen::Matrix3f pivotedPtMatrix = nearPtTranslationMatrix * (nomPivotAngleRotationMatrix * nearToFarTranslationMatrix);

        float pivotedPtRange = sqrt(pow(pivotedPtMatrix(0, 2), 2) + pow(pivotedPtMatrix(1, 2), 2));
        float pivotedPtTheta = std::atan2(pivotedPtMatrix(1, 2), pivotedPtMatrix(0, 2));

        if ( std::isnan(pivotedPtTheta) || std::isnan(pivotedPtRange) || 
            std::isinf(pivotedPtTheta) || std::isinf(pivotedPtRange) )
        {
            ROS_WARN_STREAM_NAMED("GapManipulator", "[convertRadialGap]: bad value, pivotedPtTheta: " << pivotedPtTheta << ", pivotedPtRange: " << pivotedPtRange);
            return;
        }

        int pivotedPtIdx = theta2idx(pivotedPtTheta); 

        float newLeftIdx = 0.0, newRightIdx = 0.0, newLeftRange = 0.0, newRightRange = 0.0;
        if (right) 
        {
            newLeftIdx = pivotedPtIdx;
            newLeftRange = pivotedPtRange;
            newRightIdx = nearIdx;
            newRightRange = nearRange;
            gap->getLeftGapPt()->getModel()->setNewPosition(pivotedPtTheta, pivotedPtRange); // manipulating left point
            gap->getLeftGapPt()->getModel()->setRGC(); // manipulating left point, so set vel to 0
        } else 
        {
            newLeftIdx = nearIdx;
            newLeftRange = nearRange;
            newRightIdx = pivotedPtIdx;
            newRightRange = pivotedPtRange;
            gap->getRightGapPt()->getModel()->setNewPosition(pivotedPtTheta, pivotedPtRange); // manipulating right point
            gap->getRightGapPt()->getModel()->setRGC(); // manipulating right point, so set vel to 0
        }

        gap->setManipPoints(newLeftIdx, newRightIdx, newLeftRange, newRightRange);

        gap->getManipulatedLCartesian(xLeft, yLeft);
        gap->getManipulatedRCartesian(xRight, yRight);

        gap->setRGC();

        // ROS_INFO_STREAM("        post-RGC gap in polar. left: (" << newLeftIdx << ", " << newLeftRange << "), right: (" << newRightIdx << ", " << newRightRange << ")");
        // ROS_INFO_STREAM_NAMED("GapManipulator",  "post-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
    }

    bool GapManipulator::inflateGapSides(Gap * gap) 
    {
        // get points

        sensor_msgs::LaserScan desScan = *scan_.get();
        int leftIdx = gap->manipLeftIdx();
        int rightIdx = gap->manipRightIdx();
        float leftRange = gap->manipLeftRange();
        float rightRange = gap->manipRightRange();

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);
        float xLeft = (leftRange) * cos(leftTheta);
        float yLeft = (leftRange) * sin(leftTheta);
        float xRight = (rightRange) * cos(rightTheta);
        float yRight = (rightRange) * sin(rightTheta);
        
        Eigen::Vector2f leftPt(xLeft, yLeft);
        Eigen::Vector2f rightPt(xRight, yRight);

        ROS_INFO_STREAM_NAMED("GapManipulator", "    [inflateGapSides()]");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-inflate gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        Eigen::Vector2f leftUnitNorm = leftPt.normalized();
        Eigen::Vector2f rightUnitNorm = rightPt.normalized();
        float leftToRightAngle = getSweptLeftToRightAngle(leftUnitNorm, rightUnitNorm);

        ///////////////////////
        // ANGULAR INFLATION //
        ///////////////////////

        float alpha_left = std::asin(cfg_->rbt.r_inscr * cfg_->traj.inf_ratio / leftPt.norm() );
        float alpha_right = std::asin(cfg_->rbt.r_inscr * cfg_->traj.inf_ratio / rightPt.norm() );

        float beta_left = (M_PI_OVER_TWO) - alpha_left;
        float beta_right = (M_PI_OVER_TWO) - alpha_right;

        float r_infl_left = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio / sin(beta_left);
        float r_infl_right = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio / sin(beta_right);

        Eigen::Vector2f leftAngularInflDir = Rnegpi2 * leftUnitNorm;
        Eigen::Vector2f rightAngularInflDir = Rpi2 * rightUnitNorm;

        ROS_INFO_STREAM_NAMED("GapManipulator", "        leftAngularInflDir: (" << leftAngularInflDir.transpose());
        ROS_INFO_STREAM_NAMED("GapManipulator", "        rightAngularInflDir: (" << rightAngularInflDir.transpose());

        // perform inflation
        Eigen::Vector2f inflatedLeftPt = leftPt + leftAngularInflDir * r_infl_left;
        Eigen::Vector2f inflatedRightPt = rightPt + rightAngularInflDir * r_infl_right;

        ROS_INFO_STREAM_NAMED("GapManipulator", "        inflatedLeftPt: (" << inflatedLeftPt.transpose());
        ROS_INFO_STREAM_NAMED("GapManipulator", "        inflatedRightPt: (" << inflatedRightPt.transpose());

        float inflatedLeftTheta = std::atan2(inflatedLeftPt[1], inflatedLeftPt[0]);
        float inflatedRightTheta = std::atan2(inflatedRightPt[1], inflatedRightPt[0]);

        // Check if inflation worked properly
        Eigen::Vector2f inflatedLeftUnitNorm(std::cos(inflatedLeftTheta), std::sin(inflatedLeftTheta));
        Eigen::Vector2f inflatedRightUnitNorm(std::cos(inflatedRightTheta), std::sin(inflatedRightTheta));
        float newLeftToRightAngle = getSweptLeftToRightAngle(inflatedLeftUnitNorm, inflatedRightUnitNorm);

        // update gap points
        int inflatedLeftIdx = theta2idx(inflatedLeftTheta);
        int inflatedRightIdx = theta2idx(inflatedRightTheta);
        
        // float leftToInflatedLeftAngle = getSweptLeftToRightAngle(leftUnitNorm, inflatedLeftUnitNorm);
        // float leftToInflatedRightAngle = getSweptLeftToRightAngle(leftUnitNorm, inflatedRightUnitNorm);
        float inflatedLeftRange = inflatedLeftPt.norm(); // leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedLeftAngle, leftToRightAngle);
        float inflatedRightRange = inflatedRightPt.norm(); // leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedRightAngle, leftToRightAngle);

        // if gap is too small, mark it to be discarded
        if (newLeftToRightAngle > leftToRightAngle)
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "        inflation has failed, new points in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");

            return false;
        }

        if (inflatedRightIdx == inflatedLeftIdx) // // ROS_INFO_STREAM("manipulated indices are same");
            inflatedLeftIdx++;

        gap->setManipPoints(inflatedLeftIdx, inflatedRightIdx, inflatedLeftRange, inflatedRightRange);

        gap->getLeftGapPt()->getModel()->setNewPosition(inflatedLeftTheta, inflatedLeftRange); // manipulating left point
        gap->getRightGapPt()->getModel()->setNewPosition(inflatedRightTheta, inflatedRightRange); // manipulating right point

        gap->getManipulatedLCartesian(xLeft, yLeft);
        gap->getManipulatedRCartesian(xRight, yRight);
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-inflate gap in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        // Eigen::Vector2f trailingLeftPt = leftPt - leftAngularInflDir * r_infl_left;
        // Eigen::Vector2f trailingRightPt = rightPt - rightAngularInflDir * r_infl_right;

        // ROS_INFO_STREAM("        trailingLeftPt: (" << trailingLeftPt.transpose());
        // ROS_INFO_STREAM("        trailingRightPt: (" << trailingRightPt.transpose());

        // float trailingLeftTheta = std::atan2(trailingLeftPt[1], trailingLeftPt[0]);
        // float trailingRightTheta = std::atan2(trailingRightPt[1], trailingRightPt[0]);

        // int trailingLeftIdx = theta2idx(trailingLeftTheta);
        // int trailingRightIdx = theta2idx(trailingRightTheta);
        
        // float trailingLeftRange = trailingLeftPt.norm(); // leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedLeftAngle, leftToRightAngle);
        // float trailingRightRange = trailingRightPt.norm(); // leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedRightAngle, leftToRightAngle);

        // gap->setManipTrailingLeftIdx(trailingLeftIdx);
        // gap->setManipTrailingRightIdx(trailingRightIdx);
        // gap->setManipTrailingLeftRange(trailingLeftRange);
        // gap->setManipTrailingRightRange(trailingRightRange);

        // gap->getManipulatedTrailingLCartesian(xLeft, yLeft);
        // gap->getManipulatedTrailingRCartesian(xRight, yRight);
        // ROS_INFO_STREAM("        post-inflate gap in polar. trailing left: (" << trailingLeftIdx << ", " << trailingLeftRange << "), trailing right: (" << trailingRightIdx << ", " << trailingRightRange << ")");
        // ROS_INFO_STREAM("        post-inflate gap in cart. trailing left: (" << xLeft << ", " << yLeft << "), trailing right: (" << xRight << ", " << yRight << ")");


        return true;
    }
}
