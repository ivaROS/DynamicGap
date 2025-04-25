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


    void GapManipulator::convertRadialGap(std::vector<Gap *> const & gaps, const int & gapIdx) 
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "    [convertRadialGap()]");

        Gap * gap = gaps.at(gapIdx);

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

        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RGC gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");    

        if (!gap->isRadial())
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap is not radial, no conversion needed");
            return;
        }

        gap->getLeftGapPt()->getModel()->isolateGapDynamics();
        gap->getRightGapPt()->getModel()->isolateGapDynamics();

        Eigen::Vector4f leftGapState = gap->getLeftGapPt()->getModel()->getGapState();
        Eigen::Vector4f rightGapState = gap->getRightGapPt()->getModel()->getGapState();

        // ROS_INFO_STREAM("        pre-RGC gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
        // // ROS_INFO_STREAM("pre-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        bool right = gap->isRightType();
        // Rotate radial gap by an amount theta so that its width is visible
        
        // start with a nominal pivot angle, we will adjust this angle to find the actual theta that we pivot by
        float nomPivotAngle = cfg_->gap_manip.rgc_angle; // std::atan2(cfg_->gap_manip.epsilon2, cfg_->gap_manip.epsilon1) + 1e-3;
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
                ROS_INFO_STREAM_NAMED("GapManipulator", "        left part is part of ungap, no conversion needed");
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
                ROS_INFO_STREAM_NAMED("GapManipulator", "        right part is part of ungap, no conversion needed");
                return;                
            }
        }

        // ROS_INFO_STREAM("        near: (" << nearIdx << ", " << nearRange << "), far: (" << farIdx << ", " << farRange << ")");

        ROS_INFO_STREAM_NAMED("GapManipulator", "        signedNomPivotAngle: " << signedNomPivotAngle);

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

        ROS_INFO_STREAM_NAMED("GapManipulator", "        nearPtTranslationMatrix: ");
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearPtTranslationMatrix(0, 0) << ", " << nearPtTranslationMatrix(0, 1) << ", " << nearPtTranslationMatrix(0, 2));
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearPtTranslationMatrix(1, 0) << ", " << nearPtTranslationMatrix(1, 1) << ", " << nearPtTranslationMatrix(1, 2));
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearPtTranslationMatrix(2, 0) << ", " << nearPtTranslationMatrix(2, 1) << ", " << nearPtTranslationMatrix(2, 2));

        farPtTranslationMatrix  << 1., 0., farRange * cos(farTheta),
                                    0., 1., farRange * sin(farTheta),
                                    0., 0., 1.;

        ROS_INFO_STREAM_NAMED("GapManipulator", "        farPtTranslationMatrix: ");
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << farPtTranslationMatrix(0, 0) << ", " << farPtTranslationMatrix(0, 1) << ", " << farPtTranslationMatrix(0, 2));
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << farPtTranslationMatrix(1, 0) << ", " << farPtTranslationMatrix(1, 1) << ", " << farPtTranslationMatrix(1, 2));
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << farPtTranslationMatrix(2, 0) << ", " << farPtTranslationMatrix(2, 1) << ", " << farPtTranslationMatrix(2, 2));

        // just translation of near to far
        Eigen::Matrix3f nearToFarTranslationMatrix = nearPtTranslationMatrix.inverse() * farPtTranslationMatrix;


        // pivotedPtRotationMatrix: rotate vector from near point to far point by the desired angle
        ROS_INFO_STREAM_NAMED("GapManipulator", "        nearToFarTranslationMatrix: ");
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearToFarTranslationMatrix(0, 0) << ", " << nearToFarTranslationMatrix(0, 1) << ", " << nearToFarTranslationMatrix(0, 2));
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearToFarTranslationMatrix(1, 0) << ", " << nearToFarTranslationMatrix(1, 1) << ", " << nearToFarTranslationMatrix(1, 2));
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearToFarTranslationMatrix(2, 0) << ", " << nearToFarTranslationMatrix(2, 1) << ", " << nearToFarTranslationMatrix(2, 2));
        
        Eigen::Matrix3f pivotedPtRotationMatrix = nearPtTranslationMatrix * nomPivotAngleRotationMatrix * nearToFarTranslationMatrix;
        // ROS_INFO_STREAM_NAMED("GapManipulator",  "pivotedPtRotationMatrix: " << pivotedPtRotationMatrix);

        ROS_INFO_STREAM_NAMED("GapManipulator", "        pivotedPtRotationMatrix: ");
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << pivotedPtRotationMatrix(0, 0) << ", " << pivotedPtRotationMatrix(0, 1) << ", " << pivotedPtRotationMatrix(0, 2));
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << pivotedPtRotationMatrix(1, 0) << ", " << pivotedPtRotationMatrix(1, 1) << ", " << pivotedPtRotationMatrix(1, 2));
        ROS_INFO_STREAM_NAMED("GapManipulator", "           " << pivotedPtRotationMatrix(2, 0) << ", " << pivotedPtRotationMatrix(2, 1) << ", " << pivotedPtRotationMatrix(2, 2));

        // Extracting theta and idx of pivoted point
        float nomPivotedTheta = std::atan2(pivotedPtRotationMatrix(1, 2), pivotedPtRotationMatrix(0, 2));
        int nomPivotedIdx = theta2idx(nomPivotedTheta);

        ROS_INFO_STREAM_NAMED("GapManipulator", "        nomPivotedTheta: " << nomPivotedTheta);
        ROS_INFO_STREAM_NAMED("GapManipulator", "        nomPivotedIdx: " << nomPivotedIdx);

        // Clip pivoted theta to not overlap with adjacent gaps
        if (right)
        {
            int nextGapIdx = (gapIdx + 1) % gaps.size();
            Gap * adjGap = gaps.at(nextGapIdx);
            int adjIdx = adjGap->manipRightIdx();
            float adjTheta = idx2theta(adjIdx);

            ROS_INFO_STREAM_NAMED("GapManipulator", "        adjIdx: " << adjIdx);
            ROS_INFO_STREAM_NAMED("GapManipulator", "        adjTheta: " << adjTheta);

            Eigen::Vector2f originalPtUnitNorm = Eigen::Vector2f(cos(leftTheta), sin(leftTheta));
            Eigen::Vector2f pivotPtUnitNorm = Eigen::Vector2f(cos(nomPivotedTheta), sin(nomPivotedTheta));
            Eigen::Vector2f adjGapPtUnitNorm = Eigen::Vector2f(cos(adjTheta), sin(adjTheta));

            float adjGapPtToOriginalGapPtAngle = getSweptLeftToRightAngle(adjGapPtUnitNorm, originalPtUnitNorm);
            float pivotedGapPtToOriginalGapPtAngle = getSweptLeftToRightAngle(pivotPtUnitNorm, originalPtUnitNorm);

            if (adjGapPtToOriginalGapPtAngle <= pivotedGapPtToOriginalGapPtAngle)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "        pivoted gap point overlaps with adjacent gap (right), need to clip");

                nomPivotedIdx = (adjIdx -1 + cfg_->scan.full_scan) % cfg_->scan.full_scan;
                nomPivotedTheta = idx2theta(nomPivotedIdx);

                ROS_INFO_STREAM_NAMED("GapManipulator", "        new pivotedIdx: " << nomPivotedIdx);
                ROS_INFO_STREAM_NAMED("GapManipulator", "        new pivotedTheta: " << nomPivotedTheta);
            }

        } else
        {
            int prevGapIdx = (gapIdx - 1 + gaps.size()) % gaps.size();
            Gap * adjGap = gaps.at(prevGapIdx);
            int adjIdx = adjGap->manipLeftIdx();
            float adjTheta = idx2theta(adjIdx);

            ROS_INFO_STREAM_NAMED("GapManipulator", "        adjIdx: " << adjIdx);
            ROS_INFO_STREAM_NAMED("GapManipulator", "        adjTheta: " << adjTheta);

            Eigen::Vector2f originalPtUnitNorm = Eigen::Vector2f(cos(rightTheta), sin(rightTheta));
            Eigen::Vector2f pivotPtUnitNorm = Eigen::Vector2f(cos(nomPivotedTheta), sin(nomPivotedTheta));
            Eigen::Vector2f adjGapPtUnitNorm = Eigen::Vector2f(cos(adjTheta), sin(adjTheta));

            float originalGapPtToAdjGapPtAngle = getSweptLeftToRightAngle(originalPtUnitNorm, adjGapPtUnitNorm);
            float originalGapPtToPivotedGapPtAngle = getSweptLeftToRightAngle(originalPtUnitNorm, pivotPtUnitNorm);

            if (originalGapPtToAdjGapPtAngle <= originalGapPtToPivotedGapPtAngle)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "        pivoted gap point overlaps with adjacent gap (left), need to clip");

                nomPivotedIdx = (adjIdx + 1) % cfg_->scan.full_scan;
                nomPivotedTheta = idx2theta(nomPivotedIdx);

                ROS_INFO_STREAM_NAMED("GapManipulator", "        new pivotedIdx: " << nomPivotedIdx);
                ROS_INFO_STREAM_NAMED("GapManipulator", "        new pivotedTheta: " << nomPivotedTheta);
            }
        }

        // // Search along current scan to from initial gap point to pivoted gap point
        // // to obtain range value to assign to the pivoted gap point
        // int scanSearchStartIdx = 0, scanSearchEndIdx = 0;
        // if (right)
        // {   
        //     scanSearchStartIdx = leftIdx;
        //     scanSearchEndIdx = nomPivotedIdx;
        // } else
        // {
        //     scanSearchStartIdx = nomPivotedIdx;
        //     scanSearchEndIdx = rightIdx;
        // }

        // // ROS_INFO_STREAM("scanSearchStartIdx: " << scanSearchStartIdx << 
        //                                         // ", scanSearchEndIdx: " << scanSearchEndIdx);

        // // ROS_INFO_STREAM("wrapped scanSearchStartIdx: " << scanSearchStartIdx << ", wrapped scanSearchEndIdx: " << scanSearchEndIdx);
        // int scanSearchSize = scanSearchEndIdx - scanSearchStartIdx;

        // // what if search size == 0?

        // if (scanSearchSize <= 0)
        //     scanSearchSize += cfg_->scan.full_scan; // int(2*gap->half_scan);


        // int gapIdxSpan = (leftIdx - rightIdx);
        // if (gapIdxSpan <= 0)
        //     gapIdxSpan += cfg_->scan.full_scan; // int(2*gap->half_scan);
        // // // ROS_INFO_STREAM("gapIdxSpan: " << gapIdxSpan);

        // // using the law of cosines to find the index between init/final indices
        // // that's shortest distance between near point and laser scan
        // // // ROS_INFO_STREAM("ranges size: " << stored_scan_msgs.ranges.size());
        // std::vector<float> nearPtToScanDists(scanSearchSize);

        // int checkIdx = 0;
        // float checkRange = 0.0, checkIdxSpan = 0.0;
        // for (int i = 0; i < nearPtToScanDists.size(); i++) 
        // {
        //     checkIdx = (i + scanSearchStartIdx) % cfg_->scan.full_scan; // int(2 * gap->half_scan);
        //     checkRange = desScan.ranges.at(checkIdx);
        //     checkIdxSpan = gapIdxSpan + (scanSearchSize - i);
        //     nearPtToScanDists.at(i) = sqrt(pow(nearRange, 2) + pow(checkRange, 2) -
        //                                 2.0 * nearRange * checkRange * cos(checkIdxSpan * cfg_->scan.angle_increment));
        //     // // ROS_INFO_STREAM("checking idx: " << checkIdx << ", range of: " << range << ", diff in idx: " << checkIdxSpan << ", dist of " << dist.at(i));
        // }

        // auto minDistIter = std::min_element(nearPtToScanDists.begin(), nearPtToScanDists.end());
        // int minDistIdx = (scanSearchStartIdx + std::distance(nearPtToScanDists.begin(), minDistIter)) % cfg_->scan.full_scan; // int(2*gap->half_scan);
        
        // float minDistRange = *minDistIter;
        
        int pivotedPtIdx = nomPivotedIdx;
        float pivotedPtTheta = idx2theta(pivotedPtIdx);
        float pivotedPtRange = std::min(farRange, desScan.ranges.at(pivotedPtIdx));

        // int pivotedPtIdx = minDistIdx;
        // float pivotedPtTheta = idx2theta(pivotedPtIdx);
        // float pivotedPtRange = desScan.ranges.at(minDistIdx);

        
        // float minDistRange = *minDistIter;

        // ROS_INFO_STREAM_NAMED("GapManipulator", "        from " << scanSearchStartIdx << " to " << scanSearchEndIdx << ", min dist of " << minDistRange << " at " << minDistIdx);         

        // // pivoting around near point, pointing towards far point or something?

        // float nearToFarDistance = sqrt(pow(nearToFarTranslationMatrix(0, 2), 2) + pow(nearToFarTranslationMatrix(1, 2), 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "        nearToFarDistance: " << nearToFarDistance);

        // // ROS_INFO_STREAM("translation norm: " << nearToFarDistance);
        
        // // augmenting near to far direction by pivoted point, multiplying by min dist        
        // // nearToFarTranslationMatrix(0, 2) *= epsilonDivide(minDistRange, nearToFarDistance);     
        // // nearToFarTranslationMatrix(1, 2) *= epsilonDivide(minDistRange, nearToFarDistance);

        // Eigen::Matrix3f rescaledNearToFarTranslationMatrix = nearToFarTranslationMatrix;
        // Eigen::Vector2f pivotedPtTranslationVector = Eigen::Vector2f(rescaledNearToFarTranslationMatrix(0, 2), rescaledNearToFarTranslationMatrix(1, 2));
        // rescaledNearToFarTranslationMatrix(0, 2) = pivotedPtTranslationVector[0] * epsilonDivide(minDistRange, nearToFarDistance);
        // rescaledNearToFarTranslationMatrix(1, 2) = pivotedPtTranslationVector[1] * epsilonDivide(minDistRange, nearToFarDistance);
        // rescaledNearToFarTranslationMatrix(0, 2) = epsilonDivide(minDistRange, nearToFarDistance);
        // rescaledNearToFarTranslationMatrix(1, 2) *= epsilonDivide(minDistRange, nearToFarDistance);

        // ROS_INFO_STREAM_NAMED("GapManipulator", "        re-scaled nearToFarTranslationMatrix: ");
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << rescaledNearToFarTranslationMatrix(0, 0) << ", " << rescaledNearToFarTranslationMatrix(0, 1) << ", " << rescaledNearToFarTranslationMatrix(0, 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << rescaledNearToFarTranslationMatrix(1, 0) << ", " << rescaledNearToFarTranslationMatrix(1, 1) << ", " << rescaledNearToFarTranslationMatrix(1, 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << rescaledNearToFarTranslationMatrix(2, 0) << ", " << rescaledNearToFarTranslationMatrix(2, 1) << ", " << rescaledNearToFarTranslationMatrix(2, 2));

        // ROS_INFO_STREAM("        re-scaled nearToFarTranslationMatrix: " << nearToFarTranslationMatrix);

        // ROS_INFO_STREAM_NAMED("GapManipulator", "        nearPtTranslationMatrix: ");
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearPtTranslationMatrix(0, 0) << ", " << nearPtTranslationMatrix(0, 1) << ", " << nearPtTranslationMatrix(0, 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearPtTranslationMatrix(1, 0) << ", " << nearPtTranslationMatrix(1, 1) << ", " << nearPtTranslationMatrix(1, 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nearPtTranslationMatrix(2, 0) << ", " << nearPtTranslationMatrix(2, 1) << ", " << nearPtTranslationMatrix(2, 2));

        // ROS_INFO_STREAM_NAMED("GapManipulator", "        nomPivotAngleRotationMatrix: ");
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nomPivotAngleRotationMatrix(0, 0) << ", " << nomPivotAngleRotationMatrix(0, 1) << ", " << nomPivotAngleRotationMatrix(0, 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nomPivotAngleRotationMatrix(1, 0) << ", " << nomPivotAngleRotationMatrix(1, 1) << ", " << nomPivotAngleRotationMatrix(1, 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << nomPivotAngleRotationMatrix(2, 0) << ", " << nomPivotAngleRotationMatrix(2, 1) << ", " << nomPivotAngleRotationMatrix(2, 2));

        // Eigen::Matrix3f pivotedPtMatrix = nearPtTranslationMatrix * nomPivotAngleRotationMatrix * rescaledNearToFarTranslationMatrix;

        // ROS_INFO_STREAM_NAMED("GapManipulator", "        pivotedPtMatrix: ");
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << pivotedPtMatrix(0, 0) << ", " << pivotedPtMatrix(0, 1) << ", " << pivotedPtMatrix(0, 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << pivotedPtMatrix(1, 0) << ", " << pivotedPtMatrix(1, 1) << ", " << pivotedPtMatrix(1, 2));
        // ROS_INFO_STREAM_NAMED("GapManipulator", "           " << pivotedPtMatrix(2, 0) << ", " << pivotedPtMatrix(2, 1) << ", " << pivotedPtMatrix(2, 2));

        // float pivotedPtRange = sqrt(pow(pivotedPtMatrix(0, 2), 2) + pow(pivotedPtMatrix(1, 2), 2));
        // float pivotedPtTheta = std::atan2(pivotedPtMatrix(1, 2), pivotedPtMatrix(0, 2));

        // ROS_INFO_STREAM_NAMED("GapManipulator", "        pivotedPtRange: " << pivotedPtRange);
        // ROS_INFO_STREAM_NAMED("GapManipulator", "        pivotedPtTheta: " << pivotedPtTheta);


        // if ( std::isnan(pivotedPtTheta) || std::isnan(pivotedPtRange) || 
        //     std::isinf(pivotedPtTheta) || std::isinf(pivotedPtRange) )
        // {
        //     ROS_WARN_STREAM_NAMED("GapManipulator", "[convertRadialGap]: bad value, pivotedPtTheta: " << pivotedPtTheta << ", pivotedPtRange: " << pivotedPtRange);
        //     return;
        // }

        // int pivotedPtIdx = theta2idx(pivotedPtTheta); 

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

        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-RGC gap in polar. left: (" << newLeftIdx << ", " << newLeftRange << "), right: (" << newRightIdx << ", " << newRightRange << ")");
        ROS_INFO_STREAM_NAMED("GapManipulator", "        post-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
    }

    void GapManipulator::inflateGapSides(Gap * gap) 
    {
        // get points

        float inf_ratio = cfg_->traj.inf_ratio;

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

        ROS_INFO_STREAM_NAMED("GapManipulator", "        leftToRightAngle: " << leftToRightAngle);;

        float newLeftToRightAngle = leftToRightAngle;
        float inflatedLeftTheta = leftTheta;
        float inflatedRightTheta = rightTheta;
        int inflatedLeftIdx = leftIdx;
        int inflatedRightIdx = rightIdx;
        float inflatedLeftRange = leftRange;
        float inflatedRightRange = rightRange;
        bool successful_inflation = false;
        while (!successful_inflation && inf_ratio >= 1.0) // try current inflation ratio, scale it down if that fails
        {
            ///////////////////////
            // ANGULAR INFLATION //
            ///////////////////////
            ROS_INFO_STREAM_NAMED("GapManipulator", "        inflating gap sides with ratio: " << inf_ratio);
    
            if ( cfg_->rbt.r_inscr * inf_ratio > leftRange)
            {
                ROS_WARN_STREAM_NAMED("GapManipulator", "        inflation ratio is too large, aborting");


                gap->setManipPoints(leftIdx, rightIdx, leftRange, rightRange);

                gap->getLeftGapPt()->getModel()->setNewPosition(leftTheta, leftRange); // manipulating left point
                gap->getRightGapPt()->getModel()->setNewPosition(rightTheta, rightRange); // manipulating right point
        
                return;
                // return false;
            }

            float alpha_left = std::asin(cfg_->rbt.r_inscr * inf_ratio / leftPt.norm() );
            float alpha_right = std::asin(cfg_->rbt.r_inscr * inf_ratio / rightPt.norm() );
    
            float beta_left = (M_PI_OVER_TWO) - alpha_left;
            float beta_right = (M_PI_OVER_TWO) - alpha_right;
    
            float r_infl_left = cfg_->rbt.r_inscr * inf_ratio / sin(beta_left);
            float r_infl_right = cfg_->rbt.r_inscr * inf_ratio / sin(beta_right);
    
            Eigen::Vector2f leftAngularInflDir = Rnegpi2 * leftUnitNorm;
            Eigen::Vector2f rightAngularInflDir = Rpi2 * rightUnitNorm;
    
            ROS_INFO_STREAM_NAMED("GapManipulator", "        leftAngularInflDir: (" << leftAngularInflDir.transpose() << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        rightAngularInflDir: (" << rightAngularInflDir.transpose() << ")");
    
            // perform inflation
            Eigen::Vector2f inflatedLeftPt = leftPt + leftAngularInflDir * r_infl_left;
            Eigen::Vector2f inflatedRightPt = rightPt + rightAngularInflDir * r_infl_right;
    
            ROS_INFO_STREAM_NAMED("GapManipulator", "        inflatedLeftPt: (" << inflatedLeftPt.transpose() << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        inflatedRightPt: (" << inflatedRightPt.transpose() << ")");
    
            inflatedLeftTheta = std::atan2(inflatedLeftPt[1], inflatedLeftPt[0]);
            inflatedRightTheta = std::atan2(inflatedRightPt[1], inflatedRightPt[0]);
    
            // Check if inflation worked properly
            Eigen::Vector2f inflatedLeftUnitNorm(std::cos(inflatedLeftTheta), std::sin(inflatedLeftTheta));
            Eigen::Vector2f inflatedRightUnitNorm(std::cos(inflatedRightTheta), std::sin(inflatedRightTheta));
            float newLeftToRightAngle = getSweptLeftToRightAngle(inflatedLeftUnitNorm, inflatedRightUnitNorm);
    
            // update gap points
            inflatedLeftIdx = theta2idx(inflatedLeftTheta);
            inflatedRightIdx = theta2idx(inflatedRightTheta);
            
            // float leftToInflatedLeftAngle = getSweptLeftToRightAngle(leftUnitNorm, inflatedLeftUnitNorm);
            // float leftToInflatedRightAngle = getSweptLeftToRightAngle(leftUnitNorm, inflatedRightUnitNorm);
            inflatedLeftRange = inflatedLeftPt.norm(); // leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedLeftAngle, leftToRightAngle);
            inflatedRightRange = inflatedRightPt.norm(); // leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedRightAngle, leftToRightAngle);
    
            // if gap is too small, mark it to be discarded
            if (newLeftToRightAngle > leftToRightAngle)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "        inflation has failed, new points in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
    
                
                inf_ratio -= 0.1; // = std::max(1.0, inf_ratio - 0.1);

                // if (inf_ratio == 1.0)
                // {
                //     successful_inflation = true;
                //     // inf_ratio = 1.0;
                //     // break;
                // }
                // continue;
                // set back to original values
                // inflatedLeftIdx = leftIdx;
                // inflatedLeftRange = leftRange;
                // inflatedRightIdx = rightIdx;
                // inflatedRightRange = rightRange;
                // inflatedLeftTheta = leftTheta;
                // inflatedRightTheta = rightTheta;
    
                // return false;
            } else
            {
                successful_inflation = true;
                ROS_INFO_STREAM_NAMED("GapManipulator", "        inflation succeeded, new points in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
            }
        }

        if (! successful_inflation)
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "        inflation has failed for good.");
         
            gap->setManipPoints(leftIdx, rightIdx, leftRange, rightRange);
         
            gap->getLeftGapPt()->getModel()->setNewPosition(leftTheta, leftRange); // manipulating left point
            gap->getRightGapPt()->getModel()->setNewPosition(rightTheta, rightRange); // manipulating right point
    
            return;
            // return false;
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

        return;
        // return true;
    }
}
