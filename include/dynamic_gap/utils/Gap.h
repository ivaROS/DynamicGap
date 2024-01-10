#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Utils.h>
// #include <geometry_msgs/Point.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>
// #include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dynamic_gap/gap_estimation/RotatingFrameCartesianKalmanFilter.h>
#include <dynamic_gap/gap_estimation/PerfectEstimator.h>

namespace dynamic_gap
{
    class Gap
    {
        public:
            // Gap() {};

            // colon used here is an initialization list. helpful for const variables.
            Gap(const std::string & frame, const int & rightIdx, const float & rangeRight, const bool & radial, const float & minSafeDist_) : 
                frame_(frame), rightIdx_(rightIdx), rightDist_(rangeRight), radial_(radial), minSafeDist_(minSafeDist_)
            {
                extendedGapOrigin_ << 0.0, 0.0;
                termExtendedGapOrigin_ << 0.0, 0.0;
                
                leftBezierOrigin_ << 0.0, 0.0;
                rightBezierOrigin_ << 0.0, 0.0;

                // Here, you can define what type of model you want to use
                leftGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                rightGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
            };

            
            // For now, just using the default copy constructor
            //      used in 
            Gap(const dynamic_gap::Gap & otherGap)
            {
                ROS_INFO_STREAM_NAMED("Gap", "in copy constructor");
                frame_ = otherGap.frame_;

                // copy all the variables
                leftIdx_ = otherGap.leftIdx_;
                leftDist_ = otherGap.leftDist_;
                rightIdx_ = otherGap.rightIdx_;
                rightDist_ = otherGap.rightDist_;

                termLeftIdx_ = otherGap.termLeftIdx_;
                termLeftDist_ = otherGap.termLeftDist_;
                termRightIdx_ = otherGap.termRightIdx_;
                termRightDist_ = otherGap.termRightDist_;

                convex.leftIdx_ = otherGap.convex.leftIdx_;
                convex.leftDist_ = otherGap.convex.leftDist_;
                convex.rightIdx_ = otherGap.convex.rightIdx_;
                convex.rightDist_ = otherGap.convex.rightDist_;

                convex.termLeftIdx_ = otherGap.convex.termLeftIdx_;
                convex.termLeftDist_ = otherGap.convex.termLeftDist_;
                convex.termRightIdx_ = otherGap.convex.termRightIdx_;
                convex.termRightDist_ = otherGap.convex.termRightDist_;

                rightType_ = otherGap.rightType_;
                terminalRightType_ = otherGap.terminalRightType_;

                radial_ = otherGap.radial_;
                termRadial_ = otherGap.termRadial_;

                minSafeDist_ = otherGap.minSafeDist_;
                terminalMinSafeDist_ = otherGap.terminalMinSafeDist_;

                category_ = otherGap.category_;

                crossingPt_ = otherGap.crossingPt_;
                closingPt_ = otherGap.closingPt_;

                goal.x_ = otherGap.goal.x_;
                goal.y_ = otherGap.goal.y_;

                terminalGoal.x_ = otherGap.terminalGoal.x_;
                terminalGoal.y_ = otherGap.terminalGoal.y_;

                gapLifespan_ = otherGap.gapLifespan_;

                peakVelX_ = otherGap.peakVelX_;
                peakVelY_ = otherGap.peakVelY_;

                crossed_ = otherGap.crossed_;
                closed_ = otherGap.closed_;
                crossedBehind_ = otherGap.crossedBehind_;
                artificial_ = otherGap.artificial_;

                mode.reduced_ = otherGap.mode.reduced_;
                mode.convex_ = otherGap.mode.convex_;
                mode.RGC_ = otherGap.mode.RGC_;
                mode.termReduced_ = otherGap.mode.termReduced_;
                mode.termConvex_ = otherGap.mode.termConvex_;
                mode.termRGC_ = otherGap.mode.termRGC_;

                // make new models
                // Here, you can define what type of model you want to use
                leftGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                rightGapPtModel_ = new RotatingFrameCartesianKalmanFilter();

                // transfer models (need to deep copy the models, not just the pointers)
                leftGapPtModel_->transfer(*otherGap.leftGapPtModel_);
                rightGapPtModel_->transfer(*otherGap.rightGapPtModel_);
            }

            ~Gap() 
            {
                delete leftGapPtModel_;
                delete rightGapPtModel_;
            };
            
            // Setters and Getters for LR Distance and Index (initial and terminal gaps)
            int LIdx() const { return leftIdx_; }
            void setLIdx(const int & lidx) { leftIdx_ = lidx; }

            int RIdx() const { return rightIdx_; }
            void setRIdx(const int & ridx) { rightIdx_ = ridx; }

            float LDist() const { return leftDist_; }
            void setLDist(const float & ldist) { leftDist_ = ldist; }

            float RDist() const { return rightDist_; }
            void setRDist(const float & rdist) { rightDist_ = rdist; }

            int termLIdx() const { return termLeftIdx_; }
            void setTermLIdx(const int & termLeftIdx) { termLeftIdx_ = termLeftIdx; }

            int termRIdx() const { return termRightIdx_; }
            void setTermRIdx(const int & termRightIdx) { termRightIdx_ = termRightIdx; }

            float termLDist() const { return termLeftDist_; }
            void setTermLDist(const float & termLDist) { termLeftDist_ = termLDist; }

            float termRDist() const { return termRightDist_; }
            void setTermRDist(const float & termRDist) { termRightDist_ = termRDist; }

            int cvxLeftIdx() const { return convex.leftIdx_; }
            void setCvxLeftIdx(const int & cvxLeftIdx) { convex.leftIdx_ = cvxLeftIdx; }

            int cvxRightIdx() const { return convex.rightIdx_; }
            void setCvxRightIdx(const int & cvxRightIdx) { convex.rightIdx_ = cvxRightIdx; }

            float cvxLeftDist() const { return convex.leftDist_; }
            void setCvxLeftDist(const float & cvxLeftDist) { convex.leftDist_ = cvxLeftDist; }

            float cvxRightDist() const { return convex.rightDist_; }
            void setCvxRightDist(const float & cvxRightDist) { convex.rightDist_ = cvxRightDist; }

            int cvxTermLeftIdx() const { return convex.termLeftIdx_; }
            void setcvxTermLeftIdx(const int & cvxTermLeftIdx) { convex.termLeftIdx_ = cvxTermLeftIdx; }

            int cvxTermRightIdx() const { return convex.termRightIdx_; }
            void setcvxTermRightIdx(const int & cvxTermRightIdx) { convex.termRightIdx_ = cvxTermRightIdx; }

            float cvxTermLeftDist() const { return convex.termLeftDist_; }
            void setcvxTermLeftDist(const float & cvxTermLeftDist) { convex.termLeftDist_ = cvxTermLeftDist; }

            float cvxTermRightDist() const { return convex.termRightDist_; }
            void setcvxTermRightDist(const float & cvxTermRightDist) { convex.termRightDist_ = cvxTermRightDist; }

            // Concluding the Gap after constructing with left information
            void addLeftInformation(const int & left_idx, const float & rangeLeft) 
            {
                leftIdx_ = left_idx;
                leftDist_ = rangeLeft;
                rightType_ = rightDist_ < leftDist_;

                // if (!radial_)
                // {
                //     radial_ = isRadial();
                // }

                // initializing convex polar gap coordinates to raw ones
                convex.rightIdx_ = rightIdx_;
                convex.leftIdx_ = leftIdx_;
                convex.rightDist_ = rightDist_;
                convex.leftDist_ = leftDist_;

                setRadial();
            }

            void addTerminalRightInformation()
            {
                terminalRightType_ = termRightDist_ < termLeftDist_;

                // if (!termRadial_)
                // {
                //     termRadial_ = isRadial();
                // }

                convex.termRightIdx_ = termRightIdx_;
                convex.termLeftIdx_ = termLeftIdx_;
                convex.termRightDist_ = termRightDist_;
                convex.termLeftDist_ = termLeftDist_;

                setRadial(false);
            }

            // Get Left Cartesian Distance
            // edited by Max: float &x, float &y
            void getLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(leftIdx_);
                x = (leftDist_) * cos(thetaLeft);
                y = (leftDist_) * sin(thetaLeft);
            }

            // Get Right Cartesian Distance
            void getRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(rightIdx_);
                x = (rightDist_) * cos(thetaRight);
                y = (rightDist_) * sin(thetaRight);
            }

            void getSimplifiedLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(convex.leftIdx_);
                // std::cout << "rightDist_: " << rightDist_ << ", rightIdx_: " << rightIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.leftDist_) * cos(thetaLeft);
                y = (convex.leftDist_) * sin(thetaLeft);
            }

            void getSimplifiedRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(convex.rightIdx_);
                // std::cout << "leftDist_: " << leftDist_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.rightDist_) * cos(thetaRight);
                y = (convex.rightDist_) * sin(thetaRight);
            }

            void getSimplifiedTerminalLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(convex.termLeftIdx_);
                // std::cout << "rightDist_: " << rightDist_ << ", rightIdx_: " << rightIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.termLeftDist_) * cos(thetaLeft);
                y = (convex.termLeftDist_) * sin(thetaLeft);
            }

            void getSimplifiedTerminalRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(convex.termRightIdx_);
                // std::cout << "leftDist_: " << leftDist_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.termRightDist_) * cos(thetaRight);
                y = (convex.termRightDist_) * sin(thetaRight);
            }

            void initManipIndices() 
            {
                convex.rightIdx_ = rightIdx_;
                convex.rightDist_ = rightDist_;
                convex.leftIdx_ = leftIdx_;
                convex.leftDist_ = leftDist_;

                convex.termRightIdx_ = termRightIdx_;
                convex.termRightDist_ = termRightDist_;
                convex.termLeftIdx_ = termLeftIdx_;
                convex.termLeftDist_ = termLeftDist_;
            }

            void setRadial(const bool & initial = true)
            {
                // ROS_INFO_STREAM_NAMED("Gap", "setRadial:");
                // does resoln here imply 360 deg FOV?
                int checkLeftIdx = initial ? leftIdx_ : termLeftIdx_;
                int checkRightIdx = initial ? rightIdx_ : termRightIdx_;

                float checkLeftDist = initial ? leftDist_ : termLeftDist_;
                float checkRightDist = initial ? rightDist_ : termRightDist_;

                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftIdx: " << checkLeftIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftDist: " << checkLeftDist);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightIdx: " << checkRightIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightDist: " << checkRightDist);

                float resoln = M_PI / half_scan;
                float gapAngle = (checkLeftIdx - checkRightIdx) * resoln;
                if (gapAngle < 0)
                    gapAngle += 2*M_PI;

                // ROS_INFO_STREAM_NAMED("Gap", "   gapAngle: " << gapAngle);
                float nearRange = rightType_ ? checkRightDist : checkLeftDist;
                // law of cosines
                float leftPtToRightPtDist = sqrt(pow(checkRightDist, 2) + pow(checkLeftDist, 2) - 2 * checkRightDist * checkLeftDist * cos(gapAngle));
                // law of sines
                float farSideAngle = asin(epsilonDivide(nearRange, leftPtToRightPtDist) * sin(gapAngle));
                
                // ROS_INFO_STREAM_NAMED("Gap", "nearRange: " << nearRange);
                // ROS_INFO_STREAM_NAMED("Gap", "leftPtToRightPtDist: " << leftPtToRightPtDist);
                // ROS_INFO_STREAM_NAMED("Gap", "small angle: " << farSideAngle);

                // ROS_INFO_STREAM_NAMED("Gap", "   farSideAngle: " << farSideAngle);
                // ROS_INFO_STREAM_NAMED("Gap", "   gapAngle: " << gapAngle);
                float nearSideAngle = (M_PI - farSideAngle - gapAngle);
                // ROS_INFO_STREAM_NAMED("Gap", "   nearSideAngle: " << nearSideAngle);

                if (initial)
                    radial_ = nearSideAngle > 0.75 * M_PI;
                else
                    termRadial_ = nearSideAngle > 0.75 * M_PI;     
            }

            bool isRadial(const bool & initial = true) const
            {
                return (initial ? radial_ : termRadial_);
            }

            bool isRightType(const bool & initial = true) const
            {
                if (initial)
                    return rightType_;
                else
                    return terminalRightType_;
            }

            float getMinSafeDist() { return minSafeDist_; }

            void setTerminalMinSafeDist(const float & dist) { terminalMinSafeDist_ = dist; }
            float getTerminalMinSafeDist() { return terminalMinSafeDist_; }

            void setCategory(const std::string & category) { category_ = category; }
            std::string getCategory() { return category_; }

            void setCrossingPoint(const float & x, const float & y) { crossingPt_ << x,y; }
            Eigen::Vector2f getCrossingPoint() { return crossingPt_; }

            void setClosingPoint(const float & x, const float & y) { closingPt_ << x,y; }
            Eigen::Vector2f getClosingPoint() { return closingPt_; }

            // used in calculating nearSideAngle, the angle formed between the two gap lines and the robot. (angle of the gap).
            // calculates the euclidean distance between the left and right gap points using the law of cosines
            float get_gap_euclidean_dist() const 
            {
                float resoln = M_PI / half_scan;
                float gapAngle = (leftIdx_ - rightIdx_) * resoln;
                if (gapAngle < 0)
                    gapAngle += 2*M_PI;

                return sqrt(pow(rightDist_, 2) + pow(leftDist_, 2) - 2 * rightDist_ * leftDist_ * cos(gapAngle));
            }

            void setTerminalPoints(const float & termLeftIdx, const float & termLeftDist, 
                                   const float & termRightIdx, const float & termRightDist) 
            {    
                termLeftIdx_ = termLeftIdx;
                termLeftDist_ = termLeftDist;
                termRightIdx_ = termRightIdx;
                termRightDist_ = termRightDist;

                if (termLeftIdx_ == termRightIdx_) 
                {
                    // ROS_INFO_STREAM_NAMED("Gap", "terminal indices are the same");
                    termLeftIdx_ = (termLeftIdx_ + 1) % 512;
                }
                // ROS_INFO_STREAM_NAMED("Gap", "setting terminal points to, left: (" << termLeftIdx_ << ", " << termLeftDist_ << "), right: ("  << termRightIdx_ << ", " << termRightDist_ << ")");
            
                terminalRightType_ = termRightDist_ < termLeftDist_;

                setRadial(false);
            }

            void printCartesianPoints(const bool & initial, const bool & simplified) 
            {
                float xLeft = 0.0, yLeft = 0.0, xRight = 0.0, yRight = 0.0;
                float thetaLeft = 0.0, thetaRight = 0.0, rangeLeft = 0.0, rangeRight = 0.0;
                if (initial) 
                {
                    if (simplified) 
                    {
                        thetaLeft = idx2theta(leftIdx_);
                        thetaRight = idx2theta(rightIdx_);
                        rangeLeft = leftDist_;
                        rangeRight = rightDist_;
                    } else 
                    {
                        thetaLeft = idx2theta(convex.leftIdx_);
                        thetaRight = idx2theta(convex.rightIdx_);    
                        rangeLeft = convex.leftDist_;
                        rangeRight = convex.rightDist_;                                                         
                    }
                } else 
                {
                    if (simplified) 
                    {
                        thetaLeft = idx2theta(termLeftIdx_);
                        thetaRight = idx2theta(termRightIdx_);     
                        rangeLeft = termLeftDist_;
                        rangeRight = termRightDist_;          
                    } else 
                    {
                        thetaLeft = idx2theta(convex.termLeftIdx_);
                        thetaRight = idx2theta(convex.termRightIdx_);    
                        rangeLeft = convex.termLeftDist_;
                        rangeRight = convex.termRightDist_;                       
                    }
                }

                xLeft = rangeLeft * cos(thetaLeft);
                yLeft = rangeLeft * sin(thetaLeft);
                xRight = rangeRight * cos(thetaRight);
                yRight = rangeRight * sin(thetaRight);

                ROS_INFO_STREAM_NAMED("Gap", "xLeft, yLeft: (" << xLeft << ", " << yLeft << "), xRight,yRight: (" << xRight << ", " << yRight << ")");
            }   
            
            void setGoal(const bool & initial, const Eigen::Vector2f & goalVector)
            {
                if (initial)
                {
                    goal.x_ = goalVector[0];
                    goal.y_ = goalVector[1];
                } else
                {
                    terminalGoal.x_ = goalVector[0];
                    terminalGoal.y_ = goalVector[1];
                }
            }

            float gapLifespan_ = 5.0;

            float minSafeDist_ = 0.0, terminalMinSafeDist_ = 0.0;
            Eigen::Vector2f extendedGapOrigin_, termExtendedGapOrigin_;
            Eigen::Vector2f leftBezierOrigin_, rightBezierOrigin_;
            float half_scan = 256;

            std::string frame_ = "";
            bool radial_ = false;
            bool termRadial_ = false;
            bool rightType_ = false;
            bool terminalRightType_ = false;

            float peakVelX_ = 0.0, peakVelY_ = 0.0;

            struct GapMode 
            {
                bool reduced_ = false;
                bool convex_ = false;
                bool RGC_ = false;
                bool termReduced_ = false;
                bool termConvex_ = false;
                bool termRGC_ = false;
            } mode;

            struct Goal 
            {
                float x_ = 0.0, y_ = 0.0;
                // bool set = false;
                // bool discard = false;
                // bool goalwithin = false;
            } goal;

            struct TerminalGoal 
            {
                float x_ = 0.0, y_ = 0.0;
                // bool set = false;
                // bool discard = false;
                // bool goalwithin = false;
            } terminalGoal;

            Estimator * rightGapPtModel_ = NULL;
            Estimator * leftGapPtModel_ = NULL;

            // int _index;
            std::string category_ = "";
            Eigen::Vector2f crossingPt_, closingPt_;

            bool crossed_ = false;
            bool closed_ = false;
            bool crossedBehind_ = false;

            bool artificial_ = false;

            float leftWeight_ = 0.0, rightWeight_ = 0.0;
            Eigen::MatrixXd leftRightCenters_, allCurvePts_;
            Eigen::Vector4f splineXCoefs_, splineYCoefs_;

            Eigen::Vector2d leftPt0_, leftPt1_, rightPt0_, rightPt1_;
            int numLeftRGEPoints_ = 0, numRightRGEPoints_ = 0;

        private:

            int leftIdx_ = 511;
            int rightIdx_ = 0;
            float rightDist_ = 5;
            float leftDist_ = 5;
            int termLeftIdx_ = 511;
            int termRightIdx_ = 0;
            float termRightDist_ = 5;
            float termLeftDist_ = 5;
            
            struct Convex 
            {
                int leftIdx_ = 511;
                int rightIdx_ = 0;
                float leftDist_ = 5;
                float rightDist_ = 5;
                int termLeftIdx_ = 511;
                int termRightIdx_ = 0;
                float termLeftDist_ = 5;
                float termRightDist_ = 5;
            } convex;


    };
}