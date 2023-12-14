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
#include <dynamic_gap/gap_estimation/StaticEstimator.h>

namespace dynamic_gap
{
    class Gap
    {
        public:
            Gap() {};

            // colon used here is an initialization list. helpful for const variables.
            Gap(std::string frame, int right_idx, float rangeRight, bool radial, float minSafeDist_) : 
                frame_(frame), rightIdx_(right_idx), rightDist_(rangeRight), radial_(radial), minSafeDist_(minSafeDist_)
            {
                extendedGapOrigin_ << 0.0, 0.0;
                termExtendedGapOrigin_ << 0.0, 0.0;
                rightBezierOrigin_ << 0.0, 0.0;
                leftBezierOrigin_ << 0.0, 0.0;
            };

            ~Gap() {};
            
            // Setters and Getters for LR Distance and Index (initial and terminal gaps)
            int LIdx() const { return leftIdx_; }
            void setLIdx(int lidx) { leftIdx_ = lidx; }

            int RIdx() const { return rightIdx_; }
            void setRIdx(int ridx) { rightIdx_ = ridx; }

            float LDist() const { return leftDist_; }
            void setLDist(float ldist) { leftDist_ = ldist; }

            float RDist() const { return rightDist_; }
            void setRDist(float rdist) { rightDist_ = rdist; }

            int termRIdx() const { return termRightIdx_; }
            void setTermRIdx(int termRightIdx_) { termRightIdx_ = termRightIdx_; }

            int termLIdx() const { return termLeftIdx_; }
            void setTermLIdx(int termLeftIdx_) { termLeftIdx_ = termLeftIdx_; }

            float termRDist() const { return termRightDist_; }
            void setTermRDist(float termRDist) { termRightDist_ = termRDist; }

            float termLDist() const { return termLeftDist_; }
            void setTermLDist(float termLDist) { termLeftDist_ = termLDist; }

            int cvxRightIdx() const { return convex.rightIdx_; }
            void setCvxRightIdx(int cvxRightIdx) { convex.rightIdx_ = cvxRightIdx; }

            int cvxLeftIdx() const { return convex.leftIdx_; }
            void setCvxLeftIdx(int cvxLeftIdx) { convex.leftIdx_ = cvxLeftIdx; }

            float cvxRightDist() const { return convex.rightDist_; }
            void setCvxRightDist(float cvxRightDist) { convex.rightDist_ = cvxRightDist; }

            float cvxLeftDist() const { return convex.leftDist_; }
            void setCvxLeftDist(float cvxLeftDist) { convex.leftDist_ = cvxLeftDist; }

            int cvxTermRightIdx() const { return convex.termRightIdx_; }
            void setcvxTermRightIdx(int cvxTermRightIdx) { convex.termRightIdx_ = cvxTermRightIdx; }

            int cvxTermLeftIdx() const { return convex.termLeftIdx_; }
            void setcvxTermLeftIdx(int cvxTermLeftIdx) { convex.termLeftIdx_ = cvxTermLeftIdx; }

            float cvxTermRightDist() const { return convex.termRightDist_; }
            void setcvxTermRightDist(float cvxTermRightDist) { convex.termRightDist_ = cvxTermRightDist; }

            float cvxTermLeftDist() const { return convex.termLeftDist_; }
            void setcvxTermLeftDist(float cvxTermLeftDist) { convex.termLeftDist_ = cvxTermLeftDist; }

            // Concluding the Gap after constructing with left information
            void addLeftInformation(int left_idx, float rangeLeft) 
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

            void setRadial(bool initial = true)
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
                float nearangeRight = rightType_ ? checkRightDist : checkLeftDist;
                // law of cosines
                float leftPtToRightPtDist = sqrt(pow(checkRightDist, 2) + pow(checkLeftDist, 2) - 2 * checkRightDist * checkLeftDist * cos(gapAngle));
                // law of sines
                float farSideAngle = asin((nearangeRight / leftPtToRightPtDist) * sin(gapAngle));
                
                // ROS_INFO_STREAM_NAMED("Gap", "nearangeRight: " << nearangeRight);
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

            bool isRadial(bool initial = true) const
            {
                return (initial ? radial_ : termRadial_);
            }

            bool isRightType(bool initial = true) const
            {
                if (initial)
                    return rightType_;
                else
                    return terminalRightType_;
            }

            float getMinSafeDist() { return minSafeDist_; }

            void setTerminalMinSafeDist(float _dist) { terminalMinSafeDist_ = _dist; }
            float getTerminalMinSafeDist() { return terminalMinSafeDist_; }

            void setCategory(std::string category) { category_ = category; }
            std::string getCategory() { return category_; }

            void setCrossingPoint(float x, float y) { crossingPt_ << x,y; }
            Eigen::Vector2f getCrossingPoint() { return crossingPt_; }

            void setClosingPoint(float x, float y) { closingPt_ << x,y; }
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

            void setTerminalPoints(float termLeftIdx, float termLeftDist, float termRightIdx, float termRightDist) 
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

            void printCartesianPoints(bool initial, bool simplified) 
            {
                float xLeft, yLeft, xRight, yRight;
                float thetaLeft, thetaRight, rangeLeft, rangeRight;
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
            
            float gapLifespan_ = 5.0;

            float minSafeDist_, terminalMinSafeDist_;
            Eigen::Vector2f extendedGapOrigin_, termExtendedGapOrigin_;
            Eigen::Vector2f leftBezierOrigin_, rightBezierOrigin_;
            float half_scan = 256;

            std::string frame_ = "";
            bool radial_ = false;
            bool termRadial_ = false;
            bool rightType_ = false;
            bool terminalRightType_ = false;

            float peakVelX_, peakVelY_ = 0.0;

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
                float x_, y_;
                // bool set = false;
                // bool discard = false;
                // bool goalwithin = false;
            } goal;

            struct TerminalGoal 
            {
                float x_, y_;
                // bool set = false;
                // bool discard = false;
                // bool goalwithin = false;
            } terminalGoal;

            Estimator * rightGapPtModel_;
            Estimator * leftGapPtModel_;
            // int _index;
            std::string category_;
            Eigen::Vector2f crossingPt_, closingPt_;

            bool crossed_ = false;
            bool closed_ = false;
            bool crossedBehind_ = false;

            bool artificial_ = false;

            float leftWeight_, rightWeight_;
            Eigen::MatrixXd leftRightCenters_, allCurvePts_;
            Eigen::Vector4f splineXCoefs_, splineYCoefs_;

            Eigen::Vector2d leftPt0_, leftPt1_, rightPt0_, rightPt1_;
            int numLeftRGEPoints_, numRightRGEPoints_;

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