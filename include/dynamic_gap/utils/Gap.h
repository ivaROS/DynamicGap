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
            Gap(std::string frame, int right_idx, float rdist, bool radial, float minSafeDist_) : 
                frame_(frame), rightIdx_(right_idx), rightDist_(rdist), radial_(radial), minSafeDist_(minSafeDist_)
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
            void setCvxRIdx(int cvxRightIdx) { convex.rightIdx_ = cvxRightIdx; }

            int cvxLeftIdx() const { return convex.leftIdx_; }
            void setCvxLIdx(int cvxLeftIdx) { convex.leftIdx_ = cvxLeftIdx; }

            float cvxRightDist() const { return convex.rightDist_; }
            void setCvxRDist(float cvxRightDist) { convex.rightDist_ = cvxRightDist; }

            float cvxLeftDist() const { return convex.leftDist_; }
            void setCvxLDist(float cvxLeftDist) { convex.leftDist_ = cvxLeftDist; }

            int cvxTermRIdx() const { return convex.termRightIdx_; }
            void setCvxTermRIdx(int cvxTermRIdx) { convex.termRightIdx_ = cvxTermRIdx; }

            int cvxTermLIdx() const { return convex.termLeftIdx_; }
            void setCvxTermLIdx(int cvxTermLIdx) { convex.termLeftIdx_ = cvxTermLIdx; }

            float cvxTermRDist() const { return convex.termRightDist_; }
            void setCvxTermRDist(float cvxTermRDist) { convex.termRightDist_ = cvxTermRDist; }

            float cvxTermLDist() const { return convex.termLeftDist_; }
            void setCvxTermLDist(float cvxTermLDist) { convex.termLeftDist_ = cvxTermLDist; }

            // Concluding the Gap after constructing with left information
            void addLeftInformation(int left_idx, float ldist) 
            {
                leftIdx_ = left_idx;
                leftDist_ = ldist;
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
                float ltheta = idx2theta(leftIdx_);
                x = (leftDist_) * cos(ltheta);
                y = (leftDist_) * sin(ltheta);
            }

            // Get Right Cartesian Distance
            void getRCartesian(float &x, float &y) const
            {
                float rtheta = idx2theta(rightIdx_);
                x = (rightDist_) * cos(rtheta);
                y = (rightDist_) * sin(rtheta);
            }

            void getSimplifiedLCartesian(float &x, float &y) const
            {
                float ltheta = idx2theta(convex.leftIdx_);
                // std::cout << "rightDist_: " << rightDist_ << ", rightIdx_: " << rightIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.leftDist_) * cos(ltheta);
                y = (convex.leftDist_) * sin(ltheta);
            }

            void getSimplifiedRCartesian(float &x, float &y) const
            {
                float rtheta = idx2theta(convex.rightIdx_);
                // std::cout << "leftDist_: " << leftDist_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.rightDist_) * cos(rtheta);
                y = (convex.rightDist_) * sin(rtheta);
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
                // ROS_INFO_STREAM("setRadial:");
                // does resoln here imply 360 deg FOV?
                int check_l_idx = initial ? leftIdx_ : termLeftIdx_;
                int check_r_idx = initial ? rightIdx_ : termRightIdx_;

                float check_l_dist = initial ? leftDist_ : termLeftDist_;
                float check_r_dist = initial ? rightDist_ : termRightDist_;

                // ROS_INFO_STREAM("   check_l_idx: " << check_l_idx);
                // ROS_INFO_STREAM("   check_l_dist: " << check_l_dist);
                // ROS_INFO_STREAM("   check_r_idx: " << check_r_idx);
                // ROS_INFO_STREAM("   check_r_dist: " << check_r_dist);

                float resoln = M_PI / half_scan;
                float gap_angle = (check_l_idx - check_r_idx) * resoln;
                if (gap_angle < 0)
                    gap_angle += 2*M_PI;

                // ROS_INFO_STREAM("   gap_angle: " << gap_angle);
                float short_side = rightType_ ? check_r_dist : check_l_dist;
                // law of cosines
                float opp_side = sqrt(pow(check_r_dist, 2) + pow(check_l_dist, 2) - 2 * check_r_dist * check_l_dist * cos(gap_angle));
                // law of sines
                float small_angle = asin((short_side / opp_side) * sin(gap_angle));
                // ROS_INFO_STREAM("short_side: " << short_side);
                // ROS_INFO_STREAM("opp_side: " << opp_side);
                // ROS_INFO_STREAM("small angle: " << small_angle);

                // ROS_INFO_STREAM("   small_angle: " << small_angle);
                // ROS_INFO_STREAM("   gap_angle: " << gap_angle);
                float alpha = (M_PI - small_angle - gap_angle);
                // ROS_INFO_STREAM("   alpha: " << alpha);

                if (initial)
                    radial_ = alpha > 0.75 * M_PI;
                else
                    termRadial_ = alpha > 0.75 * M_PI;     
            }

            bool isRadial(bool initial = true) const
            {
                return (initial ? radial_ : termRadial_);
            }

            // void setRadial()
            // {
            //     radial_ = false;
            // }

            bool isRightType(bool initial = true) const
            {
                if (initial)
                    return rightType_;
                else
                    return terminalRightType_;
            }

            /*
            void resetFrame(std::string frame) 
            {
                frame_ = frame;
            }
            */

            float getMinSafeDist() 
            {
                return minSafeDist_;
            }

            void setTerminalMinSafeDist(float _dist) 
            {
                terminalMinSafeDist_ = _dist;
            }

            float getTerminalMinSafeDist() 
            {
                return terminalMinSafeDist_;
            }

            /*
            std::string getFrame() 
            {
                return frame_;
            }
            */

            void setCategory(std::string category) 
            {
                // ROS_INFO_STREAM("setting category to: " << _category_);
                category_ = category;
            }

            std::string getCategory() {
                return category_;
            }

            void setCrossingPoint(float x, float y) {
                // ROS_INFO_STREAM("setting crossing point to: " << x << ", " << y);
                crossingPt_ << x,y;
            }

            Eigen::Vector2f getCrossingPoint() {
                return crossingPt_;
            }

            void setClosingPoint(float x, float y) {
                // ROS_INFO_STREAM("setting closing point to: " << x << ", " << y);
                closingPt_ << x,y;
            }

            Eigen::Vector2f getClosingPoint() {
                return closingPt_;
            }

            // used in calculating alpha, the angle formed between the two gap lines and the robot. (angle of the gap).
            // calculates the euclidean distance between the left and right gap points using the law of cosines
            float get_gap_euclidean_dist() const 
            {
                int idx_diff = leftIdx_ - rightIdx_;
                if (idx_diff < 0) {
                    idx_diff += (2*half_scan);
                } 
                float resoln = M_PI / half_scan;
                float gap_angle = float(idx_diff) * resoln;
                return sqrt(pow(rightDist_, 2) + pow(leftDist_, 2) - 2 * rightDist_ * leftDist_ * cos(gap_angle));
            }

            void setTerminalPoints(float termLeftIdx, float termLeftDist, float termRightIdx, float termRightDist) 
            {    

                termLeftIdx_ = termLeftIdx;
                termLeftDist_ = termLeftDist;
                termRightIdx_ = termRightIdx;
                termRightDist_ = termRightDist;

                if (termLeftIdx_ == termRightIdx_) 
                {
                    // ROS_INFO_STREAM("terminal indices are the same");
                    termLeftIdx_ = (termLeftIdx_ + 1) % 512;
                }
                // ROS_INFO_STREAM("setting terminal points to, left: (" << termLeftIdx_ << ", " << termLeftDist_ << "), right: ("  << termRightIdx_ << ", " << termRightDist_ << ")");
            
                terminalRightType_ = termRightDist_ < termLeftDist_;

                setRadial(false);
            }

            void printCartesianPoints(bool initial, bool simplified) 
            {
                float x_l, y_l, x_r, y_r;
                float ltheta, rtheta, ldist, rdist;
                if (initial) 
                {
                    if (simplified) 
                    {
                        ltheta = idx2theta(leftIdx_);
                        rtheta = idx2theta(rightIdx_);
                        ldist = leftDist_;
                        rdist = rightDist_;
                    } else 
                    {
                        ltheta = idx2theta(convex.leftIdx_);
                        rtheta = idx2theta(convex.rightIdx_);    
                        ldist = convex.leftDist_;
                        rdist = convex.rightDist_;                                                         
                    }
                } else 
                {
                    if (simplified) 
                    {
                        ltheta = idx2theta(termLeftIdx_);
                        rtheta = idx2theta(termRightIdx_);     
                        ldist = termLeftDist_;
                        rdist = termRightDist_;          
                    } else 
                    {
                        ltheta = idx2theta(convex.termLeftIdx_);
                        rtheta = idx2theta(convex.termRightIdx_);    
                        ldist = convex.termLeftDist_;
                        rdist = convex.termRightDist_;                       
                    }
                }

                x_l = ldist * cos(ltheta);
                y_l = ldist * sin(ltheta);
                x_r = rdist * cos(rtheta);
                y_r = rdist * sin(rtheta);

                ROS_INFO_STREAM("x_l, y_l: (" << x_l << ", " << y_l << "), x_r,y_r: (" << x_r << ", " << y_r << ")");
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