#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Utils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dynamic_gap/gap_estimation/RotatingFrameCartesianKalmanFilter.h>
#include <dynamic_gap/gap_estimation/PerfectEstimator.h>

namespace dynamic_gap
{
    /**
    * \brief Class for a single gap
    */
    class Gap
    {
        public:
            // Gap() {};

            // colon used here is an initialization list. helpful for const variables.
            Gap(const std::string & frame, const int & rightIdx, const float & rangeRight, const bool & radial, const float & minSafeDist_) : 
                frame_(frame), rightIdx_(rightIdx), rightRange_(rangeRight), radial_(radial), minSafeDist_(minSafeDist_)
            {
                extendedGapOrigin_ << 0.0, 0.0;
                termExtendedGapOrigin_ << 0.0, 0.0;

                // Here, you can define what type of model you want to use
                leftGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                rightGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                // leftGapPtModel_ = new PerfectEstimator();
                // rightGapPtModel_ = new PerfectEstimator();
            };

            Gap(const dynamic_gap::Gap & otherGap)
            {
                // ROS_INFO_STREAM_NAMED("Gap", "in copy constructor");
                gapLifespan_ = otherGap.gapLifespan_;
                minSafeDist_ = otherGap.minSafeDist_;
                extendedGapOrigin_ = otherGap.extendedGapOrigin_;
                termExtendedGapOrigin_ = otherGap.termExtendedGapOrigin_;

                frame_ = otherGap.frame_;

                radial_ = otherGap.radial_;

                rightType_ = otherGap.rightType_;

                goal = otherGap.goal;
                // goal.x_ = otherGap.goal.x_;
                // goal.y_ = otherGap.goal.y_;

                terminalGoal = otherGap.terminalGoal;
                // terminalGoal.x_ = otherGap.terminalGoal.x_;
                // terminalGoal.y_ = otherGap.terminalGoal.y_;

                // deep copy for new models
                // Here, you can define what type of model you want to use

                leftGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                rightGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                // leftGapPtModel_ = new PerfectEstimator();
                // rightGapPtModel_ = new PerfectEstimator();

                // transfer models (need to deep copy the models, not just the pointers)
                leftGapPtModel_->transfer(*otherGap.leftGapPtModel_);
                rightGapPtModel_->transfer(*otherGap.rightGapPtModel_);

                globalGoalWithin = otherGap.globalGoalWithin;

                t_intercept_left = otherGap.t_intercept_left;
                t_intercept_right = otherGap.t_intercept_right;
                gamma_intercept_left = otherGap.gamma_intercept_left;
                gamma_intercept_right = otherGap.gamma_intercept_right;

                end_condition = otherGap.end_condition;

                // copy all the variables
                leftIdx_ = otherGap.leftIdx_;
                leftRange_ = otherGap.leftRange_;
                rightIdx_ = otherGap.rightIdx_;
                rightRange_ = otherGap.rightRange_;

                // termLeftIdx_ = otherGap.termLeftIdx_;
                // termLeftRange_ = otherGap.termLeftRange_;
                // termRightIdx_ = otherGap.termRightIdx_;
                // termRightRange_ = otherGap.termRightRange_;

                manip = otherGap.manip;

                rgc_ = otherGap.rgc_;

                // manip.leftIdx_ = otherGap.manip.leftIdx_;
                // manip.leftRange_ = otherGap.manip.leftRange_;
                // manip.rightIdx_ = otherGap.manip.rightIdx_;
                // manip.rightRange_ = otherGap.manip.rightRange_;

                // manip.termLeftIdx_ = otherGap.manip.termLeftIdx_;
                // manip.termLeftRange_ = otherGap.manip.termLeftRange_;
                // manip.termRightIdx_ = otherGap.manip.termRightIdx_;
                // manip.termRightRange_ = otherGap.manip.termRightRange_;
            }

            ~Gap() 
            {
                delete leftGapPtModel_;
                delete rightGapPtModel_;
            };
            
            /**
            * \brief Getter for initial left gap point index
            * \return initial left gap point index
            */
            int LIdx() const { return leftIdx_; }

            /**
            * \brief Setter for initial left gap point index
            * \param lidx initial left gap point index
            */
            void setLIdx(const int & lidx) { leftIdx_ = lidx; }

            /**
            * \brief Getter for initial right gap point index
            * \return initial right gap point index
            */
            int RIdx() const { return rightIdx_; }

            /**
            * \brief Setter for initial right gap point index
            * \param ridx initial right gap point index
            */            
            void setRIdx(const int & ridx) { rightIdx_ = ridx; }

            /**
            * \brief Getter for initial left gap point range
            * \return initial left gap point range
            */
            float LRange() const { return leftRange_; }

            /**
            * \brief Setter for initial left gap point range
            * \param lrange initial left gap point range
            */
            void setLRange(const float & lrange) { leftRange_ = lrange; }

            /**
            * \brief Getter for initial right gap point range
            * \param initial right gap point range
            */
            float RRange() const { return rightRange_; }

            /**
            * \brief Setter for initial right gap point range
            * \param rrange initial right gap point range
            */            
            void setRRange(const float & rrange) { rightRange_ = rrange; }

            // /**
            // * \brief Getter for terminal left gap point index
            // * \return terminal left gap point index
            // */
            // int termLIdx() const { return termLeftIdx_; }

            // /**
            // * \brief Setter for terminal left gap point index
            // * \param termLeftIdx terminal left gap point index
            // */            
            // void setTermLIdx(const int & termLeftIdx) { termLeftIdx_ = termLeftIdx; }

            // /**
            // * \brief Getter for terminal right gap point index
            // * \return terminal right gap point index
            // */
            // int termRIdx() const { return termRightIdx_; }
            
            // /**
            // * \brief Setter for terminal right gap point index
            // * \param termRightIdx terminal right gap point index
            // */            
            // void setTermRIdx(const int & termRightIdx) { termRightIdx_ = termRightIdx; }

            // /**
            // * \brief Getter for terminal left gap point distance
            // * \return terminal left gap point distance
            // */
            // float termLRange() const { return termLeftRange_; }

            // /**
            // * \brief Setter for terminal left gap point distance
            // * \param termLRange terminal left gap point distance
            // */            
            // void setTermLRange(const float & termLRange) { termLeftRange_ = termLRange; }

            // /**
            // * \brief Getter for terminal right gap point distance
            // * \return terminal right gap point distance
            // */
            // float termRRange() const { return termRightRange_; }

            // /**
            // * \brief Setter for terminal right gap point distance
            // * \param termRRange terminal right gap point distance
            // */            
            // void setTermRRange(const float & termRRange) { termRightRange_ = termRRange; }

            /**
            * \brief Getter for initial manipulated left gap point index
            * \return initial manipulated left gap point index
            */
            int manipLeftIdx() const { return manip.leftIdx_; }

            /**
            * \brief Setter for initial manipulated left gap point index
            * \param manipLeftIdx manipulated left gap point index
            */            
            void setManipLeftIdx(const int & manipLeftIdx) { manip.leftIdx_ = manipLeftIdx; }

            /**
            * \brief Getter for initial manipulated right gap point index
            * \return initial manipulated right gap point index
            */
            int manipRightIdx() const { return manip.rightIdx_; }

            /**
            * \brief Setter for initial manipulated right gap point index
            * \param manipRightIdx initial manipulated right gap point index
            */            
            void setManipRightIdx(const int & manipRightIdx) { manip.rightIdx_ = manipRightIdx; }

            /**
            * \brief Getter for initial manipulated left gap point distance
            * \return manipulated left gap point distance
            */
            float manipLeftRange() const { return manip.leftRange_; }

            /**
            * \brief Setter for initial manipulated left gap point distance
            * \param manipLeftRange manipulated left gap point distance
            */
            void setManipLeftRange(const float & manipLeftRange) { manip.leftRange_ = manipLeftRange; }

            /**
            * \brief Getter for initial manipulated right gap point distance
            * \return manipulated right gap point distance
            */
            float manipRightRange() const { return manip.rightRange_; }

            /**
            * \brief Setter for initial manipulated right gap point distance
            * \param manipRightRange manipulated right gap point distance
            */            
            void setManipRightRange(const float & manipRightRange) { manip.rightRange_ = manipRightRange; }

            /**
            * \brief Getter for trailing manipulated left gap point index
            * \return trailing manipulated left gap point index
            */
            int manipTrailingLeftIdx() const { return manip.trailingLeftIdx_; }

            /**
            * \brief Setter for trailing manipulated left gap point index
            * \param manipLeftIdx manipulated left gap point index
            */            
            void setManipTrailingLeftIdx(const int & manipTrailingLeftIdx) { manip.trailingLeftIdx_ = manipTrailingLeftIdx; }

            /**
            * \brief Getter for trailing manipulated right gap point index
            * \return trailing manipulated right gap point index
            */
            int manipTrailingRightIdx() const { return manip.trailingRightIdx_; }

            /**
            * \brief Setter for trailing manipulated right gap point index
            * \param manipRightIdx trailing manipulated right gap point index
            */            
            void setManipTrailingRightIdx(const int & manipTrailingRightIdx) { manip.trailingRightIdx_ = manipTrailingRightIdx; }

            /**
            * \brief Getter for trailing manipulated left gap point distance
            * \return manipulated left gap point distance
            */
            float manipTrailingLeftRange() const { return manip.trailingLeftRange_; }

            /**
            * \brief Setter for trailing manipulated left gap point distance
            * \param manipLeftRange manipulated left gap point distance
            */
            void setManipTrailingLeftRange(const float & manipTrailingLeftRange) { manip.trailingLeftRange_ = manipTrailingLeftRange; }

            /**
            * \brief Getter for trailing manipulated right gap point distance
            * \return manipulated right gap point distance
            */
            float manipTrailingRightRange() const { return manip.trailingRightRange_; }

            /**
            * \brief Setter for trailing manipulated right gap point distance
            * \param manipRightRange manipulated right gap point distance
            */            
            void setManipTrailingRightRange(const float & manipTrailingRightRange) { manip.trailingRightRange_ = manipTrailingRightRange; }

            /**
            * \brief Getter for terminal manipulated left gap point index
            * \return terminal manipulated left gap point index
            */
            int manipTermLeftIdx() const { return manip.termLeftIdx_; }

            // /**
            // * \brief Setter for terminal manipulated left gap point index
            // * \param manipTermLeftIdx manipulated left gap point index
            // */            
            // void setManipTermLeftIdx(const int & manipTermLeftIdx) { manip.termLeftIdx_ = manipTermLeftIdx; }

            /**
            * \brief Getter for terminal manipulated right gap point index
            * \return terminal manipulated right gap point index
            */
            int manipTermRightIdx() const { return manip.termRightIdx_; }

            // /**
            // * \brief Setter for terminal manipulated right gap point index
            // * \param manipTermRightIdx terminal manipulated right gap point index
            // */            
            // void setManipTermRightIdx(const int & manipTermRightIdx) { manip.termRightIdx_ = manipTermRightIdx; }

            /**
            * \brief Getter for terminal manipulated left gap point distance
            * \return terminal manipulated left gap point distance
            */
            float manipTermLeftRange() const { return manip.termLeftRange_; }

            // /**
            // * \brief Setter for terminal manipulated left gap point distance
            // * \param manipTermLeftRange terminal manipulated left gap point distance
            // */            
            // void setManipTermLeftRange(const float & manipTermLeftRange) { manip.termLeftRange_ = manipTermLeftRange; }

            /**
            * \brief Getter for terminal manipulated right gap point distance
            * \return terminal manipulated right gap point distance
            */
            float manipTermRightRange() const { return manip.termRightRange_; }

            // /**
            // * \brief Setter for terminal manipulated right gap point distance
            // * \param manipTermRightRange terminal manipulated right gap point distance
            // */            
            // void setManipTermRightRange(const float & manipTermRightRange) { manip.termRightRange_ = manipTermRightRange; }

            /**
            * \brief Conclude gap construction by populating gap's initial left side information 
            * and remaining characteristics
            * \param leftIdx initial left gap point index
            * \param leftRange initial left gap point range
            */
            void addLeftInformation(const int & leftIdx, const float & leftRange) 
            {
                leftIdx_ = leftIdx;
                leftRange_ = leftRange;
                rightType_ = rightRange_ < leftRange_;

                // initializing convex polar gap coordinates to raw ones
                manip.leftIdx_ = leftIdx_;
                manip.rightIdx_ = rightIdx_;
                manip.leftRange_ = leftRange_;
                manip.rightRange_ = rightRange_;

                setRadial();
            }

            // /**
            // * \brief Set gap's terminal points after propagation
            // * \param termLeftIdx terminal left gap point index
            // * \param termLeftRange terminal left gap point distance
            // * \param termRightIdx terminal right gap point index
            // * \param termRightRange terminal right gap point distance
            // */
            // void setTerminalPoints(const float & termLeftIdx, 
            //                        const float & termLeftRange, 
            //                        const float & termRightIdx, 
            //                        const float & termRightRange) 
            // {    
            //     termLeftIdx_ = termLeftIdx;
            //     termLeftRange_ = termLeftRange;
            //     termRightIdx_ = termRightIdx;
            //     termRightRange_ = termRightRange;

            //     if (termLeftIdx_ == termRightIdx_) 
            //     {
            //         // ROS_INFO_STREAM_NAMED("Gap", "terminal indices are the same");
            //         termLeftIdx_ = (termLeftIdx_ + 1) % 512;
            //     }
            //     // ROS_INFO_STREAM_NAMED("Gap", "setting terminal points to, left: (" << termLeftIdx_ << ", " << termLeftRange_ << "), right: ("  << termRightIdx_ << ", " << termRightRange_ << ")");
            // }

            void setGapLifespan(const float & gapLifespan)
            {
                gapLifespan_ = gapLifespan;
            }

            /**
            * \brief Getter for initial left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(leftIdx_);
                x = (leftRange_) * cos(thetaLeft);
                y = (leftRange_) * sin(thetaLeft);
            }

            /**
            * \brief Getter for initial right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(rightIdx_);
                x = (rightRange_) * cos(thetaRight);
                y = (rightRange_) * sin(thetaRight);
            }

            Eigen::Vector2f getLPosition() const
            {
                float x = 0.0, y = 0.0;
                getLCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            Eigen::Vector2f getRPosition() const
            {
                float x = 0.0, y = 0.0;
                getRCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getManipulatedLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(manip.leftIdx_);
                // std::cout << "rightRange_: " << rightRange_ << ", rightIdx_: " << rightIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (manip.leftRange_) * cos(thetaLeft);
                y = (manip.leftRange_) * sin(thetaLeft);
            }

            /**
            * \brief Getter for initial manipulated right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getManipulatedRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(manip.rightIdx_);
                // std::cout << "leftRange_: " << leftRange_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (manip.rightRange_) * cos(thetaRight);
                y = (manip.rightRange_) * sin(thetaRight);
            }

            Eigen::Vector2f getManipulatedLPosition() const
            {
                float x = 0.0, y = 0.0;
                getManipulatedLCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            Eigen::Vector2f getManipulatedRPosition() const
            {
                float x = 0.0, y = 0.0;
                getManipulatedRCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getManipulatedTrailingLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(manip.trailingLeftIdx_);
                // std::cout << "rightRange_: " << rightRange_ << ", rightIdx_: " << rightIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (manip.trailingLeftRange_) * cos(thetaLeft);
                y = (manip.trailingLeftRange_) * sin(thetaLeft);
            }

            /**
            * \brief Getter for initial manipulated right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getManipulatedTrailingRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(manip.trailingRightIdx_);
                // std::cout << "leftRange_: " << leftRange_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (manip.trailingRightRange_) * cos(thetaRight);
                y = (manip.trailingRightRange_) * sin(thetaRight);
            }

            Eigen::Vector2f getManipulatedTrailingLPosition() const
            {
                float x = 0.0, y = 0.0;
                getManipulatedTrailingLCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            Eigen::Vector2f getManipulatedTrailingRPosition() const
            {
                float x = 0.0, y = 0.0;
                getManipulatedTrailingRCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            void setTerminalManipulatedLPosition(const Eigen::Vector2f & p_left_term_lead)
            {
                // get theta
                float theta = std::atan2(p_left_term_lead[1], p_left_term_lead[0]);

                // get r
                float r = p_left_term_lead.norm();

                // get idx
                int idx = theta2idx(theta);

                manip.termLeftIdx_ = idx;
                manip.termLeftRange_ = r;
            }

            void setTerminalManipulatedRPosition(const Eigen::Vector2f & p_right_term_lead)
            {
                // get theta
                float theta = std::atan2(p_right_term_lead[1], p_right_term_lead[0]);

                // get r
                float r = p_right_term_lead.norm();

                // get idx
                int idx = theta2idx(theta);

                manip.termRightIdx_ = idx;
                manip.termRightRange_ = r;
            }


            // /**
            // * \brief Pretty printer for gap's left and right points in Cartesian frame
            // * \param initial boolean for printing initial points
            // * \param simplified boolean for printing manipulated points
            // */
            // void printCartesianPoints(const bool & initial, 
            //                           const bool & simplified) 
            // {
            //     float xLeft = 0.0, yLeft = 0.0, xRight = 0.0, yRight = 0.0;
            //     float thetaLeft = 0.0, thetaRight = 0.0, leftRange = 0.0, rangeRight = 0.0;
            //     if (initial) 
            //     {
            //         if (simplified) 
            //         {
            //             thetaLeft = idx2theta(leftIdx_);
            //             thetaRight = idx2theta(rightIdx_);
            //             leftRange = leftRange_;
            //             rangeRight = rightRange_;
            //         } else 
            //         {
            //             thetaLeft = idx2theta(manip.leftIdx_);
            //             thetaRight = idx2theta(manip.rightIdx_);    
            //             leftRange = manip.leftRange_;
            //             rangeRight = manip.rightRange_;                                                         
            //         }
            //     } else 
            //     {
            //         if (simplified) 
            //         {
            //             thetaLeft = idx2theta(termLeftIdx_);
            //             thetaRight = idx2theta(termRightIdx_);     
            //             leftRange = termLeftRange_;
            //             rangeRight = termRightRange_;          
            //         } else 
            //         {
            //             thetaLeft = idx2theta(manip.termLeftIdx_);
            //             thetaRight = idx2theta(manip.termRightIdx_);    
            //             leftRange = manip.termLeftRange_;
            //             rangeRight = manip.termRightRange_;                       
            //         }
            //     }

            //     xLeft = leftRange * cos(thetaLeft);
            //     yLeft = leftRange * sin(thetaLeft);
            //     xRight = rangeRight * cos(thetaRight);
            //     yRight = rangeRight * sin(thetaRight);

            //     ROS_INFO_STREAM_NAMED("Gap", "xLeft, yLeft: (" << xLeft << ", " << yLeft << "), xRight,yRight: (" << xRight << ", " << yRight << ")");
            // }   

            /**
            * \brief Determine if gap is radial
            * \param initial boolean for evaluating initial or terminal gap
            *
            *   far pt _____
            *          \ A  `___          
            *           \       `___      
            *            \          `___  
            *             \           B ` near pt
            *              \            / 
            *               \          /     A - far side angle 
            *                \        /      B - near side angle
            *                 \      /       C - gap angle
            *                  \    /
            *                   \ C/
            *                    \/
            *                 gap origin
            */
            void setRadial()
            {
                // ROS_INFO_STREAM_NAMED("Gap", "setRadial:");
                int checkLeftIdx = leftIdx_;
                int checkRightIdx = rightIdx_;

                float checkLeftRange = leftRange_;
                float checkRightRange = rightRange_;

                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftIdx: " << checkLeftIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftRange: " << checkLeftRange);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightIdx: " << checkRightIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightRange: " << checkRightRange);

                float resoln = M_PI / half_num_scan;
                float gapAngle = (checkLeftIdx - checkRightIdx) * resoln;
                if (gapAngle < 0)
                    gapAngle += 2*M_PI;

                // ROS_INFO_STREAM_NAMED("Gap", "   gapAngle: " << gapAngle);
                float nearRange = rightType_ ? checkRightRange : checkLeftRange;
                // law of cosines
                float leftPtToRightPtDist = sqrt(pow(checkRightRange, 2) + pow(checkLeftRange, 2) - 2 * checkRightRange * checkLeftRange * cos(gapAngle));
                // law of sines
                float farSideAngle = asin(epsilonDivide(nearRange, leftPtToRightPtDist) * sin(gapAngle));
                
                // ROS_INFO_STREAM_NAMED("Gap", "nearRange: " << nearRange);
                // ROS_INFO_STREAM_NAMED("Gap", "leftPtToRightPtDist: " << leftPtToRightPtDist);
                // ROS_INFO_STREAM_NAMED("Gap", "small angle: " << farSideAngle);

                // ROS_INFO_STREAM_NAMED("Gap", "   farSideAngle: " << farSideAngle);
                // ROS_INFO_STREAM_NAMED("Gap", "   gapAngle: " << gapAngle);
                float nearSideAngle = (M_PI - farSideAngle - gapAngle);
                // ROS_INFO_STREAM_NAMED("Gap", "   nearSideAngle: " << nearSideAngle);

                radial_ = nearSideAngle > 0.75 * M_PI;
 
            }

            /**
            * \brief Getter for gap radial condition
            * \param initial boolean for evaluating initial or terminal gap
            * \return Gap radial condition
            */
            bool isRadial() const
            {
                return radial_;
            }

            /**
            * \brief Getter for gap "right type" (if right point is closer than left point) condition
            * \param initial boolean for evaluating initial or terminal gap
            * \return Gap "right type" condition
            */
            bool isRightType() const
            {
                return rightType_;

            }

            /**
            * \brief Getter for initial minimum safe distance for gap
            * \return initial minimum safe distance for gap
            */
            float getMinSafeDist() { return minSafeDist_; }

            /** 
            * \brief Calculates the euclidean distance between the left and right gap points using the law of cosines
            * \return distance between left and right gap points
            */
            float getGapEuclideanDist() const 
            {
                float resoln = M_PI / half_num_scan;
                float gapAngle = (leftIdx_ - rightIdx_) * resoln;
                if (gapAngle < 0)
                    gapAngle += 2*M_PI;

                return sqrt(pow(rightRange_, 2) + pow(leftRange_, 2) - 2 * rightRange_ * leftRange_ * cos(gapAngle));
            }
            
            /**
            * \brief Setter for gap goal point
            * \param initial boolean for setting initial gap point
            * \param goalPt gap goal point
            */
            void setGoal(const Eigen::Vector2f & goalPt)
            {
                goal.x_ = goalPt[0];
                goal.y_ = goalPt[1];
            }

            /**
            * \brief Setter for gap goal point
            * \param initial boolean for setting terminal gap point
            * \param goalPt gap goal point
            */
            void setTerminalGoal(const Eigen::Vector2f & goalPt)
            {
                ROS_INFO_STREAM("[setTerminalGoal()]");
                
                terminalGoal.x_ = goalPt[0];
                terminalGoal.y_ = goalPt[1];

                ROS_INFO_STREAM("   terminalGoal, x: " << terminalGoal.x_ << ", " << terminalGoal.y_);
            }

            float gapLifespan_ = 5.0; /**< Gap lifespan over prediction horizon */

            float minSafeDist_ = 0.0; /**< minimum safe distance for current gap */

            Eigen::Vector2f extendedGapOrigin_; /**< current extended gap origin point */
            Eigen::Vector2f termExtendedGapOrigin_; /**< terminal extended gap origin point */

            std::string frame_ = ""; /**< Frame ID for gap */
            
            bool radial_ = false; /**< Initial gap radial characteristic identifier */
            
            bool rightType_ = false; /**< Initial gap right type characteristic identifier */

            bool rgc_ = false; /**< flag for if gap has been converted into swept gap */
            /**
            * \brief Gap's initial goal
            */
            struct Goal 
            {
                float x_ = 0.0; /**< Gap initial goal x-value */
                float y_ = 0.0; /**< Gap initial goal y-value */

            } goal;

            /**
            * \brief Gap's terminal goal
            */
            struct TerminalGoal 
            {
                float x_ = 0.0; /**< Gap terminal goal x-value */
                float y_ = 0.0; /**< Gap terminal goal y-value */
            } terminalGoal;

            Estimator * leftGapPtModel_ = NULL; /**< Left gap point estimator */
            Estimator * rightGapPtModel_ = NULL; /**< Right gap point estimator */

            bool globalGoalWithin = false;

            float t_intercept_left = 0.0;
            float t_intercept_right = 0.0;
            float gamma_intercept_left = 0.0;
            float gamma_intercept_right = 0.0;

            float gamma_intercept_goal = 0.0;

            int end_condition = -1; // 0 - collision, 1 - shut, 2 - overlapped, 3 - timed out

        private:

            int leftIdx_ = 511; /**< Initial left gap point index */
            float leftRange_ = 5; /**< Initial left gap point range */

            int rightIdx_ = 0; /**< Initial right gap point index */
            float rightRange_ = 5; /**< Initial right gap point range */

            // int termLeftIdx_ = 511; /**< Terminal left gap point index */
            // float termLeftRange_ = 5; /**< Terminal left gap point range */

            // int termRightIdx_ = 0; /**< Terminal right gap point index */
            // float termRightRange_ = 5; /**< Terminal right gap point range */
            
            /**
            * \brief Parameters of manipulated form of gap
            */
            struct Manip 
            {
                // "leading" gap points that define gap itself
                int leftIdx_ = 511; /**< Initial manipulated left gap point index */
                float leftRange_ = 5; /**< Initial manipulated left gap point range */

                int rightIdx_ = 0; /**< Initial manipulated right gap point index */
                float rightRange_ = 5; /**< Initial manipulated right gap point range */

                int termLeftIdx_ = 511; /**< Terminal manipulated left gap point index */
                float termLeftRange_ = 5; /**< Terminal manipulated left gap point range */

                int termRightIdx_ = 0; /**< Terminal manipulated right gap point index */
                float termRightRange_ = 5; /**< Terminal manipulated right gap point range */
            
                // "trailing" gap points that define back points;

                int trailingLeftIdx_ = 511;  /**< Trailing manipulated left gap point index */
                float trailingLeftRange_ = 5; /**< Trailing manipulated left gap point index */

                int trailingRightIdx_ = 0; /**< Trailing manipulated right gap point index */
                int trailingRightRange_ = 5; /**< Trailing manipulated right gap point range */
            } manip;
    };
}