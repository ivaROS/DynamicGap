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
                // shallow copy
                frame_ = otherGap.frame_;

                // copy all the variables
                leftIdx_ = otherGap.leftIdx_;
                leftRange_ = otherGap.leftRange_;
                rightIdx_ = otherGap.rightIdx_;
                rightRange_ = otherGap.rightRange_;

                termLeftIdx_ = otherGap.termLeftIdx_;
                termLeftRange_ = otherGap.termLeftRange_;
                termRightIdx_ = otherGap.termRightIdx_;
                termRightRange_ = otherGap.termRightRange_;

                manip.leftIdx_ = otherGap.manip.leftIdx_;
                manip.leftRange_ = otherGap.manip.leftRange_;
                manip.rightIdx_ = otherGap.manip.rightIdx_;
                manip.rightRange_ = otherGap.manip.rightRange_;

                manip.termLeftIdx_ = otherGap.manip.termLeftIdx_;
                manip.termLeftRange_ = otherGap.manip.termLeftRange_;
                manip.termRightIdx_ = otherGap.manip.termRightIdx_;
                manip.termRightRange_ = otherGap.manip.termRightRange_;

                rightType_ = otherGap.rightType_;
                terminalRightType_ = otherGap.terminalRightType_;

                radial_ = otherGap.radial_;
                termRadial_ = otherGap.termRadial_;

                minSafeDist_ = otherGap.minSafeDist_;

                goal.x_ = otherGap.goal.x_;
                goal.y_ = otherGap.goal.y_;

                terminalGoal.x_ = otherGap.terminalGoal.x_;
                terminalGoal.y_ = otherGap.terminalGoal.y_;

                gapLifespan_ = otherGap.gapLifespan_;

                // deep copy for new models
                // Here, you can define what type of model you want to use

                // leftGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                // rightGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                leftGapPtModel_ = new PerfectEstimator();
                rightGapPtModel_ = new PerfectEstimator();

                // transfer models (need to deep copy the models, not just the pointers)
                leftGapPtModel_->transfer(*otherGap.leftGapPtModel_);
                rightGapPtModel_->transfer(*otherGap.rightGapPtModel_);
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
            * \brief Getter for initial left gap point distance
            * \return initial left gap point distance
            */
            float LRange() const { return leftRange_; }

            /**
            * \brief Setter for initial left gap point distance
            * \param ldist initial left gap point distance
            */
            void setLDist(const float & ldist) { leftRange_ = ldist; }

            /**
            * \brief Getter for initial right gap point distance
            * \param initial right gap point distance
            */
            float RDist() const { return rightRange_; }

            /**
            * \brief Setter for initial right gap point distance
            * \param rdist initial right gap point distance
            */            
            void setRDist(const float & rdist) { rightRange_ = rdist; }

            /**
            * \brief Getter for terminal left gap point index
            * \return terminal left gap point index
            */
            int termLIdx() const { return termLeftIdx_; }

            /**
            * \brief Setter for terminal left gap point index
            * \param termLeftIdx terminal left gap point index
            */            
            void setTermLIdx(const int & termLeftIdx) { termLeftIdx_ = termLeftIdx; }

            /**
            * \brief Getter for terminal right gap point index
            * \return terminal right gap point index
            */
            int termRIdx() const { return termRightIdx_; }
            
            /**
            * \brief Setter for terminal right gap point index
            * \param termRightIdx terminal right gap point index
            */            
            void setTermRIdx(const int & termRightIdx) { termRightIdx_ = termRightIdx; }

            /**
            * \brief Getter for terminal left gap point distance
            * \return terminal left gap point distance
            */
            float termLRange() const { return termLeftRange_; }

            /**
            * \brief Setter for terminal left gap point distance
            * \param termLRange terminal left gap point distance
            */            
            void setTermLRange(const float & termLRange) { termLeftRange_ = termLRange; }

            /**
            * \brief Getter for terminal right gap point distance
            * \return terminal right gap point distance
            */
            float termRRange() const { return termRightRange_; }

            /**
            * \brief Setter for terminal right gap point distance
            * \param termRRange terminal right gap point distance
            */            
            void setTermRRange(const float & termRRange) { termRightRange_ = termRRange; }

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
            * \brief Getter for terminal manipulated left gap point index
            * \return terminal manipulated left gap point index
            */
            int manipTermLeftIdx() const { return manip.termLeftIdx_; }

            /**
            * \brief Setter for terminal manipulated left gap point index
            * \param manipTermLeftIdx manipulated left gap point index
            */            
            void setManipTermLeftIdx(const int & manipTermLeftIdx) { manip.termLeftIdx_ = manipTermLeftIdx; }

            /**
            * \brief Getter for terminal manipulated right gap point index
            * \return terminal manipulated right gap point index
            */
            int manipTermRightIdx() const { return manip.termRightIdx_; }

            /**
            * \brief Setter for terminal manipulated right gap point index
            * \param manipTermRightIdx terminal manipulated right gap point index
            */            
            void setManipTermRightIdx(const int & manipTermRightIdx) { manip.termRightIdx_ = manipTermRightIdx; }

            /**
            * \brief Getter for terminal manipulated left gap point distance
            * \return terminal manipulated left gap point distance
            */
            float manipTermLeftRange() const { return manip.termLeftRange_; }

            /**
            * \brief Setter for terminal manipulated left gap point distance
            * \param manipTermLeftRange terminal manipulated left gap point distance
            */            
            void setManipTermLeftRange(const float & manipTermLeftRange) { manip.termLeftRange_ = manipTermLeftRange; }

            /**
            * \brief Getter for terminal manipulated right gap point distance
            * \return terminal manipulated right gap point distance
            */
            float manipTermRightRange() const { return manip.termRightRange_; }

            /**
            * \brief Setter for terminal manipulated right gap point distance
            * \param manipTermRightRange terminal manipulated right gap point distance
            */            
            void setManipTermRightRange(const float & manipTermRightRange) { manip.termRightRange_ = manipTermRightRange; }

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

            /**
            * \brief Conclude gap construction by populating gap's terminal left side information 
            * and remaining characteristics
            */
            void addTerminalRightInformation()
            {
                terminalRightType_ = termRightRange_ < termLeftRange_;

                manip.termLeftIdx_ = termLeftIdx_;
                manip.termRightIdx_ = termRightIdx_;
                manip.termLeftRange_ = termLeftRange_;
                manip.termRightRange_ = termRightRange_;

                setRadial(false);
            }

            /**
            * \brief Set gap's terminal points after propagation
            * \param termLeftIdx terminal left gap point index
            * \param termLeftRange terminal left gap point distance
            * \param termRightIdx terminal right gap point index
            * \param termRightRange terminal right gap point distance
            */
            void setTerminalPoints(const float & termLeftIdx, 
                                   const float & termLeftRange, 
                                   const float & termRightIdx, 
                                   const float & termRightRange) 
            {    
                termLeftIdx_ = termLeftIdx;
                termLeftRange_ = termLeftRange;
                termRightIdx_ = termRightIdx;
                termRightRange_ = termRightRange;

                if (termLeftIdx_ == termRightIdx_) 
                {
                    // ROS_INFO_STREAM_NAMED("Gap", "terminal indices are the same");
                    termLeftIdx_ = (termLeftIdx_ + 1) % 512;
                }
                // ROS_INFO_STREAM_NAMED("Gap", "setting terminal points to, left: (" << termLeftIdx_ << ", " << termLeftRange_ << "), right: ("  << termRightIdx_ << ", " << termRightRange_ << ")");
            
                terminalRightType_ = termRightRange_ < termLeftRange_;

                setRadial(false);
            }

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

            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getSimplifiedLCartesian(float &x, float &y) const
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
            void getSimplifiedRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(manip.rightIdx_);
                // std::cout << "leftRange_: " << leftRange_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (manip.rightRange_) * cos(thetaRight);
                y = (manip.rightRange_) * sin(thetaRight);
            }

            /**
            * \brief Getter for terminal manipulated left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getSimplifiedTerminalLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(manip.termLeftIdx_);
                // std::cout << "rightRange_: " << rightRange_ << ", rightIdx_: " << rightIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (manip.termLeftRange_) * cos(thetaLeft);
                y = (manip.termLeftRange_) * sin(thetaLeft);
            }

            /**
            * \brief Getter for terminal manipulated right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getSimplifiedTerminalRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(manip.termRightIdx_);
                // std::cout << "leftRange_: " << leftRange_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (manip.termRightRange_) * cos(thetaRight);
                y = (manip.termRightRange_) * sin(thetaRight);
            }

            /**
            * \brief Pretty printer for gap's left and right points in Cartesian frame
            * \param initial boolean for printing initial points
            * \param simplified boolean for printing manipulated points
            */
            void printCartesianPoints(const bool & initial, 
                                      const bool & simplified) 
            {
                float xLeft = 0.0, yLeft = 0.0, xRight = 0.0, yRight = 0.0;
                float thetaLeft = 0.0, thetaRight = 0.0, leftRange = 0.0, rangeRight = 0.0;
                if (initial) 
                {
                    if (simplified) 
                    {
                        thetaLeft = idx2theta(leftIdx_);
                        thetaRight = idx2theta(rightIdx_);
                        leftRange = leftRange_;
                        rangeRight = rightRange_;
                    } else 
                    {
                        thetaLeft = idx2theta(manip.leftIdx_);
                        thetaRight = idx2theta(manip.rightIdx_);    
                        leftRange = manip.leftRange_;
                        rangeRight = manip.rightRange_;                                                         
                    }
                } else 
                {
                    if (simplified) 
                    {
                        thetaLeft = idx2theta(termLeftIdx_);
                        thetaRight = idx2theta(termRightIdx_);     
                        leftRange = termLeftRange_;
                        rangeRight = termRightRange_;          
                    } else 
                    {
                        thetaLeft = idx2theta(manip.termLeftIdx_);
                        thetaRight = idx2theta(manip.termRightIdx_);    
                        leftRange = manip.termLeftRange_;
                        rangeRight = manip.termRightRange_;                       
                    }
                }

                xLeft = leftRange * cos(thetaLeft);
                yLeft = leftRange * sin(thetaLeft);
                xRight = rangeRight * cos(thetaRight);
                yRight = rangeRight * sin(thetaRight);

                ROS_INFO_STREAM_NAMED("Gap", "xLeft, yLeft: (" << xLeft << ", " << yLeft << "), xRight,yRight: (" << xRight << ", " << yRight << ")");
            }   

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
            void setRadial(const bool & initial = true)
            {
                // ROS_INFO_STREAM_NAMED("Gap", "setRadial:");
                int checkLeftIdx = initial ? leftIdx_ : termLeftIdx_;
                int checkRightIdx = initial ? rightIdx_ : termRightIdx_;

                float checkLeftRange = initial ? leftRange_ : termLeftRange_;
                float checkRightRange = initial ? rightRange_ : termRightRange_;

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

                if (initial)
                    radial_ = nearSideAngle > 0.75 * M_PI;
                else
                    termRadial_ = nearSideAngle > 0.75 * M_PI;     
            }

            /**
            * \brief Getter for gap radial condition
            * \param initial boolean for evaluating initial or terminal gap
            * \return Gap radial condition
            */
            bool isRadial(const bool & initial = true) const
            {
                return (initial ? radial_ : termRadial_);
            }

            /**
            * \brief Getter for gap "right type" (if right point is closer than left point) condition
            * \param initial boolean for evaluating initial or terminal gap
            * \return Gap "right type" condition
            */
            bool isRightType(const bool & initial = true) const
            {
                if (initial)
                    return rightType_;
                else
                    return terminalRightType_;
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

            float gapLifespan_ = 5.0; /**< Gap lifespan over prediction horizon */

            float minSafeDist_ = 0.0; /**< minimum safe distance for current gap */

            Eigen::Vector2f extendedGapOrigin_; /**< current extended gap origin point */
            Eigen::Vector2f termExtendedGapOrigin_; /**< terminal extended gap origin point */
            // float half_scan = 256;

            std::string frame_ = ""; /**< Frame ID for gap */
            
            bool radial_ = false; /**< Initial gap radial characteristic identifier */
            bool termRadial_ = false; /**< Terminal gap radial characteristic identifier */
            
            bool rightType_ = false; /**< Initial gap right type characteristic identifier */
            bool terminalRightType_ = false; /**< Terminal gap right type characteristic identifier */

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

        private:

            int leftIdx_ = 511; /**< Initial left gap point index */
            float leftRange_ = 5; /**< Initial left gap point distance */

            int rightIdx_ = 0; /**< Initial right gap point index */
            float rightRange_ = 5; /**< Initial right gap point distance */

            int termLeftIdx_ = 511; /**< Terminal left gap point index */
            float termLeftRange_ = 5; /**< Terminal left gap point distance */

            int termRightIdx_ = 0; /**< Terminal right gap point index */
            float termRightRange_ = 5; /**< Terminal right gap point distance */
            
            /**
            * \brief Parameters of manipulated form of gap
            */
            struct Manip 
            {
                int leftIdx_ = 511; /**< Initial manipulated left gap point index */
                float leftRange_ = 5; /**< Initial manipulated left gap point distance */

                int rightIdx_ = 0; /**< Initial manipulated right gap point index */
                float rightRange_ = 5; /**< Initial manipulated right gap point distance */

                int termLeftIdx_ = 511; /**< Terminal manipulated left gap point index */
                float termLeftRange_ = 5; /**< Terminal manipulated left gap point distance */

                int termRightIdx_ = 0; /**< Terminal manipulated right gap point index */
                float termRightRange_ = 5; /**< Terminal manipulated right gap point distance */
            } manip;
    };
}