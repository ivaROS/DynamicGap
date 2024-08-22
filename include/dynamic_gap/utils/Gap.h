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
                frame_(frame), rightIdx_(rightIdx), rightDist_(rangeRight), radial_(radial), minSafeDist_(minSafeDist_)
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
            float LDist() const { return leftDist_; }

            /**
            * \brief Setter for initial left gap point distance
            * \param ldist initial left gap point distance
            */
            void setLDist(const float & ldist) { leftDist_ = ldist; }

            /**
            * \brief Getter for initial right gap point distance
            * \param initial right gap point distance
            */
            float RDist() const { return rightDist_; }

            /**
            * \brief Setter for initial right gap point distance
            * \param rdist initial right gap point distance
            */            
            void setRDist(const float & rdist) { rightDist_ = rdist; }

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
            float termLDist() const { return termLeftDist_; }

            /**
            * \brief Setter for terminal left gap point distance
            * \param termLDist terminal left gap point distance
            */            
            void setTermLDist(const float & termLDist) { termLeftDist_ = termLDist; }

            /**
            * \brief Getter for terminal right gap point distance
            * \return terminal right gap point distance
            */
            float termRDist() const { return termRightDist_; }

            /**
            * \brief Setter for terminal right gap point distance
            * \param termRDist terminal right gap point distance
            */            
            void setTermRDist(const float & termRDist) { termRightDist_ = termRDist; }

            /**
            * \brief Getter for initial convexified left gap point index
            * \return initial convexified left gap point index
            */
            int cvxLeftIdx() const { return convex.leftIdx_; }

            /**
            * \brief Setter for initial convexified left gap point index
            * \param cvxLeftIdx convexified left gap point index
            */            
            void setCvxLeftIdx(const int & cvxLeftIdx) { convex.leftIdx_ = cvxLeftIdx; }

            /**
            * \brief Getter for initial convexified right gap point index
            * \return initial convexified right gap point index
            */
            int cvxRightIdx() const { return convex.rightIdx_; }

            /**
            * \brief Setter for initial convexified right gap point index
            * \param cvxRightIdx initial convexified right gap point index
            */            
            void setCvxRightIdx(const int & cvxRightIdx) { convex.rightIdx_ = cvxRightIdx; }

            /**
            * \brief Getter for initial convexified left gap point distance
            * \return convexified left gap point distance
            */
            float cvxLeftDist() const { return convex.leftDist_; }

                /**
            * \brief Setter for initial convexified left gap point distance
            * \param cvxLeftDist convexified left gap point distance
            */
            void setCvxLeftDist(const float & cvxLeftDist) { convex.leftDist_ = cvxLeftDist; }

            /**
            * \brief Getter for initial convexified right gap point distance
            * \return convexified right gap point distance
            */
            float cvxRightDist() const { return convex.rightDist_; }

            /**
            * \brief Setter for initial convexified right gap point distance
            * \param cvxRightDist convexified right gap point distance
            */            
            void setCvxRightDist(const float & cvxRightDist) { convex.rightDist_ = cvxRightDist; }

            /**
            * \brief Getter for terminal convexified left gap point index
            * \return terminal convexified left gap point index
            */
            int cvxTermLeftIdx() const { return convex.termLeftIdx_; }

            /**
            * \brief Setter for terminal convexified left gap point index
            * \param cvxTermLeftIdx convexified left gap point index
            */            
            void setcvxTermLeftIdx(const int & cvxTermLeftIdx) { convex.termLeftIdx_ = cvxTermLeftIdx; }

            /**
            * \brief Getter for terminal convexified right gap point index
            * \return terminal convexified right gap point index
            */
            int cvxTermRightIdx() const { return convex.termRightIdx_; }

            /**
            * \brief Setter for terminal convexified right gap point index
            * \param cvxTermRightIdx terminal convexified right gap point index
            */            
            void setcvxTermRightIdx(const int & cvxTermRightIdx) { convex.termRightIdx_ = cvxTermRightIdx; }

            /**
            * \brief Getter for terminal convexified left gap point distance
            * \return terminal convexified left gap point distance
            */
            float cvxTermLeftDist() const { return convex.termLeftDist_; }

            /**
            * \brief Setter for terminal convexified left gap point distance
            * \param cvxTermLeftDist terminal convexified left gap point distance
            */            
            void setcvxTermLeftDist(const float & cvxTermLeftDist) { convex.termLeftDist_ = cvxTermLeftDist; }

            /**
            * \brief Getter for terminal convexified right gap point distance
            * \return terminal convexified right gap point distance
            */
            float cvxTermRightDist() const { return convex.termRightDist_; }

            /**
            * \brief Setter for terminal convexified right gap point distance
            * \param cvxTermRightDist terminal convexified right gap point distance
            */            
            void setcvxTermRightDist(const float & cvxTermRightDist) { convex.termRightDist_ = cvxTermRightDist; }

            /**
            * \brief Conclude gap construction by populating gap's initial left side information 
            * and remaining characteristics
            * \param leftIdx initial left gap point index
            * \param leftRange initial left gap point range
            */
            void addLeftInformation(const int & leftIdx, const float & leftRange) 
            {
                leftIdx_ = leftIdx;
                leftDist_ = leftRange;
                rightType_ = rightDist_ < leftDist_;

                // initializing convex polar gap coordinates to raw ones
                convex.leftIdx_ = leftIdx_;
                convex.rightIdx_ = rightIdx_;
                convex.leftDist_ = leftDist_;
                convex.rightDist_ = rightDist_;

                setRadial();
            }

            /**
            * \brief Conclude gap construction by populating gap's terminal left side information 
            * and remaining characteristics
            */
            void addTerminalRightInformation()
            {
                terminalRightType_ = termRightDist_ < termLeftDist_;

                convex.termLeftIdx_ = termLeftIdx_;
                convex.termRightIdx_ = termRightIdx_;
                convex.termLeftDist_ = termLeftDist_;
                convex.termRightDist_ = termRightDist_;

                setRadial(false);
            }

            /**
            * \brief Set gap's terminal points after propagation
            * \param termLeftIdx terminal left gap point index
            * \param termLeftDist terminal left gap point distance
            * \param termRightIdx terminal right gap point index
            * \param termRightDist terminal right gap point distance
            */
            void setTerminalPoints(const float & termLeftIdx, 
                                   const float & termLeftDist, 
                                   const float & termRightIdx, 
                                   const float & termRightDist) 
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
                x = (leftDist_) * cos(thetaLeft);
                y = (leftDist_) * sin(thetaLeft);
            }

            /**
            * \brief Getter for initial right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(rightIdx_);
                x = (rightDist_) * cos(thetaRight);
                y = (rightDist_) * sin(thetaRight);
            }

            /**
            * \brief Getter for initial convexified left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getSimplifiedLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(convex.leftIdx_);
                // std::cout << "rightDist_: " << rightDist_ << ", rightIdx_: " << rightIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.leftDist_) * cos(thetaLeft);
                y = (convex.leftDist_) * sin(thetaLeft);
            }

            /**
            * \brief Getter for initial convexified right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getSimplifiedRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(convex.rightIdx_);
                // std::cout << "leftDist_: " << leftDist_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.rightDist_) * cos(thetaRight);
                y = (convex.rightDist_) * sin(thetaRight);
            }

            /**
            * \brief Getter for terminal convexified left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getSimplifiedTerminalLCartesian(float &x, float &y) const
            {
                float thetaLeft = idx2theta(convex.termLeftIdx_);
                // std::cout << "rightDist_: " << rightDist_ << ", rightIdx_: " << rightIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.termLeftDist_) * cos(thetaLeft);
                y = (convex.termLeftDist_) * sin(thetaLeft);
            }

            /**
            * \brief Getter for terminal convexified right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getSimplifiedTerminalRCartesian(float &x, float &y) const
            {
                float thetaRight = idx2theta(convex.termRightIdx_);
                // std::cout << "leftDist_: " << leftDist_ << ", leftIdx_: " << leftIdx_ << ", half_scan: " << half_scan << std::endl;
                x = (convex.termRightDist_) * cos(thetaRight);
                y = (convex.termRightDist_) * sin(thetaRight);
            }

            /**
            * \brief Pretty printer for gap's left and right points in Cartesian frame
            * \param initial boolean for printing initial points
            * \param simplified boolean for printing convexified points
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
                        leftRange = leftDist_;
                        rangeRight = rightDist_;
                    } else 
                    {
                        thetaLeft = idx2theta(convex.leftIdx_);
                        thetaRight = idx2theta(convex.rightIdx_);    
                        leftRange = convex.leftDist_;
                        rangeRight = convex.rightDist_;                                                         
                    }
                } else 
                {
                    if (simplified) 
                    {
                        thetaLeft = idx2theta(termLeftIdx_);
                        thetaRight = idx2theta(termRightIdx_);     
                        leftRange = termLeftDist_;
                        rangeRight = termRightDist_;          
                    } else 
                    {
                        thetaLeft = idx2theta(convex.termLeftIdx_);
                        thetaRight = idx2theta(convex.termRightIdx_);    
                        leftRange = convex.termLeftDist_;
                        rangeRight = convex.termRightDist_;                       
                    }
                }

                xLeft = leftRange * cos(thetaLeft);
                yLeft = leftRange * sin(thetaLeft);
                xRight = rangeRight * cos(thetaRight);
                yRight = rangeRight * sin(thetaRight);

                ROS_INFO_STREAM_NAMED("Gap", "xLeft, yLeft: (" << xLeft << ", " << yLeft << "), xRight,yRight: (" << xRight << ", " << yRight << ")");
            }   

            /*
            void initManipIndices() 
            {
                convex.leftIdx_ = leftIdx_;
                convex.leftDist_ = leftDist_;
                convex.rightIdx_ = rightIdx_;
                convex.rightDist_ = rightDist_;
             
                convex.termRightIdx_ = termRightIdx_;
                convex.termRightDist_ = termRightDist_;
                convex.termLeftIdx_ = termLeftIdx_;
                convex.termLeftDist_ = termLeftDist_;
            }
            */

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

                float checkLeftDist = initial ? leftDist_ : termLeftDist_;
                float checkRightDist = initial ? rightDist_ : termRightDist_;

                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftIdx: " << checkLeftIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftDist: " << checkLeftDist);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightIdx: " << checkRightIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightDist: " << checkRightDist);

                float resoln = M_PI / half_num_scan;
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
            * \brief Setter for initial minimum safe distance for gap
            * \param dist initial minimum safe distance for gap
            */
            void setTerminalMinSafeDist(const float & dist) { terminalMinSafeDist_ = dist; }

            /**
            * \brief Getter for terminal minimum safe distance for gap
            * \return terminal minimum safe distance for gap
            */            
            float getTerminalMinSafeDist() { return terminalMinSafeDist_; }

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

                return sqrt(pow(rightDist_, 2) + pow(leftDist_, 2) - 2 * rightDist_ * leftDist_ * cos(gapAngle));
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
            float terminalMinSafeDist_ = 0.0; /**< minimum safe distance for terminal gap */

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
            float leftDist_ = 5; /**< Initial left gap point distance */

            int rightIdx_ = 0; /**< Initial right gap point index */
            float rightDist_ = 5; /**< Initial right gap point distance */

            int termLeftIdx_ = 511; /**< Terminal left gap point index */
            float termLeftDist_ = 5; /**< Terminal left gap point distance */

            int termRightIdx_ = 0; /**< Terminal right gap point index */
            float termRightDist_ = 5; /**< Terminal right gap point distance */
            
            /**
            * \brief Parameters of convexified form of gap
            */
            struct Convex 
            {
                int leftIdx_ = 511; /**< Initial convexified left gap point index */
                float leftDist_ = 5; /**< Initial convexified left gap point distance */

                int rightIdx_ = 0; /**< Initial convexified right gap point index */
                float rightDist_ = 5; /**< Initial convexified right gap point distance */

                int termLeftIdx_ = 511; /**< Terminal convexified left gap point index */
                float termLeftDist_ = 5; /**< Terminal convexified left gap point distance */

                int termRightIdx_ = 0; /**< Terminal convexified right gap point index */
                float termRightDist_ = 5; /**< Terminal convexified right gap point distance */
            } convex;
    };
}