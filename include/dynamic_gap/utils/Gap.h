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
            Gap(const std::string & frame, 
                const int & rightIdx, 
                const float & rangeRight, 
                const bool & radial, 
                const float & minSafeDist_) : 
                frame_(frame), 
                rightIdx_(rightIdx), 
                rightRange_(rangeRight), 
                radial_(radial), 
                minSafeDist_(minSafeDist_)
            {
                extendedGapOrigin_ << 0.0, 0.0;
                termExtendedGapOrigin_ << 0.0, 0.0;

                // Here, you can define what type of model you want to use
                leftGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                rightGapPtModel_ = new RotatingFrameCartesianKalmanFilter();
                // leftGapPtModel_ = new PerfectEstimator();
                // rightGapPtModel_ = new PerfectEstimator();

                if (frame.empty())
                {
                    ROS_WARN_STREAM("Gap frame is empty");
                }
            };

            Gap(const Gap & otherGap)
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

                terminalGoal = otherGap.terminalGoal;

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
                gamma_intercept_goal = otherGap.gamma_intercept_goal;

                end_condition = otherGap.end_condition;

                // copy all the variables
                leftIdx_ = otherGap.leftIdx_;
                leftRange_ = otherGap.leftRange_;
                rightIdx_ = otherGap.rightIdx_;
                rightRange_ = otherGap.rightRange_;

                manip = otherGap.manip;

                rgc_ = otherGap.rgc_;
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

            void setManipPoints(const int & newLeftIdx, const int & newRightIdx, 
                                const float & newLeftRange, const float & newRightRange)
            {
                manip.leftIdx_ = newLeftIdx;
                manip.rightIdx_ = newRightIdx;
                manip.leftRange_ = newLeftRange;
                manip.rightRange_ = newRightRange;
            }        

            bool checkManipPoints()
            {
                if (std::isnan(manip.leftRange_) || std::isnan(manip.rightRange_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Manipulated gap points have NaN ranges");
                    return false;
                }

                if (std::isinf(manip.leftRange_) || std::isinf(manip.rightRange_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Manipulated gap points have Inf ranges");
                    return false;
                }

                if (std::isnan(manip.leftIdx_) || std::isnan(manip.rightIdx_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Manipulated gap points have NaN indices");
                    return false;
                }

                if (std::isinf(manip.leftIdx_) || std::isinf(manip.rightIdx_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Manipulated gap points have Inf indices");
                    return false;
                }

                if (manip.leftIdx_ < 0 || manip.rightIdx_ < 0)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Manipulated gap points have negative indices");
                    return false;
                }

                if (manip.leftRange_ < 0 || manip.rightRange_ < 0)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Manipulated gap points have negative ranges");
                    return false;
                }

                if (manip.leftIdx_ == manip.rightIdx_)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Manipulated gap points have same indices");
                    return false;
                }

                if (manip.leftIdx_ >= 2*half_num_scan || manip.rightIdx_ >= 2*half_num_scan)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Manipulated gap points have indices out of bounds");
                    return false;
                }

                return true;
            }

            /**
            * \brief Getter for initial manipulated left gap point index
            * \return initial manipulated left gap point index
            */
            int manipLeftIdx() const { return manip.leftIdx_; }

            /**
            * \brief Getter for initial manipulated right gap point index
            * \return initial manipulated right gap point index
            */
            int manipRightIdx() const { return manip.rightIdx_; }

            /**
            * \brief Getter for initial manipulated left gap point distance
            * \return manipulated left gap point distance
            */
            float manipLeftRange() const { return manip.leftRange_; }

            /**
            * \brief Getter for initial manipulated right gap point distance
            * \return manipulated right gap point distance
            */
            float manipRightRange() const { return manip.rightRange_; }

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
            * \brief Getter for right gap point in Cartesian frame
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
            * \brief Getter for left gap point
             */
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

            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \return Initial manipulated left gap point in Cartesian frame
            */
            Eigen::Vector2f getManipulatedLPosition() const
            {
                float x = 0.0, y = 0.0;
                getManipulatedLCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            /**
            * \brief Getter for initial manipulated right gap point in Cartesian frame
            * \return Initial manipulated right gap point in Cartesian frame
            */
            Eigen::Vector2f getManipulatedRPosition() const
            {
                float x = 0.0, y = 0.0;
                getManipulatedRCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            /**
            * \brief Determine if gap is radial
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

                radial_ = nearSideAngle > (0.75 * M_PI);
            }

            /**
            * \brief Getter for gap radial condition
            * \return Gap radial condition
            */
            bool isRadial() const
            {
                return radial_;
            }

            /**
            * \brief Getter for gap "right type" (if right point is closer than left point) condition
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
            * \param goalPt gap goal point
            */
            void setGoal(const Eigen::Vector2f & goalPt)
            {
                goal.x_ = goalPt[0];
                goal.y_ = goalPt[1];
            }

            /**
            * \brief Setter for gap goal veloicty
            * \param goalVel gap goal velocity
            */
            void setGoalVel(const Eigen::Vector2f & goalVel)
            {
                goal.vx_ = goalVel[0];
                goal.vy_ = goalVel[1];
            }

            /**
            * \brief Setter for terminal gap goal point
            * \param goalPt terminal gap goal point
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

                float vx_ = 0.0; /**< Gap initial goal x-vel */
                float vy_ = 0.0; /**< Gap initial goal y-vel */

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

            bool globalGoalWithin = false; /**< Flag for if global goal lies within gap's angular span */

            float t_intercept_left = 0.0; /**< Intercept time for left gap point */
            float gamma_intercept_left = 0.0; /**< Intercept angle for left gap point */

            float t_intercept_right = 0.0; /**< Intercept time for right gap point */
            float gamma_intercept_right = 0.0; /**< Intercept angle for right gap point */

            float t_intercept_goal = 0.0;  /**< Intercept time for gap goal point */
            float gamma_intercept_goal = 0.0; /**< Intercept angle for gap goal point */

            int end_condition = -1; // 0 - collision, 1 - shut, 2 - overlapped, 3 - timed out

        private:

            int leftIdx_ = 511; /**< Initial left gap point index */
            float leftRange_ = 5; /**< Initial left gap point range */

            int rightIdx_ = 0; /**< Initial right gap point index */
            float rightRange_ = 5; /**< Initial right gap point range */
            
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
            } manip;
    };
}