#pragma once

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/utils/GapGoal.h>
#include <dynamic_gap/utils/GapPoint.h>
#include <dynamic_gap/utils/PropagatedGapPoint.h>
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
            // , const float & minSafeDist_
            Gap(const std::string & frame, 
                const int & rightIdx, 
                const float & rangeRight, 
                const bool & radial)
            {
                frame_ = frame;
                radial_ = radial;

                leftGapPt_ = new GapPoint(-1, -1.0); // temp values
                rightGapPt_ = new GapPoint(rightIdx, rangeRight);

                gapStart_ = 0.0;

                goal_ = new GapGoal();

                if (frame.empty())
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap frame is empty");
                }
            };

            // This constructor should only be run
            //      After:
            //          Gap Detection (points are set)
            //      Before:
            //          Gap Propagation
            //          Gap Feasibility Analysis
            //          Gap Manipulation

            // ALSO Running during gap propagation now!!
            Gap(const Gap & otherGap)
            {
                // ROS_INFO_STREAM_NAMED("Gap", "in copy constructor");
                // gapLifespan_ = otherGap.gapLifespan_;

                frame_ = otherGap.frame_;
                radial_ = otherGap.radial_;
                rightType_ = otherGap.rightType_;

                gapStart_ = otherGap.gapStart_;
                gapLifespan_ = otherGap.gapLifespan_;

                available_ = otherGap.available_;

                // deep copy for new points
                leftGapPt_ = new GapPoint(*otherGap.leftGapPt_);
                rightGapPt_ = new GapPoint(*otherGap.rightGapPt_);

                goal_ = new GapGoal();

                // globalGoalWithin = otherGap.globalGoalWithin;

                // gamma_intercept_goal = otherGap.gamma_intercept_goal;

                // goal_ = otherGap.goal_;

                // end_condition = otherGap.end_condition;

                // rgc_ = otherGap.rgc_;
            }

            Gap(const std::string & frame,
                const PropagatedGapPoint & leftPropagatedGapPoint,
                const PropagatedGapPoint & rightPropagatedGapPoint,
                const float & gapStart,
                const bool & available)
            {
                if (leftPropagatedGapPoint.isLeft() && rightPropagatedGapPoint.isRight())
                {
                    frame_ = frame;
                    available_ = available;

                    leftGapPt_ = new GapPoint(leftPropagatedGapPoint);
                    rightGapPt_ = new GapPoint(rightPropagatedGapPoint);

                    gapStart_ = gapStart;

                    // initializing convex polar gap coordinates to raw ones
                    // leftGapPt_->initManipPoint();
                    // rightGapPt_->initManipPoint();

                    // radial_ = radial;
                    // rightType_ = rightType;
                    setRadial();
                    setRightType();

                    goal_ = new GapGoal();
                }
                else
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap points are not left and right");
                }


            }

            ~Gap() 
            {
                delete goal_;
                delete leftGapPt_;
                delete rightGapPt_;
            };
            
            /**
            * \brief Getter for initial left gap point index
            * \return initial left gap point index
            */
            int LIdx() const { return leftGapPt_->getOrigIdx(); }

            /**
            * \brief Setter for initial left gap point index
            * \param lidx initial left gap point index
            */
            void setLIdx(const int & lidx) { leftGapPt_->setOrigIdx(lidx); }

            /**
            * \brief Getter for initial right gap point index
            * \return initial right gap point index
            */
            int RIdx() const { return rightGapPt_->getOrigIdx(); }

            /**
            * \brief Setter for initial right gap point index
            * \param ridx initial right gap point index
            */            
            void setRIdx(const int & ridx) { rightGapPt_->setOrigIdx(ridx); }

            /**
            * \brief Getter for initial left gap point range
            * \return initial left gap point range
            */
            float LRange() const { return leftGapPt_->getOrigRange(); }

            /**
            * \brief Setter for initial left gap point range
            * \param lrange initial left gap point range
            */
            void setLRange(const float & lrange) { leftGapPt_->setOrigRange(lrange); }

            /**
            * \brief Getter for initial right gap point range
            * \param initial right gap point range
            */
            float RRange() const { return rightGapPt_->getOrigRange(); }

            /**
            * \brief Setter for initial right gap point range
            * \param rrange initial right gap point range
            */            
            void setRRange(const float & rrange) { rightGapPt_->setOrigRange(rrange); }

            void setManipPoints(const int & newLeftIdx, const int & newRightIdx, 
                                const float & newLeftRange, const float & newRightRange)
            {
                leftGapPt_->setManipIdx(newLeftIdx);
                leftGapPt_->setManipRange(newLeftRange);
                rightGapPt_->setManipIdx(newRightIdx);
                rightGapPt_->setManipRange(newRightRange);
            }        

            bool checkPoints()
            {
                return (leftGapPt_->checkOrigPoint() && rightGapPt_->checkOrigPoint() && 
                        leftGapPt_->checkManipPoint() && rightGapPt_->checkManipPoint());
            }

            /**
            * \brief Getter for initial manipulated left gap point index
            * \return initial manipulated left gap point index
            */
            int manipLeftIdx() const { return leftGapPt_->getManipIdx(); }

            /**
            * \brief Getter for initial manipulated right gap point index
            * \return initial manipulated right gap point index
            */
            int manipRightIdx() const { return rightGapPt_->getManipIdx(); }

            /**
            * \brief Getter for initial manipulated left gap point distance
            * \return manipulated left gap point distance
            */
            float manipLeftRange() const { return leftGapPt_->getManipRange(); }

            /**
            * \brief Getter for initial manipulated right gap point distance
            * \return manipulated right gap point distance
            */
            float manipRightRange() const { return rightGapPt_->getManipRange(); }

            /**
            * \brief Conclude gap construction by populating gap's initial left side information 
            * and remaining characteristics
            * \param leftIdx initial left gap point index
            * \param leftRange initial left gap point range
            */
            void addLeftInformation(const int & leftIdx, const float & leftRange) 
            {
                leftGapPt_->setOrigIdx(leftIdx); // leftIdx_ = leftIdx;
                leftGapPt_->setOrigRange(leftRange); // leftRange_ = leftRange;

                // initializing convex polar gap coordinates to raw ones
                leftGapPt_->initManipPoint();
                rightGapPt_->initManipPoint();

                setRadial();
                setRightType();
            }

            void setGapLifespan(const float & gapLifespan) { gapLifespan_ = gapLifespan; }

            void updateGapLifespan(const float & t_current) { gapLifespan_ = t_current - gapStart_; }

            float getGapLifespan() const { return gapLifespan_; }

            /**
            * \brief Getter for initial left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getLCartesian(float &x, float &y) const { leftGapPt_->getOrigCartesian(x, y); }

            /**
            * \brief Getter for right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getRCartesian(float &x, float &y) const { rightGapPt_->getOrigCartesian(x, y); }

            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \param x x-position for left gap point
            * \param y y-position for left gap point
            */
            void getManipulatedLCartesian(float &x, float &y) const { leftGapPt_->getManipCartesian(x, y); }

            /**
            * \brief Getter for initial manipulated right gap point in Cartesian frame
            * \param x x-position for right gap point
            * \param y y-position for right gap point
            */
            void getManipulatedRCartesian(float &x, float &y) const { rightGapPt_->getManipCartesian(x, y); }

            Eigen::Vector2f getLPosition() const { return leftGapPt_->getOrigCartesian(); }

            Eigen::Vector2f getRPosition() const { return rightGapPt_->getOrigCartesian(); }

            /**
            * \brief Getter for initial left gap velocity in Cartesian frame
            * \return Initial left gap velocity in Cartesian frame
            */
            Eigen::Vector2f getLVelocity() const { return leftGapPt_->getModel()->getGapVelocity(); }

            /**
            * \brief Getter for initial right gap velocity in Cartesian frame
            * \return Initial right gap velocity in Cartesian frame
            */
            Eigen::Vector2f getRVelocity() const { return rightGapPt_->getModel()->getGapVelocity(); }


            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \return Initial manipulated left gap point in Cartesian frame
            */
            Eigen::Vector2f getManipulatedLPosition() const { return leftGapPt_->getManipCartesian(); }

            /**
            * \brief Getter for initial manipulated right gap point in Cartesian frame
            * \return Initial manipulated right gap point in Cartesian frame
            */
            Eigen::Vector2f getManipulatedRPosition() const { return rightGapPt_->getManipCartesian(); }

            /**
            * \brief Getter for initial manipulated left gap point in Cartesian frame
            * \return Initial manipulated left gap point in Cartesian frame
            */
            Eigen::Vector2f getManipulatedLVelocity() const { return leftGapPt_->getModel()->getManipGapVelocity(); }

            /**
            * \brief Getter for initial manipulated right gap point in Cartesian frame
            * \return Initial manipulated right gap point in Cartesian frame
            */
            Eigen::Vector2f getManipulatedRVelocity() const { return rightGapPt_->getModel()->getManipGapVelocity(); }

            void setRightType() { rightType_ = rightGapPt_->getOrigRange() < leftGapPt_->getOrigRange(); }

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
                int checkLeftIdx = leftGapPt_->getOrigIdx(); // leftIdx_;
                int checkRightIdx = rightGapPt_->getOrigIdx(); // rightIdx_;

                float checkLeftRange = leftGapPt_->getOrigRange(); // leftRange_;
                float checkRightRange = rightGapPt_->getOrigRange(); // rightRange_;

                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftIdx: " << checkLeftIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkLeftRange: " << checkLeftRange);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightIdx: " << checkRightIdx);
                // ROS_INFO_STREAM_NAMED("Gap", "   checkRightRange: " << checkRightRange);

                float gapAngle = (checkLeftIdx - checkRightIdx) * angle_increment;
                if (gapAngle < 0)
                    gapAngle += TWO_M_PI;

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

            std::string getFrame() const { return frame_; }
            
            void setEndCondition(const int & end_condition) { end_condition_ = end_condition; }
            int getEndCondition() const { return end_condition_; }

            GapPoint * getLeftGapPt() const { return leftGapPt_; }
            GapPoint * getRightGapPt() const { return rightGapPt_; }

            /**
            * \brief Getter for gap radial condition
            * \return Gap radial condition
            */
            bool isRadial() const { return radial_; }

            /**
            * \brief Getter for gap "right type" (if right point is closer than left point) condition
            * \return Gap "right type" condition
            */
            bool isRightType() const { return rightType_; }

            void setRGC() { rgc_ = true; }

            /**
            * \brief Getter for gap converted to swept gap condition
            * \return Gap converted to swept gap condition
            */
            bool isRGC() const { return rgc_; }

            bool isAvailable() const { return available_; }

            GapGoal * getGoal() const { return goal_; }

            void setGlobalGoalWithin() { globalGoalWithin = true; }
            bool getGlobalGoalWithin() const { return globalGoalWithin; }

            void setTInterceptGoal(const float & t_intercept_goal) { tInterceptGoal_ = t_intercept_goal; }
            float getTInterceptGoal() const { return tInterceptGoal_; }

            void setGammaInterceptGoal(const float & gamma_intercept_goal) { gammaInterceptGoal_ = gamma_intercept_goal; }
            float getGammaInterceptGoal() const { return gammaInterceptGoal_; }

            // void setSafeToDelete() { safe_to_delete_ = true; }
            // bool getSafeToDelete() const { return safe_to_delete_; }

            float getGapStart() const { return gapStart_; }

        private:
            std::string frame_ = ""; /**< Frame ID for gap */

            float gapStart_ = -1.0; /**< Gap start time */
            float gapLifespan_ = -1.0; /**< Gap lifespan over prediction horizon */

            int end_condition_ = UNSET; 

            // bool safe_to_delete_ = false; /**< Flag for if gap is safe to delete */

            // float minSafeDist_ = 0.0; /**< minimum safe distance for current gap */

            // Eigen::Vector2f extendedGapOrigin_; /**< current extended gap origin point */
            // Eigen::Vector2f termExtendedGapOrigin_; /**< terminal extended gap origin point */

            GapPoint * leftGapPt_ = NULL; /**< Left gap point */
            GapPoint * rightGapPt_ = NULL; /**< Right gap point */
            
            bool opening_ = false; /**< Opening gap identifier */

            bool radial_ = false; /**< Initial gap radial characteristic identifier */
            
            bool rightType_ = false; /**< Initial gap right type characteristic identifier */

            bool rgc_ = false; /**< flag for if gap has been converted into swept gap */

            bool available_ = true; /**< flag for if gap is available */

            GapGoal * goal_ = NULL; /**< Gap goal */
            bool globalGoalWithin = false; /**< Flag for if global goal lies within gap's angular span */
            float tInterceptGoal_ = 0.0;  /**< Intercept time for gap goal point */
            float gammaInterceptGoal_ = 0.0; /**< Intercept angle for gap goal point */

    };
}