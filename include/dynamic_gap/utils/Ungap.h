#pragma once

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/utils/GapPoint.h>
#include <dynamic_gap/utils/UngapPoint.h>
#include <dynamic_gap/utils/UngapGoal.h>
#include <dynamic_gap/gap_estimation/RotatingFrameCartesianKalmanFilter.h>
#include <dynamic_gap/gap_estimation/PerfectEstimator.h>

namespace dynamic_gap
{
    /**
    * \brief Class for a single un-gap
    */
    class Ungap
    {
        public:
            Ungap( GapPoint * leftGapPt, GapPoint * rightGapPt, const int & ungapID)
            {
                ungapID_ = ungapID;
                leftUngapPt_ = new UngapPoint(*leftGapPt, ungapID);
                rightUngapPt_ = new UngapPoint(*rightGapPt, ungapID);

                goal_ = new UngapGoal();
            }

            ~Ungap()
            {
                delete goal_;
                delete leftUngapPt_;
                delete rightUngapPt_;
            }

            UngapPoint * getLeftUngapPt() const { return leftUngapPt_; }
            UngapPoint * getRightUngapPt() const { return rightUngapPt_; }
            UngapGoal * getGoal() const { return goal_; }

            void setTInterceptGoal(const float & t_intercept_goal) { tInterceptGoal_ = t_intercept_goal; }
            float getTInterceptGoal() const { return tInterceptGoal_; }

            void setGammaInterceptGoal(const float & gamma_intercept_goal) { gammaInterceptGoal_ = gamma_intercept_goal; }
            float getGammaInterceptGoal() const { return gammaInterceptGoal_; }

            void setRbtSpeed(const float & rbt_speed) { rbtSpeed_ = rbt_speed; }
            float getRbtSpeed() const { return rbtSpeed_; }

        private:
            int ungapID_ = -1; // set one time at init

            UngapGoal * goal_ = NULL; /**< Gap goal */
            UngapPoint * leftUngapPt_;
            UngapPoint * rightUngapPt_;

            float tInterceptGoal_ = 0.0;  /**< Intercept time for gap goal point */
            float gammaInterceptGoal_ = 0.0; /**< Intercept angle for gap goal point */

            float rbtSpeed_ = 0.0; /**< Ungap speed */
    };
}