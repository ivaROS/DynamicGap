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
                leftUngapPt_ = new UngapPoint(*leftGapPt);
                rightUngapPt_ = new UngapPoint(*rightGapPt);
                ungapID_ = ungapID;

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


        private:
            int ungapID_ = -1; // set one time at init

            UngapGoal * goal_ = NULL; /**< Gap goal */
            UngapPoint * leftUngapPt_;
            UngapPoint * rightUngapPt_;
    };
}