#pragma once

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/utils/GapPoint.h>
#include <dynamic_gap/utils/UngapPoint.h>
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
                left = new UngapPoint(*leftGapPt);
                right = new UngapPoint(*rightGapPt);
                ungapID_ = ungapID;
            }

            ~Ungap()
            {
                delete left;
                delete right;
            }

        private:
            int ungapID_ = -1; // set one time at init

            UngapPoint * left;
            UngapPoint * right;
    };
}