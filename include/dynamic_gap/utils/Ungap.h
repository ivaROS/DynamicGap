#pragma once

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_gap/utils/Utils.h>
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

        private:
            int ungapID_ = -1; // set one time at init

            UngapPoint * left;
            UngapPoint * right;
    };
}