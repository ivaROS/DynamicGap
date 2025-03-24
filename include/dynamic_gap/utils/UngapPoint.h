#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/utils/PropagatedGapPoint.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dynamic_gap/gap_estimation/RotatingFrameCartesianKalmanFilter.h>
#include <dynamic_gap/gap_estimation/PerfectEstimator.h>

namespace dynamic_gap
{
    /**
    * \brief Class for a single un-gap point
    */
    class UngapPoint
    {
        public:

            UngapPoint() 
            { 
                // Do NOT want to initialize model here
            }

        private:
            Estimator * model_ = NULL; /**< Gap point estimator */

    };
}