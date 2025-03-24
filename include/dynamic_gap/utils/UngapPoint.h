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

            UngapPoint(const GapPoint & gapPoint)
            {
                orig.idx_ = gapPoint.getOrigIdx();
                orig.range_ = gapPoint.getOrigRange();

                // Here, you can define what type of model you want to use
                // Deep copy of models
                model_ = new RotatingFrameCartesianKalmanFilter();
                // model_ = new PerfectEstimator();

                model_->transfer(*gapPoint.getModel());
            }

            ~UngapPoint()
            {
                delete model_;
            }

        private:

            /**
            * \brief Parameters of original form of gap
            */        
            struct Orig
            {
                int idx_ = -1; /**< Original gap point index */
                float range_ = -1.0; /**< Original gap point range */
            } orig;

            /**
            * \brief Parameters of manipulated form of gap
            */
            struct Manip
            {
                int idx_ = -1; /**< Manipulated gap point index */
                float range_ = -1.0; /**< Manipulated gap point range */
            } manip;

            Estimator * model_ = NULL; /**< Gap point estimator */

    };
}