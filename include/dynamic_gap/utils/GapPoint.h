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
    * \brief Class for a single gap point
    */
    class GapPoint
    {
        public:
            GapPoint(const int & idx, const float & range)
            {
                orig.idx_ = idx;
                orig.range_ = range;
            }

            void setOrigIdx(const int & idx)
            {
                orig.idx_ = idx;
            }

            void setOrigRange(const float & range)
            {
                orig.range_ = range;
            }

            int getOrigIdx() const
            {
                return orig.idx_;
            }

            float getOrigRange() const
            {
                return orig.range_;
            }

            void setManipIdx(const int & idx)
            {
                manip.idx_ = idx;
            }

            void setManipRange(const float & range)
            {
                manip.range_ = range;
            }

            int getManipIdx() const
            {
                return manip.idx_;
            }

            float getManipRange() const
            {
                return manip.range_;
            }

            void setModel(Estimator * model)
            {
                model_ = model;
            }

        private:

            Estimator * model_ = NULL; /**< Gap point estimator */

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
            }

    };
}