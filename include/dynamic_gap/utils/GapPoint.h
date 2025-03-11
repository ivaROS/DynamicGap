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
    * \brief Class for a single gap point
    */
    class GapPoint
    {
        public:

            GapPoint() 
            { 
                // Do NOT want to initialize model here
            }

            GapPoint(const int & idx, const float & range)
            {
                orig.idx_ = idx;
                orig.range_ = range;

                // Here, you can define what type of model you want to use
                model_ = new RotatingFrameCartesianKalmanFilter();
                // model_ = new PerfectEstimator();
            }

            GapPoint(const GapPoint & otherGapPoint)
            {
                orig.idx_ = otherGapPoint.orig.idx_;
                orig.range_ = otherGapPoint.orig.range_;

                manip.idx_ = otherGapPoint.manip.idx_;
                manip.range_ = otherGapPoint.manip.range_;

                // Deep copy of models
                model_ = new RotatingFrameCartesianKalmanFilter();
                // model_ = new PerfectEstimator();

                model_->transfer(*otherGapPoint.model_);
            }

            // Run before manipulation
            GapPoint(const PropagatedGapPoint & propagatedGapPoint)
            {
                orig.idx_ = theta2idx(propagatedGapPoint.getModel()->getGapBearing());
                orig.range_ = propagatedGapPoint.getModel()->getGapRange();

                // Here, you can define what type of model you want to use
                model_ = new RotatingFrameCartesianKalmanFilter();
                // model_ = new PerfectEstimator();

                model_->transferFromPropagatedGapPoint(*propagatedGapPoint.getModel());
            }

            bool checkOrigPoint()
            {
                if (std::isnan(orig.range_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has NaN range");
                    return false;
                }

                if (std::isinf(orig.range_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has Inf range");
                    return false;
                }

                if (std::isnan(orig.idx_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has NaN index");
                    return false;
                }

                if (std::isinf(orig.idx_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has negative or Inf index");
                    return false;
                }

                if (orig.idx_ < 0)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has a negative index");
                    return false;
                }

                if (orig.range_ < 0)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has a negative range");
                    return false;
                }

                if (orig.idx_ >= 2*half_num_scan)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has an index out of bounds");
                    return false;
                }

                return true;                
            }

            bool checkManipPoint()
            {
                if (std::isnan(manip.range_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has NaN range");
                    return false;
                }

                if (std::isinf(manip.range_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has Inf range");
                    return false;
                }

                if (std::isnan(manip.idx_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has NaN index");
                    return false;
                }

                if (std::isinf(manip.idx_))
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has negative or Inf index");
                    return false;
                }

                if (manip.idx_ < 0)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has a negative index");
                    return false;
                }

                if (manip.range_ < 0)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has a negative range");
                    return false;
                }

                if (manip.idx_ >= 2*half_num_scan)
                {
                    ROS_WARN_STREAM_NAMED("Gap", "Gap point has an index out of bounds");
                    return false;
                }

                return true;                
            }

            ~GapPoint()
            {
                delete model_;
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

            Estimator * getModel() const
            {
                return model_;
            }

            void getOrigCartesian(float &x, float &y) const
            {
                float theta = idx2theta(orig.idx_);
                x = (orig.range_) * std::cos(theta);
                y = (orig.range_) * std::sin(theta);
            }

            void getManipCartesian(float &x, float &y) const
            {
                float theta = idx2theta(manip.idx_);
                x = (manip.range_) * std::cos(theta);
                y = (manip.range_) * std::sin(theta);
            }            

            Eigen::Vector2f getManipCartesian() const
            {
                float x = 0.0, y = 0.0;
                getManipCartesian(x, y);

                return Eigen::Vector2f(x, y);
            }

            void initManipPoint()
            {
                manip.idx_ = orig.idx_;
                manip.range_ = orig.range_;
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