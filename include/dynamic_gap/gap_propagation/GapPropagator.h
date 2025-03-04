#pragma once

#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>


namespace dynamic_gap 
{
    /**
    * \brief Class responsible for evaluating a gap's feasibility by forward-simulating the gap
    * according to its dynamics model and determining if it is feasible for the robot
    * to pass through the gap before it closes
    */
    class GapPropagator 
    {
        public: 
            GapPropagator(const DynamicGapConfig& cfg) {cfg_ = &cfg;};

            /**
            * \brief Set terminal range and bearing values for gap based on 
            * where gap crossed
            * \param gap incoming gap whose points we want to propagate
            */                 
            void propagateGapPoints(Gap * gap);

        private:
            /**
            * \brief Rewind crossed gap to find last point in time in which
            * robot can fit through gap 
            * \param t crossing time of gap
            * \param gap incoming gap whose feasibility we want to evaluate
            * \return last point in time in which robot can fit through gap
            */
            float rewindGapPoints(const float & t, 
                                    Gap * gap);


            const DynamicGapConfig* cfg_; /**< Planner hyperparameter config list */        
    };
}