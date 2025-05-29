#pragma once

#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_gap/utils/Ungap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>


namespace dynamic_gap 
{
    /**
    * \brief Class responsible for evaluating a gap's feasibility by forward-simulating the gap
    * according to its dynamics model and determining if it is feasible for the robot
    * to pass through the gap before it closes
    */
    class UngapFeasibilityChecker 
    {
        public: 
            UngapFeasibilityChecker(const DynamicGapConfig& cfg) {cfg_ = &cfg;};

            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            /**
            * \brief Check gap lifespan against pursuit guidance kinematics to
            * determine the gap's feasibility
            * \param gap incoming gap whose feasibility we want to evaluate
            */   
            bool pursuitGuidanceAnalysis(Ungap * ungap);

        private:

            bool parallelNavigationFeasibilityCheck(Ungap * ungap);

            void parallelNavigationHelper(const Eigen::Vector2f & p_target, 
                                            const Eigen::Vector2f & v_target, 
                                            const float speed_robot,
                                            float & t_intercept, 
                                            float & gamma_intercept);

            const DynamicGapConfig* cfg_; /**< Planner hyperparameter config list */

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */            
    };
}