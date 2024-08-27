#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <vector>
// #include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap 
{
    /**
    * \brief Class responsible for evaluating a gap's feasibility by forward-simulating the gap
    * according to its dynamics model and determining if it is feasible for the robot
    * to pass through the gap before it closes
    */
    class GapFeasibilityChecker 
    {
        public: 
            GapFeasibilityChecker(const ros::NodeHandle & nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg;};

            /**
            * \brief Set terminal range and bearing values for gap based on 
            * where gap crossed
            * \param gap incoming gap whose points we want to propagate
            */                 
            void propagateGapPoints(dynamic_gap::Gap * gap);

            /**
            * \brief Check gap lifespan against pursuit guidance kinematics to
            * determine the gap's feasibility
            * \param gap incoming gap whose feasibility we want to evaluate
            */   
            bool pursuitGuidanceAnalysis(dynamic_gap::Gap * gap);

        private:
            /**
            * \brief Rewind crossed gap to find last point in time in which
            * robot can fit through gap 
            * \param t crossing time of gap
            * \param gap incoming gap whose feasibility we want to evaluate
            * \return last point in time in which robot can fit through gap
            */
            float generateCrossedGapTerminalPoints(const float & t, 
                                                    dynamic_gap::Gap * gap);

            /**
            * \brief Set terminal range and bearing values for gap based on 
            * where gap crossed
            * \param gap incoming gap whose feasibility we want to evaluate
            * \param leftCrossPt last feasible point for left gap point
            * \param rightCrossPt last feasible point for right gap point
            */                        
            void generateTerminalPoints(dynamic_gap::Gap * gap, 
                                        const Eigen::Vector4f & leftCrossPt,
                                        const Eigen::Vector4f & rightCrossPt);

            bool parallelNavigationFeasibilityCheck(dynamic_gap::Gap * gap);

            void parallelNavigationHelper(const Eigen::Vector2f & p_target, 
                                            const Eigen::Vector2f & v_target, 
                                            const float speed_robot,
                                            float & t_intercept, 
                                            float & gamma_intercept);

            bool purePursuitFeasibilityCheck(dynamic_gap::Gap * gap);

            void purePursuitHelper(const Eigen::Vector2f & p_target, 
                                    const Eigen::Vector2f & v_target, 
                                    const float speed_robot,
                                    float & t_intercept);

            const DynamicGapConfig* cfg_; /**< Planner hyperparameter config list */
    };
}