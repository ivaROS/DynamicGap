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
            * \brief Forward propagate a gap through its dynamics model and evaluate
            * whether it is feasible for ego-robot to pass through the gap within prediction horizon
            * \param gap incoming gap whose feasibility we want to evaluate
            * \return boolean for if gap is feasible
            */
            bool indivGapFeasibilityCheck(dynamic_gap::Gap * gap);
            
        private:
            /**
            * \brief Build approximate robot trajectory through gap using cubic spline
            * and evaluate spline's velocity/acceleration to see if trajectory through gap is feasible
            * \param gap incoming gap whose feasibility we want to evaluate
            * \param crossingPt position where gap crosses and shuts (if this occurs)
            * \param gapCrossingTime time when gap crosses and shuts (if this occurs)
            */
            void gapSplinecheck(dynamic_gap::Gap * gap, 
                                 const Eigen::Vector2f & crossingPt,
                                 float & gapCrossingTime);

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

            /**
            * \brief Set terminal range and bearing values for gap based on 
            * where gap crossed
            * \param gap incoming gap whose feasibility we want to evaluate
            * \param gapCrossingPt 2D position in space at which gap shuts (left and right points meet)
            */                 
            float propagateGapPoints(dynamic_gap::Gap * gap, 
                                            Eigen::Vector2f& gapCrossingPt);

            const DynamicGapConfig* cfg_; /**< Planner hyperparameter config list */
    };
}