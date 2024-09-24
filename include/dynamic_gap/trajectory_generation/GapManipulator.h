#pragma once

#include <ros/ros.h>
#include <math.h>

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h>

namespace dynamic_gap 
{
    /**
    * \brief Class responsible for manipulating feasible gaps to ensure that the gaps meet whatever assumptions
    * are necessary for our safety guarantee
    */    
    class GapManipulator 
    {
        public: 
            GapManipulator(const dynamic_gap::DynamicGapConfig& cfg) { cfg_ = &cfg; };

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            /**
            * \brief algorithm for setting gap goal 
            * \param gap queried gap
            * \param globalPathLocalWaypointRobotFrame global path local waypoint in robot frame
            * \param globalGoalRobotFrame global goal in robot frame
            */
            void setGapGoal(dynamic_gap::Gap * gap, 
                            const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame, 
                            const geometry_msgs::PoseStamped & globalGoalRobotFrame); 
            
            /**
            * \brief function for extending gap behind robot to ensure that robot starts its trajectory within gap
            * \param gap queried gap
            */
            void radialExtendGap(dynamic_gap::Gap * gap); 
            
            /**
            * \brief function for inflating gap radially and angularly to account for robot size
            * \param gap queried gap
            */            
            bool inflateGapSides(dynamic_gap::Gap * gap);
            
            /**
            * \brief function for convering radial gaps into swept gaps to allow maneuvering around corners
            * \param gap queried gap
            */                   
            void convertRadialGap(dynamic_gap::Gap * gap);

        private:

            /**
            * \brief checking if global path local waypoint lies within gap
            * \param leftPt left gap point
            * \param rightPt right gap point
            * \param globalPathLocalWaypoint local waypoint along global path in robot frame
            * \return boolean for if global path local waypoint lies within gap
            */              
            bool checkWaypointVisibility(const Eigen::Vector2f & leftPt, 
                                            const Eigen::Vector2f & rightPt,
                                            const Eigen::Vector2f & globalPathLocalWaypoint);
            /**
            * \brief determining what bearing within gap to bias gap goal placement towards
            * \param leftTheta orientation of left gap point
            * \param rightTheta orientation of right gap point
            * \param globalPathLocalWaypointTheta orientation of global path local waypoint
            * \param leftToRightAngle angle swept out from left gap point to right gap point
            * \param leftToWaypointAngle angle swept out from left gap point to global path local waypoint
            * \param rightToWaypointAngle angle swept out from right gap point to global path local waypoint
            * \return biased bearing for gap goal placement
            */
            float setBiasedGapGoalTheta(const float & leftTheta, 
                                        const float & rightTheta, 
                                        const float & globalPathLocalWaypointTheta,
                                        const float & leftToRightAngle, 
                                        const float & leftToWaypointAngle, 
                                        const float & rightToWaypointAngle);

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */

            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
    };
}