#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/trajectory_scoring/TrajectoryScorer.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap 
{
    /**
    * \brief Class responsible for manipulating feasible gaps to ensure that the gaps meet whatever assumptions
    * are necessary for our safety guarantee
    */    
    class GapManipulator 
    {
        public: 
            GapManipulator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg;};

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
            // void updateStaticEgoCircle(const sensor_msgs::LaserScan &);
            // void updateDynamicEgoCircle(dynamic_gap::Gap * gap,
            //                             const std::vector<sensor_msgs::LaserScan> & futureScans);

            /**
            * \brief algorithm for setting gap goal 
            * \param gap queried gap
            * \param globalPathLocalWaypoint local waypoint along global path in robot frame
            * \param initial boolean for if we are setting initial gap goal or terminal gap goal
            */
            void setGapGoal(dynamic_gap::Gap * gap, 
                            const geometry_msgs::PoseStamped & globalPathLocalWaypoint, 
                            const bool & initial); 
            
            /**
            * \brief set desired position at end of prediction horizon for robot within queried gap
            * \param gap queried gap
            * \param globalPathLocalWaypoint local waypoint along global path in robot frame
            */                            
            void setGapTerminalGoal(dynamic_gap::Gap * gap, 
                                    const geometry_msgs::PoseStamped & globalPathLocalWaypoint);
            
            /**
            * \brief function for reducing gap's angle to ensure that gap is convex (angle < 180 degrees)
            * \param gap queried gap
            * \param globalPathLocalWaypoint local waypoint along global path in robot frame
            * \param initial boolean for if we are setting initial gap parameters or terminal gap parameters
            */
            void reduceGap(dynamic_gap::Gap * gap, 
                            const geometry_msgs::PoseStamped & globalPathLocalWaypoint, 
                            const bool & initial);

            /**
            * \brief function for converting a radial gap into a swept goal to allow for
            * higher gap visibility
            * \param gap queried gap
            * \param initial boolean for if we are setting initial gap parameters or terminal gap parameters
            */
            void convertRadialGap(dynamic_gap::Gap * gap, 
                                    const bool & initial);

            /**
            * \brief function for extending gap behind robot to ensure that robot starts its trajectory within gap
            * \param gap queried gap
            * \param initial boolean for if we are setting initial gap parameters or terminal gap parameters
            */
            void radialExtendGap(dynamic_gap::Gap * gap, 
                                    const bool & initial); 
            
            /**
            * \brief function for inflating gap radially and angularly to account for robot size
            * \param gap queried gap
            * \param initial boolean for if we are setting initial gap parameters or terminal gap parameters
            */            
            void inflateGapSides(dynamic_gap::Gap * gap, 
                                    const bool & initial);

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
            * \param rightToGoalAngle angle swept out from right goal point to global path local waypoint
            * \param leftToWaypointAngle angle swept out from left goal point to global path local waypoint
            * \return biased bearing for gap goal placement
            */
            float setBiasedGapGoalTheta(const float & leftTheta, 
                                        const float & rightTheta, 
                                        const float & globalPathLocalWaypointTheta,
                                        const float & leftToRightAngle, 
                                        const float & rightToGoalAngle, 
                                        const float & leftToWaypointAngle);

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */

            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            // sensor_msgs::LaserScan staticScan_; /**< Current laser scan for static portion of environment */
            // sensor_msgs::LaserScan dynamicScan_; /**< Propagated laser scan for future timestep 
    };
}