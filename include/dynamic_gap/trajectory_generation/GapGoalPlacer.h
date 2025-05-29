#pragma once

#include <ros/ros.h>
#include <math.h>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>

namespace dynamic_gap 
{
    /**
    * \brief Class responsible for placing goals within gaps
    */    
    class GapGoalPlacer 
    {
        public: 
            GapGoalPlacer(const DynamicGapConfig& cfg) { cfg_ = &cfg; };

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            // /**
            // * \brief algorithm for setting gap goal 
            // * \param gap queried gap
            // * \param globalPathLocalWaypointRobotFrame global path local waypoint in robot frame
            // * \param globalGoalRobotFrame global goal in robot frame
            // */
            // void setGapGoal(Gap * gap, 
            //                 const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame, 
            //                 const geometry_msgs::PoseStamped & globalGoalRobotFrame); 

            /**
            * \brief algorithm for setting gap goal 
            * \param gap queried gap
            * \param globalPathLocalWaypointRobotFrame global path local waypoint in robot frame
            * \param globalGoalRobotFrame global goal in robot frame
            */
            void setGapGoalV2(Gap * gap, 
                                const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame, 
                                const geometry_msgs::PoseStamped & globalGoalRobotFrame); 

            void setGapGoalFromPriorV2(Gap * gap,
                                        Gap * priorGap);
                                                     
            void setGapGoalFromNextV2(Gap * gap,
                                        Gap * nextGap);

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