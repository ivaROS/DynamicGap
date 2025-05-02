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
            GapManipulator(const DynamicGapConfig& cfg) { cfg_ = &cfg; };

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            /**
            * \brief function for inflating gap radially and angularly to account for robot size
            * \param gap queried gap
            */            
            void inflateGapSides(Gap * gap);
            
            /**
            * \brief function for convering radial gaps into swept gaps to allow maneuvering around corners
            * \param gap queried gap
            */                   
            void convertRadialGap(std::vector<Gap *> const & gaps, const int & gapIdx);

        private:
        
            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */

            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */

    };
}