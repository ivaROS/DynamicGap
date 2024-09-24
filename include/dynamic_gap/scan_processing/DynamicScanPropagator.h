#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>

#include <dynamic_gap/utils/Gap.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap
{    
    /**
    * \brief Class responsible for propagating the current laser scan forward in time
    *        for scoring future poses in trajectories
    */
    class DynamicScanPropagator 
    {
        public: 
            DynamicScanPropagator(ros::NodeHandle& nh, const DynamicGapConfig& cfg);

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            /**
            * \brief propagate laser scan forward in time using raw gap models

            * \param rawGaps set of current raw gaps to extract models from to determine what parts of scan are dynamic
            * \return set of propagated laser scans for scoring
            * */
            std::vector<sensor_msgs::LaserScan> propagateCurrentLaserScan(const std::vector<dynamic_gap::Gap *> & rawGaps);


        private:

            /**
            * \brief function for publishing propagated laser scans
            * \param dynamicLaserScan propagated laser scan to publish in RViz
            */
            void visualizePropagatedEgocircle(const sensor_msgs::LaserScan & dynamicLaserScan);

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */

            ros::Publisher propagatedEgocirclePublisher_; /**< Publisher for propagated egocircle */
            
            const DynamicGapConfig* cfg_;
    };


}