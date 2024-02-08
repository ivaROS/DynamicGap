#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
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
            DynamicScanPropagator(const DynamicGapConfig& cfg);

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            /**
            * \brief Function for propagating laser scans forward for collision checking in the future 
            */
            std::vector<sensor_msgs::LaserScan> propagateCurrentLaserScanCheat(const std::vector<geometry_msgs::Pose> & currentTrueAgentPoses,
                                                                                const std::vector<geometry_msgs::Vector3Stamped> & currentTrueAgentVels);

        private:
            /**
            * \brief Function for propagating agents forward in time and populating propagated laser scan
            * \param t_i current time step
            * \param t_iplus1 next time step
            * \param propagatedAgents set of estimated agents to propagate forward in time
            * \param dynamicLaserScan propagated laser scan to populate
            */
            void recoverDynamicEgocircle(const float & t_i, 
                                        const float & t_iplus1, 
                                        std::vector<Eigen::Vector4f> & propagatedAgents,
                                        sensor_msgs::LaserScan & dynamicLaserScan);
        
            /**
            * \brief function for sorting through current set of agents and pruning those outside of current laser scan
            * \param agentPoses set of current agents in environment
            * \return sorted and pruned set of current agents in environment
            */
            std::vector<Eigen::Vector4f> sortAndPrune(const std::vector<Eigen::Vector4f> & agentPoses);

            /**
            * \brief Gap state comparator that uses the 2D position norm
            */
            struct Comparator 
            {
                bool operator() (Eigen::Vector4f & a, Eigen::Vector4f & b) 
                {
                    float aNorm = pow(a[0], 2) + pow(a[1], 2);
                    float bNorm = pow(b[0], 2) + pow(b[1], 2);
                    return aNorm < bNorm;
                }
            };


            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            std::vector<sensor_msgs::LaserScan> futureScans_; /**< Vector of (predicted) future scans */

            const DynamicGapConfig* cfg_;
    };


}