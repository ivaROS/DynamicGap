#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dynamic_gap/utils/Gap.h>
#include <sensor_msgs/LaserScan.h>

namespace dynamic_gap
{
    class StaticScanSeparator 
    {
        public: 
            StaticScanSeparator(const DynamicGapConfig& cfg) { cfg_ = &cfg; }

            StaticScanSeparator& operator=(StaticScanSeparator other) {cfg_ = other.cfg_; return *this; }

            StaticScanSeparator(const StaticScanSeparator &t) {cfg_ = t.cfg_;}
            std::vector<Eigen::Vector4f> getCurrAgents() { return currAgents; }

            sensor_msgs::LaserScan staticDynamicScanSeparation(const std::vector<dynamic_gap::Gap *> & observed_gaps, 
                                                                boost::shared_ptr<sensor_msgs::LaserScan const> msg);

        private:
            bool checkModelSimilarity(dynamic_gap::Estimator * currModel, dynamic_gap::Estimator * prevModel);
            void createAgentFromModels(dynamic_gap::Estimator * currModel,    
                                        dynamic_gap::Estimator * prevModel,
                                        std::vector<Eigen::Vector4f> & agents);
            void clearAgentFromStaticScan(dynamic_gap::Estimator * currModel, 
                                            dynamic_gap::Estimator * prevModel,
                                            sensor_msgs::LaserScan & curr_scan);           


            const DynamicGapConfig* cfg_;
            std::vector<Eigen::Vector4f> currAgents;
    };
}
