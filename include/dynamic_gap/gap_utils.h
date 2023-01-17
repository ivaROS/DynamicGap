#ifndef GAP_FINDER_H
#define GAP_FINDER_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_gap/gap.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <dynamic_gap/dynamicgap_config.h>

namespace dynamic_gap {
    class GapUtils 
    {
        public: 
        GapUtils();

        ~GapUtils();

        GapUtils(const DynamicGapConfig& cfg);

        GapUtils& operator=(GapUtils other) {cfg_ = other.cfg_;};

        GapUtils(const GapUtils &t) {cfg_ = t.cfg_;};

        bool sweptGapStartedOrEnded(float scan_dist_i, float scan_dist_imin1, float max_scan_dist);

        bool sweptGapSizeCheck(dynamic_gap::Gap detected_gap, float half_scan);

        bool radialGapSizeCheck(float scan_dist_i, float scan_dist_imin1, float max_scan_dist, float gap_angle);

        bool bridgeCondition(std::vector<dynamic_gap::Gap> raw_gaps, int scan_size);

        bool terminalGoalGapCheck(geometry_msgs::PoseStamped final_goal_rbt, 
                                        sensor_msgs::LaserScan stored_scan_msgs,
                                        int & final_goal_idx);

        std::vector<dynamic_gap::Gap> hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
                                                    geometry_msgs::PoseStamped final_goal_rbt);
    
        std::vector<dynamic_gap::Gap> mergeGapsOneGo(boost::shared_ptr<sensor_msgs::LaserScan const>, std::vector<dynamic_gap::Gap>&);

        void addTerminalGoal(int, 
                                std::vector<dynamic_gap::Gap> &, 
                                sensor_msgs::LaserScan);        

        void setMergeThreshold(float);
        void setIdxThreshold(int);
        std::vector<Eigen::Matrix<double, 4, 1> > getCurrAgents();

        sensor_msgs::LaserScan staticDynamicScanSeparation(std::vector<dynamic_gap::Gap> observed_gaps, 
                                                            boost::shared_ptr<sensor_msgs::LaserScan const> msg,
                                                            bool print);

        private:
            const DynamicGapConfig* cfg_;
            std::vector<Eigen::Matrix<double, 4, 1> > curr_agents;

    };


}


#endif