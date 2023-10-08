#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <vector>
#include <numeric>
// #include <map>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
// #include <omp.h>
#include <boost/thread/mutex.hpp>
// #include "tf/transform_datatypes.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace dynamic_gap
{
    class TrajectoryScorer
    {
        public:
        TrajectoryScorer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
        TrajectoryScorer& operator=(TrajectoryScorer other) {cfg_ = other.cfg_; return *this;};
        TrajectoryScorer(const TrajectoryScorer &t) {cfg_ = t.cfg_;};
        
        void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
        void updateStaticEgoCircle(const sensor_msgs::LaserScan &);
        void updateLocalGoal(geometry_msgs::PoseStamped, geometry_msgs::TransformStamped);

        std::vector<double> scoreGaps();
        
        // Full Scoring
        // std::vector<double> scoreTrajectories(std::vector<geometry_msgs::PoseArray>);
        geometry_msgs::PoseStamped getLocalGoal() {return local_goal; }; // in robot frame
        std::vector<double> scoreTrajectory(const geometry_msgs::PoseArray & traj, 
                                            const std::vector<double> & time_arr, 
                                            const std::vector<dynamic_gap::Gap>& current_raw_gaps,
                                            const std::vector<geometry_msgs::Pose> & _agent_odoms, 
                                            const std::vector<geometry_msgs::Vector3Stamped> & _agent_vels,
                                            const std::vector<sensor_msgs::LaserScan> & future_scans,
                                            bool print,
                                            bool vis);
        
        void recoverDynamicEgoCircle(double t_i, double t_iplus1, 
                                     std::vector<Eigen::Matrix<double, 4, 1> > & curr_agents_lc,                            
                                     sensor_msgs::LaserScan& dynamic_laser_scan,
                                     bool print);
        
        void visualizePropagatedEgocircle(const sensor_msgs::LaserScan & dynamic_laser_scan);

        double terminalGoalCost(geometry_msgs::Pose pose);

        private:
            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            sensor_msgs::LaserScan static_scan;
            std::vector<dynamic_gap::Gap> gaps;
            geometry_msgs::PoseStamped local_goal;
            boost::mutex gap_mutex, gplan_mutex, egocircle_mutex;

            std::vector< std::vector<double> > sort_and_prune(const std::vector<Eigen::Matrix<double, 4, 1> > & _odom_vects);
            // std::vector< std::vector<double> > sort_and_prune(std::vector<geometry_msgs::Pose> _odom_vects);
            
            int sgn_star(float dy);
            double scorePose(geometry_msgs::Pose pose);
            int dynamicGetMinDistIndex(geometry_msgs::Pose pose, 
                                        const sensor_msgs::LaserScan & dynamic_laser_scan, 
                                        bool print);

            double dynamicScorePose(geometry_msgs::Pose pose, double theta, double range);
            double chapterScore(double d);
            double dynamicChapterScore(double d);
            double dist2Pose(float theta, float dist, geometry_msgs::Pose pose);
            
            int search_idx = -1;

            ros::Publisher propagatedEgocirclePublisher;
    };
}