#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Trajectory.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <vector>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <boost/thread/mutex.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <dynamic_gap/GapFeaturePrediction.h>
#include <ros/ros.h>
#include <map>
#include <string>
#include <cmath>
namespace dynamic_gap
{
    struct GruGapFeatureDensityEstimate
    {
        float pred_sector_density = 0.0f;
        ros::Time stamp;
        bool valid = false;
        int seq_len_used = 0;
    };

    /**
    * \brief Class responsible for scoring candidate trajectory according to
    * trajectory's proximity to local environment and global path's local waypoint
    */
    class TrajectoryEvaluator
    {
        public:
            TrajectoryEvaluator(
            ros::NodeHandle& nh,
            const DynamicGapConfig& cfg);

            /**
            * \brief receive new laser scan and update member variable accordingly
            * \param scan new laser scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
            
            // void updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan);
            
            /**
            * \brief Helper function for transforming global path local waypoint into robot frame
            * \param globalPathLocalWaypointOdomFrame Current local waypoint along global plan in robot frame
            * \param odom2rbt transformation from odom frame to robot frame
            */
            void transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                            const geometry_msgs::TransformStamped & odom2rbt);

            /**
            * \brief Function for evaluating pose-wise scores along candidate trajectory
            * \param traj candidate trajectory to score
            */
            void evaluateTrajectory(
                const Trajectory & traj,
                std::vector<float> & posewiseCosts,
                float & terminalPoseCost,
                const std::vector<sensor_msgs::LaserScan> & futureScans,
                const int & scanIdx,
                const int & densityModelID);

            /**
             * \brief Stores latest GRU density prediction for a gap model
             * \param msg incoming GRU density prediction message
             */
            void gruGapFeatureDensityCB(
                const dynamic_gap::GapFeaturePrediction::ConstPtr& msg);

            /**
             * \brief Gets latest valid GRU density prediction for a model
             * \param modelID gap model ID to query
             * \param currentStamp current time for freshness check
             * \param predDensityOut returned predicted density
             * \return true if a fresh valid prediction was found
             */
            bool getLatestGruGapFeatureDensityForModel(
                const int& modelID,
                const ros::Time& currentStamp,
                float& predDensityOut) const;
                        
        private:

            //////////////////////////////////////////////////////
            // GRU gap-density trajectory cost
            //////////////////////////////////////////////////////

            ros::Subscriber gruGapFeatureDensitySub_;

            bool useGruGapFeatureDensityCost_ = true;
            double maxGruGapFeaturePredictionAgeSec_ = 3.0;
            float gruGapDensityCostWeight_ = 1.0f;

            mutable boost::mutex gruGapFeatureDensityMutex_;

            std::map<int, GruGapFeatureDensityEstimate>
                latestGruGapFeatureDensityByModelID_;

            /**
            * \brief function for evaluating terminal waypoint cost for candidate trajectory
            * \param pose final pose in candidate trajectory to check against terminal waypoint
            * \return terminal waypoint cost for candidate trajectory
            */
            float terminalGoalCost(const geometry_msgs::Pose & pose);

            /**
            * \brief function for evaluating intermediate cost of pose for candidate trajectory (in static environment)
            * \param pose pose within candidate trajectory to evaluate
            * \return intermediate cost of pose
            */
            float evaluatePose(const geometry_msgs::Pose & pose,
                                const sensor_msgs::LaserScan scan_k) ;
            
            /**
            * \brief function for calculating intermediate trajectory cost (in static environment)
            * \param rbtToScanDist minimum distance from robot pose to current scan
            * \return intermediate cost of pose
            */
            float chapterCost(const float & rbtToScanDist);

            boost::mutex globalPlanMutex_; /**< mutex locking thread for updating current global plan */
            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            const DynamicGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */

            // sensor_msgs::LaserScan staticScan_;
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_; /**< Current local waypoint along global plan in robot frame */
    };
}