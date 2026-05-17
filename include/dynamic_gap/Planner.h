#pragma once
#include <limits>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <numeric>
#include <iostream>
#include <chrono>
// #include <map>

#include <math.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
// #include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>

#include <dynamic_gap/utils/Ungap.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Trajectory.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/gap_association/GapPointAssociator.h>
#include <dynamic_gap/gap_detection/GapDetector.h>
#include <dynamic_gap/gap_propagation/GapPropagator.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/visualization/GapVisualizer.h>
#include <dynamic_gap/visualization/GoalVisualizer.h>
#include <dynamic_gap/visualization/AgentVisualizer.h>
#include <dynamic_gap/visualization/TrajectoryVisualizer.h>
#include <dynamic_gap/global_plan_management/GlobalPlanManager.h>
#include <dynamic_gap/scan_processing/DynamicScanPropagator.h>
#include <dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h>
#include <dynamic_gap/trajectory_generation/GapManipulator.h>
#include <dynamic_gap/trajectory_generation/GapGoalPlacer.h>
#include <dynamic_gap/trajectory_generation/GapTrajectoryGenerator.h>
#include <dynamic_gap/trajectory_generation/UngapTrajectoryGenerator.h>
#include <dynamic_gap/trajectory_tracking/TrajectoryController.h>
#include <dynamic_gap/gap_feasibility/GapFeasibilityChecker.h>
#include <dynamic_gap/ungap_feasibility/UngapFeasibilityChecker.h>
#include <dynamic_gap/TimeKeeper.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <limits>
#include <dynamic_gap/GapPointObservation.h>
#include <dynamic_gap/GapVelocityPrediction.h>
#include <map>
#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_gap/GapFeatureObservation.h>

namespace dynamic_gap
{
    /**
    * \brief Class responsible for the core planning loop of dynamic gap.
    */
    class Planner
    {

        public:
            Planner() {}

            ~Planner();

            /**
            * \brief initialize Planner class
            * 
            * \param name planner name (used for ROS namespaces) 
            * \return initialization success / failure
            */
            bool initialize(const std::string & name);

            /**
            * \brief Indicator for if planner has been initialized
            * \return boolean for if planner has been initialized 
            */
            int initialized() { return initialized_; } 

            void setParams(const EstimationParameters & estParams, const ControlParameters & ctrlParams);

            /**
            * \brief Check if global goal has been reached by robot
            * \return boolean for if global goal has been reached or not
            */
            bool isGoalReached();

            float gapPointDensityRadius_ = 3.0f;

           struct PerfectGapVelocityLabel
            {
                Eigen::Vector2f rel_vel;
                Eigen::Vector2f world_vel_robot;

                std::string matched_agent_id;
                float match_dist = std::numeric_limits<float>::infinity();
                bool matched_dynamic_agent = false;


            };

            bool rawGapPointMatchesDynamicGroundTruthAgent( // for density calculation
            const Eigen::Vector2f& rawGapPointRobotFrame,
            std::string& matchedAgentID,
            float& matchDist,
            float& matchedAgentGroundTruthSpeed) const;

            void computeSimplifiedGapGroundTruthSectorDensity(
            const int& simplifiedGapIndex,
            int& gtSectorDynamicRawGapPointCount,
            float& gtSectorDensity) const;

            PerfectGapVelocityLabel computePerfectGapVelocityLabel( // for GRU learn gap point velocity
            const Eigen::Vector2f& gapPtRobotFrame,
            const std::map<std::string, geometry_msgs::Pose>& trueAgentPoses,
            const std::map<std::string, geometry_msgs::Vector3Stamped>& trueAgentVels,
            const geometry_msgs::TwistStamped& robotVelRobotFrame,
            const float& matchThresh) const;

            geometry_msgs::TwistStamped getRobotVelocityForGapLabel(
            const std::vector<geometry_msgs::TwistStamped>& intermediateRbtVels) const;
            float perfectGapVelMatchThresh_ = 0.6; // TODO:  add to config. 0.6 should be more than enough
            // i eyeballed rviz measured the DIAMETER of the agent to 0.3 
            
            ros::Publisher gapPointObservationPublisher_;
            bool publishGapPointObservations_ = true;

            void publishGapPointObservation(
            const ros::Time& stamp,
            const int& gapIndex,
            const std::string& side,
            Estimator* model,
            const Eigen::Vector2f& measurement,
            const Eigen::Vector4f& kalmanState,
            const PerfectGapVelocityLabel& perfectLabel);

            /**
            * \brief Function for core planning loop of dynamic gap
            * \param chosenTraj trajectory that is synthesized during planning loop
            * \param trajFlag flag for if robot is idling or moving
            * \return selected trajectory in odometry frame to track
            */
            void runPlanningLoop(Trajectory & chosenTraj, int & trajFlag);        

            /**
            * \brief Generate command velocity based on trajectory tracking and safety modules
            * \param localTrajectory selected local trajectory in odometry frame to track
            * \param trajFlag flag for if robot is idling or moving
            * \return command velocity to send to robot
            */
            geometry_msgs::Twist ctrlGeneration(const geometry_msgs::PoseArray & localTrajectory,
                                                int & trajFlag);

            /**
            * \brief Final command velocity pre-processing to check if robot is stuck
            * and planning has failed
            * \param cmdVel most recent command velocity
            * \return boolean for if planner is functioning and command velocity is fine
            */
            bool recordAndCheckVel(const geometry_msgs::Twist & cmdVel, const int & trajFlag);

            /**
            * \brief Interface function for receiving incoming global plan and updating
            * the global plan local waypoint
            * \param globalPlanMapFrame incoming global plan in map frame
            * \return boolean type on whether planner successfully registered goal
            */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame);

            /**
            * \brief Apply hard reset to planner to empty command velocity buffer
            * and wipe currently tracking trajectory
            */
            void reset();

            /**
            * \brief Function to check if global goal has been reached
            * \param status whether or not global goal has been reached 
            */
            void setReachedGlobalGoal(const bool & status) { reachedGlobalGoal_ = status; }


            struct GruGapVelocityEstimate
            {
                Eigen::Vector2f rel_vel;
                ros::Time stamp;
                bool valid = false;
                int seq_len_used = 0;
            };
            //
            ros::Subscriber gruGapVelocitySub_;

            std::map<int, GruGapVelocityEstimate> latestGruGapVelocityByModelID_;
            mutable boost::mutex gruGapVelocityMutex_;

            bool useGruGapVelocity_ = true;
            double maxGruPredictionAgeSec_ = 0.50;

            void gruGapVelocityCB(const dynamic_gap::GapVelocityPrediction::ConstPtr& msg);

            bool getLatestGruVelocityForModel(
                const int& modelID,
                const ros::Time& currentStamp,
                Eigen::Vector2f& relVelOut) const;

            int scanIdxCircularWidth(
                const int& rightIdx,
                const int& leftIdx,
                const int& scanSize) const;

            float gapAngularWidthRad(
                const int& rightIdx,
                const int& leftIdx) const;

            float getGapPointDynamicSpeedForDensity(
                Estimator* model) const;

            void computeSimplifiedGapSectorDensity(
                const int& simplifiedGapIndex,
                int& sectorDynamicRawGapPointCount,
                float& sectorRadius,
                float& sectorAngleRad,
                float& sectorArea,
                float& sectorDensity) const;

           void publishGapFeatureObservation(
            const ros::Time& stamp,
            const int& gapIndex,
            const int& modelID,
            const std::string& side,
            const float& visX,
            const float& visY,
            const float& sectorDensity,
            const int& sectorDynamicRawGapPointCount,
            const int& containedRawGapPointCount,
            const float& sectorArea,
            const float& sectorAngleRad,
            const float& sectorRadius,
            const float& gtSectorDensity,
            const int& gtSectorDynamicRawGapPointCount);
            float gapDensityDynamicSpeedThresh_ = 0.05f;


            int getGapLeftModelIDForDensityCost(
            Gap* gap) const;

        private:

            void attachUngapIDs(const std::vector<Gap *> & planningGaps,
                                        std::vector<Ungap *> & ungaps);

            bool isUngap(const Eigen::Vector4f & ptIState, const Eigen::Vector4f & ptJState);

            std::vector<Ungap *> pruneApproachingUngaps(const std::vector<Ungap *> & ungaps);

            /**
            * \brief Function for updating the gap models
            * \param gaps set of gaps whose models we are updating
            * \param intermediateRbtVels intermediate robot velocity values between last model update and current model update
            * \param intermediateRbtAccs intermediate robot acceleration values between last model update and current model update
            * \param tCurrentFilterUpdate time step for current estimator update
            */
            void updateModels(
            std::vector<Gap *> & gaps,
                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                const ros::Time & tCurrentFilterUpdate,
                const bool& logSimplifiedGapVelocityLabels
            );


            /**
            * \brief Function for updating a single gap's models
            * \param idx index of gap whose models we must update
            * \param gaps set of gaps whose models we are updating
            * \param intermediateRbtVels intermediate robot velocity values between last model update and current model update
            * \param intermediateRbtAccs intermediate robot acceleration values between last model update and current model update
            * \param tCurrentFilterUpdate time step for current estimator update
            */
                void updateModel(
                const int & idx,
                std::vector<Gap *> & gaps,
                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,
                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
                const ros::Time & tCurrentFilterUpdate,
                const bool& logSimplifiedGapVelocityLabels
            );

            /**
            * \brief Call back function for other agent odometry messages
            * \param agentOdomMsg incoming agent odometry message
            */
            void pedOdomCB(const pedsim_msgs::AgentStatesConstPtr& agentOdomMsg);

            // /**
            // * \brief Call back function for MPC output
            // * \param mpcOutput output decision variables for MPC
            // */
            // void mpcOutputCB(boost::shared_ptr<geometry_msgs::PoseArray> mpcOutput);

            /**
            * \brief Call back function to robot laser scan
            * \param scan incoming laser scan msg
            */
            void laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan> scan);

            /**
            * \brief Joint call back function for robot pose (position + velocity) and robot acceleration messages
            * \param rbtOdomMsg incoming robot odometry message
            * \param rbtAccelMsg incoming robot acceleration message
            */
            void jointPoseAccCB(const nav_msgs::Odometry::ConstPtr & rbtOdomMsg, 
                                const geometry_msgs::TwistStamped::ConstPtr & rbtAccelMsg);

            /**
            * \brief Function for updating all tf transform at the beginning of every planning cycle
            * \param msg incoming agent odometry message
            */
            void tfCB(const tf2_msgs::TFMessage& msg);

            /**
            * \brief Function for all member objects updating their current egocircles
            */
            void updateEgoCircle();

            // /**
            // * \brief Function for evaluating the feasibility of a set of gaps
            // * \param manipulatedGaps set of manipulated gaps which we will evaluate for feasibility
            // * \param isCurrentGapFeasible boolean for if the gap the robot is currently traveling through is feasible
            // * \return set of feasible gaps
            // */
            // std::vector<Gap *> gapSetFeasibilityCheck(const std::vector<Gap *> & manipulatedGaps, 
            //                                             bool & isCurrentGapFeasible);

            std::vector<GapTube *> gapSetFeasibilityCheckV2(const std::vector<GapTube *> & gapTubes, 
                                                            bool & isCurrentGapFeasible);         

            void ungapSetFeasibilityCheck(const std::vector<Ungap *> & recedingUngaps);

            // /**
            // * \brief Function for propagating current gap points/models forward in time
            // * \param planningGaps set of gaps we will use to plan 
            // * \return set of propagated gaps we will use to plan
            // */
            // void propagateGapPoints(const std::vector<Gap *> & planningGaps);            
            
            /**
            * \brief Function for propagating current gap points/models forward in time (v2)
            * \param planningGaps set of gaps we will use to plan
            * \return set of propagated gaps we will use to plan
            */
            void propagateGapPointsV2(const std::vector<Gap *> & planningGaps,
                                        std::vector<GapTube *> & gapTubes);

            /**
            * \brief Function for performing gap manipulation steps
            * \param planningGaps set of gaps we will use to plan 
            * \return manipulated set of gaps
            */
            std::vector<Gap *> manipulateGaps(const std::vector<Gap *> & planningGaps);

            // /**
            // * \brief Function for generating candidate trajectories through the current set of gaps
            // * \param gaps incoming set of gaps through which we want to generate trajectories
            // * \param generatedTrajs set of generated trajectories
            // * \param pathPoseScores set of posewise scores for all paths
            // * \param pathTerminalPoseScores set of terminal pose scores for all paths
            // * \param futureScans set of propagated scans to use during scoring
            // * \return Vector of pose-wise scores for the generated trajectories
            // */
            // void generateGapTrajs(std::vector<Gap *> & gaps, 
            //                         std::vector<Trajectory> & generatedTrajs,
            //                         std::vector<std::vector<float>> & pathPoseScores,
            //                         std::vector<float> & pathTerminalPoseScores,
            //                         const std::vector<sensor_msgs::LaserScan> & futureScans);

            void generateIdlingTraj(std::vector<Trajectory> & generatedTrajs,
                                    std::vector<std::vector<float>> & pathPoseCosts,
                                    std::vector<float> & pathTerminalPoseCosts,
                                    const std::vector<sensor_msgs::LaserScan> & futureScans);

            /**
            * \brief Function for generating candidate trajectories through the current set of gaps
            * \param gaps incoming set of gaps through which we want to generate trajectories
            * \param generatedTrajs set of generated trajectories
            * \param pathPoseScores set of posewise scores for all paths
            * \param pathTerminalPoseScores set of terminal pose scores for all paths
            * \param futureScans set of propagated scans to use during scoring
            * \return Vector of pose-wise scores for the generated trajectories
            */
            void generateUngapTrajs(std::vector<Ungap *> & ungaps, 
                                    std::vector<Trajectory> & generatedTrajs,
                                    std::vector<std::vector<float>> & pathPoseScores,
                                    std::vector<float> & pathTerminalPoseScores,
                                    const std::vector<sensor_msgs::LaserScan> & futureScans);

            /**
            * \brief Function for generating candidate trajectories through the current set of gaps
            * \param gaps incoming set of gaps through which we want to generate trajectories
            * \param generatedTrajs set of generated trajectories
            * \param pathPoseScores set of posewise scores for all paths
            * \param pathTerminalPoseScores set of terminal pose scores for all paths
            * \param futureScans set of propagated scans to use during scoring
            * \return Vector of pose-wise scores for the generated trajectories
            */
            void generateGapTrajsV2(std::vector<GapTube *> & gapTubes, 
                                    std::vector<Trajectory> & generatedTrajs,
                                    std::vector<std::vector<float>> & pathPoseScores,
                                    std::vector<float> & pathTerminalPoseScores,
                                    const std::vector<sensor_msgs::LaserScan> & futureScans);

            void gapGoalPlacementV2(std::vector<GapTube *> & gapTubes);

            void ungapGoalPlacement(const std::vector<Ungap *> & recedingUngaps);

            /**
            * \brief Function for selecting the best trajectory out of the set of recently generated trajectories
            * \param trajs set of recently generated trajectories
            * \param pathPoseScores vector of pose-wise scores for the generated trajectories
            * \param pathTerminalPoseScores set of terminal pose scores for all paths
            * \return index of the highest score trajectory
            */
            void pickTraj(int & trajFlag,
                            int & lowestCostTrajIdx,
                            const std::vector<Trajectory> & gapTrajs, 
                            const std::vector<std::vector<float>> & gapTrajPoseCosts, 
                            const std::vector<float> & gapTrajTerminalPoseCosts,
                            const std::vector<Trajectory> & ungapTrajs, 
                            const std::vector<std::vector<float>> & ungapTrajPoseCosts, 
                            const std::vector<float> & ungapTrajTerminalPoseCosts,                          
                            const std::vector<Trajectory> & idlingTrajs, 
                            const std::vector<std::vector<float>> & idlingTrajPoseCosts, 
                            const std::vector<float> & idlingTrajTerminalPoseCosts);

            /**
            * \brief Helper function for switching to a new trajectory
            * \param incomingGap incoming gap to switch to
            * \param incomingTraj incoming trajectory to switch to
            * \param switchToIncoming boolean for if planner is to switch to the incoming trajectory
            * \return trajectory that planner will switch to
            */
            Trajectory changeTrajectoryHelper(const Trajectory & incomingTraj, 
                                                const bool & switchToIncoming,
                                                const int & trajFlag,
                                                Gap * incomingGap);

            // Gap * incomingGap,
            /**
            * \brief Compare incoming highest scoring trajectory to the trajectory that the
            * robot is currently following to determine if robot should switch to the incoming trajectory
            * \param feasibleGaps set of feasible gaps which we are manipulating
            * \param trajs set of trajectories to compare against current trajectory
            * \param lowestCostTrajIdx index of lowest cost trajectory
            * \param trajFlag flag for if robot is idling or moving
            * \param isIncomingGapFeasible boolean for if the incoming gap is feasible 
            * \param futureScans set of propagated scans to use during scoring
            * \return the trajectory that the robot will track
            */
            Trajectory compareToCurrentTraj(const Trajectory & incomingTraj,
                                            // const std::vector<Gap *> & feasibleGaps, 
                                            // const std::vector<Trajectory> & trajs,
                                            // const int & lowestCostTrajIdx,
                                            // const bool & isIncomingGapFeasible,
                                            const std::vector<sensor_msgs::LaserScan> & futureScans,
                                            const int & trajFlag,
                                            Gap * incomingGap,
                                            const bool & isIncomingGapFeasible);

            /**
            * \brief Function for getting index of closest pose in trajectory
            * \param currTrajRbtFrame current trajectory in robot frame
            * \return index of closest pose in trajectory
            */
            int getClosestTrajectoryPoseIdx(const geometry_msgs::PoseArray & currTrajRbtFrame);

            /**
            * \brief Function for deep copying simplified gaps
            * \return Deep copied simplified gaps
            */
            std::vector<Gap *> deepCopyCurrentSimplifiedGaps();

            /**
            * \brief Function for deep copying raw gaps
            * \return Deep copied raw gaps
            */
            std::vector<Gap *> deepCopyCurrentRawGaps();

            /**
            * \brief Setter for current trajectory
            * \param currentTraj incoming trajectory that robot is going to start tracking
            */
            void setCurrentTraj(const Trajectory & currentTraj) { currentTraj_ = currentTraj; }

            void setCurrentTrajTrackingStartTime(const ros::Time & startTime) { currentTrajTrackingStartTime_ = startTime; }

            void setCurrentTrajLifespan(const ros::Duration & lifespan) { currentTrajLifespan_ = lifespan; }

            /**
            * \brief Getter for current trajectory
            * \return trajectory that robot is currently tracking
            */
            Trajectory getCurrentTraj() { return currentTraj_; }

            /**
            * \brief Setter for estimator of current gap's left point
            * \param leftModel estimator of current gap's left point
            */
            void setCurrentLeftGapPtModelID(Estimator * leftModel);

            /**
            * \brief Setter for estimator of current gap's left point
            * \param rightModel estimator of current gap's right point
            */            
            void setCurrentRightGapPtModelID(Estimator * rightModel);

            /**
            * \brief Getter for ID of model for current gap's left point
            * \return ID of model for current gap's left point
            */
            int getCurrentLeftGapPtModelID();

            /**
            * \brief Getter for ID of model for current gap's left point
            * \return ID of model for current gap's right point
            */            
            int getCurrentRightGapPtModelID();

            /**
            * \brief Helper function for pretty printing estimator states 
            * the left and right points of the current set of gaps
            * \param gaps current set of gaps
            */
            void printGapModels(const std::vector<Gap *> & gaps);
            
            /**
            * \brief Helper function for validating estimator states 
            * the left and right points of the current set of gaps
            * \param gaps current set of gaps
            */
            void checkGapModels(const std::vector<Gap *> & gaps);

            boost::mutex gapMutex_; /**< Current set of gaps mutex */
            DynamicGapConfig cfg_; /**< Planner hyperparameter config list */

            ros::NodeHandle nh_; /**< ROS node handle for local path planner */

            ros::Publisher mpcInputPublisher_; /**< ROS publisher for mpc input terms */
            ros::Subscriber mpcOutputSubscriber_; /**< ROS subscriber for mpc output */

            ros::Publisher containedRawGapPointsMarkerPublisher_;
            ros::Publisher gapFeatureObservationPublisher_;

            ros::Subscriber tfSub_; /**< Subscriber to TF tree */
            ros::Subscriber laserSub_; /**< Subscriber to incoming laser scan */

            std::vector<ros::Subscriber> agentPoseSubs_; /**< Subscribers for agent poses */

            message_filters::Subscriber<nav_msgs::Odometry> rbtPoseSub_; /**< Subscriber to incoming robot pose */
            message_filters::Subscriber<geometry_msgs::TwistStamped> rbtAccSub_; /**< Subscriber to incoming robot acceleration */

            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::TwistStamped> rbtPoseAndAccSyncPolicy; /**< Custom synchronization policy for robot pose and acceleration messages */
            typedef message_filters::Synchronizer<rbtPoseAndAccSyncPolicy> CustomSynchronizer; /**< Custom synchronizer for robot pose and acceleration messages */
            boost::shared_ptr<CustomSynchronizer> sync_; /**< Shared pointer to custom synchronizer */

            ros::Subscriber pedOdomSub_; /**< Subscriber to incoming robot acceleration */

            geometry_msgs::TransformStamped map2rbt_; /**< Transformation from map frame to robot frame */
            geometry_msgs::TransformStamped odom2rbt_; /**< Transformation from odometry frame to robot frame */
            geometry_msgs::TransformStamped rbt2odom_; /**< Transformation from robot frame to odometry frame */
            geometry_msgs::TransformStamped map2odom_; /**< Transformation from map frame to odometry frame */
            // geometry_msgs::TransformStamped cam2odom_; /**< Transformation from camera frame to odometry frame */
            // geometry_msgs::TransformStamped rbt2cam_; /**< Transformation from robot frame to camera frame */

            geometry_msgs::PoseStamped rbtPoseInRbtFrame_; /**< Robot pose in robot frame */
            geometry_msgs::PoseStamped rbtPoseInSensorFrame_; /**< Robot pose in sensor frame */
            geometry_msgs::PoseStamped rbtPoseInOdomFrame_; /**< Robot pose in odometry frame */

            tf2_ros::Buffer tfBuffer_; /**< ROS transform buffer */
            tf2_ros::TransformListener * tfListener_ = NULL; /**< ROS transform listener */

            std::vector<int> rawAssocation_; /**< Association vector for current set of raw gaps */
            std::vector<int> simpAssociation_; /**< Association vector for current set of simplified gaps */
            std::vector<std::vector<float>> rawDistMatrix_; /**< Distance matrix for current set of raw gaps */
            std::vector<std::vector<float>> simpDistMatrix_; /**< Distance matrix for current set of raw gaps */

            // Goals and stuff
            geometry_msgs::PoseStamped globalGoalOdomFrame_; /**< Global goal in odometry frame */
            geometry_msgs::PoseStamped globalGoalRobotFrame_; /**< Global goal in robot frame */
            geometry_msgs::PoseStamped globalPathLocalWaypointOdomFrame_; /**< Global path local waypoint in odometry frame */

            // Gaps
            std::vector<Gap *> currRawGaps_; /**< Current set of raw gaps */
            std::vector<Gap *> currSimplifiedGaps_; /**< Current set of simplified gaps */
            std::vector<Gap *> prevRawGaps_; /**< Previous set of raw gaps */
            std::vector<Gap *> prevSimplifiedGaps_; /**< Previous set of simplified gaps */

            int currentLeftGapPtModelID = -1; /**< Model ID for estimator of current gap's left point */
            int currentRightGapPtModelID = -1; /**< Model ID for estimator of current gap's right point */

            // Eigen::Vector4f currentLeftGapPtState; /**< State for estimator of current gap's left point */
            // Eigen::Vector4f currentRightGapPtState; /**< State for estimator of current gap's right point */

            // float currentInterceptTime_ = 0.0; /**< Intercept time for current gap */
            // float currentMinSafeDist_ = 0.0; /**< Minimum safe distance for current gap */

            // bool publishToMpc_ = false; /**< Flag for publishing trajectory to trigger MPC */

            // std::chrono::steady_clock::time_point prevTrajSwitchTime_; /**< Time step of previous trajectory switch */
            // std::chrono::steady_clock::time_point prevIdlingTime_; 
            // int prevTrajFlag_ = -1; /**< Previous trajectory flag */

            int currentModelIdx_ = 0; /**< Counter for instantiated models throughout planner's existence */

            ros::Time tPreviousModelUpdate_; /**< ROS time step of previous gap point model update */

            // Trajectories
            Trajectory currentTraj_; /**< Trajectory that robot is currently tracking */
            ros::Time currentTrajTrackingStartTime_; /**< Time step of when robot started tracking current trajectory */
            ros::Duration currentTrajLifespan_; /**< Duration of time that robot has been tracking current trajectory */

            int targetTrajectoryPoseIdx_ = 0; /**< Index of closest pose along the robot's current trajectory */

            int trajectoryChangeCount_ = 0; /**< Counter for times that planner has switched local trajectories */

            // Scans
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */

            // geometry_msgs::Twist mpcTwist_; /**< Command velocity output for MPC */

            bool haveTFs_ = false; /**< Flag for if transforms have been received */

            std::map<std::string, geometry_msgs::Pose> currentTrueAgentPoses_; /**< Ground truth poses of agents currently in local environment */
            std::map<std::string, geometry_msgs::Vector3Stamped> currentTrueAgentVels_; /**< Ground truth velocities of agents currently in local environment */

            GapDetector * gapDetector_ = NULL; /**< Gap detector */
            GapVisualizer * gapVisualizer_ = NULL; /**< Gap visualizer */
            AgentVisualizer * agentVisualizer_ = NULL; /**< Pedestrian visualizer */
            GlobalPlanManager * globalPlanManager_ = NULL; /**< Goal selector */
            TrajectoryVisualizer * trajVisualizer_ = NULL; /**< Trajectory visualizer */
            GoalVisualizer * goalVisualizer_ = NULL; /**< Goal visualizer */
            TrajectoryEvaluator * trajEvaluator_ = NULL; /**< Trajectory scorer */
            DynamicScanPropagator * dynamicScanPropagator_ = NULL; /**< Dynamic scan propagator */
            GapGoalPlacer * gapGoalPlacer_ = NULL; /**< Gap goal placer */
            GapTrajectoryGenerator * gapTrajGenerator_ = NULL; /**< Gap trajectory generator */
            UngapTrajectoryGenerator * ungapTrajGenerator_ = NULL; /**< Ungap trajectory generator */
            GapManipulator * gapManipulator_ = NULL; /**< Gap manipulator */
            TrajectoryController * trajController_ = NULL; /**< Trajectory controller */
            GapPointAssociator * gapPointAssociator_ = NULL; /**< Gap associator */
            GapFeasibilityChecker * gapFeasibilityChecker_ = NULL; /**< Gap feasibility checker */
            GapPropagator * gapPropagator_ = NULL; /**< Gap propagator */

            UngapFeasibilityChecker * ungapFeasibilityChecker_ = NULL; /**< Ungap feasibility checker */

            TimeKeeper * timeKeeper_ = NULL; /**< Time keeper */

            // Status
            bool hasGlobalGoal_ = false; /**< Indicator for if planner's global goal has been set */
            bool initialized_ = false; /**< Indicator for if planner has been initialized */
            bool hasLaserScan_ = false; /**< Indicator for if planner has read in a laser scan */
            bool reachedGlobalGoal_ = false; /**< Flag for if global goal has been reached */
            bool colliding_ = false; /**< Flag for if robot is currently in collision */

            // Velocities
            boost::circular_buffer<float> cmdVelBuffer_; /**< Buffer of prior command velocities */

            geometry_msgs::TwistStamped currentRbtVel_; /**< Current robot velocity */
            geometry_msgs::TwistStamped currentRbtAcc_; /**< Current robot acceleration */

            std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_; /**< Intermediate robot velocities between last model update and upcoming model update */
            std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs_; /**< Intermediate robot accelerations between last model update and upcoming model update */

            float ungapRbtSpeed_ = 0.0; /**< Speed of robot when traversing through ungap */
                    
                        /**
             * \brief CSV file stream for logging simplified gap velocity training labels.
             */
            std::ofstream gapVelocityCsvFile_;

            /**
             * \brief Enables/disables simplified gap velocity CSV logging.
             */
            bool gapVelocityCsvLoggingEnabled_ = true;

            /**
             * \brief Full path to the active gap velocity CSV file.
             */
            std::string gapVelocityCsvPath_;

            /**
             * \brief Unique file/session name generated from wall-clock time.
             */
            std::string gapVelocityCsvSessionId_;

            /**
             * \brief Monotonic row counter used to preserve sequence order without storing ROS time.
             */
            unsigned long long gapVelocityCsvSampleIdx_ = 0;

            /**
             * \brief Initializes the CSV logger for simplified gap velocity labels.
             */
            void initializeGapVelocityCsvLogger();

            /**
             * \brief Writes one simplified gap velocity training row to CSV.
             * \param gap_index Index of the gap in the current simplified gap vector.
             * \param model_id Persistent model ID for the gap point.
             * \param side Gap side, usually "left" or "right".
             * \param gap_x Gap point x-position in robot frame.
             * \param gap_y Gap point y-position in robot frame.
             * \param kalman_vx Velocity x from the estimator/Kalman state.
             * \param kalman_vy Velocity y from the estimator/Kalman state.
             * \param perfect_rel_vx Perfect relative velocity x label.
             * \param perfect_rel_vy Perfect relative velocity y label.
             * \param perfect_world_robot_vx Matched agent/world velocity x expressed in robot frame.
             * \param perfect_world_robot_vy Matched agent/world velocity y expressed in robot frame.
             * \param matched_agent_id ID of the closest matched agent, empty if none.
             * \param match_dist Distance from gap point to matched agent.
             * \param matched_dynamic_agent True if the gap point matched a dynamic agent.
             */
               void logSimplifiedGapVelocityCsvRow(
                    const int& gap_index,
                    const int& model_id,
                    const std::string& side,
                    const float& gap_x,
                    const float& gap_y,
                    const float& kalman_vx,
                    const float& kalman_vy,
                    const float& perfect_rel_vx,
                    const float& perfect_rel_vy,
                    const float& perfect_world_robot_vx,
                    const float& perfect_world_robot_vy,
                    const std::string& matched_agent_id,
                    const float& match_dist,
                    const bool& matched_dynamic_agent,
                    const int& contained_raw_gap_point_count,
                    const std::string& contained_raw_gap_points,
                    const float& sector_radius,
                    const float& sector_angle_rad,
                    const float& sector_area,
                    const int& sector_dynamic_raw_gap_point_count,
                    const float& sector_density,
                    const int& gt_sector_dynamic_raw_gap_point_count,
                    const float& gt_sector_density);

                bool scanIdxInsideGapRange(
                const int& idx,
                const int& rightIdx,
                const int& leftIdx) const;

                void publishContainedRawGapPointsMarkerArray(
                const std::vector<Gap*>& simplifiedGaps,
                const std::vector<Gap*>& rawGaps,
                const ros::Time& stamp) const;

                std::string serializeRawGapPointsInsideSimplifiedGap(
                const int& simplifiedGapIndex,
                int& containedRawGapPointCount) const;
    };
}