#pragma once

#include <dynamic_gap/Planner.h>

namespace dynamic_gap
{
    /**
    * \brief Test helper with access to Planner's internal state (declared friend in Planner.h).
    */
    class PlannerTestAccess
    {
        public:
            static void setTimeKeeper(Planner & planner, TimeKeeper * timeKeeper) { planner.timeKeeper_ = timeKeeper; }
            static void setInitialized(Planner & planner, bool value) { planner.initialized_ = value; }
            static void setHasLaserScan(Planner & planner, bool value) { planner.hasLaserScan_ = value; }
            static void setHasGlobalGoal(Planner & planner, bool value) { planner.hasGlobalGoal_ = value; }
            static void setColliding(Planner & planner, bool value) { planner.colliding_ = value; }
            static bool colliding(Planner & planner) { return planner.colliding_; }
            static void setManualMode(Planner & planner, bool value) { planner.manualSelectionMode_ = value; }
            static bool manualMode(Planner & planner) { return planner.manualSelectionMode_; }
            static void setHaveTFs(Planner & planner, bool value) { planner.haveTFs_ = value; }
            static void setRobotFrame(Planner & planner, const std::string & frame) { planner.cfg_.robot_frame_id = frame; }
            static void setRobotInscribedRadius(Planner & planner, float r) { planner.cfg_.rbt.r_inscr = r; }
            static void setGapDetector(Planner & planner, GapDetector * detector) { planner.gapDetector_ = detector; }
            static void setAgentVisualizer(Planner & planner, AgentVisualizer * visualizer) { planner.agentVisualizer_ = visualizer; }
            static GapDetector * makeGapDetector(Planner & planner) { return new GapDetector(planner.cfg_); }
            static AgentVisualizer * makeAgentVisualizer(Planner & planner) { return new AgentVisualizer(planner.nh_, planner.cfg_); }
            static tf2_ros::Buffer & tfBuffer(Planner & planner) { return planner.tfBuffer_; }

            static boost::shared_ptr<sensor_msgs::LaserScan const> scan(Planner & planner) { return planner.scan_; }
            static void laserScanCB(Planner & planner, boost::shared_ptr<sensor_msgs::LaserScan> scan) { planner.laserScanCB(scan); }

            static void pedOdomCB(Planner & planner, const pedsim_msgs::AgentStatesConstPtr & msg) { planner.pedOdomCB(msg); }
            static void scenarioResetCB(Planner & planner, const std_msgs::Int16::ConstPtr & msg) { planner.scenarioResetCB(msg); }
            static const std::map<std::string, geometry_msgs::Pose> & agentPoses(Planner & planner) { return planner.currentTrueAgentPoses_; }
            static const std::map<std::string, geometry_msgs::Vector3Stamped> & agentVels(Planner & planner) { return planner.currentTrueAgentVels_; }

            static void trajCreationPointCB(Planner & planner, const geometry_msgs::PointStamped::ConstPtr & msg) { planner.trajCreationPointCB(msg); }
            static bool hasTrajCreationPoint(Planner & planner) { return planner.hasTrajCreationPoint_; }
            static void setTrackingCreatedTraj(Planner & planner, bool value) { planner.isTrackingCreatedTraj_ = value; }
            static bool trackingCreatedTraj(Planner & planner) { return planner.isTrackingCreatedTraj_; }

            static void cursorCB(Planner & planner, const manual_gap_msgs::CursorState::ConstPtr & msg) { planner.cursorCB(msg); }
            static void setCursorRxWall(Planner & planner, const ros::WallTime & t) { planner.cursorRxWall_ = t; }
            static bool hasCursorPos(Planner & planner) { return planner.hasCursorPos_; }
            static geometry_msgs::Point cursorOdom(Planner & planner) { return planner.cursorPosOdomFrame_.point; }
            static bool cursorUsable(Planner & planner) { return planner.isCursorUsable(); }
            static void setOdomFrame(Planner & planner, const std::string & frame) { planner.cfg_.odom_frame_id = frame; }
            static void setHasTrajCreationPoint(Planner & planner, bool value) { planner.hasTrajCreationPoint_ = value; }
            static int episodeId(Planner & planner) { return planner.episodeId_; }
            static void addGaps(Planner & planner, int count)
            {
                for (int i = 0; i < count; i++)
                {
                    Gap * gap = new Gap("base_link", 10, 1.0f, 20, 1.0f, false);
                    planner.prevRawGaps_.push_back(gap);
                    planner.currRawGaps_.push_back(gap);
                    Gap * simp = new Gap("base_link", 10, 1.0f, 20, 1.0f, false);
                    planner.prevSimplifiedGaps_.push_back(simp);
                    planner.currSimplifiedGaps_.push_back(simp);
                }
            }
            static size_t gapCount(Planner & planner) { return planner.currRawGaps_.size() + planner.currSimplifiedGaps_.size() + planner.prevRawGaps_.size() + planner.prevSimplifiedGaps_.size(); }
            static bool holdStill(Planner & planner) { return planner.manualHoldStill_; }
            static void setHoldStill(Planner & planner, bool value) { planner.manualHoldStill_ = value; }
            static void setTrajVisualizer(Planner & planner) { planner.trajVisualizer_ = new TrajectoryVisualizer(planner.nh_, planner.cfg_); }
            static int selectManualTrajectory(Planner & planner, int & trajFlag, const std::vector<GapTube *> & gapTubes,
                                              const std::vector<Trajectory> & gapTrajs, manual_gap_msgs::ManualGapSample & sample,
                                              Trajectory & chosenTraj)
            { return planner.selectManualTrajectory(trajFlag, gapTubes, gapTrajs, {}, {}, sample, chosenTraj); }
            static int selectManualTrajectory(Planner & planner, int & trajFlag, const std::vector<GapTube *> & gapTubes,
                                              const std::vector<Trajectory> & gapTrajs,
                                              const std::vector<std::vector<float>> & poseCosts,
                                              const std::vector<float> & terminalCosts,
                                              manual_gap_msgs::ManualGapSample & sample, Trajectory & chosenTraj)
            { return planner.selectManualTrajectory(trajFlag, gapTubes, gapTrajs, poseCosts, terminalCosts, sample, chosenTraj); }
            static void pickTraj(Planner & planner, int & trajFlag, int & trajIdx, const std::vector<Trajectory> & gapTrajs,
                                 const std::vector<std::vector<float>> & poseCosts, const std::vector<float> & terminalCosts)
            { planner.pickTraj(trajFlag, trajIdx, gapTrajs, poseCosts, terminalCosts, {}, {}, {}, {}, {}, {}); }
            static Trajectory currentTraj(Planner & planner) { return planner.currentTraj_; }
            static ros::Duration currentLifespan(Planner & planner) { return planner.currentTrajLifespan_; }
            static int trajectoryChangeCount(Planner & planner) { return planner.trajectoryChangeCount_; }
            static void setCurrentTraj(Planner & planner, const Trajectory & traj) { planner.currentTraj_ = traj; }

            static void setLearnedMode(Planner & planner, bool value) { planner.learnedSelectionMode_ = value; }
            static bool loadLearnedScorer(Planner & planner, const std::string & path)
            { return planner.learnedGapScorer_.load(path, planner.learnedGapScorerError_); }
            static const LearnedGapScorer & learnedScorer(Planner & planner) { return planner.learnedGapScorer_; }
            static int selectLearnedTrajectory(Planner & planner, int & trajFlag, const std::vector<GapTube *> & gapTubes,
                                               const std::vector<Trajectory> & gapTrajs,
                                               const std::vector<std::vector<float>> & poseCosts,
                                               const std::vector<float> & terminalCosts,
                                               manual_gap_msgs::ManualGapSample & sample, Trajectory & chosenTraj)
            { return planner.selectLearnedTrajectory(trajFlag, gapTubes, gapTrajs, poseCosts, terminalCosts, sample, chosenTraj); }

            static void fillSampleEdges(Planner & planner, const std::vector<Gap *> & rawGaps,
                                        manual_gap_msgs::ManualGapSample & sample)
            { planner.fillSampleEdges(rawGaps, sample); }
    };
}
