#pragma once

#include <chrono>
#include <map>

#include <ros/ros.h>
#include <ros/console.h>

namespace dynamic_gap
{

    enum planningStepIdxs { GAP_DET = 0, 
                            GAP_SIMP = 1, 
                            GAP_ASSOC = 2, 
                            GAP_EST = 3,
                            SCAN = 4,
                            GAP_PROP = 5,
                            GAP_MANIP = 6,
                            GAP_FEAS = 7,
                            SCAN_PROP = 8,
                            UNGAP_TRAJ_GEN = 9,
                            GAP_TRAJ_GEN = 10,
                            IDLING_TRAJ_GEN = 11,
                            TRAJ_PICK = 12,
                            TRAJ_COMP = 13,
                            PLAN = 14,
                            FEEBDACK = 15,
                            PO = 16,
                            CONTROL = 17
                            };

    const std::map<int, std::string> planningSteps = { {GAP_DET, "Gap Detection"},
                                                    {GAP_SIMP, "Gap Simplification"},  
                                                    {GAP_ASSOC, "Gap Association"},
                                                    {GAP_EST, "Gap Estimation"},
                                                    {SCAN, "Scan Loop"},
                                                    {GAP_PROP, "Gap Propagation"},
                                                    {GAP_MANIP, "Gap Manipulation"},
                                                    {GAP_FEAS, "Gap Feasibility Check"},
                                                    {SCAN_PROP, "Scan Propagation"},
                                                    {UNGAP_TRAJ_GEN, "Ungap Trajectory Generation"},
                                                    {GAP_TRAJ_GEN, "Gap Trajectory Generation"},
                                                    {IDLING_TRAJ_GEN, "Idling Trajectory Generation"},
                                                    {TRAJ_PICK, "Trajectory Picking"},
                                                    {TRAJ_COMP, "Trajectory Comparison"},
                                                    {PLAN, "Planning Loop"},
                                                    {FEEBDACK, "Feedback Control"},
                                                    {PO, "Projection Operator"},
                                                    {CONTROL, "Control Loop"}
                                                };


    /**
    * \brief Class responsible for time keeping.
    */
    class TimeKeeper
    {
        public:
            void startTimer(const int & planningStepIdx);
            void stopTimer(const int & planningStepIdx);

            int getPlanningLoopCalls();

        private:
            /**
            * \brief Helper function for computing average computation times for planning
            * \param timeTaken time taken for particular step
            * \param planningStepIdx index for particular step
            */
            float computeAverageTimeTaken(const float & currTimeTaken, const int & planningStepIdx);

            /**
            * \brief Helper function for computing average number of gaps planned over per planning loop
            * \param numGaps number of gaps planned over per planning loop
            */
            float computeAverageNumberGaps(const int & numGaps);

            /**
            * \brief Calculate time that has elapsed in seconds since given start time
            * \param startTime start time
            * \return elapsed time in seconds
            */
            inline float timeTaken(const std::chrono::steady_clock::time_point & startTime,
                                    const std::chrono::steady_clock::time_point & stopTime);

            // Timekeeping
            std::chrono::steady_clock::time_point gapDetectionStartTime; /**< Start time for gap detection */
            std::chrono::steady_clock::time_point gapDetectionEndTime; /**< End time for gap detection */
            float totalGapDetectionTimeTaken = 0.0f; /**< Total time taken for gap detection */
            int gapDetectionCalls = 0; /**< Total number of calls for gap detection */

            std::chrono::steady_clock::time_point gapSimplificationStartTime; /**< Start time for gap simplification */
            std::chrono::steady_clock::time_point gapSimplificationEndTime; /**< End time for gap simplification */
            float totalGapSimplificationTimeTaken = 0.0f; /**< Total time taken for gap simplification */
            int gapSimplificationCalls = 0; /**< Total number of calls for gap simplification */

            std::chrono::steady_clock::time_point gapAssociationCheckStartTime; /**< Start time for gap association */
            std::chrono::steady_clock::time_point gapAssociationCheckEndTime; /**< End time for gap association */
            float totalGapAssociationCheckTimeTaken = 0.0f; /**< Total time taken for gap association */
            int gapAssociationCheckCalls = 0; /**< Total number of calls for gap association */

            std::chrono::steady_clock::time_point gapEstimationStartTime; /**< Start time for gap estimation */
            std::chrono::steady_clock::time_point gapEstimationEndTime; /**< End time for gap estimation */
            float totalGapEstimationTimeTaken = 0.0f; /**< Total time taken for gap estimation */
            int gapEstimationCalls = 0; /**< Total number of calls for gap estimation */

            std::chrono::steady_clock::time_point scanningStartTime; /**< Start time for scan loop */
            std::chrono::steady_clock::time_point scanningEndTime; /**< End time for scan loop */
            float totalScanningTimeTaken = 0.0f; /**< Total time taken for scan loop */
            int scanningLoopCalls = 0; /**< Total number of calls for scan loop */

            std::chrono::steady_clock::time_point gapPropagationStartTime; /**< Start time for gap propagation */
            std::chrono::steady_clock::time_point gapPropagationEndTime; /**< End time for gap propagation */
            float totalGapPropagationTimeTaken = 0.0f; /**< Total time taken for gap propagation */
            int gapPropagationCalls = 0; /**< Total number of calls for gap propagation */

            std::chrono::steady_clock::time_point gapManipulationStartTime; /**< Start time for gap manipulation */
            std::chrono::steady_clock::time_point gapManipulationEndTime; /**< End time for gap manipulation */
            float totalGapManipulationTimeTaken = 0.0f; /**< Total time taken for gap manipulation */
            int gapManipulationCalls = 0; /**< Total number of calls for gap manipulation */

            std::chrono::steady_clock::time_point gapFeasibilityCheckStartTime; /**< Start time for gap feasibility analysis */
            std::chrono::steady_clock::time_point gapFeasibilityCheckEndTime; /**< End time for gap feasibility analysis */
            float totalGapFeasibilityCheckTimeTaken = 0.0f; /**< Total time taken for gap feasibility analysis */
            int gapFeasibilityCheckCalls = 0; /**< Total number of calls for gap feasibility analysis */

            std::chrono::steady_clock::time_point scanPropagationStartTime; /**< Start time for scan propagation */
            std::chrono::steady_clock::time_point scanPropagationEndTime; /**< End time for scan propagation */
            float totalScanPropagationTimeTaken = 0.0f; /**< Total time taken for scan propagation */
            int scanPropagationCalls = 0; /**< Total number of calls for scan propagation */

            std::chrono::steady_clock::time_point ungapTrajectoryGenerationStartTime; /**< Start time for un-gap trajectory generation */
            std::chrono::steady_clock::time_point ungapTrajectoryGenerationEndTime; /**< End time for un-gap trajectory generation */
            float totalGenerateUngapTrajTimeTaken = 0.0f; /**< Total time taken for un-gap trajectory synthesis */
            int generateUngapTrajCalls = 0; /**< Total number of calls for un-gap trajetory synthesis */

            std::chrono::steady_clock::time_point gapTrajectoryGenerationStartTime; /**< Start time for gap trajectory generation */
            std::chrono::steady_clock::time_point gapTrajectoryGenerationEndTime; /**< End time for gap trajectory generation */
            float totalGenerateGapTrajTimeTaken = 0.0f; /**< Total time taken for gap trajectory synthesis */
            int generateGapTrajCalls = 0; /**< Total number of calls for gap trajetory synthesis */

            std::chrono::steady_clock::time_point idlingTrajectoryGenerationStartTime; /**< Start time for idling trajectory generation */
            std::chrono::steady_clock::time_point idlingTrajectoryGenerationEndTime; /**< End time for idling trajectory generation */
            float totalGenerateIdlingTrajTimeTaken = 0.0f; /**< Total time taken for idling trajectory synthesis */
            int generateIdlingTrajCalls = 0; /**< Total number of calls for idling trajetory synthesis */

            std::chrono::steady_clock::time_point selectGapTrajectoryStartTime; /**< Start time for trajectory selection */
            std::chrono::steady_clock::time_point selectGapTrajectoryEndTime; /**< End time for trajectory selection */
            float totalSelectGapTrajTimeTaken = 0.0f; /**< Total time taken for trajectory selection */
            int selectGapTrajCalls = 0; /**< Total number of calls for trajectory selection */

            std::chrono::steady_clock::time_point compareToCurrentTrajectoryStartTime; /**< Start time for trajectory comparison */
            std::chrono::steady_clock::time_point compareToCurrentTrajectoryEndTime; /**< End time for trajectory comparison */
            float totalCompareToCurrentTrajTimeTaken = 0.0f; /**< Total time taken for trajectory comparison */
            int compareToCurrentTrajCalls = 0; /**< Total number of calls for trajectory comparison */

            std::chrono::steady_clock::time_point totalPlanningStartTime; /**< Start time for planning loop */
            std::chrono::steady_clock::time_point totalPlanningEndTime; /**< End time for planning loop */
            float totalPlanningTimeTaken = 0.0f; /**< Total time taken for planning loop */
            int planningLoopCalls = 0; /**< Total number of calls for planning loop */

            std::chrono::steady_clock::time_point feedbackControlStartTime; /**< Start time for feedback control */
            std::chrono::steady_clock::time_point feedbackControlEndTime; /**< End time for feedback control */
            float totalFeedbackControlTimeTaken = 0.0f; /**< Total time taken for feedback control */
            int feedbackControlCalls = 0; /**< Total number of calls for feedback control */

            std::chrono::steady_clock::time_point projectionOperatorStartTime; /**< Start time for projection operator */
            std::chrono::steady_clock::time_point projectionOperatorEndTime; /**< End time for projection operator */
            float totalProjectionOperatorTimeTaken = 0.0f; /**< Total time taken for projection operator */
            int projectionOperatorCalls = 0; /**< Total number of calls for projection operator */

            std::chrono::steady_clock::time_point controlLoopStartTime; /**< Start time for control loop */
            std::chrono::steady_clock::time_point controlLoopEndTime; /**< End time for control loop */
            float totalControlTimeTaken = 0.0f; /**< Total time taken for control loop */
            int controlCalls = 0; /**< Total number of calls for control loop */

            int totalNumGaps = 0; /**< Total number of gaps planned over during deployment */
    };
}