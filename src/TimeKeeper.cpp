#include <dynamic_gap/TimeKeeper.h>

namespace dynamic_gap
{
    int TimeKeeper::getPlanningLoopCalls()
    {
        return planningLoopCalls;
    }

    float TimeKeeper::timeTaken(const std::chrono::steady_clock::time_point & startTime,
                                const std::chrono::steady_clock::time_point & stopTime)
    {
        float timeTakenInMilliseconds = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime).count();
        float timeTakenInSeconds = timeTakenInMilliseconds * 1.0e-6;
        return timeTakenInSeconds;
    }

    // Run after computeAverageTimeTaken, we are using planningLoopCalls
    float TimeKeeper::computeAverageNumberGaps(const int & gapCount)
    {
        totalNumGaps += gapCount;
        float avgNumberGaps = (totalNumGaps / (float) planningLoopCalls);
        ROS_INFO_STREAM_NAMED("Timing", "      [Planning Loop average number of gaps: " << avgNumberGaps << "]");

        return avgNumberGaps;
    }

    void TimeKeeper::startTimer(const int & planningStepIdx)
    {
        switch (planningStepIdx)
        {
            case GAP_DET:
                gapDetectionStartTime = std::chrono::steady_clock::now();
                break;
            case GAP_SIMP:
                gapSimplificationStartTime = std::chrono::steady_clock::now();
                break;
            case GAP_ASSOC:
                gapAssociationCheckStartTime = std::chrono::steady_clock::now();
                break;
            case GAP_EST:
                gapEstimationStartTime = std::chrono::steady_clock::now();
                break;
            case SCAN:
                scanningStartTime = std::chrono::steady_clock::now();
                break;
            case GAP_PROP:
                gapPropagationStartTime = std::chrono::steady_clock::now();
                break;
            case GAP_MANIP:
                gapManipulationStartTime = std::chrono::steady_clock::now();
                break;
            case GAP_FEAS:
                gapFeasibilityCheckStartTime = std::chrono::steady_clock::now();
                break;
            case SCAN_PROP:
                scanPropagationStartTime = std::chrono::steady_clock::now();
                break;
            case UNGAP_TRAJ_GEN:
                ungapTrajectoryGenerationStartTime = std::chrono::steady_clock::now();
                break;
            case GAP_TRAJ_GEN:
                gapTrajectoryGenerationStartTime = std::chrono::steady_clock::now();
                break;
            case IDLING_TRAJ_GEN:
                idlingTrajectoryGenerationStartTime = std::chrono::steady_clock::now();
                break;
            case TRAJ_PICK:
                selectGapTrajectoryStartTime = std::chrono::steady_clock::now();
                break;
            case TRAJ_COMP:
                compareToCurrentTrajectoryStartTime = std::chrono::steady_clock::now();
                break;
            case PLAN:
                totalPlanningStartTime = std::chrono::steady_clock::now();
                break;
            case FEEBDACK:
                feedbackControlStartTime = std::chrono::steady_clock::now();
                break;
            case PO:
                projectionOperatorStartTime = std::chrono::steady_clock::now();
                break;
            case CONTROL:
                controlLoopStartTime = std::chrono::steady_clock::now();
                break;
        }
    }

    void TimeKeeper::stopTimer(const int & planningStepIdx)
    {
        float currTimeTaken = 0.0f;
        float avgTimeTaken = 0.0f;
        switch (planningStepIdx)
        {
            case GAP_DET:
                gapDetectionEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(gapDetectionStartTime, gapDetectionEndTime);  
                break;
            case GAP_SIMP:
                gapSimplificationEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(gapSimplificationStartTime, gapSimplificationEndTime);  
                break;
            case GAP_ASSOC:
                gapAssociationCheckEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(gapAssociationCheckStartTime, gapAssociationCheckEndTime);  
                break;
            case GAP_EST:
                gapEstimationEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(gapEstimationStartTime, gapEstimationEndTime);  
                break;
            case SCAN:
                scanningEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(scanningStartTime, scanningEndTime);  
                break;
            case GAP_PROP:
                gapPropagationEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(gapPropagationStartTime, gapPropagationEndTime);  
                break;
            case GAP_MANIP:
                gapManipulationEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(gapManipulationStartTime, gapManipulationEndTime);  
                break;
            case GAP_FEAS:
                gapFeasibilityCheckEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(gapFeasibilityCheckStartTime, gapFeasibilityCheckEndTime);  
                break;
            case SCAN_PROP:
                scanPropagationEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(scanPropagationStartTime, scanPropagationEndTime);  
                break;
            case UNGAP_TRAJ_GEN:
                ungapTrajectoryGenerationEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(ungapTrajectoryGenerationStartTime, ungapTrajectoryGenerationEndTime);  
                break;
            case GAP_TRAJ_GEN:
                gapTrajectoryGenerationEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(gapTrajectoryGenerationStartTime, gapTrajectoryGenerationEndTime);  
                break;
            case IDLING_TRAJ_GEN:
                idlingTrajectoryGenerationEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(idlingTrajectoryGenerationStartTime, idlingTrajectoryGenerationEndTime);  
                break;
            case TRAJ_PICK:
                selectGapTrajectoryEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(selectGapTrajectoryStartTime, selectGapTrajectoryEndTime);  
                break;
            case TRAJ_COMP:
                compareToCurrentTrajectoryEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(compareToCurrentTrajectoryStartTime, compareToCurrentTrajectoryEndTime);  
                break;
            case PLAN:
                totalPlanningEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(totalPlanningStartTime, totalPlanningEndTime);  
                break;
            case FEEBDACK:
                feedbackControlEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(feedbackControlStartTime, feedbackControlEndTime);  
                break;
            case PO:
                projectionOperatorEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(projectionOperatorStartTime, projectionOperatorEndTime);  
                break;
            case CONTROL:
                controlLoopEndTime = std::chrono::steady_clock::now();
                currTimeTaken = timeTaken(controlLoopStartTime, controlLoopEndTime);  
                break;
        }

        avgTimeTaken = computeAverageTimeTaken(currTimeTaken, planningStepIdx);     
        ROS_INFO_STREAM_NAMED("Timing", "      [" << planningSteps.at(planningStepIdx) << " took " << currTimeTaken << " seconds]");
        ROS_INFO_STREAM_NAMED("Timing", "      [" << planningSteps.at(planningStepIdx) << " average time: " << avgTimeTaken << " seconds (" << (1.0 / avgTimeTaken) << " Hz) ]");
    }

    float TimeKeeper::computeAverageTimeTaken(const float & currTimeTaken, const int & planningStepIdx)
    {
        float averageTimeTaken = 0.0f;
        switch (planningStepIdx)
        {
            case GAP_DET:
                totalGapDetectionTimeTaken += currTimeTaken;
                gapDetectionCalls++;
                averageTimeTaken = (totalGapDetectionTimeTaken / gapDetectionCalls);
                break;
            case GAP_SIMP:
                totalGapSimplificationTimeTaken += currTimeTaken;
                gapSimplificationCalls++;
                averageTimeTaken = (totalGapSimplificationTimeTaken / gapSimplificationCalls);
                break;
            case GAP_ASSOC:
                totalGapAssociationCheckTimeTaken += currTimeTaken;
                gapAssociationCheckCalls++;
                averageTimeTaken = (totalGapAssociationCheckTimeTaken / gapAssociationCheckCalls);
                break;                
            case GAP_EST:
                totalGapEstimationTimeTaken += currTimeTaken;
                gapEstimationCalls++;
                averageTimeTaken = (totalGapEstimationTimeTaken / gapEstimationCalls);
                break;      
            case SCAN:
                totalScanningTimeTaken += currTimeTaken;
                scanningLoopCalls++;
                averageTimeTaken = (totalScanningTimeTaken / scanningLoopCalls);
                break;   
            case GAP_PROP:
                totalGapPropagationTimeTaken += currTimeTaken;
                gapPropagationCalls++;
                averageTimeTaken = (totalGapPropagationTimeTaken / gapPropagationCalls);
                break;                   
            case GAP_MANIP:
                totalGapManipulationTimeTaken += currTimeTaken;
                gapManipulationCalls++;
                averageTimeTaken = (totalGapManipulationTimeTaken / gapManipulationCalls);
                break;                                                 
            case GAP_FEAS:
                totalGapFeasibilityCheckTimeTaken += currTimeTaken;
                gapFeasibilityCheckCalls++;
                averageTimeTaken = (totalGapFeasibilityCheckTimeTaken / gapFeasibilityCheckCalls);
                break;   
            case SCAN_PROP:
                totalScanPropagationTimeTaken += currTimeTaken;
                scanPropagationCalls++;
                averageTimeTaken = (totalScanPropagationTimeTaken / scanPropagationCalls);
                break;    
            case GAP_TRAJ_GEN:
                totalGenerateGapTrajTimeTaken += currTimeTaken;
                generateGapTrajCalls++;
                averageTimeTaken = (totalGenerateGapTrajTimeTaken / generateGapTrajCalls);
                break;
            case UNGAP_TRAJ_GEN:
                totalGenerateUngapTrajTimeTaken += currTimeTaken;
                generateUngapTrajCalls++;
                averageTimeTaken = (totalGenerateUngapTrajTimeTaken / generateUngapTrajCalls);
                break;
            case IDLING_TRAJ_GEN:
                totalGenerateIdlingTrajTimeTaken += currTimeTaken;
                generateIdlingTrajCalls++;
                averageTimeTaken = (totalGenerateIdlingTrajTimeTaken / generateIdlingTrajCalls);
                break;                                
            case TRAJ_PICK:
                totalSelectGapTrajTimeTaken += currTimeTaken;
                selectGapTrajCalls++;
                averageTimeTaken = (totalSelectGapTrajTimeTaken / selectGapTrajCalls);
                break;                                                        
            case TRAJ_COMP:
                totalCompareToCurrentTrajTimeTaken += currTimeTaken;
                compareToCurrentTrajCalls++;
                averageTimeTaken = (totalCompareToCurrentTrajTimeTaken / compareToCurrentTrajCalls);
                break;       
            case PLAN:
                totalPlanningTimeTaken += currTimeTaken;
                planningLoopCalls++;
                averageTimeTaken = (totalPlanningTimeTaken / planningLoopCalls);
                break; 
            case FEEBDACK:
                totalFeedbackControlTimeTaken += currTimeTaken;
                feedbackControlCalls++;
                averageTimeTaken = (totalFeedbackControlTimeTaken / feedbackControlCalls);
                break;      
            case PO:
                totalProjectionOperatorTimeTaken += currTimeTaken;
                projectionOperatorCalls++;
                averageTimeTaken = (totalProjectionOperatorTimeTaken / projectionOperatorCalls);
                break;       
            case CONTROL:
                totalControlTimeTaken += currTimeTaken;
                controlCalls++;
                averageTimeTaken = (totalControlTimeTaken / controlCalls);
                break;                                                                   
        }

        return averageTimeTaken;
    }
}