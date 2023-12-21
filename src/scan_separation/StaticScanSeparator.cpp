#include <dynamic_gap/scan_separation/StaticScanSeparator.h>

namespace dynamic_gap
{
    ////////////////// STATIC SCAN SEPARATION ///////////////////////
    /* This code is heavily experimental. Does not really work.
     *
     *
    **/


    bool StaticScanSeparator::checkModelSimilarity(dynamic_gap::Estimator * currModel, 
                                                   dynamic_gap::Estimator * prevModel) 
    {
        float eps = 0.00001;
        
        Eigen::Vector4f currState = currModel->getGapState();
        Eigen::Vector4f prevState = prevModel->getGapState();
        
        Eigen::Vector2f currGapPtVel(currState[2], currState[3]);
        Eigen::Vector2f prevGapPtVel(prevState[2], prevState[3]);

        Eigen::Vector2f currGapPtVelUnitNorm = unitNorm(currGapPtVel);
        Eigen::Vector2f prevGapPtVelUnitNorm = unitNorm(prevGapPtVel);
        // ROS_INFO_STREAM("currGapPtVelocity: " << currState[2] << ", " << currState[3] << ", prevGapPtVelocity: " << prevState[2] << ", " << prevState[3]);
        // ROS_INFO_STREAM("currGapPtVelUnitNorm: " << currGapPtVelUnitNorm[0] << ", " << currGapPtVelUnitNorm[1]);
        // ROS_INFO_STREAM("prevGapPtVelUnitNorm: " << prevGapPtVelUnitNorm[0] << ", " << prevGapPtVelUnitNorm[1]);
        float velDirDotProd = currGapPtVelUnitNorm.dot(prevGapPtVelUnitNorm);
        // ROS_INFO_STREAM("dot product: " << velDirDotProd);
        float norm_ratio = prevGapPtVel.norm() / (currGapPtVel.norm() + eps);

        bool similarVelDirs = velDirDotProd > 0.5;
        bool similarVelNorms = std::abs(norm_ratio - 1.0) < 0.1;
        // ROS_INFO_STREAM("norm ratio: " << norm_ratio);

        return (velDirDotProd > 0.5 && currGapPtVel.norm() > 0.1 && similarVelNorms);
    }

    void StaticScanSeparator::createAgentFromModels(dynamic_gap::Estimator * currModel,    
                                                    dynamic_gap::Estimator * prevModel,
                                                    std::vector<Eigen::Vector4f> & agents) 
    {
        Eigen::Vector4f currState = currModel->getGapState();
        Eigen::Vector4f prevState = prevModel->getGapState();
        
        Eigen::Vector4f newAgentState = (currState + prevState) / 2;
        // ROS_INFO_STREAM("instantiating agent: " << newAgentState[0] << ", " << newAgentState[1] << ", " << newAgentState[2] << ", " << newAgentState[3]);
        agents.push_back(newAgentState);                                  
    }

    void StaticScanSeparator::clearAgentFromStaticScan(dynamic_gap::Estimator * currModel, 
                                                        dynamic_gap::Estimator * prevModel,
                                                        sensor_msgs::LaserScan & scan) 
    {
        // int half_num_scan = scan.ranges.size() / 2;

        Eigen::Vector2f currModelMeasurement = currModel->get_x_tilde();
        Eigen::Vector2f prevModelMeasurement = prevModel->get_x_tilde();

        float currModelTheta = std::atan2(currModelMeasurement[1], currModelMeasurement[0]);
        float prevModelTheta = std::atan2(prevModelMeasurement[1], prevModelMeasurement[0]);

        int currModelScanIdx = theta2idx(currModelTheta); // int(std::round( * (half_num_scan / M_PI))) + half_num_scan;
        int prevModelScanIdx = theta2idx(prevModelTheta); // int(std::round(std::atan2() * (half_num_scan / M_PI))) + half_num_scan;
        // ROS_INFO_STREAM("prevModelScanIdx: " << prevModelScanIdx << ", currModelScanIdx: " << currModelScanIdx);

        int prevFreeScanIdx, currFreeScanIdx;
        float prevFreeScanRange, currFreeScanRange;
        // for (int j = 1; j < 2; j++) 
        // {
        prevFreeScanIdx = subtractAndWrapScanIndices(prevModelScanIdx - 1, cfg_->scan.full_scan); // (prevModelScanIdx - j); // 
            // if (prevFreeScanIdx < 0) {
            //     prevFreeScanIdx += 2*half_num_scan;
            // }
        prevFreeScanRange = scan.ranges.at(prevFreeScanIdx);
            // ROS_INFO_STREAM("range at " << prevFreeScanIdx << ": " << prevFreeScanRange);
        // }

        // for (int j = 1; j < 2; j++) 
        // {
        currFreeScanIdx = (currModelScanIdx + 1) % (cfg_->scan.full_scan); // (2*half_num_scan);
        currFreeScanRange = scan.ranges.at(currFreeScanIdx);
        //     ROS_INFO_STREAM("range at " << currFreeScanIdx << ": " << currFreeScanRange);
        // }

        // need distance from curr to prev
        float prevToCurrIdxSpan = subtractAndWrapScanIndices(currFreeScanIdx - prevFreeScanIdx, cfg_->scan.full_scan); // (currFreeScanIdx - prevFreeScanIdx); // 
        // if (prevToCurrIdxSpan < 0) {
        //     prevToCurrIdxSpan += 2*half_num_scan;
        // }
        // ROS_INFO_STREAM("prevToCurrIdxSpan: " << prevToCurrIdxSpan);

        for (int counter = 1; counter < prevToCurrIdxSpan; counter++) 
        {
            int intermediateScanIdx = (prevFreeScanIdx + counter) % (cfg_->scan.full_scan); // (2*half_num_scan);
            float intermediateScanDist = prevFreeScanRange + (currFreeScanRange - prevFreeScanRange) * (counter / prevToCurrIdxSpan);
            // ROS_INFO_STREAM("replacing range at " << intermediateScanIdx << " from " << scan.ranges.at(intermediateScanIdx) << " to " << intermediateScanDist);
            scan.ranges.at(intermediateScanIdx) = intermediateScanDist;
        }
    }

    bool compareModelBearingValues(dynamic_gap::Estimator* model1, dynamic_gap::Estimator* model2) 
    {
        Eigen::Vector4f state1 = model1->getGapState();
        Eigen::Vector4f state2 = model2->getGapState();
        
        return atan2(state1[1], state1[0]) < atan2(state2[1], state2[0]);
    }

    sensor_msgs::LaserScan StaticScanSeparator::staticDynamicScanSeparation(const std::vector<dynamic_gap::Gap> & rawGaps, 
                                                                            boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr) 
    {
        sensor_msgs::LaserScan scan = *scanPtr.get(); 
        try
        {
            if (rawGaps.size() == 0)
                return scan;

            std::vector<dynamic_gap::Estimator *> gapModels;
            for (const dynamic_gap::Gap & gap : rawGaps) 
            {
                gapModels.push_back(gap.leftGapPtModel_);
                gapModels.push_back(gap.rightGapPtModel_);
            }

            for (dynamic_gap::Estimator * & model : gapModels) 
                model->isolateGapDynamics();
            
            sort(gapModels.begin(), gapModels.end(), compareModelBearingValues);

            // iterate through models
            dynamic_gap::Estimator * prevModel = gapModels[0];
            dynamic_gap::Estimator * currModel = gapModels[1];

            std::vector<Eigen::Vector4f> agents;

            // ROS_INFO_STREAM("looping through models");
            for (int i = 1; i < gapModels.size(); i++) 
            {
                currModel = gapModels[i];

                if (checkModelSimilarity(currModel, prevModel)) 
                {
                    clearAgentFromStaticScan(currModel, prevModel, scan);
                    createAgentFromModels(currModel, prevModel, agents);
                }

                prevModel = currModel;
            }

            // bridging models
            prevModel = gapModels.back();
            currModel = gapModels[0];

            if (checkModelSimilarity(currModel, prevModel)) 
            {
                clearAgentFromStaticScan(currModel, prevModel, scan);
                createAgentFromModels(currModel, prevModel, agents);
            }

            currAgents = agents;

            // create a static scan (would need to interpolate to fill in spots where agents are)
            
            // create a pose for each agent, forward propagate it, somehow turn that pose into bearings for scan 

            // 
        } catch (...)
        {
            ROS_ERROR_STREAM("[staticDynamicScanSeparation() failed");
        }

        return scan;
    }
}