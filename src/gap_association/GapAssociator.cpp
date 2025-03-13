///////////////////////////////////////////////////////////////////////////////
// Hungarian.cpp: Implementation file for Class HungarianAlgorithm.
// 
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
// 
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
// 

#include <dynamic_gap/gap_association/GapAssociator.h>

namespace dynamic_gap 
{
	void GapAssociator::updateParams(const EstimationParameters & estParams) 
	{
		estParams_ = estParams;
	}

	std::vector<std::vector<float>> GapAssociator::populateDistMatrix(const std::vector<Gap *> & currentGaps, 
																			const std::vector<Gap *> & previousGaps) 
	{
		std::vector<std::vector<float>> distMatrix(currentGaps.size(), std::vector<float>(previousGaps.size()));
		
		std::chrono::steady_clock::time_point populateDistMatrixStartTime = std::chrono::steady_clock::now();

		//std::cout << "number of current gaps: " << currentGaps.size() << std::endl;
		//std::cout << "number of previous gaps: " << previousGaps.size() << std::endl;
		// // ROS_INFO_STREAM_NAMED("GapAssociator", "getting previous points:");
		// previousGapPoints = obtainGapPoints(previousGaps);
		// // ROS_INFO_STREAM_NAMED("GapAssociator", "getting current points:");
		// currentGapPoints = obtainGapPoints(currentGaps);
		
		//std::cout << "dist matrix size: " << distMatrix.size() << ", " << distMatrix.at(0).size() << std::endl;
		// populate distance matrix
		// // ROS_INFO_STREAM_NAMED("GapAssociator", "Distance matrix: ");
		for (int i = 0; i < distMatrix.size(); i++) 
		{
			for (int j = 0; j < distMatrix.at(i).size(); j++) 
			{
				//std::cout << i << ", " << j <<std::endl;

				distMatrix.at(i).at(j) = calculateDistance(currentGaps.at(i),
															previousGaps.at(i));
				// ROS_INFO_STREAM_NAMED("GapAssociator",distMatrix.at(i).at(j) << ", ");
			}
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "" << std::endl;
		}


		float populateDistMatrixTime = timeTaken(populateDistMatrixStartTime);
		// // ROS_INFO_STREAM_NAMED("GapAssociator", "populateDistMatrix time taken: " << populateDistMatrixTime << " seconds for " << currentGapPoints.size() << " gaps");
		
		return distMatrix;
	}

	float calculateDistance(Gap * currentGap, Gap * previousGap)
	{
        Eigen::Vector2f currentGapLeftPt = currentGap->getLPosition();
        Eigen::Vector2f previousGapLeftPt = previousGap->getLPosition();
        Eigen::Vector2f currentGapRightPt = currentGap->getRPosition();
        Eigen::Vector2f previousGapRightPt = previousGap->getRPosition();

		// float pointToPointDistSq = 0.0;

        float pointToPointDist = (currentGapLeftPt - previousGapLeftPt).norm() + (currentGapRightPt - previousGapRightPt).norm();

		// float pointToPointDist = std::sqrt(pointToPointDistSq);
		//std::cout << "pointToPointDist: " << pointToPointDist << std::endl;

		return pointToPointDist;
	}
	
	void printGapAssociations(const std::vector<Gap *> & currentGaps, 
							  const std::vector<Gap *> & previousGaps, 
							  const std::vector<int> & association,
							  const std::vector<std::vector<float>> & distMatrix) 
	{
        // std::cout << "printing associations" << std::endl;

        float currX = 0.0, currY = 0.0, prevX = 0.0, prevY = 0.0;
		int currentGapIdx = 0, previousGapIdx = 0;
		int currentGapModelID = -1, previousGapModelID = -1;
        for (int i = 0; i < association.size(); i++) 
		{
            std::vector<int> pair{i, association.at(i)};
            // ROS_INFO_STREAM_NAMED("GapAssociator", "pair (" << i << ", " << association.at(i) << ")");
            if (i >= 0 && association.at(i) >= 0) 
			{
                currentGapIdx = int(std::floor(0.5 * pair.at(0)));
                previousGapIdx = int(std::floor(0.5 * pair.at(1)));

                if (pair.at(0) % 2 == 0) // curr left
				{
                    currentGaps.at(currentGapIdx)->getLCartesian(currX, currY);
					currentGapModelID = currentGaps.at(currentGapIdx)->getLeftGapPt()->getModel()->getID();
				} else // curr right
                {
					currentGaps.at(currentGapIdx)->getRCartesian(currX, currY);
					currentGapModelID = currentGaps.at(currentGapIdx)->getRightGapPt()->getModel()->getID();
				}

                if (pair.at(1) % 2 == 0) // prev left
				{
                    previousGaps.at(previousGapIdx)->getLCartesian(prevX, prevY);
					previousGapModelID = previousGaps.at(previousGapIdx)->getLeftGapPt()->getModel()->getID();
				} else // prev right
                {
				   	previousGaps.at(previousGapIdx)->getRCartesian(prevX, prevY);
					previousGapModelID = previousGaps.at(previousGapIdx)->getRightGapPt()->getModel()->getID();				
				}

                // ROS_INFO_STREAM_NAMED("GapAssociator", "From { pt: (" << prevX << ", " << prevY << "), ID: " << previousGapModelID << "} to { pt: (" << currX << ", " << currY << "), ID: " << currentGapModelID << "} with a distance of " << distMatrix.at(pair.at(0)).at(pair.at(1)));
            } else 
			{
                // ROS_INFO_STREAM_NAMED("GapAssociator", "From NULL to { pt: (" << currX << ", " << currY << "), ID: " << currentGapModelID << "}");
            }
			
        }
    }

	std::string printVectorSingleLine(const std::vector<int> & vector)
	{
		std::string vectorString = "[";
		for (int i = 0; i < vector.size(); i++)
		{
			vectorString += std::to_string(vector.at(i));
			if (vector.at(i) != vector.back())
				vectorString += ", ";

		}
		vectorString += "]";
		return vectorString;
	}

}
