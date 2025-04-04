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

#include <dynamic_gap/gap_association/GapPointAssociator.h>

namespace dynamic_gap 
{
	void GapPointAssociator::updateParams(const EstimationParameters & estParams) 
	{
		estParams_ = estParams;
	}

	std::vector< std::vector<float>> GapPointAssociator::obtainGapPoints(const std::vector<Gap *> & gaps) 
	{
		std::vector< std::vector<float>> points(2*gaps.size(), std::vector<float>(2));
		int count = 0;
		int lidx = 0, ridx = 0;
		float ldist = 0.0, rdist = 0.0, ltheta = 0.0, rtheta = 0.0;
		float lX = 0.0, lY = 0.0;

		float rX = 0.0, rY = 0.0;

		for (Gap * gap : gaps) 
		{	
			// lidx = gap->LIdx();
			// ridx = gap->RIdx();
			// ldist = gap->LRange();
			// rdist = gap->RRange();
			// ltheta = idx2theta(lidx);
			// rtheta = idx2theta(ridx);
			gap->getLCartesian(lX, lY);
			gap->getRCartesian(rX, rY);		

			points.at(count).at(0) = lX; // ldist * cos(ltheta);
			points.at(count).at(1) = lY; // ldist * sin(ltheta);
			count++;
			points.at(count).at(0) = rX; // rdist * cos(rtheta);
			points.at(count).at(1) = rY; // rdist * sin(rtheta);

			count++;
        }
		return points;
	}

	std::vector<std::vector<float>> GapPointAssociator::populateDistMatrix(const std::vector<Gap *> & currentGaps, 
																			const std::vector<Gap *> & previousGaps) 
	{
		std::vector<std::vector<float>> distMatrix(2 * currentGaps.size(), std::vector<float>(2 * previousGaps.size()));
		
		// std::chrono::steady_clock::time_point populateDistMatrixStartTime = std::chrono::steady_clock::now();

		//std::cout << "number of current gaps: " << currentGaps.size() << std::endl;
		//std::cout << "number of previous gaps: " << previousGaps.size() << std::endl;
		// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "getting previous points:");
		previousGapPoints = obtainGapPoints(previousGaps);
		// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "getting current points:");
		currentGapPoints = obtainGapPoints(currentGaps);
		
		//std::cout << "dist matrix size: " << distMatrix.size() << ", " << distMatrix.at(0).size() << std::endl;
		// populate distance matrix
		// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "Distance matrix: ");
		for (int i = 0; i < distMatrix.size(); i++) 
		{
			for (int j = 0; j < distMatrix.at(i).size(); j++) 
			{
				//std::cout << i << ", " << j <<std::endl;

				distMatrix.at(i).at(j) = calculateDistance(currentGapPoints.at(i),
															previousGapPoints.at(j));
				// ROS_INFO_STREAM_NAMED("GapPointAssociator",distMatrix.at(i).at(j) << ", ");
			}
			// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "" << std::endl;
		}


		// float populateDistMatrixTime = timeTaken(populateDistMatrixStartTime);
		// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "populateDistMatrix time taken: " << populateDistMatrixTime << " seconds for " << currentGapPoints.size() << " gaps");
		
		return distMatrix;
	}

	float GapPointAssociator::calculateDistance(const std::vector<float> & currentGapPoint,
							const std::vector<float> & previousGapPoint)
	{
		float pointToPointDistSq = 0.0;
		for (int k = 0; k < currentGapPoint.size(); k++) 
			pointToPointDistSq += pow(currentGapPoint.at(k) - previousGapPoint.at(k), 2);		
	
		float pointToPointDist = std::sqrt(pointToPointDistSq);
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
            // ROS_INFO_STREAM_NAMED("GapPointAssociator", "pair (" << i << ", " << association.at(i) << ")");
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

                // ROS_INFO_STREAM_NAMED("GapPointAssociator", "From { pt: (" << prevX << ", " << prevY << "), ID: " << previousGapModelID << "} to { pt: (" << currX << ", " << currY << "), ID: " << currentGapModelID << "} with a distance of " << distMatrix.at(pair.at(0)).at(pair.at(1)));
            } else 
			{
                // ROS_INFO_STREAM_NAMED("GapPointAssociator", "From NULL to { pt: (" << currX << ", " << currY << "), ID: " << currentGapModelID << "}");
            }
			
        }
    }

	void GapPointAssociator::instantiateNewModel(const int & i,
											const std::vector<Gap *> & currentGaps, 
											int & currentModelIdx,
											const ros::Time & scanTime,
									 		const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,		 
                      				 		const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs)
	{
		float gapPtX = currentGapPoints.at(i).at(0);
		float gapPtY = currentGapPoints.at(i).at(1);

    	geometry_msgs::TwistStamped lastRbtVel = (!intermediateRbtVels.empty()) ? intermediateRbtVels.back() : geometry_msgs::TwistStamped();
	    geometry_msgs::TwistStamped lastRbtAcc = (!intermediateRbtAccs.empty()) ? intermediateRbtAccs.back() : geometry_msgs::TwistStamped();

		int currentGapIdx = int(std::floor(0.5 * i));

		if (i % 2 == 0) 
		{   // curr left
			currentGaps.at(currentGapIdx)->getLeftGapPt()->getModel()->initialize("left", currentModelIdx, gapPtX, gapPtY, 
																			scanTime, lastRbtVel, lastRbtAcc, estParams_);				
		} else 
		{
			currentGaps.at(currentGapIdx)->getRightGapPt()->getModel()->initialize("right", currentModelIdx, gapPtX, gapPtY,
																			scanTime, lastRbtVel, lastRbtAcc, estParams_);				
		}
		currentModelIdx += 1;
	}

	void GapPointAssociator::handOffModel(const std::vector<int> & pair,
									 const std::vector<Gap *> & currentGaps, 
									 const std::vector<Gap *> & previousGaps)
	{
		// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "					[handOffModel()]");
		int currentGapIdx = int(std::floor(0.5 * pair.at(0)));
		int previousGapIdx = int(std::floor(0.5 * pair.at(1)));
		// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "					currentGapIdx: " << currentGapIdx << ", previousGapIdx: " << previousGapIdx);

		if (pair.at(0) % 2 == 0)  // curr left
		{
			// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "						transfering left");
			currentGaps.at(currentGapIdx)->getLeftGapPt()->getModel()->transfer((pair.at(1) % 2 == 0) ? *previousGaps.at(previousGapIdx)->getLeftGapPt()->getModel() :
																			  				*previousGaps.at(previousGapIdx)->getRightGapPt()->getModel());			
			// currentGaps.at(currentGapIdx)->getLeftGapPt()->getModel() = (pair.at(1) % 2 == 0) ? previousGaps.at(previousGapIdx)->getLeftGapPt()->getModel() :
			// 																  previousGaps.at(previousGapIdx)->getRightGapPt()->getModel();
			currentGaps.at(currentGapIdx)->getLeftGapPt()->getModel()->setParams(estParams_);
		} else // curr right
		{
			// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "						transfering right");
			currentGaps.at(currentGapIdx)->getRightGapPt()->getModel()->transfer((pair.at(1) % 2 == 0) ? *previousGaps.at(previousGapIdx)->getLeftGapPt()->getModel() :
																			   				 *previousGaps.at(previousGapIdx)->getRightGapPt()->getModel());		
			// currentGaps.at(currentGapIdx)->getRightGapPt()->getModel() = (pair.at(1) % 2 == 0) ? previousGaps.at(previousGapIdx)->getLeftGapPt()->getModel() :
			// 																   previousGaps.at(previousGapIdx)->getRightGapPt()->getModel();
			currentGaps.at(currentGapIdx)->getRightGapPt()->getModel()->setParams(estParams_);
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

	void GapPointAssociator::assignModels(const std::vector<int> & association, 
									 const std::vector<std::vector<float>> & distMatrix, 
									 std::vector<Gap *> & currentGaps, 
									 const std::vector<Gap *> & previousGaps,
									 int & currentModelIdx,
                                     const ros::Time & scanTime, 
									 const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                      				 const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs)
	{
		// try
		// {

		// ROS_INFO_STREAM_NAMED("GapPointAssociator", "[assignModels()]");
		// std::chrono::steady_clock::time_point assignModelsStartTime = std::chrono::steady_clock::now();
		// initializing models for current gaps
		// float gapPtX, gapPtY;

		// ROS_INFO_STREAM_NAMED("GapPointAssociator", "    number of observed gaps: " << currentGaps.size() << ", number of previous gaps: " << previousGaps.size());

		// ROS_INFO_STREAM_NAMED("GapPointAssociator", "	association size: " << association.size());

		// ROS_INFO_STREAM_NAMED("GapPointAssociator", "	association: " << printVectorSingleLine(association));

		printGapAssociations(currentGaps, previousGaps, association, distMatrix);

		for (int i = 0; i < currentGapPoints.size(); i++) 
		{
			bool validAssociation = false;
			if (i < association.size()) // clause 1: previous gaps size
			{
				std::vector<int> pair{i, association.at(i)};	
				// ROS_INFO_STREAM_NAMED("GapPointAssociator", "			pair (" << pair.at(0) << ", " << pair.at(1) << ")");
				if (association.at(i) >= 0) // clause 2: association existence check
				{
					// ROS_INFO_STREAM_NAMED("GapPointAssociator", "				current point: (" << currentGapPoints.at(i).at(0) << ", " << currentGapPoints.at(i).at(1) << ")");
					// ROS_INFO_STREAM_NAMED("GapPointAssociator", "				previous point: (" << previousGapPoints.at(association.at(i)).at(0) << ", " << previousGapPoints.at(association.at(i)).at(1) << ")");
					// ROS_INFO_STREAM_NAMED("GapPointAssociator", "				association distance: " << distMatrix.at(pair.at(0)).at(pair.at(1)));
										
					// ROS_INFO_STREAM_NAMED("GapPointAssociator", "			checking association distance");

					// checking if current gap pt has association under distance threshold
					bool assoc_idx_in_range = previousGaps.size() > int(std::floor(0.5 * pair.at(1)));

					bool assoc_dist_in_thresh = (distMatrix.at(pair.at(0)).at(pair.at(1)) <= assocThresh);
					validAssociation = assoc_dist_in_thresh;
					if (validAssociation) 
					{
						// ROS_INFO_STREAM_NAMED("GapPointAssociator", "				association meets distance threshold");
						//std::cout << "associating" << std::endl;	
						//std::cout << "distance under threshold" << std::endl;
						handOffModel(pair, currentGaps, previousGaps);
					} else
					{
						// ROS_INFO_STREAM_NAMED("GapPointAssociator", "				association does not meet distance threshold");
						instantiateNewModel(i, currentGaps, currentModelIdx, scanTime, intermediateRbtVels, intermediateRbtAccs);
					}
				} else // instantiate new model
				{
					// ROS_INFO_STREAM_NAMED("GapPointAssociator", "			current gap point not associated");
					instantiateNewModel(i, currentGaps, currentModelIdx, scanTime, intermediateRbtVels, intermediateRbtAccs);				
				}
			} else
			{
				// ROS_INFO_STREAM_NAMED("GapPointAssociator", "			association does not exist");
				instantiateNewModel(i, currentGaps, currentModelIdx, scanTime, intermediateRbtVels, intermediateRbtAccs);				
			}
		}

		// ASSOCIATING MODELS
		// std::cout << "accepting associations" << std::endl;
		// for (int i = 0; i < association.size(); i++) 
		// {
		// 	//std::cout << "i " << i << std::endl;
		// 	// the values in associations are indexes for observed gaps
			
		// }

		// float assignModelsTime = timeTaken(assignModelsStartTime);
		// // ROS_INFO_STREAM_NAMED("GapPointAssociator", "assignModels time taken: " << assignModelsTime << " seconds for " << currentGaps.size() << " gaps");
	
		// } catch (...)
		// {
		// 	ROS_WARN_STREAM_NAMED("GapPointAssociator", "assignModels failed");
		// 	ROS_WARN_STREAM_NAMED("GapPointAssociator", "	distMatrix size: (" << distMatrix.size() << ", " << (distMatrix.size() > 0 ? distMatrix.at(0).size() : 0 ) << ")");
		// 	ROS_WARN_STREAM_NAMED("GapPointAssociator", "	association size: " << association.size());
	
		// 	ROS_WARN_STREAM_NAMED("GapPointAssociator", "	association: " << printVectorSingleLine(association));

		// 	for (int i = 0; i < currentGapPoints.size(); i++) 
		// 	{
		// 		instantiateNewModel(i, currentGaps, currentModelIdx, scanTime, intermediateRbtVels, intermediateRbtAccs);
		// 		// currentModelIdx += 1;
		// 	}
		// }
	}
        
}
