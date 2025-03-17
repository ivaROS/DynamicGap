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
															previousGaps.at(j));
				// ROS_INFO_STREAM_NAMED("GapAssociator",distMatrix.at(i).at(j) << ", ");
			}
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "" << std::endl;
		}


		float populateDistMatrixTime = timeTaken(populateDistMatrixStartTime);
		// // ROS_INFO_STREAM_NAMED("GapAssociator", "populateDistMatrix time taken: " << populateDistMatrixTime << " seconds for " << currentGapPoints.size() << " gaps");
		
		return distMatrix;
	}

	float GapAssociator::calculateDistance(Gap * currentGap, Gap * previousGap)
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

    void GapAssociator::assignGaps(const std::vector<int> & association, 
									const std::vector< std::vector<float> > & distMatrix, 
									const std::vector<Gap *> & currentGaps, 
									const std::vector<Gap *> & previousGaps,
									std::vector<GapTube *> & gapTubes,
									const float & t_current)
    {
        ROS_INFO_STREAM_NAMED("GapAssociator", "            [assignGaps()]");

        for (int i = 0; i < currentGaps.size(); i++) 
		{
			Gap * currentGap = currentGaps.at(i);
		
			bool validAssociation = false;
		
			if (i < association.size()) // clause 1: previous gaps size
			{
				std::vector<int> pair{i, association.at(i)};	
				// ROS_INFO_STREAM_NAMED("GapAssociator", "			pair (" << pair.at(0) << ", " << pair.at(1) << ")");
				if (association.at(i) >= 0) // clause 2: association existence check
				{
					// currentGap->setSafeToDelete();
					if (gapTubes.size() > 0)
					{
						// gapTubes.at(0)->addGap(currentGap);
						currentGap->setSafeToDelete();
					} else
					{
						currentGap->setSafeToDelete();
					}

			// 		Gap * previousGap = previousGaps.at(pair.at(1));

			// 		// find tube for previous gap
			// 		int previousTubeIdx = -1;
			// 		for (int k = 0; k < gapTubes.size(); k++)
			// 		{
			// 			GapTube * tube = gapTubes.at(k);
			// 			if (tube->getMostRecentGap()->getLeftGapPt()->getModel()->getID() == previousGap->getLeftGapPt()->getModel()->getID() &&
			// 				tube->getMostRecentGap()->getRightGapPt()->getModel()->getID() == previousGap->getRightGapPt()->getModel()->getID())
			// 			{
			// 				previousTubeIdx = k;
			// 				break;
			// 			}
			// 		}

			// 		if (previousTubeIdx < 0)
			// 		{
			// 			ROS_WARN_STREAM_NAMED("GapAssociator", "				previous gap not found in gap tubes");
			// 			// ROS_INFO_STREAM_NAMED("GapAssociator", " 				previous gap not found in gap tubes");
			// 			// throw std::runtime_error("previous gap not found in gap tubes");
			// 		}

			// 		GapTube * previousGapTube = gapTubes.at(previousTubeIdx);

			// 		// set lifespan for previous gap
			// 		previousGap->setGapLifespan(t_current);

			// 		ROS_INFO_STREAM_NAMED("GapAssociator", "				current gap: "); 
            //         ROS_INFO_STREAM_NAMED("GapAssociator", "					left point: (" << currentGap->getLPosition().transpose() << ")");
            //         ROS_INFO_STREAM_NAMED("GapAssociator", "					left ID: (" << currentGap->getLeftGapPt()->getModel()->getID() << ")");
			// 		ROS_INFO_STREAM_NAMED("GapAssociator", "					right point: (" << currentGap->getRPosition().transpose() << ")");
			// 		ROS_INFO_STREAM_NAMED("GapAssociator", "					right ID: (" << currentGap->getRightGapPt()->getModel()->getID() << ")");
			// 		ROS_INFO_STREAM_NAMED("GapAssociator", "				previous gap: ");
            //         ROS_INFO_STREAM_NAMED("GapAssociator", "					left point: (" << previousGap->getLPosition().transpose() << ")");
            //         ROS_INFO_STREAM_NAMED("GapAssociator", "					left ID: (" << previousGap->getLeftGapPt()->getModel()->getID() << ")");                    
			// 		ROS_INFO_STREAM_NAMED("GapAssociator", "					right point: (" << previousGap->getRPosition().transpose() << ")");
            //         ROS_INFO_STREAM_NAMED("GapAssociator", "					right ID: (" << previousGap->getRightGapPt()->getModel()->getID() << ")");                    
			// 		ROS_INFO_STREAM_NAMED("GapAssociator", "				association distance: " << distMatrix.at(pair.at(0)).at(pair.at(1)));
                        	
			// 		// ROS_INFO_STREAM_NAMED("GapAssociator", "			checking association distance");

			// 		if (currentGap->getLeftGapPt()->getModel()->getID() == previousGap->getLeftGapPt()->getModel()->getID() &&
			// 			currentGap->getRightGapPt()->getModel()->getID() == previousGap->getRightGapPt()->getModel()->getID())
			// 		{
			// 			ROS_INFO_STREAM_NAMED("GapAssociator", "				gap points have same model IDs! No need to do anything");
						
			// 			// currentGap->setSafeToDelete();
			// 		} else
			// 		{
			// 			ROS_INFO_STREAM_NAMED("GapAssociator", "				gap points have different model IDs! Need to update");

			// 			// find previous gap in gap tubes

			// 			// append previous gap
						
			// 			// previousGapTube->addGap(currentGap);
			// 		}

				} else // instantiate new model
				{
					ROS_INFO_STREAM_NAMED("GapAssociator", "			current gap not associated");
					currentGap->setSafeToDelete();
				}
			} else
			{
				ROS_INFO_STREAM_NAMED("GapAssociator", "			association does not exist");
				currentGap->setSafeToDelete();
			}
		}        
    }
}
