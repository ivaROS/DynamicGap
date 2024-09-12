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

#include <dynamic_gap/gap_estimation/GapAssociator.h>

namespace dynamic_gap 
{
	std::vector< std::vector<float>> GapAssociator::obtainGapPoints(const std::vector<dynamic_gap::Gap *> & gaps) 
	{
		std::vector< std::vector<float>> points(2*gaps.size(), std::vector<float>(2));
		int count = 0;
		int lidx = 0, ridx = 0;
		float ldist = 0.0, rdist = 0.0, ltheta = 0.0, rtheta = 0.0;
		for (dynamic_gap::Gap * gap : gaps) 
		{	
			lidx = gap->LIdx();
			ridx = gap->RIdx();
			ldist = gap->LRange();
			rdist = gap->RRange();
			ltheta = idx2theta(lidx);
			rtheta = idx2theta(ridx);		

			points.at(count).at(0) = ldist * cos(ltheta);
			points.at(count).at(1) = ldist * sin(ltheta);
			count++;
			points.at(count).at(0) = rdist * cos(rtheta);
			points.at(count).at(1) = rdist * sin(rtheta);

			count++;
        }
		return points;
	}

	std::vector<std::vector<float>> GapAssociator::obtainDistMatrix(const std::vector<dynamic_gap::Gap *> & currentGaps, 
																	const std::vector<dynamic_gap::Gap *> & previousGaps) 
	{
		std::vector<std::vector<float>> distMatrix(2 * currentGaps.size(), std::vector<float>(2 * previousGaps.size()));
		
		try
		{
			std::chrono::steady_clock::time_point obtainDistMatrixStartTime = std::chrono::steady_clock::now();

			//std::cout << "number of current gaps: " << currentGaps.size() << std::endl;
			//std::cout << "number of previous gaps: " << previousGaps.size() << std::endl;
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "getting previous points:");
			previousGapPoints = obtainGapPoints(previousGaps);
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "getting current points:");
			currentGapPoints = obtainGapPoints(currentGaps);
			
			//std::cout << "dist matrix size: " << distMatrix.size() << ", " << distMatrix.at(0).size() << std::endl;
			// populate distance matrix
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "Distance matrix: ");
			for (int i = 0; i < distMatrix.size(); i++) 
			{
				for (int j = 0; j < distMatrix.at(i).size(); j++) 
				{
					float pointToPointDist = 0;
					//std::cout << i << ", " << j <<std::endl;
					for (int k = 0; k < currentGapPoints.at(i).size(); k++) 
						pointToPointDist += pow(currentGapPoints.at(i).at(k) - previousGapPoints.at(j).at(k), 2);

					//std::cout << "pointToPointDist: " << pointToPointDist << std::endl;
					distMatrix.at(i).at(j) = sqrt(pointToPointDist);
					// ROS_INFO_STREAM_NAMED("GapAssociator",distMatrix.at(i).at(j) << ", ");
				}
				// // ROS_INFO_STREAM_NAMED("GapAssociator", "" << std::endl;
			}


			float obtainDistMatrixTime = timeTaken(obtainDistMatrixStartTime);
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "obtainDistMatrix time taken: " << obtainDistMatrixTime << " seconds for " << currentGapPoints.size() << " gaps");
		} catch (...)
		{
			ROS_WARN_STREAM_NAMED("GapAssociator", "obtainDistMatrix failed");
		}

		return distMatrix;
}
	
	void printGapAssociations(const std::vector<dynamic_gap::Gap *> & currentGaps, 
							  const std::vector<dynamic_gap::Gap *> & previousGaps, 
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
                currentGapIdx = int(std::floor(pair.at(0) / 2.0));
                previousGapIdx = int(std::floor(pair.at(1) / 2.0));

                if (pair.at(0) % 2 == 0) // curr left
				{
                    currentGaps.at(currentGapIdx)->getLCartesian(currX, currY);
					currentGapModelID = currentGaps.at(currentGapIdx)->leftGapPtModel_->getID();
				} else // curr right
                {
					currentGaps.at(currentGapIdx)->getRCartesian(currX, currY);
					currentGapModelID = currentGaps.at(currentGapIdx)->rightGapPtModel_->getID();
				}

                if (pair.at(1) % 2 == 0) // prev left
				{
                    previousGaps.at(previousGapIdx)->getLCartesian(prevX, prevY);
					previousGapModelID = previousGaps.at(previousGapIdx)->leftGapPtModel_->getID();
				} else // prev right
                {
				   	previousGaps.at(previousGapIdx)->getRCartesian(prevX, prevY);
					previousGapModelID = previousGaps.at(previousGapIdx)->rightGapPtModel_->getID();				
				}

                // ROS_INFO_STREAM_NAMED("GapAssociator", "From { pt: (" << prevX << ", " << prevY << "), ID: " << previousGapModelID << "} to { pt: (" << currX << ", " << currY << "), ID: " << currentGapModelID << "} with a distance of " << distMatrix.at(pair.at(0)).at(pair.at(1)));
            } else 
			{
                // ROS_INFO_STREAM_NAMED("GapAssociator", "From NULL to { pt: (" << currX << ", " << currY << "), ID: " << currentGapModelID << "}");
            }
			
        }
    }

	void printGapTransition(const std::vector<dynamic_gap::Gap *> & currentGaps, 
							const std::vector<dynamic_gap::Gap *> & previousGaps,
							const std::vector<std::vector<float>> & distMatrix, 
							const std::vector<int> & pair,
							const bool & validAssociation) 
	{
		/*
		// may be broken
		int currentGapIdx = int(std::floor(pair.at(0) / 2.0));
		int previousGapIdx = int(std::floor(pair.at(1) / 2.0));
		// ROS_INFO_STREAM_NAMED("GapAssociator", "    pair (" << pair.at(0) << ", " << pair.at(1) << ")");


		float currX = 0.0, currY = 0.0, prevX = 0.0, prevY = 0.0;
		if (pair.at(0) % 2 == 0)  // curr left
			currentGaps.at(currentGapIdx)->getLCartesian(currX, currY);
		else
			currentGaps.at(currentGapIdx)->getRCartesian(currX, currY);

		if (validAssociation) 
		{
			if (pair.at(1) % 2 == 0) 
			{
				previousGaps.at(previousGapIdx)->getLCartesian(prevX, prevY);
				// ROS_INFO_STREAM_NAMED("GapAssociator", "    	accepting transition of index " << previousGaps.at(previousGapIdx)->leftGapPtModel_->getID());				
			} else 
			{
				previousGaps.at(previousGapIdx)->getRCartesian(prevX, prevY);
				// ROS_INFO_STREAM_NAMED("GapAssociator", "    	accepting transition of index " << previousGaps.at(previousGapIdx)->rightGapPtModel_->getID());
			}
			// ROS_INFO_STREAM_NAMED("GapAssociator", "    	from (" << prevX << ", " << prevY << ") to (" << currX << ", " << currY << ") with a distance of " << distMatrix.at(pair.at(0)).at(pair.at(1)));

		} else 
		{
			if (pair.at(1) >= 0) 
			{
				if (pair.at(1) % 2 == 0) 
				{ 
					previousGaps.at(previousGapIdx)->getLCartesian(prevX, prevY);
					// ROS_INFO_STREAM_NAMED("GapAssociator", "    	rejecting transition of index " << previousGaps.at(previousGapIdx)->leftGapPtModel_->getID());
				} else 
				{
					previousGaps.at(previousGapIdx)->getRCartesian(prevX, prevY);
					// ROS_INFO_STREAM_NAMED("GapAssociator", "    	rejecting transition of index " << previousGaps.at(previousGapIdx)->rightGapPtModel_->getID());					
				}
				// ROS_INFO_STREAM_NAMED("GapAssociator", "    	from (" << prevX << ", " << prevY << ") to (" << currX << ", " << currY << ") with a distance of " << distMatrix.at(pair.at(0)).at(pair.at(1)));

			} else 
			{
				// ROS_INFO_STREAM_NAMED("GapAssociator", "    	rejecting, more current gaps than previous gaps");
			}
		}
		*/
	}

	void GapAssociator::instantiateNewModel(const int & i,
											const std::vector<dynamic_gap::Gap *> & currentGaps, 
											int & currentModelIdx,
											const ros::Time & scanTime,
									 		const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,		 
                      				 		const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs)
	{
		float gapPtX = currentGapPoints.at(i).at(0);
		float gapPtY = currentGapPoints.at(i).at(1);

    	geometry_msgs::TwistStamped lastRbtVel = (!intermediateRbtVels.empty()) ? intermediateRbtVels.back() : geometry_msgs::TwistStamped();
	    geometry_msgs::TwistStamped lastRbtAcc = (!intermediateRbtAccs.empty()) ? intermediateRbtAccs.back() : geometry_msgs::TwistStamped();

		int currentGapIdx = int(std::floor(i / 2.0));

		if (i % 2 == 0) 
		{   // curr left
			currentGaps.at(currentGapIdx)->leftGapPtModel_->initialize("left", currentModelIdx, gapPtX, gapPtY, 
																			scanTime, lastRbtVel, lastRbtAcc);				
		} else 
		{
			currentGaps.at(currentGapIdx)->rightGapPtModel_->initialize("right", currentModelIdx, gapPtX, gapPtY,
																			scanTime, lastRbtVel, lastRbtAcc);				
		}
		currentModelIdx += 1;
	}

	void GapAssociator::handOffModel(const std::vector<int> & pair,
									 const std::vector<dynamic_gap::Gap *> & currentGaps, 
									 const std::vector<dynamic_gap::Gap *> & previousGaps)
	{
		// // ROS_INFO_STREAM_NAMED("GapAssociator", "					[handOffModel()]");
		int currentGapIdx = int(std::floor(pair.at(0) / 2.0));
		int previousGapIdx = int(std::floor(pair.at(1) / 2.0));
		// // ROS_INFO_STREAM_NAMED("GapAssociator", "					currentGapIdx: " << currentGapIdx << ", previousGapIdx: " << previousGapIdx);

		if (pair.at(0) % 2 == 0)  // curr left
		{
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "						transfering left");
			currentGaps.at(currentGapIdx)->leftGapPtModel_->transfer((pair.at(1) % 2 == 0) ? *previousGaps.at(previousGapIdx)->leftGapPtModel_ :
																			  				*previousGaps.at(previousGapIdx)->rightGapPtModel_);			
			// currentGaps.at(currentGapIdx)->leftGapPtModel_ = (pair.at(1) % 2 == 0) ? previousGaps.at(previousGapIdx)->leftGapPtModel_ :
			// 																  previousGaps.at(previousGapIdx)->rightGapPtModel_;
		} else // curr right
		{
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "						transfering right");
			currentGaps.at(currentGapIdx)->rightGapPtModel_->transfer((pair.at(1) % 2 == 0) ? *previousGaps.at(previousGapIdx)->leftGapPtModel_ :
																			   				 *previousGaps.at(previousGapIdx)->rightGapPtModel_);		
			// currentGaps.at(currentGapIdx)->rightGapPtModel_ = (pair.at(1) % 2 == 0) ? previousGaps.at(previousGapIdx)->leftGapPtModel_ :
			// 																   previousGaps.at(previousGapIdx)->rightGapPtModel_;
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

	void GapAssociator::assignModels(const std::vector<int> & association, 
									 const std::vector<std::vector<float>> & distMatrix, 
									 std::vector<dynamic_gap::Gap *> & currentGaps, 
									 const std::vector<dynamic_gap::Gap *> & previousGaps,
									 int & currentModelIdx,
                                     const ros::Time & scanTime, 
									 const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                      				 const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs)
	{
		try
		{
			// ROS_INFO_STREAM_NAMED("GapAssociator", "[assignModels()]");
			std::chrono::steady_clock::time_point assignModelsStartTime = std::chrono::steady_clock::now();
			// initializing models for current gaps
			// float gapPtX, gapPtY;

			// ROS_INFO_STREAM_NAMED("GapAssociator", "    number of observed gaps: " << currentGaps.size() << ", number of previous gaps: " << previousGaps.size());

			// ROS_INFO_STREAM_NAMED("GapAssociator", "	association size: " << association.size());

			// ROS_INFO_STREAM_NAMED("GapAssociator", "	association: " << printVectorSingleLine(association));

			printGapAssociations(currentGaps, previousGaps, association, distMatrix);

			for (int i = 0; i < currentGapPoints.size(); i++) 
			{
				bool validAssociation = false;
				if (i < association.size()) // clause 1: previous gaps size
				{
					std::vector<int> pair{i, association.at(i)};	
					// ROS_INFO_STREAM_NAMED("GapAssociator", "			pair (" << pair.at(0) << ", " << pair.at(1) << ")");
					if (association.at(i) >= 0) // clause 2: association existence check
					{
						// ROS_INFO_STREAM_NAMED("GapAssociator", "				current point: (" << currentGapPoints.at(i).at(0) << ", " << currentGapPoints.at(i).at(1) << ")");
						// ROS_INFO_STREAM_NAMED("GapAssociator", "				previous point: (" << previousGapPoints.at(association.at(i)).at(0) << ", " << previousGapPoints.at(association.at(i)).at(1) << ")");
						// ROS_INFO_STREAM_NAMED("GapAssociator", "				association distance: " << distMatrix.at(pair.at(0)).at(pair.at(1)));
											
						// ROS_INFO_STREAM_NAMED("GapAssociator", "			checking association distance");

						// checking if current gap pt has association under distance threshold
						bool assoc_idx_in_range = previousGaps.size() > int(std::floor(pair.at(1) / 2.0));

						bool assoc_dist_in_thresh = (distMatrix.at(pair.at(0)).at(pair.at(1)) <= assocThresh);
						validAssociation = assoc_dist_in_thresh;
						if (validAssociation) 
						{
							// ROS_INFO_STREAM_NAMED("GapAssociator", "				association meets distance threshold");
							//std::cout << "associating" << std::endl;	
							//std::cout << "distance under threshold" << std::endl;
							handOffModel(pair, currentGaps, previousGaps);
						} else
						{
							// ROS_INFO_STREAM_NAMED("GapAssociator", "				association does not meet distance threshold");
							instantiateNewModel(i, currentGaps, currentModelIdx, scanTime, intermediateRbtVels, intermediateRbtAccs);
						}
					} else // instantiate new model
					{
						// ROS_INFO_STREAM_NAMED("GapAssociator", "			current gap point not associated");
						instantiateNewModel(i, currentGaps, currentModelIdx, scanTime, intermediateRbtVels, intermediateRbtAccs);				
					}
					printGapTransition(currentGaps, previousGaps, distMatrix, pair, validAssociation);
				} else
				{
					// ROS_INFO_STREAM_NAMED("GapAssociator", "			association does not exist");
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

			float assignModelsTime = timeTaken(assignModelsStartTime);
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "assignModels time taken: " << assignModelsTime << " seconds for " << currentGaps.size() << " gaps");
		} catch (...)
		{
			ROS_WARN_STREAM_NAMED("GapAssociator", "assignModels failed");
			ROS_WARN_STREAM_NAMED("GapAssociator", "	distMatrix size: (" << distMatrix.size() << ", " << (distMatrix.size() > 0 ? distMatrix.at(0).size() : 0 ) << ")");
			ROS_WARN_STREAM_NAMED("GapAssociator", "	association size: " << association.size());
	
			ROS_WARN_STREAM_NAMED("GapAssociator", "	association: " << printVectorSingleLine(association));

			for (int i = 0; i < currentGapPoints.size(); i++) 
			{
				instantiateNewModel(i, currentGaps, currentModelIdx, scanTime, intermediateRbtVels, intermediateRbtAccs);
				// currentModelIdx += 1;
			}
		}
	}
        

	//////////////////////////////////////////////////////////
	// 	Everything below this was taken from                //
	// https://github.com/mcximing/hungarian-algorithm-cpp  //
	//////////////////////////////////////////////////////////

	std::vector<int> GapAssociator::associateGaps(const std::vector< std::vector<float> > & distMatrix) 
	{
		std::vector<int> association;

		try
		{
			// NEW ASSIGNMENT OBTAINED
			std::chrono::steady_clock::time_point associateGapsStartTime = std::chrono::steady_clock::now();

			// std::cout << "obtaining new assignment" << std::endl;
			if (distMatrix.size() > 0 && distMatrix.at(0).size() > 0) 
			{
				//std::cout << "solving" << std::endl;
				float cost = Solve(distMatrix, association);
				//std::cout << "done solving" << std::endl;
			}

			// float associateGapsTime = timeTaken(associateGapsStartTime);
			// // ROS_INFO_STREAM_NAMED("GapAssociator", "associateGaps time taken: " << associateGapsTime << " seconds for " << currentGapPoints.size() << " gaps");
		} catch (...)
		{
			ROS_WARN_STREAM_NAMED("GapAssociator", "associateGaps failed");			
		}

		return association;
    }
	
	//********************************************************//
	// A single function wrapper for solving assignment problem.
	//********************************************************//
	float GapAssociator::Solve(const std::vector <std::vector<float> >& DistMatrix, std::vector<int>& Assignment)
	{
		unsigned int nRows = DistMatrix.size();
		unsigned int nCols = DistMatrix.at(0).size();

		float *distMatrixIn = new float[nRows * nCols];
		int *assignment = new int[nRows];
		float cost = 0.0;

		// Fill in the distMatrixIn. Mind the index is "i + nRows * j".
		// Here the cost matrix of size MxN is defined as a float precision array of N*M elements. 
		// In the solving functions matrices are seen to be saved MATLAB-internally in row-order.
		// (i.e. the matrix [1 2; 3 4] will be stored as a vector [1 3 2 4], NOT [1 2 3 4]).
		for (unsigned int i = 0; i < nRows; i++)
			for (unsigned int j = 0; j < nCols; j++)
				distMatrixIn[i + nRows * j] = DistMatrix.at(i).at(j);
		
		// call solving function
		assignmentoptimal(assignment, &cost, distMatrixIn, nRows, nCols);

		Assignment.clear();
		for (unsigned int r = 0; r < nRows; r++)
			Assignment.push_back(assignment[r]);

		delete[] distMatrixIn;
		delete[] assignment;
		return cost;
	}


	//********************************************************//
	// Solve optimal solution for assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
	//********************************************************//
	void GapAssociator::assignmentoptimal(int *assignment, float *cost, float *distMatrixIn, int nOfRows, int nOfColumns)
	{
		float *distMatrix, *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
		bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix;
		int nOfElements, minDim, row, col;

		/* initialization */
		*cost = 0;
		for (row = 0; row<nOfRows; row++)
			assignment[row] = -1;

		/* generate working copy of distance Matrix */
		/* check if all matrix elements are positive */
		nOfElements = nOfRows * nOfColumns;
		distMatrix = (float *)malloc(nOfElements * sizeof(float));
		distMatrixEnd = distMatrix + nOfElements;

		for (row = 0; row<nOfElements; row++)
		{
			value = distMatrixIn[row];
			if (value < 0)
				std::cerr << "All matrix elements have to be non-negative." << std::endl;
			distMatrix[row] = value;
		}


		/* memory allocation */
		coveredColumns = (bool *)calloc(nOfColumns, sizeof(bool));
		coveredRows = (bool *)calloc(nOfRows, sizeof(bool));
		starMatrix = (bool *)calloc(nOfElements, sizeof(bool));
		primeMatrix = (bool *)calloc(nOfElements, sizeof(bool));
		newStarMatrix = (bool *)calloc(nOfElements, sizeof(bool)); /* used in step4 */

		/* preliminary steps */
		if (nOfRows <= nOfColumns)
		{
			minDim = nOfRows;

			for (row = 0; row<nOfRows; row++)
			{
				/* find the smallest element in the row */
				distMatrixTemp = distMatrix + row;
				minValue = *distMatrixTemp;
				distMatrixTemp += nOfRows;
				while (distMatrixTemp < distMatrixEnd)
				{
					value = *distMatrixTemp;
					if (value < minValue)
						minValue = value;
					distMatrixTemp += nOfRows;
				}

				/* subtract the smallest element from each element of the row */
				distMatrixTemp = distMatrix + row;
				while (distMatrixTemp < distMatrixEnd)
				{
					*distMatrixTemp -= minValue;
					distMatrixTemp += nOfRows;
				}
			}

			/* Steps 1 and 2a */
			for (row = 0; row<nOfRows; row++)
				for (col = 0; col<nOfColumns; col++)
					if (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)
						if (!coveredColumns[col])
						{
							starMatrix[row + nOfRows*col] = true;
							coveredColumns[col] = true;
							break;
						}
		}
		else /* if(nOfRows > nOfColumns) */
		{
			minDim = nOfColumns;

			for (col = 0; col<nOfColumns; col++)
			{
				/* find the smallest element in the column */
				distMatrixTemp = distMatrix + nOfRows*col;
				columnEnd = distMatrixTemp + nOfRows;

				minValue = *distMatrixTemp++;
				while (distMatrixTemp < columnEnd)
				{
					value = *distMatrixTemp++;
					if (value < minValue)
						minValue = value;
				}

				/* subtract the smallest element from each element of the column */
				distMatrixTemp = distMatrix + nOfRows*col;
				while (distMatrixTemp < columnEnd)
					*distMatrixTemp++ -= minValue;
			}

			/* Steps 1 and 2a */
			for (col = 0; col<nOfColumns; col++)
				for (row = 0; row<nOfRows; row++)
					if (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON)
						if (!coveredRows[row])
						{
							starMatrix[row + nOfRows*col] = true;
							coveredColumns[col] = true;
							coveredRows[row] = true;
							break;
						}
			for (row = 0; row<nOfRows; row++)
				coveredRows[row] = false;

		}

		/* move to step 2b */
		step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);

		/* compute cost and remove invalid assignments */
		computeassignmentcost(assignment, cost, distMatrixIn, nOfRows);

		/* free allocated memory */
		free(distMatrix);
		free(coveredColumns);
		free(coveredRows);
		free(starMatrix);
		free(primeMatrix);
		free(newStarMatrix);

		return;
	}

	/********************************************************/
	void GapAssociator::buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns)
	{
		int row, col;

		for (row = 0; row<nOfRows; row++)
			for (col = 0; col<nOfColumns; col++)
				if (starMatrix[row + nOfRows*col])
				{
	#ifdef ONE_INDEXING
					assignment[row] = col + 1; /* MATLAB-Indexing */
	#else
					assignment[row] = col;
	#endif
					break;
				}
	}

	/********************************************************/
	void GapAssociator::computeassignmentcost(int *assignment, float *cost, float *distMatrix, int nOfRows)
	{
		int row, col;

		for (row = 0; row<nOfRows; row++)
		{
			col = assignment[row];
			if (col >= 0)
				*cost += distMatrix[row + nOfRows*col];
		}
	}

	/********************************************************/
	void GapAssociator::step2a(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
	{
		bool *starMatrixTemp, *columnEnd;
		int col;

		/* cover every column containing a starred zero */
		for (col = 0; col<nOfColumns; col++)
		{
			starMatrixTemp = starMatrix + nOfRows*col;
			columnEnd = starMatrixTemp + nOfRows;
			while (starMatrixTemp < columnEnd){
				if (*starMatrixTemp++)
				{
					coveredColumns[col] = true;
					break;
				}
			}
		}

		/* move to step 3 */
		step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}

	/********************************************************/
	void GapAssociator::step2b(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
	{
		int col, nOfCoveredColumns;

		/* count covered columns */
		nOfCoveredColumns = 0;
		for (col = 0; col<nOfColumns; col++)
			if (coveredColumns[col])
				nOfCoveredColumns++;

		if (nOfCoveredColumns == minDim)
		{
			/* algorithm finished */
			buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
		}
		else
		{
			/* move to step 3 */
			step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
		}

	}

	/********************************************************/
	void GapAssociator::step3(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
	{
		bool zerosFound;
		int row, col, starCol;

		zerosFound = true;
		while (zerosFound)
		{
			zerosFound = false;
			for (col = 0; col<nOfColumns; col++)
				if (!coveredColumns[col])
					for (row = 0; row<nOfRows; row++)
						if ((!coveredRows[row]) && (fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON))
						{
							/* prime zero */
							primeMatrix[row + nOfRows*col] = true;

							/* find starred zero in current row */
							for (starCol = 0; starCol<nOfColumns; starCol++)
								if (starMatrix[row + nOfRows*starCol])
									break;

							if (starCol == nOfColumns) /* no starred zero found */
							{
								/* move to step 4 */
								step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
								return;
							}
							else
							{
								coveredRows[row] = true;
								coveredColumns[starCol] = false;
								zerosFound = true;
								break;
							}
						}
		}

		/* move to step 5 */
		step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}

	/********************************************************/
	void GapAssociator::step4(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
	{
		int n, starRow, starCol, primeRow, primeCol;
		int nOfElements = nOfRows*nOfColumns;

		/* generate temporary copy of starMatrix */
		for (n = 0; n<nOfElements; n++)
			newStarMatrix[n] = starMatrix[n];

		/* star current zero */
		newStarMatrix[row + nOfRows*col] = true;

		/* find starred zero in current column */
		starCol = col;
		for (starRow = 0; starRow<nOfRows; starRow++)
			if (starMatrix[starRow + nOfRows*starCol])
				break;

		while (starRow<nOfRows)
		{
			/* unstar the starred zero */
			newStarMatrix[starRow + nOfRows*starCol] = false;

			/* find primed zero in current row */
			primeRow = starRow;
			for (primeCol = 0; primeCol<nOfColumns; primeCol++)
				if (primeMatrix[primeRow + nOfRows*primeCol])
					break;

			/* star the primed zero */
			newStarMatrix[primeRow + nOfRows*primeCol] = true;

			/* find starred zero in current column */
			starCol = primeCol;
			for (starRow = 0; starRow<nOfRows; starRow++)
				if (starMatrix[starRow + nOfRows*starCol])
					break;
		}

		/* use temporary copy as new starMatrix */
		/* delete all primes, uncover all rows */
		for (n = 0; n<nOfElements; n++)
		{
			primeMatrix[n] = false;
			starMatrix[n] = newStarMatrix[n];
		}
		for (n = 0; n<nOfRows; n++)
			coveredRows[n] = false;

		/* move to step 2a */
		step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}

	/********************************************************/
	void GapAssociator::step5(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
	{
		float h, value;
		int row, col;

		/* find smallest uncovered element h */
		h = DBL_MAX;
		for (row = 0; row<nOfRows; row++)
			if (!coveredRows[row])
				for (col = 0; col<nOfColumns; col++)
					if (!coveredColumns[col])
					{
						value = distMatrix[row + nOfRows*col];
						if (value < h)
							h = value;
					}

		/* add h to each covered row */
		for (row = 0; row<nOfRows; row++)
			if (coveredRows[row])
				for (col = 0; col<nOfColumns; col++)
					distMatrix[row + nOfRows*col] += h;

		/* subtract h from each uncovered column */
		for (col = 0; col<nOfColumns; col++)
			if (!coveredColumns[col])
				for (row = 0; row<nOfRows; row++)
					distMatrix[row + nOfRows*col] -= h;

		/* move to step 3 */
		step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}
}
