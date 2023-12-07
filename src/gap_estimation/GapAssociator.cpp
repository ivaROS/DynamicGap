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
	std::vector< std::vector<float>> obtainGapPoints(const std::vector<dynamic_gap::Gap> & gaps) 
	{
		std::vector< std::vector<float>> points(2*gaps.size(), std::vector<float>(2));
		int count = 0;
		for (const dynamic_gap::Gap & gap : gaps) 
		{	
			// this is still true, I just don't know how to handle indexing right now
            // populating the coordinates of the gap points (in rbt frame) to compute distances
			//std::cout << "adding left points" << std::endl;
			//std::cout << "convex l dist: " << g.convex.leftDist_ << ", half scan: " << g.half_scan << ", convex l idx: " << g.convex.leftIdx_ << std::endl;
			//std::cout << "convex r dist: " << g.convex.rightDist_ << ", half scan: " << g.half_scan << ", convex r idx: " << g.convex.rightIdx_ << std::endl;
			// std::string print_string;					

			int lidx = gap.LIdx();
			int ridx = gap.RIdx();
			float ldist = gap.LDist();
			float rdist = gap.RDist();
			float ltheta = idx2theta(lidx);
			float rtheta = idx2theta(ridx);		
			// float left_x = 
			// float left_y = 
			// float right_x = 
			// float right_y = 

			points[count][0] = ldist * cos(ltheta);
			points[count][1] = ldist * sin(ltheta);
			count++;
			points[count][0] = rdist * cos(rtheta);
			points[count][1] = rdist * sin(rtheta);

			count++;
			// ROS_INFO_STREAM(print_string);
        }
		return points;
	}

	std::vector<std::vector<float>> GapAssociator::obtainDistMatrix(const std::vector<dynamic_gap::Gap> & currentGaps, 
																	const std::vector<dynamic_gap::Gap> & previousGaps) 
	{
		float start_time = ros::Time::now().toSec(); 
		//std::cout << "number of current gaps: " << currentGaps.size() << std::endl;
		//std::cout << "number of previous gaps: " << previousGaps.size() << std::endl;
		// ROS_INFO_STREAM("getting previous points:");
		previousGapPoints = obtainGapPoints(previousGaps);
		// ROS_INFO_STREAM("getting current points:");
        currentGapPoints = obtainGapPoints(currentGaps);
        
		std::vector<std::vector<float>> distMatrix(currentGapPoints.size(), std::vector<float>(previousGapPoints.size()));
        //std::cout << "dist matrix size: " << distMatrix.size() << ", " << distMatrix[0].size() << std::endl;
		// populate distance matrix
		// ROS_INFO_STREAM("Distance matrix: ");
        for (int i = 0; i < distMatrix.size(); i++) 
		{
            for (int j = 0; j < distMatrix[i].size(); j++) 
			{
                float pointToPointDist = 0;
                //std::cout << i << ", " << j <<std::endl;
                for (int k = 0; k < currentGapPoints[i].size(); k++) 
                    pointToPointDist += pow(currentGapPoints[i][k] - previousGapPoints[j][k], 2);

                //std::cout << "pointToPointDist: " << pointToPointDist << std::endl;
                distMatrix[i][j] = sqrt(pointToPointDist);
                // ROS_INFO_STREAM(distMatrix[i][j] << ", ");
            }
			// ROS_INFO_STREAM("" << std::endl;
        }

		return distMatrix;

		// ROS_INFO_STREAM("obtainDistMatrix time elapsed: " << ros::Time::now().toSec() - start_time);
	}
	
	void printGapAssociations(const std::vector<dynamic_gap::Gap> & currentGaps, 
							  const std::vector<dynamic_gap::Gap> & previousGaps, 
							  const std::vector<int> & association,
							  const std::vector<std::vector<float>> & distMatrix) 
	{
        // std::cout << "printing associations" << std::endl;

        float currX, currY, prevX, prevY;
        for (int i = 0; i < association.size(); i++) 
		{
            std::vector<int> pair{i, association[i]};
            ROS_INFO_STREAM("pair (" << i << ", " << association[i] << ")");
            if (i >= 0 && association[i] >= 0) 
			{
                int currentGapIdx = int(std::floor(pair[0] / 2.0));
                int previousGapIdx = int(std::floor(pair[1] / 2.0));

                if (pair[0] % 2 == 0) // curr left
                    currentGaps.at(currentGapIdx).getLCartesian(currX, currY);
                else // curr right
                    currentGaps.at(currentGapIdx).getRCartesian(currX, currY);
                
                if (pair[1] % 2 == 0) // prev left
                    previousGaps.at(previousGapIdx).getLCartesian(prevX, prevY);
                else // prev right
                    previousGaps.at(previousGapIdx).getRCartesian(prevX, prevY);
                
                ROS_INFO_STREAM("From (" << prevX << ", " << prevY << ") to (" << currX << ", " << currY << ") with a distance of " << distMatrix[pair[0]][pair[1]]);
            } else 
			{
                ROS_INFO_STREAM("From NULL to (" << currX << ", " <<  currY << ")");
            }
			
        }
    }

	void printGapTransition(const std::vector<dynamic_gap::Gap> & currentGaps, 
							const std::vector<dynamic_gap::Gap> & previousGaps,
							const std::vector< std::vector<float> > & distMatrix, 
							const std::vector<int> & pair,
							bool validAssociation) 
	{
		int currentGapIdx = int(std::floor(pair[0] / 2.0));
		int previousGapIdx = int(std::floor(pair[1] / 2.0));
		float currX, currY, prevX, prevY;
		ROS_INFO_STREAM("    pair (" << pair[0] << ", " << pair[1] << ")");

		if (validAssociation) 
		{
			if (pair[0] % 2 == 0)  // curr left
				currentGaps.at(currentGapIdx).getLCartesian(currX, currY);
			else
				currentGaps.at(currentGapIdx).getRCartesian(currX, currY);

			if (pair[1] % 2 == 0) 
			{
				previousGaps.at(previousGapIdx).getLCartesian(prevX, prevY);
				ROS_INFO_STREAM("    accepting transition of index " << previousGaps[previousGapIdx].leftGapPtModel_->getID());				
			} else 
			{
				previousGaps.at(previousGapIdx).getRCartesian(prevX, prevY);
				ROS_INFO_STREAM("    accepting transition of index " << previousGaps[previousGapIdx].rightGapPtModel_->getID());
			}

			ROS_INFO_STREAM("    from (" << prevX << ", " << prevY << ") to (" << currX << ", " << currY << ") with a distance of " << distMatrix[pair[0]][pair[1]]);
		} else {
			if (pair[0] % 2 == 0)  // curr left
				currentGaps.at(currentGapIdx).getLCartesian(currX, currY);
			else
				currentGaps.at(currentGapIdx).getRCartesian(currX, currY);

			if (pair[1] >=0) 
			{
				if (pair[1] % 2 == 0) 
				{ 
					previousGaps.at(previousGapIdx).getLCartesian(prevX, prevY);
					ROS_INFO_STREAM("    rejecting transition of index " << previousGaps[previousGapIdx].leftGapPtModel_->getID());
				} else 
				{
					previousGaps.at(previousGapIdx).getRCartesian(prevX, prevY);
					ROS_INFO_STREAM("    rejecting transition of index " << previousGaps[previousGapIdx].rightGapPtModel_->getID());					
				}
				ROS_INFO_STREAM("    from (" << prevX << ", " << prevY << ") to (" << currX << ", " << currY << ") with a distance of " << distMatrix[pair[0]][pair[1]]);
			} else 
			{
				ROS_INFO_STREAM("    rejecting, more current gaps than previous gaps");
			}
		}
	}

	void GapAssociator::instantiateNewModel(int i,
											std::vector<dynamic_gap::Gap> & currentGaps, 
											int & currentModelIdx_,
											const ros::Time & scanTime,
									 		const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,		 
                      				 		const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs)
	{
		float gapPtX = currentGapPoints[i][0];
		float gapPtY = currentGapPoints[i][1];

    	geometry_msgs::TwistStamped lastRbtVel = (!intermediateRbtVels.empty()) ? intermediateRbtVels[intermediateRbtVels.size() - 1] : geometry_msgs::TwistStamped();
	    geometry_msgs::TwistStamped lastRbtAcc = (!intermediateRbtAccs.empty()) ? intermediateRbtAccs[intermediateRbtAccs.size() - 1] : geometry_msgs::TwistStamped();

		int currentGapIdx = int(std::floor(i / 2.0));
		if (i % 2 == 0) 
		{   // curr left
			// currentGaps[currentGapIdx].leftGapPtModel = new dynamic_gap::StaticEstimator("left", currentModelIdx_, init_r, init_beta, 
			// 																						scanTime, lastRbtVel, lastRbtAcc);
			currentGaps[currentGapIdx].leftGapPtModel_ = 
						new dynamic_gap::RotatingFrameCartesianKalmanFilter("left", currentModelIdx_, gapPtX, gapPtY, 
																			scanTime, lastRbtVel, lastRbtAcc);				
		} else 
		{
			// currentGaps[currentGapIdx].rightGapPtModel_= new dynamic_gap::StaticEstimator("right", currentModelIdx_, init_r, init_beta, 
			// 																						scanTime, lastRbtVel, lastRbtAcc);
			currentGaps[currentGapIdx].rightGapPtModel_= 
						new dynamic_gap::RotatingFrameCartesianKalmanFilter("right", currentModelIdx_, gapPtX, gapPtY,
																			scanTime, lastRbtVel, lastRbtAcc);				
		}
	}

	void GapAssociator::handOffModel(int i,
									 std::vector<dynamic_gap::Gap> & currentGaps, 
									 const std::vector<dynamic_gap::Gap> & previousGaps,
									 std::vector<int> & pair)
	{
		int currentGapIdx = int(std::floor(i / 2.0));
		int previousGapIdx = int(std::floor(pair[1] / 2.0));

		if (pair[0] % 2 == 0)  // curr left
		{
			currentGaps[currentGapIdx].leftGapPtModel_ = (pair[1] % 2 == 0) ? previousGaps[previousGapIdx].leftGapPtModel_ :
																			  previousGaps[previousGapIdx].rightGapPtModel_;
		} else // curr right
		{
			currentGaps[currentGapIdx].rightGapPtModel_ = (pair[1] % 2 == 0) ? previousGaps[previousGapIdx].leftGapPtModel_ :
																			   previousGaps[previousGapIdx].rightGapPtModel_;
		} 		
	}

	void GapAssociator::assignModels(std::vector<int> & association, 
									 const std::vector< std::vector<float> > & distMatrix, 
									 std::vector<dynamic_gap::Gap> & currentGaps, 
									 const std::vector<dynamic_gap::Gap> & previousGaps,
									 int & currentModelIdx_,
                                     const ros::Time & scanTime, 
									 const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                      				 const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
									 bool print)
	{
		if (print) ROS_INFO_STREAM("[assignModels()]");
		// float start_time = ros::Time::now().toSec();
		// initializing models for current gaps
		// float gapPtX, gapPtY;

		if (print) ROS_INFO_STREAM("    number of observed gaps: " << currentGaps.size() << ", number of previous gaps: " << previousGaps.size());

		if (print) ROS_INFO_STREAM("	association size: " << association.size());

		if (print) ROS_INFO_STREAM("	association: ");
		if (print)
		{
			for (int i = 0; i < association.size(); i++)
				ROS_INFO_STREAM("		(" << i << ", " << association[i] << ")");
		}

		for (int i = 0; i < currentGapPoints.size(); i++) 
		{
			if (i < association.size()) // try association
			{
				int previousGapPtIdx = association[i];
				std::vector<int> pair{i, previousGapPtIdx};

				// if current gap pt has valid association and association is under distance threshold
				bool assoc_idx_in_range = previousGaps.size() > int(std::floor(pair[1] / 2.0));
				bool assoc_idx_out_of_range = (pair[1] < 0);
				bool assoc_dist_in_thresh = (distMatrix[pair[0]][pair[1]] <= assocThresh);
				bool validAssociation = !assoc_idx_out_of_range && assoc_dist_in_thresh; 
				if (validAssociation) 
				{
					//std::cout << "associating" << std::endl;	
					//std::cout << "distance under threshold" << std::endl;
					handOffModel(i, currentGaps, previousGaps, pair);
				} else
				{
					instantiateNewModel(i, currentGaps, currentModelIdx_, scanTime, intermediateRbtVels, intermediateRbtAccs);
				}
				if (print) printGapTransition(currentGaps, previousGaps, distMatrix, pair, validAssociation);
			} else // instantiate new model
			{
				instantiateNewModel(i, currentGaps, currentModelIdx_, scanTime, intermediateRbtVels, intermediateRbtAccs);
			}
			currentModelIdx_ += 1;
		}

		// delete old models
		/*
		// BROKEN RIGHT NOW
		for (int previousGapPtIdx = 0; previousGapPtIdx < previousGapPoints.size(); previousGapPtIdx++) 
		{
			bool deleteModel = true;
			int previousGapIdx = int(std::floor(previousGapPtIdx / 2.0));

			for (int currentGapPtIdx = 0; currentGapPtIdx < association.size(); currentGapPtIdx++)
			{
				if (association[currentGapPtIdx] == previousGapPtIdx) // or if association was rejected
					deleteModel = false;
			}

			if (deleteModel)
			{
				if (print) ROS_INFO_STREAM("	deleting previous gap model " << previousGapPtIdx << ", gap: " << previousGapIdx);
				if (previousGapPtIdx % 2 == 0)
					delete previousGaps[previousGapIdx].leftGapPtModel_;
				else
					delete previousGaps[previousGapIdx].rightGapPtModel_;
			}
		}
		*/

		// if (print) printGapAssociations(currentGaps, previousGaps, association, distMatrix);

		// ASSOCIATING MODELS
		// std::cout << "accepting associations" << std::endl;
		// for (int i = 0; i < association.size(); i++) 
		// {
		// 	//std::cout << "i " << i << std::endl;
		// 	// the values in associations are indexes for observed gaps
			
		// }

		//ROS_INFO_STREAM("assignModels time elapsed: " << ros::Time::now().toSec() - start_time); 
	}
        

	std::vector<int> GapAssociator::associateGaps(const std::vector< std::vector<float> > & distMatrix) 
	{
		// NEW ASSIGNMENT OBTAINED
		//float start_time = ros::Time::now().toSec();

		// std::cout << "obtaining new assignment" << std::endl;
		std::vector<int> association;
        if (distMatrix.size() > 0 && distMatrix[0].size() > 0) 
		{
			//std::cout << "solving" << std::endl;
            float cost = Solve(distMatrix, association);
			//std::cout << "done solving" << std::endl;
        }

		//ROS_INFO_STREAM("associateGaps time elapsed: " << ros::Time::now().toSec() - start_time);
        return association;
    }
	
	//********************************************************//
	// A single function wrapper for solving assignment problem.
	//********************************************************//
	float GapAssociator::Solve(const std::vector <std::vector<float> >& DistMatrix, std::vector<int>& Assignment)
	{
		unsigned int nRows = DistMatrix.size();
		unsigned int nCols = DistMatrix[0].size();

		float *distMatrixIn = new float[nRows * nCols];
		int *assignment = new int[nRows];
		float cost = 0.0;

		// Fill in the distMatrixIn. Mind the index is "i + nRows * j".
		// Here the cost matrix of size MxN is defined as a float precision array of N*M elements. 
		// In the solving functions matrices are seen to be saved MATLAB-internally in row-order.
		// (i.e. the matrix [1 2; 3 4] will be stored as a vector [1 3 2 4], NOT [1 2 3 4]).
		for (unsigned int i = 0; i < nRows; i++)
			for (unsigned int j = 0; j < nCols; j++)
				distMatrixIn[i + nRows * j] = DistMatrix[i][j];
		
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
