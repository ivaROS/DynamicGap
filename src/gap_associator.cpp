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

#include <dynamic_gap/gap_associator.h>

namespace dynamic_gap {

	std::vector< std::vector<float>> obtainGapPoints(std::vector<dynamic_gap::Gap> gaps) {
		std::vector< std::vector<float>> points(2*gaps.size(), std::vector<float>(2));
		int count = 0;
		for (auto & g : gaps) {	
			// this is still true, I just don't know how to handle indexing right now
            // populating the coordinates of the gap points (in rbt frame) to compute distances
			//std::cout << "adding left points" << std::endl;
			//std::cout << "convex l dist: " << g.convex.convex_ldist << ", half scan: " << g.half_scan << ", convex l idx: " << g.convex.convex_lidx << std::endl;
			//std::cout << "convex r dist: " << g.convex.convex_rdist << ", half scan: " << g.half_scan << ", convex r idx: " << g.convex.convex_ridx << std::endl;
			// std::string print_string;					

			int lidx = g.LIdx();
			int ridx = g.RIdx();
			float ldist = g.LDist();
			float rdist = g.RDist();
			float left_x = rdist * cos(-((float) g.half_scan - ridx) / g.half_scan * M_PI);
			float left_y = rdist * sin(-((float) g.half_scan - ridx) / g.half_scan * M_PI);
			float right_x = ldist * cos(-((float) g.half_scan - lidx) / g.half_scan * M_PI);
			float right_y = ldist * sin(-((float) g.half_scan - lidx) / g.half_scan * M_PI);				

			points[count][0] = left_x;
			points[count][1] = left_y;
			count++;
			points[count][0] = right_x;
			points[count][1] = right_y;
			// print_string += ("left: (" + std::to_string(points[count - 1][0]) + ", " + std::to_string(points[count - 1][1]) + "), right: (" + std::to_string(points[count][0]) + ", " + std::to_string(points[count][1]) + "), ");
			count++;
			// ROS_INFO_STREAM(print_string);
        }
		return points;
	}

	std::vector<std::vector<double>> GapAssociator::obtainDistMatrix(std::vector<dynamic_gap::Gap> observed_gaps, 
															std::vector<dynamic_gap::Gap> previous_gaps) {
		double start_time = ros::Time::now().toSec(); 
		//std::cout << "number of current gaps: " << observed_gaps.size() << std::endl;
		//std::cout << "number of previous gaps: " << previous_gaps.size() << std::endl;
		// ROS_INFO_STREAM("getting previous points:");
		previous_gap_points = obtainGapPoints(previous_gaps);
		// ROS_INFO_STREAM("getting current points:");
        observed_gap_points = obtainGapPoints(observed_gaps);
        
		std::vector< std::vector<double> > distMatrix(observed_gap_points.size(), std::vector<double>(previous_gap_points.size()));
        //std::cout << "dist matrix size: " << distMatrix.size() << ", " << distMatrix[0].size() << std::endl;
		// populate distance matrix
		// ROS_INFO_STREAM("Distance matrix: ");
        for (int i = 0; i < distMatrix.size(); i++) {
            for (int j = 0; j < distMatrix[i].size(); j++) {
                double accum = 0;
                //std::cout << i << ", " << j <<std::endl;
                for (int k = 0; k < observed_gap_points[i].size(); k++) {
                    // std::cout << previous_gap_points[i][k] << ", " << observed_gap_points[j][k];
                    accum += pow(observed_gap_points[i][k] - previous_gap_points[j][k], 2);
                }
                //std::cout << "accum: " << accum << std::endl;
                distMatrix[i][j] = sqrt(accum);
                // ROS_INFO_STREAM(distMatrix[i][j] << ", ");
            }
			// ROS_INFO_STREAM("" << std::endl;
        }

		return distMatrix;

		// ROS_INFO_STREAM("obtainDistMatrix time elapsed: " << ros::Time::now().toSec() - start_time);
	}
	
	void printGapAssociations(std::vector<dynamic_gap::Gap> current_gaps, 
							  std::vector<dynamic_gap::Gap> previous_gaps, 
							  std::vector<int> association,
							  std::vector< std::vector<double> > distMatrix) {
        // std::cout << "printing associations" << std::endl;

        float curr_x, curr_y, prev_x, prev_y;
        for (int i = 0; i < association.size(); i++) {
            std::vector<int> pair{i, association[i]};
            ROS_INFO_STREAM_NAMED("laserScanCB", "pair (" << i << ", " << association[i] << ")");
            if (i >= 0 && association[i] >= 0) {
                int current_gap_idx = int(std::floor(pair[0] / 2.0));
                int previous_gap_idx = int(std::floor(pair[1] / 2.0));
                if (pair[0] % 2 == 0) {  // curr left
                    current_gaps.at(current_gap_idx).getRCartesian(curr_x, curr_y);
                } else { // curr right
                    current_gaps.at(current_gap_idx).getLCartesian(curr_x, curr_y);
                }
                if (pair[1] % 2 == 0) { // prev left
                    previous_gaps.at(previous_gap_idx).getRCartesian(prev_x, prev_y);
                } else { // prev right
                    previous_gaps.at(previous_gap_idx).getLCartesian(prev_x, prev_y);
                }
                ROS_INFO_STREAM_NAMED("laserScanCB", "From (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << distMatrix[pair[0]][pair[1]]);
            } else {
                ROS_INFO_STREAM_NAMED("laserScanCB", "From NULL to (" << curr_x << ", " <<  curr_y << ")");
            }
			
        }
    }

	void printGapTransition(std::vector<dynamic_gap::Gap> observed_gaps, 
							std::vector<dynamic_gap::Gap> previous_gaps,
							std::vector< std::vector<double> > distMatrix, 
							std::vector<int> pair,
							bool valid_assoc) {

		int current_gap_idx = int(std::floor(pair[0] / 2.0));
		int previous_gap_idx = int(std::floor(pair[1] / 2.0));
		float curr_x, curr_y, prev_x, prev_y;
		ROS_INFO_STREAM_NAMED("laserScanCB", "pair (" << pair[0] << ", " << pair[1] << ")");

		if (valid_assoc) {
			if (pair[0] % 2 == 0) {  // curr left
				observed_gaps.at(current_gap_idx).getRCartesian(curr_x, curr_y);
			} else {
				observed_gaps.at(current_gap_idx).getLCartesian(curr_x, curr_y);
			}

			if (pair[1] % 2 == 0) {
				previous_gaps.at(previous_gap_idx).getRCartesian(prev_x, prev_y);
				ROS_INFO_STREAM_NAMED("laserScanCB", "accepting transition of index " << previous_gaps[previous_gap_idx].right_model->get_index());
			} else {
				previous_gaps.at(previous_gap_idx).getLCartesian(prev_x, prev_y);
				ROS_INFO_STREAM_NAMED("laserScanCB","accepting transition of index " << previous_gaps[previous_gap_idx].left_model->get_index());
			}

			ROS_INFO_STREAM_NAMED("laserScanCB", "from (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << distMatrix[pair[0]][pair[1]]);
		} else {
			if (pair[0] % 2 == 0) {  // curr left
				observed_gaps.at(current_gap_idx).getRCartesian(curr_x, curr_y);
			} else {
				observed_gaps.at(current_gap_idx).getLCartesian(curr_x, curr_y);
			}

			if (pair[1] >=0) {
				if (pair[1] % 2 == 0) { 
					previous_gaps.at(previous_gap_idx).getRCartesian(prev_x, prev_y);
					ROS_INFO_STREAM_NAMED("laserScanCB", "rejecting transition of index " << previous_gaps[previous_gap_idx].right_model->get_index());
				} else {
					previous_gaps.at(previous_gap_idx).getLCartesian(prev_x, prev_y);
					ROS_INFO_STREAM_NAMED("laserScanCB", "rejecting transition of index " << previous_gaps[previous_gap_idx].left_model->get_index());
				}
				ROS_INFO_STREAM_NAMED("laserScanCB", "from (" << prev_x << ", " << prev_y << ") to (" << curr_x << ", " << curr_y << ") with a distance of " << distMatrix[pair[0]][pair[1]]);
			} else {
				ROS_INFO_STREAM_NAMED("laserScanCB", "rejecting, more current gaps than previous gaps");
			}
		}

	}

	void GapAssociator::assignModels(std::vector<int> association, 
									 std::vector< std::vector<double> > distMatrix, 
									 std::vector<dynamic_gap::Gap>& observed_gaps, 
									 std::vector<dynamic_gap::Gap> previous_gaps,
									 int * model_idx,
                                     const ros::Time & t_kf_update, 
									 const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied, 
                      				 const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied,
									 bool print)
	{
		double start_time = ros::Time::now().toSec();
		// initializing models for current gaps
		double init_r, init_beta;

    	geometry_msgs::TwistStamped last_ego_rbt_vel = (!ego_rbt_vels_copied.empty()) ? ego_rbt_vels_copied[ego_rbt_vels_copied.size() - 1] : geometry_msgs::TwistStamped();
	    geometry_msgs::TwistStamped last_ego_rbt_acc = (!ego_rbt_accs_copied.empty()) ? ego_rbt_accs_copied[ego_rbt_accs_copied.size() - 1] : geometry_msgs::TwistStamped();


		for (int i = 0; i < observed_gap_points.size(); i++) {
			init_r = sqrt(pow(observed_gap_points[i][0], 2) + pow(observed_gap_points[i][1],2));
			init_beta = std::atan2(observed_gap_points[i][1], observed_gap_points[i][0]);
			if (i % 2 == 0) {  // curr left
				observed_gaps[int(std::floor(i / 2.0))].right_model = new dynamic_gap::cart_model("right", *model_idx, init_r, init_beta, t_kf_update, last_ego_rbt_vel, last_ego_rbt_acc, *cfg_);
			} else {
				observed_gaps[int(std::floor(i / 2.0))].left_model = new dynamic_gap::cart_model("left", *model_idx, init_r, init_beta, t_kf_update, last_ego_rbt_vel, last_ego_rbt_acc, *cfg_);
			}
			*model_idx += 1;
		}

		// if (print) printGapAssociations(observed_gaps, previous_gaps, association, distMatrix);
		if (print) ROS_INFO_STREAM_NAMED("laserScanCB", "number of observed gaps: " << observed_gaps.size() << ", number of previous gaps: " << previous_gaps.size());

		// ASSOCIATING MODELS
		// std::cout << "accepting associations" << std::endl;
		for (int i = 0; i < association.size(); i++) {
			//std::cout << "i " << i << std::endl;
			// the values in associations are indexes for observed gaps
			int previous_gap_idx = association[i];
			std::vector<int> pair{i, previous_gap_idx};

			// if current gap pt has valid association and association is under distance threshold
			bool assoc_idx_in_range = previous_gaps.size() > int(std::floor(pair[1] / 2.0));
			bool assoc_idx_out_of_range = (pair[1] < 0);
			bool assoc_dist_in_thresh = distMatrix[pair[0]][pair[1]] <= assoc_thresh;
			bool valid_assoc = !assoc_idx_out_of_range && assoc_dist_in_thresh; 
			if (valid_assoc) {
				//std::cout << "associating" << std::endl;	
				//std::cout << "distance under threshold" << std::endl;
				
				if (pair[0] % 2 == 0) {  // curr left
					if (pair[1] % 2 == 0) { // prev left
						observed_gaps[int(std::floor(pair[0] / 2.0))].right_model = previous_gaps[int(std::floor(pair[1] / 2.0))].right_model;
					} else { // prev right;
						observed_gaps[int(std::floor(pair[0] / 2.0))].right_model = previous_gaps[int(std::floor(pair[1] / 2.0))].left_model;
						// ROS_INFO_STREAM("transitioning index " << previous_gaps[int(std::floor(pair[1] / 2.0))].left_model->get_index());
					}
				} else { // curr right
					if (pair[1] % 2 == 0) { // prev left
						observed_gaps[int(std::floor(pair[0] / 2.0))].left_model = previous_gaps[int(std::floor(pair[1] / 2.0))].right_model;
						// ROS_INFO_STREAM("transitioning index " << previous_gaps[int(std::floor(pair[1] / 2.0))].right_model->get_index());
					} else { // prev right
						observed_gaps[int(std::floor(pair[0] / 2.0))].left_model = previous_gaps[int(std::floor(pair[1] / 2.0))].left_model;
						// ROS_INFO_STREAM("transitioning index " << previous_gaps[int(std::floor(pair[1] / 2.0))].left_model->get_index());
					}
				} 
			}
			if (print) printGapTransition(observed_gaps, previous_gaps, distMatrix, pair, valid_assoc);
		}

		//ROS_INFO_STREAM("assignModels time elapsed: " << ros::Time::now().toSec() - start_time); 
	}
        

	std::vector<int> GapAssociator::associateGaps(std::vector< std::vector<double> > distMatrix) {
		// NEW ASSIGNMENT OBTAINED
		//double start_time = ros::Time::now().toSec();

		// std::cout << "obtaining new assignment" << std::endl;
		std::vector<int> association;
        if (distMatrix.size() > 0 && distMatrix[0].size() > 0) {
			//std::cout << "solving" << std::endl;
            double cost = Solve(distMatrix, association);
			//std::cout << "done solving" << std::endl;
        }

		//ROS_INFO_STREAM("associateGaps time elapsed: " << ros::Time::now().toSec() - start_time);
        return association;
    }
	
	//********************************************************//
	// A single function wrapper for solving assignment problem.
	//********************************************************//
	double GapAssociator::Solve(std::vector <std::vector<double> >& DistMatrix, std::vector<int>& Assignment)
	{
		unsigned int nRows = DistMatrix.size();
		unsigned int nCols = DistMatrix[0].size();

		double *distMatrixIn = new double[nRows * nCols];
		int *assignment = new int[nRows];
		double cost = 0.0;

		// Fill in the distMatrixIn. Mind the index is "i + nRows * j".
		// Here the cost matrix of size MxN is defined as a double precision array of N*M elements. 
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
	void GapAssociator::assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
	{
		double *distMatrix, *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
		bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix;
		int nOfElements, minDim, row, col;

		/* initialization */
		*cost = 0;
		for (row = 0; row<nOfRows; row++)
			assignment[row] = -1;

		/* generate working copy of distance Matrix */
		/* check if all matrix elements are positive */
		nOfElements = nOfRows * nOfColumns;
		distMatrix = (double *)malloc(nOfElements * sizeof(double));
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
	void GapAssociator::computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows)
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
	void GapAssociator::step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
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
	void GapAssociator::step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
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
	void GapAssociator::step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
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
	void GapAssociator::step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
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
	void GapAssociator::step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
	{
		double h, value;
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
