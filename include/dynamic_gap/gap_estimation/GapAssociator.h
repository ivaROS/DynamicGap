///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class HungarianAlgorithm.
// 
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
// 
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
// 

#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
// #include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()
#include <chrono>

#include <Eigen/Core>

// using namespace std;

namespace dynamic_gap
{
	class GapAssociator
	{
	public:
		GapAssociator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; assocThresh = cfg_->gap_assoc.assoc_thresh; };
		
		std::vector<std::vector<float>> obtainDistMatrix(const std::vector<dynamic_gap::Gap> & currentGaps, 
														 const std::vector<dynamic_gap::Gap> & previousGaps);
		std::vector<int> associateGaps(const std::vector< std::vector<float> > & distMatrix);
        void assignModels(const std::vector<int> & association, 
						const std::vector< std::vector<float> > & distMatrix, 
						std::vector<dynamic_gap::Gap>& currentGaps, 
						const std::vector<dynamic_gap::Gap> & previousGaps,
						int & currentModelIdx_,
						const ros::Time & scanTime, 
						const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
						const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs,
						const bool & rawGaps);

	private:
		const DynamicGapConfig* cfg_;
		float assocThresh;
		
		void handOffModel(int i,
							std::vector<dynamic_gap::Gap> & currentGaps, 
							const std::vector<dynamic_gap::Gap> & previousGaps,
							std::vector<int> & pair);	
		void instantiateNewModel(int i,
								std::vector<dynamic_gap::Gap> & currentGaps, 
								int & currentModelIdx_,
								const ros::Time & scanTime,
								const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,		 
								const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs);

		float Solve(const std::vector <std::vector<float> >& DistMatrix, std::vector<int>& Assignment);
		void assignmentoptimal(int *assignment, float *cost, float *distMatrix, int nOfRows, int nOfColumns);
		void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
		void computeassignmentcost(int *assignment, float *cost, float *distMatrix, int nOfRows);
		void step2a(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step2b(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step3(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step4(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
		void step5(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	
		std::vector< std::vector<float>> previousGapPoints, currentGapPoints;

	};
}