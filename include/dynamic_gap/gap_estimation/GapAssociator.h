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

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
// #include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()
#include <Eigen/Core>

// using namespace std;

namespace dynamic_gap
{
	class GapAssociator
	{
	public:
		GapAssociator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; assoc_thresh = cfg_->gap_assoc.assoc_thresh; };
		
		std::vector<std::vector<float>> obtainDistMatrix(const std::vector<dynamic_gap::Gap> & observed_gaps, 
														  const std::vector<dynamic_gap::Gap> & previous_gaps);
		std::vector<int> associateGaps(const std::vector< std::vector<float> > & distMatrix);
        void assignModels(std::vector<int> & association, 
						  const std::vector< std::vector<float> > & distMatrix, 
							std::vector<dynamic_gap::Gap>& observed_gaps, 
							const std::vector<dynamic_gap::Gap> & previous_gaps, 
							int * model_idx, 
							const ros::Time & t_kf_update, 
							const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied, 
                      		const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied,
					  		bool print);

	private:
		const DynamicGapConfig* cfg_;
		float assoc_thresh;
		float Solve(const std::vector <std::vector<float> >& DistMatrix, std::vector<int>& Assignment);
		void assignmentoptimal(int *assignment, float *cost, float *distMatrix, int nOfRows, int nOfColumns);
		void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
		void computeassignmentcost(int *assignment, float *cost, float *distMatrix, int nOfRows);
		void step2a(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step2b(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step3(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step4(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
		void step5(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	
		std::vector< std::vector<float>> previous_gap_points;
		std::vector< std::vector<float>> observed_gap_points;

	};
}