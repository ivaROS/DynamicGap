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

#ifndef GAP_ASSOCIATOR_H
#define GAP_ASSOCIATOR_H

#include <dynamic_gap/gap.h>
#include <dynamic_gap/dynamicgap_config.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>

using namespace std;

namespace dynamic_gap
{
	class GapAssociator
	{
	public:
		GapAssociator(){};
		~GapAssociator(){};

		GapAssociator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; assoc_thresh = cfg_->gap_assoc.assoc_thresh; };
		std::vector<int> associateGaps(vector< vector<double> > distMatrix);
        void assignModels(std::vector<int> association, vector< vector<double> > distMatrix, std::vector<dynamic_gap::Gap>& observed_gaps, std::vector<dynamic_gap::Gap> previous_gaps, geometry_msgs::Twist current_rbt_vel, int * model_idx, bool print);
		vector<vector<double>> obtainDistMatrix(std::vector<dynamic_gap::Gap> observed_gaps, std::vector<dynamic_gap::Gap> previous_gaps, std::string ns);


	private:
		const DynamicGapConfig* cfg_;
		double assoc_thresh;
		double Solve(vector <vector<double> >& DistMatrix, vector<int>& Assignment);
		void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
		void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
		void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
		void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
		void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	
		std::vector< std::vector<float>> previous_gap_points;
		std::vector< std::vector<float>> observed_gap_points;

	};
}


#endif