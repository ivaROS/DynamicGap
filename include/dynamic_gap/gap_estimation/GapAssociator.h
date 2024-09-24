
/**
* Adapted from https://github.com/mcximing/hungarian-algorithm-cpp/blob/master/Hungarian.h
*/
#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cfloat> // for DBL_MAX
#include <cmath>  // for fabs()
#include <chrono>

#include <Eigen/Core>

namespace dynamic_gap
{
    /** 
    * \brief Class responsible for assocating gaps at consecutive time steps
	* to obtain a minimum distance pairing between gaps and subsequently
	* pass off gap estimator models.
    */	
	class GapAssociator
	{
	public:
		/**
		* \brief Constructor with nodehandle and cfg
		*/
		GapAssociator(const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; assocThresh = cfg_->gap_assoc.assoc_thresh; };
		
		/**
		* \brief Populate distance matrix between points in current gaps and points in previous gaps.
		* 
		* \param currentGaps current set of gaps
		* \param previousGaps previous set of gaps
		* \return distance matrix: 2D matrix with entries that represent distance between gap points at corresponding indices 
		*/		
		std::vector<std::vector<float>> obtainDistMatrix(const std::vector<dynamic_gap::Gap *> & currentGaps, 
														 const std::vector<dynamic_gap::Gap *> & previousGaps);
		
		/**
		* \brief Obtain minimum distance association between current gap points and previous gap points 
		* \param distMatrix populated distance matrix
		* \return minimum distance association
		*/				
		std::vector<int> associateGaps(const std::vector< std::vector<float> > & distMatrix);
        
		/**
		* \brief Function for handling the transfer of gap estimator models from previous gaps to current gaps
		* \param association minimum distance association
		* \param distMatrix populated distance matrix
		* \param currentGaps current set of gaps
		* \param previousGaps previous set of gaps		
		* \param currentModelIdx counter for model ID
		* \param scanTime ROS timestamp at which scan is read in to assign to models
		* \param intermediateRbtVels sequence of ego-robot velocities received since last model update
		* \param intermediateRbtAccs sequence of ego-robot accelerations received since last model update
		*/				
		void assignModels(const std::vector<int> & association, 
						  const std::vector< std::vector<float> > & distMatrix, 
						  std::vector<dynamic_gap::Gap *>& currentGaps, 
						  const std::vector<dynamic_gap::Gap *> & previousGaps,
						  int & currentModelIdx,
						  const ros::Time & scanTime, 
						  const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
						  const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs);

	private:

		/**
		* \brief Populate vector of gap points for a set of gaps
		* \param gaps incoming set of gaps
		* \return vector of gap points
		*/			
		std::vector< std::vector<float>> obtainGapPoints(const std::vector<dynamic_gap::Gap *> & gaps);

		/**
		* \brief Helper function for handing off a model from a previous gap point to a current gap point
		* \param pair pair of indices for associated previous and current gap points
		* \param currentGaps current set of gaps
		* \param previousGaps previous set of gaps	
		*/		
		void handOffModel(const std::vector<int> & pair,
						  const std::vector<dynamic_gap::Gap *> & currentGaps, 
						  const std::vector<dynamic_gap::Gap *> & previousGaps);	
		
		/**
		* \brief Helper function for instantiating a new model for a current gap point
		* \param i index for current gap point that needs new model
		* \param currentGaps current set of gaps
		* \param currentModelIdx counter for model ID
		* \param scanTime ROS timestamp at which scan is read in to assign to models
		* \param intermediateRbtVels sequence of ego-robot velocities received since last model update
		* \param intermediateRbtAccs sequence of ego-robot accelerations received since last model update
		*/							  
		void instantiateNewModel(const int & i,
									const std::vector<dynamic_gap::Gap *> & currentGaps, 
									int & currentModelIdx,
									const ros::Time & scanTime,
									const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,		 
									const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs);

		/**
		* \brief A single function wrapper for solving rectangular assignment problem
		* \param DistMatrix populated distance matrix
		* \param Assignment minimum distance association
		* \return total "cost" of minimum distance association
		*/
		float Solve(const std::vector <std::vector<float> >& DistMatrix, std::vector<int>& Assignment);

		/**
		* \brief Solve optimal solution for assignment problem using Munkres algorithm, also known as Hungarian Algorithm.
		* \param assignment minimum distance association
		* \param cost total "cost" of minimum distance association
		* \param distMatrix populated distance matrix
		* \param nOfRows number of rows in distMatrix
		* \param nOfColumns number of columns in distMatrix
		*/
		void assignmentoptimal(int *assignment, float *cost, float *distMatrix, int nOfRows, int nOfColumns);
		
		/**
		* \brief Inner contents of Hungarian algorithm.
		*/
		void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
		
		/**
		* \brief Inner contents of Hungarian algorithm.
		*/		
		void computeassignmentcost(int *assignment, float *cost, float *distMatrix, int nOfRows);
		
		/**
		* \brief Inner contents of Hungarian algorithm.
		*/		
		void step2a(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		
		/**
		* \brief Inner contents of Hungarian algorithm.
		*/		
		void step2b(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		
		/**
		* \brief Inner contents of Hungarian algorithm.
		*/		
		void step3(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
		
		/**
		* \brief Inner contents of Hungarian algorithm.
		*/		
		void step4(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
		
		/**
		* \brief Inner contents of Hungarian algorithm.
		*/		
		void step5(int *assignment, float *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	
		std::vector< std::vector<float>> previousGapPoints; /**< sequence of points within previous gaps */
		std::vector< std::vector<float>> currentGapPoints; /**< sequence of points within current gaps */
		const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
		float assocThresh; /**<  maximum distance threshold for which we will consider an association between models valid */
	};
}