
/**
* Adapted from https://github.com/mcximing/hungarian-algorithm-cpp/blob/master/Hungarian.h
*/
#pragma once

#include <dynamic_gap/gap_association/Associator.h>

namespace dynamic_gap
{
    /** 
    * \brief Class responsible for assocating gaps at consecutive time steps
	* to obtain a minimum distance pairing between gaps and subsequently
	* pass off gap estimator models.
    */	
	class GapPointAssociator : public Associator
	{
		public:
			/**
			* \brief Constructor with nodehandle and cfg
			*/
			GapPointAssociator(const DynamicGapConfig& cfg) {cfg_ = &cfg; assocThresh = cfg_->gap_assoc.assoc_thresh; };
			
			void updateParams(const EstimationParameters & estParams);

			/**
			* \brief Populate distance matrix between points in current gaps and points in previous gaps.
			* 
			* \param currentGaps current set of gaps
			* \param previousGaps previous set of gaps
			* \return distance matrix: 2D matrix with entries that represent distance between gap points at corresponding indices 
			*/		
			std::vector<std::vector<float>> populateDistMatrix(const std::vector<Gap *> & currentGaps, 
																const std::vector<Gap *> & previousGaps);
			
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
							std::vector<Gap *>& currentGaps, 
							const std::vector<Gap *> & previousGaps,
							int & currentModelIdx,
							const ros::Time & scanTime, 
							const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
							const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs);

		private:

			float calculateDistance(const std::vector<float> & currentGapPoint,
									const std::vector<float> & previousGapPoint);

			/**
			* \brief Populate vector of gap points for a set of gaps
			* \param gaps incoming set of gaps
			* \return vector of gap points
			*/			
			std::vector< std::vector<float>> obtainGapPoints(const std::vector<Gap *> & gaps);

			/**
			* \brief Helper function for handing off a model from a previous gap point to a current gap point
			* \param pair pair of indices for associated previous and current gap points
			* \param currentGaps current set of gaps
			* \param previousGaps previous set of gaps	
			*/		
			void handOffModel(const std::vector<int> & pair,
							const std::vector<Gap *> & currentGaps, 
							const std::vector<Gap *> & previousGaps);	
			
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
										const std::vector<Gap *> & currentGaps, 
										int & currentModelIdx,
										const ros::Time & scanTime,
										const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels,		 
										const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs);
		
			std::vector< std::vector<float>> previousGapPoints; /**< sequence of points within previous gaps */
			std::vector< std::vector<float>> currentGapPoints; /**< sequence of points within current gaps */
			const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
			float assocThresh; /**<  maximum distance threshold for which we will consider an association between models valid */
			EstimationParameters estParams_; /**< parameters for gap estimation */
	};
}