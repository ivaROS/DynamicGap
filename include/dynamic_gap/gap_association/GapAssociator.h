
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
			std::vector<std::vector<float>> obtainDistMatrix(const std::vector<Gap *> & currentGaps, 
															const std::vector<Gap *> & previousGaps);
			

		private:

			/**
			* \brief Populate vector of gap points for a set of gaps
			* \param gaps incoming set of gaps
			* \return vector of gap points
			*/			
			std::vector< std::vector<float>> obtainGapPoints(const std::vector<Gap *> & gaps);

			std::vector< std::vector<float>> previousGapPoints; /**< sequence of points within previous gaps */
			std::vector< std::vector<float>> currentGapPoints; /**< sequence of points within current gaps */
			const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
			float assocThresh; /**<  maximum distance threshold for which we will consider an association between models valid */
	};
}