
/**
* Adapted from https://github.com/mcximing/hungarian-algorithm-cpp/blob/master/Hungarian.h
*/
#pragma once

#include <dynamic_gap/gap_association/Associator.h>

#include <dynamic_gap/utils/GapTube.h>

namespace dynamic_gap
{
    /** 
    * \brief Class responsible for assocating gaps at consecutive time steps
	* to obtain a minimum distance pairing between gaps and subsequently
	* pass off gap estimator models.
    */	
	class GapAssociator : public Associator
	{
		public:
			/**
			* \brief Constructor with nodehandle and cfg
			*/
			GapAssociator(const DynamicGapConfig& cfg) 
            {   
                cfg_ = &cfg; 
                // assocThresh = cfg_->gap_assoc.assoc_thresh; 
            };
			
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
			void assignGaps(const std::vector<int> & association, 
							const std::vector< std::vector<float> > & distMatrix, 
							const std::vector<Gap *> & currentGaps, 
							const std::vector<Gap *> & previousGaps,
							std::vector<GapTube *> & gapTubes,
							const float & t_current);

		private:

			float calculateDistance(Gap * currentGap, Gap * previousGap);

			/**
			* \brief Populate vector of gap points for a set of gaps
			* \param gaps incoming set of gaps
			* \return vector of gap points
			*/			
			std::vector< std::vector<float>> obtainGapPoints(const std::vector<Gap *> & gaps);

			const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
			float assocThresh = 0.5; /**<  maximum distance threshold for which we will consider an association between models valid */
	};
}