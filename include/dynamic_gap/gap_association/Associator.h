
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
// #include <chrono>

#include <Eigen/Core>

namespace dynamic_gap
{
    class Associator 
    {
        public: 
            Associator() {};

            Associator(const DynamicGapConfig& cfg) { };

			/**
			* \brief Obtain minimum distance association between current gap points and previous gap points 
			* \param distMatrix populated distance matrix
			* \return minimum distance association
			*/				
			std::vector<int> associate(const std::vector< std::vector<float> > & distMatrix);
			
			/**
			* \brief Populate distance matrix between points in current gaps and points in previous gaps.
			* 
			* \param currentGaps current set of gaps
			* \param previousGaps previous set of gaps
			* \return distance matrix: 2D matrix with entries that represent distance between gap points at corresponding indices 
			*/		
			virtual std::vector<std::vector<float>> populateDistMatrix(const std::vector<Gap *> & currentGaps, 
					        										    const std::vector<Gap *> & previousGaps) = 0;

        protected:

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
    };
}