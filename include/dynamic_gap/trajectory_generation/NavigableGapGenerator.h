#pragma once

#include <dynamic_gap/utils/Gap.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dynamic_gap 
{
    /**
    * \brief Class responsible for generating navigable gap region through which we will generate
    *        the local gap trajectories
    */    
    class NavigableGapGenerator 
    {
        public:
            NavigableGapGenerator(const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; };

            void generateNavigableGap(dynamic_gap::Gap * gap);

        private:

            void parallelNavigation();

            void purePursuit();

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
    };
}