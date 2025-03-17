#pragma once

#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/GapTube.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>

#include <dynamic_gap/gap_association/GapAssociator.h>

namespace dynamic_gap 
{
    /**
    * \brief Class responsible for evaluating a gap's feasibility by forward-simulating the gap
    * according to its dynamics model and determining if it is feasible for the robot
    * to pass through the gap before it closes
    */
    class GapPropagator 
    {
        public: 
            GapPropagator(const DynamicGapConfig& cfg) { cfg_ = &cfg; gapAssociator_ = new GapAssociator(cfg); };

            ~GapPropagator() { delete gapAssociator_; }

            /**
            * \brief Set terminal range and bearing values for gap based on 
            * where gap crossed
            * \param gap incoming gap whose points we want to propagate
            */                 
            void propagateGapPoints(Gap * gap);

            void propagateGapPointsV2(const std::vector<Gap *> & gaps);

        private:
            void getGaps(std::vector<Gap *> & currentGaps,
                            const float & t_iplus1);

            void runGapAssociation(const std::vector<Gap *> & currentGaps, 
                                    const std::vector<Gap *> & previousGaps,
                                    const float & t_iplus1);

            void convertGapsToGapPoints(const std::vector<Gap *> & gaps);

            void assignUnGapIDsToGapPoints();

            bool isUngap(const Eigen::Vector4f & ptIState, const Eigen::Vector4f & ptJState);

            /**
            * \brief Rewind crossed gap to find last point in time in which
            * robot can fit through gap 
            * \param t crossing time of gap
            * \param gap incoming gap whose feasibility we want to evaluate
            * \return last point in time in which robot can fit through gap
            */
            float rewindGapPoints(const float & t, 
                                    Gap * gap);


            const DynamicGapConfig* cfg_; /**< Planner hyperparameter config list */        

            struct PropagatedGapPointComparator
            {
                bool operator() (PropagatedGapPoint * lhs, PropagatedGapPoint * rhs) const
                {
                    return lhs->getModel()->getGapBearing() < rhs->getModel()->getGapBearing();
                }
            };

            std::vector<PropagatedGapPoint *> propagatedGapPoints_;

            std::vector<GapTube *> gapTubes_;

            GapAssociator * gapAssociator_ = NULL; /**< Gap point associator */
            std::vector<std::vector<float>> distMatrix_; /**< Distance matrix for gaps */
            std::vector<int> assocation_; /**< Association vector for gaps */

        };
}