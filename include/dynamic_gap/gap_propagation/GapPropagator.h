#pragma once

#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>


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
            GapPropagator(const DynamicGapConfig& cfg) {cfg_ = &cfg;};

            /**
            * \brief Set terminal range and bearing values for gap based on 
            * where gap crossed
            * \param gap incoming gap whose points we want to propagate
            */                 
            void propagateGapPoints(Gap * gap);

            void propagateGapPointsV2(const std::vector<Gap *> & gaps);

        private:
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

            struct PropagatedGapPoint 
            {
                public:
                    PropagatedGapPoint(Estimator * model, const int & scanIdx, const bool & isLeft) : model_(model), scanIdx_(scanIdx), isLeft_(isLeft), isRight_(!isLeft) {}
                    // PropagatedGapPoint(Estimator * model, const int & scanIdx, const int & ungapID) : model_(model), scanIdx_(scanIdx), ungapID_(ungapID) {}

                    void propagate(const float & stept) 
                    { 
                        model_->gapStatePropagate(stept); 
                        setScanIdx(theta2idx(model_->getGapBearing()));                    
                        reset();
                    }

                    ~PropagatedGapPoint() 
                    {
                        // do not want to delete model
                    }

                    Estimator * getModel() { return model_; }

                    void setUngapID(const int & ungapID) { ungapID_ = ungapID; }
                    int getUngapID() { return ungapID_; }

                    int getScanIdx() { return scanIdx_; }
                    int getScanIdx() const { return scanIdx_; }

                    bool isLeft() { return isLeft_; }

                    bool isRight() { return isRight_; }

                    bool isAssignedToGap() { return isAssignedToGap_; }

                    void assignToGap() { isAssignedToGap_ = true; }

                private:
                    
                    void reset() 
                    { 
                        isAssignedToGap_ = false; 
                    }

                    void setScanIdx(const int & scanIdx) { scanIdx_ = scanIdx; }

                    Estimator * model_ = NULL;
                    int ungapID_ = -1; // set one time at init
                    int scanIdx_ = -1; // update
                    bool isLeft_ = false; // set one time at init
                    bool isRight_ = false; // set one time at init
                    bool isAssignedToGap_ = false; // update
            };

            struct PropagatedGapPointComparator
            {
                bool operator() (PropagatedGapPoint * lhs, PropagatedGapPoint * rhs) const
                {
                    return lhs->getScanIdx() < rhs->getScanIdx();
                }
            };

            std::vector<PropagatedGapPoint *> gapPoints_;
        
        };
}