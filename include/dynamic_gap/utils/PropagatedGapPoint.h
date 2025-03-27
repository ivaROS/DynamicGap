#pragma once

#include <ros/ros.h>

#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/gap_estimation/Estimator.h>

namespace dynamic_gap
{
    class PropagatedGapPoint 
    {
        public:
            PropagatedGapPoint(Estimator * model, 
                                const std::string & frame,
                                const int & ungapID,
                                // const int & scanIdx, 
                                const bool & isLeft) : 
                                model_(model), 
                                frame_(frame),
                                ungapID_(ungapID),
                                // scanIdx_(scanIdx), 
                                isLeft_(isLeft), 
                                isRight_(!isLeft) 
            {
                ROS_INFO_STREAM_NAMED("Gap", "in PropagatedGapPoint constructor");
                // ROS_INFO_STREAM_NAMED("Gap", "  scanIdx: " << scanIdx);
                ROS_INFO_STREAM_NAMED("Gap", " orig state: " << model_->getGapState().transpose());
                ROS_INFO_STREAM_NAMED("Gap", " manip state: " << model_->getManipGapState().transpose());

            }
            // PropagatedGapPoint(Estimator * model, const int & scanIdx, const int & ungapID) : model_(model), scanIdx_(scanIdx), ungapID_(ungapID) {}

            void propagate(const float & stept) 
            { 
                ROS_INFO_STREAM_NAMED("Gap", " pre-propagate manip state: " << model_->getManipGapState().transpose());
                ROS_INFO_STREAM_NAMED("Gap", "  propagating manip gap point...");
                model_->gapStatePropagate(stept);
                model_->manipGapStatePropagate(stept); 
                ROS_INFO_STREAM_NAMED("Gap", " post-propagate manip state: " << model_->getManipGapState().transpose());

                // setScanIdx(theta2idx(model_->getGapBearing()));                    
                reset();
            }

            ~PropagatedGapPoint() 
            {
                // do not want to delete model
            }

            Estimator * getModel() { return model_; }
            Estimator * getModel() const { return model_; }

            void setUngapID(const int & ungapID) { ungapID_ = ungapID; }
            int getUngapID() { return ungapID_; }

            // int getScanIdx() { return scanIdx_; }
            // int getScanIdx() const { return scanIdx_; }

            bool isLeft() const { return isLeft_; }

            bool isRight() const { return isRight_; }

            bool isAssignedToGap() { return isAssignedToGap_; }

            void assignToGap() { isAssignedToGap_ = true; }

            std::string getFrame() { return frame_; }

        private:
            
            void reset() 
            { 
                isAssignedToGap_ = false; 
            }

            /**
            * \brief Parameters of original form of gap
            */        
            struct Orig
            {
                int idx_ = -1; /**< Original gap point index */
                float range_ = -1.0; /**< Original gap point range */
            } orig;

            /**
            * \brief Parameters of manipulated form of gap
            */
            struct Manip
            {
                int idx_ = -1; /**< Manipulated gap point index */
                float range_ = -1.0; /**< Manipulated gap point range */
            } manip;

            Estimator * model_ = NULL;
            int ungapID_ = -1; // set one time at init
            // int scanIdx_ = -1; // update
            bool isLeft_ = false; // set one time at init
            bool isRight_ = false; // set one time at init
            bool isAssignedToGap_ = false; // update
            std::string frame_ = ""; // set one time at init
    };    
}