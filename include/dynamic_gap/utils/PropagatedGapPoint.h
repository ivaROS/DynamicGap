
namespace dynamic_gap
{
    class PropagatedGapPoint 
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
}