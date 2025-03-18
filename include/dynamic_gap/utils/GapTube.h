#pragma once

#include <dynamic_gap/utils/Gap.h>

namespace dynamic_gap
{
    /**
    * \brief Class for a single gap
    */
    class GapTube
    {
        public:
            GapTube(Gap * gap)
            {
                tube_.push_back(new Gap(*gap)); // deep copy
            }

            ~GapTube()
            {
                for (Gap * gap : tube_)
                {
                    delete gap;
                }
                tube_.clear();
            }

            void print()
            {
                Gap * gap;
                for (int i = 0; i < tube_.size(); i++)
                {
                    ROS_INFO_STREAM_NAMED("GapTube", "           Gap " << i << ": ");
                    gap = tube_.at(i);
                    ROS_INFO_STREAM_NAMED("GapTube", "                      start time: " << gap->getGapStart());
                    ROS_INFO_STREAM_NAMED("GapTube", "                      lifespan: " << gap->getGapLifespan());
                    ROS_INFO_STREAM_NAMED("GapTube", "                      left point:" << gap->getManipulatedLPosition().transpose());
                    ROS_INFO_STREAM_NAMED("GapTube", "                      left vel:" << gap->getManipulatedLVelocity().transpose());
                    ROS_INFO_STREAM_NAMED("GapTube", "		                left ID: (" << gap->getLeftGapPt()->getModel()->getID() << ")");                        
                    ROS_INFO_STREAM_NAMED("GapTube", "                      right point:" << gap->getManipulatedRPosition().transpose());
                    ROS_INFO_STREAM_NAMED("GapTube", "                      right vel:" << gap->getManipulatedRVelocity().transpose());
                    ROS_INFO_STREAM_NAMED("GapTube", "				        right ID: (" << gap->getRightGapPt()->getModel()->getID() << ")");
                    ROS_INFO_STREAM_NAMED("GapTube", "                      gap available: " << gap->isAvailable());
                }

            }

            void addGap(Gap * gap)
            {
                // should construct using propagated gap point
                tube_.push_back(new Gap(*gap)); // deep copy
            }

            Gap * getMostRecentGap()
            {
                return tube_.back();
            }

            std::vector<Gap *> getTube()
            {
                return tube_;
            }

            int size()
            {
                return tube_.size();
            }

            Gap * at(const int & idx)
            {
                if (idx < 0 || idx >= tube_.size())
                {
                    ROS_WARN_STREAM_NAMED("GapTube", "Index out of bounds");
                    return nullptr;
                }

                return tube_.at(idx);
            }

        private:
            std::vector<Gap *> tube_;
    };
}