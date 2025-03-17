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

            void addGap(Gap * gap)
            {
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

        private:
            std::vector<Gap *> tube_;
    };
}