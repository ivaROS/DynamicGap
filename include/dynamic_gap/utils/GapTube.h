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
                tube_.clear();
                tube_.push_back(gap);
            }

            void addGap(Gap * gap)
            {
                tube_.push_back(gap);
            }

            Gap * getMostRecentGap()
            {
                return tube_.back();
            }

        private:
            std::vector<Gap *> tube_;
    };
}