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
                tube_.push_back(gap);
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
                tube_.push_back(gap);
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