

namespace dynamic_gap
{
    /** 
    * \brief Class for a gap goal
    */
    class GapGoal
    {
        public:
            GapGoal()
            {

            }

        private:

            float t_intercept_ = 0.0; /**< Time to intercept gap goal */
            float gamma_intercept_ = 0.0; /**< Angle to intercept gap goal */

            float vx_ = 0.0; /**< Original gap goal x-vel */
            float vy_ = 0.0; /**< Original gap goal y-vel */

            struct Orig
            {
                float x_ = 0.0; /**< Original gap goal x-value */
                float y_ = 0.0; /**< Original gap goal y-value */
            } orig;

            struct Term
            {
                float x_ = 0.0; /**< Terminal gap goal x-value */
                float y_ = 0.0; /**< Terminal gap goal y-value */
            } term;
    }
}