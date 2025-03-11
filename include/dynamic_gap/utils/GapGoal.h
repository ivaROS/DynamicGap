

namespace dynamic_gap
{
    /** 
    * \brief Class for a gap goal
    */
    class GapGoal
    {
        public:
            GapGoal() {}

            void setOrigGoalPos(const Eigen::Vector2f & goalPt)
            {
                orig.x_ = goalPt[0];
                orig.y_ = goalPt[1];
            }

            void setOrigGoalVel(const Eigen::Vector2f & goalVel)
            {
                vx_ = goalVel[0];
                vy_ = goalVel[1];
            }

            void setTermGoalPos(const Eigen::Vector2f & goalPt)
            {
                term.x_ = goalPt[0];
                term.y_ = goalPt[1];
            }

            float getOrigGoalPosX() const
            {
                return orig.x_;
            }

            float getOrigGoalPosY() const
            {
                return orig.y_;
            }

            Eigen::Vector2f getOrigGoalPos() const
            {
                return Eigen::Vector2f(orig.x_, orig.y_);
            }

            Eigen::Vector2f getOrigGoalVel() const
            {
                return Eigen::Vector2f(vx_, vy_);
            }

            float getTermGoalPosX() const
            {
                return term.x_;
            }

            float getTermGoalPosY() const
            {
                return term.y_;
            }

            Eigen::Vector2f getTermGoalPos() const
            {
                return Eigen::Vector2f(term.x_, term.y_);
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
    };
}