

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
                origGoalPos_ = Eigen::Vector2f(0.0, 0.0);
                origGoalVel_ = Eigen::Vector2f(0.0, 0.0);
                termGoalPos_ = Eigen::Vector2f(0.0, 0.0);
            }

            void setOrigGoalPos(const Eigen::Vector2f & goalPt)
            {
                origGoalPos_ = goalPt;
            }

            void setOrigGoalVel(const Eigen::Vector2f & goalVel)
            {
                origGoalVel_ = goalVel;
            }

            void setTermGoalPos(const Eigen::Vector2f & goalPt)
            {
                termGoalPos_ = goalPt;
            }

            float getOrigGoalPosX() const
            {
                return origGoalPos_[0];
            }

            float getOrigGoalPosY() const
            {
                return origGoalPos_[1];
            }

            Eigen::Vector2f getOrigGoalPos() const
            {
                return origGoalPos_;
            }

            Eigen::Vector2f getOrigGoalVel() const
            {
                return origGoalVel_;
            }

            float getTermGoalPosX() const
            {
                return termGoalPos_[0];
            }

            float getTermGoalPosY() const
            {
                return termGoalPos_[1];
            }

            Eigen::Vector2f getTermGoalPos() const
            {
                return termGoalPos_;
            }

        private:

            float t_intercept_ = 0.0; /**< Time to intercept gap goal */
            float gamma_intercept_ = 0.0; /**< Angle to intercept gap goal */

            // float vx_ = 0.0; /**< Original gap goal x-vel */
            // float vy_ = 0.0; /**< Original gap goal y-vel */

            Eigen::Vector2f origGoalPos_; /**< Original gap goal position */

            Eigen::Vector2f origGoalVel_; /**< Original gap goal velocity */

            Eigen::Vector2f termGoalPos_; /**< Terminal gap goal position */

            // struct Orig
            // {
            //     float x_ = 0.0; /**< Original gap goal x-value */
            //     float y_ = 0.0; /**< Original gap goal y-value */
            // } orig;

            // struct Term
            // {
            //     float x_ = 0.0; /**< Terminal gap goal x-value */
            //     float y_ = 0.0; /**< Terminal gap goal y-value */
            // } term;
    };
}