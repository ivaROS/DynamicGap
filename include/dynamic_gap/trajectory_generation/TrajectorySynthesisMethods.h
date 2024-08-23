#pragma once

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace dynamic_gap 
{
    typedef boost::array<float, 8> robotAndGapState; /**< state vector which includes 2D robot, left gap point, right gap point, and gap goal positions */

    /**
    * \brief Structure for generating rotational potential field that provides
    * collision-free trajectories for convex gaps in static environments
    */
    struct PolarGapField
    {
        float xLeft_; /**< x-position of left gap point */
        float xRight_; /**< x-position of right gap point */
        float yLeft_; /**< y-position of left gap point */
        float yRight_; /**< y-position of right gap point */  
        float xGoal_; /**< x-position of gap goal point */
        float yGoal_; /**< y-position of gap goal point */
        float sigma_; /**< standard deviation-ish parameter used in expontential term of potential field */
        bool radial_; /**< boolean for if gap is radial*/
        float xRbtInit_; /**< initial x-position of robot */
        float yRbtInit_; /**< initial y-position of robot */   
        float vRbtLinMax_; /**< maximum linear velocity of robot */
        float aRbtLinMax_; /**< maximum linear acceleration of robt */
        Eigen::Matrix2f Rpi2_; /**< rotation matrix for pi/2 */
        Eigen::Matrix2f Rnegpi2_; /**< rotation matrix for -pi/2 */

        PolarGapField(const float & xRight, 
                        const float & xLeft, 
                        const float & yRight, 
                        const float & yLeft, 
                        const float & xGoal, 
                        const float & yGoal, 
                        const bool & radial, 
                        const float & xRbtInit, 
                        const float & yRbtInit,
                        const float & vRbtLinMax, 
                        const float & aRbtLinMax)
            : xRight_(xRight), xLeft_(xLeft), yRight_(yRight), yLeft_(yLeft), xGoal_(xGoal), yGoal_(yGoal), 
              radial_(radial), xRbtInit_(xRbtInit), yRbtInit_(yRbtInit), vRbtLinMax_(vRbtLinMax), aRbtLinMax_(aRbtLinMax) 
        {
            float rotAngle = M_PI / 2;
            Rpi2_ << std::cos(rotAngle), -std::sin(rotAngle), 
                     std::sin(rotAngle), std::cos(rotAngle);
            Rnegpi2_ << std::cos(-rotAngle), -std::sin(-rotAngle), 
                         std::sin(-rotAngle), std::cos(-rotAngle);

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "POLAR GAP FIELD");
        }

        /**
        * \brief Helper function for clipping velocities to maximum allowed velocities
        * \param rbtVel current robot velocity
        */
        void clipVelocities(Eigen::Vector2f & rbtVel) 
        {
            // std::cout << "in clipVelocities with " << vX << ", " << vY << std::endl;
            // Eigen::Vector2f origVel(vX, vY);
            float speedX = std::abs(rbtVel[0]);
            float speedY = std::abs(rbtVel[1]);
            if (speedX <= vRbtLinMax_ && speedY <= vRbtLinMax_) 
            {
                // std::cout << "not clipping" << std::endl;
                return;
            } else {
                // std::cout << "max: " << vx_absmax << ", norm: " << origVel.norm() << std::endl;
                // Eigen::Vector2f clipVel = vRbtLinMax_ * origVel / std::max(speedX, speedY);
                rbtVel = epsilonDivide(vRbtLinMax_ * rbtVel,  std::max(speedX, speedY));
                // return clipVel;
            }
        }

        /**
        * \brief () operator for rotational potential field that updates trajectory state
        * \param x trajectory state
        * \param dxdt trajectory state rate of change
        * \param t current timestep
        */
        void operator() (const robotAndGapState & x, robotAndGapState & dxdt, const float & t)
        {
            // IN HERE xRight_,yRight_ SHOULD BE RIGHT FROM ROBOT POV, xLeft_,yLeft_ SHOULD BE LEFT FROM ROBOT POV

            Eigen::Vector2f pLeft(xLeft_, yLeft_);
            Eigen::Vector2f pRight(xRight_, yRight_);

            if (getSweptLeftToRightAngle(pLeft, pRight) < M_PI)
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "don't need to switch");
            } else
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "now need to switch");
                std::swap(yRight_, yLeft_);
                std::swap(xRight_, xLeft_);
                pRight << xRight_, yRight_;
                pLeft << xLeft_, yLeft_;
            }

            /*
            if (atan2(yRight_, xRight_) > atan2(yLeft_, xLeft_)) 
            {
                std::swap(yRight_, yLeft_);
                std::swap(xRight_, xLeft_);
            }
            */

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   t: " << t);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rbt_0: (" << xRbtInit_ << ", " << yRbtInit_ << ")");

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rbt: (" << x[0] << ", " << x[1] << ")");

            Eigen::Vector2f rbtPosn(x[0], x[1]);

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   pRight: " << pRight[0] << ", " << pRight[1]);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   pLeft: " << pLeft[0] << ", " << pLeft[1]);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   p_goal: " << xGoal_ << ", " << yGoal_);

            Eigen::Vector2f rbtToLeft = pLeft - rbtPosn;
            Eigen::Vector2f rbtToRight = pRight - rbtPosn;

            Eigen::Vector2f gapGoal(xGoal_, yGoal_);
            Eigen::Vector2f rbtToGoal = gapGoal - rbtPosn;

            float rbtToGoalDistance = rbtToGoal.norm();
            float thetaLeft = atan2(yLeft_, xLeft_);
            float thetaRight = atan2(yRight_, xRight_);
            float thetaRbt = atan2(x[1], x[0]);
            float thetaRbtToGoal = atan2(rbtToGoal(1), rbtToGoal(0));

            float newThetaRbtToGoal = std::min(std::max(thetaRbtToGoal, thetaRight), thetaLeft); // this can break for behind gaps

            float rbtToLeftAngle = std::abs(thetaLeft - thetaRbt);
            float rbtToRightAngle = std::abs(thetaRbt - thetaRight);
            float sigma_ = 0.50; // must be > 0
            // 0.05: jittery

            Eigen::Vector2f leftTerm = Rnegpi2_ * unitNorm(rbtToLeft) * exp(- rbtToLeftAngle / sigma_); // on robot's left
            Eigen::Vector2f rightTerm = Rpi2_ * unitNorm(rbtToRight) * exp(- rbtToRightAngle / sigma_); // on robot's right

            // Since local goal will definitely be within the range of the gap, this limit poses no difference
            Eigen::Vector2f newRbtToGoal(rbtToGoalDistance * cos(newThetaRbtToGoal), rbtToGoalDistance * sin(newThetaRbtToGoal));

            Eigen::Vector2f initToLeft(xLeft_ - xRbtInit_, yLeft_ - yRbtInit_);
            Eigen::Vector2f initToRight(xRight_ - xRbtInit_, yRight_ - yRbtInit_);
            Eigen::Vector2f currToLeft(xLeft_ - rbtPosn[0], yLeft_ - rbtPosn[1]);  
            Eigen::Vector2f currToRight(xRight_ - rbtPosn[0], yRight_ - rbtPosn[1]);
            Eigen::Vector2f initToGoal(xGoal_ - xRbtInit_, yGoal_ - yRbtInit_);
            Eigen::Vector2f currToGoal(xGoal_ - rbtPosn[0], yGoal_ - rbtPosn[1]);

            bool pastGoal = (initToGoal.dot(currToGoal) < 0);
            bool pastLeftGapPt = initToLeft.dot(currToLeft) < 0;
            bool pastRightGapPt = initToRight.dot(currToRight) < 0;
            
            bool pastGapPoints;
            if (radial_) 
            {
                //pastGap = (rbt.norm() > std::min(p_right.norm(), p_left.norm()) + 0.18) && rbt.norm() > gapGoal.norm();
                pastGapPoints = pastLeftGapPt || pastRightGapPt;
            } else 
            {
                // pastGap = (rbt.norm() > std::max(p_right.norm(), p_left.norm()) + 0.18) && rbt.norm() > gapGoal.norm();
                pastGapPoints = pastLeftGapPt && pastRightGapPt;
            }
            
            bool pastGap = pastGapPoints || pastGoal;

            Eigen::Vector2f rbtVel(0, 0); 

            // float eps = 0.0000001;
            if (!pastGap)
            {
                float circulationCoeff = pastGapPoints ? 0.0 : 1.0;

                float currToLeftNorm = currToLeft.norm();
                float currToRightNorm = currToRight.norm();

                float w1 = epsilonDivide(currToLeftNorm, currToLeftNorm + currToRightNorm);
                float w2 = epsilonDivide(currToRightNorm, currToLeftNorm + currToRightNorm);
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rightTerm: (" << rightTerm[0] << ", " << rightTerm[1] << ")");
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   leftTerm: (" << leftTerm[0] << ", " << leftTerm[1] << ")");

                Eigen::Vector2f weightedCirculationSum = w1*rightTerm + w2*leftTerm;

                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   weightedCirculationSum: (" << weightedCirculationSum[0] << ", " 
                //                                                  << weightedCirculationSum[1] << ")");

                Eigen::Vector2f circulationTerm = circulationCoeff * unitNorm(weightedCirculationSum);
                Eigen::Vector2f attractionField = unitNorm(newRbtToGoal);
                // std::cout << "inte_t: " << t << std::endl;
                //std::cout << "robot to left: (" << rbtToLeft[0] << ", " << rbtToLeft[1] << "), robot to right: (" << rbtToRight[0] << ", " << rbtToRight[1] << ")" << std::endl;
                //std::cout << "angular difference to left: " << rbtToLeftAngle << ", angular difference to right: " << rbtToRightAngle << std::endl;
                //std::cout << "left weight: " << w2 << ", left circulation component: (" << leftTerm[0] << ", " << leftTerm[1] << "), right weight: " << w1 << ", right circulation component: (" << rightTerm[0] << ", " << rightTerm[1] << ")" << std::endl;  
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   circulation: (" << circulationTerm[0] << ", " << circulationTerm[1] << ")");
                //std::cout << "robot to goal: (" << goal_vec[0] << ", " << goal_vec[1] << ")" << std::endl;
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   attraction: (" << attractionField[0] << ", " << attractionField[1] << ")");
                rbtVel = circulationTerm + attractionField;
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rbtVel: (" << rbtVel[0] << ", " << rbtVel[1] << ")");
            }
            clipVelocities(rbtVel);
            // Eigen::Vector2d acc(K_acc*(rbtVel(0) - x[2]), K_acc*(rbtVel(1) - x[3]));
            // acc = clipVelocities(acc[0], acc[1], aRbtLinMax_);

            dxdt[0] = rbtVel[0];
            dxdt[1] = rbtVel[1];
            // dxdt[2] = acc[0];
            // dxdt[3] = acc[1];
            return;
        }
    };

    /**
    * \brief Structure for generating artifical harmonic potential field that provides
    * collision-free trajectories for navigable gaps in dynamic environments
    */
    struct AHPF 
    {
        float vRbtLinMax_; /**< maximum linear velocity of robot */
        float rbtToGoalDistance_; /**< distance between robot and gap goal */
        float eps; /**< infinitesimal value to avoid dividing by zero */
        float KAttraction_; /**< Gain for attractive potential term applied to AHPF */
        bool radial_; /**< boolean for if gap is radial*/
        bool pastLeftGapPt_; /** boolean for if robot has passed the left gap point */
        bool pastRightGapPt_; /** boolean for if robot has passed the right gap point */
        bool pastGapPoints_; /**< boolean for if robot has passed the left and right gap points */
        bool pastGoal_; /**< boolean for if robot has passed the goal */
        bool pastGap_; /**< boolean for if robot has passed the gap and trajectory generation should be stopped */

        Eigen::Vector2d rbtPosn_; /**< 2D position of robot */
        Eigen::Vector2d rbtToLeft_; /**< 2D robot to left gap point vector */
        Eigen::Vector2d rbtToRight_; /**< 2D robot to right gap point vector */
        Eigen::Vector2d pLeft_; /**< 2D left gap point */
        Eigen::Vector2d pRight_; /**< 2D right gap point */
        Eigen::Vector2d pGoal_; /**< 2D gap goal point */
        Eigen::Vector2d rbtToGoal_; /**< 2D robot to gap goal vector */
        Eigen::Vector2d rbtVelRaw_; /**< raw robot velocity that is output from AHPF */
        Eigen::Vector2d rbtVelCmd_; /**< robot command velocity that is scaled and clipped */
        Eigen::Vector2d vLeft_; /**< velocity of 2D left gap point */
        Eigen::Vector2d vRight_; /**< velocity of 2D right gap point */
        // Eigen::Vector2d vGoal_; /**< velocity of 2D gap goal */

        Eigen::MatrixXd harmonicWeights_; /**< weights of all of the harmonic terms that comprise the AHPF */
        Eigen::MatrixXd harmonicCenters_; /**< center positions of the harmonic terms that comprise the AHPF */
        Eigen::MatrixXd gapBoundaryInwardNorms_; /**< inward-pointing norms of the left and right gap boundaries */
        Eigen::MatrixXd harmonicCentersToRbt_; /**< harmonic centers to robot displacement vector */
        Eigen::MatrixXd harmonicGradients; /**< gradient vectors of AHPF */
        
        Eigen::VectorXd harmonicCenterstoRbtSqNorm; /**< squared norms of displacement vectors of harmonic centers to robot position */

        AHPF(const float & vRbtLinMax, 
             const Eigen::MatrixXd & harmonicCenters, 
             const Eigen::MatrixXd & gapBoundaryInwardNorms, 
             const Eigen::MatrixXd & harmonicWeights,
             const Eigen::Vector2d & vLeft, 
             const Eigen::Vector2d & vRight, 
             const Eigen::Vector2d & pGoal) 
            : vRbtLinMax_(vRbtLinMax), harmonicCenters_(harmonicCenters), 
              gapBoundaryInwardNorms_(gapBoundaryInwardNorms), harmonicWeights_(harmonicWeights),
                vLeft_(vLeft), vRight_(vRight), pGoal_(pGoal)
        { 
            eps = 0.0000001;              
        }

        /**
        * \brief Helper function for clipping velocities to maximum allowed velocities
        * \param rbtVel current robot velocity
        */
        void clipVelocities(Eigen::Vector2f & rbtVel) 
        {
            // std::cout << "in clipVelocities with " << vX << ", " << vY << std::endl;
            // Eigen::Vector2f origVel(vX, vY);
            float speedX = std::abs(rbtVel[0]);
            float speedY = std::abs(rbtVel[1]);
            if (speedX <= vRbtLinMax_ && speedY <= vRbtLinMax_) 
            {
                // std::cout << "not clipping" << std::endl;
                return;
            } else {
                // std::cout << "max: " << vx_absmax << ", norm: " << origVel.norm() << std::endl;
                // Eigen::Vector2f clipVel = vRbtLinMax_ * origVel / std::max(speedX, speedY);
                rbtVel = epsilonDivide(vRbtLinMax_ * rbtVel,  std::max(speedX, speedY));
                // return clipVel;
            }
        }

        /**
        * \brief () operator for AHPF that updates trajectory state
        * \param x trajectory state
        * \param dxdt trajectory state rate of change
        * \param t current timestep
        */
        void operator() (const robotAndGapState &x, robotAndGapState &dxdt, const float & t)
        {                
            // Just use the same state to be able to make these past checks
            rbtPosn_ << x[0], x[1];
            pLeft_ << x[2], x[3];
            pRight_ << x[4], x[5];
            // pGoal_ << x[6], x[7];

            rbtToLeft_ = pLeft_ - rbtPosn_;
            rbtToRight_ = pRight_ - rbtPosn_;
            rbtToGoal_ = pGoal_ - rbtPosn_;

            /// CHECK TO SEE IF ROBOT IS PAST GAP ///
            pastGoal_ = pGoal_.dot(rbtToGoal_) < 0; 
            pastLeftGapPt_ = pLeft_.dot(rbtToLeft_) < 0;
            pastRightGapPt_ = pRight_.dot(rbtToRight_) < 0;
            
            if (radial_) 
            {
                pastGapPoints_ = pastLeftGapPt_ || pastRightGapPt_;
            } else {
                pastGapPoints_ = pastLeftGapPt_ && pastRightGapPt_;
            }
            
            pastGap_ = pastGapPoints_ || pastGoal_;

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "               t: " << t);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "                   rbt state: " << x[0] << ", " << x[1]);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "                   left state: " << x[2] << ", " << x[3]);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "                   right state: " << x[4] << ", " << x[5]);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "                   goal state: " << x[6] << ", " << x[7]);

            if (pastGap_) 
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "past gap");
                dxdt[0] = 0; dxdt[1] = 0; dxdt[2] = 0; dxdt[3] = 0; 
                dxdt[4] = 0; dxdt[5] = 0; dxdt[6] = 0; dxdt[7] = 0;
                return;
            } 

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "left_pos: " << pLeft_[0] << ", " << pLeft_[1]);            
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "right_pos: " << pRight_[0] << ", " << pRight_[1]);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "goal_pos: " << pGoal_[0] << ", " << pGoal_[1]);            

            // APF

            rbtToGoalDistance_ = rbtToGoal_.norm();

            KAttraction_ = - pow(rbtToGoalDistance_, 2);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "attractive term: " << KAttraction_);

            // Eigen::MatrixXd gradient_of_pti_wrt_centers(Kplus1, 2); // (2, Kplus1); //other one used is Kplus1, 2
   
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "harmonicCenters_ size: " << harmonicCenters_.rows() << ", " << harmonicCenters_.cols());
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "gapBoundaryInwardNorms_ size: " << gapBoundaryInwardNorms_.rows() << ", " << gapBoundaryInwardNorms_.cols());

            harmonicCentersToRbt_ = (-harmonicCenters_).rowwise() + rbtPosn_.transpose(); // size (r: Kplus1, c: 2)
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "harmonicCentersToRbt_ size: " << harmonicCentersToRbt_.rows() << ", " << harmonicCentersToRbt_.cols());

            /// CHECK TO SEE IF ROBOT HAS LEFT GAP THROUGH SIDES ///
            Eigen::Index maxIndex;
            float maxNorm = harmonicCentersToRbt_.rowwise().norm().minCoeff(&maxIndex);
            
            if (maxIndex > 0) 
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "                   min norm index: " << maxIndex);
                Eigen::Vector2d closestCenterToRbt = harmonicCentersToRbt_.row(maxIndex);
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "                   min norm center to rbt dir: " << closestCenterToRbt.transpose());
                Eigen::Vector2d closestCenterInwardNorm = gapBoundaryInwardNorms_.row(maxIndex - 1);
                ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "                   min norm inward dir: " << closestCenterInwardNorm.transpose());
                if (closestCenterToRbt.dot(closestCenterInwardNorm) < 0) 
                {
                    ROS_WARN_STREAM_NAMED("GapTrajectoryGenerator", "During AHPF trajectory synthesis, robot has left gap");
                    dxdt[0] = 0; dxdt[1] = 0; dxdt[2] = 0; dxdt[3] = 0; 
                    dxdt[4] = 0; dxdt[5] = 0; dxdt[6] = 0; dxdt[7] = 0;
                    return;
                }
            }

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "               " << 1);

            harmonicCenterstoRbtSqNorm = harmonicCentersToRbt_.rowwise().squaredNorm();
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "harmonicCenterstoRbtSqNorm size: " << harmonicCenterstoRbtSqNorm.rows() << ", " << harmonicCenterstoRbtSqNorm.cols());

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "               " << 2);

            harmonicGradients = harmonicCentersToRbt_.array().colwise() / (harmonicCenterstoRbtSqNorm.array() + eps);          
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "harmonicGradients size: " << harmonicGradients.rows() << ", " << harmonicGradients.cols());

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "               " << 3);

            // output of AHPF
            rbtVelRaw_ = KAttraction_ * harmonicGradients.transpose() * harmonicWeights_;

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "               " << 4);

            // Normalizing raw output and scaling by velocity limit
            rbtVelCmd_ = vRbtLinMax_ * unitNorm(rbtVelRaw_);

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "               " << 5);

            // CLIPPING DESIRED VELOCITIES
            // rbtVelCmd_ = clipVelocities(rbtVelCmd_[0], rbtVelCmd_[1], vRbtLinMax_);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "                   rbtVelCmd_: " << rbtVelCmd_[0] << ", " << rbtVelCmd_[1]);

            dxdt[0] = rbtVelCmd_[0]; // rbt_x
            dxdt[1] = rbtVelCmd_[1]; // rbt_y

            dxdt[2] = vLeft_[0]; // left point r_x (absolute)
            dxdt[3] = vLeft_[1]; // left point r_y (absolute)

            dxdt[4] = vRight_[0]; // right point r_x (absolute)
            dxdt[5] = vRight_[1]; // right point r_y (absolute)

            // dxdt[6] = vGoal_[0]; // goal point r_x (absolute)
            // dxdt[7] = vGoal_[1]; // goal point r_y (absolute)
        }
    };
    
    /**
    * \brief Structure for generating trajectories with parallel navigation technique
    */    
    struct ParallelNavigation 
    {
        float speedRobot_; /**< speed of robot */
        float gammaIntercept_; /**< intercept angle of robot */
        float rInscr_; /**< inscribed radius of robot */
        Eigen::Vector2f leftGapPtVel_, rightGapPtVel_; /**< left and right gap point velocities */

        ParallelNavigation(const float & gamma_intercept,
                            const float & speed_robot,
                            const float & r_inscr,
                            const Eigen::Vector2f & leftGapPtVel,
                            const Eigen::Vector2f & rightGapPtVel) : gammaIntercept_(gamma_intercept), 
                                                                        speedRobot_(speed_robot), 
                                                                        rInscr_(r_inscr),
                                                                        leftGapPtVel_(leftGapPtVel),
                                                                        rightGapPtVel_(rightGapPtVel) {}

        /**
        * \brief () operator for AHPF that updates trajectory state
        * \param x trajectory state
        * \param dxdt trajectory state rate of change
        * \param t current timestep
        */
        void operator() (const robotAndGapState & x, robotAndGapState & dxdt, const float & t)
        {
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "t: " << t << ", x: " << x[0] << ", " << x[1] << ", goal: " << x[12] << ", " << x[13] << ", rbtToGoalDistance: " << rbtToGoalDistance);

            Eigen::Vector2f n_gamma_intercept(std::cos(gammaIntercept_), std::sin(gammaIntercept_));

            Eigen::Vector2f robotVelocity = speedRobot_ * n_gamma_intercept;

            Eigen::Vector2f rbtPos(x[0], x[1]);
            Eigen::Vector2f leftGapPos(x[2], x[3]);
            Eigen::Vector2f rightGapPos(x[4], x[5]);


            // include some check for when robot has passed gap
            //      v1: rbt vector is further than left pt vector and right pt vector [DONE]
            //      v2: rbt vector is beyond range of gap arc at rbt's particular bearing (tricky because robot will leave gap slice)

            if (rbtPos.norm() > leftGapPos.norm() && rbtPos.norm() > rightGapPos.norm())
            {
                // stop trajectory prematurely
                dxdt[0] = 0.0; dxdt[1] = 0.0; dxdt[2] = 0.0; dxdt[3] = 0.0;
                dxdt[4] = 0.0; dxdt[5] = 0.0; dxdt[6] = 0.0; dxdt[7] = 0.0;

                return;
            }

            // if (rbtToGoalDistance < 0.1) 
            // {
            //     // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "t: " << t << ", stopping at x: " << x[0] << ", " << x[1] << ", goal: " << x[12] << ", " << x[13]);

            //     dxdt[0] = 0.0;
            //     dxdt[1] = 0.0;
            //     return;
            // }

            dxdt[0] = robotVelocity[0];
            dxdt[1] = robotVelocity[1];
            dxdt[2] = leftGapPtVel_[0];
            dxdt[3] = leftGapPtVel_[1];
            dxdt[4] = rightGapPtVel_[0];
            dxdt[5] = rightGapPtVel_[1];

            return;
        }
    };

    /**
    * \brief Structure for generating trajectories that head straight to goal
    */    
    struct GoToGoal 
    {
        float vRbtLinMax_; /**< maximum linear velocity of robot */

        GoToGoal(const float & vRbtLinMax) : vRbtLinMax_(vRbtLinMax) {}

        /**
        * \brief Helper function for clipping velocities to maximum allowed velocities
        * \param rbtVel current robot velocity
        */
        void clipVelocities(Eigen::Vector2f & rbtVel) 
        {
            // std::cout << "in clipVelocities with " << vX << ", " << vY << std::endl;
            // Eigen::Vector2f origVel(vX, vY);
            float speedX = std::abs(rbtVel[0]);
            float speedY = std::abs(rbtVel[1]);
            if (speedX <= vRbtLinMax_ && speedY <= vRbtLinMax_) 
            {
                // std::cout << "not clipping" << std::endl;
                return;
            } else {
                // std::cout << "max: " << vx_absmax << ", norm: " << origVel.norm() << std::endl;
                // Eigen::Vector2f clipVel = vRbtLinMax_ * origVel / std::max(speedX, speedY);
                rbtVel = epsilonDivide(vRbtLinMax_ * rbtVel,  std::max(speedX, speedY));
                // return clipVel;
            }
        }

        /**
        * \brief () operator for AHPF that updates trajectory state
        * \param x trajectory state
        * \param dxdt trajectory state rate of change
        * \param t current timestep
        */
        void operator() (const robotAndGapState & x, robotAndGapState & dxdt, const float & t)
        {
            // std::cout << "x: " << std::endl;
            float rbtToGoalDistance = sqrt(pow(x[6] - x[0], 2) + pow(x[7] - x[1], 2));
            Eigen::Vector2f rbtVelDes_(0.0, 0.0);

            // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "t: " << t << ", x: " << x[0] << ", " << x[1] << ", goal: " << x[12] << ", " << x[13] << ", rbtToGoalDistance: " << rbtToGoalDistance);

            // we want to make it so once robot reaches gap, trajectory ends, even if goal keeps moving
            if (rbtToGoalDistance < 0.1) 
            {
                // ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "t: " << t << ", stopping at x: " << x[0] << ", " << x[1] << ", goal: " << x[12] << ", " << x[13]);
                rbtVelDes_(0) = 0;
                rbtVelDes_(1) = 0;

                dxdt[0] = 0.0;
                dxdt[1] = 0.0;
                dxdt[6] = 0.0;
                dxdt[7] = 0.0;
                return;
            }


            rbtVelDes_[0] = (x[6] - x[0]);
            rbtVelDes_[1] = (x[7] - x[1]);
            
            clipVelocities(rbtVelDes_);

            dxdt[0] = rbtVelDes_[0];
            dxdt[1] = rbtVelDes_[1];
            dxdt[6] = 0.0;
            dxdt[7] = 0.0;
            return;
        }
    };

    /**
    * \brief Structure for logging trajectories and recording their contents
    */
    struct TrajectoryLogger
    {
        geometry_msgs::PoseArray & path_; /**< resulting path of trajectory */
        std::string frame_; /**< frame ID of trajectory */
        // float _scale;
        std::vector<float>& pathTiming_; /**< resulting path timing of trajectory */

        TrajectoryLogger(geometry_msgs::PoseArray & path, const std::string & frame, 
                         std::vector<float> & pathTiming): 
                         path_(path), frame_(frame), pathTiming_(pathTiming) {}

        /**
        * \brief () operator for logger that records trajectory state
        * \param x trajectory state
        * \param t current timestep
        */
        void operator() (const robotAndGapState &x , const float & t)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame_;

            pose.pose.position.x = x[0];
            pose.pose.position.y = x[1];
            pose.pose.position.z = 0;

            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            path_.poses.push_back(pose.pose);

            pathTiming_.push_back(t);
        }
    };

}