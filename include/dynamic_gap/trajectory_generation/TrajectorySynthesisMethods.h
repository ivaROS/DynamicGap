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
    * \brief Structure for generating trajectories with pure pursuit technique
    */    
    struct PurePursuit 
    {
        float speedRobot_; /**< speed of robot */
        float rInscr_; /**< inscribed radius of robot */
        Eigen::Vector2f leftGapPtVel_, rightGapPtVel_; /**< left and right gap point velocities */
        Eigen::Vector2f goalPtVel_; /**< gap goal point velocity */

        PurePursuit(const float & speed_robot,
                    const float & r_inscr,
                    const Eigen::Vector2f & leftGapPtVel,
                    const Eigen::Vector2f & rightGapPtVel,
                    const Eigen::Vector2f & goalPtVel) : speedRobot_(speed_robot), 
                                                            rInscr_(r_inscr),
                                                            leftGapPtVel_(leftGapPtVel),
                                                            rightGapPtVel_(rightGapPtVel),
                                                            goalPtVel_(goalPtVel) {}

        /**
        * \brief () operator for parallel navigation scheme that updates trajectory state
        * \param x trajectory state
        * \param dxdt trajectory state rate of change
        * \param t current timestep
        */
        void operator() (const robotAndGapState & x, robotAndGapState & dxdt, const float & t)
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "t: " << t);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   rbt: (" << x[0] << ", " << x[1] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   left: (" << x[2] << ", " << x[3] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   right: (" << x[4] << ", " << x[5] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   goal: (" << x[6] << ", " << x[7] << ")");

            Eigen::Vector2f rbtPos(x[0], x[1]);
            Eigen::Vector2f leftGapPos(x[2], x[3]);
            Eigen::Vector2f rightGapPos(x[4], x[5]);
            Eigen::Vector2f goalPos(x[6], x[7]);

            Eigen::Vector2f rbtToGoal = goalPos - rbtPos;
            Eigen::Vector2f n_rbtToGoal = rbtToGoal / rbtToGoal.norm();

            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "   n_rbtToGoal: " << n_rbtToGoal.transpose());

            Eigen::Vector2f robotVelocity = speedRobot_ * n_rbtToGoal;


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

            dxdt[0] = robotVelocity[0];
            dxdt[1] = robotVelocity[1];
            dxdt[2] = leftGapPtVel_[0];
            dxdt[3] = leftGapPtVel_[1];
            dxdt[4] = rightGapPtVel_[0];
            dxdt[5] = rightGapPtVel_[1];
            dxdt[6] = goalPtVel_[0];
            dxdt[7] = goalPtVel_[1];

            return;
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
        Eigen::Vector2f goalPtVel_; /**< gap goal point velocity */
        Eigen::Vector2f terminalGoal_; /**< terminal gap goal point */

        ParallelNavigation(const float & gamma_intercept,
                            const float & speed_robot,
                            const float & r_inscr,
                            const Eigen::Vector2f & leftGapPtVel,
                            const Eigen::Vector2f & rightGapPtVel,
                            const Eigen::Vector2f & goalPtVel,
                            const Eigen::Vector2f & terminalGoal) : gammaIntercept_(gamma_intercept), 
                                                                        speedRobot_(speed_robot), 
                                                                        rInscr_(r_inscr),
                                                                        leftGapPtVel_(leftGapPtVel),
                                                                        rightGapPtVel_(rightGapPtVel),
                                                                        goalPtVel_(goalPtVel),
                                                                        terminalGoal_(terminalGoal) {}

        /**
        * \brief () operator for parallel navigation scheme that updates trajectory state
        * \param x trajectory state
        * \param dxdt trajectory state rate of change
        * \param t current timestep
        */
        void operator() (const robotAndGapState & x, robotAndGapState & dxdt, const float & t)
        {
            ROS_INFO_STREAM("t: " << t);
            ROS_INFO_STREAM("   x: " << x[0] << ", " << x[1]);

            Eigen::Vector2f n_gamma_intercept(std::cos(gammaIntercept_), std::sin(gammaIntercept_));

            Eigen::Vector2f robotVelocity = speedRobot_ * n_gamma_intercept;

            Eigen::Vector2f rbtPos(x[0], x[1]);
            Eigen::Vector2f leftGapPos(x[2], x[3]);
            Eigen::Vector2f rightGapPos(x[4], x[5]);
            Eigen::Vector2f goalPos(x[6], x[7]);

            Eigen::Vector2f rbtToTerminalGoal = terminalGoal_ - rbtPos;

            // include some check for when robot has passed gap
            //      v1: rbt vector is further than left pt vector and right pt vector [DONE]
            //      v2: rbt vector is beyond range of gap arc at rbt's particular bearing (tricky because robot will leave gap slice)

            if (rbtToTerminalGoal.norm() < 0.25 ||
                (rbtPos.norm() > leftGapPos.norm() && rbtPos.norm() > rightGapPos.norm()))
            {
                ROS_INFO_STREAM("   prematured stop");
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
            dxdt[6] = goalPtVel_[0];
            dxdt[7] = goalPtVel_[1];

            return;
        }
    };

    /**
    * \brief Structure for generating trajectories that head straight to goal
    */    
    struct GoToGoal 
    {
        float speedRobot_; /**< maximum linear velocity of robot */

        GoToGoal(const float & vRbtLinMax) : speedRobot_(vRbtLinMax) {}

        /**
        * \brief () operator for AHPF that updates trajectory state
        * \param x trajectory state
        * \param dxdt trajectory state rate of change
        * \param t current timestep
        */
        void operator() (const robotAndGapState & x, robotAndGapState & dxdt, const float & t)
        {
            // std::cout << "x: " << std::endl;
            Eigen::Vector2f rbtToGoal(x[6] - x[0], x[7] - x[1]);
            float rbtToGoalDistance = rbtToGoal.norm();
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

            rbtVelDes_ = rbtToGoal;

            // rbtVelDes_[0] = (x[6] - x[0]);
            // rbtVelDes_[1] = (x[7] - x[1]);
            
            clipVelocities(rbtVelDes_);

            dxdt[0] = rbtVelDes_[0];
            dxdt[1] = rbtVelDes_[1];
            dxdt[6] = 0.0;
            dxdt[7] = 0.0;
            return;
        }

        void clipVelocities(Eigen::Vector2f & linVel) 
        {
            float speedLinXFeedback = std::abs(linVel[0]);
            float speedLinYFeedback = std::abs(linVel[0]);
            
            if (speedLinXFeedback <= speedRobot_ && speedLinYFeedback <= speedRobot_) 
            {
                // std::cout << "not clipping" << std::endl;
            } else 
            {
                linVel[0] *= epsilonDivide(speedRobot_, std::max(speedLinXFeedback, speedLinYFeedback));
                linVel[1] *= epsilonDivide(speedRobot_, std::max(speedLinXFeedback, speedLinYFeedback));
            }

            // std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, velAngFeedback));
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