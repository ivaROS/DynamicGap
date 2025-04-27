
#include <dynamic_gap/gap_feasibility/GapFeasibilityChecker.h>

namespace dynamic_gap 
{
    void GapFeasibilityChecker::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        scan_ = scan;
    }

    // bool GapFeasibilityChecker::pursuitGuidanceAnalysis(Gap * gap)
    // {
    //     // check what method we are using
    //     if (cfg_->planning.pursuit_guidance_method == 0)
    //     {
    //         // pure pursuit
    //         // return purePursuitFeasibilityCheck(gap);
    //         throw std::runtime_error("Should not be using pure pursuit");

    //     } else
    //     {
    //         // parallel navigation
    //         return parallelNavigationFeasibilityCheck(gap);
    //     }
    // }
    
    bool GapFeasibilityChecker::pursuitGuidanceAnalysisV2(Gap * gap, const Eigen::Vector2f & startPt)
    {
        // check what method we are using
        if (cfg_->planning.pursuit_guidance_method == 0)
        {
            // pure pursuit
            // return purePursuitFeasibilityCheck(gap);
            throw std::runtime_error("Should not be using pure pursuit");

        } else
        {
            // parallel navigation
            return parallelNavigationFeasibilityCheckV2(gap, startPt);
        }
    }    

    // bool GapFeasibilityChecker::purePursuitFeasibilityCheck(Gap * gap)
    // {
    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                [purePursuitFeasibilityCheck()]"); 
    //     // calculate intercept angle and intercept time using center point (gap goal)

    //     throw std::runtime_error("Should not be using pure pursuit");

    //     return false;
    // }

    // void GapFeasibilityChecker::purePursuitHelper(const Eigen::Vector2f & p_target, 
    //                                                 const Eigen::Vector2f & v_target, 
    //                                                 const float speed_robot,
    //                                                 float & t_intercept)
    // {
    //     float lambda = std::atan2(p_target[1], p_target[0]);
    //     float gamma = std::atan2(v_target[1], v_target[0]);

    //     if (v_target.norm() < eps) // static gap point
    //     {
    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                           static gap point"); 

    //         t_intercept = p_target.norm() / speed_robot;
    //     } else
    //     {
    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                           dynamic gap point"); 

    //         float K = epsilonDivide(speed_robot, v_target.norm()); // just set to one dimensional norm

    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                               K: " << K); 

    //         // assert(K > 1);

    //         Eigen::Vector2f n_lambda(std::cos(lambda), std::sin(lambda));
    //         Eigen::Vector2f n_gamma(std::cos(gamma), std::sin(gamma));

    //         float theta = getSignedLeftToRightAngle(n_gamma, n_lambda);

    //         t_intercept = (p_target.norm() / v_target.norm()) * (epsilonDivide(K + std::cos(theta), (K * K - 1)));
    //     }
    // }

    bool GapFeasibilityChecker::parallelNavigationFeasibilityCheckV2(Gap * gap, const Eigen::Vector2f & startPt)
    {
        ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                [parallelNavigationFeasibilityCheckV2()]"); 

        // gap->getLeftGapPt()->getModel()->isolateManipGapDynamics();
        // gap->getRightGapPt()->getModel()->isolateManipGapDynamics();

        // Eigen::Vector4f leftGapState = gap->getLeftGapPt()->getModel()->getManipGapState();
        // Eigen::Vector4f rightGapState = gap->getRightGapPt()->getModel()->getManipGapState();

        Eigen::Vector2f leftGapPtVel = gap->getManipulatedLVelocity();
        Eigen::Vector2f rightGapPtVel = gap->getManipulatedRVelocity();

        // Eigen::Vector2f leftGapPtPos = leftGapState.head(2);
        // Eigen::Vector2f leftGapPtVel = leftGapState.tail(2);

        // Eigen::Vector2f rightGapPtPos = rightGapState.head(2);
        // Eigen::Vector2f rightGapPtVel = rightGapState.tail(2);

        // ROS_INFO_STREAM("                       leftGapState: " << leftGapState.transpose()); 

        // ROS_INFO_STREAM("                       rightGapState: " << rightGapState.transpose()); 

        ///////////////////////////////
        // Evaluating left gap point //
        ///////////////////////////////
        
        // float t_intercept_left = 0.0;
        // float gamma_intercept_left = 0.0;
        // parallelNavigationHelper(gap->getManipulatedLPosition(), 
        //                             leftGapPtVel, 
        //                             cfg_->rbt.vx_absmax,
        //                             t_intercept_left, 
        //                             gamma_intercept_left);

        // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_left: " << t_intercept_left); 
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gamma_intercept_left: " << gamma_intercept_left); 

        // gap->t_intercept_left = t_intercept_left;
        // gap->gamma_intercept_left = gamma_intercept_left;

        ///////////////////////////////
        // Evaluating right gap point //
        ///////////////////////////////

        // float t_intercept_right = 0.0;
        // float gamma_intercept_right = 0.0;
        // parallelNavigationHelper(gap->getManipulatedRPosition(), 
        //                             rightGapPtVel, 
        //                             cfg_->rbt.vx_absmax,
        //                             t_intercept_right, 
        //                             gamma_intercept_right);

        // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_right: " << t_intercept_right); 
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gamma_intercept_right: " << gamma_intercept_right); 

        // gap->t_intercept_right = t_intercept_right;
        // gap->gamma_intercept_right = gamma_intercept_right;

        // set target position to gap goal
        Eigen::Vector2f goalPt = gap->getGoal()->getOrigGoalPos();
        Eigen::Vector2f p_target = goalPt - startPt;  // (gap->goal.x_, gap->goal.y_);

        // set target velocity to mean of left and right gap points
        Eigen::Vector2f v_target = gap->getGoal()->getOrigGoalVel(); // (gap->goal.vx_, gap->goal.vy_);

        ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       startPt: " << startPt.transpose()); 
        ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       goalPt: " << goalPt.transpose()); 
        ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       p_target: " << p_target.transpose()); 
        ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       v_target: " << v_target.transpose()); 

        float t_intercept_goal = 0.0;
        float gamma_intercept_goal = 0.0;
        parallelNavigationHelper(p_target, 
                                    v_target, 
                                    cfg_->rbt.vx_absmax,
                                    t_intercept_goal, 
                                    gamma_intercept_goal);

        ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       t_intercept_goal: " << t_intercept_goal); 
        ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       gamma_intercept_goal: " << gamma_intercept_goal); 

        if (isnan(t_intercept_goal) || isnan(gamma_intercept_goal)) // can happen if K < 1
        {
            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                    gap is not feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->getGapLifespan()); 
            return false;
        // } else if (!gap->isRGC() && 
        //             gap->getEndCondition() == SHUT && 
        //             t_intercept_goal > gap->getGapLifespan())
        // {
        //     ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                    gap is not feasible!");
        //     ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       t_intercept_goal: " << t_intercept_goal);
        //     ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       gap lifespan: " << gap->getGapLifespan()); 
        //     ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       gap end condition: " << gap->getEndCondition()); 

        //     return false;
        } else
        {
            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                    gap is feasible! " 
                                                                        //    << t_intercept_left << 
                                                                            // ", t_intercept_right: " << 
                                                                            // t_intercept_right << 
                                                                            << ", t_intercept_goal: " << 
                                                                            t_intercept_goal << 
                                                                            ", gap lifespan: " << gap->getGapLifespan()); 
            
            // How to handle situation where t_intercept is negative?

            Eigen::Vector2f terminalGoal;
            if (t_intercept_goal < 0) //      Can happen when K < 0
            {
                terminalGoal = p_target;
            } else if (t_intercept_goal > gap->getGapLifespan()) //      Can happen when ego-robot not fast enough
            {
                terminalGoal = p_target + v_target * gap->getGapLifespan();
            } else
            {
                terminalGoal = p_target + v_target * t_intercept_goal;
            }

            // clip at scan
            float terminalGoalTheta = std::atan2(terminalGoal[1], terminalGoal[0]);
            int terminalGoalScanIdx = theta2idx(terminalGoalTheta);

            // if terminal goal lives beyond scan
            if (scan_->ranges.at(terminalGoalScanIdx) < (terminalGoal.norm() + cfg_->traj.max_pose_to_scan_dist))
            {
                float newTerminalGoalRange = scan_->ranges.at(terminalGoalScanIdx) - cfg_->traj.max_pose_to_scan_dist;
                terminalGoal << newTerminalGoalRange * std::cos(terminalGoalTheta),
                                newTerminalGoalRange * std::sin(terminalGoalTheta);
            }

            // set gamma_rbt
            gap->setTInterceptGoal(t_intercept_goal);
            gap->setGammaInterceptGoal(terminalGoalTheta);

            // gap->setTerminalGoal(terminalGoal);
            gap->getGoal()->setTermGoalPos(terminalGoal);

            return true;            
        }

    }

    // bool GapFeasibilityChecker::parallelNavigationFeasibilityCheck(Gap * gap)
    // {
    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                [parallelNavigationFeasibilityCheck()]"); 

    //     gap->getLeftGapPt()->getModel()->isolateGapDynamics();
    //     gap->getRightGapPt()->getModel()->isolateGapDynamics();

    //     Eigen::Vector4f leftGapState = gap->getLeftGapPt()->getModel()->getGapState();
    //     Eigen::Vector4f rightGapState = gap->getRightGapPt()->getModel()->getGapState();

    //     // Eigen::Vector2f leftGapPtPos = leftGapState.head(2);
    //     Eigen::Vector2f leftGapPtVel = leftGapState.tail(2);

    //     // Eigen::Vector2f rightGapPtPos = rightGapState.head(2);
    //     Eigen::Vector2f rightGapPtVel = rightGapState.tail(2);

    //     // ROS_INFO_STREAM("                       leftGapState: " << leftGapState.transpose()); 

    //     // ROS_INFO_STREAM("                       rightGapState: " << rightGapState.transpose()); 

    //     ///////////////////////////////
    //     // Evaluating left gap point //
    //     ///////////////////////////////
        
    //     float t_intercept_left = 0.0;
    //     float gamma_intercept_left = 0.0;
    //     parallelNavigationHelper(gap->getManipulatedLPosition(), 
    //                                 leftGapPtVel, 
    //                                 cfg_->rbt.vx_absmax,
    //                                 t_intercept_left, 
    //                                 gamma_intercept_left);

    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_left: " << t_intercept_left); 
    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gamma_intercept_left: " << gamma_intercept_left); 

    //     // gap->t_intercept_left = t_intercept_left;
    //     // gap->gamma_intercept_left = gamma_intercept_left;

    //     ///////////////////////////////
    //     // Evaluating right gap point //
    //     ///////////////////////////////

    //     float t_intercept_right = 0.0;
    //     float gamma_intercept_right = 0.0;
    //     parallelNavigationHelper(gap->getManipulatedRPosition(), 
    //                                 rightGapPtVel, 
    //                                 cfg_->rbt.vx_absmax,
    //                                 t_intercept_right, 
    //                                 gamma_intercept_right);

    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_right: " << t_intercept_right); 
    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gamma_intercept_right: " << gamma_intercept_right); 

    //     // gap->t_intercept_right = t_intercept_right;
    //     // gap->gamma_intercept_right = gamma_intercept_right;

    //     // set target position to gap goal
    //     Eigen::Vector2f p_target = gap->getGoal()->getOrigGoalPos();  // (gap->goal.x_, gap->goal.y_);

    //     // set target velocity to mean of left and right gap points
    //     Eigen::Vector2f v_target = gap->getGoal()->getOrigGoalVel(); // (gap->goal.vx_, gap->goal.vy_);

    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                       p_target: " << p_target.transpose()); 
    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                       v_target: " << v_target.transpose()); 

    //     float t_intercept_goal = 0.0;
    //     float gamma_intercept_goal = 0.0;
    //     parallelNavigationHelper(p_target, 
    //                                 v_target, 
    //                                 cfg_->rbt.vx_absmax,
    //                                 t_intercept_goal, 
    //                                 gamma_intercept_goal);

    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_goal: " << t_intercept_goal); 
    //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gamma_intercept_goal: " << gamma_intercept_goal); 

    //     if (isnan(t_intercept_goal) || isnan(gamma_intercept_goal)) // can happen if K < 1
    //     {
    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is not feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->getGapLifespan()); 
    //         return false;
    //     } else if (!gap->isRGC() && 
    //                 gap->getEndCondition() == SHUT && 
    //                 t_intercept_goal > gap->getGapLifespan())
    //     {
    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is not feasible!");
    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_goal: " << t_intercept_goal);
    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gap lifespan: " << gap->getGapLifespan()); 
    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gap end condition: " << gap->getEndCondition()); 

    //         return false;
    //     } else
    //     {
    //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is feasible! t_intercept_left: " 
    //                                                                        << t_intercept_left << 
    //                                                                         ", t_intercept_right: " << 
    //                                                                         t_intercept_right << 
    //                                                                         ", t_intercept_goal: " << 
    //                                                                         t_intercept_goal << 
    //                                                                         ", gap lifespan: " << gap->getGapLifespan()); 
            
    //         // How to handle situation where t_intercept is negative?

    //         Eigen::Vector2f terminalGoal;
    //         if (t_intercept_goal < 0) //      Can happen when K < 0
    //         {
    //             terminalGoal = p_target;
    //         } else if (t_intercept_goal > gap->getGapLifespan()) //      Can happen when ego-robot not fast enough
    //         {
    //             terminalGoal = p_target + v_target * gap->getGapLifespan();
    //         } else
    //         {
    //             terminalGoal = p_target + v_target * t_intercept_goal;
    //         }

    //         // clip at scan
    //         float terminalGoalTheta = std::atan2(terminalGoal[1], terminalGoal[0]);
    //         int terminalGoalScanIdx = theta2idx(terminalGoalTheta);

    //         // if terminal goal lives beyond scan
    //         if (scan_->ranges.at(terminalGoalScanIdx) < (terminalGoal.norm() + cfg_->traj.max_pose_to_scan_dist))
    //         {
    //             float newTerminalGoalRange = scan_->ranges.at(terminalGoalScanIdx) - cfg_->traj.max_pose_to_scan_dist;
    //             terminalGoal << newTerminalGoalRange * std::cos(terminalGoalTheta),
    //                             newTerminalGoalRange * std::sin(terminalGoalTheta);
    //         }

    //         // set gamma_rbt
    //         gap->setTInterceptGoal(t_intercept_goal);
    //         gap->setGammaInterceptGoal(terminalGoalTheta);

    //         // gap->setTerminalGoal(terminalGoal);
    //         gap->getGoal()->setTermGoalPos(terminalGoal);

    //         return true;            
    //     }

    // }

    void GapFeasibilityChecker::parallelNavigationHelper(const Eigen::Vector2f & p_target_init, 
                                                            const Eigen::Vector2f & v_target_init, 
                                                            const float speed_robot,
                                                            float & t_intercept, 
                                                            float & gamma_intercept)
    {
        ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       [parallelNavigationHelper()]"); 

        // float eps = 0.00001;

        // float scaling_factor = 0.99;

        Eigen::Vector2f p_target = p_target_init;
        Eigen::Vector2f v_target = v_target_init;

        // t_intercept = std::numeric_limits<float>::quiet_NaN();
        // gamma_intercept = std::numeric_limits<float>::quiet_NaN();

        // while ( isnan(t_intercept) )
        // {
        float lambda = std::atan2(p_target[1], p_target[0]);
        float gamma = std::atan2(v_target[1], v_target[0]);

        if (v_target.norm() < eps) // static gap point
        {
            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                           static gap point"); 

            t_intercept = p_target.norm() / speed_robot;
            gamma_intercept = lambda;
        } else // moving gap point
        {
            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                           dynamic gap point");

            float K = epsilonDivide(speed_robot, v_target.norm()); // just set to one dimensional norm

            if (K < 0)
            {
                ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                       something has gone very wrong");
            }

            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                           K: " << K);

            Eigen::Vector2f n_lambda(std::cos(lambda), std::sin(lambda));
            Eigen::Vector2f n_gamma(std::cos(gamma), std::sin(gamma));

            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                           n_lambda: " << n_lambda.transpose());
            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                           n_gamma: " << n_gamma.transpose());

            float theta = getSignedLeftToRightAngle(n_gamma, n_lambda);

            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                           theta: " << theta);

            float delta = std::asin( epsilonDivide(std::sin(theta), K));

            ROS_INFO_STREAM_NAMED("GapFeasibilityV2", "                           delta: " << delta);

            t_intercept = (p_target.norm() / v_target.norm()) * (epsilonDivide(1.0, (K * std::cos(delta) - std::cos(theta))));

            gamma_intercept = lambda + delta;

            // if ( isnan(t_intercept) )
            // {
            //     ROS_INFO_STREAM("                               t_intercept was nan! scaling down: ");
            //     p_target *= scaling_factor; // iteratively lessen and lessen until we can intercept
            //     v_target *= scaling_factor; // iteratively lessen and lessen until we can intercept
            // }
        }
        // }

        // if (isnan(t_intercept))
        // {
        //     ROS_WARN_STREAM_NAMED("GapFeasibility", "                           t_interecept is nan!"); 
        // }

        // if (isnan(gamma_intercept))
        // {
        //     ROS_WARN_STREAM_NAMED("GapFeasibility", "                           gamma_intercept is nan!"); 
        // }

        return;
    }

}
