#include <dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h>


namespace dynamic_gap 
{
    TrajectoryEvaluator::TrajectoryEvaluator(const DynamicGapConfig& cfg)
    {
        cfg_ = & cfg;
    }

    void TrajectoryEvaluator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    void TrajectoryEvaluator::transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                                            const geometry_msgs::TransformStamped & odom2rbt) 
    {
        boost::mutex::scoped_lock lock(globalPlanMutex_);
        tf2::doTransform(globalPathLocalWaypointOdomFrame, globalPathLocalWaypointRobotFrame_, odom2rbt);
    }

    float TrajectoryEvaluator::compute_h(
        Eigen::Vector2f humanVel,
        Eigen::Vector2f relativeGapPos,
        // Eigen::Vector2f trajPos,
        Eigen::Vector2f robotVel)
    {
    //btw this is just a copy of the DPCBF function i just renamed it so it doesn't get confusing
    // Eigen::Vector2f relVel = -relativeVel; // so velocity and position vector point in the same direction for the dot product
    //  Eigen::Vector2f v_rel = robotVel - humanVel; // human vel is the vel of the dynamic endpoint which represents the human  
     //todo might need to fix relvel^ , DPCBF code uses: v_rel = v_obs - v_robot
     Eigen::Vector2f v_rel = -robotVel + humanVel; // human vel is the vel of the dynamic endpoint which represents the human  

    // ROS_ERROR_STREAM("negative v_rel: " << v_rel.transpose());
    // ROS_ERROR_STREAM("relativeGapPos: " << relativeGapPos.transpose());
    // ROS_ERROR_STREAM("trajPos: " << trajPos.transpose());
    // ROS_ERROR_STREAM("robotVel: " << robotVel.transpose());
    // ROS_ERROR_STREAM_NAMED("relvel: ", relVel << ", relativeGapPos: " << relativeGapPos << ", robotVel: " << robotVel);

    // ROS_ERROR_STREAM_NAMED("relvel cost", "relativeGapPos: ");
    // ROS_ERROR_STREAM_NAMED("relvel cost", relativeGapPos);

    Eigen::Vector2f p_rel = relativeGapPos;  //- trajPos;// distance between the current pos we're looking at and the gap point (which represents the dynamic obstacle)
    // ROS_ERROR_STREAM_NAMED("relvel cost", "posToGapPtDist: ");
    // ROS_ERROR_STREAM_NAMED("relvel cost", posToGapPtDist);


    float rot_angle = std::atan2(p_rel.y(), p_rel.x());
    // Rotation matrix R = [[cos a, sin a], [-sin a, cos a]]
    float c = std::cos(rot_angle);
    float s = std::sin(rot_angle);
    Eigen::Matrix2f R;
    R <<  c,  s,
        -s,  c;

    // v_rel_new = R * v_rel
    Eigen::Vector2f v_rel_new = R * v_rel;
    float v_rel_new_x = v_rel_new.x();
    float v_rel_new_y = v_rel_new.y();

    // Magnitudes
    float p_rel_mag = p_rel.norm();
    float v_rel_mag = v_rel.norm();

    // float r_obs = 0.2; 
    // float s = 1; 

    // float ego_dim = (r_obs +  cfg_->rbt.r_inscr) * s; 

    const float eps = 1e-6f;

   
    const float r_obs = 0.2f;                
    const float s_margin = 1.05f; //1.05f;            // must be > 1 for sqrt(s^2-1)

    const float r_robot = cfg_->rbt.r_inscr;

    // inflated combined safety radius
    float ego_dim = (r_obs + r_robot) * s_margin;

    // d_safe = max(||p||^2 - ego_dim^2, eps)
    float d_safe = std::max(p_rel_mag * p_rel_mag - ego_dim * ego_dim, eps);

    // Base gains (copy author DT code constants)
    float k_lambda = 0.1f * std::sqrt(s_margin * s_margin - 1.0f) / ego_dim;
    float k_mu     = 0.5f * std::sqrt(s_margin * s_margin - 1.0f) / ego_dim;

    // Guard v_rel_mag to avoid blow-up
    float v_rel_mag_safe = std::max(v_rel_mag, eps);

    // lambda = k_lambda * sqrt(d_safe) / ||v_rel||
    float lambda = k_lambda * std::sqrt(d_safe) / v_rel_mag_safe;

    // mu = k_mu * sqrt(d_safe)
    float mu = k_mu * std::sqrt(d_safe);

    // Barrier: h = v_rel_new_x + lambda * v_rel_new_y^2 + mu
    float h = v_rel_new_x + lambda * (v_rel_new_y * v_rel_new_y) + mu;
    // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "h: " << h); 

    return h;
}


    void TrajectoryEvaluator::evaluateTrajectory(const Trajectory & traj,
                                                std::vector<float> & posewiseCosts,
                                                float & terminalPoseCost,
                                                const std::vector<sensor_msgs::LaserScan> & futureScans,
                                                const int & scanIdx) 
    {    
        try
        {
            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
            // Requires LOCAL FRAME

            geometry_msgs::PoseArray path = traj.getPathRbtFrame();
            std::vector<float> pathTiming = traj.getPathTiming();

            //   ROS_ERROR_STREAM("in evaluator path.size()=" << path.poses.size()
            //      << " addr=" << &path);
                 

            
            posewiseCosts = std::vector<float>(path.poses.size());

            if (path.poses.size() > futureScans.size()) 
            {
                ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            posewiseCosts-futureScans size mismatch: " << posewiseCosts.size() << " vs " << futureScans.size());
                return;
            }

            if (posewiseCosts.size() != path.poses.size()) 
            {
                ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            posewiseCosts-pathPoses size mismatch: " << posewiseCosts.size() << " vs " << path.poses.size());
                return;
            }


            for (int i = 0; i < posewiseCosts.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " (total scan idx: " << (scanIdx + i) << "): ");
                // std::cout << "regular range at " << i << ": ";
                posewiseCosts.at(i) = evaluatePose(path.poses.at(i), futureScans.at(scanIdx + i)); //  / posewiseCosts.size()
            }
            float totalTrajCost = std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0)) / posewiseCosts.size();
            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "             avg pose-wise cost: " << totalTrajCost);

            // obtain terminalGoalCost, scale by Q
            terminalPoseCost = cfg_->traj.Q_f * terminalGoalCost(path.poses.back());

            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            terminal cost: " << terminalPoseCost);
            
            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "evaluateTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            evaluateTrajectory out of range exception: " << e.what());
        }
        
        return;
    }

    float TrajectoryEvaluator::calcSpeedCost(const float v_cmd, const float v_max)
        {
            // Prefer higher speeds but stay within limits.
            // Cost = 0 when v_cmd == v_max, increases as speed drops.
            float speed_cost = v_max - std::clamp(v_cmd, 0.0f, v_max);

            // Normalize by v_max so cost is in [0,1].
            speed_cost /= (v_max + 1e-6f);
            // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "v_cmd: " << v_cmd << " speed_cost: " << speed_cost);

            return speed_cost;
        }


    // void TrajectoryEvaluator::dwa_evaluateTrajectory(geometry_msgs::PoseArray & pose_array,
    //                                             std::vector<float> & posewiseCosts,
    //                                             float & terminalPoseCost,
    //                                             const std::vector<sensor_msgs::LaserScan> & futureScans,
    //                                             const int & scanIdx) 
        void TrajectoryEvaluator::dwa_evaluateTrajectory(float & totalTrajCost, dwa_Trajectory & dwa_traj,
                                std::vector<float> & posewiseCosts,
                                std::vector<float> &dwa_PathCosts, 
                                float & terminalPoseCost,
                                const std::vector<sensor_msgs::LaserScan> & futureScans,
                                const int & scanIdx,
                                const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet, 
                                dynamic_gap::Gap* gap,
                                std::vector<float> &dwa_RelVelPoseCosts)
    {    
        try
        {
            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
            // Requires LOCAL FRAME

            // geometry_msgs::PoseArray path = traj.getPathRbtFrame();
            // std::vector<float> pathTiming = traj.getPathTiming();

            //   ROS_ERROR_STREAM("in evaluator path.size()=" << path.poses.size()
            //      << " addr=" << &path);
                 

            geometry_msgs::PoseArray pose_array = dwa_traj.pose_array; 
            posewiseCosts = std::vector<float>(pose_array.poses.size());
            dwa_PathCosts = std::vector<float>(pose_array.poses.size());
            dwa_RelVelPoseCosts = std::vector<float>(pose_array.poses.size());

            if (pose_array.poses.size() > futureScans.size()) 
            {
                ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            posewiseCosts-futureScans size mismatch: " << posewiseCosts.size() << " vs " << futureScans.size());
                
                return;
            }

            if (posewiseCosts.size() != pose_array.poses.size()) 
            {
                ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            posewiseCosts-pathPoses size mismatch: " << posewiseCosts.size() << " vs " << pose_array.poses.size());
                return;
            }


            for (int i = 0; i < posewiseCosts.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " (total scan idx: " << (scanIdx + i) << "): ");
                // std::cout << "regular range at " << i << ": ";
                // IMPORTANT NOTE: this is only the distance-related cost not other costs like path cost
                posewiseCosts.at(i) = dwa_evaluatePose(pose_array.poses.at(i), futureScans.at(scanIdx + i)); //  / posewiseCosts.size()
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " (cost: " << posewiseCosts.at(i) << "): ");

            }
            // float totalTrajCost = std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0)) / posewiseCosts.size();
            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "             avg pose-wise cost: " << totalTrajCost);

            // obtain terminalGoalCost, scale by Q
            terminalPoseCost = cfg_->traj.Q_f * terminalGoalCost(pose_array.poses.back());
            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            terminal cost: " << terminalPoseCost);

            //////////////////// path costs //////////////////
            // --- Path-distance cost (distance to visible global plan snippet) ---
            // std::vector<geometry_msgs::PoseStamped> visiblePlan =
                // globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
            int idx = 0; 
            float path_cost_sum = 0.0f;
            for (const auto& pose : pose_array.poses) //todo: combine with loop above
            {
                dwa_PathCosts.at(idx) = calcDistToGlobalPath(pose, globalPlanSnippet);
                path_cost_sum += dwa_PathCosts.at(idx);
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "pose" << idx << " path distance cost: " << path_cost_sum);
                idx++; 
            }

            float avg_path_cost = path_cost_sum / std::max(1.0f, float(pose_array.poses.size()));
            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "avg path distance cost: " << avg_path_cost);
            
///////////////////////////////////////////////////// social code //////////////////////////////////////////////////////////////////////////////////////////////////////
             ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
        // Requires LOCAL FRAME
        double dynamic_thres = 0.15; // threshold for what's considered a dynamic gap endpoint
        Eigen::Vector2f leftGapRelVel(0, 0);
        geometry_msgs::TwistStamped RbtVelMsg;
        Eigen::Vector2f RbtVel(0, 0);
        Eigen::Vector2f leftGapRelPos(0, 0);
        Eigen::Vector2f rightGapRelVel(0, 0);
        Eigen::Vector2f rightGapRelPos(0, 0); 
        bool leftGapPtIsDynamic = false; 
        bool rightGapPtIsDynamic = false; 

        Eigen::Vector2f leftVel(0,0);
        Eigen::Vector2f rightVel(0,0);

        
        if(cfg_->planning.social_cost_function == 1)
        {
        if(gap)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());

        // leftGapPtIsDynamic = gap->getLeftGapPt()->getUngapID()>=0; 

        int gapID = gap->getLeftGapPt()->getModel()->getID();

        // ROS_ERROR_STREAM("[TE] requested modelID=" << gapID);

        if (leftVelDictPtr_)
        {
            auto it = leftVelDictPtr_->find(gapID);
            // ROS_ERROR_STREAM("leftVelDictPtr_->end(): " << leftVelDictPtr_->end());
            // ROS_ERROR_STREAM("[TE] leftVelDictPtr_ size = "
            //      << std::distance(leftVelDictPtr_->begin(), leftVelDictPtr_->end()));

            if (it != leftVelDictPtr_->end())
            {
                leftVel = it->second;
            }
            else
            {
                ROS_WARN_STREAM("Missing leftVel for modelID=" << gapID);
            }
        }
        else
        {
            ROS_WARN_STREAM("leftVelDictPtr_ is null");
        }
        // ROS_ERROR_STREAM_NAMED("evalTraj", "leftGapRelVel: " << leftVel);
        
        // leftGapPtIsDynamic = true; //JUST FOR DEBUGGING!

        if (leftVel.norm() > dynamic_thres) {leftGapPtIsDynamic = true;}
        if(leftGapPtIsDynamic)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());
         gap->getLeftGapPt()->getModel()->isolateGapDynamics();
        //  gap->leftGapPt__model_->isolateGapDynamics();
         leftGapRelVel = gap->getLeftGapPt()->getModel()->getGapVelocity(); // todo: delete this, its actually not used
         RbtVelMsg = gap->getLeftGapPt()->getModel()->getRobotVel();
         RbtVel << RbtVelMsg.twist.linear.x, RbtVelMsg.twist.linear.y;


         Eigen::Vector4f leftState  = gap->getLeftGapPt()->getModel()->getGapState();

         geometry_msgs::Point posUncovertedL = pose_array.poses.at(0).position; //todo delete this. Its used for trajpos but I want to delete that too because it's unused in the actual cbf function
                Eigen::Vector2f leftPos;
                leftPos.x() = posUncovertedL.x; 
                leftPos.y() = posUncovertedL.y;
            
        // dwa_traj.H_left = DPCBF(leftVel, leftGapRelPos, PosL, RbtVel);
        //   float v  = dwa_traj.v_cmd; 
        //   float w = dwa_traj.w_cmd; 
           

        //   Eigen::Vector2f u_nom = Eigen::Vector2f::Zero();
        //   Eigen::Vector2f left_cbf_vel = DPCBFProjectVelocity(leftVel, leftGapRelPos, PosL, u_nom, v, w); // todo rename this, it's not h, this is the new u i computed
        //   ROS_ERROR_STREAM("left_cbf_vel: " << left_cbf_vel); 

          // //// just testing ////////////////
        // Eigen::Vector2f leftVel  = leftState.tail<2>();
        // // Debug print
        // ROS_ERROR_STREAM_NAMED("DWA",
        //     "[GapVel] left=("  << leftVel.transpose()  << ")");
        // //// just testing ////////////////

        


        // Eigen::Vector2f leftVel  = latestGapLeftVelPtr_->at(gapID);

        // ROS_ERROR_STREAM("DWA gapID=" << gapID
        //     << " leftVel=" << leftVel.transpose());


         // ROS_ERROR_STREAM_NAMED("evalTraj", "leftGapRelVel: ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftGapRelVel);  
         leftGapRelPos = gap->getLeftGapPt()->getModel()->getState().head<2>(); //distance from robot to gap.
        // ROS_ERROR_STREAM_NAMED("evalTraj", "gap->leftGapPtModel_->getState(): ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftGapRelPos);  

        dwa_traj.humanVelLeft = leftVel;
        dwa_traj.gapPosLeft = leftGapRelPos;
        dwa_traj.trajPosLeft = leftPos; //20260127 I plan on removing trajPos bc I'm not using it
        dwa_traj.robotVel = RbtVel;

        
        // ROS_ERROR_STREAM_NAMED("eval",
        // "inside traj evaluator right after populating the object:"
        // << " humanVel=(" << dwa_traj.humanVelLeft.x() << "," << dwa_traj.humanVelLeft.y() << ")"
        // << " gapPos=(" << dwa_traj.gapPosLeft.x() << "," << dwa_traj.gapPosLeft.y() << ")"
        // << " robotVel=(" << dwa_traj.robotVel.x() << "," << dwa_traj.robotVel.y() << ")");


        }

        // rightGapPtIsDynamic = gap->getRightGapPt()->getUngapID()>=0; 

        int right_gapID = gap->getRightGapPt()->getModel()->getID();

            // ROS_ERROR_STREAM("[TE] requested RIGHT modelID=" << right_gapID);

            if (rightVelDictPtr_)
            {
                auto it = rightVelDictPtr_->find(right_gapID);
                // ROS_ERROR_STREAM("[TE] rightVelDictPtr_ size = "
                //     << std::distance(rightVelDictPtr_->begin(), rightVelDictPtr_->end()));

                if (it != rightVelDictPtr_->end())
                {
                    rightVel = it->second;
                }
                else
                {
                    ROS_WARN_STREAM("Missing rightVel for modelID=" << right_gapID);
                }
            }
            else
            {
                ROS_WARN_STREAM("rightVelDictPtr_ is null");
            }

            // ROS_ERROR_STREAM_NAMED("evalTraj", "rightGapRelVel: " << rightVel);

            // rightGapRelVel = rightVel;

        // rightGapPtIsDynamic = true; // debugging
        if (rightVel.norm() > dynamic_thres) {rightGapPtIsDynamic = true;
        // ROS_ERROR_STREAM_NAMED("evalTraj", "right gap point is dynamic greater than 0.15 threshold");
        }
        if(rightGapPtIsDynamic)
        {
            gap->getRightGapPt()->getModel()->isolateGapDynamics();

            rightGapRelPos =
                gap->getRightGapPt()->getModel()->getState().head<2>();

            dwa_traj.humanVelRight = rightVel;
            dwa_traj.gapPosRight = rightGapRelPos;

        //     ROS_ERROR_STREAM_NAMED("eval",
        // "right endpoint ------- inside traj evaluator right after populating the object:"
        // << " humanVelRight=(" << dwa_traj.humanVelRight.x() << "," << dwa_traj.humanVelRight.y() << ")"
        // << " gapPos=(" << dwa_traj.gapPosRight.x() << "," << dwa_traj.gapPosRight.y() << ")");
        
        }



        
    //     int i = 0; //ATODO DELETE
    //     for(i = 0; i < 1; i++)
    // {
        
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
    //     INFO("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());
        
    // }
        }
    }

        // geometry_msgs::PoseArray path = traj.getPathRbtFrame();
        // std::vector<float> pathTiming = traj.getPathTiming();
        
        // posewiseCosts = std::vector<float>(pose_array.poses.size());

        float leftGapPtCost = 0; 
        float rightGapPtCost = 0; 
        float weight = cfg_->traj.w_relvel; 


        for (int i = 0; i < posewiseCosts.size(); i++) //todo combine with the path and pose cost loops above
        {
            if (leftGapPtIsDynamic){ // if(leftGapPtIsDynamic){
                geometry_msgs::Point posUncoverted = pose_array.poses.at(i).position;
                Eigen::Vector2f Pos;
                Pos.x() = posUncoverted.x; 
                Pos.y() = posUncoverted.y;
                
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "leftGapRelPos (which is distance of robot to gap): " << leftGapRelPos); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftGapRelPos);

                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "Pos: "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", Pos);

                leftGapPtCost = relativeVelocityCost(leftVel, leftGapRelPos, Pos, RbtVel);

                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(leftGapRelVel, leftGapRelPos, RbtVel)"); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftGapPtCost); 
            }
            
            
            if(rightGapPtIsDynamic){ //(rightGapPtIsDynamic){
                geometry_msgs::Point posUncoverted = pose_array.poses.at(i).position;
                Eigen::Vector2f Pos;
                Pos.x() = posUncoverted.x; 
                Pos.y() = posUncoverted.y;
                
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "rightGapRelPos: "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightGapRelPos);

                rightGapPtCost = relativeVelocityCost(rightVel, rightGapRelPos, Pos, RbtVel);
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(rightGapRelVel, rightGapRelPos, RbtVel)"); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightGapPtCost); 
            }
            dwa_RelVelPoseCosts.at(i) = leftGapPtCost + rightGapPtCost;
            // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "dwa_RelVelPoseCosts.at(i): " << dwa_RelVelPoseCosts.at(i)); 
        }

        ///////////////////////////////////////////////////// social code //////////////////////////////////////////////////////////////////////////////////////////////////////

        ///////////////////// h related cost 

        float left_h = compute_h(dwa_traj.humanVelLeft, dwa_traj.gapPosLeft, dwa_traj.robotVel); 
        float right_h = compute_h(dwa_traj.humanVelRight, dwa_traj.gapPosRight, dwa_traj.robotVel); 
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "            in traj eval left_h: " << left_h << " right_h: " << right_h);

        // Violation magnitudes (only when H < 0)
        float vL = std::max(0.0f, -left_h);
        float vR = std::max(0.0f, -right_h);

        // Combine endpoints (two bad sides worse than one)
        float V = vL + vR;

        // Deadband to ignore tiny negatives (tune this)
        const float epsilon = 0.05f;
        float V_eff = std::max(0.0f, V - epsilon);

        // Amplify (helps because H kinda saturates around 0.4–0.5)
        // float h_cost = V_eff * V_eff;
        float h_cost = V_eff; 

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "            h_cost: " <<  h_cost);
        terminalPoseCost += h_cost; // just taching h_cost onto this

            // Combine into a final cost
            totalTrajCost =
                (std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0)) / posewiseCosts.size()) +
                cfg_->traj.w_path * avg_path_cost + terminalPoseCost + 
                cfg_->traj.w_relvel * (std::accumulate(dwa_RelVelPoseCosts.begin(), dwa_RelVelPoseCosts.end(), float(0)) / dwa_RelVelPoseCosts.size());
        
////////////////////////// just debugingg!!!!! ADD THE REST OF THE COSTS BACK!! 
            // totalTrajCost = cfg_->traj.w_relvel * (std::accumulate(dwa_RelVelPoseCosts.begin(), dwa_RelVelPoseCosts.end(), float(0)) / dwa_RelVelPoseCosts.size());
        


            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "evaluateTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            evaluateTrajectory out of range exception: " << e.what());
        }
        
        return;
    }


      void TrajectoryEvaluator::dwa_evaluateTrajectory_outside(
        // float & totalTrajCost, 
                                Trajectory & traj,
                                std::vector<float> & posewiseCosts,
                                // std::vector<float> &dwa_PathCosts, 
                                float & terminalPoseCost,
                                const std::vector<sensor_msgs::LaserScan> & futureScans,
                                const int & scanIdx,
                                // const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet, 
                                dynamic_gap::Gap* gap)
                                // std::vector<float> &dwa_RelVelPoseCosts)
    {    
        try
        {
             ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
            // Requires LOCAL FRAME

            geometry_msgs::PoseArray path = traj.getPathRbtFrame();
            std::vector<float> pathTiming = traj.getPathTiming();

            //   ROS_ERROR_STREAM("in evaluator path.size()=" << path.poses.size()
            //      << " addr=" << &path);
                 

            
            posewiseCosts = std::vector<float>(path.poses.size());
            std::vector<float> dwa_RelVelPoseCosts = std::vector<float>(path.poses.size()); // only using this locally

            if (path.poses.size() > futureScans.size()) 
            {
                ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            posewiseCosts-futureScans size mismatch: " << posewiseCosts.size() << " vs " << futureScans.size());
                return;
            }

            if (posewiseCosts.size() != path.poses.size()) 
            {
                ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            posewiseCosts-pathPoses size mismatch: " << posewiseCosts.size() << " vs " << path.poses.size());
                return;
            }


            for (int i = 0; i < posewiseCosts.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " (total scan idx: " << (scanIdx + i) << "): ");
                // std::cout << "regular range at " << i << ": ";
                posewiseCosts.at(i) = evaluatePose(path.poses.at(i), futureScans.at(scanIdx + i)); //  / posewiseCosts.size() // this is ame as dwa_evaluatePose() btw
            }
            float totalTrajCost = std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0)) / posewiseCosts.size();
            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "             avg pose-wise cost: " << totalTrajCost);

            // obtain terminalGoalCost, scale by Q
            terminalPoseCost = cfg_->traj.Q_f * terminalGoalCost(path.poses.back());

            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            terminal cost: " << terminalPoseCost);
            
            //////////////////// path costs //////////////////
            // --- Path-distance cost (distance to visible global plan snippet) ---
            // std::vector<geometry_msgs::PoseStamped> visiblePlan =
                // globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
            // int idx = 0; 
            // float path_cost_sum = 0.0f;
            // for (const auto& pose : pose_array.poses) //todo: combine with loop above
            // {
            //     dwa_PathCosts.at(idx) = calcDistToGlobalPath(pose, globalPlanSnippet);
            //     path_cost_sum += dwa_PathCosts.at(idx);
            //     // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "pose" << idx << " path distance cost: " << path_cost_sum);
            //     idx++; 
            // }

            // float avg_path_cost = path_cost_sum / std::max(1.0f, float(pose_array.poses.size()));
            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "avg path distance cost: " << avg_path_cost);
            
///////////////////////////////////////////////////// social code //////////////////////////////////////////////////////////////////////////////////////////////////////
             ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
        // Requires LOCAL FRAME
        double dynamic_thres = 0.15; // threshold for what's considered a dynamic gap endpoint
        Eigen::Vector2f leftGapRelVel(0, 0);
        geometry_msgs::TwistStamped RbtVelMsg;
        Eigen::Vector2f RbtVel(0, 0);
        Eigen::Vector2f leftGapRelPos(0, 0);
        Eigen::Vector2f rightGapRelVel(0, 0);
        Eigen::Vector2f rightGapRelPos(0, 0); 
        bool leftGapPtIsDynamic = false; 
        bool rightGapPtIsDynamic = false; 

        Eigen::Vector2f leftVel(0,0);
        Eigen::Vector2f rightVel(0,0);

        
        if(cfg_->planning.social_cost_function == 1)
        {
        if(gap)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());

        // leftGapPtIsDynamic = gap->getLeftGapPt()->getUngapID()>=0; 

        int gapID = gap->getLeftGapPt()->getModel()->getID();

        // ROS_ERROR_STREAM("[TE] requested modelID=" << gapID);

        if (leftVelDictPtr_)
        {
            auto it = leftVelDictPtr_->find(gapID);
            // ROS_ERROR_STREAM("leftVelDictPtr_->end(): " << leftVelDictPtr_->end());
            // ROS_ERROR_STREAM("[TE] leftVelDictPtr_ size = "
            //      << std::distance(leftVelDictPtr_->begin(), leftVelDictPtr_->end()));

            if (it != leftVelDictPtr_->end())
            {
                leftVel = it->second;
            }
            else
            {
                ROS_WARN_STREAM("Missing leftVel for modelID=" << gapID);
            }
        }
        else
        {
            ROS_WARN_STREAM("leftVelDictPtr_ is null");
        }
        // ROS_ERROR_STREAM_NAMED("evalTraj", "leftGapRelVel: " << leftVel);
        
        // leftGapPtIsDynamic = true; //JUST FOR DEBUGGING!

        if (leftVel.norm() > dynamic_thres) {leftGapPtIsDynamic = true;}
        if(leftGapPtIsDynamic)
        {
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());
         gap->getLeftGapPt()->getModel()->isolateGapDynamics();
        //  gap->leftGapPt__model_->isolateGapDynamics();
         leftGapRelVel = gap->getLeftGapPt()->getModel()->getGapVelocity(); // todo: delete this, its actually not used
         RbtVelMsg = gap->getLeftGapPt()->getModel()->getRobotVel();
         RbtVel << RbtVelMsg.twist.linear.x, RbtVelMsg.twist.linear.y;


         Eigen::Vector4f leftState  = gap->getLeftGapPt()->getModel()->getGapState();

         geometry_msgs::Point posUncovertedL = path.poses.at(0).position; //todo delete this. Its used for trajpos but I want to delete that too because it's unused in the actual cbf function
                Eigen::Vector2f leftPos;
                leftPos.x() = posUncovertedL.x; 
                leftPos.y() = posUncovertedL.y;
            
        // dwa_traj.H_left = DPCBF(leftVel, leftGapRelPos, PosL, RbtVel);
        //   float v  = dwa_traj.v_cmd; 
        //   float w = dwa_traj.w_cmd; 
           

        //   Eigen::Vector2f u_nom = Eigen::Vector2f::Zero();
        //   Eigen::Vector2f left_cbf_vel = DPCBFProjectVelocity(leftVel, leftGapRelPos, PosL, u_nom, v, w); // todo rename this, it's not h, this is the new u i computed
        //   ROS_ERROR_STREAM("left_cbf_vel: " << left_cbf_vel); 

          // //// just testing ////////////////
        // Eigen::Vector2f leftVel  = leftState.tail<2>();
        // // Debug print
        // ROS_ERROR_STREAM_NAMED("DWA",
        //     "[GapVel] left=("  << leftVel.transpose()  << ")");
        // //// just testing ////////////////

        


        // Eigen::Vector2f leftVel  = latestGapLeftVelPtr_->at(gapID);

        // ROS_ERROR_STREAM("DWA gapID=" << gapID
        //     << " leftVel=" << leftVel.transpose());


         // ROS_ERROR_STREAM_NAMED("evalTraj", "leftGapRelVel: ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftGapRelVel);  
         leftGapRelPos = gap->getLeftGapPt()->getModel()->getState().head<2>(); //distance from robot to gap.
        // ROS_ERROR_STREAM_NAMED("evalTraj", "gap->leftGapPtModel_->getState(): ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftGapRelPos);  
        // replaced dwa_traj with traj. I only used dwa_traj in generateTrajV2 because there were mulitple trajs per gap. but after that I repackage 
        // all the info like human vel into max's Trajectory struct anways
        traj.humanVelLeft = leftVel;
        traj.gapPosLeft = leftGapRelPos;
        traj.trajPosLeft = leftPos; //20260127 I plan on removing trajPos bc I'm not using it
        traj.robotVel = RbtVel;

        
        // ROS_ERROR_STREAM_NAMED("eval",
        // "inside traj evaluator right after populating the object:"
        // << " humanVel=(" << traj.humanVelLeft.x() << "," << traj.humanVelLeft.y() << ")"
        // << " gapPos=(" << traj.gapPosLeft.x() << "," << traj.gapPosLeft.y() << ")"
        // << " robotVel=(" << traj.robotVel.x() << "," << traj.robotVel.y() << ")");


        }

        // rightGapPtIsDynamic = gap->getRightGapPt()->getUngapID()>=0; 

        int right_gapID = gap->getRightGapPt()->getModel()->getID();

            // ROS_ERROR_STREAM("[TE] requested RIGHT modelID=" << right_gapID);

            if (rightVelDictPtr_)
            {
                auto it = rightVelDictPtr_->find(right_gapID);
                // ROS_ERROR_STREAM("[TE] rightVelDictPtr_ size = "
                //     << std::distance(rightVelDictPtr_->begin(), rightVelDictPtr_->end()));

                if (it != rightVelDictPtr_->end())
                {
                    rightVel = it->second;
                }
                else
                {
                    ROS_WARN_STREAM("Missing rightVel for modelID=" << right_gapID);
                }
            }
            else
            {
                ROS_WARN_STREAM("rightVelDictPtr_ is null");
            }

            // ROS_ERROR_STREAM_NAMED("evalTraj", "rightGapRelVel: " << rightVel);

            // rightGapRelVel = rightVel;

        // rightGapPtIsDynamic = true; // debugging
        if (rightVel.norm() > dynamic_thres) {rightGapPtIsDynamic = true;
        // ROS_ERROR_STREAM_NAMED("evalTraj", "right gap point is dynamic greater than 0.15 threshold");
        }
        if(rightGapPtIsDynamic)
        {
            gap->getRightGapPt()->getModel()->isolateGapDynamics();

            rightGapRelPos =
                gap->getRightGapPt()->getModel()->getState().head<2>();

            traj.humanVelRight = rightVel;
            traj.gapPosRight = rightGapRelPos;

        //     ROS_ERROR_STREAM_NAMED("eval",
        // "right endpoint ------- inside traj evaluator right after populating the object:"
        // << " humanVelRight=(" << traj.humanVelRight.x() << "," << traj.humanVelRight.y() << ")"
        // << " gapPos=(" << traj.gapPosRight.x() << "," << traj.gapPosRight.y() << ")");
        
        }



        
    //     int i = 0; //ATODO DELETE
    //     for(i = 0; i < 1; i++)
    // {
        
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
    //     INFO("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());
        
    // }
        }
    }

        // geometry_msgs::PoseArray path = traj.getPathRbtFrame();
        // std::vector<float> pathTiming = traj.getPathTiming();
        
        // posewiseCosts = std::vector<float>(pose_array.poses.size());

        float leftGapPtCost = 0; 
        float rightGapPtCost = 0; 
        float weight = cfg_->traj.w_relvel; 


        for (int i = 0; i < posewiseCosts.size(); i++) //todo combine with the path and pose cost loops above
        {
            if (leftGapPtIsDynamic){ // if(leftGapPtIsDynamic){
                geometry_msgs::Point posUncoverted = path.poses.at(i).position;
                Eigen::Vector2f Pos;
                Pos.x() = posUncoverted.x; 
                Pos.y() = posUncoverted.y;
                
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "leftGapRelPos (which is distance of robot to gap): " << leftGapRelPos); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftGapRelPos);

                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "Pos: "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", Pos);

                leftGapPtCost = relativeVelocityCost(leftVel, leftGapRelPos, Pos, RbtVel);

                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(leftGapRelVel, leftGapRelPos, RbtVel)"); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftGapPtCost); 
            }
            
            
            if(rightGapPtIsDynamic){ //(rightGapPtIsDynamic){
                geometry_msgs::Point posUncoverted = path.poses.at(i).position;
                Eigen::Vector2f Pos;
                Pos.x() = posUncoverted.x; 
                Pos.y() = posUncoverted.y;
                
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "rightGapRelPos: "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightGapRelPos);

                rightGapPtCost = relativeVelocityCost(rightVel, rightGapRelPos, Pos, RbtVel);
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(rightGapRelVel, rightGapRelPos, RbtVel)"); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightGapPtCost); 
            }
            dwa_RelVelPoseCosts.at(i) = leftGapPtCost + rightGapPtCost;
            // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "dwa_RelVelPoseCosts.at(i): " << dwa_RelVelPoseCosts.at(i)); 
        }

        ///////////////////////////////////////////////////// social code //////////////////////////////////////////////////////////////////////////////////////////////////////

        ///////////////////// h related cost 

        float left_h = compute_h(traj.humanVelLeft, traj.gapPosLeft, traj.robotVel); 
        float right_h = compute_h(traj.humanVelRight, traj.gapPosRight, traj.robotVel); 
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "            in traj eval left_h: " << left_h << " right_h: " << right_h);

        // Violation magnitudes (only when H < 0)
        float vL = std::max(0.0f, -left_h);
        float vR = std::max(0.0f, -right_h);

        // Combine endpoints (two bad sides worse than one)
        float V = vL + vR;

        // Deadband to ignore tiny negatives (tune this)
        const float epsilon = 0.05f;
        float V_eff = std::max(0.0f, V - epsilon);

        // Amplify (helps because H kinda saturates around 0.4–0.5)
        // float h_cost = V_eff * V_eff;
        float h_cost = V_eff; 
        
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "            h_cost: " <<  h_cost);
        terminalPoseCost += h_cost; // just taching h_cost onto this
        // because the h_cost is already added to terminalPoseCost, I don't need to do anyrepackaging related to that (i'm talking about this function btw dwa_evaluateTrajectory_outside)


            // Combine into a final cost
            // totalTrajCost =
            //     (std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0)) / posewiseCosts.size()) +
            //     cfg_->traj.w_path * avg_path_cost + terminalPoseCost + 
            //     cfg_->traj.w_relvel * (std::accumulate(dwa_RelVelPoseCosts.begin(), dwa_RelVelPoseCosts.end(), float(0)) / dwa_RelVelPoseCosts.size());
        
        
////////////////////////// just debugingg!!!!! ADD THE REST OF THE COSTS BACK!! 
            // totalTrajCost = cfg_->traj.w_relvel * (std::accumulate(dwa_RelVelPoseCosts.begin(), dwa_RelVelPoseCosts.end(), float(0)) / dwa_RelVelPoseCosts.size());
        
        
        // this is very ugly but I need to repackage posewiseCosts so it contains all the costs that occur at every pose. this is what compareToCurrentTraj() expects
            for (int i = 0; i < posewiseCosts.size(); i++) 
            {
                posewiseCosts.at(i) += dwa_RelVelPoseCosts.at(i); // tach on the relvel costs to it 
                // in future if you want to add path costs, you have to add it here as well 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " (dwa_evaluateTrajectory_outside cost: " << posewiseCosts.at(i) << "): ");
            }
            // because the h_cost is already added to terminalPoseCost, I don't need to do anyrepackaging related to that 

            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "evaluateTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            evaluateTrajectory out of range exception: " << e.what());
        }
        
        return;
    }

     void TrajectoryEvaluator::dwa_evaluateTrajectory_reducedCurrentTraj(
        // float & totalTrajCost, 
                                Trajectory & traj,
                                std::vector<float> & posewiseCosts,
                                // std::vector<float> &dwa_PathCosts, 
                                float & terminalPoseCost,
                                const std::vector<sensor_msgs::LaserScan> & futureScans,
                                const int & scanIdx,
                                // const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet, 
                                // dynamic_gap::Gap* gap,
                                int rightGapPtID, 
                                int leftGapPtID,
                                Eigen::Vector2f RbtVel
                                )
                                // std::vector<float> &dwa_RelVelPoseCosts)
    {    
        try
        {
             ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
            // Requires LOCAL FRAME

            geometry_msgs::PoseArray path = traj.getPathRbtFrame();
            std::vector<float> pathTiming = traj.getPathTiming();

            //   ROS_ERROR_STREAM("in evaluator path.size()=" << path.poses.size()
            //      << " addr=" << &path);
                 

            
            posewiseCosts = std::vector<float>(path.poses.size());
            std::vector<float> dwa_RelVelPoseCosts = std::vector<float>(path.poses.size()); // only using this locally

            if (path.poses.size() > futureScans.size()) 
            {
                ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            posewiseCosts-futureScans size mismatch: " << posewiseCosts.size() << " vs " << futureScans.size());
                return;
            }

            if (posewiseCosts.size() != path.poses.size()) 
            {
                ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            posewiseCosts-pathPoses size mismatch: " << posewiseCosts.size() << " vs " << path.poses.size());
                return;
            }


            for (int i = 0; i < posewiseCosts.size(); i++) 
            {
                ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " (total scan idx: " << (scanIdx + i) << "): ");
                // std::cout << "regular range at " << i << ": ";
                posewiseCosts.at(i) = evaluatePose(path.poses.at(i), futureScans.at(scanIdx + i)); //  / posewiseCosts.size() // this is ame as dwa_evaluatePose() btw
            }
            float totalTrajCost = std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0)) / posewiseCosts.size();
            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "             avg pose-wise cost: " << totalTrajCost);

            // obtain terminalGoalCost, scale by Q
            terminalPoseCost = cfg_->traj.Q_f * terminalGoalCost(path.poses.back());

            ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            terminal cost: " << terminalPoseCost);
            
            //////////////////// path costs //////////////////
            // --- Path-distance cost (distance to visible global plan snippet) ---
            // std::vector<geometry_msgs::PoseStamped> visiblePlan =
                // globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);
            // int idx = 0; 
            // float path_cost_sum = 0.0f;
            // for (const auto& pose : pose_array.poses) //todo: combine with loop above
            // {
            //     dwa_PathCosts.at(idx) = calcDistToGlobalPath(pose, globalPlanSnippet);
            //     path_cost_sum += dwa_PathCosts.at(idx);
            //     // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "pose" << idx << " path distance cost: " << path_cost_sum);
            //     idx++; 
            // }

            // float avg_path_cost = path_cost_sum / std::max(1.0f, float(pose_array.poses.size()));
            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "avg path distance cost: " << avg_path_cost);
            
///////////////////////////////////////////////////// social code //////////////////////////////////////////////////////////////////////////////////////////////////////
             ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "         [evaluateTrajectory()]");
        // Requires LOCAL FRAME
        double dynamic_thres = 0.15; // threshold for what's considered a dynamic gap endpoint
        Eigen::Vector2f leftGapRelVel(0, 0);

        Eigen::Vector2f leftGapRelPos(0, 0);
        Eigen::Vector2f rightGapRelVel(0, 0);
        Eigen::Vector2f rightGapRelPos(0, 0); 
        bool leftGapPtIsDynamic = false; 
        bool rightGapPtIsDynamic = false; 

        Eigen::Vector2f leftVel(0,0);
        Eigen::Vector2f rightVel(0,0);

        
        if(cfg_->planning.social_cost_function == 1)
        {
        // if(gap)
        if(rightGapPtID != -1 && leftGapPtID != -1) // they're initialized to -1
        {
        ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "rightGapPtID: " << rightGapPtID << " leftGapPtID: " << leftGapPtID);
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());

        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());

        // leftGapPtIsDynamic = gap->getLeftGapPt()->getUngapID()>=0; 

        // int leftGapPtID = gap->getLeftGapPt()->getModel()->getID();

        // ROS_ERROR_STREAM("[TE] requested modelID=" << leftGapPtID);

          if (leftPosDictPtr_) {
                auto it = leftPosDictPtr_->find(leftGapPtID);
                if (it != leftPosDictPtr_->end()) {
                    leftGapRelPos = it->second;
                    ROS_ERROR_STREAM("leftgapID: " << leftGapPtID << " leftGapRelPos: " << leftGapRelPos);
                } else {
                    ROS_WARN_STREAM("Missing leftPos for modelID=" << leftGapPtID);
                }
            } else {
                ROS_WARN_STREAM("leftPosDictPtr_ is null");
            }

          if (rightPosDictPtr_) {
                auto it = rightPosDictPtr_->find(rightGapPtID);
                if (it != rightPosDictPtr_->end()) {
                    rightGapRelPos = it->second;
                    ROS_ERROR_STREAM("rightgapID: " << rightGapPtID << " rightGapRelPos: " << rightGapRelPos);

                } else {
                    ROS_WARN_STREAM("Missing rightPos for modelID=" << rightGapPtID);
                }
            } else {
                ROS_WARN_STREAM("rightPosDictPtr_ is null");
            }


        if (leftVelDictPtr_)
        {
            auto it = leftVelDictPtr_->find(leftGapPtID);
            // ROS_ERROR_STREAM("leftVelDictPtr_->end(): " << leftVelDictPtr_->end());
            // ROS_ERROR_STREAM("[TE] leftVelDictPtr_ size = "
            //      << std::distance(leftVelDictPtr_->begin(), leftVelDictPtr_->end()));

            if (it != leftVelDictPtr_->end())
            {
                leftVel = it->second;
            }
            else
            {
                ROS_WARN_STREAM("Missing leftVel for modelID=" << leftGapPtID);
            }
        }
        else
        {
            ROS_WARN_STREAM("leftVelDictPtr_ is null");
        }
        // ROS_ERROR_STREAM_NAMED("evalTraj", "leftGapRelVel: " << leftVel);
        
        // leftGapPtIsDynamic = true; //JUST FOR DEBUGGING!
        
        if (leftVel.norm() > dynamic_thres) {leftGapPtIsDynamic = true;}
        if(leftGapPtIsDynamic)
        {
        


        //  Eigen::Vector4f leftState  = gap->getLeftGapPt()->getModel()->getGapState();

         geometry_msgs::Point posUncovertedL = path.poses.at(0).position; //todo delete this. Its used for trajpos but I want to delete that too because it's unused in the actual cbf function
                Eigen::Vector2f leftPos;
                leftPos.x() = posUncovertedL.x; 
                leftPos.y() = posUncovertedL.y;
            
        // dwa_traj.H_left = DPCBF(leftVel, leftGapRelPos, PosL, RbtVel);
        //   float v  = dwa_traj.v_cmd; 
        //   float w = dwa_traj.w_cmd; 
           

        //   Eigen::Vector2f u_nom = Eigen::Vector2f::Zero();
        //   Eigen::Vector2f left_cbf_vel = DPCBFProjectVelocity(leftVel, leftGapRelPos, PosL, u_nom, v, w); // todo rename this, it's not h, this is the new u i computed
        //   ROS_ERROR_STREAM("left_cbf_vel: " << left_cbf_vel); 

          // //// just testing ////////////////
        // Eigen::Vector2f leftVel  = leftState.tail<2>();
        // // Debug print
        // ROS_ERROR_STREAM_NAMED("DWA",
        //     "[GapVel] left=("  << leftVel.transpose()  << ")");
        // //// just testing ////////////////

        


        // Eigen::Vector2f leftVel  = latestGapLeftVelPtr_->at(leftGapPtID);

        // ROS_ERROR_STREAM("DWA leftGapPtID=" << leftGapPtID
        //     << " leftVel=" << leftVel.transpose());


         // ROS_ERROR_STREAM_NAMED("evalTraj", "leftGapRelVel: ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftGapRelVel);  
        //  leftGapRelPos = gap->getLeftGapPt()->getModel()->getState().head<2>(); //distance from robot to gap.
        // ROS_ERROR_STREAM_NAMED("evalTraj", "gap->leftGapPtModel_->getState(): ");
        // ROS_ERROR_STREAM_NAMED("evalTraj", leftGapRelPos);  
        // replaced dwa_traj with traj. I only used dwa_traj in generateTrajV2 because there were mulitple trajs per gap. but after that I repackage 
        // all the info like human vel into max's Trajectory struct anways
        traj.humanVelLeft = leftVel;
        traj.gapPosLeft = leftGapRelPos;
        traj.trajPosLeft = leftPos; //20260127 I plan on removing trajPos bc I'm not using it
        traj.robotVel = RbtVel;

        
        // ROS_ERROR_STREAM_NAMED("eval",
        // "inside traj evaluator right after populating the object:"
        // << " humanVel=(" << traj.humanVelLeft.x() << "," << traj.humanVelLeft.y() << ")"
        // << " gapPos=(" << traj.gapPosLeft.x() << "," << traj.gapPosLeft.y() << ")"
        // << " robotVel=(" << traj.robotVel.x() << "," << traj.robotVel.y() << ")");


        }

        // rightGapPtIsDynamic = gap->getRightGapPt()->getUngapID()>=0; 

        // int right_gapID = gap->getRightGapPt()->getModel()->getID();

            // ROS_ERROR_STREAM("[TE] requested RIGHT modelID=" << rightGapPtID);

            if (rightVelDictPtr_)
            {
                auto it = rightVelDictPtr_->find(rightGapPtID);
                // ROS_ERROR_STREAM("[TE] rightVelDictPtr_ size = "
                //     << std::distance(rightVelDictPtr_->begin(), rightVelDictPtr_->end()));

                if (it != rightVelDictPtr_->end())
                {
                    rightVel = it->second;
                }
                else
                {
                    ROS_WARN_STREAM("Missing rightVel for modelID=" << rightGapPtID);
                }
            }
            else
            {
                ROS_WARN_STREAM("rightVelDictPtr_ is null");
            }

            // ROS_ERROR_STREAM_NAMED("evalTraj", "rightGapRelVel: " << rightVel);

            // rightGapRelVel = rightVel;

        // rightGapPtIsDynamic = true; // debugging
        if (rightVel.norm() > dynamic_thres) {rightGapPtIsDynamic = true;
        // ROS_ERROR_STREAM_NAMED("evalTraj", "right gap point is dynamic greater than 0.15 threshold");
        }
        if(rightGapPtIsDynamic)
        {
            // gap->getRightGapPt()->getModel()->isolateGapDynamics();

            // rightGapRelPos =
            //     gap->getRightGapPt()->getModel()->getState().head<2>();

            traj.humanVelRight = rightVel;
            traj.gapPosRight = rightGapRelPos;

        //     ROS_ERROR_STREAM_NAMED("eval",
        // "right endpoint ------- inside traj evaluator right after populating the object:"
        // << " humanVelRight=(" << traj.humanVelRight.x() << "," << traj.humanVelRight.y() << ")"
        // << " gapPos=(" << traj.gapPosRight.x() << "," << traj.gapPosRight.y() << ")");
        
        }



        
    //     int i = 0; //ATODO DELETE
    //     for(i = 0; i < 1; i++)
    // {
        
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getRightGapPt()->getUngapID()");
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", gap->getRightGapPt()->getUngapID());
    //     ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "gap->getLeftGapPt()->getUngapID()");
    //     INFO("TrajectoryEvaluator", gap->getLeftGapPt()->getUngapID());
        
    // }
        }
    }

        // geometry_msgs::PoseArray path = traj.getPathRbtFrame();
        // std::vector<float> pathTiming = traj.getPathTiming();
        
        // posewiseCosts = std::vector<float>(pose_array.poses.size());

        float leftGapPtCost = 0; 
        float rightGapPtCost = 0; 
        float weight = cfg_->traj.w_relvel; 


        for (int i = 0; i < posewiseCosts.size(); i++) //todo combine with the path and pose cost loops above
        {
            if (leftGapPtIsDynamic){ // if(leftGapPtIsDynamic){
                geometry_msgs::Point posUncoverted = path.poses.at(i).position;
                Eigen::Vector2f Pos;
                Pos.x() = posUncoverted.x; 
                Pos.y() = posUncoverted.y;
                
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "leftGapRelPos (which is distance of robot to gap): " << leftGapRelPos); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftGapRelPos);

                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "Pos: "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", Pos);

                leftGapPtCost = relativeVelocityCost(leftVel, leftGapRelPos, Pos, RbtVel);

                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(leftGapRelVel, leftGapRelPos, RbtVel)"); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", leftGapPtCost); 
            }
            
            
            if(rightGapPtIsDynamic){ //(rightGapPtIsDynamic){
                geometry_msgs::Point posUncoverted = path.poses.at(i).position;
                Eigen::Vector2f Pos;
                Pos.x() = posUncoverted.x; 
                Pos.y() = posUncoverted.y;
                
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "rightGapRelPos: "); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightGapRelPos);

                rightGapPtCost = relativeVelocityCost(rightVel, rightGapRelPos, Pos, RbtVel);
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "relativeVelocityCost(rightGapRelVel, rightGapRelPos, RbtVel)"); 
                // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", rightGapPtCost); 
            }
            dwa_RelVelPoseCosts.at(i) = leftGapPtCost + rightGapPtCost;
            // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "dwa_RelVelPoseCosts.at(i): " << dwa_RelVelPoseCosts.at(i)); 
        }

        ///////////////////////////////////////////////////// social code //////////////////////////////////////////////////////////////////////////////////////////////////////

        ///////////////////// h related cost 

        float left_h = compute_h(traj.humanVelLeft, traj.gapPosLeft, traj.robotVel); 
        float right_h = compute_h(traj.humanVelRight, traj.gapPosRight, traj.robotVel); 
        // ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "            in traj eval left_h: " << left_h << " right_h: " << right_h);

        // Violation magnitudes (only when H < 0)
        float vL = std::max(0.0f, -left_h);
        float vR = std::max(0.0f, -right_h);

        // Combine endpoints (two bad sides worse than one)
        float V = vL + vR;

        // Deadband to ignore tiny negatives (tune this)
        const float epsilon = 0.05f;
        float V_eff = std::max(0.0f, V - epsilon);

        // Amplify (helps because H kinda saturates around 0.4–0.5)
        // float h_cost = V_eff * V_eff;
        float h_cost = V_eff; 
        
        ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "            h_cost: " <<  h_cost);
        terminalPoseCost += h_cost; // just taching h_cost onto this
        // because the h_cost is already added to terminalPoseCost, I don't need to do anyrepackaging related to that (i'm talking about this function btw dwa_evaluateTrajectory_outside)


            // Combine into a final cost
            // totalTrajCost =
            //     (std::accumulate(posewiseCosts.begin(), posewiseCosts.end(), float(0)) / posewiseCosts.size()) +
            //     cfg_->traj.w_path * avg_path_cost + terminalPoseCost + 
            //     cfg_->traj.w_relvel * (std::accumulate(dwa_RelVelPoseCosts.begin(), dwa_RelVelPoseCosts.end(), float(0)) / dwa_RelVelPoseCosts.size());
        
        
////////////////////////// just debugingg!!!!! ADD THE REST OF THE COSTS BACK!! 
            // totalTrajCost = cfg_->traj.w_relvel * (std::accumulate(dwa_RelVelPoseCosts.begin(), dwa_RelVelPoseCosts.end(), float(0)) / dwa_RelVelPoseCosts.size());
        
        
        // this is very ugly but I need to repackage posewiseCosts so it contains all the costs that occur at every pose. this is what compareToCurrentTraj() expects
            for (int i = 0; i < posewiseCosts.size(); i++) 
            {
                posewiseCosts.at(i) += dwa_RelVelPoseCosts.at(i); // tach on the relvel costs to it
                // FYI ^ thats always run, regardless of whether if(gap) returns false, which means you cant even compute relvel. Always running ^ that is a waste of compute, but it keeps code simple 
                
                // in future if you want to add path costs, you have to add it here as well 
                ROS_ERROR_STREAM_NAMED("TrajectoryEvaluator", "           pose " << i << " (dwa_evaluateTrajectory_reducedCurrentTraj cost: " << posewiseCosts.at(i) << "): ");
            }
            // because the h_cost is already added to terminalPoseCost, I don't need to do anyrepackaging related to that 

            // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "evaluateTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("TrajectoryEvaluator", "            evaluateTrajectory out of range exception: " << e.what());
        }
        
        return;
    }


      void TrajectoryEvaluator::update_human_info(
        // float & totalTrajCost, 
                                Trajectory & traj,
                                // std::vector<float> & posewiseCosts,
                                // // std::vector<float> &dwa_PathCosts, 
                                // float & terminalPoseCost,
                                // const std::vector<sensor_msgs::LaserScan> & futureScans,
                                // const int & scanIdx,
                                // const std::vector<geometry_msgs::PoseStamped> & globalPlanSnippet, 
                                // dynamic_gap::Gap* gap,
                                int rightGapPtID, 
                                Eigen::Vector2f rbtVel

                            )
                                // std::vector<float> &dwa_RelVelPoseCosts)
    {
        ROS_ERROR_STREAM("rbtVel: " << rbtVel);

        Eigen::Vector2f rightVel(0,0);
         if (rightVelDictPtr_)
            {
                auto it = rightVelDictPtr_->find(rightGapPtID);
                // ROS_ERROR_STREAM("[TE] rightVelDictPtr_ size = "
                //     << std::distance(rightVelDictPtr_->begin(), rightVelDictPtr_->end()));

                if (it != rightVelDictPtr_->end())
                {
                    rightVel = it->second;
                    ROS_ERROR_STREAM("rightgapID: " << rightGapPtID << " rightVel: " << rightVel);
                }
                else
                {
                    ROS_WARN_STREAM("Missing rightVel for modelID=" << rightGapPtID);
                }
            }
            else
            {
                ROS_WARN_STREAM("rightVelDictPtr_ is null");
            }


            Eigen::Vector2f rightPos(0,0);

            if (rightPosDictPtr_) {
                auto it = rightPosDictPtr_->find(rightGapPtID);
                if (it != rightPosDictPtr_->end()) {
                    rightPos = it->second;
                    ROS_ERROR_STREAM("rightgapID: " << rightGapPtID << " rightPos: " << rightPos);

                } else {
                    ROS_WARN_STREAM("Missing rightPos for modelID=" << rightGapPtID);
                }
            } else {
                ROS_WARN_STREAM("rightPosDictPtr_ is null");
            }

            // Eigen::Vector2f leftPos(0,0);

            // if (leftPosDictPtr_) {
            //     auto it = leftPosDictPtr_->find(rightGapPtID);
            //     if (it != leftPosDictPtr_->end()) {
            //         leftPos = it->second;
            //     } else {
            //         ROS_WARN_STREAM("Missing leftPos for modelID=" << rightGapPtID);
            //     }
            // } else {
            //     ROS_WARN_STREAM("leftPosDictPtr_ is null");
            // }


                }


    float TrajectoryEvaluator::terminalGoalCost(const geometry_msgs::Pose & pose) 
    {
        boost::mutex::scoped_lock planlock(globalPlanMutex_);
        // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", pose);
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            final pose: (" << pose.position.x << ", " << pose.position.y << "), local goal: (" << globalPathLocalWaypointRobotFrame_.pose.position.x << ", " << globalPathLocalWaypointRobotFrame_.pose.position.y << ")");
        float dx = pose.position.x - globalPathLocalWaypointRobotFrame_.pose.position.x;
        float dy = pose.position.y - globalPathLocalWaypointRobotFrame_.pose.position.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    float TrajectoryEvaluator::dwa_evaluatePose(const geometry_msgs::Pose & pose, const sensor_msgs::LaserScan scan_k) // to do make this more efficient 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        // sensor_msgs::LaserScan scan = *scan_.get();

        // obtain orientation and idx of pose

        // dist is size of scan
        std::vector<float> scan2RbtDists(scan_k.ranges.size());

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        // float currScan2RbtDist = 0.0;
        for (int i = 0; i < scan2RbtDists.size(); i++) 
        {
            scan2RbtDists.at(i) = dist2Pose(idx2theta(i), scan_k.ranges.at(i), pose);
        }

        auto iter = std::min_element(scan2RbtDists.begin(), scan2RbtDists.end());
        // std::cout << "robot pose: " << pose.position.x << ", " << pose.position.y << ")" << std::endl;
        int minDistIdx = std::distance(scan2RbtDists.begin(), iter);
        float range = scan_k.ranges.at(minDistIdx);
        float theta = idx2theta(minDistIdx);
        float cost = chapterCost(*iter);
        //std::cout << *iter << ", regular cost: " << cost << std::endl;
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            robot pose: " << pose.position.x << ", " << pose.position.y << 
                                                        ", closest scan point: " << range * std::cos(theta) << ", " << range * std::sin(theta) << ", cost: " << cost);
        return cost;
    }

    float TrajectoryEvaluator::evaluatePose(const geometry_msgs::Pose & pose, const sensor_msgs::LaserScan scan_k) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        // sensor_msgs::LaserScan scan = *scan_.get();

        // obtain orientation and idx of pose

        // dist is size of scan
        std::vector<float> scan2RbtDists(scan_k.ranges.size());

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        // float currScan2RbtDist = 0.0;
        for (int i = 0; i < scan2RbtDists.size(); i++) 
        {
            scan2RbtDists.at(i) = dist2Pose(idx2theta(i), scan_k.ranges.at(i), pose);
        }

        auto iter = std::min_element(scan2RbtDists.begin(), scan2RbtDists.end());
        // std::cout << "robot pose: " << pose.position.x << ", " << pose.position.y << ")" << std::endl;
        int minDistIdx = std::distance(scan2RbtDists.begin(), iter);
        float range = scan_k.ranges.at(minDistIdx);
        float theta = idx2theta(minDistIdx);
        float cost = chapterCost(*iter);
        //std::cout << *iter << ", regular cost: " << cost << std::endl;
        ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "            robot pose: " << pose.position.x << ", " << pose.position.y << 
                                                        ", closest scan point: " << range * std::cos(theta) << ", " << range * std::sin(theta) << ", cost: " << cost);
        return cost;
    }

    float TrajectoryEvaluator::chapterCost(const float & rbtToScanDist) 
    {
        // if the distance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterCost with distance: " << d << std::endl;
        float inflRbtRad = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 

        float inflRbtToScanDist = rbtToScanDist - inflRbtRad;

        if (inflRbtToScanDist < 0.0) 
        {   
            return std::numeric_limits<float>::infinity();
        }

        // if pose is sufficiently far away from scan, return no cost
        if (rbtToScanDist > cfg_->traj.max_pose_to_scan_dist) 
            return 0;

        /*   y
         *   ^
         * Q |\ 
         *   | \
         *   |  \
         *       `
         *   |    ` 
         *         ` ---
         *   |          _________
         *   --------------------->x
         *
         */


        return cfg_->traj.Q * std::exp(-cfg_->traj.pen_exp_weight * inflRbtToScanDist);
    }
    float TrajectoryEvaluator::calcDistToGlobalPath(const geometry_msgs::Pose& pose,
                                                const std::vector<geometry_msgs::PoseStamped>& globalPlanSnippet)
{
    if (globalPlanSnippet.empty())
        return 0.0f;

    // We'll use the first and last visible global plan points
    geometry_msgs::Point e1 = globalPlanSnippet.front().pose.position;
    geometry_msgs::Point e2 = globalPlanSnippet.back().pose.position;

    // Line coefficients (ax + by + c = 0)
    const float a = e2.y - e1.y;
    const float b = -(e2.x - e1.x);
    const float c = -a * e1.x - b * e1.y;

    geometry_msgs::Point p = pose.position;

    // perpendicular distance from point to line
    float dist = std::fabs(a * p.x + b * p.y + c) / (std::hypot(a, b) + 1e-6);

    return dist;
}

float TrajectoryEvaluator::relativeVelocityCost(
        Eigen::Vector2f humanVel,
        Eigen::Vector2f relativeGapPos,
        Eigen::Vector2f trajPos,
        Eigen::Vector2f robotVel)
{
    const float eps = 1e-3f;

    // Magnitudes
    float vr = robotVel.norm();
    float vh = humanVel.norm();

    // If either not moving → no perpendicular conflict
    if (vr < eps || vh < eps)
        return 0.0f;

    // 2D cross product magnitude = |x1*y2 - y1*x2|
    float cross = std::abs(robotVel.x()*humanVel.y()
                           - robotVel.y()*humanVel.x());

    // Perpendicular factor |sin(theta)| in [0,1]
    float sin_theta = cross / (vr*vh + eps);

    // Distance from trajectory sample to dynamic gap point (human)
    Eigen::Vector2f d = relativeGapPos - trajPos;
    float dist = std::max(d.norm(), eps);
    float divide_factor = 2; 

    // MINIMAL perpendicular-cutoff cost
    float cost = (sin_theta + vh) / (dist * divide_factor);

    // DEBUG OUTPUT -----------------------------------------------------------
    // ROS_ERROR_STREAM("\n[PerpVelocityCost Debug]"
    //     << "\nrobotVel:        " << robotVel.transpose() << "  |Vr|=" << vr
    //     << "\nhumanVel:        " << humanVel.transpose() << "  |Vh|=" << vh
    //     << "\nrelativeGapPos:  " << relativeGapPos.transpose()
    //     << "\ntrajPos:         " << trajPos.transpose()
    //     << "\nposToGap d:      " << d.transpose() << "  dist=" << dist
    //     << "\ncross:           " << cross
    //     << "\nsin(theta):      " << sin_theta
    //     << "\ncost:            " << cost
    //     << "\n-----------------------------------------------------"
    // );

    return cost;
}


// ============================================================
// 2) Safety filter: project u_nom onto linearized h(u) >= 0
// ============================================================
//
// This replaces the *projection operator* idea:
// - Compute barrier h at u_nom.
// - If h >= 0, do nothing.
// - If h < 0, compute grad wrt u via finite diff (2D).
// - Project u_nom onto halfspace defined by linearization.
//

// Eigen::Vector2f TrajectoryEvaluator::DPCBFProjectVelocity(const Eigen::Vector2f& humanVel,
//                                                           const Eigen::Vector2f& gapPos,
//                                                           const Eigen::Vector2f& trajPos,
//                                                           const Eigen::Vector2f& u_nom,
//                                                           float v_cmd, 
//                                                           float w_cmd)
// {
//     if (!std::isnan(v_cmd) && !std::isnan(w_cmd))
//                 {
//                     // ROS_INFO_STREAM_NAMED("Controller", "Using external DWA command v=" << v_cmd << ", w=" << w_cmd);
//                     geometry_msgs::Twist rawCmdVel = geometry_msgs::Twist();

//                     rawCmdVel.linear.x  = v_cmd;
//                     rawCmdVel.linear.y  = 0.0;
//                     rawCmdVel.angular.z = w_cmd;
//                     // ROS_ERROR_STREAM_NAMED("Controller", "right before processCmdVelNonHolonomic H_left: " << H_left);

//                     geometry_msgs::Twist nonholoCmdVel = rawCmdVel; // putting into this variable so code below stays the same 
//                 }

//     else
//                 { 
//                         ROS_ERROR_STREAM_NAMED("Controller",
//                                             "DPCBFProjectVelocity: external cmd invalid (NaN). "
//                                             << "v_cmd=" << v_cmd << ", w_cmd=" << w_cmd
//                                             << ". Returning u_nom.");
//                         return u_nom;
//                 }
//     //--------------------------- start of pasting processCmdVelNonHolonomic code ----------------------------
//     geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
//     tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
//     float currYaw = quaternionToYaw(currQuat); 

//     // get current x,y,theta
//     geometry_msgs::Point currPosn = currentPoseOdomFrame.position;
//     Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

//     ROS_INFO_STREAM_NAMED("Controller", "        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);

//      // Map nonholonomic command velocities to holonomic command velocities
//     geometry_msgs::Twist holoCmdVel = geometry_msgs::Twist();
//     holoCmdVel.linear.x = nonholoCmdVel.linear.x;
//     holoCmdVel.linear.y = l_ * nonholoCmdVel.angular.z;
//     holoCmdVel.angular.z = 0.0;

//     //!!!!!!!! todo  add rest of code from processCmdVelNonHolonomic !!!!!!!!!!!!!!!!!!!!!!1
//     //--------------------------- end of processCmdVelNonHolonomic code ----------------------------
//     // Evaluate at nominal
    
//     u_nom.x() = holoCmdVel.linear.x; 
//     u_nom.y() = holoCmdVel.linear.y; 

//     float h0 = this->DPCBF(humanVel, gapPos, trajPos, u_nom);

//     // Already safe: no change
//     if (h0 >= 0.0f) {
//         return u_nom;
//     }

//     // Finite-difference gradient wrt u = [u_x, u_y]
//     float fd_eps = 1e-3f; 
//     const float du = std::max(fd_eps, 1e-6f);

//     Eigen::Vector2f u_dx = u_nom;
//     u_dx.x() += du;
//     float h_dx = this->DPCBF(humanVel, gapPos, trajPos, u_dx);

//     Eigen::Vector2f u_dy = u_nom;
//     u_dy.y() += du;
//     float h_dy = this->DPCBF(humanVel, gapPos, trajPos, u_dy);

//     Eigen::Vector2f grad_h;
//     grad_h.x() = (h_dx - h0) / du;
//     grad_h.y() = (h_dy - h0) / du;


//     float denom = grad_h.squaredNorm();

//     // If gradient is degenerate, fallback (stop or keep nominal)
//     if (denom < 1e-9f) {
//         // Conservative fallback: stop
//         return Eigen::Vector2f::Zero();
//     }

//     // Closed-form orthogonal projection of u_nom onto linearized halfspace:
//     // u_safe = u_nom + (-h0 / ||grad||^2) * grad
//     Eigen::Vector2f u_safe = u_nom + (-h0 / denom) * grad_h;

//     return u_safe;
// }


}
