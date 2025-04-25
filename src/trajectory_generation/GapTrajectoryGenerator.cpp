#include <dynamic_gap/trajectory_generation/GapTrajectoryGenerator.h>

namespace dynamic_gap 
{
    Trajectory GapTrajectoryGenerator::generateGoToGoalTrajectoryV2(Gap * selectedGap, 
                                                                    const geometry_msgs::PoseStamped & currPose, 
                                                                    // const geometry_msgs::TwistStamped & currVel,
                                                                    const geometry_msgs::PoseStamped & globalGoalRobotFrame)
    {
        geometry_msgs::PoseArray path;
        std::vector<float> pathTiming;

        try
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            running go to goal");                

            // std::chrono::steady_clock::time_point generateTrajectoryStartTime = std::chrono::steady_clock::now();

            path.header.stamp = currPose.header.stamp;
            path.header.frame_id = cfg_->sensor_frame_id;

            Eigen::Vector4f rbtState(currPose.pose.position.x, 
                                        currPose.pose.position.y,
                                        0.0, 
                                        0.0);

            // get gap points in cartesian
            float xLeft = 0.0, yLeft = 0.0, xRight = 0.0, yRight = 0.0;
            selectedGap->getManipulatedLCartesian(xLeft, yLeft);
            selectedGap->getManipulatedRCartesian(xRight, yRight);

            robotAndGapState x = {rbtState[0], rbtState[1], 
                                    xLeft, yLeft, 
                                    xRight, yRight, 
                                    globalGoalRobotFrame.pose.position.x, 
                                    globalGoalRobotFrame.pose.position.y};
        
            float desiredHeading = std::atan2(globalGoalRobotFrame.pose.position.y - rbtState[1], 
                                                globalGoalRobotFrame.pose.position.x - rbtState[0]);
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            desiredHeading: " << desiredHeading);
            Eigen::Quaternionf desiredQ = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                                            Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                                            Eigen::AngleAxisf(desiredHeading, Eigen::Vector3f::UnitZ());
            desiredQ.normalize();
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            desiredQ: " << desiredQ.x() << ", " << desiredQ.y() << ", " << desiredQ.z() << ", " << desiredQ.w());

            float t_max = std::min(selectedGap->getGapLifespan(), cfg_->traj.integrate_maxt);

            TrajectoryLogger logger(path, pathTiming, cfg_->robot_frame_id, desiredQ);

            GoToGoal goToGoal(cfg_->rbt.vx_absmax);
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<robotAndGapState>(),
                                            goToGoal, x, 0.0f, t_max, cfg_->traj.integrate_stept, logger);
            // float generateTrajectoryTime = timeTaken(generateTrajectoryStartTime);
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            generateTrajectory (g2g) time taken: " << generateTrajectoryTime << " seconds");                
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "            generateTrajectory (g2g) out of range exception: " << e.what());
        }
        
        Trajectory traj(path, pathTiming);
        return traj;
    }

    Trajectory GapTrajectoryGenerator::generateTrajectoryV2(Gap * selectedGap, 
                                                            const geometry_msgs::PoseStamped & currPose, 
                                                            // const geometry_msgs::TwistStamped & currVel,
                                                            const geometry_msgs::PoseStamped & globalGoalRobotFrame) 
    {
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "        [generateTrajectory()]");

        geometry_msgs::PoseArray path;
        std::vector<float> pathTiming;

        // std::chrono::steady_clock::time_point generateTrajectoryStartTime = std::chrono::steady_clock::now();

        path.header.stamp = currPose.header.stamp;
        path.header.frame_id = cfg_->sensor_frame_id;

        Eigen::Vector4f rbtState(currPose.pose.position.x, 
                                    currPose.pose.position.y,
                                    0.0, 
                                    0.0);

        // get gap points in cartesian
        float xLeft = 0.0, yLeft = 0.0, xRight = 0.0, yRight = 0.0;
        selectedGap->getManipulatedLCartesian(xLeft, yLeft);
        selectedGap->getManipulatedRCartesian(xRight, yRight);

        Eigen::Vector4f leftGapState = selectedGap->getLeftGapPt()->getModel()->getManipGapState();
        Eigen::Vector4f rightGapState = selectedGap->getRightGapPt()->getModel()->getManipGapState();
        Eigen::Vector2f initialGoal = selectedGap->getGoal()->getOrigGoalPos(); // (selectedGap->goal.x_, selectedGap->goal.y_);
        Eigen::Vector2f goalPtVel = selectedGap->getGoal()->getOrigGoalVel(); // (selectedGap->goal.vx_, selectedGap->goal.vy_);

        Eigen::Vector2f leftGapPtVel = leftGapState.tail(2);
        Eigen::Vector2f rightGapPtVel = rightGapState.tail(2);

        float leftVelX = leftGapPtVel[0];
        float leftVelY = leftGapPtVel[1];

        float rightVelX = rightGapPtVel[0];
        float rightVelY = rightGapPtVel[1];

        float gapGoalVelX = goalPtVel[0];
        float gapGoalVelY = goalPtVel[1];

        // Eigen::Vector2f goalPtVel(gapGoalVelX, gapGoalVelY);

        // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial robot pos: (" << rbtState[0] << ", " << rbtState[1] << ")");
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            inital robot velocity: " << rbtState[2] << ", " << rbtState[3] << ")");
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial left gap point: (" << xLeft << ", " << yLeft << "), initial right point: (" << xRight << ", " << yRight << ")"); 
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial left gap point velocity: (" << leftVelX << ", " << leftVelY << "), initial right gap point velocity: (" << rightVelX << ", " << rightVelY << ")"); 
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial goal: (" << initialGoal[0] << ", " << initialGoal[1] << ")"); 
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial goal velocity: (" << gapGoalVelX << ", " << gapGoalVelY << ")"); 

        if (cfg_->planning.pursuit_guidance_method == 0) // pure pursuit
        {
            throw std::runtime_error("Pure pursuit not implemented");

        } else // parallel navigation
        {
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            running pursuit guidance");                

            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial robot pos: (" << rbtState[0] << ", " << rbtState[1] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            inital robot velocity: " << rbtState[2] << ", " << rbtState[3] << ")");
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial left gap point: (" << xLeft << ", " << yLeft << "), initial right point: (" << xRight << ", " << yRight << ")"); 
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial left gap point velocity: (" << leftVelX << ", " << leftVelY << "), initial right gap point velocity: (" << rightVelX << ", " << rightVelY << ")"); 
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial goal: (" << initialGoal[0] << ", " << initialGoal[1] << ")"); 
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            initial goal velocity: (" << gapGoalVelX << ", " << gapGoalVelY << ")"); 

            // For PN, we will drive at terminal Goal, so pass it on to know when to stop                
            Eigen::Vector2f terminalGoal = selectedGap->getGoal()->getTermGoalPos();  // (selectedGap->terminalGoal.x_, selectedGap->terminalGoal.y_);

            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            actual terminal goal: (" << terminalGoal[0] << ", " << terminalGoal[1] << ")"); 

            // // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "pre-integration, x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);

            robotAndGapState x = {rbtState[0], rbtState[1], xLeft, yLeft, xRight, yRight, initialGoal[0], initialGoal[1]};
            
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            intercept time: " << selectedGap->getGapLifespan()); 
            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            intercept angle: " << selectedGap->getGammaInterceptGoal()); 

            float desiredHeading = selectedGap->getGammaInterceptGoal();

            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            desiredHeading: " << desiredHeading);

            Eigen::Quaternionf desiredQ = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                                            Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                                            Eigen::AngleAxisf(desiredHeading, Eigen::Vector3f::UnitZ());

            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            desiredQ: " << desiredQ.x() << ", " << desiredQ.y() << ", " << desiredQ.z() << ", " << desiredQ.w());

            TrajectoryLogger logger(path, pathTiming, cfg_->robot_frame_id, desiredQ);

            ParallelNavigation parallelNavigation(selectedGap->getGammaInterceptGoal(), 
                                                    cfg_->rbt.vx_absmax,
                                                    cfg_->rbt.r_inscr,
                                                    leftGapPtVel,
                                                    rightGapPtVel,
                                                    goalPtVel,
                                                    terminalGoal);

            float t_max = std::min(selectedGap->getGapLifespan(), cfg_->traj.integrate_maxt);

            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<robotAndGapState>(),
                                                    parallelNavigation, x, 0.0f, t_max, 
                                                    cfg_->traj.integrate_stept, logger);
        }


        Trajectory traj(path, pathTiming);
        // float generateTrajectoryTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - generateTrajectoryStartTime).count() * 1.0e-6;
        // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            generateTrajectory (pursuit guidance) time taken: " << generateTrajectoryTime << " seconds");
        return traj;
    }

    Trajectory GapTrajectoryGenerator::generateIdlingTrajectoryV2(Gap * gap,
                                                                    const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                                                    const Trajectory & runningTraj)
                                                                    
                                                                    // const geometry_msgs::PoseStamped & rbtPoseInOdomFrame)
    {
        geometry_msgs::PoseArray path;
        std::vector<float> pathTiming;

        try
        {
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            running go to goal");                

            // std::chrono::steady_clock::time_point generateTrajectoryStartTime = std::chrono::steady_clock::now();

            path.header.stamp = rbtPoseInSensorFrame.header.stamp;
            path.header.frame_id = cfg_->sensor_frame_id;

            Eigen::Vector4f rbtState(rbtPoseInSensorFrame.pose.position.x, 
                                        rbtPoseInSensorFrame.pose.position.y,
                                        0.0, 
                                        0.0);

            robotAndGapState x = {rbtState[0], rbtState[1], 
                                    0.0, 0.0, 
                                    0.0, 0.0, 
                                    0.0, 0.0};
        

            geometry_msgs::PoseArray pathRbtFrame = runningTraj.getPathRbtFrame();

            Eigen::Quaternionf desiredQ(0, 0, 0, 1);
            if (pathRbtFrame.poses.empty())
            {
                ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "            running traj is empty for idling");
                // return Trajectory(path, pathTiming);
            } else
            {
                geometry_msgs::Pose lastPose = pathRbtFrame.poses.back();
                desiredQ = Eigen::Quaternionf(lastPose.orientation.w, 
                                                lastPose.orientation.x, 
                                                lastPose.orientation.y, 
                                                lastPose.orientation.z);
                
            }

            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            desiredQ: " << desiredQ.x() << ", " << desiredQ.y() << ", " << desiredQ.z() << ", " << desiredQ.w());
            
            TrajectoryLogger logger(path, pathTiming, cfg_->robot_frame_id, desiredQ);
            Idling idling(cfg_->rbt.vx_absmax);

            float t_max;
            if (gap)
            {
                t_max = std::min(gap->getGapLifespan(), cfg_->traj.integrate_maxt);
            } else
            {
                t_max = cfg_->traj.integrate_maxt;
            }


            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<robotAndGapState>(),
                                                    idling, x, 0.0f, t_max, 
                                                    cfg_->traj.integrate_stept, logger);

            // float generateTrajectoryTime = timeTaken(generateTrajectoryStartTime);
            
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            generateTrajectory (g2g) time taken: " << generateTrajectoryTime << " seconds");                
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "            generateTrajectory (idling) out of range exception: " << e.what());
        }
        
        Trajectory traj(path, pathTiming);
        return traj;        
    }

    // Transform local trajectory between two frames of choice
    geometry_msgs::PoseArray GapTrajectoryGenerator::transformPath(const geometry_msgs::PoseArray & path,
                                                                    const geometry_msgs::TransformStamped & transform)
    {
        geometry_msgs::PoseStamped sourcePose;
        sourcePose.header.frame_id = transform.header.frame_id; // cfg_->robot_frame_id;

        geometry_msgs::PoseStamped destPose;
        destPose.header.frame_id = transform.child_frame_id; // cfg_->odom_frame_id;

        geometry_msgs::PoseArray transformedPath;
        for (const geometry_msgs::Pose pose : path.poses)
        {
            sourcePose.pose = pose;
            tf2::doTransform(sourcePose, destPose, transform);
            transformedPath.poses.push_back(destPose.pose);
        }
        transformedPath.header.frame_id = destPose.header.frame_id; // cfg_->odom_frame_id;
        transformedPath.header.stamp = transform.header.stamp;

        if (transformedPath.poses.size() != path.poses.size())
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "transformed path size mismatch: " << transformedPath.poses.size() << " vs " << path.poses.size());
        }

        if (transformedPath.header.frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "transformed path frame id is empty");
        }

        // ROS_WARN_STREAM("leaving transform back with length: " << transformedPath.poses.size());
        return transformedPath;
    }

    Trajectory GapTrajectoryGenerator::processTrajectory(const Trajectory & traj)
                                                            // const geometry_msgs::PoseStamped & currPose, 
                                                            // const geometry_msgs::TwistStamped & currVel,
                                                            // const bool & prune)
    {
        ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            [processTrajectory]");

        geometry_msgs::PoseArray rawPath = traj.getPathRbtFrame();
        std::vector<float> rawPathTiming = traj.getPathTiming();
        
        try
        {    
            // remove any poses that go beyond the max scan range
            int lastIdx = rawPath.poses.size();
            for (int i = 0; i < rawPath.poses.size(); i++)
            {
                geometry_msgs::Pose pose = rawPath.poses[i];
                float poseDist = std::sqrt(std::pow(pose.position.x, 2) + std::pow(pose.position.y, 2));
                if (poseDist > (cfg_->scan.range_max - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio))
                {
                    lastIdx = i;
                    break;
                }
            }

            // delete all poses after the lastIdx
            rawPath.poses.erase(rawPath.poses.begin() + lastIdx, rawPath.poses.end());
            rawPathTiming.erase(rawPathTiming.begin() + lastIdx, rawPathTiming.end());


            // geometry_msgs::Pose originPose;
            // originPose.position.x = currPose.pose.position.x;
            // originPose.position.y = currPose.pose.position.y;
            // originPose.position.z = 0;
            // originPose.orientation.x = 0;
            // originPose.orientation.y = 0;
            // originPose.orientation.z = 0;
            // originPose.orientation.w = 1;
            // std::cout << "entering at : " << path.poses.size() << std::endl;
            //std::cout << "starting pose: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << std::endl; 
            //std::cout << "final pose: " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << std::endl;

            // add origin pose to the path
            // std::vector<geometry_msgs::Pose> processedPoses = rawPath.poses;
            // processedPoses.push_back(originPose);
            // processedPathTiming.push_back(0.0);
            // processedPoses.insert(processedPoses.end(), rawPath.poses.begin(), rawPath.poses.end());
            // processedPathTiming.insert(processedPathTiming.end(), rawPathTiming.begin(), rawPathTiming.end());

            // float poseToPoseDistThreshold = 0.1;
            // for (int i = 1; i < rawPath.poses.size(); i++) 
            // {
            //     geometry_msgs::Pose rawPose = rawPath.poses[i];
            //     poseToPoseDiffX = rawPose.position.x - processedPoses.back().position.x;
            //     poseToPoseDiffY = rawPose.position.y - processedPoses.back().position.y;
            //     poseToPoseDist = sqrt(pow(poseToPoseDiffX, 2) + pow(poseToPoseDiffY, 2));
            //     if (!prune || (prune && poseToPoseDist > poseToPoseDistThreshold))
            //     {
            //         // // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "poseToPoseDist " << i << " kept at " << poseToPoseDist);
            //         processedPoses.push_back(rawPose);
            //         processedPathTiming.push_back(rawPathTiming.at(i));
            //     } else 
            //     {
            //         // // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "poseToPoseDist " << i << " cut at " << poseToPoseDist);
            //     }
            // }
            // ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "leaving at : " << processedPoses.size());
            
            geometry_msgs::PoseArray processedPath = rawPath;        
            // processedPath.poses = processedPoses;
            std::vector<float> processedPathTiming = rawPathTiming;

            // float poseToPoseDiffX = 0.0, poseToPoseDiffY = 0.0, poseToPoseDist = 0.0;

            // // Fix rotation along local trajectory
            // geometry_msgs::Pose processedPose, prevProcessedPose;
            // Eigen::Quaternionf q;
            // float poseToPoseDiffTheta = 0.0;
            // for (int idx = 1; idx < processedPath.poses.size(); idx++)
            // {
            //     processedPose = processedPath.poses.at(idx);
            //     prevProcessedPose = processedPath.poses.at(idx-1);
            //     poseToPoseDiffX = processedPose.position.x - prevProcessedPose.position.x;
            //     poseToPoseDiffY = processedPose.position.y - prevProcessedPose.position.y;
            //     poseToPoseDiffTheta = std::atan2(poseToPoseDiffY, poseToPoseDiffX);
            //     q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
            //         Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
            //         Eigen::AngleAxisf(poseToPoseDiffTheta, Eigen::Vector3f::UnitZ());
            //     q.normalize();
            //     processedPath.poses.at(idx-1).orientation.x = q.x();
            //     processedPath.poses.at(idx-1).orientation.y = q.y();
            //     processedPath.poses.at(idx-1).orientation.z = q.z();
            //     processedPath.poses.at(idx-1).orientation.w = q.w();
            // }

            // // Set rotation for last pose
            // if (processedPath.poses.size() > 1)
            // {
            //     int lastIdx = processedPath.poses.size() - 1;

            //     processedPath.poses.at(lastIdx).orientation.x = processedPath.poses.at(lastIdx-1).orientation.x;
            //     processedPath.poses.at(lastIdx).orientation.y = processedPath.poses.at(lastIdx-1).orientation.y;
            //     processedPath.poses.at(lastIdx).orientation.z = processedPath.poses.at(lastIdx-1).orientation.z;
            //     processedPath.poses.at(lastIdx).orientation.w = processedPath.poses.at(lastIdx-1).orientation.w;
            // }

            ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            processedPath: ");
            for (int idx = 0; idx < processedPath.poses.size(); idx++)
            {
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "             idx: " << idx);
                geometry_msgs::Pose processedPose = processedPath.poses.at(idx);
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "                 pose: (" << processedPose.position.x << ", " << processedPose.position.y << ")");
                ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "                 orientation: (" << processedPose.orientation.x << ", " << processedPose.orientation.y << ", " << processedPose.orientation.z << ", " << processedPose.orientation.w << ")");
            
            }

            Trajectory processedTrajectory(processedPath, processedPathTiming);
            return processedTrajectory;            
        } catch (const std::out_of_range& e) 
        {
            ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "            processTrajectory out of range exception: " << e.what());

            geometry_msgs::PoseArray emptyPath;
            emptyPath.header.stamp = traj.getPathRbtFrame().header.stamp;
            emptyPath.header.frame_id = cfg_->sensor_frame_id;
            std::vector<float> emptyPathTiming;
            Trajectory emptyTrajectory(emptyPath, emptyPathTiming);
            return emptyTrajectory;
        }
    
    }

    // Trajectory GapTrajectoryGenerator::processIdlingTrajectory(const Trajectory & traj,
    //                                                             const Trajectory & runningTraj)
    // {
    //     ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            [processIdlingTrajectory]");

    //     geometry_msgs::PoseArray rawPath = traj.getPathRbtFrame();
    //     std::vector<float> rawPathTiming = traj.getPathTiming();

    //     geometry_msgs::PoseArray processedPath = rawPath;        
    //     // processedPath.poses = processedPoses;
    //     std::vector<float> processedPathTiming = rawPathTiming;

    //     Eigen::Quaternionf q;
    //     if (runningTraj.getPathRbtFrame().poses.size() > 0)
    //     {
    //         geometry_msgs::PoseArray runningPathRbtFrame = runningTraj.getPathRbtFrame();
    //         geometry_msgs::Pose runningPose = runningPathRbtFrame.poses.back();
    //         q = Eigen::Quaternionf(runningPose.orientation.w, 
    //                                 runningPose.orientation.x, 
    //                                 runningPose.orientation.y, 
    //                                 runningPose.orientation.z);
    //         q.normalize();
    //     } else
    //     {
    //         ROS_WARN_STREAM_NAMED("GapTrajectoryGeneratorV2", "            running trajectory is empty");
    //     }

    //     ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w());

    //     for (int idx = 0; idx < processedPath.poses.size(); idx++)
    //     {
    //         // set orientation of processed path to be the same as running trajectory
    //         geometry_msgs::Pose processedPose = processedPath.poses.at(idx);

    //         processedPose.orientation.x = q.x();
    //         processedPose.orientation.y = q.y();
    //         processedPose.orientation.z = q.z();
    //         processedPose.orientation.w = q.w();
    //         processedPath.poses.at(idx) = processedPose;
    //     }

    //     ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "            processedPath: ");
    //     for (int idx = 0; idx < processedPath.poses.size(); idx++)
    //     {
    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "             idx: " << idx);
    //         geometry_msgs::Pose processedPose = processedPath.poses.at(idx);
    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "                 pose: (" << processedPose.position.x << ", " << processedPose.position.y << ")");
    //         ROS_INFO_STREAM_NAMED("GapTrajectoryGeneratorV2", "                 orientation: (" << processedPose.orientation.x << ", " << processedPose.orientation.y << ", " << processedPose.orientation.z << ", " << processedPose.orientation.w << ")");
        
    //     }

    //     Trajectory processedTrajectory(processedPath, processedPathTiming);
    //     return processedTrajectory;           
    // }
}