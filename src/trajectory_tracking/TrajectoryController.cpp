#include <dynamic_gap/trajectory_tracking/TrajectoryController.h>

namespace dynamic_gap
{
    TrajectoryController::TrajectoryController(ros::NodeHandle& nh, const DynamicGapConfig& cfg)
    {
        projOpPublisher_ = nh.advertise<visualization_msgs::Marker>("po_dir", 10);
        cfg_ = & cfg;

        manualVelX_ = 0.0f;
        manualVelY_ = 0.0f;
        manualVelAng_ = 0.0f;
        manualVelLinIncrement_ = 0.05f;
        manualVelAngIncrement_ = 0.10f;

        prevDesired_ = geometry_msgs::Pose();

        cp_ = 1.0;
        clambda_ = 1.0;
        lambda_ = 0.5;
        epsilon_ = 0.01;

        l_ = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; // error.norm();

        // startTime_ = std::chrono::steady_clock::now();
    }

    void TrajectoryController::updateParams(const ControlParameters & ctrlParams)
    {
        ctrlParams_ = ctrlParams;
    }


    void TrajectoryController::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan)
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    // geometry_msgs::Twist TrajectoryController::manualControlLawPrescribed(const geometry_msgs::Pose & current) 
    // {
    //     ROS_INFO_STREAM_NAMED("Controller", "Manual Control");

    //     geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

    //     // std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();

    //     // int64_t timeElapsed = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - startTime_).count();
    //     // double timeElapsedSec = timeElapsed * 1.0e-6;

    //     // Pure linear
    //     // if (timeElapsedSec < 5)
    //     // {
    //     //     cmdVel.linear.x = 0.4f;
    //     //     cmdVel.linear.y = 0.0f;
    //     //     cmdVel.angular.z = 0.0f;
    //     // } else if (timeElapsedSec < 10)
    //     // {
    //     //     cmdVel.linear.x = -0.4f;
    //     //     cmdVel.linear.y = 0.0;
    //     //     cmdVel.angular.z = 0.0f;
    //     // } else
    //     // {
    //     //     startTime_ = std::chrono::steady_clock::now();
    //     // }

    //     // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
    //     geometry_msgs::Quaternion currOrient = current.orientation;
    //     tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
    //     float currYaw = quaternionToYaw(currQuat); 

    //     // Linear + rotational
    //     float ptOneTime = 6.0;
    //     float ptTwoTime = 6.0;
    //     if (timeElapsedSec < ptOneTime)
    //     {
    //         float desTheta = (M_PI_OVER_TWO) + (M_PI_OVER_FOUR) * std::sin(timeElapsedSec * (TWO_M_PI / ptOneTime)); // should go from 0.0 to pi/4
    //         cmdVel.linear.x = 0.75f;
    //         cmdVel.linear.y = 0.0f;
    //         cmdVel.angular.z = 1.0 * (desTheta - currYaw);
    //     } else if (timeElapsedSec < (ptOneTime + ptTwoTime))
    //     {
    //         float desTheta = (M_PI_OVER_TWO) - (M_PI_OVER_FOUR) * std::sin((timeElapsedSec - ptOneTime) * (TWO_M_PI / ptTwoTime));
    //         cmdVel.linear.x = -0.75f;
    //         cmdVel.linear.y = 0.0;
    //         cmdVel.angular.z = 1.0 * (desTheta - currYaw);
    //     } 
    //     // else
    //     // {
    //     //     startTime_ = std::chrono::steady_clock::now();
    //     // }        

    //     return cmdVel;
    // }

    geometry_msgs::Twist TrajectoryController::manualControlLawReconfig() 
    {
        ROS_INFO_STREAM_NAMED("Controller", "Manual Control");

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        // char key = getch();
        // ROS_INFO_STREAM_NAMED("Controller", "Keyboard input: " << key);

        // if (key == 'w')
        //     manualVelX_ += manualVelLinIncrement_;
        // else if (key == 'a')
        //     manualVelY_ += manualVelLinIncrement_;
        // else if (key == 's')
        // {
        //     manualVelX_ = 0.0f;
        //     manualVelY_ = 0.0f;
        //     manualVelAng_ = 0.0f;
        // } else if (key == 'd')
        //     manualVelY_ -= manualVelLinIncrement_;
        // else if (key == 'o')
        //     manualVelAng_ += manualVelAngIncrement_;
        // else if (key == 'p')
        //     manualVelAng_ -= manualVelAngIncrement_;

        cmdVel.linear.x = ctrlParams_.linear_vel_x_;
        cmdVel.linear.y = ctrlParams_.linear_vel_y_;
        cmdVel.angular.z = ctrlParams_.angular_vel_z_;
    
        ROS_WARN_STREAM_NAMED("Controller", "Manual control command:");
        ROS_WARN_STREAM_NAMED("Controller", "       linear x: " << cmdVel.linear.x);
        ROS_WARN_STREAM_NAMED("Controller", "       linear y: " << cmdVel.linear.y);
        ROS_WARN_STREAM_NAMED("Controller", "       angular z: " << cmdVel.angular.z);

        return cmdVel;
    }

    // For non-blocking keyboard inputs
    int getch(void)
    {
        int ch;
        struct termios oldt;
        struct termios newt;
        
        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        
        // Make required changes and apply the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        tcsetattr(fileno(stdin), TCSANOW, &newt);
        
        // Get the current character
        ch = getchar();
        
        // Reapply old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        
        return ch;
    }

    geometry_msgs::Twist TrajectoryController::manualControlLawKeyboard() 
    {
        ROS_INFO_STREAM_NAMED("Controller", "Manual Control");

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        char key = getch();
        // ROS_INFO_STREAM_NAMED("Controller", "Keyboard input: " << key);

        if (key == 'w')
            manualVelX_ += manualVelLinIncrement_;
        else if (key == 'a')
            manualVelY_ += manualVelLinIncrement_;
        else if (key == 's')
        {
            manualVelX_ = 0.0f;
            manualVelY_ = 0.0f;
            manualVelAng_ = 0.0f;
        } else if (key == 'd')
            manualVelY_ -= manualVelLinIncrement_;
        else if (key == 'o')
            manualVelAng_ += manualVelAngIncrement_;
        else if (key == 'p')
            manualVelAng_ -= manualVelAngIncrement_;

        cmdVel.linear.x = manualVelX_;
        cmdVel.linear.y = manualVelY_;
        cmdVel.angular.z = manualVelAng_;
    
        ROS_WARN_STREAM_NAMED("Controller", "Manual control command:");
        ROS_WARN_STREAM_NAMED("Controller", "       linear x: " << cmdVel.linear.x);
        ROS_WARN_STREAM_NAMED("Controller", "       linear y: " << cmdVel.linear.y);
        ROS_WARN_STREAM_NAMED("Controller", "       angular z: " << cmdVel.angular.z);

        return cmdVel;
    }

    /*
    Taken from the code provided in stdr_simulator repo. Probably does not perform too well.
    */
    geometry_msgs::Twist TrajectoryController::obstacleAvoidanceControlLaw() 
    {
        ROS_INFO_STREAM_NAMED("Controller", "obstacle avoidance control");
        float safeDirX = 0;
        float safeDirY = 0;                                   
        
        float scanRange = 0.0, scanTheta = 0.0;
        for (int i = 0; i < scan_->ranges.size(); i++) 
        {
            scanRange = scan_->ranges.at(i);
            scanTheta =  idx2theta(i);

            safeDirX += epsilonDivide(-1.0 * std::cos(scanTheta), pow(scanRange, 2));
            safeDirY += epsilonDivide(-1.0 * std::sin(scanTheta), pow(scanRange, 2));
        }

        safeDirX /= scan_->ranges.size();
        safeDirY /= scan_->ranges.size();

        float cmdVelX = safeDirX;
        float cmdVelY = safeDirY;
        float cmdVelTheta = 0.0;


        ROS_INFO_STREAM_NAMED("Controller", "raw safe vels: " << cmdVelX << ", " << cmdVelY);
        ROS_INFO_STREAM_NAMED("Controller", "weighted safe vels: " << cmdVelX << ", " << cmdVelY);

        clipRobotVelocity(cmdVelX, cmdVelY, cmdVelTheta);

        ROS_INFO_STREAM_NAMED("Controller", "final safe vels: " << cmdVelX << ", " << cmdVelY);

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();
        cmdVel.linear.x = cmdVelX;
        cmdVel.linear.y = cmdVelY; 
        cmdVel.angular.z = cmdVelTheta;    

        return cmdVel;
    }

    /*
    Taken from the code provided in stdr_simulator repo. Probably does not perform too well.
    */
    geometry_msgs::Twist TrajectoryController::obstacleAvoidanceControlLawNonHolonomic() 
    {
        ROS_INFO_STREAM_NAMED("Controller", "obstacle avoidance control");
        float safeDirX = 0;
        float safeDirZ = 0;                                   
        
        float scanRange = 0.0, scanTheta = 0.0;
        for (int i = 0; i < scan_->ranges.size(); i++) 
        {
            scanRange = scan_->ranges.at(i);
            scanTheta =  idx2theta(i);

            safeDirX += epsilonDivide(-1.0 * std::cos(scanTheta), pow(scanRange, 2));
            safeDirZ += epsilonDivide(-1.0 * std::sin(scanTheta), pow(scanRange, 2));
        }

        safeDirX /= scan_->ranges.size();
        safeDirZ /= scan_->ranges.size();

        ROS_INFO_STREAM_NAMED("Controller", "raw safe vels: x: " << safeDirX << ", z: " << safeDirZ);

        // clipRobotVelocity(cmdVelX, cmdVelY, cmdVelTheta);

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        float clippedCmdVelX = 0.0;
        if (std::abs(safeDirX) < cfg_->rbt.vx_absmax)
        {
            clippedCmdVelX = safeDirX;
        } else
        {
            clippedCmdVelX = cfg_->rbt.vx_absmax * epsilonDivide(safeDirX, std::abs(safeDirX));
        }

        cmdVel.linear.x = clippedCmdVelX;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, safeDirZ));

        ROS_INFO_STREAM_NAMED("Controller", "final safe vels: " << cmdVel.linear.x << ", " << cmdVel.angular.z);

        return cmdVel;
    }

    geometry_msgs::Twist TrajectoryController::constantVelocityControlLawNonHolonomicLookahead(const geometry_msgs::Pose & currentPoseOdomFrame, 
                                                                                                const geometry_msgs::Pose & desiredPoseOdomFrame,
                                                                                                const float & desiredSpeed) 
    { 
        ROS_INFO_STREAM_NAMED("Controller", "    [constantVelocityControlLawNonHolonomicLookahead()]");
        // Setup Vars
        // boost::mutex::scoped_lock lock(scanMutex_);

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        // get current x,y,theta
        geometry_msgs::Point currPosn = currentPoseOdomFrame.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        ROS_INFO_STREAM_NAMED("Controller", "        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);

        // obtaining RPY of desired orientation
        geometry_msgs::Point desPosn = desiredPoseOdomFrame.position;
        geometry_msgs::Quaternion desOrient = desiredPoseOdomFrame.orientation;
        tf::Quaternion desQuat(desOrient.x, desOrient.y, desOrient.z, desOrient.w);
        float desYaw = quaternionToYaw(desQuat);

        ROS_INFO_STREAM_NAMED("Controller", "        desired pose x: " << desiredPoseOdomFrame.position.x << ", y: " << desiredPoseOdomFrame.position.y << ", yaw: "<< desYaw);

        // get desired x,y,theta
        Eigen::Matrix2cf desRbtTransform = getComplexMatrix(desPosn.x, desPosn.y, desYaw);

        // get x,y,theta error
        Eigen::Matrix2cf errorMat = currRbtTransform.inverse() * desRbtTransform;
        float errorX = desPosn.x - currPosn.x; // errorMat.real()(0, 1);
        float errorY = desPosn.y - currPosn.y; // errorMat.imag()(0, 1);
        float errorTheta = std::arg(errorMat(0, 0));

        Eigen::Vector2f error(errorX, errorY);

        Eigen::Vector2f errorDir = epsilonDivide(error, error.norm());

        Eigen::Vector2f constantVelocityCommand = desiredSpeed * errorDir;

        // lookahead distance

        // float l_adj = l; // 0.5 * l;

        ROS_INFO_STREAM_NAMED("Controller", "        error: " << error.transpose() << ", l: " << l_); //  << ", l_adj: " << l_adj

        Eigen::Matrix2f negRotMat = getRotMat(-currYaw);

        Eigen::Matrix2f nidMat = Eigen::Matrix2f::Identity();
        nidMat(1, 1) = (1.0 / l_);

        Eigen::Vector2f nonholoVelocityCommand = nidMat * negRotMat * constantVelocityCommand;

        float velLinXFeedback = nonholoVelocityCommand[0];
        float velLinYFeedback = 0.0;
        float velAngFeedback = nonholoVelocityCommand[1];

        // Just storing desired displacement for now
        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;

        return cmdVel; 
    }

    geometry_msgs::Twist TrajectoryController::constantVelocityControlLaw(const geometry_msgs::Pose & currentPoseOdomFrame, 
                                                                            const geometry_msgs::Pose & desiredPoseOdomFrame,
                                                                            const float & desiredSpeed) 
    { 
        ROS_INFO_STREAM_NAMED("Controller", "    [constantVelocityControlLaw()]");
        // Setup Vars
        // boost::mutex::scoped_lock lock(scanMutex_);

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        // get current x,y,theta
        geometry_msgs::Point currPosn = currentPoseOdomFrame.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        // obtaining RPY of desired orientation
        geometry_msgs::Point desPosn = desiredPoseOdomFrame.position;
        geometry_msgs::Quaternion desOrient = desiredPoseOdomFrame.orientation;
        tf::Quaternion desQuat(desOrient.x, desOrient.y, desOrient.z, desOrient.w);

        float desYaw = quaternionToYaw(desQuat);

        // get desired x,y,theta
        Eigen::Matrix2cf desRbtTransform = getComplexMatrix(desPosn.x, desPosn.y, desYaw);

        // get x,y,theta error
        Eigen::Matrix2cf errorMat = currRbtTransform.inverse() * desRbtTransform;
        float errorX = errorMat.real()(0, 1);
        float errorY = errorMat.imag()(0, 1);
        float errorTheta = std::arg(errorMat(0, 0));

        Eigen::Vector2f error(errorX, errorY);
        Eigen::Vector2f errorDir = epsilonDivide(error, error.norm());

        Eigen::Vector2f constantVelocityCommand = desiredSpeed * errorDir;

        float velLinXFeedback = constantVelocityCommand[0];
        float velLinYFeedback = constantVelocityCommand[1];
        float velAngFeedback = 0.0; 

        ROS_INFO_STREAM_NAMED("Controller", "        generating control signal");            
        ROS_INFO_STREAM_NAMED("Controller", "        desired pose x: " << desiredPoseOdomFrame.position.x << ", y: " << desiredPoseOdomFrame.position.y << ", yaw: "<< desYaw);
        ROS_INFO_STREAM_NAMED("Controller", "        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);
        ROS_INFO_STREAM_NAMED("Controller", "        errorX: " << errorX << ", errorY: " << errorY << ", errorTheta: " << errorTheta);
        ROS_INFO_STREAM_NAMED("Controller", "        Feedback command velocities, v_x: " << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);
        
        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;

        return cmdVel; 

    }

    // const geometry_msgs::TwistStamped & currRbtVel, 
    // const geometry_msgs::TwistStamped & currRbtAcc
    geometry_msgs::Twist TrajectoryController::processCmdVel(const geometry_msgs::Twist & rawCmdVel,
                                                             const geometry_msgs::PoseStamped & rbtPoseInSensorFrame) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "    [processCmdVel()]");

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        float velLinXFeedback = rawCmdVel.linear.x;
        float velLinYFeedback = rawCmdVel.linear.y;
        float velAngFeedback = rawCmdVel.angular.z;

        // ROS_INFO_STREAM_NAMED("Controller", rbtPoseInSensorFrame.pose);
        float minRangeTheta = 0;
        float minRange = 0;

        ROS_INFO_STREAM_NAMED("Controller", "        feedback command velocities: " << velLinXFeedback << ", " << velLinYFeedback);

        // applies PO
        float velLinXSafe = 0.;
        float velLinYSafe = 0.;
        
        if (cfg_->planning.projection_operator)
        {
            ROS_INFO_STREAM_NAMED("Controller", "        running projection operator");
            
            Eigen::Vector2f cmdVelFeedback(rawCmdVel.linear.x, rawCmdVel.linear.y);

            runProjectionOperator(rbtPoseInSensorFrame,
                                    cmdVelFeedback, velLinXSafe, velLinYSafe,
                                    minRangeTheta, minRange);
            
        } else 
        {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(10, "Controller", "Projection operator off");
        }
        
        float weightedVelLinXSafe = cfg_->projection.k_po_x * velLinXSafe;
        float weightedVelLinYSafe = cfg_->projection.k_po_x * velLinYSafe;

        ROS_INFO_STREAM_NAMED("Controller", "        safe command velocity, v_x:" << weightedVelLinXSafe << ", v_y: " << weightedVelLinYSafe);

        // cmdVel_safe
        visualizeProjectionOperator(weightedVelLinXSafe, weightedVelLinYSafe, minRangeTheta, minRange);

        velLinXFeedback += weightedVelLinXSafe;
        velLinYFeedback += weightedVelLinYSafe; 

        ROS_INFO_STREAM_NAMED("Controller", "        summed command velocity, v_x:" << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);
        clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        clipped command velocity, v_x:" << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);

        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;

        // ROS_INFO_STREAM_NAMED("Controller", "ultimate command velocity: " << cmdVel.linear.x << ", " << cmdVel.linear.y << ", " << cmdVel.angular.z);

        return cmdVel;
    }
  
    geometry_msgs::Twist TrajectoryController::processCmdVelNonHolonomic(const geometry_msgs::Pose & currentPoseOdomFrame,
                                                                            const geometry_msgs::Pose & desiredPoseOdomFrame,
                                                                            const geometry_msgs::Twist & nonholoCmdVel,
                                                                            const geometry_msgs::PoseStamped & rbtPoseInSensorFrame) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "    [processCmdVelNonHolonomic()]");

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        // get current x,y,theta
        geometry_msgs::Point currPosn = currentPoseOdomFrame.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        ROS_INFO_STREAM_NAMED("Controller", "        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);

        // // obtaining RPY of desired orientation
        // geometry_msgs::Point desPosn = desiredPoseOdomFrame.position;
        // geometry_msgs::Quaternion desOrient = desiredPoseOdomFrame.orientation;
        // tf::Quaternion desQuat(desOrient.x, desOrient.y, desOrient.z, desOrient.w);
        // float desYaw = quaternionToYaw(desQuat);

        // ROS_INFO_STREAM_NAMED("Controller", "        desired pose x: " << desiredPoseOdomFrame.position.x << ", y: " << desiredPoseOdomFrame.position.y << ", yaw: "<< desYaw);

        // // get desired x,y,theta
        // Eigen::Matrix2cf desRbtTransform = getComplexMatrix(desPosn.x, desPosn.y, desYaw);

        // // get x,y,theta error
        // Eigen::Matrix2cf errorMat = currRbtTransform.inverse() * desRbtTransform;
        // float errorX = desPosn.x - currPosn.x; // errorMat.real()(0, 1);
        // float errorY = desPosn.y - currPosn.y; // errorMat.imag()(0, 1);
        // float errorTheta = std::arg(errorMat(0, 0));

        // Eigen::Vector2f error(errorX, errorY);

        // float l = error.norm();

        // float l_adj = l; // 0.5 * l;

        // Map nonholonomic command velocities to holonomic command velocities
        geometry_msgs::Twist holoCmdVel = geometry_msgs::Twist();
        holoCmdVel.linear.x = nonholoCmdVel.linear.x;
        holoCmdVel.linear.y = l_ * nonholoCmdVel.angular.z;
        holoCmdVel.angular.z = 0.0;



        // float errorX = rawCmdVel.linear.x;
        // float errorY = rawCmdVel.linear.y;
        // float errorTheta = rawCmdVel.angular.z;

        // ROS_INFO_STREAM_NAMED("Controller", rbtPoseInSensorFrame.pose);
        float minRangeTheta = 0;
        float minRange = 0;

        // ROS_INFO_STREAM_NAMED("Controller", "        feedback errors: x: " << errorX << ", y: " << errorY << ", theta: " << errorTheta);

        // applies PO
        float velLinXSafe = 0.;
        float velLinYSafe = 0.;
        
        if (cfg_->planning.projection_operator)
        {
            ROS_INFO_STREAM_NAMED("Controller", "        running projection operator");

            Eigen::Vector2f holoCmdVelVector(holoCmdVel.linear.x, holoCmdVel.linear.y);

            runProjectionOperator(rbtPoseInSensorFrame,
                                    holoCmdVelVector, velLinXSafe, velLinYSafe,
                                    minRangeTheta, minRange);
            
        } else 
        {
            ROS_DEBUG_STREAM_THROTTLE_NAMED(10, "Controller", "Projection operator off");
        }
        
        float weightedVelLinXSafe = cfg_->projection.k_po_x * velLinXSafe;
        float weightedVelLinYSafe = cfg_->projection.k_po_x * velLinYSafe;

        // cmdVel_safe
        visualizeProjectionOperator(weightedVelLinXSafe, weightedVelLinYSafe, minRangeTheta, minRange);

        ROS_INFO_STREAM_NAMED("Controller", "        safe desired direction, v_x:" << weightedVelLinXSafe << ", v_y: " << weightedVelLinYSafe);

        float safeErrorX = holoCmdVel.linear.x + weightedVelLinXSafe;
        float safeErrorY = holoCmdVel.linear.y + weightedVelLinYSafe; 

        ROS_INFO_STREAM_NAMED("Controller", "        summed desired direction, v_x:" << safeErrorX << ", v_y: " << safeErrorY);
        Eigen::Vector2f safeError(safeErrorX, safeErrorY);

        // Eigen::Matrix2f negRotMat = getRotMat(-currYaw);

        Eigen::Matrix2f nidMat = Eigen::Matrix2f::Identity();
        nidMat(1, 1) = (1.0 / l_);

        Eigen::Vector2f nonholoVelocityCommand = nidMat * safeError; // negRotMat * 

        float velLinXFeedback = nonholoVelocityCommand[0]; // nonholoCmdVel.linear.x; // 
        float velLinYFeedback = 0.0;
        float velAngFeedback = nonholoVelocityCommand[1]; // nonholoCmdVel.angular.z; //  

        ROS_INFO_STREAM_NAMED("Controller", "        generating nonholonomic control signal");            
        ROS_INFO_STREAM_NAMED("Controller", "        Feedback command velocities, v_x: " << velLinXFeedback << ", v_ang: " << velAngFeedback);

        float clippedVelLinXFeedback = 0.0;
        if (std::abs(velLinXFeedback) < cfg_->rbt.vx_absmax)
        {
            clippedVelLinXFeedback = velLinXFeedback;
        } else
        {
            clippedVelLinXFeedback = cfg_->rbt.vx_absmax * epsilonDivide(velLinXFeedback, std::abs(velLinXFeedback));
        }

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();
        cmdVel.linear.x = clippedVelLinXFeedback;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, velAngFeedback));

        // clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        clipped nonholonomic command velocity, v_x:" << cmdVel.linear.x << ", v_ang: " << cmdVel.angular.z);

        return cmdVel;
    }


    float TrajectoryController::DPCBF(
        Eigen::Vector2f humanVel,
        Eigen::Vector2f relativeGapPos,
        Eigen::Vector2f trajPos,
        Eigen::Vector2f robotVel)
    {
    // Eigen::Vector2f relVel = -relativeVel; // so velocity and position vector point in the same direction for the dot product
    //  Eigen::Vector2f v_rel = robotVel - humanVel; // human vel is the vel of the dynamic endpoint which represents the human  
     //todo might need to fix relvel^ , DPCBF code uses: v_rel = v_obs - v_robot
     Eigen::Vector2f v_rel = -robotVel + humanVel; // human vel is the vel of the dynamic endpoint which represents the human  

    ROS_ERROR_STREAM("negative v_rel: " << v_rel.transpose());
    // ROS_ERROR_STREAM("relativeGapPos: " << relativeGapPos.transpose());
    // ROS_ERROR_STREAM("trajPos: " << trajPos.transpose());
    // ROS_ERROR_STREAM("robotVel: " << robotVel.transpose());
    // ROS_ERROR_STREAM_NAMED("relvel: ", relVel << ", relativeGapPos: " << relativeGapPos << ", robotVel: " << robotVel);

    // ROS_ERROR_STREAM_NAMED("relvel cost", "relativeGapPos: ");
    // ROS_ERROR_STREAM_NAMED("relvel cost", relativeGapPos);

    Eigen::Vector2f p_rel = relativeGapPos - trajPos;// distance between the current pos we're looking at and the gap point (which represents the dynamic obstacle)
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

    

    geometry_msgs::Twist TrajectoryController::cbf_processCmdVelNonHolonomic(const geometry_msgs::Pose & currentPoseOdomFrame,
                                                                            const geometry_msgs::Pose & desiredPoseOdomFrame,
                                                                            const geometry_msgs::Twist & nonholoCmdVel,
                                                                            const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                                                            Eigen::Vector2f humanVel,
                                                                            Eigen::Vector2f relativeGapPos,
                                                                            Eigen::Vector2f trajPos,
                                                                            Eigen::Vector2f robotVel) 
    {
        
        trajPos = Eigen::Vector2f::Zero(); //todo, get rid of trajPos all together

        if (!std::isfinite(humanVel.x()) || !std::isfinite(humanVel.y()) ||
            !std::isfinite(relativeGapPos.x()) || !std::isfinite(relativeGapPos.y()) ||
            !std::isfinite(trajPos.x()) || !std::isfinite(trajPos.y()) ||
            !std::isfinite(robotVel.x()) || !std::isfinite(robotVel.y()))
        {
            // ROS_ERROR_STREAM_NAMED("CBFDBG",
            //     "NaN/Inf in CBF inputs");

            ROS_ERROR_STREAM_NAMED("CBFDBG",
            " Look a NaN/Inf in CBF inputs: "
            << " humanVel=(" << humanVel.x() << "," << humanVel.y() << ")"
            << " gapPos=(" << relativeGapPos.x() << "," << relativeGapPos.y() << ")"
            << " trajPos=(" << trajPos.x() << "," << trajPos.y() << ")"
            << " robotVel=(" << robotVel.x() << "," << robotVel.y() << ")");

   
            return nonholoCmdVel;
        }


        //--------------------------- start of pasting processCmdVelNonHolonomic code ----------------------------
        geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        // get current x,y,theta
        geometry_msgs::Point currPosn = currentPoseOdomFrame.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        ROS_INFO_STREAM_NAMED("Controller", "        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);

        // Map nonholonomic command velocities to holonomic command velocities
        geometry_msgs::Twist holoCmdVel = geometry_msgs::Twist();
        holoCmdVel.linear.x = nonholoCmdVel.linear.x;
        holoCmdVel.linear.y = l_ * nonholoCmdVel.angular.z;
        holoCmdVel.angular.z = 0.0;

         ROS_ERROR_STREAM_NAMED("CBF",
        "\n[CBF INPUT]\n"
        << " nonholo: v=" << nonholoCmdVel.linear.x
        << " w=" << nonholoCmdVel.angular.z
        << "\n holo: ux=" << holoCmdVel.linear.x
        << " uy=" << holoCmdVel.linear.y);

        ROS_ERROR_STREAM_NAMED("CBFDBG",
        "CBF state:"
        << " humanVel=(" << humanVel.x() << "," << humanVel.y() << ")"
        << " gapPos=(" << relativeGapPos.x() << "," << relativeGapPos.y() << ")"
        << " trajPos=(" << trajPos.x() << "," << trajPos.y() << ")"
        << " robotVel=(" << robotVel.x() << "," << robotVel.y() << ")");

        //!!!!!!!! todo  add rest of code from processCmdVelNonHolonomic !!!!!!!!!!!!!!!!!!!!!!1
        //--------------------------- end of processCmdVelNonHolonomic code ----------------------------
        // Evaluate at nominal
        Eigen::Vector2f u_nom; 
        u_nom.x() = holoCmdVel.linear.x; 
        u_nom.y() = holoCmdVel.linear.y; 

        float h0 = DPCBF(humanVel, relativeGapPos, trajPos, u_nom); // i haven't pasted this function into this file yet btw
        ROS_ERROR_STREAM_NAMED("CBFDBG", "h0 = " << h0);

        // Already safe: no change
        if (h0 >= 0.0f) {
            return nonholoCmdVel;
        }

        // Finite-difference gradient wrt u = [u_x, u_y]
        float fd_eps = 1e-3f; 
        const float du = std::max(fd_eps, 1e-6f);

        Eigen::Vector2f u_dx = u_nom;
        u_dx.x() += du;
        float h_dx = DPCBF(humanVel, relativeGapPos, trajPos, u_dx);

        Eigen::Vector2f u_dy = u_nom;
        u_dy.y() += du;
        float h_dy = DPCBF(humanVel, relativeGapPos, trajPos, u_dy);

        Eigen::Vector2f grad_h;
        grad_h.x() = (h_dx - h0) / du;
        grad_h.y() = (h_dy - h0) / du;

        ROS_ERROR_STREAM_NAMED("CBFDBG",
        "h_dx=" << h_dx << " h_dy=" << h_dy);


        float denom = grad_h.squaredNorm();

        // If gradient is degenerate, fallback (stop or keep nominal)
        if (denom < 1e-6f) {
            ROS_INFO_STREAM_NAMED("Controller","denom = grad_h.squaredNorm(): " << denom << " < 1e-6, returning original cmdVel");            
            return nonholoCmdVel;
        }
        ROS_ERROR_STREAM_NAMED("CBFDBG",
        "grad_h=(" << grad_h.x() << "," << grad_h.y() << ")"
        << " denom=" << denom);

        ROS_ERROR_STREAM_NAMED("CBFDBG", " denom=" << denom);


        // Closed-form orthogonal projection of u_nom onto linearized halfspace:
        // u_safe = u_nom + (-h0 / ||grad||^2) * grad
        Eigen::Vector2f u_safe = u_nom + (-h0 / denom) * grad_h;
        // return u_safe;
        Eigen::Vector2f safeError(u_safe.x(), u_safe.y());

        //-----------------------------------------------more original processCmdVelNonHolonomic code------------------------------------------------------------------


        Eigen::Matrix2f nidMat = Eigen::Matrix2f::Identity();
        nidMat(1, 1) = (1.0 / l_);

        Eigen::Vector2f nonholoVelocityCommand = nidMat * safeError; // negRotMat * 

        float velLinXFeedback = nonholoVelocityCommand[0]; // nonholoCmdVel.linear.x; // 
        float velLinYFeedback = 0.0;
        float velAngFeedback = nonholoVelocityCommand[1]; // nonholoCmdVel.angular.z; //  

        ROS_INFO_STREAM_NAMED("Controller", "        generating nonholonomic control signal");            
        ROS_INFO_STREAM_NAMED("Controller", "        Feedback command velocities, v_x: " << velLinXFeedback << ", v_ang: " << velAngFeedback);

        float clippedVelLinXFeedback = 0.0;
        if (std::abs(velLinXFeedback) < cfg_->rbt.vx_absmax)
        {
            clippedVelLinXFeedback = velLinXFeedback;
        } else
        {
            clippedVelLinXFeedback = cfg_->rbt.vx_absmax * epsilonDivide(velLinXFeedback, std::abs(velLinXFeedback));
        }

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();
        cmdVel.linear.x = clippedVelLinXFeedback;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, velAngFeedback));

        // clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        clipped nonholonomic command velocity, v_x:" << cmdVel.linear.x << ", v_ang: " << cmdVel.angular.z);

         ROS_ERROR_STREAM_NAMED("CBF",
        "\n----------------------OUTPUT----------------------------------\n"
        << " v=" << cmdVel.linear.x
        << " w=" << cmdVel.angular.z
         );

        return cmdVel;

        //-----------------------------------------------end of original processCmdVelNonHolonomic code------------------------------------------------------------------


        // cbf note: add some of this clipping stuff PO did back in 
        /* 
        // ROS_INFO_STREAM_NAMED("Controller", rbtPoseInSensorFrame.pose);
        float minRangeTheta = 0;
        float minRange = 0;

        // ROS_INFO_STREAM_NAMED("Controller", "        feedback errors: x: " << errorX << ", y: " << errorY << ", theta: " << errorTheta);

        // applies PO
        float velLinXSafe = 0.;
        float velLinYSafe = 0.;
        
        // if (cfg_->planning.projection_operator) // not using PO
        // {
        //     ROS_INFO_STREAM_NAMED("Controller", "        running projection operator");

        //     Eigen::Vector2f holoCmdVelVector(holoCmdVel.linear.x, holoCmdVel.linear.y);

        //     runProjectionOperator(rbtPoseInSensorFrame,
        //                             holoCmdVelVector, velLinXSafe, velLinYSafe,
        //                             minRangeTheta, minRange);
            
        // } else 
        // {
        //     ROS_DEBUG_STREAM_THROTTLE_NAMED(10, "Controller", "Projection operator off");
        // }
        
        float weightedVelLinXSafe = cfg_->projection.k_po_x * velLinXSafe;
        float weightedVelLinYSafe = cfg_->projection.k_po_x * velLinYSafe;

        // cmdVel_safe
        visualizeProjectionOperator(weightedVelLinXSafe, weightedVelLinYSafe, minRangeTheta, minRange);

        ROS_INFO_STREAM_NAMED("Controller", "        safe desired direction, v_x:" << weightedVelLinXSafe << ", v_y: " << weightedVelLinYSafe);

        float safeErrorX = holoCmdVel.linear.x + weightedVelLinXSafe;
        float safeErrorY = holoCmdVel.linear.y + weightedVelLinYSafe; 

        ROS_INFO_STREAM_NAMED("Controller", "        summed desired direction, v_x:" << safeErrorX << ", v_y: " << safeErrorY);
        Eigen::Vector2f safeError(safeErrorX, safeErrorY);

        // Eigen::Matrix2f negRotMat = getRotMat(-currYaw);

        Eigen::Matrix2f nidMat = Eigen::Matrix2f::Identity();
        nidMat(1, 1) = (1.0 / l_);

        Eigen::Vector2f nonholoVelocityCommand = nidMat * safeError; // negRotMat * 

        float velLinXFeedback = nonholoVelocityCommand[0]; // nonholoCmdVel.linear.x; // 
        float velLinYFeedback = 0.0;
        float velAngFeedback = nonholoVelocityCommand[1]; // nonholoCmdVel.angular.z; //  

        ROS_INFO_STREAM_NAMED("Controller", "        generating nonholonomic control signal");            
        ROS_INFO_STREAM_NAMED("Controller", "        Feedback command velocities, v_x: " << velLinXFeedback << ", v_ang: " << velAngFeedback);

        float clippedVelLinXFeedback = 0.0;
        if (std::abs(velLinXFeedback) < cfg_->rbt.vx_absmax)
        {
            clippedVelLinXFeedback = velLinXFeedback;
        } else
        {
            clippedVelLinXFeedback = cfg_->rbt.vx_absmax * epsilonDivide(velLinXFeedback, std::abs(velLinXFeedback));
        }

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();
        cmdVel.linear.x = clippedVelLinXFeedback;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, velAngFeedback));

        // clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        clipped nonholonomic command velocity, v_x:" << cmdVel.linear.x << ", v_ang: " << cmdVel.angular.z);

        return cmdVel;
        */
    }
    

    void TrajectoryController::visualizeProjectionOperator(const float & weightedVelLinXSafe, 
                                                           const float & weightedVelLinYSafe,
                                                           const float & minRangeTheta, 
                                                           const float & minRange) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "[visualizeProjectionOperator()]");

        if (cfg_->robot_frame_id.empty())
        {
            ROS_WARN_STREAM_NAMED("Controller", "robot_frame_id not set, cannot visualize projection operator");
            return;
        }

        visualization_msgs::Marker projOpMarker;
        projOpMarker.header.frame_id = cfg_->robot_frame_id;
        projOpMarker.header.stamp = ros::Time();
        projOpMarker.id = 0;

        projOpMarker.type = visualization_msgs::Marker::ARROW;
        projOpMarker.action = visualization_msgs::Marker::ADD;
        projOpMarker.pose.position.x = minRange * std::cos(minRangeTheta);
        projOpMarker.pose.position.y = minRange * std::sin(minRangeTheta);
        projOpMarker.pose.position.z = 0.01;
        float dir = std::atan2(weightedVelLinYSafe, weightedVelLinXSafe);
        tf2::Quaternion projOpQuat;
        projOpQuat.setRPY(0, 0, dir);
        projOpMarker.pose.orientation = tf2::toMsg(projOpQuat);

        projOpMarker.scale.x = sqrt(pow(weightedVelLinXSafe, 2) + pow(weightedVelLinYSafe, 2)) + 0.00001;
        projOpMarker.scale.y = 0.1;
        projOpMarker.scale.z = 0.000001;
        

        
        projOpMarker.color.a = 1;
        projOpMarker.color.r = 0.0;
        projOpMarker.color.g = 0.0;
        projOpMarker.color.b = 0.0;
        projOpMarker.lifetime = ros::Duration(0);

        projOpPublisher_.publish(projOpMarker);
    }

    void TrajectoryController::clipRobotVelocity(float & velLinXFeedback, float & velLinYFeedback, float & velAngFeedback) 
    {
        float speedLinXFeedback = std::abs(velLinXFeedback);
        float speedLinYFeedback = std::abs(velLinYFeedback);
        
        if (speedLinXFeedback <= cfg_->rbt.vx_absmax && speedLinYFeedback <= cfg_->rbt.vy_absmax) 
        {
            // std::cout << "not clipping" << std::endl;
        } else 
        {
            velLinXFeedback *= epsilonDivide(cfg_->rbt.vx_absmax, std::max(speedLinXFeedback, speedLinYFeedback));
            velLinYFeedback *= epsilonDivide(cfg_->rbt.vy_absmax, std::max(speedLinXFeedback, speedLinYFeedback));
        }

        std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, velAngFeedback));
        return;
    }

    void TrajectoryController::runProjectionOperator(const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                                     Eigen::Vector2f & cmdVelFeedback,
                                                     float & velLinXSafe, float & velLinYSafe,
                                                     float & minRangeTheta, float & minRange) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "        [runProjectionOperator()]");
        float Psi = 0.0;
        Eigen::Vector2f dPsiDx(0.0, 0.0);

        // iterates through current egocircle and finds the minimum distance to the robot's pose
        // ROS_INFO_STREAM_NAMED("Controller", "rbtPoseInSensorFrame pose: " << rbtPoseInSensorFrame.pose.position.x << ", " << rbtPoseInSensorFrame.pose.position.y);
        std::vector<float> minScanDists(scan_->ranges.size());
        float theta = 0.0, dist = 0.0;
        for (int i = 0; i < minScanDists.size(); i++) 
        {
            theta = idx2theta(i);
            dist = scan_->ranges.at(i);
            minScanDists.at(i) = dist2Pose(theta, dist, rbtPoseInSensorFrame.pose);
        }
        auto minDistScanIter = std::min_element(minScanDists.begin(), minScanDists.end());
        int minDistScanIdx = std::distance(minScanDists.begin(), minDistScanIter);
        minRangeTheta = idx2theta(minDistScanIdx);

        minRange = minScanDists.at(minDistScanIdx);

        ROS_INFO_STREAM_NAMED("Controller", "           minDistScanIdx: " << minDistScanIdx << ", minRangeTheta: "<< minRangeTheta << ", minRange: " << minRange);
        // ROS_INFO_STREAM_NAMED("Controller", "min_x: " << min_x << ", min_y: " << min_y);
              
        Eigen::Vector2f closestScanPtToRobot(-minRange * std::cos(minRangeTheta), -minRange * std::sin(minRangeTheta));

        // float psi = 0.0;
        // Eigen::Vector2f dPsiDx(0.0, 0.0);
        // Eigen::Vector3f PsiDerAndPsi = 
        calculateProjectionOperator(closestScanPtToRobot, Psi, dPsiDx); // return Psi, and dPsiDx
        // dPsiDx = Eigen::Vector2f(PsiDerAndPsi(0), PsiDerAndPsi(1));

        Eigen::Vector2f normDPsiDx = dPsiDx.normalized();

        float projOpDotProd = cmdVelFeedback.dot(normDPsiDx);

        // Psi = PsiDerAndPsi(2);

        ROS_INFO_STREAM_NAMED("Controller", "           Psi: " << Psi);
        ROS_INFO_STREAM_NAMED("Controller", "           dPsiDx: " << dPsiDx[0] << ", " << dPsiDx[1]);
        ROS_INFO_STREAM_NAMED("Controller", "           Dot product check: " << projOpDotProd);

        if (Psi >= 0 && projOpDotProd >= 0)
        {
            velLinXSafe = - Psi * projOpDotProd * normDPsiDx(0);
            velLinYSafe = - Psi * projOpDotProd * normDPsiDx(1);
        }
        ROS_INFO_STREAM_NAMED("Controller", "           cmdVel_safe: " << velLinXSafe << ", " << velLinYSafe);
    }

    void TrajectoryController::calculateProjectionOperator(const Eigen::Vector2f & closestScanPtToRobot,
                                                            float & Psi, Eigen::Vector2f & dPsiDx)

    {
        float rUnity = cfg_->projection.r_unity;
        float rZero = cfg_->projection.r_zero;

        float minRange = closestScanPtToRobot.norm(); // sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2)); // (closest_pt - rbt)
        
        // Psi
        Psi = (rUnity / minRange - rUnity / rZero) / (1.0 - rUnity / rZero);
        
        // dPsiDx
        float derivativeDenominator = pow(minRange, 3) * (rUnity - rZero);
        float derivatorNominatorTerm = rUnity * rZero;
        float PsiDerivativeXTerm = epsilonDivide(derivatorNominatorTerm * closestScanPtToRobot[0], derivativeDenominator);
        float PsiDerivativeYTerm = epsilonDivide(derivatorNominatorTerm * closestScanPtToRobot[1], derivativeDenominator);

        // float dPsiDxNorm = sqrt(pow(PsiDerivativeXTerm, 2) + pow(PsiDerivativeYTerm, 2));
        // float normPsiDerivativeXTerm = epsilonDivide(PsiDerivativeXTerm, dPsiDxNorm);
        // float normPsiDerivativeYTerm = epsilonDivide(PsiDerivativeYTerm, dPsiDxNorm);
        
        dPsiDx[0] = PsiDerivativeXTerm;
        dPsiDx[1] = PsiDerivativeYTerm;
        
        // return Eigen::Vector3f(normPsiDerivativeXTerm, normPsiDerivativeYTerm, Psi);
    }

    Eigen::Matrix2cf TrajectoryController::getComplexMatrix(const float & x, const float & y, const float & theta)
    {
        std::complex<float> phase(std::cos(theta), std::sin(theta));

        Eigen::Matrix2cf g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }

    int TrajectoryController::extractTargetPoseIdx(const geometry_msgs::Pose & currPose, const geometry_msgs::PoseArray & localTrajectory) 
    {
        // Find pose right ahead
        std::vector<float> localTrajectoryDeviations(localTrajectory.poses.size());
        ROS_INFO_STREAM_NAMED("Controller", "[extractTargetPoseIdx()]");

        // obtain distance from entire ref traj and current pose
        for (int i = 0; i < localTrajectoryDeviations.size(); i++) // i will always be positive, so this is fine
        {
            tf::Quaternion currQuatInv(currPose.orientation.x, currPose.orientation.y, currPose.orientation.z, -currPose.orientation.w); // -w for inverse
  
            tf::Quaternion desQuat(localTrajectory.poses[i].orientation.x, localTrajectory.poses[i].orientation.y, 
                                   localTrajectory.poses[i].orientation.z, localTrajectory.poses[i].orientation.w);
            
            tf::Quaternion deviationQuat = desQuat * currQuatInv;
            float deviationYaw = quaternionToYaw(deviationQuat);

            // ROS_INFO_STREAM_NAMED("Controller", "   pose" << i << ", yaw_curr: " << yaw_curr << ", yaw_des: " << yaw_des << ", deviationYaw: " << deviationYaw);

            localTrajectoryDeviations.at(i) = sqrt(pow(currPose.position.x - localTrajectory.poses[i].position.x, 2) + 
                                                pow(currPose.position.y - localTrajectory.poses[i].position.y, 2)) + 
                                                0.5 * std::abs(deviationYaw);
        }

        // find pose in ref traj with smallest difference
        auto minimumDeviationIter = std::min_element(localTrajectoryDeviations.begin(), localTrajectoryDeviations.end());
        
        // go n steps ahead of pose with smallest difference
        int targetPose = std::distance(localTrajectoryDeviations.begin(), minimumDeviationIter) + cfg_->ctrl.ctrl_ahead_pose;

        // make sure pose does note exceed trajectory size
        return std::min(targetPose, int(localTrajectory.poses.size() - 1));
    }
}