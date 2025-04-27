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

        // Just storing desired displacement for now
        cmdVel.linear.x = errorX;
        cmdVel.linear.y = errorY;
        cmdVel.angular.z = errorTheta;

        return cmdVel; 
    }

    geometry_msgs::Twist TrajectoryController::constantVelocityControlLawNonHolonomicNID(const geometry_msgs::Pose & currentPoseOdomFrame, 
                                                                                            const geometry_msgs::Pose & desiredPoseOdomFrame,
                                                                                            const float & desiredSpeed)
    {
        ROS_INFO_STREAM_NAMED("Controller", "    [constantVelocityControlLawNonHolonomicNID()]");
        // Setup Vars
        // boost::mutex::scoped_lock lock(scanMutex_);

        /*
            THIS IS CURRENTLY NOT WORKING. NEEDS TO BE FIXED.
        */


        ros::Time desiredTime_ = ros::Time::now();
        float lambdaDot = - clambda_ * (lambda_ - epsilon_);

        if (currentPoseOdomFrame == prevDesired_)
        {
            ROS_INFO_STREAM_NAMED("Controller", "    [constantVelocityControlLawNonHolonomicNID()] no change in desired pose");
            lambda_ += lambdaDot * (desiredTime_ - prevDesiredTime_).toSec();
        } else
        {
            ROS_INFO_STREAM_NAMED("Controller", "    [constantVelocityControlLawNonHolonomicNID()] new desired pose");
            lambda_ = lambdaInit_;
        }

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

        ROS_INFO_STREAM_NAMED("Controller", "        desired pose x: " << currentPoseOdomFrame.position.x << ", y: " << currentPoseOdomFrame.position.y << ", yaw: "<< desYaw);

        // get desired x,y,theta
        Eigen::Matrix2cf desRbtTransform = getComplexMatrix(desPosn.x, desPosn.y, desYaw);

        // get x,y,theta error
        Eigen::Matrix2cf errorMat = currRbtTransform.inverse() * desRbtTransform;
        float errorX = desPosn.x - currPosn.x; // errorMat.real()(0, 1);
        float errorY = desPosn.y - currPosn.y; // errorMat.imag()(0, 1);
        float errorTheta = std::arg(errorMat(0, 0));

        Eigen::Vector2f error(errorX, errorY);
        Eigen::Vector2f errorDir = epsilonDivide(error, error.norm());

        Eigen::Matrix2f lambdaRotMatInv;
        lambdaRotMatInv(0, 0) = std::cos(lambda_);
        lambdaRotMatInv(0, 1) = std::sin(lambda_);
        lambdaRotMatInv(1, 0) = -std::sin(lambda_) / lambda_;
        lambdaRotMatInv(1, 1) = std::cos(lambda_) / lambda_;

        Eigen::Vector2f x(currPosn.x, currPosn.y);
        Eigen::Vector2f rotMate1(std::cos(currYaw), std::sin(currYaw));
        Eigen::Vector2f q = x + lambda_ * rotMate1;

        Eigen::Vector2f xDesired(desPosn.x, desPosn.y);

        Eigen::Vector2f v = -cp_ * lambdaRotMatInv * (q - xDesired) - lambdaDot * Eigen::Vector2f(1.0, 0.0);     

        float velLinXFeedback = v[0]; // nonholoVelocityCommand[0];
        float velAngFeedback = v[1]; // nonholoVelocityCommand[1];

        ROS_INFO_STREAM_NAMED("Controller", "        generating nonholonomic control signal");            
        ROS_INFO_STREAM_NAMED("Controller", "        Feedback command velocities, v_x: " << velLinXFeedback << ", v_ang: " << velAngFeedback);
        
        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = velAngFeedback;

        prevDesired_ = currentPoseOdomFrame;
        prevDesiredTime_ = desiredTime_;

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
        float velAngFeedback = 0.0; // cfg_->planning.heading * errorTheta * cfg_->control.k_fb_theta;

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
            
            // float Psi = 0.0;
            // Eigen::Vector2f dPsiDx(0.0, 0.0);
            Eigen::Vector2f cmdVelFeedback(rawCmdVel.linear.x, rawCmdVel.linear.y);

            runProjectionOperator(rbtPoseInSensorFrame,
                                    cmdVelFeedback, velLinXSafe, velLinYSafe,
                                    minRangeTheta, minRange); // Psi, dPsiDx, 
            
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

    // const geometry_msgs::TwistStamped & currRbtVel, 
    // const geometry_msgs::TwistStamped & currRbtAcc    
    geometry_msgs::Twist TrajectoryController::processCmdVelNonHolonomic(const geometry_msgs::Pose & currentPoseOdomFrame,
                                                                            const geometry_msgs::Twist & rawCmdVel,
                                                                            const geometry_msgs::PoseStamped & rbtPoseInSensorFrame) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "    [processCmdVelNonHolonomic()]");

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = currentPoseOdomFrame.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        float errorX = rawCmdVel.linear.x;
        float errorY = rawCmdVel.linear.y;
        float errorTheta = rawCmdVel.angular.z;

        // ROS_INFO_STREAM_NAMED("Controller", rbtPoseInSensorFrame.pose);
        float minRangeTheta = 0;
        float minRange = 0;

        ROS_INFO_STREAM_NAMED("Controller", "        feedback errors: x: " << errorX << ", y: " << errorY << ", theta: " << errorTheta);

        // applies PO
        float velLinXSafe = 0.;
        float velLinYSafe = 0.;
        
        if (cfg_->planning.projection_operator)
        {
            ROS_INFO_STREAM_NAMED("Controller", "        running projection operator");
            
            // float Psi = 0.0;
            // Eigen::Vector2f dPsiDx(0.0, 0.0);
            Eigen::Vector2f cmdVelFeedback(errorX, errorY);
            // Psi, dPsiDx, 

            runProjectionOperator(rbtPoseInSensorFrame,
                                    cmdVelFeedback, velLinXSafe, velLinYSafe,
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

        float safeErrorX = errorX + weightedVelLinXSafe;
        float safeErrorY = errorY + weightedVelLinYSafe; 

        ROS_INFO_STREAM_NAMED("Controller", "        summed desired direction, v_x:" << safeErrorX << ", v_y: " << safeErrorY);

        Eigen::Vector2f safeErrorDirection(safeErrorX, safeErrorY);
        float l = safeErrorDirection.norm();

        float l_adj = l; // 0.5 * l;

        ROS_INFO_STREAM_NAMED("Controller", "        safeErrorDirection: " << safeErrorDirection.transpose() << ", l: " << l << ", l_adj: " << l);

        Eigen::Matrix2f negRotMat = getRotMat(-currYaw);

        Eigen::Matrix2f nidMat = Eigen::Matrix2f::Identity();
        nidMat(1, 1) = (1.0 / l_adj);

        Eigen::Vector2f nonholoVelocityCommand = nidMat * negRotMat * safeErrorDirection;

        float velLinXFeedback = nonholoVelocityCommand[0];
        float velLinYFeedback = 0.0;
        float velAngFeedback = nonholoVelocityCommand[1];

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

        cmdVel.linear.x = clippedVelLinXFeedback;
        cmdVel.linear.y = 0.0;
        cmdVel.angular.z = std::max(-cfg_->rbt.vang_absmax, std::min(cfg_->rbt.vang_absmax, velAngFeedback));

        // clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        clipped nonholonomic command velocity, v_x:" << cmdVel.linear.x << ", v_ang: " << cmdVel.angular.z);



        // ROS_INFO_STREAM_NAMED("Controller", "ultimate command velocity: " << cmdVel.linear.x << ", " << cmdVel.linear.y << ", " << cmdVel.angular.z);

        return cmdVel;
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
                                                    //  float & Psi, Eigen::Vector2f & dPsiDx,
                                                     float & velLinXSafe, float & velLinYSafe,
                                                     float & minRangeTheta, float & minRange) 
{
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

        ROS_INFO_STREAM_NAMED("Controller", "minDistScanIdx: " << minDistScanIdx << ", minRangeTheta: "<< minRangeTheta << ", minRange: " << minRange);
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

        ROS_INFO_STREAM_NAMED("Controller", "Psi: " << Psi);
        ROS_INFO_STREAM_NAMED("Controller", "dPsiDx: " << dPsiDx[0] << ", " << dPsiDx[1]);
        ROS_INFO_STREAM_NAMED("Controller", "Dot product check: " << projOpDotProd);


        if (Psi >= 0 && projOpDotProd >= 0)
        {
            velLinXSafe = - Psi * projOpDotProd * normDPsiDx(0);
            velLinYSafe = - Psi * projOpDotProd * normDPsiDx(1);
            ROS_INFO_STREAM_NAMED("Controller", "cmdVel_safe: " << velLinXSafe << ", " << velLinYSafe);
        }
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
        int targetPose = std::distance(localTrajectoryDeviations.begin(), minimumDeviationIter) + cfg_->control.ctrl_ahead_pose;

        // make sure pose does note exceed trajectory size
        return std::min(targetPose, int(localTrajectory.poses.size() - 1));
    }
}