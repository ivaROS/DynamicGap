#include <dynamic_gap/trajectory_tracking/TrajectoryController.h>

namespace dynamic_gap
{
    TrajectoryController::TrajectoryController(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        projOpPublisher_ = nh.advertise<visualization_msgs::Marker>("po_dir", 10);
        cfg_ = & cfg;

        KFeedbackTheta_ = (cfg_->planning.holonomic) ? 0.8 : cfg_->control.k_fb_theta;

        manualVelX_ = 0.0f;
        manualVelY_ = 0.0f;
        manualVelAng_ = 0.0f;
        manualVelLinIncrement_ = 0.05f;
        manualVelAngIncrement_ = 0.10f;
    }

    void TrajectoryController::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan)
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
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

    geometry_msgs::Twist TrajectoryController::manualControlLaw() 
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

        float cmdVelX = cfg_->projection.k_CBF * safeDirX;
        float cmdVelY = cfg_->projection.k_CBF * safeDirY;
        float cmdVelTheta = 0.0;


        ROS_INFO_STREAM_NAMED("Controller", "raw safe vels: " << cmdVelX << ", " << cmdVelY);
        ROS_INFO_STREAM_NAMED("Controller", "weighted safe vels: " << cmdVelX << ", " << cmdVelY);

        clipRobotVelocity(cmdVelX, cmdVelY, cmdVelTheta);

        // if (sqrt(pow(cmdVelX, 2) + pow(cmdVelY, 2)) < 0.1) 
        //     cmdVelX += 0.3;

        ROS_INFO_STREAM_NAMED("Controller", "final safe vels: " << cmdVelX << ", " << cmdVelY);

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();
        cmdVel.linear.x = cmdVelX;
        cmdVel.linear.y = cmdVelY; 
        cmdVel.angular.z = cmdVelTheta;    

        return cmdVel;
    }

    geometry_msgs::Twist TrajectoryController::controlLaw(const geometry_msgs::Pose & current, 
                                                          const geometry_msgs::Pose & desired,
                                                          const geometry_msgs::TwistStamped & currentPeakSplineVel) 
    {    
        ROS_INFO_STREAM_NAMED("Controller", "    [controlLaw()]");
        // Setup Vars
        boost::mutex::scoped_lock lock(scanMutex_);

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        ROS_INFO_STREAM_NAMED("Controller", "        feedback control");
        // ROS_INFO_STREAM_NAMED("Controller", "r_unity: " << r_unity);

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = current.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        float currYaw = quaternionToYaw(currQuat); 

        // get current x,y,theta
        geometry_msgs::Point currPosn = current.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        // obtaining RPY of desired orientation
        geometry_msgs::Point desPosn = desired.position;
        geometry_msgs::Quaternion desOrient = desired.orientation;
        tf::Quaternion desQuat(desOrient.x, desOrient.y, desOrient.z, desOrient.w);

        float desYaw = quaternionToYaw(desQuat);

        // get desired x,y,theta
        Eigen::Matrix2cf desRbtTransform = getComplexMatrix(desPosn.x, desPosn.y, desYaw);

        // get x,y,theta error
        Eigen::Matrix2cf errorMat = currRbtTransform.inverse() * desRbtTransform;
        float errorX = errorMat.real()(0, 1);
        float errorY = errorMat.imag()(0, 1);
        float errorTheta = std::arg(errorMat(0, 0));

        // obtain feedback velocities
        float velLinXFeedback = errorX * cfg_->control.k_fb_x;
        float velLinYFeedback = errorY * cfg_->control.k_fb_y;
        float velAngFeedback = errorTheta * KFeedbackTheta_;

        float peakSplineSpeed = sqrt(pow(currentPeakSplineVel.twist.linear.x, 2) + pow(currentPeakSplineVel.twist.linear.y, 2));
        float cmdSpeed = sqrt(pow(velLinXFeedback, 2) + pow(velLinYFeedback, 2));

        ROS_INFO_STREAM_NAMED("Controller", "        generating control signal");            
        ROS_INFO_STREAM_NAMED("Controller", "        desired pose x: " << desPosn.x << ", y: " << desPosn.y << ", yaw: " << desYaw);
        ROS_INFO_STREAM_NAMED("Controller", "        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);
        ROS_INFO_STREAM_NAMED("Controller", "        errorX: " << errorX << ", errorY: " << errorY << ", errorTheta: " << errorTheta);
        ROS_INFO_STREAM_NAMED("Controller", "        Feedback command velocities, v_x: " << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        gap peak velocity: " << currentPeakSplineVel.twist.linear.x << ", " << currentPeakSplineVel.twist.linear.y);           
        
        if (peakSplineSpeed > cmdSpeed) 
        {
            velLinXFeedback *= 1.25 * epsilonDivide(peakSplineSpeed, cmdSpeed);
            velLinYFeedback *= 1.25 * epsilonDivide(peakSplineSpeed, cmdSpeed);
            ROS_INFO_STREAM_NAMED("Controller", "        revised feedback command velocities: " << velLinXFeedback << ", " << velLinYFeedback << ", " << velAngFeedback);
        }
    
        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;
        return cmdVel;
    }

    geometry_msgs::Twist TrajectoryController::processCmdVel(const geometry_msgs::Twist & rawCmdVel,
                                                             const geometry_msgs::PoseStamped & rbtPoseInSensorFrame, 
                                                             const dynamic_gap::Estimator * currGapLeftPtModel,
                                                             const dynamic_gap::Estimator * currGapRightPtModel, 
                                                             const geometry_msgs::TwistStamped & currRbtVel, 
                                                             const geometry_msgs::TwistStamped & currRbtAcc) 
    {
        ROS_INFO_STREAM_NAMED("Controller", "    [processCmdVel()]");

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        float velLinXFeedback = rawCmdVel.linear.x;
        float velLinYFeedback = rawCmdVel.linear.y;
        float velAngFeedback = rawCmdVel.angular.z;

        // ROS_INFO_STREAM_NAMED("Controller", rbtPoseInSensorFrame.pose);
        float minDistTheta = 0;
        float minDist = 0;

        // ROS_INFO_STREAM_NAMED("Controller", "feedback command velocities: " << cmdVelFeedback[0] << ", " << cmdVelFeedback[1]);

        // applies PO
        float velLinXSafe = 0;
        float velLinYSafe = 0;
        
        if (cfg_->planning.projection_operator && (currGapRightPtModel != nullptr && currGapLeftPtModel != nullptr))
        {
            ROS_INFO_STREAM_NAMED("Controller", "        running projection operator");
            
            float Psi = 0.0;
            Eigen::Vector2f dPsiDx(0.0, 0.0);
            Eigen::Vector2f cmdVelFeedback(rawCmdVel.linear.x, rawCmdVel.linear.y);

            runProjectionOperator(rbtPoseInSensorFrame,
                                    cmdVelFeedback, Psi, dPsiDx, velLinXSafe, velLinYSafe,
                                    minDistTheta, minDist);
            
            // float PsiCBF = 0.0;
            // Eigen::Vector4f state(rbtPoseInSensorFrame.pose.position.x, rbtPoseInSensorFrame.pose.position.y, currRbtVel.linear.x, currRbtVel.linear.y);
            // Eigen::Vector4f leftGapPtState = currGapLeftPtModel_->getState(); // flipping
            // Eigen::Vector4f rightGapPtState = currGapRightPtModel_->getState(); // flipping
            // Eigen::Vector2f current_currRbtAcc(currRbtAcc.linear.x, currRbtAcc.linear.y);
            // runBearingRateCBF(state, leftGapPtState, rightGapPtState, current_currRbtAcc, velLinXSafe, velLinYSafe, PsiCBF);
        } else 
        {
            ROS_DEBUG_STREAM_THROTTLE(10, "Projection operator off");
        }
        
        // Make sure no ejection from gap. Max question: x does not always point into gap. 
        // velLinXSafe = std::max(velLinXSafe, float(0));

        float weightedVelLinXSafe = cfg_->projection.k_CBF * velLinXSafe;
        float weightedVelLinYSafe = cfg_->projection.k_CBF * velLinYSafe;

        ROS_INFO_STREAM_NAMED("Controller", "        safe command velocity, v_x:" << weightedVelLinXSafe << ", v_y: " << weightedVelLinYSafe);

        // cmdVel_safe
        if (weightedVelLinXSafe != 0 || weightedVelLinYSafe != 0)
            visualizeProjectionOperator(weightedVelLinXSafe, weightedVelLinYSafe);

        if (cfg_->planning.holonomic)
        {

            velLinXFeedback += weightedVelLinXSafe;
            velLinYFeedback += weightedVelLinYSafe;

            // do not want robot to drive backwards
            if (velLinXFeedback < 0)
                velLinXFeedback = 0;            

        } else {
            velLinXFeedback += (cfg_->projection.k_po_x * velLinXSafe);
            velAngFeedback += (velLinYFeedback + cfg_->projection.k_po_theta * velLinYSafe);

            if (cfg_->planning.projection_operator && std::abs(minDistTheta) < M_PI / 4 && minDist < cfg_->rbt.r_inscr)
            {
                velLinXFeedback = 0;
                velAngFeedback *= 2;
            }

            velLinYFeedback = 0;

            if(velLinXFeedback < 0)
                velLinXFeedback = 0;
        }

        ROS_INFO_STREAM_NAMED("Controller", "        summed command velocity, v_x:" << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);
        clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        ROS_INFO_STREAM_NAMED("Controller", "        clipped command velocity, v_x:" << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);

        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;

        // ROS_INFO_STREAM_NAMED("Controller", "ultimate command velocity: " << cmdVel.linear.x << ", " << cmdVel.linear.y << ", " << cmdVel.angular.z);

        return cmdVel;
    }
    
    void TrajectoryController::visualizeProjectionOperator(const float & weightedVelLinXSafe, 
                                                           const float & weightedVelLinYSafe) 
    {
        visualization_msgs::Marker projOpMarker;
        projOpMarker.header.frame_id = cfg_->robot_frame_id;
        projOpMarker.type = visualization_msgs::Marker::ARROW;
        projOpMarker.action = visualization_msgs::Marker::ADD;
        projOpMarker.pose.position.x = 0;
        projOpMarker.pose.position.y = 0;
        projOpMarker.pose.position.z = 0.5;
        float dir = std::atan2(weightedVelLinYSafe, weightedVelLinXSafe);
        tf2::Quaternion projOpQuat;
        projOpQuat.setRPY(0, 0, dir);
        projOpMarker.pose.orientation = tf2::toMsg(projOpQuat);

        projOpMarker.scale.x = sqrt(pow(weightedVelLinXSafe, 2) + pow(weightedVelLinYSafe, 2));
        projOpMarker.scale.y = 0.01;  
        projOpMarker.scale.z = 0.01;
        
        projOpMarker.color.a = 1;
        projOpMarker.color.r = 0.0;
        projOpMarker.color.g = 0.0;
        projOpMarker.color.b = 0.0;
        projOpMarker.id = 0;
        projOpMarker.lifetime = ros::Duration(0.1);

        projOpPublisher_.publish(projOpMarker);
    }

    void TrajectoryController::clipRobotVelocity(float & velLinXFeedback, float & velLinYFeedback, float & velAngFeedback) 
    {
        float speedLinXFeedback = std::abs(velLinXFeedback);
        float speedLinYFeedback = std::abs(velLinYFeedback);
        
        if (speedLinXFeedback <= cfg_->control.vx_absmax && speedLinYFeedback <= cfg_->control.vy_absmax) 
        {
            // std::cout << "not clipping" << std::endl;
        } else 
        {
            velLinXFeedback *= epsilonDivide(cfg_->control.vx_absmax, std::max(speedLinXFeedback, speedLinYFeedback));
            velLinYFeedback *= epsilonDivide(cfg_->control.vy_absmax, std::max(speedLinXFeedback, speedLinYFeedback));
        }

        // std::max(-cfg_->control.vang_absmax, std::min(cfg_->control.vang_absmax, velAngFeedback));
        return;
    }

    /*
    void TrajectoryController::runBearingRateCBF(const Eigen::Vector4f & state, 
                                                 const Eigen::Vector4f & leftGapPtState,
                                                 const Eigen::Vector4f & rightGapPtState,
                                                 const Eigen::Vector2f & currRbtAcc,
                                                 float & velLinXSafe, 
                                                 float & velLinYSafe, 
                                                 float & PsiCBF) 
    {
        float hSafe = 0.0;
        Eigen::Vector4f dHSafeDx(0.0, 0.0, 0.0, 0.0);
        
        float hLeft = leftGapSideCBF(leftGapPtState);
        float hRight = rightGapSideCBF(rightGapPtState);
        Eigen::Vector4f dHLeftDx = leftGapSideCBFDerivative(leftGapPtState);
        Eigen::Vector4f dHRightDx = rightGapSideCBFDerivative(rightGapPtState);

        // need to potentially ignore if gap is non-convex
        ROS_INFO_STREAM_NAMED("Controller", "rbt velocity: " << state[2] << ", " << state[3] << ", currRbtAcc: " << currRbtAcc[0] << ", " << currRbtAcc[1]);
        ROS_INFO_STREAM_NAMED("Controller", "left rel state: " << leftGapPtState[0] << ", " << leftGapPtState[1] << ", " << leftGapPtState[2] << ", " << leftGapPtState[3]);
        ROS_INFO_STREAM_NAMED("Controller", "right rel state: " << rightGapPtState[0] << ", " << rightGapPtState[1] << ", " << rightGapPtState[2] << ", " << rightGapPtState[3]);

        Eigen::Vector2f leftBearingVect(leftGapPtState[0], leftGapPtState[1]);
        Eigen::Vector2f rightBearingVect(rightGapPtState[0], rightGapPtState[1]);

        Eigen::Vector2f leftBearingNorm = unitNorm(leftBearingVect);
        Eigen::Vector2f rightBearingNorm = unitNorm(rightBearingVect);

        float determinant = rightBearingNorm[0]*leftBearingNorm[1] - rightBearingNorm[1]*leftBearingNorm[0];      
        float dot = rightBearingNorm[0]*leftBearingNorm[0] + rightBearingNorm[1]*leftBearingNorm[1];

        // float swept_check = ;     
        float leftToRightAngle = -std::atan2(determinant, dot);

        if (leftToRightAngle < 0)
            leftToRightAngle += 2*M_PI; 

        ROS_INFO_STREAM_NAMED("Controller", "L_to_R angle: " << leftToRightAngle);
        ROS_INFO_STREAM_NAMED("Controller", "left CBF: " << hLeft);
        ROS_INFO_STREAM_NAMED("Controller", "left CBF partials: " << dHLeftDx[0] << ", " << dHLeftDx[1] << ", " << dHLeftDx[2] << ", " << dHLeftDx[3]);

        ROS_INFO_STREAM_NAMED("Controller", "right CBF: " << hRight);
        ROS_INFO_STREAM_NAMED("Controller", "right CBF partials: " << dHRightDx[0] << ", " << dHRightDx[1] << ", " << dHRightDx[2] << ", " << dHRightDx[3]);

        float cbfParam = 1.0;
        bool cvxGap = leftToRightAngle < M_PI;
        Eigen::Vector4f dxdt(state[2], state[3], currRbtAcc[0], currRbtAcc[1]);
        float PsileftGapSideCBF = dHLeftDx.dot(dxdt) + cbfParam * hLeft;
        float PsirightGapSideCBF = dHRightDx.dot(dxdt) + cbfParam * hRight;
        
        ROS_INFO_STREAM_NAMED("Controller", "PsileftGapSideCBF: " << PsileftGapSideCBF << ", PsirightGapSideCBF: " << PsirightGapSideCBF);
        float velLinXSafeLeft = 0;
        float velLinYSafeLeft = 0;
        float velLinXSafeRight = 0;
        float velLinYSafeRight = 0;

        if (cvxGap && PsileftGapSideCBF < 0)  // right less than left
        {
            Eigen::Vector2f Lg_h_left(dHLeftDx[0], dHLeftDx[1]); // Lie derivative of h wrt x (we are doing command velocities)
            Eigen::Vector2f cmdVelSafeLeft = epsilonDivide(-(Lg_h_left * PsileftGapSideCBF), Lg_h_left.dot(Lg_h_left));
            velLinXSafeLeft = cmdVelSafeLeft[0];
            velLinYSafeLeft = cmdVelSafeLeft[1];    
        }
        
        if (cvxGap && PsirightGapSideCBF < 0) // left less than or equal to right
        { 
            Eigen::Vector2f Lg_h_right(dHRightDx[0], dHRightDx[1]); // Lie derivative of h wrt x (we are doing command velocities)
            Eigen::Vector2f cmdVelSafeRight = epsilonDivide(-(Lg_h_right * PsirightGapSideCBF), Lg_h_right.dot(Lg_h_right));
            velLinXSafeRight = cmdVelSafeRight[0];
            velLinYSafeRight = cmdVelSafeRight[1];      
        }
        
        velLinXSafe = velLinXSafeLeft + velLinXSafeRight;
        velLinYSafe = velLinYSafeLeft + velLinYSafeRight;
        
        if (velLinXSafe != 0 || velLinYSafe != 0) 
        {
            ROS_INFO_STREAM_NAMED("Controller", "cmdVel_safe left: " << velLinXSafeLeft << ", " << velLinXSafeLeft);
            ROS_INFO_STREAM_NAMED("Controller", "cmdVel_safe right: " << velLinXSafeRight << ", " << velLinYSafeRight);
            ROS_INFO_STREAM_NAMED("Controller", "cmdVel_safe x: " << velLinXSafe << ", cmdVel_safe y: " << velLinYSafe);
        }
    }

    float TrajectoryController::leftGapSideCBF(const Eigen::Vector4f & leftGapPtState) 
    {
        // current design: h_right = -betadot
        float r = sqrt(pow(leftGapPtState(0), 2) + pow(leftGapPtState(1), 2));
        float betadot = (leftGapPtState(0)*leftGapPtState(3) - leftGapPtState(1)*leftGapPtState(2))/pow(r,2);
        
        float hRight = -betadot;
        
        return hRight;
    }

    Eigen::Vector4f TrajectoryController::leftGapSideCBFDerivative(const Eigen::Vector4f & leftGapPtState) 
    {
        // current design: h_right = -betadot
        Eigen::Vector4f dHLeftDX(0.0, 0.0, 0.0, 0.0);
        
        float r = sqrt(pow(leftGapPtState(0), 2) + pow(leftGapPtState(1), 2));
        float rCrossV = leftGapPtState(0)*leftGapPtState(3) - leftGapPtState(1)*leftGapPtState(2);
        float rSq = pow(r, 2);
        float rSqSq = pow(r, 4);
        // derivative with respect to r_ox, r_oy, v_ox, v_oy
        
        dHLeftDX(0) =  epsilonDivide(leftGapPtState(3), rSq) - epsilonDivide(2*leftGapPtState(0)*rCrossV, rSqSq);
        dHLeftDX(1) = epsilonDivide(-leftGapPtState(2), rSq) - epsilonDivide(2*leftGapPtState(1)*rCrossV, rSqSq);
        dHLeftDX(2) = epsilonDivide(-leftGapPtState(1), rSq);
        dHLeftDX(3) =  epsilonDivide(leftGapPtState(0), rSq);

        return dHLeftDX;
    }

    float TrajectoryController::rightGapSideCBF(const Eigen::Vector4f & rightGapPtState) 
    {
        // current design: h_left = betadot
        float r = sqrt(pow(rightGapPtState(0), 2) + pow(rightGapPtState(1), 2));
        float betadot = (rightGapPtState(0)*rightGapPtState(3) - rightGapPtState(1)*rightGapPtState(2))/pow(r, 2);

        float hLeft = betadot;

        return hLeft;
    }

    Eigen::Vector4f TrajectoryController::rightGapSideCBFDerivative(const Eigen::Vector4f & rightGapPtState) 
    {
        // current design: h_left = betadot
        Eigen::Vector4f dHRightDX(0.0, 0.0, 0.0, 0.0);
        float r = sqrt(pow(rightGapPtState(0), 2) + pow(rightGapPtState(1), 2));
        float rCrossV = rightGapPtState(0)*rightGapPtState(3) - rightGapPtState(1)*rightGapPtState(2);
        float rSq = pow(r, 2);
        float rSqSq = pow(r, 4);

        // derivative with respect to r_ox, r_oy, v_ox, v_oy
        dHRightDX(0) = epsilonDivide(-rightGapPtState(3), rSq) + epsilonDivide(2*rightGapPtState(0)*rCrossV, rSqSq);
        dHRightDX(1) =  epsilonDivide(rightGapPtState(2), rSq) + epsilonDivide(2*rightGapPtState(1)*rCrossV, rSqSq);
        dHRightDX(2) =  epsilonDivide(rightGapPtState(1), rSq);
        dHRightDX(3) = epsilonDivide(-rightGapPtState(0), rSq);
        return dHRightDX;
    }
    */

    void TrajectoryController::runProjectionOperator(const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                                     Eigen::Vector2f & cmdVelFeedback,
                                                     float & Psi, Eigen::Vector2f & dPsiDx,
                                                     float & velLinXSafe, float & velLinYSafe,
                                                     float & minDistTheta, float & minDist) 
{
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
        int minDistScanIdx = std::min_element(minScanDists.begin(), minScanDists.end()) - minScanDists.begin();
        minDistTheta = idx2theta(minDistScanIdx);

        minDist = minScanDists.at(minDistScanIdx);

        ROS_INFO_STREAM_NAMED("Controller", "minDistScanIdx: " << minDistScanIdx << ", minDistTheta: "<< minDistTheta << ", minDist: " << minDist);
        // ROS_INFO_STREAM_NAMED("Controller", "min_x: " << min_x << ", min_y: " << min_y);
              
        Eigen::Vector2f closestScanPtToRobot(-minDist * std::cos(minDistTheta), -minDist * std::sin(minDistTheta));

        Eigen::Vector3f PsiDerAndPsi = calculateProjectionOperator(closestScanPtToRobot); // return Psi, and dPsiDx
        dPsiDx = Eigen::Vector2f(PsiDerAndPsi(0), PsiDerAndPsi(1));

        float projOpDotProd = cmdVelFeedback.dot(dPsiDx);

        Psi = PsiDerAndPsi(2);

        ROS_INFO_STREAM_NAMED("Controller", "dPsiDx: " << dPsiDx[0] << ", " << dPsiDx[1]);
        ROS_INFO_STREAM_NAMED("Controller", "Psi: " << Psi << ", dot product check: " << projOpDotProd);

        if (Psi >= 0 && projOpDotProd >= 0)
        {
            velLinXSafe = - Psi * projOpDotProd * PsiDerAndPsi(0);
            velLinYSafe = - Psi * projOpDotProd * PsiDerAndPsi(1);
            ROS_INFO_STREAM_NAMED("Controller", "cmdVel_safe: " << velLinXSafe << ", " << velLinYSafe);
        }

        /*
        // Min Direction
        res.header.frame_id = cfg_->sensor_frame_id;
        res.scale.x = 1;
        projOpQuat.setRPY(0, 0, minDistTheta);
        res.pose.orientation = tf2::toMsg(projOpQuat);
        res.id = 1;
        res.color.a = 0.5;
        res.pose.position.z = 0.9;
        projOpPublisher_.publish(res);
        */
        /*
        res.header.frame_id = cfg_->robot_frame_id;
        res.type = visualization_msgs::Marker::SPHERE;
        res.action = visualization_msgs::Marker::ADD;
        res.pose.position.x = -min_diff_x;
        res.pose.position.y = -min_diff_y;
        res.pose.position.z = 1;
        res.scale.x = 0.1;
        res.scale.y = 0.1;
        res.scale.z = 0.1;
        res.id = 2;
        // res.color.a = 0.5;
        // res.pose.position.z = 0.9;
        projOpPublisher_.publish(res);
        */
    }

    Eigen::Vector3f TrajectoryController::calculateProjectionOperator(const Eigen::Vector2f & closestScanPtToRobot) 
    {
        float rUnity = cfg_->projection.r_unity;
        float rZero = cfg_->projection.r_zero;

        float minDist = closestScanPtToRobot.norm(); // sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2)); // (closest_pt - rbt)
        float Psi = (rUnity / minDist - rUnity / rZero) / (1.0 - rUnity / rZero);
        float derivativeDenominator = pow(minDist, 3) * (rUnity - rZero);
        float derivatorNominatorTerm = rUnity * rZero;
        float PsiDerivativeXTerm = epsilonDivide(derivatorNominatorTerm * closestScanPtToRobot[0], derivativeDenominator);
        float PsiDerivativeYTerm = epsilonDivide(derivatorNominatorTerm * closestScanPtToRobot[1], derivativeDenominator);

        float dPsiDxNorm = sqrt(pow(PsiDerivativeXTerm, 2) + pow(PsiDerivativeYTerm, 2));
        float normPsiDerivativeXTerm = epsilonDivide(PsiDerivativeXTerm, dPsiDxNorm);
        float normPsiDerivativeYTerm = epsilonDivide(PsiDerivativeYTerm, dPsiDxNorm);
        return Eigen::Vector3f(normPsiDerivativeXTerm, normPsiDerivativeYTerm, Psi);
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