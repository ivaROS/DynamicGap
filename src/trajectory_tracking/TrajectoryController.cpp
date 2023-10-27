#include <dynamic_gap/trajectory_tracking/TrajectoryController.h>

namespace dynamic_gap
{
    TrajectoryController::TrajectoryController(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        projOpPublisher_ = nh.advertise<visualization_msgs::Marker>("po_dir", 10);
        cfg_ = & cfg;
        distanceThresh_ = 0.1;

        // holonomic = cfg_->planning.holonomic;
        // full_fov = ;
        // projection_operator = cfg_->planning.projection_operator;

        // k_fb_x_ = cfg_->control.k_fb_x;
        // k_fb_y_ = cfg_->control.k_fb_y;
        k_fb_theta_ = (cfg_->planning.holonomic && cfg_->planning.full_fov) ? 0.8 : cfg_->control.k_fb_theta;

        // k_po_x_ = cfg_->projection.k_po_x;
        // k_CBF_ = cfg_->projection.k_CBF;
        // k_po_theta_ = cfg_->projection.k_po_theta;

        // cmd_counter_ = 0;
        manualVelX_ = 0.0f;
        manualVelY_ = 0.0f;
        manualVelAng_ = 0.0f;
        manualVelLinIncrement_ = 0.05f;
        manualVelAngIncrement_ = 0.10f;
    }

    void TrajectoryController::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan)
    {
        boost::mutex::scoped_lock lock(egocircleLock_);
        scan_ = scan;
    }

    std::vector<geometry_msgs::Point> TrajectoryController::findLocalLine(int minDistScanIdx) 
    {
        // get egocircle measurement
        auto egocircle = *scan_.get();
        std::vector<float> interScanPtDists(egocircle.ranges.size());

        // if (!msg_) {
        //     return std::vector<geometry_msgs::Point>(0);
        // }
        
        // if (egocircle.ranges.size() < 500) {
        //     ROS_FATAL_STREAM("Scan range incorrect findLocalLine");
        // }

        // iterating through egocircle
        float range_i, theta_i, range_imin1, theta_imin1;
        for (int i = 1; i < interScanPtDists.size(); i++) 
        {
            // current distance/idx
            range_i = egocircle.ranges.at(i);
            theta_i = idx2theta(i); // float(i) * egocircle.angle_increment + egocircle.angle_min;
            // prior distance/idx
            range_imin1 = egocircle.ranges.at(i - 1);
            theta_imin1 = idx2theta(i - 1); // float(i - 1) * egocircle.angle_increment + egocircle.angle_min;
            
            
            if (range_i > cfg_->scan.range_max)  // if current distance is big, set dist to big
                interScanPtDists.at(i) = 10;
            else                                 // else, get distance between two indices
                interScanPtDists.at(i) = polDist(range_i, theta_i, range_imin1, theta_imin1);
        }

        interScanPtDists.at(0) = polDist(egocircle.ranges.at(0), egocircle.angle_min, 
                                         egocircle.ranges.at(egocircle.ranges.size() - 1), idx2theta(egocircle.ranges.size() - 1));

        // searching forward for the first place where interpoint distances exceed threshold (0.1) (starting from the minDistScanIdx).
        auto fwdIter = std::find_if(interScanPtDists.begin() + minDistScanIdx, interScanPtDists.end(), 
                                        std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));

        // searching backward for first place where interpoint distances exceed threshold (0.1)
        auto revIter = std::find_if(interScanPtDists.rbegin() + (interScanPtDists.size() - minDistScanIdx), interScanPtDists.rend(),
                                        std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));
        
        if (revIter == interScanPtDists.rend())
            return std::vector<geometry_msgs::Point>(0);

        // get index for big enough distance
        int fwdIdx = std::distance(interScanPtDists.begin(), std::prev(fwdIter));
        int revIdx = std::distance(revIter, interScanPtDists.rend());

        // are indices valid
        // int minDistScanIdx_range = 0;
        // int max_idx_range = int(egocircle.ranges.size() - 1);
        // if (fwdIdx < minDistScanIdx_range || fwdIdx > max_idx_range || revIdx < minDistScanIdx_range || revIdx > max_idx_range) {
        //     return std::vector<geometry_msgs::Point>(0);
        // }
        
        float fwdRange = egocircle.ranges.at(fwdIdx);
        float revRange = egocircle.ranges.at(revIdx);
        float centRange = egocircle.ranges.at(minDistScanIdx);

        float fwdTheta = idx2theta(fwdIdx); // float(fwdIdx) * egocircle.angle_increment + egocircle.angle_min;
        float revTheta = idx2theta(revIdx); // float(revIdx) * egocircle.angle_increment + egocircle.angle_min;
        float centTheta = idx2theta(minDistScanIdx);

        if (fwdIdx < minDistScanIdx || revIdx > minDistScanIdx)
            return std::vector<geometry_msgs::Point>(0);

        // Eigen::Vector2f fwd_polar(fwdRange, fwdTheta);
        // Eigen::Vector2f rev_polar(revRange, revTheta);
        // Eigen::Vector2f cent_polar(centRange, centTheta); // float(minDistScanIdx) * egocircle.angle_increment + egocircle.angle_min);
        Eigen::Vector2f fwdPtCart(fwdRange * std::cos(fwdTheta), fwdRange * std::sin(fwdTheta)); // pol2car(fwd_polar);
        Eigen::Vector2f revPtCart(revRange * std::cos(revTheta), revRange * std::sin(revTheta)); // pol2car(rev_polar);
        Eigen::Vector2f centPtCart(centRange * std::cos(centTheta), centRange * std::sin(centTheta)); // pol2car(cent_polar);

        Eigen::Vector2f projFwdPt;
        Eigen::Vector2f projRevPt;

        if (centRange < fwdRange && centRange < revRange) 
        {
            // ROS_INFO_STREAM("Non line");
            Eigen::Vector2f centMinFwdPt = centPtCart - fwdPtCart;
            Eigen::Vector2f revMinFwdPt = revPtCart - fwdPtCart;
            Eigen::Vector2f revMinFwdNorm = revMinFwdPt / revMinFwdPt.norm(); 
            Eigen::Vector2f projection = (centMinFwdPt.dot(revMinFwdNorm)) * revMinFwdNorm;
            Eigen::Vector2f orthogonal = centMinFwdPt - projection;
            projFwdPt = fwdPtCart + orthogonal;
            projRevPt = revPtCart + orthogonal;
        } else 
        {
            projFwdPt = fwdPtCart;
            projRevPt = revPtCart;
        }

        geometry_msgs::Point lowerPoint;
        lowerPoint.x = projFwdPt(0);
        lowerPoint.y = projFwdPt(1);
        // lowerPoint.z = 3;
        geometry_msgs::Point upperPoint;
        upperPoint.x = projRevPt(0);
        upperPoint.y = projRevPt(1);
        // upperPoint.z = 3;
        // if form convex hull
        std::vector<geometry_msgs::Point> retArr;
        retArr.push_back(lowerPoint);
        retArr.push_back(upperPoint);
        return retArr;
    }

    bool TrajectoryController::leqThres(const float dist) 
    {
        return dist <= distanceThresh_;
    }

    bool TrajectoryController::geqThres(const float dist) 
    {
        return dist >= distanceThresh_;
    }

    float TrajectoryController::polDist(float range1, float theta1, float range2, float theta2) 
    {
        return abs(pow(range1, 2) + pow(range2, 2) - 2 * range1 * range2 * std::cos(theta1 - theta2));
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
        ROS_INFO_STREAM("Manual Control");

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        char key = getch();
        // ROS_INFO_STREAM("Keyboard input: " << key);

        if (key == 'w')
            manualVelX_ += manualVelLinIncrement_;
        else if (key == 'a')
            manualVelY_ += manualVelLinIncrement_;
        else if (key == 's')
            manualVelX_ = 0.0f;
        else if (key == 'd')
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
    geometry_msgs::Twist TrajectoryController::obstacleAvoidanceControlLaw(const sensor_msgs::LaserScan & scan) 
    {
        ROS_INFO_STREAM("obstacle avoidance control");
        float safeDirX = 0;
        float safeDirY = 0;                                   
        
        float scanRange, scanTheta;
        for (int i = 0; i < scan.ranges.size(); i++) 
        {
            scanRange = scan.ranges[i];
            scanTheta =  idx2theta(i); // scan.angle_min + i * scan.angle_increment;

            safeDirX += (-1.0 * std::cos(scanTheta)) / pow(scanRange, 2);
            safeDirY += (-1.0 * std::sin(scanTheta)) / pow(scanRange, 2);
        }

        safeDirX /= scan.ranges.size();
        safeDirY /= scan.ranges.size();

        float cmdVelX = cfg_->projection.k_CBF * safeDirX;
        float cmdVelY = cfg_->projection.k_CBF * safeDirY;
        float cmdVelTheta = 0.0;

        if (cfg_->debug.control_debug_log) 
        {
            ROS_INFO_STREAM("raw safe vels: " << cmdVelX << ", " << cmdVelY);
            ROS_INFO_STREAM("weighted safe vels: " << cmdVelX << ", " << cmdVelY);
        }
        clipRobotVelocity(cmdVelX, cmdVelY, cmdVelTheta);


        // float cmdVel_safe_norm = ;
        if (sqrt(pow(cmdVelX, 2) + pow(cmdVelY, 2)) < 0.1) 
            cmdVelX += 0.3;

        if (cfg_->debug.control_debug_log) 
        {
            ROS_INFO_STREAM("final safe vels: " << cmdVelX << ", " << cmdVelY);
        }

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();
        cmdVel.linear.x = cmdVelX;
        cmdVel.linear.y = cmdVelY; 
        cmdVel.angular.z = cmdVelTheta;    

        return cmdVel;
    }

    geometry_msgs::Twist TrajectoryController::controlLaw(const geometry_msgs::Pose & current, 
                                                          const geometry_msgs::Pose & desired,
                                                          const sensor_msgs::LaserScan & scan, 
                                                          const geometry_msgs::TwistStamped & currentPeakSplineVel_) 
    {    
        if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("    [controlLaw()]");
        // Setup Vars
        boost::mutex::scoped_lock lock(egocircleLock_);

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();
        // float v_lin_x_fb = 0;
        // float velLinYFeedback = 0;
        // float velAngFeedback = 0;

        if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("        feedback control");
        // ROS_INFO_STREAM("r_min: " << r_min);

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        geometry_msgs::Quaternion currOrient = current.orientation;
        tf::Quaternion currQuat(currOrient.x, currOrient.y, currOrient.z, currOrient.w);
        // tf::Matrix3x3 m_c(q_c);
        // float c_roll, c_pitch, c_yaw;
        // m_c.getRPY(c_roll, c_pitch, c_yaw);

        float currYaw = std::atan2( 2.0 * (currQuat.w() * currQuat.z() + currQuat.x() * currQuat.y()), 
                              1 - 2.0 * (currQuat.y() * currQuat.y() + currQuat.z() * currQuat.z()));

        // get current x,y,theta
        geometry_msgs::Point currPosn = current.position;
        Eigen::Matrix2cf currRbtTransform = getComplexMatrix(currPosn.x, currPosn.y, currYaw);

        // obtaining RPY of desired orientation
        geometry_msgs::Point desPosn = desired.position;
        geometry_msgs::Quaternion desOrient = desired.orientation;
        tf::Quaternion desQuat(desOrient.x, desOrient.y, desOrient.z, desOrient.w);
        // tf::Matrix3x3 m_d(desQuat);
        // float d_roll, d_pitch, desYaw;
        // m_d.getRPY(d_roll, d_pitch, desYaw);

        float desYaw = std::atan2( 2.0 * (desQuat.w() * desQuat.z() + desQuat.x() * desQuat.y()), 
                              1 - 2.0 * (desQuat.y() * desQuat.y() + desQuat.z() * desQuat.z()));

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
        float velAngFeedback = errorTheta * k_fb_theta_;

        float peakSplineSpeed = sqrt(pow(currentPeakSplineVel_.twist.linear.x, 2) + pow(currentPeakSplineVel_.twist.linear.y, 2));
        float cmdSpeed = sqrt(pow(velLinXFeedback, 2) + pow(velLinYFeedback, 2));

        if (cfg_->debug.control_debug_log) 
        {
            ROS_INFO_STREAM("        generating control signal");            
            ROS_INFO_STREAM("        desired pose x: " << desPosn.x << ", y: " << desPosn.y << ", yaw: " << desYaw);
            ROS_INFO_STREAM("        current pose x: " << currPosn.x << ", y: " << currPosn.y << ", yaw: " << currYaw);
            ROS_INFO_STREAM("        errorX: " << errorX << ", errorY: " << errorY << ", errorTheta: " << errorTheta);
            ROS_INFO_STREAM("        Feedback command velocities, v_x: " << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);
            ROS_INFO_STREAM("        gap peak velocity: " << currentPeakSplineVel_.twist.linear.x << ", " << currentPeakSplineVel_.twist.linear.y);           
        }
        
        if (peakSplineSpeed > cmdSpeed) 
        {
            velLinXFeedback *= 1.25 * (peakSplineSpeed / cmdSpeed);
            velLinYFeedback *= 1.25 * (peakSplineSpeed / cmdSpeed);
            if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("        revised feedback command velocities: " << velLinXFeedback << ", " << velLinYFeedback << ", " << velAngFeedback);
        }
    
        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;
        return cmdVel;
    }

    geometry_msgs::Twist TrajectoryController::processCmdVel(const geometry_msgs::Twist & rawCmdVel,
                                                             const sensor_msgs::LaserScan & scan, 
                                                             const geometry_msgs::PoseStamped & rbtPoseInSensorFrame, 
                                                             const dynamic_gap::Estimator * currGapRightPtModel, 
                                                             const dynamic_gap::Estimator * currGapLeftPtModel,
                                                             const geometry_msgs::TwistStamped & currRbtVel, 
                                                             const geometry_msgs::TwistStamped & currRbtAcc) 
    {
        if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("    [processCmdVel()]");

        geometry_msgs::Twist cmdVel = geometry_msgs::Twist();

        float velLinXFeedback = rawCmdVel.linear.x;
        float velLinYFeedback = rawCmdVel.linear.y;
        float velAngFeedback = rawCmdVel.angular.z;

        // ROS_INFO_STREAM(rbtPoseInSensorFrame.pose);
        float minDistTheta = 0;
        float minDist = 0;

        // ROS_INFO_STREAM("feedback command velocities: " << cmdVelFeedback[0] << ", " << cmdVelFeedback[1]);

        // if (scan.ranges.size() < 500) {
        //     ROS_FATAL_STREAM("Scan range incorrect controlLaw");
        // }

        // applies PO
        float velLinXSafe = 0;
        float velLinYSafe = 0;
        
        if (cfg_->planning.projection_operator && (currGapRightPtModel != nullptr && currGapLeftPtModel != nullptr))
        {
            if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("        running projection operator");
            
            float Psi = 0.0;
            Eigen::Vector2f dPsiDx(0.0, 0.0);
            Eigen::Vector2f cmdVelFeedback(rawCmdVel.linear.x, rawCmdVel.linear.y);

            runProjectionOperator(scan, rbtPoseInSensorFrame,
                                    cmdVelFeedback, Psi, dPsiDx, velLinXSafe, velLinYSafe,
                                    minDistTheta, minDist);
            
            // float PsiCBF = 0.0;
            // Eigen::Vector4f state(rbtPoseInSensorFrame.pose.position.x, rbtPoseInSensorFrame.pose.position.y, currRbtVel.linear.x, currRbtVel.linear.y);
            // Eigen::Vector4f leftGapPtState = currGapLeftPtModel_->getState(); // flipping
            // Eigen::Vector4f rightGapPtState = currGapRightPtModel_->getState(); // flipping
            // Eigen::Vector2f current_currRbtAcc(currRbtAcc.linear.x, currRbtAcc.linear.y);
            // runBearingRateCBF(state, rightGapPtState, leftGapPtState, current_currRbtAcc, velLinXSafe, velLinYSafe, PsiCBF);
        } else 
        {
            ROS_DEBUG_STREAM_THROTTLE(10, "Projection operator off");
        }
        
        // Make sure no ejection from gap. Max question: x does not always point into gap. 
        // velLinXSafe = std::max(velLinXSafe, float(0));

        float weightedVelLinXSafe = cfg_->projection.k_CBF * velLinXSafe;
        float weightedVelLinYSafe = cfg_->projection.k_CBF * velLinYSafe;

        if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("        safe command velocity, v_x:" << weightedVelLinXSafe << ", v_y: " << weightedVelLinYSafe);

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

        if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("        summed command velocity, v_x:" << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);
        clipRobotVelocity(velLinXFeedback, velLinYFeedback, velAngFeedback);
        if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("        clipped command velocity, v_x:" << velLinXFeedback << ", v_y: " << velLinYFeedback << ", v_ang: " << velAngFeedback);

        cmdVel.linear.x = velLinXFeedback;
        cmdVel.linear.y = velLinYFeedback;
        cmdVel.angular.z = velAngFeedback;

        // ROS_INFO_STREAM("ultimate command velocity: " << cmdVel.linear.x << ", " << cmdVel.linear.y << ", " << cmdVel.angular.z);

        return cmdVel;
    }
    
    void TrajectoryController::visualizeProjectionOperator(float weightedVelLinXSafe, float weightedVelLinYSafe) 
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
            velLinXFeedback *= cfg_->control.vx_absmax / std::max(speedLinXFeedback, speedLinYFeedback);
            velLinYFeedback *= cfg_->control.vy_absmax / std::max(speedLinXFeedback, speedLinYFeedback);
        }

        // std::max(-cfg_->control.vang_absmax, std::min(cfg_->control.vang_absmax, velAngFeedback));
        return;
    }

    void TrajectoryController::runBearingRateCBF(const Eigen::Vector4f & state, 
                                                 const Eigen::Vector4f & rightGapPtState,
                                                 const Eigen::Vector4f & leftGapPtState,
                                                 const Eigen::Vector2f & currRbtAcc,
                                                 float & velLinXSafe, float & velLinYSafe, float & PsiCBF) 
    {
        float hSafe = 0.0;
        Eigen::Vector4f dHSafeDx(0.0, 0.0, 0.0, 0.0);
        
        float hLeft = leftGapSideCBF(leftGapPtState);
        float hRight = rightGapSideCBF(rightGapPtState);
        Eigen::Vector4f dHLeftDx = leftGapSideCBFDerivative(leftGapPtState);
        Eigen::Vector4f dHRightDx = rightGapSideCBFDerivative(rightGapPtState);

        // need to potentially ignore if gap is non-convex
        ROS_INFO_STREAM("rbt velocity: " << state[2] << ", " << state[3] << ", currRbtAcc: " << currRbtAcc[0] << ", " << currRbtAcc[1]);
        ROS_INFO_STREAM("left rel state: " << leftGapPtState[0] << ", " << leftGapPtState[1] << ", " << leftGapPtState[2] << ", " << leftGapPtState[3]);
        ROS_INFO_STREAM("right rel state: " << rightGapPtState[0] << ", " << rightGapPtState[1] << ", " << rightGapPtState[2] << ", " << rightGapPtState[3]);

        Eigen::Vector2f leftBearingVect(leftGapPtState[0], leftGapPtState[1]);
        Eigen::Vector2f rightBearingVect(rightGapPtState[0], rightGapPtState[1]);

        Eigen::Vector2f leftBearingNorm = leftBearingVect / leftBearingVect.norm();
        Eigen::Vector2f rightBearingNorm = rightBearingVect / rightBearingVect.norm();

        float determinant = rightBearingNorm[0]*leftBearingNorm[1] - rightBearingNorm[1]*leftBearingNorm[0];      
        float dot = rightBearingNorm[0]*leftBearingNorm[0] + rightBearingNorm[1]*leftBearingNorm[1];

        // float swept_check = ;     
        float leftToRightAngle = -std::atan2(determinant, dot);

        if (leftToRightAngle < 0)
            leftToRightAngle += 2*M_PI; 

        ROS_INFO_STREAM("L_to_R angle: " << leftToRightAngle);
        ROS_INFO_STREAM("left CBF: " << hLeft);
        ROS_INFO_STREAM("left CBF partials: " << dHLeftDx[0] << ", " << dHLeftDx[1] << ", " << dHLeftDx[2] << ", " << dHLeftDx[3]);

        ROS_INFO_STREAM("right CBF: " << hRight);
        ROS_INFO_STREAM("right CBF partials: " << dHRightDx[0] << ", " << dHRightDx[1] << ", " << dHRightDx[2] << ", " << dHRightDx[3]);

        float cbfParam = 1.0;
        bool cvxGap = leftToRightAngle < M_PI;
        Eigen::Vector4f dxdt(state[2], state[3], currRbtAcc[0], currRbtAcc[1]);
        float PsileftGapSideCBF = dHLeftDx.dot(dxdt) + cbfParam * hLeft;
        float PsirightGapSideCBF = dHRightDx.dot(dxdt) + cbfParam * hRight;
        
        ROS_INFO_STREAM("PsileftGapSideCBF: " << PsileftGapSideCBF << ", PsirightGapSideCBF: " << PsirightGapSideCBF);
        float velLinXSafeLeft = 0;
        float velLinYSafeLeft = 0;
        float velLinXSafeRight = 0;
        float velLinYSafeRight = 0;

        if (cvxGap && PsileftGapSideCBF < 0)  // right less than left
        {
            Eigen::Vector2f Lg_h_left(dHLeftDx[0], dHLeftDx[1]); // Lie derivative of h wrt x (we are doing command velocities)
            Eigen::Vector2f cmdVelSafeLeft = -(Lg_h_left * PsileftGapSideCBF) / (Lg_h_left.dot(Lg_h_left));
            velLinXSafeLeft = cmdVelSafeLeft[0];
            velLinYSafeLeft = cmdVelSafeLeft[1];    
        }
        
        if (cvxGap && PsirightGapSideCBF < 0) // left less than or equal to right
        { 
            Eigen::Vector2f Lg_h_right(dHRightDx[0], dHRightDx[1]); // Lie derivative of h wrt x (we are doing command velocities)
            Eigen::Vector2f cmdVelSafeRight = -(Lg_h_right * PsirightGapSideCBF) / (Lg_h_right.dot(Lg_h_right));
            velLinXSafeRight = cmdVelSafeRight[0];
            velLinYSafeRight = cmdVelSafeRight[1];      
        }
        
        velLinXSafe = velLinXSafeLeft + velLinXSafeRight;
        velLinYSafe = velLinYSafeLeft + velLinYSafeRight;
        
        if (velLinXSafe != 0 || velLinYSafe != 0) 
        {
            ROS_INFO_STREAM("cmdVel_safe left: " << velLinXSafeLeft << ", " << velLinXSafeLeft);
            ROS_INFO_STREAM("cmdVel_safe right: " << velLinXSafeRight << ", " << velLinYSafeRight);
            ROS_INFO_STREAM("cmdVel_safe x: " << velLinXSafe << ", cmdVel_safe y: " << velLinYSafe);
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
        Eigen::Vector4f dHLeftDX;
        
        float r = sqrt(pow(leftGapPtState(0), 2) + pow(leftGapPtState(1), 2));
        float rCrossV = leftGapPtState(0)*leftGapPtState(3) - leftGapPtState(1)*leftGapPtState(2);
        // derivative with respect to r_ox, r_oy, v_ox, v_oy
        
        dHLeftDX(0) =  leftGapPtState(3)/pow(r,2) - 2*leftGapPtState(0)*rCrossV/pow(r,4);
        dHLeftDX(1) = -leftGapPtState(2)/pow(r,2) - 2*leftGapPtState(1)*rCrossV/pow(r,4);
        dHLeftDX(2) = -leftGapPtState(1) / pow(r,2);
        dHLeftDX(3) =  leftGapPtState(0) / pow(r,2);

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
        Eigen::Vector4f dHRightDX;
        float r = sqrt(pow(rightGapPtState(0), 2) + pow(rightGapPtState(1), 2));
        float rCrossV = rightGapPtState(0)*rightGapPtState(3) - rightGapPtState(1)*rightGapPtState(2);

        // derivative with respect to r_ox, r_oy, v_ox, v_oy
        dHRightDX(0) = -rightGapPtState(3) / pow(r,2) + 2*rightGapPtState(0)*rCrossV / pow(r, 4);
        dHRightDX(1) =  rightGapPtState(2) / pow(r,2) + 2*rightGapPtState(1)*rCrossV / pow(r, 4);
        dHRightDX(2) =  rightGapPtState(1) / pow(r,2);
        dHRightDX(3) = -rightGapPtState(0) / pow(r,2);
        return dHRightDX;
    }


    void TrajectoryController::runProjectionOperator(const sensor_msgs::LaserScan & scan, 
                                                     const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                                     Eigen::Vector2f & cmdVelFeedback,
                                                     float & Psi, Eigen::Vector2f & dPsiDx,
                                                     float & velLinXSafe, float & velLinYSafe,
                                                     float & minDistTheta, float & minDist) 
{
        float r_min = cfg_->projection.r_min;
        float r_norm = cfg_->projection.r_norm;
        float r_norm_offset = cfg_->projection.r_norm_offset; 

        // iterates through current egocircle and finds the minimum distance to the robot's pose
        // ROS_INFO_STREAM("rbtPoseInSensorFrame pose: " << rbtPoseInSensorFrame.pose.position.x << ", " << rbtPoseInSensorFrame.pose.position.y);
        std::vector<float> minScanDists(scan.ranges.size());
        for (int i = 0; i < minScanDists.size(); i++) 
        {
            float theta = idx2theta(i); // i * scan.angle_increment - M_PI;
            float dist = scan.ranges.at(i);
            minScanDists.at(i) = dist2Pose(theta, dist, rbtPoseInSensorFrame.pose);
        }
        int minDistScanIdx = std::min_element(minScanDists.begin(), minScanDists.end()) - minScanDists.begin();

        float maxRange = r_norm + r_norm_offset;
        minDist = minScanDists.at(minDistScanIdx);
        minDist = std::min(minDist, maxRange); // minDist >= maxRange ? maxRange : minDist;
        // if (minDist <= 0) 
        //     ROS_INFO_STREAM("Min dist <= 0, : " << minDist);
        // minDist = minDist <= 0 ? 0.01 : minDist;
        // minDist -= cfg_->rbt.r_inscr / 2;

        // find minimum ego circle distance
        minDistTheta = idx2theta(minDistScanIdx); // (float)(minDistScanIdx) * scan.angle_increment + scan.angle_min;
        float min_x = minDist * std::cos(minDistTheta) - rbtPoseInSensorFrame.pose.position.x;
        float min_y = minDist * std::sin(minDistTheta) - rbtPoseInSensorFrame.pose.position.y;
        minDist = sqrt(pow(min_x, 2) + pow(min_y, 2));

        if (cfg_->debug.control_debug_log) 
        {
            ROS_INFO_STREAM("minDistScanIdx: " << minDistScanIdx << ", minDistTheta: "<< minDistTheta << ", minDist: " << minDist);
            ROS_INFO_STREAM("min_x: " << min_x << ", min_y: " << min_y);
        }
        std::vector<geometry_msgs::Point> vec = findLocalLine(minDistScanIdx);

        float projOpDotProd = 0.0;

        float min_diff_x = 0;
        float min_diff_y = 0;
        Eigen::Vector3f PsiDerAndPsi;
        if (cfg_->projection.line && vec.size() > 0) 
        {
            // Dist to 
            Eigen::Vector2f lowerMinDistPt(vec.at(0).x, vec.at(0).y);
            Eigen::Vector2f upperMinDistPt(vec.at(1).x, vec.at(1).y);
            Eigen::Vector2f rbt(0, 0);
            Eigen::Vector2f a = rbt - lowerMinDistPt;
            Eigen::Vector2f b = upperMinDistPt - lowerMinDistPt;
            Eigen::Vector2f c = rbt - upperMinDistPt;
            
            if (a.dot(b) < 0) // Pt 1 is closer than pt 2
            { 
                // ROS_INFO_STREAM("lowerMinDistPt");
                min_diff_x = - lowerMinDistPt(0);
                min_diff_y = - lowerMinDistPt(1);
                PsiDerAndPsi = calculateProjectionOperator(min_diff_x, min_diff_y);
                dPsiDx = Eigen::Vector2f(PsiDerAndPsi(0), PsiDerAndPsi(1));
                projOpDotProd = cmdVelFeedback.dot(dPsiDx);
            } else if (c.dot(-b) < 0)  // Pt 2 is closer than pt 1
            {
                min_diff_x = - upperMinDistPt(0);
                min_diff_y = - upperMinDistPt(1);
                PsiDerAndPsi = calculateProjectionOperator(min_diff_x, min_diff_y);
                dPsiDx = Eigen::Vector2f(PsiDerAndPsi(0), PsiDerAndPsi(1));
                projOpDotProd = cmdVelFeedback.dot(dPsiDx);
            } else  // pt's are equidistant
            {
                float line_dist = (lowerMinDistPt(0) * upperMinDistPt(1) - upperMinDistPt(0) * lowerMinDistPt(1)) / (lowerMinDistPt - upperMinDistPt).norm();
                float sign;
                sign = line_dist < 0 ? -1 : 1;
                line_dist *= sign;
                
                float line_si = (r_min / line_dist - r_min / r_norm) / (1. - r_min / r_norm);
                float line_dPsiDx_base = r_min / (pow(line_dist, 2) * (r_min / r_norm - 1));
                float line_dPsiDx_x = - (upperMinDistPt(1) - lowerMinDistPt(1)) / (lowerMinDistPt - upperMinDistPt).norm() * line_dPsiDx_base;
                float line_dPsiDx_y = - (lowerMinDistPt(0) - upperMinDistPt(0)) / (lowerMinDistPt - upperMinDistPt).norm() * line_dPsiDx_base;
                Eigen::Vector2f der(line_dPsiDx_x, line_dPsiDx_y);
                der /= der.norm();
                PsiDerAndPsi = Eigen::Vector3f(der(0), der(1), line_si);
                dPsiDx = der;
                // dPsiDx(1);// /= 3;
                projOpDotProd = cmdVelFeedback.dot(dPsiDx);
            }
        } else 
        {
            min_diff_x = - min_x;
            min_diff_y = - min_y;
            PsiDerAndPsi = calculateProjectionOperator(min_diff_x, min_diff_y); // return Psi, and dPsiDx
            dPsiDx = Eigen::Vector2f(PsiDerAndPsi(0), PsiDerAndPsi(1));
            // dPsiDx(1);// /= 3; // deriv wrt y?
            projOpDotProd = cmdVelFeedback.dot(dPsiDx);
        }

        Psi = PsiDerAndPsi(2);

        if (cfg_->debug.control_debug_log) 
        {
            ROS_INFO_STREAM("dPsiDx: " << dPsiDx[0] << ", " << dPsiDx[1]);
            ROS_INFO_STREAM("Psi: " << Psi << ", dot product check: " << projOpDotProd);
        }

        if(Psi >= 0 && projOpDotProd >= 0)
        {
            velLinXSafe = - Psi * projOpDotProd * PsiDerAndPsi(0);
            velLinYSafe = - Psi * projOpDotProd * PsiDerAndPsi(1);
            if (cfg_->debug.control_debug_log) ROS_INFO_STREAM("cmdVel_safe: " << velLinXSafe << ", " << velLinYSafe);
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

    Eigen::Vector3f TrajectoryController::calculateProjectionOperator(float min_diff_x, float min_diff_y) 
    {
        float r_min = cfg_->projection.r_min;
        float r_norm = cfg_->projection.r_norm;

        float minDist = sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2)); // (closest_pt - rbt)
        float Psi = (r_min / minDist - r_min / r_norm) / (1.0 - r_min / r_norm);
        float base_const = pow(minDist, 3) * (r_min - r_norm);
        float up_const = r_min * r_norm;
        float dPsiDx_x = up_const * min_diff_x / base_const;
        float dPsiDx_y = up_const * min_diff_y / base_const;

        float norm_dPsiDx = sqrt(pow(dPsiDx_x, 2) + pow(dPsiDx_y, 2));
        float norm_dPsiDx_x = dPsiDx_x / norm_dPsiDx;
        float norm_dPsiDx_y = dPsiDx_y / norm_dPsiDx;
        return Eigen::Vector3f(norm_dPsiDx_x, norm_dPsiDx_y, Psi);
    }

    Eigen::Matrix2cf TrajectoryController::getComplexMatrix(float x, float y, float quat_w, float quat_z)
    {
        std::complex<float> phase(quat_w, quat_z);
        phase = phase * phase;

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

    Eigen::Matrix2cf TrajectoryController::getComplexMatrix(float x, float y, float theta)
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

    int TrajectoryController::targetPoseIdx(const geometry_msgs::Pose & currPose, const geometry_msgs::PoseArray & poseArray) 
    {
        // Find pose right ahead
        std::vector<float> pose_diff(poseArray.poses.size());
        ROS_INFO_STREAM("[targetPoseIdx()]");

        // obtain distance from entire ref traj and current pose
        for (int i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            // tf::Quaternion q_c(currPose.orientation.x, currPose.orientation.y, currPose.orientation.z, currPose.orientation.w);
            // float yaw_curr = std::atan2(2.0 * (q_c.w() * q_c.z() + q_c.x() * q_c.y()), 
            //                             1 - 2.0 * (q_c.y() * q_c.y() + q_c.z() * q_c.z()));

            tf::Quaternion q_c_inv(currPose.orientation.x, currPose.orientation.y, currPose.orientation.z, -currPose.orientation.w);
  
            tf::Quaternion q_des(poseArray.poses[i].orientation.x, poseArray.poses[i].orientation.y, 
                                 poseArray.poses[i].orientation.z, poseArray.poses[i].orientation.w);
            // float yaw_des = std::atan2( 2.0 * (q_des.w() * q_des.z() + q_des.x() * q_des.y()), 
            //                           1 - 2.0 * (q_des.y() * q_des.y() + q_des.z() * q_des.z()));
            
            
            tf::Quaternion q_res = q_des * q_c_inv;
            float yaw_res = std::atan2( 2.0 * (q_res.w() * q_res.z() + q_res.x() * q_res.y()), 
                                      1 - 2.0 * (q_res.y() * q_res.y() + q_res.z() * q_res.z()));

            // ROS_INFO_STREAM("   pose" << i << ", yaw_curr: " << yaw_curr << ", yaw_des: " << yaw_des << ", yaw_res: " << yaw_res);

            pose_diff[i] = sqrt(pow(currPose.position.x - poseArray.poses[i].position.x, 2) + 
                                pow(currPose.position.y - poseArray.poses[i].position.y, 2)) + 
                                0.5 * std::abs(yaw_res);
        }

        // find pose in ref traj with smallest difference
        auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
        
        // go n steps ahead of pose with smallest difference
        int target_pose = std::distance(pose_diff.begin(), min_element_iter) + cfg_->control.ctrl_ahead_pose;
        return std::min(target_pose, int(poseArray.poses.size() - 1));
    }

    /*
    dynamic_gap::TrajPlan TrajectoryController::trajGen(geometry_msgs::PoseArray orig_traj)
    {
        dynamic_gap::TrajPlan traj;
        traj.header.frame_id = cfg_->odom_frame_id;
        for(size_t i = 0; i < orig_traj.poses.size(); i++)
        {
            geometry_msgs::Pose ni_pose = orig_traj.poses[i];
            geometry_msgs::Twist ni_twist = geometry_msgs::Twist();
            traj.poses.push_back(ni_pose);
            traj.twist.push_back(ni_twist);
        }
        return traj;
    }
    */

    float TrajectoryController::dist2Pose(float theta, float dist, geometry_msgs::Pose pose) 
    {
        float x = dist * std::cos(theta);
        float y = dist * std::sin(theta);
        return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    }

}