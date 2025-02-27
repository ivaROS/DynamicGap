#include <dynamic_gap/trajectory_generation/GapGoalPlacer.h>

namespace dynamic_gap
{

    void GapGoalPlacer::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    void GapGoalPlacer::setGapGoal(Gap * gap, 
                                    const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame, 
                                    const geometry_msgs::PoseStamped & globalGoalRobotFrame) 
    {
        // try
        // {

        ROS_INFO_STREAM("    [setGapGoal()]");

        int leftIdx = gap->manipLeftIdx();
        int rightIdx = gap->manipRightIdx();
        float leftRange = gap->manipLeftRange();
        float rightRange = gap->manipRightRange();

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);

        float xLeft = (leftRange) * cos(leftTheta);
        float yLeft = (leftRange) * sin(leftTheta);
        float xRight = (rightRange) * cos(rightTheta);
        float yRight = (rightRange) * sin(rightTheta);

        Eigen::Vector2f leftPt(xLeft, yLeft);
        Eigen::Vector2f rightPt(xRight, yRight);

        gap->leftGapPtModel_->isolateGapDynamics();
        gap->rightGapPtModel_->isolateGapDynamics();

        Eigen::Vector4f leftGapState = gap->leftGapPtModel_->getGapState();
        Eigen::Vector4f rightGapState = gap->rightGapPtModel_->getGapState();

        ROS_INFO_STREAM("        gap polar points, left: (" << leftIdx << ", " << leftRange << ") , right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM("        gap cart points, left: (" << xLeft << ", " << yLeft << ") , right: (" << xRight << ", " << yRight << ")");

        float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

        Eigen::Vector2f globalGoalRobotFrameVector(globalGoalRobotFrame.pose.position.x, 
                                globalGoalRobotFrame.pose.position.y);

        float globalGoalTheta = std::atan2(globalGoalRobotFrameVector[1], globalGoalRobotFrameVector[0]);
        float globalGoalIdx = theta2idx(globalGoalTheta); // std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);

        // ROS_INFO_STREAM("        global goal idx: " << globalGoalIdx << 
                // ", global goal: (" << globalGoalRobotFrameVector[0] << 
                //                  ", " << globalGoalRobotFrameVector[1] << ")");


        // Check if global goal is within current scan
        //      - previously, we also checked if global goal was within *gap*,
        //        but in a corridor or corner, the global goal will not always be contained
        //        within one of our gaps. Therefore, we will perform a lazy check
        //        to enable the planner to run g2g if the global goal is within the scan,
        //        and then we can evaluate whether or not the path is fine later
        if (checkWaypointVisibility(leftPt, rightPt, globalGoalRobotFrameVector))
        {                
            // all we will do is mark it for later so we can run g2g policy on global goal.
            // Still set mid point for pursuit guidance policy and feasibility check
            gap->globalGoalWithin = true;

            ROS_INFO_STREAM("        global goal within gap");
            ROS_INFO_STREAM("            goal: " << globalGoalRobotFrameVector[0] << 
                                                    ", " << globalGoalRobotFrameVector[1]);

        }

        if (leftToRightAngle < M_PI) // M_PI / 2,  M_PI / 4
        {
            ROS_INFO_STREAM("        Option 1: gap mid point");

            float centerTheta = leftTheta - (leftToRightAngle / 2.0);
            float centerRange = (leftRange + rightRange) / 2.;
            Eigen::Vector2f centerPt(centerRange * std::cos(centerTheta),
                    centerRange * std::sin(centerTheta));
            Eigen::Vector2f centerVel = ((leftGapState.tail(2) + rightGapState.tail(2)) / 2.);

            ROS_INFO_STREAM("            original goal: " << centerPt[0] << ", " << centerPt[1]);                 

            Eigen::Vector2f gapGoalRadialOffset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * centerPt.normalized();

            Eigen::Vector2f inflatedCenterPt = centerPt + gapGoalRadialOffset;

            ROS_INFO_STREAM("            inflated goal: " << inflatedCenterPt[0] << ", " << inflatedCenterPt[1]);                 

            gap->setGoal(inflatedCenterPt);
            gap->setGoalVel(centerVel);
        } else
        {
            ROS_INFO_STREAM("        Option 2: global path local waypoint biased");

            Eigen::Vector2f globalPathLocalWaypointRobotFrameVector(globalPathLocalWaypointRobotFrame.pose.position.x, 
                                                    globalPathLocalWaypointRobotFrame.pose.position.y);
            float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypointRobotFrameVector[1], globalPathLocalWaypointRobotFrameVector[0]);

            float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalPathLocalWaypointRobotFrameVector);
            float rightToWaypointAngle = getSweptLeftToRightAngle(rightPt, globalPathLocalWaypointRobotFrameVector);

            float biasedGapGoalTheta = setBiasedGapGoalTheta(leftTheta, rightTheta, globalPathLocalWaypointTheta,
                                            leftToRightAngle, leftToWaypointAngle, rightToWaypointAngle);
            Eigen::Vector2f biasedGapGoalUnitNorm(std::cos(biasedGapGoalTheta), std::sin(biasedGapGoalTheta));

            float leftToGapGoalAngle = getSweptLeftToRightAngle(leftPt, biasedGapGoalUnitNorm); 

            // float biasedGapGoalIdx = theta2idx(biasedGapGoalTheta); // std::floor(biasedGapGoalTheta*half_num_scan/M_PI + half_num_scan);

            float biasedGapGoalDist = leftRange + (rightRange - leftRange) * leftToGapGoalAngle / leftToRightAngle;
            Eigen::Vector2f biasedGapGoal(biasedGapGoalDist * cos(biasedGapGoalTheta), biasedGapGoalDist * sin(biasedGapGoalTheta));
            Eigen::Vector2f biasedGapVel = leftGapState.tail(2) + (rightGapState.tail(2) - leftGapState.tail(2)) * leftToGapGoalAngle / leftToRightAngle;

            ROS_INFO_STREAM("            original goal: " << biasedGapGoal[0] << ", " << biasedGapGoal[1]);                 

            Eigen::Vector2f gapGoalRadialOffset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * biasedGapGoal.normalized();

            Eigen::Vector2f inflatedBiasedGapGoal = biasedGapGoal + gapGoalRadialOffset;

            ROS_INFO_STREAM("            inflated goal: " << inflatedBiasedGapGoal[0] << ", " << inflatedBiasedGapGoal[1]);                 

            gap->setGoal(inflatedBiasedGapGoal);
            gap->setGoalVel(biasedGapVel);
        }

        // } catch (...)
        // {
        // ROS_WARN_STREAM_NAMED("GapManipulator", "[setGapGoal() failed]");
        // }      
    }

    float GapGoalPlacer::setBiasedGapGoalTheta(const float & leftTheta, const float & rightTheta, const float & globalGoalTheta,
                                                const float & leftToRightAngle, const float & leftToWaypointAngle,  const float & rightToWaypointAngle)
    {
        float biasedGapGoalTheta = 0.0;
        if (leftTheta > rightTheta) // gap is not behind robot
        { 
        biasedGapGoalTheta = std::min(leftTheta, std::max(rightTheta, globalGoalTheta));
        } else // gap is behind
        { 
        if (0 < leftToWaypointAngle && leftToWaypointAngle < leftToRightAngle)
        biasedGapGoalTheta = globalGoalTheta;
        else if (std::abs(leftToWaypointAngle) < std::abs(rightToWaypointAngle))
        biasedGapGoalTheta = leftTheta;
        else
        biasedGapGoalTheta = rightTheta;
        }

        // ROS_INFO_STREAM("            leftTheta: " << leftTheta << ", rightTheta: " << rightTheta << ", globalGoalTheta: " << globalGoalTheta);
        // ROS_INFO_STREAM("            leftToRightAngle: " << leftToRightAngle << ", leftToWaypointAngle: " << leftToWaypointAngle << ", rightToWaypointAngle: " << rightToWaypointAngle);

        return biasedGapGoalTheta;
    }

    bool GapGoalPlacer::checkWaypointVisibility(const Eigen::Vector2f & leftPt, 
                                                const Eigen::Vector2f & rightPt,
                                                const Eigen::Vector2f & globalGoal) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        // with robot as 0,0 (globalGoal in robot frame as well)
        float dist2goal = globalGoal.norm(); // sqrt(pow(globalGoal.pose.position.x, 2) + pow(globalGoal.pose.position.y, 2));

        sensor_msgs::LaserScan scan = *scan_.get();
        auto minScanRange = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr)
        return true;

        // If within closest configuration space
        if (dist2goal < minScanRange - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr)
        return true;

        // Should be sufficiently far, otherwise we are in trouble
        float globalGoalAngle = std::atan2(globalGoal[1], globalGoal[0]);
        int globalGoalIdx = theta2idx(globalGoalAngle);

        // Should be sufficiently far, otherwise we are in trouble

        // get gap's range at globalGoal idx

        // float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);
        // float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalGoal);
        // float gapGoalRange = (rightPt.norm() - leftPt.norm()) * epsilonDivide(leftToWaypointAngle, leftToRightAngle) + leftPt.norm();

        float rangeAtGoalIdx = scan.ranges.at(globalGoalIdx);

        return dist2goal < rangeAtGoalIdx;
    }
}