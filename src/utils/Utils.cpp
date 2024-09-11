#include <dynamic_gap/utils/Utils.h>

namespace dynamic_gap 
{
    Eigen::Vector2f pol2car(const Eigen::Vector2f & polarVector) 
    {
        return Eigen::Vector2f(std::cos(polarVector[1]) * polarVector[0], std::sin(polarVector[1]) * polarVector[0]);
    }

    float idx2theta(const int & idx)
    {
        // assert(idx >= 0);
        // assert(idx < 2*half_num_scan);
        
        return ((float) idx - half_num_scan) * angle_increment; // * M_PI / half_num_scan;
    }

    int theta2idx(const float & theta)
    {
        float theta_wrap = theta;
        if (theta < -M_PI || theta >= M_PI)
            theta_wrap = normalize_theta(theta);
        
        // assert(theta >= -M_PI);
        // assert(theta <= M_PI);

        return int(std::round((theta + M_PI) / angle_increment));
    }

    float quaternionToYaw(const tf::Quaternion & quat)
    {
        return std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()), 
                            1 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
    }

    float quaternionToYaw(const geometry_msgs::Quaternion & quat)
    {
        return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 
                            1 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
    }

    // if we wanted to incorporate how egocircle can change, 
    float dist2Pose(const float & theta, const float & range, const geometry_msgs::Pose & pose) 
    {
        // ego circle point in local frame, pose in local frame
        // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "   theta: " << theta << ", range: " << range);
        // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "   rbt_x: " << pose.position.x << ", rbt_y: " << pose.position.y);
        float x = range * std::cos(theta);
        float y = range * std::sin(theta);
        float dist = sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2)); 
        // ROS_INFO_STREAM_NAMED("TrajectoryEvaluator", "   dist: " << dist);
        return dist;
    }

    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    // FROM GAP_FEASIBILITY_CHECKER
    float getSignedLeftToRightAngle(const Eigen::Vector2f & leftVect, 
                              const Eigen::Vector2f & rightVect) 
    {
        float determinant = leftVect[1]*rightVect[0] - leftVect[0]*rightVect[1];
        float dotProduct = leftVect[0]*rightVect[0] + leftVect[1]*rightVect[1];

        float leftToRightAngle = std::atan2(determinant, dotProduct);

        return leftToRightAngle;
    }

    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    // FROM GAP_MANIPULATOR
    float getSweptLeftToRightAngle(const Eigen::Vector2f & leftVect,
                                   const Eigen::Vector2f & rightVect) 
    {
        float leftToRightAngle = getSignedLeftToRightAngle(leftVect, rightVect);

        // wrapping to 0 < angle < 2pi
        if (leftToRightAngle < 0) 
        {
            // ROS_INFO_STREAM("wrapping " << leftToRightAngle);
            leftToRightAngle += 2*M_PI; 
        }

        return leftToRightAngle;
    }

    float getGapRange(const Eigen::Vector4f & gapState)
    {
        return sqrt(pow(gapState[0], 2) + pow(gapState[1], 2));
    }

    float getGapBearing(const Eigen::Vector4f & gapState)
    {
        return std::atan2(gapState[1], gapState[0]);
    }    
   
    float getGapBearingRateOfChange(const Eigen::Vector4f & gapState)
    {
        return (gapState[0]*gapState[3] - gapState[1]*gapState[2]) / (pow(gapState[0], 2) + pow(gapState[1], 2));
    }

    // TODO: pretty print function for matrix to string

    // TODO: pretty print for function vector to string

    /*
    float atanThetaWrap(const float & theta) 
    {
        float wrappedTheta = theta;

        while (wrappedTheta <= -M_PI) 
        {
            wrappedTheta += 2*M_PI;
            // ROS_INFO_STREAM("wrapping theta: " << theta << " to wrappedTheta: " << wrappedTheta);
        } 
        
        while (wrappedTheta >= M_PI) 
        {
            wrappedTheta -= 2*M_PI;
            // ROS_INFO_STREAM("wrapping theta: " << theta << " to wrappedTheta: " << wrappedTheta);
        }

        return wrappedTheta;
    }
    */

    int wrapScanIndex(const int & scanIdx) 
    {
        return scanIdx + (scanIdx < 0) * (2 * half_num_scan);
    }

    bool isGlobalPathLocalWaypointWithinGapAngle(const int & goalIdx, const int & lowerIdx, const int & upperIdx) 
    {
        if (lowerIdx < upperIdx) 
        {
            // ROS_INFO_STREAM("no wrapping, is goal idx between " << lowerIdx << " and " << upperIdx);
            return (goalIdx > lowerIdx && goalIdx < upperIdx); //if no wrapping occurs
        } else 
        {
            // ROS_INFO_STREAM("wrapping, is goal idx between " << lowerIdx << " and " << full_scan << ", or between " << 0 << " and " << upperIdx);
            return (goalIdx > lowerIdx && goalIdx < (2*half_num_scan)) || (goalIdx > 0 && goalIdx < upperIdx); // if wrapping occurs
        }
    }

    int signum(const float & value) 
    {
        return (value < 0 ? -1 : 1);
    }

    // to avoid dividing by zero
    float epsilonDivide(const float & numerator, const float & denominator)
    {
        return numerator / (denominator + eps); 
    }

    Eigen::Vector2f epsilonDivide(const Eigen::Vector2f & numerator, const float & denominator)
    {
        return numerator / (denominator + eps); 
    }

    Eigen::Vector2d epsilonDivide(const Eigen::Vector2d & numerator, const double & denominator)
    {
        return numerator / (denominator + eps); 
    }    

    Eigen::Vector2f unitNorm(const Eigen::Vector2f & vector)
    {
        return vector / (vector.norm() + eps);
    }

    Eigen::Vector2d unitNorm(const Eigen::Vector2d & vector)
    {
        return vector / (vector.norm() + eps);
    }

    float timeTaken(const std::chrono::steady_clock::time_point & startTime)
    {
        float timeTakenInMilliseconds = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - startTime).count();
        float timeTakenInSeconds = timeTakenInMilliseconds / 1.0e6;
        return timeTakenInSeconds;
    }
}