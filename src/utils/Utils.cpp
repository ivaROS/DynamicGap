#include <dynamic_gap/utils/Utils.h>

namespace dynamic_gap 
{
    // Utils::Utils() {}

    // Utils::~Utils() {}

    int theta2idx(const float theta)
    {
        return int((theta + M_PI) / angle_increment);
    }

    float idx2theta(const int idx)
    {
        return ((float) idx - half_num_scan) * angle_increment; // * M_PI / half_num_scan;
    }

    Eigen::Vector2f pol2car(const Eigen::Vector2f & polarVector) 
    {
        return Eigen::Vector2f(std::cos(polarVector[1]) * polarVector[0], std::sin(polarVector[1]) * polarVector[0]);
    }

    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    // FROM GAP_FEASIBILITY_CHECKER
    float getLeftToRightAngle(const Eigen::Vector2f & leftVect, 
                              const Eigen::Vector2f & rightVect) 
    {
        float determinant = leftVect[1]*rightVect[0] - leftVect[0]*rightVect[1];
        float dotProduct = leftVect[0]*rightVect[0] + leftVect[1]*rightVect[1];

        float leftToRightAngle = std::atan2(determinant, dotProduct);
        
        if (leftToRightAngle < 0)
            leftToRightAngle += 2*M_PI; 

        return leftToRightAngle;
    }

    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    // FROM GAP_MANIPULATOR
    float getLeftToRightAngle(const Eigen::Vector2f & leftVect,
                              const Eigen::Vector2f & rightVect, 
                              bool wrap) 
    {
        float determinant = leftVect[1]*rightVect[0] - leftVect[0]*rightVect[1];
        float dotProduct = leftVect[0]*rightVect[0] + leftVect[1]*rightVect[1];

        float leftToRightAngle = std::atan2(determinant, dotProduct);

        // wrapping to 0 < angle < 2pi
        if (wrap && leftToRightAngle < 0) 
        {
            // ROS_INFO_STREAM("wrapping " << leftToRightAngle);
            leftToRightAngle += 2*M_PI; 
        }

        return leftToRightAngle;
    }

    float getGapDist(const Eigen::Vector4f & gapState)
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

    float atanThetaWrap(float theta) 
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

    float quaternionToYaw(const tf::Quaternion & quat)
    {
        return std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()), 
                            1 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
    }

    int subtractAndWrapScanIndices(int a, int b) 
    {
        return (a < 0) ? a+b : a;
    }

    bool isGlobalPathLocalWaypointWithinGapAngle(int goalIdx, int lowerIdx, int upperIdx) 
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

    int signum(float value) 
    {
        return (value < 0 ? -1 : 1);
    }
}