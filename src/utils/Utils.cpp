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

    Eigen::Vector2f pol2car(const Eigen::Vector2f & polar_vector) 
    {
        return Eigen::Vector2f(std::cos(polar_vector[1]) * polar_vector[0], std::sin(polar_vector[1]) * polar_vector[0]);
    }

    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    // FROM GAP_FEASIBILITY_CHECKER
    float getLeftToRightAngle(const Eigen::Vector2f & left_norm_vect, 
                              const Eigen::Vector2f & right_norm_vect) 
    {
        float determinant = left_norm_vect[1]*right_norm_vect[0] - left_norm_vect[0]*right_norm_vect[1];
        float dot_product = left_norm_vect[0]*right_norm_vect[0] + left_norm_vect[1]*right_norm_vect[1];

        float left_to_right_angle = std::atan2(determinant, dot_product);
        
        if (left_to_right_angle < 0)
            left_to_right_angle += 2*M_PI; 

        return left_to_right_angle;
    }

    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    // FROM GAP_MANIPULATOR
    float getLeftToRightAngle(const Eigen::Vector2f & left_norm_vect,
                              const Eigen::Vector2f & right_norm_vect, 
                              bool wrap) 
    {
        float determinant = left_norm_vect[1]*right_norm_vect[0] - left_norm_vect[0]*right_norm_vect[1];
        float dot_product = left_norm_vect[0]*right_norm_vect[0] + left_norm_vect[1]*right_norm_vect[1];

        float left_to_right_angle = std::atan2(determinant, dot_product);

        // wrapping to 0 < angle < 2pi
        if (wrap && left_to_right_angle < 0) 
        {
            // ROS_INFO_STREAM("wrapping " << left_to_right_angle);
            left_to_right_angle += 2*M_PI; 
        }

        return left_to_right_angle;
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

    float atanThetaWrap(float theta) 
    {
        float new_theta = theta;
        while (new_theta <= -M_PI) 
        {
            new_theta += 2*M_PI;
            // ROS_INFO_STREAM("wrapping theta: " << theta << " to new_theta: " << new_theta);
        } 
        
        while (new_theta >= M_PI) 
        {
            new_theta -= 2*M_PI;
            // ROS_INFO_STREAM("wrapping theta: " << theta << " to new_theta: " << new_theta);
        }

        return new_theta;
    }

    float quaternionToYaw(const tf::Quaternion & quat)
    {
        return std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()), 
                            1 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
    }

    int subtract_wrap(int a, int b) 
    {
        return (a < 0) ? a+b : a;
    }

    // helper functions need to be placed above where they are used
    bool isGapLocalGoalWithin(int goal_idx, int idx_lower, int idx_upper, int full_scan) 
    {
        if (idx_lower < idx_upper) 
        {
            // ROS_INFO_STREAM("no wrapping, is goal idx between " << idx_lower << " and " << idx_upper);
            return (goal_idx > idx_lower && goal_idx < idx_upper); //if no wrapping occurs
        } else 
        {
            // ROS_INFO_STREAM("wrapping, is goal idx between " << idx_lower << " and " << full_scan << ", or between " << 0 << " and " << idx_upper);
            return (goal_idx > idx_lower && goal_idx < full_scan) || (goal_idx > 0 && goal_idx < idx_upper); // if wrapping occurs
        }
    }

    int signum(float value) 
    {
        return (value < 0 ? -1 : 1);
    }
}