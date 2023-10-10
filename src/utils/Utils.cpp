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
        return ((float) idx - half_num_scan) * M_PI / half_num_scan;
    }

    Eigen::Vector2f pol2car(Eigen::Vector2f polar_vector) 
    {
        return Eigen::Vector2f(std::cos(polar_vector[1]) * polar_vector[0], std::sin(polar_vector[1]) * polar_vector[0]);
    }

    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    // FROM GAP_FEASIBILITY_CHECKER
    float getLeftToRightAngle(Eigen::Vector2f left_norm_vect, 
                              Eigen::Vector2f right_norm_vect) 
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
    float getLeftToRightAngle(Eigen::Vector2f left_norm_vect, 
                              Eigen::Vector2f right_norm_vect, 
                              bool wrap) 
    {
        float determinant = left_norm_vect[1]*right_norm_vect[0] - left_norm_vect[0]*right_norm_vect[1];
        float dot_product = left_norm_vect[0]*right_norm_vect[0] + left_norm_vect[1]*right_norm_vect[1];

        float left_to_right_angle = std::atan2(determinant, dot_product);

        // wrapping to 0 < angle < 2pi
        if (wrap && left_to_right_angle < 0) 
        {
            ROS_INFO_STREAM("wrapping " << left_to_right_angle);
            left_to_right_angle += 2*M_PI; 
        }

        return left_to_right_angle;
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

    /*
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
    */
}