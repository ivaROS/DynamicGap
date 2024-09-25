#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <tf/tf.h>
#include <chrono>

namespace dynamic_gap 
{

    //////////////////////////////
    //        VARIABLES         //
    //////////////////////////////

    static float eps = std::numeric_limits<float>::min(); /**< Infinitesimal epsilon value */

    static int half_num_scan = 256; /**< Half of total rays in scan */
    static float angle_increment = (2*M_PI) / (2*half_num_scan - 1); /**< Angular increment of scan */

    static Eigen::Matrix2f Rpi2 = (Eigen::Matrix2f() << 0.0, -1.0, 1.0, 0.0).finished(); /**< Rotation matrix for pi/2 */
 
    static Eigen::Matrix2f Rnegpi2 = (Eigen::Matrix2f() << 0.0, 1.0, -1.0, 0.0).finished(); /**< Rotation matrix for -pi/2 */

    enum planningStepIdxs { GAP_DET = 0, 
                            GAP_SIMP = 1, 
                            GAP_ASSOC = 2, 
                            GAP_EST = 3,
                            SCAN = 4,
                            GAP_PROP = 5,
                            GAP_MANIP = 6,
                            GAP_FEAS = 7,
                            SCAN_PROP = 8,
                            TRAJ_GEN = 9,
                            TRAJ_PICK = 10,
                            TRAJ_COMP = 11,
                            PLAN = 12,
                            FEEBDACK = 13,
                            PO = 14,
                            CONTROL = 15
                            };


    //////////////////////////////
    //       CONVERSIONS        // 
    //////////////////////////////
    
    /**
    * \brief normalize angle to interval [-pi, pi)
    * \param theta incoming angle to normalize
    * \remark This function is based on normalize_theta from g2o
    *         see: https://github.com/RainerKuemmerle/g2o/blob/master/g2o/stuff/misc.h
    */
    inline float normalize_theta(const float & theta)
    {
        if (theta >= -M_PI && theta < M_PI) 
            return theta;

        float multiplier = std::floor(theta / (2.0 * M_PI));
        float normalized_theta = theta - multiplier * 2.0 * M_PI;
        
        if (normalized_theta >= M_PI) 
            normalized_theta -= 2.0 * M_PI;
        
        if (normalized_theta < -M_PI) 
            normalized_theta += 2.0 * M_PI;

        return normalized_theta;
    }

    /**
    * \brief Conversion from polar coordinates to Cartesian coordinates
    * \param polarVector polar coordinate vector
    * \return Cartesian coordinate vector
    */
    inline Eigen::Vector2f pol2car(const Eigen::Vector2f & polarVector)
    {
        return Eigen::Vector2f(std::cos(polarVector[1]) * polarVector[0], std::sin(polarVector[1]) * polarVector[0]);        
    }

    /**
    * \brief Conversion from scan index to scan theta
    * \param idx scan index
    * \return scan theta
    */
    inline float idx2theta(const int & idx)
    {
        // assert(idx >= 0);
        // assert(idx < 2*half_num_scan);
        
        return ((float) idx - half_num_scan) * angle_increment;
    }

    /**
    * \brief Conversion from scan theta to scan index
    * \param theta scan theta
    * \return scan index
    */
    inline int theta2idx(const float & theta)
    {
        float theta_wrap = theta;
        if (theta < -M_PI || theta >= M_PI)
            theta_wrap = normalize_theta(theta);
        
        // assert(theta >= -M_PI);
        // assert(theta <= M_PI);

        return int(std::round((theta + M_PI) / angle_increment));
    }

    /**
    * \brief Helper for extracting yaw angle from quaternion
    * \param quat incoming quaternion
    * \return yaw angle
    */
    inline float quaternionToYaw(const tf::Quaternion & quat)
    {
        return std::atan2(2.0 * (quat.w() * quat.z() + quat.x() * quat.y()), 
                            1 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
    }

    /**
    * \brief Helper for extracting yaw angle from quaternion
    * \param quat incoming quaternion
    * \return yaw angle
    */
    inline float quaternionToYaw(const geometry_msgs::Quaternion & quat)
    {
        return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 
                            1 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
    }
    //////////////////////////////
    //     ANGLE CONVERSIONS    // 
    //////////////////////////////

    /**
    * \brief Calculate signed angle from left gap point to right gap point (from -pi to pi)
    * \param leftVect left gap point vector
    * \param rightVect right gap point vector
    * \return signed angle from left gap point to right gap point
    */
    inline float getSignedLeftToRightAngle(const Eigen::Vector2f & leftVect, 
                                    const Eigen::Vector2f & rightVect) 
    {
        float determinant = leftVect[1]*rightVect[0] - leftVect[0]*rightVect[1];
        float dotProduct = leftVect[0]*rightVect[0] + leftVect[1]*rightVect[1];

        float leftToRightAngle = std::atan2(determinant, dotProduct);

        return leftToRightAngle;
    }

    /**
    * \brief Calculate angle swept (clockwise) from left gap point to right gap point (0 to 2pi)
    * \param leftVect left gap point vector
    * \param rightVect right gap point vector
    * \return angle swept from left gap point to right gap point
    */
    inline float getSweptLeftToRightAngle(const Eigen::Vector2f & leftVect,
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

    //////////////////////////////
    //   GAP STATE OPERATIONS   // 
    //////////////////////////////

    /**
    * \brief Calculating gap point bearing rate of change
    * \param gapState full gap state
    * \return gap point bearing rate of change
    */
    inline float getGapBearingRateOfChange(const Eigen::Vector4f & gapState)
    {
        return (gapState[0]*gapState[3] - gapState[1]*gapState[2]) / (pow(gapState[0], 2) + pow(gapState[1], 2));
    }

    //////////////////////////////
    //    SCAN OPERATIONS       //
    //////////////////////////////

    /**
    * \brief Calculate distance from a scan point to a robot pose
    * \param theta orientation of scan point
    * \param range range of scan point
    * \param pose robot pose
    * \return distance from scan point to robot pose
    */
    inline float dist2Pose(const float & theta, 
                    const float & range, 
                    const geometry_msgs::Pose & pose) 
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

    //////////////////////////////
    //       FUNCTIONS          // 
    //////////////////////////////
    
    /**
    * \brief Signum function
    * \param value input
    * \return sign of input (1 or -1)
    */
    inline int signum(const float & value) 
    {
        return (value < 0 ? -1 : 1);
    }

    //////////////////////////////
    //       OPERATIONS         // 
    //////////////////////////////

    /**
    * \brief Enhanced division operator (float) that adds epsilon to denominator to ensure that we 
    * do not divide by zero
    * \param numerator numerator of division operator
    * \param denominator denominator of division operator
    * \return result of division    
    */
    inline float epsilonDivide(const float & numerator, const float & denominator)
    {
        return numerator / (denominator + eps); 
    }

    /**
    * \brief Enhanced division operator (2D float vector) that adds epsilon to denominator to ensure that we 
    * do not divide by zero
    * \param numerator numerator of division operator
    * \param denominator denominator of division operator
    * \return result of division    
    */    
    inline Eigen::Vector2f epsilonDivide(const Eigen::Vector2f & numerator, const float & denominator)
    {
        return numerator / (denominator + eps); 
    }

    /**
    * \brief Enhanced division operator (2D double vector) that adds epsilon to denominator to ensure that we 
    * do not divide by zero
    * \param numerator numerator of division operator
    * \param denominator denominator of division operator
    * \return result of division    
    */    
    inline Eigen::Vector2d epsilonDivide(const Eigen::Vector2d & numerator, const double & denominator)
    {
        return numerator / (denominator + eps); 
    }

    //////////////////////////////
    //          TIMING          //
    //////////////////////////////

    /**
    * \brief Calculate time that has elapsed in seconds since given start time
    * \param startTime start time
    * \return elapsed time in seconds
    */
    inline float timeTaken(const std::chrono::steady_clock::time_point & startTime)
    {
        float timeTakenInMilliseconds = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - startTime).count();
        float timeTakenInSeconds = timeTakenInMilliseconds / 1.0e6;
        return timeTakenInSeconds;
    }
}