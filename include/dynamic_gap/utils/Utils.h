#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <tf/tf.h>
#include <chrono>

namespace dynamic_gap 
{
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
    * \brief Conversion from polar coordinates to Cartesian coordinates
    * \param polarVector polar coordinate vector
    * \return Cartesian coordinate vector
    */
    Eigen::Vector2f pol2car(const Eigen::Vector2f & polarVector);

    /**
    * \brief Conversion from scan index to scan theta
    * \param idx scan index
    * \return scan theta
    */
    float idx2theta(const int & idx);

    /**
    * \brief Conversion from scan theta to scan index
    * \param theta scan theta
    * \return scan index
    */
    int theta2idx(const float & theta);

    /**
    * \brief Helper for extracting yaw angle from quaternion
    * \param quat incoming quaternion
    * \return yaw angle
    */
    float quaternionToYaw(const tf::Quaternion & quat);

    /**
    * \brief Helper for extracting yaw angle from quaternion
    * \param quat incoming quaternion
    * \return yaw angle
    */
    float quaternionToYaw(const geometry_msgs::Quaternion & quat);

    //////////////////////////////
    //     ANGLE CONVERSIONS    // 
    //////////////////////////////

    // float atanThetaWrap(const float & theta); 
    // float atanThetaWrap(float theta);

    /**
    * \brief Calculate signed angle from left gap point to right gap point (from -pi to pi)
    * \param leftVect left gap point vector
    * \param rightVect right gap point vector
    * \return signed angle from left gap point to right gap point
    */
    float getSignedLeftToRightAngle(const Eigen::Vector2f & leftVect, 
                                    const Eigen::Vector2f & rightVect);

    /**
    * \brief Calculate angle swept (clockwise) from left gap point to right gap point (0 to 2pi)
    * \param leftVect left gap point vector
    * \param rightVect right gap point vector
    * \return angle swept from left gap point to right gap point
    */
    float getSweptLeftToRightAngle(const Eigen::Vector2f & leftVect, 
                                   const Eigen::Vector2f & rightVect);


    /**
    * \brief normalize angle to interval [-pi, pi)
    * \param theta incoming angle to normalize
    * \remark This function is based on normalize_theta from g2o
    *         see: https://github.com/RainerKuemmerle/g2o/blob/master/g2o/stuff/misc.h
    */
    inline float normalize_theta(const float & theta)
    {
        if (theta >= -M_PI && theta < M_PI) return theta;

        float multiplier = std::floor(theta / (2.0 * M_PI));
        float normalized_theta             = theta - multiplier * 2.0 * M_PI;
        if (normalized_theta >= M_PI) normalized_theta -= 2.0 * M_PI;
        if (normalized_theta < -M_PI) normalized_theta += 2.0 * M_PI;

        return normalized_theta;
    }

    //////////////////////////////
    //   GAP STATE OPERATIONS   // 
    //////////////////////////////

    /**
    * \brief Extract gap point range from full gap state
    * \param gapState full gap state
    * \return gap point range
    */
    float getGapRange(const Eigen::Vector4f & gapState);

    /**
    * \brief Extract gap point bearing from full gap state
    * \param gapState full gap state
    * \return gap point bearing
    */
    float getGapBearing(const Eigen::Vector4f & gapState);

    /**
    * \brief Calculating gap point bearing rate of change
    * \param gapState full gap state
    * \return gap point bearing rate of change
    */
    float getGapBearingRateOfChange(const Eigen::Vector4f & gapState);

    //////////////////////////////
    //    SCAN OPERATIONS       //
    //////////////////////////////

    /**
    * \brief Helper function for (potentially) wrapping negative scan indices
    * \param scanIdx potentially negative scan index
    * \return wrapped scan index
    */
    int wrapScanIndex(const int & scanIdx);

    /**
    * \brief Calculate distance from a scan point to a robot pose
    * \param theta orientation of scan point
    * \param range range of scan point
    * \param pose robot pose
    * \return distance from scan point to robot pose
    */
    float dist2Pose(const float & theta, 
                    const float & range, 
                    const geometry_msgs::Pose & pose);
    
    //////////////////////////////
    //       FUNCTIONS          // 
    //////////////////////////////
    
    /**
    * \brief Signum function
    * \param value input
    * \return sign of input (1 or -1)
    */
    int signum(const float & value);

    /**
    * \brief Helper function for determining if bearing of global path local waypoint falls within gap
    * \param goalIdx scan index of global path local waypoint
    * \param lowerIdx lower index of gap points (could be left or right)
    * \param upperIdx upper index of gap points (could be left or right)
    */
    bool isGlobalPathLocalWaypointWithinGapAngle(const int & goalIdx, const int & lowerIdx, const int & upperIdx);

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
    float epsilonDivide(const float & numerator, 
                        const float & denominator);
    
    /**
    * \brief Enhanced division operator (2D float vector) that adds epsilon to denominator to ensure that we 
    * do not divide by zero
    * \param numerator numerator of division operator
    * \param denominator denominator of division operator
    * \return result of division    
    */    
    Eigen::Vector2f epsilonDivide(const Eigen::Vector2f & numerator, 
                                  const float & denominator);
    
    /**
    * \brief Enhanced division operator (2D double vector) that adds epsilon to denominator to ensure that we 
    * do not divide by zero
    * \param numerator numerator of division operator
    * \param denominator denominator of division operator
    * \return result of division    
    */    
    Eigen::Vector2d epsilonDivide(const Eigen::Vector2d & numerator, 
                                  const double & denominator);

    /**
    * \brief Calculate unit norm of 2D float vector
    * \param vector 2D float vector
    * \return unit norm vector
    */
    Eigen::Vector2f unitNorm(const Eigen::Vector2f & vector);
    
    /**
    * \brief Calculate unit norm of 2D double vector
    * \param vector 2D double vector
    * \return unit norm vector
    */    
    Eigen::Vector2d unitNorm(const Eigen::Vector2d & vector);

    //////////////////////////////
    //          TIMING          //
    //////////////////////////////

    /**
    * \brief Calculate time that has elapsed in seconds since given start time
    * \param startTime start time
    * \return elapsed time in seconds
    */
    float timeTaken(const std::chrono::steady_clock::time_point & startTime);

    //////////////////////////////
    //        VARIABLES         //
    //////////////////////////////

    static float eps = std::numeric_limits<float>::min(); /**< Infinitesimal epsilon value */

    static int half_num_scan = 256; /**< Half of total rays in scan */
    static float angle_increment = (2*M_PI) / (2*half_num_scan - 1); /**< Angular increment of scan */

    static Eigen::Matrix2f Rpi2 = (Eigen::Matrix2f() << 0.0, -1.0, 1.0, 0.0).finished(); /**< Rotation matrix for pi/2 */
 
    static Eigen::Matrix2f Rnegpi2 = (Eigen::Matrix2f() << 0.0, 1.0, -1.0, 0.0).finished(); /**< Rotation matrix for -pi/2 */
}