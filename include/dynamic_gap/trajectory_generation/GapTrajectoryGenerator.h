#pragma once

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>

// #include <traj_generator.h>
// #include <turtlebot_trajectory_generator/near_identity.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/trajectory_generation/trajectory_synthesis_methods.h>
// #include <vector>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// #include "tf/transform_datatypes.h"
// #include <sensor_msgs/LaserScan.h>
#include "OsqpEigen/OsqpEigen.h"

namespace dynamic_gap 
{
    class GapTrajectoryGenerator
    {
        public:
            GapTrajectoryGenerator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; };
            // void updateTF(geometry_msgs::TransformStamped tf) {planning2odom = tf;};
            
            std::tuple<geometry_msgs::PoseArray, std::vector<float>> generateTrajectory(dynamic_gap::Gap * selectedGap, 
                                                                                        const geometry_msgs::PoseStamped & currPose, 
                                                                                        const geometry_msgs::TwistStamped & currVel,
                                                                                        bool runGoToGoal);
            geometry_msgs::PoseArray transformLocalTrajectory(const geometry_msgs::PoseArray & path,
                                                              const geometry_msgs::TransformStamped & transform,
                                                              const std::string & sourceFrame,
                                                              const std::string & destFrame);
            std::tuple<geometry_msgs::PoseArray, std::vector<float>> processTrajectory(const std::tuple<geometry_msgs::PoseArray, std::vector<float>> & traj);

        private: 
            void initializeSolver(OsqpEigen::Solver & solver, int Kplus1, const Eigen::MatrixXd & A);

            float calculateBezierArclengthDistance(const Eigen::Vector2d & bezierPt0, 
                                                    const Eigen::Vector2d & bezierPt1, 
                                                    const Eigen::Vector2d & bezierPt2, 
                                                    const float & tStart, const float & tEnd, 
                                                    const float & numPoints);

            Eigen::VectorXd arclengthParameterizeBezier(const Eigen::Vector2d & bezierPt0, 
                                                        const Eigen::Vector2d & bezierPt1, 
                                                        const Eigen::Vector2d & bezierPt2, 
                                                        const float & num_curve_points, 
                                                        float & des_dist_interval);        

            void buildExtendedGapOrigin(const int numRGEPoints,
                                        const Eigen::Vector2d & extendedGapOrigin,
                                        const Eigen::Vector2d & bezierOrigin,
                                        Eigen::MatrixXd & curvePosns,
                                        Eigen::MatrixXd & curveVels,
                                        Eigen::MatrixXd & curveInwardNorms,
                                        bool left);
            void buildBezierCurve(const int numRGEPoints,
                                    const int totalNumCurvePts,
                                    const Eigen::VectorXd & arclengthParameters,
                                    const Eigen::Vector2d & bezierOrigin,
                                    const Eigen::Vector2d & bezierInitialPt,
                                    const Eigen::Vector2d & bezierTerminalPt,
                                    Eigen::MatrixXd & curvePosns,
                                    Eigen::MatrixXd & curveVels,
                                    Eigen::MatrixXd & curveInwardNorms,
                                    bool left);
            void buildExtendedBezierCurve(dynamic_gap::Gap * selectedGap, 
                                            Eigen::MatrixXd & gapCurvesPosns,
                                            Eigen::MatrixXd & gapCurvesInwardNorms, 
                                            Eigen::MatrixXd & gapSideAHPFCenters, Eigen::MatrixXd & allAHPFCenters,
                                            const Eigen::Vector2d & leftGapPtVel, const Eigen::Vector2d & rightGapPtVel, const Eigen::Vector2d & maxRbtVel,
                                            const Eigen::Vector2d & leftCurveInitPt, const Eigen::Vector2d & leftCurveTermPt, 
                                            const Eigen::Vector2d & rightCurveInitPt, const Eigen::Vector2d & rightCurveTermPt, 
                                            const Eigen::Vector2d & gapGoalTermPt, 
                                            float leftBezierWeight, float rightBezierWeight, 
                                            float numCurvePts, int numLeftRGEPoints, int numRightRGEPoints,
                                            const Eigen::Vector2d & initRbtPos);
            void setConstraintMatrix(Eigen::MatrixXd &A, int N, int Kplus1, 
                                     const Eigen::MatrixXd & gapCurvesPosns, 
                                     const Eigen::MatrixXd & gapCurvesInwardNorms,
                                     const Eigen::MatrixXd & allAHPFCenters);


            // geometry_msgs::TransformStamped planning2odom;       
            // int numCurvePts;
            const DynamicGapConfig* cfg_;

    };
}