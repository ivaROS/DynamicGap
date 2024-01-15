#pragma once

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>

// #include <traj_generator.h>
// #include <turtlebot_trajectory_generator/near_identity.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Trajectory.h>
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
        /**
        * \brief class responsible for generating local collision-free trajectories through gap
        */
        public:
            GapTrajectoryGenerator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; };
            
            /**
            * \brief generate local collision-free trajectory through gap
            * \param selectedGap gap through which trajectory will be generated
            * \param currPose current robot pose
            * \param currVel current robot velocity
            * \param runGoToGoal boolean for if go to goal trajectory method should be run
            * \return trajectory through gap
            */
            dynamic_gap::Trajectory generateTrajectory(dynamic_gap::Gap * selectedGap, 
                                                        const geometry_msgs::PoseStamped & currPose, 
                                                        const geometry_msgs::TwistStamped & currVel,
                                                        const bool & runGoToGoal);

            /**
            * \brief helper function for transforming trajectory from source frame to destination frame
            * \param path path that is to be transformed
            * \param transform transform to apply to path
            * \return transformed path
            */
            geometry_msgs::PoseArray transformPath(const geometry_msgs::PoseArray & path,
                                                    const geometry_msgs::TransformStamped & transform);

            /**
            * \brief helper function for post-processing the resulting trajectory which includes
            * removing intermediate poses that are sufficiently close together and
            * setting intermediate pose orientations to follow the path
            * \param traj incoming trajectory to be processed
            * \return post-processed trajectory
            */                       
            dynamic_gap::Trajectory processTrajectory(const dynamic_gap::Trajectory & traj);

        private: 
            /**
            * \brief helper function initializing QP solver for AHPF weight problem
            * \param solver QP solver to initialize
            * \param Kplus1 number of optimization variables
            * \param A populated constraint matrix
            * \return boolean for if solver was succesfully initialized  
            */
            bool initializeSolver(OsqpEigen::Solver & solver, 
                                    const int & Kplus1, 
                                    const Eigen::MatrixXd & A);

            /**
            * \brief calculate distance along Bezier curve
            * \param bezierPt0 first control point for Bezier curve
            * \param bezierPt1 second control point for Bezier curve
            * \param bezierPt2 third control point for Bezier curve
            * \param tStart initial parameter value to calculate distance from
            * \param tEnd final parameter value to calculate distance from
            * \param numPoints number of points along curve to approximate distance with
            * \return distance along Bezier curve
            */
            float approximateBezierArclengthDistance(const Eigen::Vector2d & bezierPt0, 
                                                    const Eigen::Vector2d & bezierPt1, 
                                                    const Eigen::Vector2d & bezierPt2, 
                                                    const float & tStart, 
                                                    const float & tEnd, 
                                                    const float & numPoints);

            /**
            * \brief obtain discrete arclength parameterization of Bezier curve
            * \param bezierPt0 first control point for Bezier curve
            * \param bezierPt1 second control point for Bezier curve
            * \param bezierPt2 third control point for Bezier curve
            * \param numCurvePts number of points to parameterize curve with
            * \param desiredBezierPtToPtDistance desired arclength between discrete points along curve
            * \return vector of arclength parameter values along Bezier curve
            */
            Eigen::VectorXd arclengthParameterizeBezier(const Eigen::Vector2d & bezierPt0, 
                                                        const Eigen::Vector2d & bezierPt1, 
                                                        const Eigen::Vector2d & bezierPt2, 
                                                        const float & numCurvePts,
                                                        float & desiredBezierPtToPtDistance);        

            /**
            * \brief obtain discrete arclength parameterization of Bezier curve
            * \param selectedGap gap through which trajectory will be generated
            * \param gapCurvesPosns 2D positions along gap's arclength-parameterized left and right Bezier curves
            * \param gapCurvesInwardNorms inward point normal vectors along gap's arclength-parameterized left and right Bezier curves
            * \param gapSideAHPFCenters harmonic terms' centers along gap's arclength-parameterized left and right Bezier curves
            * \param allAHPFCenters harmonic terms' centers along gap's arclength-parameterized left and right Bezier curves along with gaps' radially extended origin
            * \param leftGapPtVel velocity of left gap point
            * \param rightGapPtVel velocity of right gap point
            * \param maxRbtVel maximum allowed speed for ego-robot
            * \param leftCurveInitPt initial left gap point
            * \param leftCurveTermPt terminal left gap point
            * \param rightCurveInitPt initial right gap point
            * \param rightCurveTermPt terminal right gap point
            * \param gapGoalTermPt terminal gap goal point
            * \param leftBezierWeight weighting term for left Bezier control points
            * \param rightBezierWeight weighting term for right Bezier control points
            * \param numCurvePts number of points to parameterize left and right curves with
            * \param numLeftRGEPoints number of points to parameterize left curve extension with
            * \param numRightRGEPoints number of points to parameterize right curve extension with
            */
            void buildExtendedBezierCurve(dynamic_gap::Gap * selectedGap, 
                                            Eigen::MatrixXd & gapCurvesPosns,
                                            Eigen::MatrixXd & gapCurvesInwardNorms, 
                                            Eigen::MatrixXd & gapSideAHPFCenters, 
                                            Eigen::MatrixXd & allAHPFCenters,
                                            const Eigen::Vector2d & leftGapPtVel, 
                                            const Eigen::Vector2d & rightGapPtVel, 
                                            const Eigen::Vector2d & maxRbtVel,
                                            const Eigen::Vector2d & leftCurveInitPt, 
                                            const Eigen::Vector2d & leftCurveTermPt, 
                                            const Eigen::Vector2d & rightCurveInitPt, 
                                            const Eigen::Vector2d & rightCurveTermPt, 
                                            const Eigen::Vector2d & gapGoalTermPt, 
                                            float & leftBezierWeight, 
                                            float & rightBezierWeight, 
                                            const float & numCurvePts, 
                                            int & numLeftRGEPoints, 
                                            int & numRightRGEPoints); // const Eigen::Vector2d & initRbtPos

            /**
            * \brief build parameterization for left and right Bezier curve radial extensions
            * \param numRGEPoints number of points to parameterize left curve extension with
            * \param extendedGapOrigin position of radial gap origin extension
            * \param bezierOrigin position of Bezier curve's first control point
            * \param curvePosns 2D curve positions along gap's arclength-parameterized left and right Bezier curves
            * \param curveVels 2D curve velocities along gap's arclength-parameterized left and right Bezier curves
            * \param curveInwardNorms inward point normal vectors along gap's arclength-parameterized left and right Bezier curves
            * \param left identifier boolean for building left curve or right curve 
            */
            void buildExtendedGapOrigin(const int & numRGEPoints,
                                        const Eigen::Vector2d & extendedGapOrigin,
                                        const Eigen::Vector2d & bezierOrigin,
                                        Eigen::MatrixXd & curvePosns,
                                        Eigen::MatrixXd & curveVels,
                                        Eigen::MatrixXd & curveInwardNorms,
                                        const bool & left);

            /**
            * \brief build parameterization for left and right Bezier curves
            * \param numRGEPoints number of points to parameterize left curve extension with
            * \param totalNumCurvePts number of points to parameterize left and right curves + radial extensions with
            * \param arclengthParameters vector of arclength parameter values along Bezier curve
            * \param bezierOrigin first control point for Bezier curve
            * \param bezierInitialPt second control point for Bezier curve
            * \param bezierTerminalPt third control point for Bezier curve
            * \param curvePosns 2D curve positions along gap's arclength-parameterized left and right Bezier curves
            * \param curveVels 2D curve velocities along gap's arclength-parameterized left and right Bezier curves
            * \param curveInwardNorms inward point normal vectors along gap's arclength-parameterized left and right Bezier curves
            * \param left identifier boolean for building left curve or right curve 
            */
            void buildBezierCurve(const int & numRGEPoints,
                                    const int & totalNumCurvePts,
                                    const Eigen::VectorXd & arclengthParameters,
                                    const Eigen::Vector2d & bezierOrigin,
                                    const Eigen::Vector2d & bezierInitialPt,
                                    const Eigen::Vector2d & bezierTerminalPt,
                                    Eigen::MatrixXd & curvePosns,
                                    Eigen::MatrixXd & curveVels,
                                    Eigen::MatrixXd & curveInwardNorms,
                                    const bool & left);

            /**
            * \brief helper function for initializing constraint matrix
            * \param A populated constraint matrix
            * \param N number of repulsive harmonic terms
            * \param Kplus1 number of total optimization variables
            * \param gapCurvesPosns 2D positions along gap's arclength-parameterized left and right Bezier curves
            * \param gapCurvesInwardNorms inward point normal vectors along gap's arclength-parameterized left and right Bezier curves
            * \param allAHPFCenters harmonic terms' centers along gap's arclength-parameterized left and right Bezier curves along with gaps' radially extended origin            
            * \return boolean for if solver was succesfully initialized  
            */
            void setConstraintMatrix(Eigen::MatrixXd &A, 
                                     const int & N, 
                                     const int & Kplus1, 
                                     const Eigen::MatrixXd & gapCurvesPosns, 
                                     const Eigen::MatrixXd & gapCurvesInwardNorms,
                                     const Eigen::MatrixXd & allAHPFCenters);

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
    };
}