#pragma once

#include <dynamic_gap/utils/Gap.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dynamic_gap 
{
    /**
    * \brief Class responsible for generating navigable gap region through which we will generate
    *        the local gap trajectories
    */    
    class NavigableGapGenerator 
    {
        public:
            NavigableGapGenerator(const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; };

            void generateNavigableGap(dynamic_gap::Gap * gap);

        private:

            float arclengthParameterizeBoundary(dynamic_gap::Gap * gap,
                                                Eigen::VectorXd & left_indices,
                                                Eigen::VectorXd & arc_indices,
                                                Eigen::VectorXd & right_indices,
                                                const Eigen::Vector2d & gapGoalTermPt);

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
            float approximateBezierArclength(const Eigen::Vector2d & bezierPt0, 
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
                                                        const float & desiredBezierPtToPtDistance,
                                                        const float & bezierPtToPtDistanceThresh,
                                                        float & prior_snippet_arclength,
                                                        const bool & left);        

            Eigen::VectorXd arclengthParameterizeArc(const float & leftBezierOriginTheta,
                                                        const float & rightBezierOriginTheta,
                                                        const float & d_safe,
                                                        const float & desiredBezierPtToPtDistance,
                                                        float & prior_snippet_arclength);

            // /**
            // * \brief obtain discrete arclength parameterization of Bezier curve
            // * \param gap gap through which trajectory will be generated
            // * \param gapCurvesPosns 2D positions along gap's arclength-parameterized left and right Bezier curves
            // * \param gapCurvesInwardNorms inward point normal vectors along gap's arclength-parameterized left and right Bezier curves
            // * \param gapSideAHPFCenters harmonic terms' centers along gap's arclength-parameterized left and right Bezier curves
            // * \param allAHPFCenters harmonic terms' centers along gap's arclength-parameterized left and right Bezier curves along with gaps' radially extended origin
            // * \param leftGapPtVel velocity of left gap point
            // * \param rightGapPtVel velocity of right gap point
            // * \param maxRbtVel maximum allowed speed for ego-robot
            // * \param leftCurveInitPt initial left gap point
            // * \param leftCurveTermPt terminal left gap point
            // * \param rightCurveInitPt initial right gap point
            // * \param rightCurveTermPt terminal right gap point
            // * \param gapGoalTermPt terminal gap goal point
            // * \param numCurvePts number of points to parameterize left and right curves with
            // */
            // void buildExtendedBezierCurve(dynamic_gap::Gap * gap, 
            //                                 // Eigen::MatrixXd & gapCurvesPosns,
            //                                 // Eigen::MatrixXd & gapCurvesInwardNorms, 
            //                                 // Eigen::MatrixXd & gapSideAHPFCenters, Eigen::MatrixXd & allAHPFCenters,
            //                                 const Eigen::Vector2d & leftGapPtVel, 
            //                                 const Eigen::Vector2d & rightGapPtVel, 
            //                                 const Eigen::Vector2d & maxRbtVel,
            //                                 const Eigen::Vector2d & leftCurveInitPt, 
            //                                 const Eigen::Vector2d & leftCurveTermPt, 
            //                                 const Eigen::Vector2d & rightCurveInitPt, 
            //                                 const Eigen::Vector2d & rightCurveTermPt, 
            //                                 const Eigen::Vector2d & gapGoalTermPt, 
            //                                 const float & numCurvePts); // const Eigen::Vector2d & initRbtPos

            // /**
            // * \brief build parameterization for left and right Bezier curve radial extensions
            // * \param numRGEPoints number of points to parameterize left curve extension with
            // * \param extendedGapOrigin position of radial gap origin extension
            // * \param bezierOrigin position of Bezier curve's first control point
            // * \param curvePosns 2D curve positions along gap's arclength-parameterized left and right Bezier curves
            // * \param curveVels 2D curve velocities along gap's arclength-parameterized left and right Bezier curves
            // * \param curveInwardNorms inward point normal vectors along gap's arclength-parameterized left and right Bezier curves
            // * \param left identifier boolean for building left curve or right curve 
            // */
            // void buildExtendedGapOrigin(const int & numRGEPoints,
            //                             const Eigen::Vector2d & extendedGapOrigin,
            //                             const Eigen::Vector2d & bezierOrigin,
            //                             Eigen::MatrixXd & curvePosns,
            //                             Eigen::MatrixXd & curveVels,
            //                             Eigen::MatrixXd & curveInwardNorms,
            //                             const bool & left);

            void buildArcCurve(const Eigen::VectorXd & arclengthParameters,
                                                const float & leftBezierOriginTheta,
                                                const float & rightBezierOriginTheta,
                                                const float & d_safe,
                                                Eigen::MatrixXd & curvePosns,
                                                // Eigen::MatrixXd & curveVels,
                                                Eigen::MatrixXd & curveInwardNorms);

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
            void buildBezierCurve(const Eigen::VectorXd & arclengthParameters,
                                                  const Eigen::Vector2d & bezierOrigin,
                                                  const Eigen::Vector2d & bezierInitialPt,
                                                  const Eigen::Vector2d & bezierTerminalPt,
                                                  Eigen::MatrixXd & curvePosns,
                                                //   Eigen::MatrixXd & curveVels,
                                                  Eigen::MatrixXd & curveInwardNorms,
                                                  const bool & left);        

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
    };
}