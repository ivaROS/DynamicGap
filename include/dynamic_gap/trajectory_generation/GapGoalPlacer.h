#pragma once

#include <ros/ros.h>
#include <math.h>

#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>

namespace dynamic_gap
{
    /**
    * \brief Class responsible for placing goals within gaps
    */
    class GapGoalPlacer
    {
        public:
            /**
            * \brief constructor
            * \param nh node handle, needed for the goal-skew marker publisher
            * \param cfg planner hyperparameter config list
            *
            * NOTE: the signature changed. The node handle is new. Update the call site
            *       in Planner.cpp (currently "new GapGoalPlacer(cfg_)").
            */
            GapGoalPlacer(ros::NodeHandle & nh, const DynamicGapConfig& cfg);

            /**
            * \brief update current scan
            * \param scan incoming scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

            /**
            * \brief clear the goal-skew markers and reset the per-cycle marker index.
            *
            * Call this ONCE at the top of Planner::gapGoalPlacementV2(), before the gap
            * loop. Without it, markers from previous cycles accumulate in RViz.
            */
            void resetGoalSkewMarkers();

            // /**
            // * \brief algorithm for setting gap goal
            // * \param gap queried gap
            // * \param globalPathLocalWaypointRobotFrame global path local waypoint in robot frame
            // * \param globalGoalRobotFrame global goal in robot frame
            // */
            // void setGapGoal(Gap * gap,
            //                 const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame,
            //                 const geometry_msgs::PoseStamped & globalGoalRobotFrame);

            /**
            * \brief algorithm for setting gap goal
            * \param gap queried gap
            * \param globalPathLocalWaypointRobotFrame global path local waypoint in robot frame
            * \param globalGoalRobotFrame global goal in robot frame
            */
            void setGapGoalV2(Gap * gap,
                                const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame,
                                const geometry_msgs::PoseStamped & globalGoalRobotFrame);

            void setGapGoalFromPriorV2(Gap * gap,
                                        Gap * priorGap);

            void setGapGoalFromNextV2(Gap * gap,
                                        Gap * nextGap);

        private:

            /**
            * \brief checking if global path local waypoint lies within gap
            * \param leftPt left gap point
            * \param rightPt right gap point
            * \param globalPathLocalWaypoint local waypoint along global path in robot frame
            * \return boolean for if global path local waypoint lies within gap
            */
            bool checkWaypointVisibility(const Eigen::Vector2f & leftPt,
                                            const Eigen::Vector2f & rightPt,
                                            const Eigen::Vector2f & globalPathLocalWaypoint);
            /**
            * \brief determining what bearing within gap to bias gap goal placement towards
            * \param leftTheta orientation of left gap point
            * \param rightTheta orientation of right gap point
            * \param globalPathLocalWaypointTheta orientation of global path local waypoint
            * \param leftToRightAngle angle swept out from left gap point to right gap point
            * \param leftToWaypointAngle angle swept out from left gap point to global path local waypoint
            * \param rightToWaypointAngle angle swept out from right gap point to global path local waypoint
            * \return biased bearing for gap goal placement
            */
            float setBiasedGapGoalTheta(const float & leftTheta,
                                        const float & rightTheta,
                                        const float & globalPathLocalWaypointTheta,
                                        const float & leftToRightAngle,
                                        const float & leftToWaypointAngle,
                                        const float & rightToWaypointAngle);

            //////////////////////////////////////////////////////////////////////
            //                      GOAL PLACEMENT SKEW                         //
            //////////////////////////////////////////////////////////////////////
            //
            // Moves the gap goal sideways inside the aperture, away from the gap
            // endpoint that is closing faster. A non-holonomic robot cannot move
            // sideways. It must steer, and steering takes time. Starting the turn
            // early gives the robot a non-zero rate of escape before it needs one.
            //
            // The goal position inside the aperture is a single fraction k, swept
            // from the left gap point to the right gap point:
            //
            //      theta(k) = leftTheta - k * leftToRightAngle
            //      range(k) = leftRange + (rightRange - leftRange) * k
            //
            //      k = 0   -> left gap point
            //      k = 0.5 -> gap mid point (Option 1 today)
            //      k = 1   -> right gap point
            //
            // The skew is an additive shift on k.
            //
            //////////////////////////////////////////////////////////////////////

            /**
            * \brief lateral shift of the gap goal, in fraction-of-aperture units
            *
            * c_hat = (leftPt - rightPt).normalized()          (right point to left point)
            *
            *   s_L = -leftGapVel.dot(c_hat)      left endpoint closing speed  (m/s, > 0 = inward)
            *   s_R =  rightGapVel.dot(c_hat)     right endpoint closing speed (m/s, > 0 = inward)
            *
            *   dK  = 0.5 * (s_L - s_R) * T / W
            *
            * (s_L - s_R) equals -2x the lateral drift of the aperture mid point along
            * c_hat. So dK is a first-order prediction of where the aperture centre will
            * be after T seconds, expressed as a fraction of the aperture width. It is not
            * a tuned bias.
            *
            * dK > 0 moves the goal toward the RIGHT gap point.
            *
            * IMPORTANT: this reads getLVelocity() and getRVelocity(), NOT
            * getManipulatedLVelocity(). Estimator::isolateManipGapDynamics() zeroes the
            * velocity for every point where ungap_ is false, which is every ordinary gap
            * point. getManipulatedLVelocity() therefore always returns (0, 0).
            *
            * \param gap queried gap
            * \param closingRateDiffOut diagnostic: (s_L - s_R), m/s
            * \param gapWidthOut diagnostic: manipulated aperture width, m. -1 if unusable.
            * \return the shift dK, already limited to +/- goalSkewMaxDelta_.
            *         Returns 0.0f whenever the skew cannot be computed.
            */
            float computeGoalSkewDelta(Gap * gap,
                                        float & closingRateDiffOut,
                                        float & gapWidthOut) const;

            /**
            * \brief limit the goal fraction so the goal cannot sit on an endpoint
            *
            * GapManipulator::inflateGapSides() has already inset the aperture by
            * r_inscr * inf_ratio, so k in [0, 1] is collision-free to first order. These
            * limits are a margin against the case where inflateGapSides() had to reduce
            * inf_ratio toward 1.0. Do NOT recompute the inflation here. That would
            * count the robot radius twice.
            */
            float clampGoalFraction(const float & k) const;

            /**
            * \brief publish the RViz markers for one gap's skew
            * \param baseGoalPos gap goal before the skew, robot frame
            * \param skewedGoalPos gap goal after the skew, robot frame
            */
            void publishGoalSkewMarkers(Gap * gap,
                                        const Eigen::Vector2f & baseGoalPos,
                                        const Eigen::Vector2f & skewedGoalPos);

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */

            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */

            //////////////////////////////////////////////////////////////////////
            // Goal placement skew parameters.
            //
            // These live here on purpose and NOT in the dynamic_gap config yaml.
            // Those parameters are overwritten by Arena and must not be used.
            //////////////////////////////////////////////////////////////////////

            bool useGoalPlacementSkew_ = true;      /**< master toggle. false reproduces the original behaviour exactly */
            float goalSkewLookaheadTime_ = 1.5f;    /**< T, seconds. The only physically meaningful knob */
            float goalSkewMaxDelta_ = 0.35f;        /**< largest single shift, in fraction units */
            float goalSkewMinFraction_ = 0.15f;     /**< k_min */
            float goalSkewMaxFraction_ = 0.85f;     /**< k_max */

            bool publishGoalSkewMarkers_ = false;    /**< set false for timed runs */
            float goalSkewVelocityMarkerScale_ = 1.0f; /**< metres drawn per m/s of endpoint velocity */

            ros::Publisher goalSkewMarkerPublisher_; /**< RViz topic "goal_placement_skew" */
            int goalSkewMarkerIdx_ = 0;              /**< per-cycle marker index, reset by resetGoalSkewMarkers() */

    };
}
