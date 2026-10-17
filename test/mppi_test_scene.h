#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <unordered_map>

#include <dynamic_gap/gap_detection/GapDetector.h>
#include <dynamic_gap/scan_processing/DynamicScanPropagator.h>
#include <dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h>
#include <dynamic_gap/utils/bezier_utils.h>
#include <visualization_msgs/MarkerArray.h>

namespace dynamic_gap_test
{
// A deterministic laser scene with three forward passages and two moving boxes.
// Rectangles are used only to synthesize sensor input. All trajectory costs and
// future obstacle scans come from the production Dynamic Gap implementation.
struct Box
{
    float x, y, half_x, half_y;
    Eigen::Vector2f velocity;
};

struct Scene
{
    dynamic_gap::DynamicGapConfig cfg;
    boost::shared_ptr<sensor_msgs::LaserScan> scan;
    std::vector<dynamic_gap::Gap*> gaps;
    std::vector<sensor_msgs::LaserScan> future_scans;
    std::vector<geometry_msgs::PoseStamped> plan;
    std::unordered_map<int, Eigen::Vector2f> left_velocities, right_velocities;
    std::vector<Box> boxes{{2.1f, 0.95f, 0.22f, 0.23f, {0.0f, 0.12f}},
                           {2.1f, -0.95f, 0.22f, 0.23f, {0.0f, -0.12f}}};

    explicit Scene(ros::NodeHandle& nh)
    {
        cfg.sensor_frame_id = "mppi_demo";
        cfg.robot_frame_id = cfg.sensor_frame_id;
        cfg.odom_frame_id = cfg.sensor_frame_id;
        cfg.rbt.r_inscr = 0.2f;
        cfg.rbt.vx_absmax = 1.0f;
        cfg.rbt.vang_absmax = 1.6f;
        cfg.traj.integrate_stept = 0.2f;
        cfg.traj.integrate_maxt = 4.0f;
        cfg.traj.inf_ratio = 1.3f;
        cfg.traj.max_pose_to_scan_dist = 0.8f;
        cfg.traj.Q = 3.0f;
        cfg.traj.Q_f = 2.0f;
        cfg.traj.w_path = 0.05f;
        cfg.traj.w_speed = 0.05f;
        cfg.ctrl.ctrl_ahead_pose = 2;

        scan.reset(new sensor_msgs::LaserScan());
        scan->header.frame_id = cfg.sensor_frame_id;
        scan->header.stamp = ros::Time::now();
        scan->angle_min = dynamic_gap::idx2theta(0);
        scan->angle_increment = 2.0f * M_PI / 511.0f;
        scan->angle_max = dynamic_gap::idx2theta(511);
        scan->range_min = 0.03f;
        scan->range_max = 9.0f;
        scan->ranges.resize(512);
        scan->intensities.resize(512, 1.0f);
        for (std::size_t i = 0; i < scan->ranges.size(); ++i)
        {
            const float angle = dynamic_gap::idx2theta(i);
            const Eigen::Vector2f direction(std::cos(angle), std::sin(angle));
            float range = std::min(8.0f / std::max(1e-6f, std::abs(direction.x())),
                                   5.0f / std::max(1e-6f, std::abs(direction.y())));
            for (const Box& box : boxes)
            {
                float enter = 0.0f, leave = scan->range_max;
                for (int axis = 0; axis < 2; ++axis)
                {
                    const float center = axis == 0 ? box.x : box.y;
                    const float half = axis == 0 ? box.half_x : box.half_y;
                    if (std::abs(direction[axis]) < 1e-6f)
                    {
                        if (center - half > 0 || center + half < 0) leave = -1;
                    }
                    else
                    {
                        float lo = (center - half) / direction[axis];
                        float hi = (center + half) / direction[axis];
                        if (lo > hi) std::swap(lo, hi);
                        enter = std::max(enter, lo);
                        leave = std::min(leave, hi);
                    }
                }
                if (enter <= leave) range = std::min(range, enter);
            }
            scan->ranges[i] = std::min(range, scan->range_max);
        }
        cfg.updateParamFromScan(scan);
        dynamic_gap::GapDetector detector(cfg);
        gaps = detector.gapDetection(scan);
        if (gaps.size() < 3) throw std::runtime_error("Scene did not produce multiple raw gaps");
        int model_id = 0;
        for (dynamic_gap::Gap* gap : gaps)
        {
            for (bool left : {false, true})
            {
                auto* point = left ? gap->getLeftGapPt() : gap->getRightGapPt();
                const float angle = dynamic_gap::idx2theta(point->getOrigIdx());
                const Eigen::Vector2f position(point->getOrigRange() * std::cos(angle),
                                               point->getOrigRange() * std::sin(angle));
                geometry_msgs::TwistStamped zero;
                dynamic_gap::EstimationParameters parameters;
                point->getModel()->initialize(left ? "left" : "right", model_id++,
                    position.x(), position.y(), scan->header.stamp, zero, zero, parameters);
                Eigen::Vector2f velocity = Eigen::Vector2f::Zero();
                for (const Box& box : boxes)
                    if ((position - Eigen::Vector2f(box.x, box.y)).norm() < 0.45f)
                        velocity = box.velocity;
                point->getModel()->setRelativeVelocityEstimate(velocity);
                (left ? left_velocities : right_velocities)[point->getModel()->getID()] = velocity;
            }
        }
        dynamic_gap::DynamicScanPropagator propagator(nh, cfg);
        propagator.updateEgoCircle(scan);
        future_scans = propagator.propagateCurrentLaserScan(gaps);
        for (int i = 0; i <= 20; ++i)
        {
            geometry_msgs::PoseStamped point;
            point.header = scan->header;
            point.pose.position.x = 0.2 * i;
            point.pose.orientation.w = 1.0;
            plan.push_back(point);
        }
    }

    ~Scene() { for (auto* gap : gaps) delete gap; }

    void initializeEvaluator(dynamic_gap::TrajectoryEvaluator& evaluator)
    {
        evaluator.leftVelDictPtr_ = &left_velocities;
        evaluator.rightVelDictPtr_ = &right_velocities;
        evaluator.leftPosDictPtr_ = nullptr;
        evaluator.rightPosDictPtr_ = nullptr;
        evaluator.latestGapLeftVelPtr_ = nullptr;
        evaluator.latestGapRightVelPtr_ = nullptr;
        evaluator.updateEgoCircle(scan);
        geometry_msgs::TransformStamped identity;
        identity.header.frame_id = cfg.sensor_frame_id;
        identity.child_frame_id = cfg.sensor_frame_id;
        identity.transform.rotation.w = 1;
        evaluator.transformGlobalPathLocalWaypointToRbtFrame(plan.back(), identity);
    }

    std::vector<Eigen::Vector2f> curve(float y) const
    {
        std::vector<Eigen::Vector2f> result;
        for (int i = 0; i <= 30; ++i)
            result.push_back(bezier(Eigen::Vector2f::Zero(), Eigen::Vector2f(0.8f, 0),
                                    Eigen::Vector2f(4.0f, y), float(i) / 30));
        return result;
    }

    visualization_msgs::MarkerArray markers() const
    {
        visualization_msgs::MarkerArray output;
        for (std::size_t i = 0; i < boxes.size(); ++i)
        {
            visualization_msgs::Marker box;
            box.header = scan->header;
            box.ns = "moving_obstacles";
            box.id = i;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.pose.orientation.w = 1;
            box.pose.position.x = boxes[i].x;
            box.pose.position.y = boxes[i].y;
            box.pose.position.z = 0.2;
            box.scale.x = 2 * boxes[i].half_x;
            box.scale.y = 2 * boxes[i].half_y;
            box.scale.z = 0.4;
            box.color.r = 0.65;
            box.color.g = 0.65;
            box.color.b = 0.7;
            box.color.a = 1;
            output.markers.push_back(box);
        }
        return output;
    }
};
}  // namespace dynamic_gap_test
