#pragma once

#include <xmlrpcpp/XmlRpcValue.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <manual_gap_msgs/ManualGapSample.h>

#include <algorithm>
#include <functional>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace dynamic_gap
{
    /**
    * \brief Outcome of reading the /man_gap parameter
    */
    enum class ManualModeParam { BOOL, STRING, MISSING, INVALID };

    /**
    * \brief Interpret the /man_gap parameter. Manual selection is off unless the
    * parameter clearly says true.
    * \param hasParam whether the parameter exists
    * \param value parameter value (ignored if hasParam is false)
    * \param manualMode resulting mode
    * \return how the value was interpreted
    */
    inline ManualModeParam parseManualModeParam(const bool & hasParam,
                                                XmlRpc::XmlRpcValue value,
                                                bool & manualMode)
    {
        manualMode = false;

        if (!hasParam)
            return ManualModeParam::MISSING;

        if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
        {
            manualMode = static_cast<bool>(value);
            return ManualModeParam::BOOL;
        }

        if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
        {
            std::string text = static_cast<std::string>(value);
            std::transform(text.begin(), text.end(), text.begin(),
                           [](unsigned char c) { return std::tolower(c); });

            if (text == "true" || text == "false")
            {
                manualMode = (text == "true");
                return ManualModeParam::STRING;
            }
        }

        return ManualModeParam::INVALID;
    }

    /**
    * \brief Whether tracking of a click-to-move trajectory has ended
    * \param distToTarget distance from robot to the clicked point
    * \param trackingTime time spent tracking the trajectory
    * \param estimatedTime estimated completion time (distance / max speed)
    * \param tolerance distance at which the target counts as reached
    * \return true once the target is reached, or after twice the estimated time
    */
    inline bool isCreatedTrajDone(const float & distToTarget,
                                  const double & trackingTime,
                                  const double & estimatedTime,
                                  const float & tolerance)
    {
        return distToTarget < tolerance || trackingTime > 2.0 * estimatedTime;
    }

    /**
    * \brief Runs a callable when the scope ends (used to publish the per-tick
    * sample on every return path of the planning loop)
    */
    template <typename F>
    class ScopeExit
    {
        public:
            explicit ScopeExit(F f) : f_(std::move(f)) {}
            ~ScopeExit() { f_(); }
            ScopeExit(const ScopeExit &) = delete;
            ScopeExit & operator=(const ScopeExit &) = delete;
        private:
            F f_;
    };

    /**
    * \brief Whether a gap can be recognised at a later tick: both its gap points
    * have models (a missing model has id -1, as has "no gap followed")
    */
    inline bool hasModelIDs(const int & leftModelID, const int & rightModelID)
    {
        return leftModelID != -1 && rightModelID != -1;
    }

    /**
    * \brief Numbers shown to the operator: 0, 1, 2, ... over the shown
    * candidates, -1 for the others
    * \param shown per candidate, whether it is drawn as selectable
    * \return display id per candidate
    */
    inline std::vector<int> denseDisplayIds(const std::vector<bool> & shown)
    {
        std::vector<int> ids(shown.size(), -1);
        int next = 0;
        for (size_t i = 0; i < shown.size(); i++)
        {
            if (shown[i])
                ids[i] = next++;
        }
        return ids;
    }

    /**
    * \brief Candidate whose terminal point is nearest the cursor
    * \param terminals terminal point per candidate (x, y)
    * \param feasible per candidate, whether it can be selected
    * \param cursorX cursor x, same frame as terminals
    * \param cursorY cursor y, same frame as terminals
    * \return index of the nearest feasible candidate, -1 if none; ties go to the lower index
    */
    inline int nearestCandidate(const std::vector<std::pair<float, float>> & terminals,
                                const std::vector<bool> & feasible,
                                const float & cursorX,
                                const float & cursorY)
    {
        int nearest = -1;
        float minDist = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < terminals.size() && i < feasible.size(); i++)
        {
            if (!feasible[i])
                continue;

            float dist = std::hypot(terminals[i].first - cursorX, terminals[i].second - cursorY);
            if (dist < minDist)
            {
                minDist = dist;
                nearest = i;
            }
        }
        return nearest;
    }

    /**
    * \brief Candidates whose terminal point lies in the cursor's direction from the robot
    * \param terminals terminal point per candidate (x, y)
    * \param feasible per candidate, whether it can be selected
    * \param rbtX robot x, same frame as terminals
    * \param rbtY robot y, same frame as terminals
    * \param cursorX cursor x, same frame as terminals
    * \param cursorY cursor y, same frame as terminals
    * \param maxBearingDiff largest angle (rad) between the robot-to-cursor and robot-to-terminal bearings
    * \return per candidate, feasible and within maxBearingDiff of the cursor bearing
    */
    inline std::vector<bool> inCursorDirection(const std::vector<std::pair<float, float>> & terminals,
                                               const std::vector<bool> & feasible,
                                               const float & rbtX, const float & rbtY,
                                               const float & cursorX, const float & cursorY,
                                               const float & maxBearingDiff)
    {
        std::vector<bool> inDirection(feasible.size(), false);
        float cursorBearing = std::atan2(cursorY - rbtY, cursorX - rbtX);
        for (size_t i = 0; i < terminals.size() && i < feasible.size(); i++)
        {
            if (!feasible[i])
                continue;

            float terminalBearing = std::atan2(terminals[i].second - rbtY, terminals[i].first - rbtX);
            float diff = std::atan2(std::sin(cursorBearing - terminalBearing), std::cos(cursorBearing - terminalBearing));
            inDirection[i] = std::abs(diff) <= maxBearingDiff;
        }
        return inDirection;
    }

    /**
    * \brief Keep the current candidate unless the nearest one is clearly nearer the cursor
    * \param terminals terminal point per candidate (x, y)
    * \param cursorX cursor x, same frame as terminals
    * \param cursorY cursor y, same frame as terminals
    * \param nearest index of the nearest selectable candidate, -1 if none
    * \param current index of the currently followed candidate, -1 if it is not selectable this tick
    * \param switchRatio the nearest candidate replaces the current one only if
    *        dist(nearest) < switchRatio * dist(current)
    * \return index of the candidate to follow, -1 if none
    */
    inline int holdCurrentCandidate(const std::vector<std::pair<float, float>> & terminals,
                                    const float & cursorX,
                                    const float & cursorY,
                                    const int & nearest,
                                    const int & current,
                                    const float & switchRatio)
    {
        if (current < 0 || nearest < 0 || nearest == current)
            return nearest;

        float nearestDist = std::hypot(terminals[nearest].first - cursorX, terminals[nearest].second - cursorY);
        float currentDist = std::hypot(terminals[current].first - cursorX, terminals[current].second - cursorY);
        return nearestDist < switchRatio * currentDist ? nearest : current;
    }

    /**
    * \brief Whether the last cursor message still selects a gap. The last known
    * position stays valid however old it is: a still mouse, a pointer outside
    * the panel or an RViz that stopped publishing keep the selection. Only the
    * tool being switched off ends it.
    * \param toolActive tool_active of the last cursor message
    */
    inline bool isCursorHeld(const bool & toolActive)
    {
        return toolActive;
    }

    /**
    * \brief Whether a tick is close to an episode reset
    * \param resetBeginId id of the last /scenario_reset_begin (-1 if none)
    * \param episodeId id of the last /scenario_reset (-1 if none)
    * \param secondsSinceReset wall-clock time since the last /scenario_reset (negative if none)
    * \param window seconds after a reset that still count
    */
    inline bool isInResetWindow(const int & resetBeginId, const int & episodeId,
                                const double & secondsSinceReset, const double & window)
    {
        if (resetBeginId > episodeId)
            return true;  // a reset is in progress
        return secondsSinceReset >= 0.0 && secondsSinceReset < window;
    }

    /**
    * \brief Sample at the start of a planning tick: planned, nothing selected.
    * Ticks that end before selection (not ready, colliding, no gaps) keep
    * selected_idx = -1 instead of the message default 0.
    */
    inline manual_gap_msgs::ManualGapSample newManualSample()
    {
        manual_gap_msgs::ManualGapSample sample;
        sample.status = sample.STATUS_PLANNED;
        sample.source = sample.SOURCE_NONE;
        sample.selected_idx = -1;
        sample.selected_left_model_id = -1;
        sample.selected_right_model_id = -1;
        sample.dg_pick_idx = -1;
        return sample;
    }

    /**
    * \brief Mean of a trajectory's pose-wise costs (the clearance term of pickTraj)
    */
    /**
    * \brief Neighbours of the scene edges, which are in scan order, two per raw gap
    * (right point, then left point): the other point of the gap, and toward the
    * other side the next edge, the other end of the obstacle between them. The
    * scan covers the full circle, so the last edge's obstacle reaches the first.
    */
    inline void linkEdges(std::vector<manual_gap_msgs::ManualGapEdge> & edges)
    {
        const int n = edges.size();
        for (int i = 0; i < n; i++)
        {
            edges[i].gap_neighbor = (i % 2 == 0) ? i + 1 : i - 1;
            edges[i].obstacle_neighbor = (i % 2 == 0) ? (i - 1 + n) % n : (i + 1) % n;
        }
    }

    inline float meanPoseCost(const std::vector<float> & poseCosts)
    {
        if (poseCosts.empty())
            return std::numeric_limits<float>::quiet_NaN();
        float sum = 0.0f;
        for (const float & cost : poseCosts)
            sum += cost;
        return sum / poseCosts.size();
    }

    /**
    * \brief DynamicGap's cost of one trajectory, as pickTraj ranks it: terminal
    * cost plus mean pose-wise cost, infinite for a path of fewer than 2 poses
    * \param numPoses number of poses of the trajectory
    */
    inline float dgTrajCost(const std::vector<float> & poseCosts, const float & terminalCost, const size_t & numPoses)
    {
        if (numPoses < 2)
            return std::numeric_limits<float>::infinity();
        return terminalCost + meanPoseCost(poseCosts);
    }

    /**
    * \brief Every step-th point of a path, always including the last one
    */
    inline std::vector<geometry_msgs::Point> downsamplePath(const geometry_msgs::PoseArray & path, const int & step)
    {
        std::vector<geometry_msgs::Point> points;
        const int n = path.poses.size();
        for (int i = 0; i < n; i += std::max(step, 1))
            points.push_back(path.poses[i].position);
        if (n > 0 && (n - 1) % std::max(step, 1) != 0)
            points.push_back(path.poses[n - 1].position);
        return points;
    }
}

