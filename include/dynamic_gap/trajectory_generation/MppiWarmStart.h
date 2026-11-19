#pragma once

#include <dynamic_gap/trajectory_generation/MppiControl.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace dynamic_gap
{
// Candidate control history, keyed by ordered endpoint model IDs, never tube index.
// Caller rolls the returned seed out at the CURRENT robot pose and collision checks it.
class MppiWarmStartCache
{
public:
    using Key = std::pair<int, int>;
    struct Seed
    {
        MppiControls controls;
        bool reused = false;
        double elapsed = 0;
        std::string reason = "no_history";
    };

    Seed seed(Key key, const MppiControls& fresh, double now, double dt,
              double max_age, double angular_limit) const
    {
        Seed result;
        result.controls = fresh;
        if (key.first < 0 || key.second < 0) { result.reason = "unknown_ids"; return result; }
        const auto found = history_.find(key);
        if (found == history_.end()) return result;
        const auto& old = found->second;
        result.elapsed = now - old.stamp;
        if (!std::isfinite(result.elapsed) || result.elapsed <= 0 ||
            result.elapsed > max_age || !std::isfinite(dt) || dt <= 0 ||
            old.dt != dt || old.controls.size() != fresh.size() ||
            result.elapsed >= dt * fresh.size())
        { result.reason = "stale_or_timing_changed"; return result; }
        // Average the old piecewise-constant steering over each new dt interval.
        // E.g. 0.1 s elapsed with dt=0.5 uses 80% old[t] + 20% old[t+1].
        // The uncovered tail uses the corresponding fresh reference command.
        const double shift = result.elapsed / dt;
        const std::size_t whole = static_cast<std::size_t>(std::floor(shift));
        const double fraction = shift - whole;
        for (std::size_t t = 0; t < fresh.size(); ++t)
        {
            const std::size_t index = t + whole;
            const double a = index < old.controls.size() ? old.controls[index].angular.z : fresh[t].angular.z;
            const double b = index + 1 < old.controls.size() ? old.controls[index + 1].angular.z : fresh[t].angular.z;
            result.controls[t].angular.z = std::clamp((1 - fraction) * a + fraction * b,
                                                     -angular_limit, angular_limit);
            // Fresh linear speed preserves the native per-gap distance horizon.
        }
        result.reused = true;
        result.reason = "shifted_previous";
        return result;
    }

    void remember(Key key, const MppiControls& controls, double stamp, double dt)
    {
        if (key.first < 0 || key.second < 0 || controls.empty() || !std::isfinite(stamp) ||
            !std::isfinite(dt) || dt <= 0 ||
            std::any_of(controls.begin(), controls.end(), [](const geometry_msgs::Twist& u) {
                return !std::isfinite(u.linear.x) || !std::isfinite(u.angular.z);
            }))
        { history_.erase(key); return; }
        history_[key] = {controls, stamp, dt};
    }
    void erase(Key key) { history_.erase(key); }
    void clear() { history_.clear(); }
    void retain(const std::set<Key>& active)
    {
        for (auto it = history_.begin(); it != history_.end(); )
            if (!active.count(it->first)) it = history_.erase(it);
            else ++it;
    }
private:
    struct Entry { MppiControls controls; double stamp; double dt; };
    std::map<Key, Entry> history_;
};
}
