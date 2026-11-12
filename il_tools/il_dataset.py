"""
Training set from recorded sessions (docs/IL_DATA_COLLECTION_PLAN.md, 1.7).

One example pairs the observation of tick t - lag with the gap the operator
had selected at tick t, found in tick t - lag by its identity (left and right
model ids). Only usable label ticks are used (plan 1.4); thresholds and the
lag come from the pilot.
"""

import gzip
import hashlib
import json
import math
import os

import yaml

import il_bags


DEFAULTS = {
    "lag_s": 0.0,                # reaction lag, wall clock seconds
    "max_cursor_dist": math.inf,  # m, cursor to the selected terminal pose
    "max_bearing_diff": math.inf, # rad, |bearing(cursor) - bearing(terminal)|
    "min_tie_margin": 0.0,       # m, second-nearest minus nearest cursor distance
    "max_lag_error_s": 0.25,     # wall s, observation tick at most this much older than t - lag
    "drop_collision_episodes": True,
    "splits": {"train": 0.8, "val": 0.1, "test": 0.1},
    "seed": "il_gap_selection",
}


def episode_split(session, episode, splits, seed):
    """Split by episode, deterministic: the same episode always lands in the same split."""
    digest = hashlib.sha256("{}|{}|{}".format(seed, session, episode).encode()).hexdigest()
    u = int(digest[:12], 16) / float(16 ** 12)
    acc = 0.0
    for name, share in splits.items():
        acc += share
        if u < acc:
            return name
    return list(splits)[-1]


def tie_margin(sample):
    dists = sorted(c.cursor_dist for c in sample.candidates if c.feasible and not math.isnan(c.cursor_dist))
    return dists[1] - dists[0] if len(dists) > 1 else math.inf


def passes_thresholds(sample, opts):
    c = sample.candidates[sample.selected_idx]
    return (c.cursor_dist <= opts["max_cursor_dist"]
            and abs(c.cursor_bearing_diff) <= opts["max_bearing_diff"]
            and tie_margin(sample) >= opts["min_tie_margin"])


def observation_tick(ticks, index, lag_s):
    """Latest tick of the same episode at least lag_s (wall clock) before ticks[index]."""
    target = il_bags.stamp(ticks[index].wall_stamp) - lag_s
    for j in range(index, -1, -1):
        # stamps are whole nanoseconds; allow for the rounding
        if il_bags.stamp(ticks[j].wall_stamp) <= target + 1e-6:
            return j
    return None


def is_valid_observation(tick):
    """A planned tick outside a reset (the planner's own reliability flags)."""
    return tick.status == il_bags.STATUS_PLANNED and not tick.in_reset_window


def yaw(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_robot_frame(x, y, pose):
    """Odom point (x, y) in the frame of the robot at pose."""
    c, s = math.cos(yaw(pose)), math.sin(yaw(pose))
    dx, dy = x - pose.position.x, y - pose.position.y
    return [c * dx + s * dy, -s * dx + c * dy]


def previous_selection(ticks, index):
    """Identity of the gap followed on the tick before ticks[index], None if none."""
    if index == 0:
        return None
    prev = ticks[index - 1]
    if prev.status != il_bags.STATUS_PLANNED or prev.selected_idx < 0:
        return None
    return il_bags.selected_identity(prev)


def candidate_features(c, pose, followed):
    """followed: identity of the gap followed on the previous tick (None if none).
    The dg_* fields are None in sessions recorded before schema 2."""
    nan = float("nan")
    return {
        "traj_idx": c.traj_idx,
        "left_model_id": c.left_model_id, "right_model_id": c.right_model_id,
        "feasible": c.feasible,
        "shown": c.shown,
        "tube_len": c.tube_len,
        "is_current": followed is not None and il_bags.identity(c) == followed,
        "left_range": c.left_range, "left_theta": c.left_theta,
        "right_range": c.right_range, "right_theta": c.right_theta,
        "bearing": c.bearing,
        # scan rays of the gap points (schema 4, None before): the scene edges they are
        "left_scan_idx": getattr(c, "left_scan_idx", None), "right_scan_idx": getattr(c, "right_scan_idx", None),
        "left_model": [c.left_model_pos.x, c.left_model_pos.y, c.left_model_vel.x, c.left_model_vel.y],
        "right_model": [c.right_model_pos.x, c.right_model_pos.y, c.right_model_vel.x, c.right_model_vel.y],
        "goal_odom": [c.goal_pos.x, c.goal_pos.y],
        "terminal_odom": [c.terminal_pose.position.x, c.terminal_pose.position.y],
        "path_odom": [[p.x, p.y] for p in c.path],
        "goal_rbt": to_robot_frame(c.goal_pos.x, c.goal_pos.y, pose),
        "terminal_rbt": to_robot_frame(c.terminal_pose.position.x, c.terminal_pose.position.y, pose) if c.feasible else [nan, nan],
        "path_rbt": [to_robot_frame(p.x, p.y, pose) for p in c.path],
        "dg_cost": getattr(c, "dg_cost", None),
        "dg_terminal_cost": getattr(c, "dg_terminal_cost", None),
        "dg_clearance_cost": getattr(c, "dg_clearance_cost", None),
        # the learned scorer's score (schema 3, NaN unless it ran); None before
        "model_score": getattr(c, "model_score", None),
    }


def edge_features(e):
    """One scene edge (schema 4); model_vel is relative to the robot, as recorded."""
    return {
        "scan_idx": e.scan_idx, "model_id": e.model_id, "range": e.range, "theta": e.theta,
        "model_pos": [e.model_pos.x, e.model_pos.y], "model_vel": [e.model_vel.x, e.model_vel.y],
        "gap_neighbor": e.gap_neighbor, "obstacle_neighbor": e.obstacle_neighbor,
    }


def example(session, obs, label_tick, label_idx, followed=None):
    return {
        "session": session,
        "episode": obs.episode_id,
        "obs_tick": obs.tick_seq,
        "label_tick": label_tick.tick_seq,
        "lag_wall_s": round(il_bags.stamp(label_tick.wall_stamp) - il_bags.stamp(obs.wall_stamp), 3),
        "scan": {"angle_min": obs.scan.angle_min, "angle_increment": obs.scan.angle_increment,
                 "range_max": obs.scan.range_max, "ranges": list(obs.scan.ranges)},
        "robot_vel": [obs.robot_vel.linear.x, obs.robot_vel.linear.y, obs.robot_vel.angular.z],
        "robot_pose_odom": [obs.robot_pose_odom.position.x, obs.robot_pose_odom.position.y],
        "robot_yaw_odom": yaw(obs.robot_pose_odom),
        "goal": [obs.goal_robot_frame.x, obs.goal_robot_frame.y],
        "local_waypoint": [obs.local_waypoint_robot_frame.x, obs.local_waypoint_robot_frame.y],
        "candidates": [candidate_features(c, obs.robot_pose_odom, followed) for c in obs.candidates],
        # every edge of the scene (schema 4); empty before
        "edges": [edge_features(e) for e in getattr(obs, "edges", [])],
        "schema_version": obs.schema_version,
        # DynamicGap's own pick (position in candidates); None before schema 2
        "dg_pick": getattr(obs, "dg_pick_idx", None),
        "label": label_idx,
    }


def load_set(path):
    """Episode set file (e.g. il_data/sets/*.yaml): {session dir: {episode id: (from_tick, to_tick)}}.
    Sessions are relative to the file's root; episodes are grouped under any keys. An
    entry's from_tick (optional, None if absent) drops the episode's ticks before that
    tick_seq, as labels and as observations (e.g. a collision right after the spawn); its
    to_tick (optional) drops the label ticks after that tick_seq (e.g. the lead-up to a
    collision near the end)."""
    with open(path) as f:
        spec = yaml.safe_load(f)
    root = os.path.expanduser(spec.get("root", os.path.dirname(os.path.abspath(path))))
    groups = spec["episodes"].values() if isinstance(spec["episodes"], dict) else [spec["episodes"]]
    selection = {}
    for group in groups:
        for entry in group:
            from_tick, to_tick = entry.get("from_tick"), entry.get("to_tick")
            selection.setdefault(os.path.join(root, entry["session"]), {})[int(entry["episode"])] = (
                None if from_tick is None else int(from_tick), None if to_tick is None else int(to_tick))
    return selection


def build_examples(session, samples, outcomes, opts, episodes=None):
    """Examples of one session and the counts of what was dropped. With
    episodes (ids, or {id: (from_tick, to_tick)} as load_set gives) only those are used, and
    they are kept even if they collided; ticks before an episode's from_tick and label
    ticks after its to_tick are cut."""
    counts = {"usable": 0, "not_in_set": 0, "cut": 0, "threshold": 0, "collision_episode": 0, "no_obs_tick": 0,
              "obs_too_old": 0, "obs_invalid": 0, "identity_lost": 0, "duplicate": 0, "kept": 0}
    cuts = episodes if isinstance(episodes, dict) else {}
    by_episode = {}
    for s in samples:
        if s.episode_id >= 0:
            by_episode.setdefault(s.episode_id, []).append(s)

    examples, seen = [], set()
    for episode, ticks in sorted(by_episode.items()):
        collided = outcomes.get(episode, {}).get("collision", False)
        for i, tick in enumerate(ticks):
            if not il_bags.is_usable_label(tick):
                continue
            counts["usable"] += 1
            if episodes is not None and episode not in episodes:
                counts["not_in_set"] += 1
                continue
            from_tick, to_tick = cuts.get(episode) or (None, None)
            if (from_tick is not None and tick.tick_seq < from_tick) or \
                    (to_tick is not None and tick.tick_seq > to_tick):
                counts["cut"] += 1
                continue
            if not passes_thresholds(tick, opts):
                counts["threshold"] += 1
                continue
            if collided and opts["drop_collision_episodes"] and episodes is None:
                counts["collision_episode"] += 1
                continue
            j = observation_tick(ticks, i, opts["lag_s"])
            if j is None:
                counts["no_obs_tick"] += 1
                continue
            obs = ticks[j]
            if from_tick is not None and obs.tick_seq < from_tick:
                counts["cut"] += 1
                continue
            # after a gap in the samples the latest earlier tick can be far older than the lag
            lag_error = il_bags.stamp(tick.wall_stamp) - opts["lag_s"] - il_bags.stamp(obs.wall_stamp)
            if lag_error > opts["max_lag_error_s"]:
                counts["obs_too_old"] += 1
                continue
            if not is_valid_observation(obs):
                counts["obs_invalid"] += 1
                continue
            target = il_bags.selected_identity(tick)
            if j == i:
                # no lag: the label is the recorded choice, also for a gap without identity
                matches = [tick.selected_idx]
            else:
                matches = [k for k, c in enumerate(obs.candidates)
                           if target is not None and c.feasible and il_bags.identity(c) == target]
            if not matches:
                counts["identity_lost"] += 1
                continue
            # two label ticks can map to the same observation and label
            key = (obs.tick_seq, matches[0])
            if key in seen:
                counts["duplicate"] += 1
                continue
            seen.add(key)
            # the gap followed before is unknown when that tick was cut
            followed = None if from_tick is not None and j > 0 and ticks[j - 1].tick_seq < from_tick \
                else previous_selection(ticks, j)
            ex = example(session, obs, tick, matches[0], followed)
            ex["collision_episode"] = collided
            examples.append(ex)
            counts["kept"] += 1
    return examples, counts


def build_dataset(session_dirs, out_dir, opts=None, namespace=None, selection=None):
    """selection: {session dir: {episode id: (from_tick, to_tick)}} (load_set) to use only those episodes."""
    opts = dict(DEFAULTS, **(opts or {}))
    os.makedirs(out_dir, exist_ok=True)
    files = {name: gzip.open(os.path.join(out_dir, name + ".jsonl.gz"), "wt") for name in opts["splits"]}
    summary = {"options": {k: (str(v) if isinstance(v, float) and math.isinf(v) else v) for k, v in opts.items()},
               "sessions": {}, "splits": {name: {"examples": 0, "episodes": 0} for name in opts["splits"]}}
    try:
        for session_dir in session_dirs:
            session = os.path.basename(os.path.normpath(session_dir))
            samples = il_bags.load_samples(session_dir, namespace)
            outcomes = il_bags.episode_outcomes(samples, il_bags.load_params(session_dir),
                                                il_bags.load_episode_events(session_dir))
            episodes = None if selection is None else selection.get(session_dir, {})
            examples, counts = build_examples(session, samples, outcomes, opts, episodes)
            episodes_per_split = {}
            for ex in examples:
                split = episode_split(session, ex["episode"], opts["splits"], opts["seed"])
                files[split].write(json.dumps(ex) + "\n")
                summary["splits"][split]["examples"] += 1
                episodes_per_split.setdefault(split, set()).add(ex["episode"])
            for split, episodes in episodes_per_split.items():
                summary["splits"][split]["episodes"] += len(episodes)
            summary["sessions"][session] = {"counts": counts, "report": il_bags.session_report(samples, outcomes)}
    finally:
        for f in files.values():
            f.close()
    with open(os.path.join(out_dir, "dataset.yaml"), "w") as f:
        yaml.safe_dump(summary, f, default_flow_style=False, sort_keys=False)
    return summary
