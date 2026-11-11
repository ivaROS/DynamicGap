"""Closed-loop evaluation of a gap scorer: per-episode metrics of a recorded session.

The session is one il_eval_model run (record_il.launch with the learned scorer on), one
episode per benchmark stage or several. Per episode: outcome, time to the goal, path
length, collisions (planner STATUS_COLLIDING ticks, contacts, closest scan reading and
pedestrian) and planner stalls. An episode in which the planner stopped ticking for
more than STALL_S (move_base_flex stalled or crashed) is not valid: the robot then
coasts on its last command or stands still, and its outcome says nothing about the
model.
"""

import csv
import math
import os

import il_bags

STALL_S = 2.0  # sim seconds without a planner tick inside an episode


def stage_names(stages, episodes):
    """Stage name per episode id, from the suite's stages (in order, with episode counts)."""
    names = []
    for stage in stages:
        names += [stage["name"]] * int(stage["episodes"])
    return {e: names[e] if e < len(names) else "" for e in episodes}


def contact_runs(ticks):
    """Number of contacts: runs of consecutive STATUS_COLLIDING ticks."""
    runs, previous = 0, False
    for s in ticks:
        colliding = s.status == il_bags.STATUS_COLLIDING
        runs += colliding and not previous
        previous = colliding
    return runs


def episode_metrics(session_dir, stages=(), namespace=None):
    """[metrics per episode] of a session (dicts, csv-ready)."""
    samples = il_bags.load_samples(session_dir, namespace)
    params = il_bags.load_params(session_dir)
    events = il_bags.load_episode_events(session_dir)
    outcomes = il_bags.episode_outcomes(samples, params, events)
    spans = il_bags.episode_spans(events)
    radius = il_bags.robot_radius(params, samples)
    peds = _pedestrians(session_dir)
    names = stage_names(stages, outcomes)

    rows = []
    for episode, outcome in sorted(outcomes.items()):
        ticks = [s for s in samples if s.episode_id == episode]
        live = [s for s in ticks if not s.in_reset_window]
        stamps = [il_bags.stamp(s.header.stamp) for s in live]
        span = spans.get(episode)
        # the longest stretch without a planner tick, also up to the episode's end
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        if span and span[1] is not None and stamps:
            gaps.append(span[1] - stamps[-1])
        stall = max(gaps, default=0.0)
        path = sum(math.hypot(b.robot_pose_odom.position.x - a.robot_pose_odom.position.x,
                              b.robot_pose_odom.position.y - a.robot_pose_odom.position.y)
                   for a, b in zip(live, live[1:]))
        colliding = [s for s in live if s.status == il_bags.STATUS_COLLIDING]
        period = (stamps[-1] - stamps[0]) / max(len(stamps) - 1, 1) if len(stamps) > 1 else 0.1
        selected = [il_bags.selected_identity(s) for s in live if s.selected_idx >= 0]
        reached = outcome["result"] == "GOAL_REACHED"
        rows.append({
            "episode": episode,
            "stage": names.get(episode, ""),
            "valid": stall <= STALL_S,
            "result": outcome["result"],
            "time_to_goal_s": round(outcome["duration"], 2) if reached else None,
            "duration_s": outcome["duration"],
            "path_length_m": round(path, 2),
            "final_goal_distance_m": outcome["final_goal_distance"],
            "collision": outcome["collision"],
            "contacts": contact_runs(live),
            "colliding_ticks": len(colliding),
            "contact_time_s": round(len(colliding) * period, 2),
            "min_scan_m": round(min((min(s.scan.ranges) for s in live if s.scan.ranges), default=math.nan), 3),
            "min_ped_dist_m": round(_min_ped_distance(live, peds), 3),
            "robot_radius_m": radius,
            "planner_stall_s": round(stall, 2),
            "ticks": len(ticks),
            "model_ticks": sum(s.source == il_bags.SOURCE_MODEL for s in ticks),
            "hold_still_ticks": sum(bool(s.hold_still) for s in ticks),
            "gap_switches": sum(a != b or a is None for a, b in zip(selected, selected[1:])),
        })
    return rows


def summary(rows):
    """Totals over the valid episodes of one run."""
    valid = [r for r in rows if r["valid"]]
    reached = [r for r in valid if r["result"] == "GOAL_REACHED"]
    mean = lambda xs: round(sum(xs) / len(xs), 2) if xs else None  # noqa: E731
    return {
        "episodes": len(rows),
        "valid_episodes": len(valid),
        "goal_reached": len(reached),
        "success_rate": round(len(reached) / len(valid), 3) if valid else None,
        "timeouts": sum(r["result"] == "TIMEOUT" for r in valid),
        "mean_time_to_goal_s": mean([r["time_to_goal_s"] for r in reached]),
        "mean_path_length_m": mean([r["path_length_m"] for r in reached]),
        "collision_episodes": sum(r["collision"] for r in valid),
        "contacts": sum(r["contacts"] for r in valid),
        "contact_time_s": round(sum(r["contact_time_s"] for r in valid), 2),
        "min_ped_dist_m": min((r["min_ped_dist_m"] for r in valid if not math.isnan(r["min_ped_dist_m"])),
                              default=None),
    }


def write_csv(rows, path):
    if not rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def _pedestrians(session_dir):
    """[(sim time, [(x, y)])] of the recorded pedestrians."""
    out = []
    for _, msg, t in il_bags.read_messages(session_dir, ["/pedsim_simulator/pedsim_agents_data"]):
        out.append((t.to_sec(), [(a.pose.position.x, a.pose.position.y) for a in msg.agent_states]))
    return out


def _min_ped_distance(ticks, peds):
    """Smallest robot-to-pedestrian centre distance over the ticks (latest pedestrian
    message at or before each tick)."""
    if not peds or not ticks:
        return math.nan
    best, j = math.inf, 0
    for s in ticks:
        t = il_bags.stamp(s.header.stamp)
        while j + 1 < len(peds) and peds[j + 1][0] <= t:
            j += 1
        p = s.robot_pose_odom.position
        for x, y in peds[j][1]:
            best = min(best, math.hypot(x - p.x, y - p.y))
    return best


def episode_windows(session_dir):
    """{episode: (wall start, wall end)} from episodes.csv: the episode's reset to the
    next begin (end None while it was still running), for cutting a screen recording."""
    windows, current = {}, None
    for e in il_bags.load_episode_events(session_dir):
        try:
            episode, wall = int(e["episode"]), float(e["wall_time"])
        except (KeyError, ValueError):
            continue
        if e.get("event") == "reset":
            windows[episode] = (wall, None)
            current = episode
        elif e.get("event") == "begin" and current is not None and windows[current][1] is None:
            windows[current] = (windows[current][0], wall)
    return windows


def output_dirs(root):
    os.makedirs(os.path.join(root, "videos"), exist_ok=True)
    return root
