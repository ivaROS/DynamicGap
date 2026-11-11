"""
Offline reading and checking of imitation learning sessions recorded with
arena_bringup/launch/record_il.launch (see docs/IL_DATA_COLLECTION_PLAN.md,
sections 1.6 and 1.7).

A session folder holds session_*.bag, params.yaml, versions.yaml, episodes.csv
and notes.txt. Only rosbag and the generated manual_gap_msgs Python messages
are needed; the planner does not have to be built.
"""

import csv
import glob
import math
import os
import re

import yaml

SAMPLE_TOPIC_SUFFIX = "/manual_gap_sample"

# planner/task defaults used when params.yaml does not have them
DEFAULT_GOAL_RADIUS = 1.0
DEFAULT_TIMEOUT = 180.0
GOAL_CHECK_PERIOD = 0.5  # s, task generator goal check
ROBOT_RADIUS = 0.2  # m, rto
COLLISION_EPS = 0.01  # m, as arena_evaluation metrics

SOURCE_NONE, SOURCE_GAP_CURSOR, SOURCE_CLICK_TO_MOVE, SOURCE_MODEL = 0, 1, 2, 3
STATUS_PLANNED = 0
STATUS_COLLIDING = 2
CURSOR_FRESH_S = 0.5  # wall s; the tool publishes every 0.1 s, older means it stopped

# manual selection rule (IL fix 43), planner defaults
DEFAULT_MAX_BEARING_DEG = 60.0
DEFAULT_SWITCH_RATIO = 0.7
# the planner selects with the robot pose at selection time, the sample records
# it at publish; odometry (50 Hz) can move in between, so a choice this close to
# a threshold counts as reproduced
SELECTION_BEARING_TOL = math.radians(2.0)
SELECTION_RATIO_TOL = 0.02


def bag_files(session_dir):
    """session_0.bag, session_1.bag, ... in split order."""
    def index(path):
        match = re.search(r"_(\d+)\.bag$", path)
        return int(match.group(1)) if match else -1
    return sorted(glob.glob(os.path.join(session_dir, "*.bag")), key=index)


def active_bag_files(session_dir):
    """Bags rosbag never closed (crash, kill -9)."""
    return sorted(glob.glob(os.path.join(session_dir, "*.bag.active")))


def read_messages(session_dir, topics=None):
    """(topic, msg, bag time) of every bag of the session, in order."""
    import rosbag
    for path in bag_files(session_dir):
        with rosbag.Bag(path) as bag:
            for topic, msg, t in bag.read_messages(topics=topics):
                yield topic, msg, t


def sample_topic(session_dir, namespace=None):
    if namespace:
        return "/" + namespace.strip("/") + SAMPLE_TOPIC_SUFFIX
    import rosbag
    for path in bag_files(session_dir):
        with rosbag.Bag(path) as bag:
            for topic in bag.get_type_and_topic_info().topics:
                if topic.endswith(SAMPLE_TOPIC_SUFFIX):
                    return topic
    return None


def load_samples(session_dir, namespace=None):
    """ManualGapSample messages of a session, in recording order."""
    topic = sample_topic(session_dir, namespace)
    if topic is None:
        return []
    return [msg for _, msg, _ in read_messages(session_dir, [topic])]


def load_params(session_dir):
    path = os.path.join(session_dir, "params.yaml")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def load_episode_events(session_dir):
    """Rows of episodes.csv: event (begin/reset), episode, wall_time, sim_time, start, goal."""
    path = os.path.join(session_dir, "episodes.csv")
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def task_settings(params):
    """Timeout and goal radius of a session. The robot manager takes its goal
    radius from the global goal_radius param, else (unset in every launch) from
    the task generator's config when it starts: the server's goal_radius."""
    server = params.get("task_generator_server", {}) or {}
    timeout = server.get("timeout", DEFAULT_TIMEOUT)
    return {
        "goal_radius": float(params.get("goal_radius", server.get("goal_radius", DEFAULT_GOAL_RADIUS))),
        "timeout": math.inf if timeout is None or timeout < 0 else float(timeout),
    }


# ---------------------------------------------------------------- labels


def nearest_feasible(sample):
    """Index of the feasible candidate whose terminal pose is nearest the
    recorded cursor point (the planner's rule), -1 if none."""
    best, best_dist = -1, math.inf
    for i, c in enumerate(sample.candidates):
        if not c.feasible:
            continue
        dist = math.hypot(c.terminal_pose.position.x - sample.cursor_odom.x,
                          c.terminal_pose.position.y - sample.cursor_odom.y)
        if dist < best_dist:
            best, best_dist = i, dist
    return best


def selection_rule(params):
    """(max bearing difference in rad, switch ratio) of the planner's manual
    selection (IL fix 43), with the planner's fallbacks for out-of-range
    values; None for sessions recorded before fix 43 (nearest feasible)."""
    if "man_gap_max_bearing_deg" not in params and "man_gap_switch_ratio" not in params:
        return None

    def number(key, default, low, high):
        value = params.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not low < value <= high:
            return default
        return float(value)

    return (math.radians(number("man_gap_max_bearing_deg", DEFAULT_MAX_BEARING_DEG, 0.0, 180.0)),
            number("man_gap_switch_ratio", DEFAULT_SWITCH_RATIO, 0.0, 1.0))


def rule_selection(sample, followed, max_bearing, switch_ratio):
    """The planner's gap-cursor choice (selectManualTrajectory, IL fix 43):
    feasible candidates whose terminal point is within max_bearing of the
    robot-to-cursor bearing, the nearest of those to the cursor, and the
    followed gap (identity, None if none) kept unless the nearest one's cursor
    distance is below switch_ratio times its own. Index into candidates, -1 if none."""
    rbt = sample.robot_pose_odom.position
    cx, cy = sample.cursor_odom.x, sample.cursor_odom.y
    cursor_bearing = math.atan2(cy - rbt.y, cx - rbt.x)

    def dist(i):
        p = sample.candidates[i].terminal_pose.position
        return math.hypot(p.x - cx, p.y - cy)

    selectable = []
    for i, c in enumerate(sample.candidates):
        if not c.feasible:
            continue
        p = c.terminal_pose.position
        diff = cursor_bearing - math.atan2(p.y - rbt.y, p.x - rbt.x)
        if abs(math.atan2(math.sin(diff), math.cos(diff))) <= max_bearing:
            selectable.append(i)
    if not selectable:
        return -1

    nearest = min(selectable, key=dist)  # ties go to the lower index, as in the planner
    current = next((i for i in selectable if followed is not None and identity(sample.candidates[i]) == followed), -1)
    if current < 0 or nearest == current:
        return nearest
    return nearest if dist(nearest) < switch_ratio * dist(current) else current


def selection_explained(sample, followed, rule):
    """Whether the rule gives the recorded selected_idx, allowing for the
    robot moving between selection and publish (SELECTION_*_TOL)."""
    max_bearing, ratio = rule
    return any(rule_selection(sample, followed, b, r) == sample.selected_idx
               for b in (max_bearing, max_bearing - SELECTION_BEARING_TOL, max_bearing + SELECTION_BEARING_TOL)
               for r in (ratio, ratio - SELECTION_RATIO_TOL, ratio + SELECTION_RATIO_TOL))


def selection_mismatches(samples, params):
    """Ticks whose recorded choice the planner's selection rule does not
    reproduce: (tick_seq list, number of ticks checked).

    Before fix 43, every gap-cursor tick must select the nearest feasible
    candidate. From fix 43 on, the rule depends on the gap followed before, so
    it is replayed tick by tick: the followed gap is the last gap-cursor
    selection, cleared by click-to-move, by a selection that found no gap in
    the cursor's direction and by an episode reset. Checked are gap-cursor
    ticks and ticks where the cursor was usable but nothing was selected,
    outside reset windows and after any hole in tick_seq (the followed gap is
    then unknown until the next selection)."""
    rule = selection_rule(params)
    if rule is None:
        cursor_ticks = [s for s in samples if s.source == SOURCE_GAP_CURSOR]
        return [s.tick_seq for s in cursor_ticks if nearest_feasible(s) != s.selected_idx], len(cursor_ticks)

    unknown = object()
    followed, episode, prev = None, None, None
    mismatched, checked = [], 0
    for s in samples:
        if (prev is None and s.tick_seq != 0) or (prev is not None and s.tick_seq != prev.tick_seq + 1):
            followed = unknown  # recording started late, a hole, or a planner restart
        prev = s
        if s.status != STATUS_PLANNED:
            continue  # the planner returned before selection
        if episode is not None and s.episode_id != episode:
            followed = None
        episode = s.episode_id
        if s.source == SOURCE_CLICK_TO_MOVE:
            followed = None
            continue
        if s.source != SOURCE_GAP_CURSOR and not s.cursor_valid:
            continue  # no usable cursor: selection did not run, the followed gap stays
        if followed is not unknown and not s.in_reset_window:
            checked += 1
            if not selection_explained(s, followed, rule):
                mismatched.append(s.tick_seq)
        followed = selected_identity(s) if s.source == SOURCE_GAP_CURSOR else None
    return mismatched, checked


def is_usable_label(sample):
    """Plan 1.4: the human was choosing on this tick."""
    return (sample.source == SOURCE_GAP_CURSOR and sample.cursor_live
            and not sample.in_reset_window and sample.selected_idx >= 0)


def known_identity(left_model_id, right_model_id):
    """A gap's identity across ticks, None if a gap point has no model (id -1): the
    planner then cannot tell it from another gap (hasModelIDs)."""
    if left_model_id == -1 or right_model_id == -1:
        return None
    return (left_model_id, right_model_id)


def identity(candidate):
    return known_identity(candidate.left_model_id, candidate.right_model_id)


def selected_identity(sample):
    return known_identity(sample.selected_left_model_id, sample.selected_right_model_id)


def stamp(t):
    return t.secs + t.nsecs * 1e-9


# ---------------------------------------------------------------- episodes


def robot_radius(params, samples):
    """robot_radius of the recorded robot namespace, else the rto default."""
    for s in samples:
        namespace = s.header.frame_id.split("/")[0]
        value = (params.get(namespace, {}) or {}).get("robot_radius")
        if value is not None:
            return float(value)
        break
    return ROBOT_RADIUS


def episode_spans(events):
    """{episode: (start, end)} in sim time from episodes.csv: from the
    episode's scenario_reset to the next scenario_reset_begin (end None while
    the episode was still running when recording stopped)."""
    spans, current = {}, None
    for e in events:
        try:
            episode, sim_time = int(e["episode"]), float(e["sim_time"])
        except (KeyError, ValueError):
            continue
        if e.get("event") == "reset":
            spans[episode] = (sim_time, None)
            current = episode
        elif e.get("event") == "begin" and current is not None and spans[current][1] is None:
            spans[current] = (spans[current][0], sim_time)
    return spans


def episode_settings(events, settings):
    """{episode: task settings}: the timeout from episodes.csv's reset rows (benchmark
    stages on one map run in one session with their own timeout), else the session's.
    The goal radius is the session's: the robot manager reads it once, at startup."""
    per_episode = {}
    for e in events:
        if e.get("event") != "reset":
            continue
        try:
            episode = int(e["episode"])
            timeout = float(e.get("timeout") or "nan")
        except (KeyError, ValueError):
            continue
        per_episode[episode] = dict(
            settings, timeout=settings["timeout"] if math.isnan(timeout) else (math.inf if timeout < 0 else timeout))
    return per_episode


def episode_outcomes(samples, params, events=None):
    """Outcome per episode id: GOAL_REACHED if the last pose is within the
    goal radius (plus the distance covered in one goal check), TIMEOUT if the
    episode lasted the task timeout, else ABORTED; collision if the scan came
    within the robot radius or the planner reported STATUS_COLLIDING (those
    ticks carry no scan, so the scan alone misses the contact itself). The duration comes from episodes.csv when it has
    the episode (the planner can stop ticking before the episode ends), else
    from the samples; the timeout from its reset row there, else from
    params.yaml."""
    session_settings = task_settings(params)
    settings_of = episode_settings(events or [], session_settings)
    spans = episode_spans(events or [])
    radius = robot_radius(params, samples)
    episodes = {}
    for s in samples:
        if s.episode_id < 0 or s.in_reset_window:
            continue
        episodes.setdefault(s.episode_id, []).append(s)

    outcomes = {}
    for episode, ticks in sorted(episodes.items()):
        first, last = ticks[0], ticks[-1]
        goal = last.goal_robot_frame
        speed = math.hypot(last.robot_vel.linear.x, last.robot_vel.linear.y)
        distance = math.hypot(goal.x, goal.y)
        span = spans.get(episode)
        if span and span[1] is not None:
            duration = span[1] - span[0]
        else:
            duration = stamp(last.header.stamp) - stamp(first.header.stamp)
        min_range = min((min(s.scan.ranges) for s in ticks if s.scan.ranges), default=math.inf)
        settings = settings_of.get(episode, session_settings)
        if distance < settings["goal_radius"] + speed * GOAL_CHECK_PERIOD:
            result = "GOAL_REACHED"
        elif duration >= settings["timeout"] - 1.0:
            result = "TIMEOUT"
        else:
            result = "ABORTED"
        outcomes[episode] = {
            "result": result,
            "collision": (min_range <= radius + COLLISION_EPS
                          or any(s.status == STATUS_COLLIDING for s in ticks)),
            "ticks": len(ticks),
            "duration": round(duration, 2),
            "final_goal_distance": round(distance, 2),
        }
    return outcomes


# ---------------------------------------------------------------- checks


def check_session(session_dir, namespace=None, samples=None):
    """Automatic checks of plan 1.6. Returns (checks, report): checks is a list
    of (name, ok, detail)."""
    samples = load_samples(session_dir, namespace) if samples is None else samples
    params = load_params(session_dir)
    checks = []

    def check(name, ok, detail=""):
        checks.append((name, bool(ok), detail))

    check("bags present", bag_files(session_dir), "{} bag(s)".format(len(bag_files(session_dir))))
    active = active_bag_files(session_dir)
    check("no .bag.active files", not active, ", ".join(os.path.basename(a) for a in active))
    check("samples present", samples, "{} samples".format(len(samples)))

    # tick_seq contiguous (a planner restart starts again at 0)
    gaps, restarts = [], 0
    for prev, cur in zip(samples, samples[1:]):
        if cur.tick_seq == prev.tick_seq + 1:
            continue
        if cur.tick_seq < prev.tick_seq:
            restarts += 1
        else:
            gaps.append((prev.tick_seq, cur.tick_seq))
    check("tick_seq contiguous", not gaps,
          "{} gap(s){}; {} planner restart(s)".format(len(gaps), " e.g. {}".format(gaps[:3]) if gaps else "", restarts))

    # the planner's selection rule, replayed, reproduces the recorded choice
    mismatched, checked = selection_mismatches(samples, params)
    check("selection reproducible", not mismatched,
          "{} of {} selection ticks differ ({}){}".format(
              len(mismatched), checked,
              "nearest feasible" if selection_rule(params) is None else "direction cone + switch ratio",
              " e.g. tick {}".format(mismatched[:3]) if mismatched else ""))

    # one clock: cursor stamps (RViz) and sample stamps (planner) agree; only
    # ticks whose cursor message is fresh (the tool may have been switched off)
    off_clock = [s.tick_seq for s in samples
                 if s.cursor.header.stamp.secs and s.cursor_rx_wall.secs
                 and stamp(s.wall_stamp) - stamp(s.cursor_rx_wall) < CURSOR_FRESH_S
                 and abs(stamp(s.cursor.header.stamp) - stamp(s.header.stamp)) > 5.0]
    backwards = [cur.tick_seq for prev, cur in zip(samples, samples[1:])
                 if cur.tick_seq == prev.tick_seq + 1 and stamp(cur.header.stamp) < stamp(prev.header.stamp)
                 and cur.episode_id == prev.episode_id]
    check("one clock", not off_clock and not backwards,
          "{} tick(s) with cursor clock off by > 5 s, {} backwards stamp(s) within an episode".format(len(off_clock), len(backwards)))

    # every episode has an outcome
    events = load_episode_events(session_dir)
    check("episodes.csv present", events, "{} event(s)".format(len(events)))
    outcomes = episode_outcomes(samples, params, events)
    started = sorted({int(e["episode"]) for e in events if e.get("event") == "reset"})
    missing = [e for e in started if e not in outcomes]
    # the last episode may still be running when recording stops
    if missing and started and missing[-1] == started[-1]:
        missing = missing[:-1]
    check("every episode has an outcome", outcomes and not missing,
          "{} episode(s); without samples: {}".format(len(outcomes), missing))

    report = session_report(samples, outcomes)
    return checks, report


def session_report(samples, outcomes):
    """Counts per session, in decisions rather than ticks (plan 1.7)."""
    usable = [s for s in samples if is_usable_label(s)]
    decisions, previous, last_move = 0, None, None
    for s in usable:
        ident = selected_identity(s)
        moved = last_move is not None and stamp(s.cursor.last_move_wall) != last_move
        if previous is not None and ident != previous and moved:
            decisions += 1
        previous, last_move = ident, stamp(s.cursor.last_move_wall)

    results = {}
    for o in outcomes.values():
        results[o["result"]] = results.get(o["result"], 0) + 1

    return {
        "ticks": len(samples),
        "usable_ticks": len(usable),
        "decisions": decisions,
        "ticks_with_choice": sum(1 for s in usable if sum(c.feasible for c in s.candidates) > 1),
        "cursor_ticks": sum(1 for s in samples if s.source == SOURCE_GAP_CURSOR),
        "click_to_move_ticks": sum(1 for s in samples if s.source == SOURCE_CLICK_TO_MOVE),
        "model_ticks": sum(1 for s in samples if s.source == SOURCE_MODEL),
        "hold_still_ticks": sum(1 for s in samples if s.hold_still),
        "episodes": len(outcomes),
        "results": results,
        "collision_episodes": sum(1 for o in outcomes.values() if o["collision"]),
    }
