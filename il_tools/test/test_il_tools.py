#!/usr/bin/env python3
"""Offline IL tools: bag checks (plan 1.6) and dataset building (plan 1.7) on synthetic sessions.
Needs rosbag and the manual_gap_msgs Python messages (source the workspace)."""

import gzip
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

import rosbag
import rospy
import yaml
from manual_gap_msgs.msg import ManualGapCandidate, ManualGapSample

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import il_bags  # noqa: E402
import il_dataset  # noqa: E402

TOPIC = "/rto/manual_gap_sample"


def candidate(idx, left, right, terminal, feasible=True, cursor=None):
    c = ManualGapCandidate()
    c.traj_idx = idx
    c.left_model_id, c.right_model_id = left, right
    c.feasible = feasible
    c.shown = feasible
    if feasible:
        c.terminal_pose.position.x, c.terminal_pose.position.y = terminal
        c.terminal_pose.orientation.w = 1.0
    c.cursor_dist = math.hypot(terminal[0] - cursor[0], terminal[1] - cursor[1]) if (cursor and feasible) else float("nan")
    c.cursor_bearing_diff = 0.0 if (cursor and feasible) else float("nan")
    return c


def sample(seq, t, episode, cursor=(3.0, 0.0), gaps=((1, 2, (3.0, 1.0)), (3, 4, (3.0, -1.0))),
           select=True, live=True, reset_window=False, goal=(10.0, 0.0), scan_min=2.0):
    s = ManualGapSample()
    s.schema_version = s.SCHEMA_VERSION
    s.header.stamp = rospy.Time.from_sec(t)
    s.header.frame_id = "rto/odom"
    s.wall_stamp = rospy.Time.from_sec(1000.0 + t)
    s.tick_seq = seq
    s.episode_id = episode
    s.in_reset_window = reset_window
    s.scan.ranges = [scan_min, 4.0]
    s.goal_robot_frame.x, s.goal_robot_frame.y = goal
    s.cursor.header.stamp = rospy.Time.from_sec(t)
    s.cursor.tool_active = True
    s.cursor.pick_ok = live
    s.cursor.last_move_wall = rospy.Time.from_sec(1000.0 + t)
    s.cursor_rx_wall = rospy.Time.from_sec(1000.0 + t)
    s.cursor_odom.x, s.cursor_odom.y = cursor
    s.cursor_valid = True
    s.cursor_live = live
    s.candidates = [candidate(i, l, r, term, cursor=cursor) for i, (l, r, term) in enumerate(gaps)]
    if select:
        best = il_bags.nearest_feasible(s)
        s.source = s.SOURCE_GAP_CURSOR
        s.selected_idx = best
        s.selected_left_model_id, s.selected_right_model_id = gaps[best][0], gaps[best][1]
    else:
        s.source = s.SOURCE_NONE
        s.selected_idx = -1
    return s


class SessionTestCase(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        with open(os.path.join(self.dir, "params.yaml"), "w") as f:
            yaml.safe_dump({"task_generator_server": {"timeout": 60.0, "goal_radius": 1.0}}, f)
        with open(os.path.join(self.dir, "episodes.csv"), "w") as f:
            f.write("event,episode,wall_time,sim_time,start,goal\nreset,0,0,0,,\nreset,1,0,0,,\n")

    def tearDown(self):
        shutil.rmtree(self.dir)

    def write(self, samples, name="session_0.bag"):
        with rosbag.Bag(os.path.join(self.dir, name), "w") as bag:
            for s in samples:
                bag.write(TOPIC, s, s.header.stamp)

    def good_samples(self):
        samples = [sample(i, 0.1 * i, 0) for i in range(20)]
        # episode 1: cursor points at the lower gap; the last tick is at the goal
        samples += [sample(20 + i, 2.0 + 0.1 * i, 1, cursor=(3.0, -1.2), goal=(0.5, 0.0) if i == 19 else (5.0, 0.0))
                    for i in range(20)]
        return samples


class CheckTest(SessionTestCase):

    def checks(self):
        checks, report = il_bags.check_session(self.dir)
        return {name: ok for name, ok, _ in checks}, report

    def test_good_session_passes(self):
        self.write(self.good_samples())
        checks, report = self.checks()
        self.assertTrue(all(checks.values()), checks)
        self.assertEqual(report["usable_ticks"], 40)
        self.assertEqual(report["episodes"], 2)
        self.assertEqual(report["results"], {"ABORTED": 1, "GOAL_REACHED": 1})

    def test_split_bags_are_read_in_order(self):
        samples = self.good_samples()
        self.write(samples[:15], "session_0.bag")
        self.write(samples[15:], "session_1.bag")
        checks, _ = self.checks()
        self.assertTrue(checks["tick_seq contiguous"])

    def test_tick_gap_detected(self):
        samples = self.good_samples()
        del samples[5]
        self.write(samples)
        self.assertFalse(self.checks()[0]["tick_seq contiguous"])

    def test_wrong_label_detected(self):
        samples = self.good_samples()
        samples[3].selected_idx = 1 - samples[3].selected_idx
        self.write(samples)
        self.assertFalse(self.checks()[0]["selection reproducible"])

    def test_clock_mismatch_detected(self):
        samples = self.good_samples()
        samples[7].cursor.header.stamp = rospy.Time.from_sec(1.7e9)  # wall clock stamp
        self.write(samples)
        self.assertFalse(self.checks()[0]["one clock"])

    def test_tool_switched_off_is_not_a_clock_error(self):
        samples = self.good_samples()
        # tool deactivated at tick 5: later samples carry its last message
        for s in samples[5:]:
            s.cursor = samples[4].cursor
            s.cursor_rx_wall = samples[4].cursor_rx_wall
            s.cursor_valid = s.cursor_live = False
            s.source, s.selected_idx = s.SOURCE_NONE, -1
        self.write(samples)
        self.assertTrue(self.checks()[0]["one clock"])

    def test_missing_episodes_csv_fails(self):
        os.remove(os.path.join(self.dir, "episodes.csv"))
        self.write(self.good_samples())
        self.assertFalse(self.checks()[0]["episodes.csv present"])

    def test_outcome_duration_from_episodes_csv(self):
        # the planner stopped ticking at 1.9 s, but the episode ran 60 s until the next begin
        with open(os.path.join(self.dir, "episodes.csv"), "w") as f:
            f.write("event,episode,wall_time,sim_time,start,goal\n"
                    "reset,0,0,0.0,,\nbegin,1,0,60.5,,\nreset,1,0,61.0,,\n")
        outcomes = il_bags.episode_outcomes(self.good_samples(), il_bags.load_params(self.dir),
                                            il_bags.load_episode_events(self.dir))
        self.assertEqual(outcomes[0]["result"], "TIMEOUT")
        self.assertEqual(outcomes[0]["duration"], 60.5)
        # episode 1 is still running when recording stops: duration from samples
        self.assertEqual(outcomes[1]["result"], "GOAL_REACHED")

    def test_outcome_timeout_of_the_episode(self):
        # one session over two benchmark stages: episode 0 runs with a 90 s timeout,
        # episode 1 with 120 s; params.yaml has the first stage's (60 s here)
        with open(os.path.join(self.dir, "episodes.csv"), "w") as f:
            f.write("event,episode,wall_time,sim_time,start,goal,timeout\n"
                    "reset,0,0,0.0,,,90.0\nbegin,1,0,95.0,,,90.0\n"
                    "reset,1,0,96.0,,,120.0\nbegin,2,0,191.0,,,120.0\n")
        events = il_bags.load_episode_events(self.dir)
        settings = il_bags.episode_settings(events, {"timeout": 60.0, "goal_radius": 1.0})
        self.assertEqual({0: {"timeout": 90.0, "goal_radius": 1.0}, 1: {"timeout": 120.0, "goal_radius": 1.0}},
                         settings)
        samples = self.good_samples()
        for s in samples:  # never at the goal: TIMEOUT or ABORTED by duration alone
            s.goal_robot_frame.x = 5.0
        outcomes = il_bags.episode_outcomes(samples, il_bags.load_params(self.dir), events)
        self.assertEqual("TIMEOUT", outcomes[0]["result"])  # 95 s of 90 s
        self.assertEqual("ABORTED", outcomes[1]["result"])  # 95 s of 120 s (60 s would say TIMEOUT)
        # older episodes.csv without the columns: the session's
        self.assertEqual({0: {"timeout": 60.0, "goal_radius": 1.0}},
                         il_bags.episode_settings([{"event": "reset", "episode": "0"}],
                                                  {"timeout": 60.0, "goal_radius": 1.0}))

    def test_goal_radius_of_the_robot_manager(self):
        # the global goal_radius if set, else the task generator's
        self.assertEqual(0.4, il_bags.task_settings({"goal_radius": 0.4, "task_generator_server": {"goal_radius": 0.5}})["goal_radius"])
        self.assertEqual(0.5, il_bags.task_settings({"task_generator_server": {"goal_radius": 0.5}})["goal_radius"])

    def test_active_bag_detected(self):
        self.write(self.good_samples())
        open(os.path.join(self.dir, "session_1.bag.active"), "w").close()
        self.assertFalse(self.checks()[0]["no .bag.active files"])

    def test_episode_without_samples_detected(self):
        with open(os.path.join(self.dir, "episodes.csv"), "a") as f:
            f.write("reset,2,0,0,,\nreset,3,0,0,,\n")
        self.write(self.good_samples())
        # episode 3 may still be running; episode 2 has no samples at all
        self.assertFalse(self.checks()[0]["every episode has an outcome"])

    def use_selection_rule(self):
        """params.yaml of a session recorded after IL fix 43."""
        with open(os.path.join(self.dir, "params.yaml"), "w") as f:
            yaml.safe_dump({"task_generator_server": {"timeout": 60.0, "goal_radius": 1.0},
                            "man_gap_max_bearing_deg": 60.0, "man_gap_switch_ratio": 0.7}, f)

    def follow(self, s, idx):
        s.source = s.SOURCE_GAP_CURSOR
        s.selected_idx = idx
        c = s.candidates[idx]
        s.selected_left_model_id, s.selected_right_model_id = c.left_model_id, c.right_model_id

    def test_sticky_selection_reproduced(self):
        self.use_selection_rule()
        samples = self.good_samples()
        # episode 0: the upper gap (3, 1) is followed; the cursor moves to (3, -0.1),
        # slightly nearer the lower gap but not by the switch ratio, so the upper gap is kept
        for s in samples[10:20]:
            s.cursor_odom.y = -0.1
            self.follow(s, 0)
        self.write(samples)
        self.assertTrue(self.checks()[0]["selection reproducible"])

    def test_sticky_switch_when_clearly_nearer(self):
        self.use_selection_rule()
        samples = self.good_samples()
        for s in samples[10:20]:
            s.cursor_odom.y = -0.9  # clearly nearer the lower gap
            self.follow(s, 0)  # wrong: the planner switches
        self.write(samples)
        self.assertFalse(self.checks()[0]["selection reproducible"])
        for s in samples[10:20]:
            self.follow(s, 1)
        self.write(samples)
        self.assertTrue(self.checks()[0]["selection reproducible"])

    def test_gap_without_model_is_not_held(self):
        # the planner cannot recognise a gap whose point has no model (id -1): it takes
        # the nearest gap, as when nothing is followed
        self.use_selection_rule()
        samples = self.good_samples()
        for s in samples[10:20]:
            s.cursor_odom.y = -0.1  # slightly nearer the lower gap
            s.candidates[0].right_model_id = -1
            self.follow(s, 0)  # held: what the planner would do with known ids
        self.write(samples)
        self.assertFalse(self.checks()[0]["selection reproducible"])
        for s in samples[10:20]:
            self.follow(s, 1)
        self.write(samples)
        self.assertTrue(self.checks()[0]["selection reproducible"])

    def test_no_gap_in_cursor_direction(self):
        self.use_selection_rule()
        samples = self.good_samples()
        # cursor behind the robot: no gap within 60 degrees, so nothing is selected
        for s in samples[5:8]:
            s.cursor_odom.x, s.cursor_odom.y = -3.0, 0.0
            s.source, s.selected_idx = s.SOURCE_NONE, -1
        self.write(samples)
        self.assertTrue(self.checks()[0]["selection reproducible"])
        self.follow(samples[6], 0)  # a gap pointing away from the cursor
        self.write(samples)
        self.assertFalse(self.checks()[0]["selection reproducible"])

    def test_followed_gap_unknown_after_tick_hole(self):
        self.use_selection_rule()
        samples = self.good_samples()
        for s in samples[:9]:
            s.cursor_odom.y = -0.9
            self.follow(s, 1)
        for s in samples[10:20]:
            s.cursor_odom.y = -0.1
            self.follow(s, 0)
        # tick 9, where the planner switched to the upper gap, is missing: tick 10
        # cannot be checked, ticks 11-19 keep the upper gap
        del samples[9]
        self.write(samples)
        self.assertTrue(self.checks()[0]["selection reproducible"])
        _, checked = il_bags.selection_mismatches(samples, il_bags.load_params(self.dir))
        self.assertEqual(checked, 38)

    def test_collision_flagged(self):
        samples = self.good_samples()
        samples[30].scan.ranges = [0.15, 4.0]
        outcomes = il_bags.episode_outcomes(samples, il_bags.load_params(self.dir))
        self.assertFalse(outcomes[0]["collision"])
        self.assertTrue(outcomes[1]["collision"])

    def test_colliding_status_flagged(self):
        # the planner records colliding ticks without a scan
        samples = self.good_samples()
        samples[30].status = 2
        samples[30].scan.ranges = []
        outcomes = il_bags.episode_outcomes(samples, il_bags.load_params(self.dir))
        self.assertFalse(outcomes[0]["collision"])
        self.assertTrue(outcomes[1]["collision"])


class DatasetTest(SessionTestCase):

    def build(self, **opts):
        out = os.path.join(self.dir, "out")
        summary = il_dataset.build_dataset([self.dir], out, dict({"splits": {"train": 1.0}}, **opts))
        with gzip.open(os.path.join(out, "train.jsonl.gz"), "rt") as f:
            examples = [json.loads(line) for line in f]
        return examples, summary["sessions"][os.path.basename(self.dir)]["counts"]

    def test_examples_and_labels(self):
        self.write(self.good_samples())
        examples, counts = self.build()
        self.assertEqual(counts["kept"], 40)
        self.assertTrue(all(e["obs_tick"] == e["label_tick"] for e in examples))
        self.assertEqual({e["label"] for e in examples if e["episode"] == 0}, {0})
        self.assertEqual({e["label"] for e in examples if e["episode"] == 1}, {1})

    def test_lag_pairs_earlier_observation_by_identity(self):
        samples = self.good_samples()
        # in episode 0 the candidate order flips half way: identity, not index, must be used
        for s in samples[10:20]:
            s.candidates = list(reversed(s.candidates))
            s.selected_idx = il_bags.nearest_feasible(s)
        self.write(samples)
        examples, counts = self.build(lag_s=0.3)
        # first three ticks of each episode have no tick 0.3 s earlier
        self.assertEqual(counts["no_obs_tick"], 6)
        ex = [e for e in examples if e["label_tick"] == 11][0]
        self.assertEqual(ex["obs_tick"], 8)
        self.assertEqual(ex["label"], 0)  # gap (1, 2) is still index 0 at tick 8

    def test_identity_lost_dropped(self):
        samples = self.good_samples()
        samples[11].candidates[0].left_model_id = 99  # gap (1, 2) split at tick 11
        self.write(samples)
        # only tick 13 is paired with tick 11
        _, counts = self.build(lag_s=0.15)
        self.assertEqual(counts["identity_lost"], 1)

    def test_gap_without_model_labelled_without_lag(self):
        samples = self.good_samples()
        s = samples[11]
        s.candidates[s.selected_idx].right_model_id = -1
        s.selected_right_model_id = -1
        self.write(samples)
        # without lag the label is the recorded index; with lag the gap cannot be found again
        examples, counts = self.build(lag_s=0.0)
        self.assertEqual(0, counts["identity_lost"])
        self.assertIn(samples[11].selected_idx, [ex["label"] for ex in examples if ex["obs_tick"] == 11])
        _, counts = self.build(lag_s=0.15)
        self.assertGreaterEqual(counts["identity_lost"], 1)

    def test_unusable_ticks_and_thresholds(self):
        samples = self.good_samples()
        samples[2].cursor_live = False
        samples[3].in_reset_window = True
        samples[4] = sample(4, 0.4, 0, select=False)
        self.write(samples)
        _, counts = self.build(max_cursor_dist=0.5)
        self.assertEqual(counts["usable"], 37)
        # episode 0 cursor is 1 m from both terminals; episode 1 is 0.2 m from its pick
        self.assertEqual(counts["threshold"], 17)

    def test_collision_episodes_dropped_or_kept(self):
        samples = self.good_samples()
        samples[30].scan.ranges = [0.15, 4.0]
        self.write(samples)
        _, counts = self.build()
        self.assertEqual(counts["collision_episode"], 20)
        shutil.rmtree(os.path.join(self.dir, "out"))
        _, counts = self.build(drop_collision_episodes=False)
        self.assertEqual(counts["kept"], 40)

    def test_episode_set_selects_and_keeps_collisions(self):
        samples = self.good_samples()
        samples[30].scan.ranges = [0.15, 4.0]  # episode 1 collides
        self.write(samples)
        set_file = os.path.join(self.dir, "set.yaml")
        with open(set_file, "w") as f:
            yaml.safe_dump({"root": os.path.dirname(self.dir),
                            "episodes": {"corner": [{"session": os.path.basename(self.dir), "episode": 1}]}}, f)
        selection = il_dataset.load_set(set_file)
        self.assertEqual(selection, {self.dir: {1: None}})
        out = os.path.join(self.dir, "out")
        summary = il_dataset.build_dataset(list(selection), out, {"splits": {"train": 1.0}}, selection=selection)
        counts = summary["sessions"][os.path.basename(self.dir)]["counts"]
        self.assertEqual((counts["not_in_set"], counts["collision_episode"], counts["kept"]), (20, 0, 20))

    def test_episode_set_cuts_ticks_before_from_tick(self):
        # a collision right after the spawn: the episode is kept from tick 25 on
        samples = self.good_samples()
        samples[21].status = samples[21].STATUS_COLLIDING
        self.write(samples)
        set_file = os.path.join(self.dir, "set.yaml")
        with open(set_file, "w") as f:
            yaml.safe_dump({"root": os.path.dirname(self.dir),
                            "episodes": {"g": [{"session": os.path.basename(self.dir), "episode": 1, "from_tick": 25}]}}, f)
        selection = il_dataset.load_set(set_file)
        self.assertEqual(selection, {self.dir: {1: 25}})
        out = os.path.join(self.dir, "out")
        summary = il_dataset.build_dataset(list(selection), out, {"splits": {"train": 1.0}, "lag_s": 0.15},
                                           selection=selection)
        counts = summary["sessions"][os.path.basename(self.dir)]["counts"]
        with gzip.open(os.path.join(out, "train.jsonl.gz"), "rt") as f:
            examples = [json.loads(line) for line in f]
        self.assertTrue(examples)
        self.assertTrue(all(ex["obs_tick"] >= 25 for ex in examples))
        self.assertGreater(counts["cut"], 0)

    def test_observation_in_reset_window_or_not_planned_dropped(self):
        samples = self.good_samples()
        samples[7].in_reset_window = True  # obs of tick 10 at lag 0.25
        samples[8].status = samples[8].STATUS_COLLIDING  # obs of tick 11
        self.write(samples)
        # a colliding tick makes episode 0 a collision episode; keep it to see the drop
        _, counts = self.build(lag_s=0.25, drop_collision_episodes=False)
        self.assertEqual(counts["obs_invalid"], 2)

    def test_stale_observation_dropped(self):
        samples = self.good_samples()
        # a 3 s hole in the samples of episode 0 after tick 9
        for s in samples[10:20]:
            s.wall_stamp = rospy.Time.from_sec(il_bags.stamp(s.wall_stamp) + 3.0)
        self.write(samples)
        _, counts = self.build(lag_s=0.25)
        # ticks 10-12 would pair with tick 9, 3 s too old
        self.assertEqual(counts["obs_too_old"], 3)

    def test_same_observation_and_label_kept_once(self):
        samples = self.good_samples()
        # tick 6 missing: ticks 8 and 9 (lag 0.25 -> obs 5.5, 6.5) would both pair with tick 5
        del samples[6]
        for s in samples[6:]:
            s.tick_seq -= 1
        self.write(samples)
        _, counts = self.build(lag_s=0.25, max_lag_error_s=1.0)
        self.assertEqual(counts["duplicate"], 1)

    def test_dynamic_gap_costs_and_pick_exported(self):
        samples = self.good_samples()
        for s in samples:
            s.dg_pick_idx = 1
            for c, cost in zip(s.candidates, (2.0, 1.5)):
                c.dg_cost, c.dg_terminal_cost, c.dg_clearance_cost = cost, cost - 1.0, 1.0
        samples[5].candidates[0].dg_cost = float("inf")
        self.write(samples)
        examples, _ = self.build()
        ex = [e for e in examples if e["obs_tick"] == 4][0]
        self.assertEqual(ex["schema_version"], ManualGapSample.SCHEMA_VERSION)
        self.assertEqual(ex["dg_pick"], 1)
        self.assertEqual([c["dg_cost"] for c in ex["candidates"]], [2.0, 1.5])
        self.assertEqual(ex["candidates"][1]["dg_terminal_cost"], 0.5)
        self.assertTrue(math.isinf([e for e in examples if e["obs_tick"] == 5][0]["candidates"][0]["dg_cost"]))

    def test_is_current_from_previous_tick(self):
        self.write(self.good_samples())
        examples, _ = self.build()
        by_tick = {e["obs_tick"]: e for e in examples}
        # first tick of an episode follows nothing
        self.assertFalse(any(c["is_current"] for c in by_tick[0]["candidates"]))
        self.assertEqual([c["is_current"] for c in by_tick[1]["candidates"]], [True, False])
        self.assertEqual([c["is_current"] for c in by_tick[21]["candidates"]], [False, True])

    def test_robot_frame_geometry(self):
        samples = self.good_samples()
        for s in samples:
            # robot at (1, 0) facing +y
            s.robot_pose_odom.position.x = 1.0
            s.robot_pose_odom.orientation.z = math.sin(math.pi / 4)
            s.robot_pose_odom.orientation.w = math.cos(math.pi / 4)
        self.write(samples)
        examples, _ = self.build()
        ex = examples[0]
        self.assertAlmostEqual(ex["robot_yaw_odom"], math.pi / 2)
        # odom terminal (3, 1): 1 m ahead, 2 m to the right
        x, y = ex["candidates"][0]["terminal_rbt"]
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, -2.0)
        self.assertTrue(ex["candidates"][0]["shown"])

    def test_split_is_by_episode_and_deterministic(self):
        splits = {"train": 0.5, "val": 0.5}
        a = [il_dataset.episode_split("s", e, splits, "seed") for e in range(50)]
        b = [il_dataset.episode_split("s", e, splits, "seed") for e in range(50)]
        self.assertEqual(a, b)
        self.assertEqual(set(a), {"train", "val"})


if __name__ == "__main__":
    unittest.main()
