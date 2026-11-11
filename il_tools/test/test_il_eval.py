#!/usr/bin/env python3
"""il_eval: per-episode metrics of a closed-loop run (needs rosbag and manual_gap_msgs)."""

import os
import sys
import unittest

import rospy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import il_bags  # noqa: E402
import il_eval  # noqa: E402
from test_il_tools import SessionTestCase, sample  # noqa: E402

STAGES = [{"name": "a", "episodes": 1}, {"name": "b", "episodes": 1}]


class EvalTest(SessionTestCase):

    def run_samples(self):
        samples = []
        for i in range(20):  # episode 0: drives 0.1 m per tick, at the goal at the end
            s = sample(i, 0.1 * i, 0, goal=(0.5, 0.0) if i == 19 else (5.0, 0.0))
            s.robot_pose_odom.position.x = 0.1 * i
            s.source = s.SOURCE_MODEL
            samples.append(s)
        for i in range(20):  # episode 1: two contacts, then the planner stops for 3 s
            t = 2.0 + 0.1 * i + (3.0 if i >= 15 else 0.0)
            s = sample(20 + i, t, 1)
            s.source = s.SOURCE_MODEL
            if i in (3, 4, 8):
                s.status = s.STATUS_COLLIDING
            samples.append(s)
        return samples

    def test_metrics(self):
        self.write(self.run_samples())
        rows = il_eval.episode_metrics(self.dir, STAGES)
        a, b = rows
        self.assertEqual(("a", "b"), (a["stage"], b["stage"]))
        self.assertEqual("GOAL_REACHED", a["result"])
        self.assertTrue(a["valid"])
        self.assertAlmostEqual(1.9, a["path_length_m"])
        self.assertEqual(a["duration_s"], a["time_to_goal_s"])
        self.assertEqual((0, 0), (a["contacts"], a["colliding_ticks"]))
        self.assertEqual(20, a["model_ticks"])
        # two runs of colliding ticks; a stall of 3 s makes the episode not valid
        self.assertEqual((2, 3), (b["contacts"], b["colliding_ticks"]))
        self.assertTrue(b["collision"])
        self.assertFalse(b["valid"])
        self.assertGreater(b["planner_stall_s"], il_eval.STALL_S)
        self.assertIsNone(b["time_to_goal_s"])
        s = il_eval.summary(rows)
        self.assertEqual((2, 1, 1, 1.0), (s["episodes"], s["valid_episodes"], s["goal_reached"], s["success_rate"]))
        self.assertEqual(0, s["collision_episodes"])  # the invalid episode is left out

    def test_episode_windows(self):
        with open(os.path.join(self.dir, "episodes.csv"), "w") as f:
            f.write("event,episode,wall_time,sim_time,start,goal\n"
                    "begin,0,100.0,0,,\nreset,0,101.0,0,,\nbegin,1,130.0,30,,\nreset,1,131.0,30,,\n")
        self.assertEqual({0: (101.0, 130.0), 1: (131.0, None)}, il_eval.episode_windows(self.dir))

    def test_stage_names(self):
        self.assertEqual({0: "a", 1: "a", 2: "b", 3: ""},
                         il_eval.stage_names([{"name": "a", "episodes": 2}, {"name": "b", "episodes": 1}], [0, 1, 2, 3]))


if __name__ == "__main__":
    unittest.main()
