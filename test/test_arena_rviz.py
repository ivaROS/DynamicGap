#!/usr/bin/env python3
"""Issues 19 and 14: arena.rviz offers no tools that move the robot, the goal or reset
the episode, and uses the namespaced manual-mode topics."""

import os
import unittest

import yaml

RVIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rviz", "arena.rviz")


class ArenaRvizTest(unittest.TestCase):

    def setUp(self):
        with open(RVIZ) as f:
            self.config = yaml.safe_load(f)["Visualization Manager"]

    def tools(self):
        return [tool["Class"] for tool in self.config["Tools"]]

    def test_no_tools_that_change_the_episode(self):
        for dangerous in ("rviz/SetInitialPose", "rviz/SetGoal", "rviz/PublishPoint"):
            self.assertNotIn(dangerous, self.tools())

    def test_kept_tools(self):
        self.assertEqual(self.tools(), [
            "rviz/Interact", "rviz/MoveCamera", "rviz/Select", "rviz/FocusCamera",
            "rviz/Measure", "rviz_plugins/ContinuousGapSelectionTool",
        ])

    def test_gap_selection_tool_prefix(self):
        tool = [t for t in self.config["Tools"] if t["Class"] == "rviz_plugins/ContinuousGapSelectionTool"][0]
        self.assertEqual(tool["Topic Prefix"], "/rto/")

    def test_candidate_markers_topic(self):
        def displays(items):
            for item in items:
                yield item
                yield from displays(item.get("Displays", []))

        topics = [d.get("Marker Topic") for d in displays(self.config["Displays"]) if d.get("Class") == "rviz/MarkerArray"]
        self.assertIn("/rto/manual_candidate_markers", topics)
        self.assertNotIn("/manual_candidate_markers", topics)


if __name__ == "__main__":
    unittest.main()
