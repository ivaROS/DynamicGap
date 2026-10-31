#!/usr/bin/env python3
"""Density inference contract checks; uses generated ROS messages without a master."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from collections import defaultdict, deque
from unittest.mock import patch

import numpy as np
import pandas as pd
import rospy
import torch
from dynamic_gap.msg import GapFeatureObservation

SCRIPTS = Path(__file__).resolve().parents[1] / "ml_gap_velocity/scripts"


def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


inference = module("density_inference", "gru_gap_feature_node.py")
training = module("density_training", "train_gap_density_gru.py")


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class LastDensity(torch.nn.Module):
    def forward(self, x):
        return x[:, -1, :1]


class DensityPythonTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.stats_path = Path(self.directory.name) / "stats.json"
        self.stats = dict(input_features=inference.INPUT_FEATURES,
                          output_features=inference.OUTPUT_FEATURES,
                          x_mean=[1] * 6, x_std=[2] * 6,
                          y_mean=[3], y_std=[4], seq_len=2)
        self.stats_path.write_text(json.dumps(self.stats))
        self.normalizer = inference.Normalizer(str(self.stats_path))

    def node(self):
        node = inference.GRUGapFeatureNode.__new__(inference.GRUGapFeatureNode)
        node.normalizer = self.normalizer
        node.seq_len = 2
        node.input_features = self.normalizer.input_features
        node.output_features = self.normalizer.output_features
        node.buffers = defaultdict(lambda: deque(maxlen=node.seq_len))
        node.last_stamp_by_key = {}
        node.max_buffer_time_gap = .5
        node.device = torch.device("cpu")
        # Exercise the actual TorchScript inference interface.
        node.model = torch.jit.trace(LastDensity(), torch.zeros(1, 2, 6))
        node.pred_pub = Publisher()
        node.marker_pub = Publisher()
        node.text_marker_height = .28
        node.text_marker_z = .35
        return node

    def observation(self, stamp, model=7):
        message = GapFeatureObservation()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.header.frame_id = "rto/base_link"
        message.model_id = model
        message.side = "left"
        # Wire ordering may vary; the stats contract determines the tensor ordering.
        message.feature_names = list(reversed(inference.INPUT_FEATURES))
        message.feature_values = [11, 9, 7, 5, 3, 1]
        return message

    def test_normalization_parity_and_named_order(self):
        self.assertEqual(training.INPUT_FEATURES, inference.INPUT_FEATURES)
        sample = self.node().build_model_input_sample(self.observation(10))
        np.testing.assert_array_equal(sample, [1, 3, 5, 7, 9, 11])
        np.testing.assert_array_equal(self.normalizer.normalize_input(sample), [0, 1, 2, 3, 4, 5])
        np.testing.assert_array_equal(self.normalizer.denormalize_target(np.array([2.])), [11])
        # Trainer already saves epsilon in std. Inference must not add it again.
        self.normalizer.input_std[:] = 1e-8
        np.testing.assert_allclose(self.normalizer.normalize_input(np.array([1+1e-8]*6)), 1)

    def test_warmup_resets_invalid_data_and_unknown_gt(self):
        node = self.node()
        with patch.object(rospy, "loginfo_throttle"), patch.object(rospy, "logerr_throttle"):
            node.observation_cb(self.observation(10))
            self.assertFalse(node.pred_pub.messages[-1].valid)
            node.observation_cb(self.observation(10.1))
            self.assertTrue(node.pred_pub.messages[-1].valid)
            self.assertEqual(node.pred_pub.messages[-1].output_values, [3.])
            self.assertIn("GT: N/A", node.marker_pub.messages[-1].markers[0].text)
            self.assertEqual(node.marker_pub.messages[-1].markers[0].header.frame_id, "rto/base_link")
            self.assertEqual(node.marker_pub.messages[-1].markers[0].id, 7)
            node.observation_cb(self.observation(11))
            self.assertFalse(node.pred_pub.messages[-1].valid)  # timestamp gap
            node.observation_cb(self.observation(9))
            self.assertFalse(node.pred_pub.messages[-1].valid)  # rewind
            bad = self.observation(9.1)
            bad.feature_values[0] = float("nan")
            node.observation_cb(bad)
            self.assertFalse(node.pred_pub.messages[-1].valid)
            node.observation_cb(self.observation(9.2))
            self.assertFalse(node.pred_pub.messages[-1].valid)  # fresh warmup
            node.observation_cb(self.observation(9.3, model=8))
            self.assertFalse(node.pred_pub.messages[-1].valid)  # distinct identity
            right = self.observation(9.4); right.side = "right"
            count = len(node.pred_pub.messages)
            node.observation_cb(right)
            self.assertEqual(len(node.pred_pub.messages), count)

    def test_stats_reject_incompatible_model(self):
        self.stats["input_features"] = ["x", "y"]
        self.stats_path.write_text(json.dumps(self.stats))
        with self.assertRaises(RuntimeError):
            inference.Normalizer(str(self.stats_path))

    def test_training_omits_unknown_targets_and_splits_time_gaps(self):
        rows = []
        for index, stamp in enumerate([10, 10.1, 11, 11.1, 12]):
            row = dict(sample_idx=index, stamp=stamp, model_id=7, side="left",
                       gt_sector_density=2 if index < 4 else float("nan"))
            row.update({name: float(index) for name in training.INPUT_FEATURES})
            rows.append(row)
        csv = Path(self.directory.name) / "density.csv"
        pd.DataFrame(rows).to_csv(csv, index=False)
        dataset = training.GapSectorDensitySequenceDataset(str(csv), seq_len=2)
        self.assertEqual(len(dataset), 2)
        self.assertTrue(np.isfinite(dataset.samples_x).all())
        self.assertTrue(np.isfinite(dataset.samples_y).all())


if __name__ == "__main__":
    unittest.main()
