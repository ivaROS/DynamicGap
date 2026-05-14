#!/usr/bin/env python3

import json
from collections import defaultdict, deque

import numpy as np
import rospy
import torch

from dynamic_gap.msg import GapFeatureObservation
from dynamic_gap.msg import GapFeaturePrediction


class Normalizer:
    def __init__(self, stats_path):
        self.input_mean = None
        self.input_std = None
        self.target_mean = None
        self.target_std = None

        self.input_features = None
        self.output_features = None

        if not stats_path:
            raise RuntimeError(
                "stats_path is required for this generic feature node "
                "because it defines the expected input/output feature names."
            )

        with open(stats_path, "r") as f:
            stats = json.load(f)

        rospy.loginfo("Loaded normalization stats keys: %s", list(stats.keys()))

        self.input_mean = self._get_array(
            stats,
            ["input_mean", "x_mean", "X_mean", "feature_mean", "features_mean"]
        )
        self.input_std = self._get_array(
            stats,
            ["input_std", "x_std", "X_std", "feature_std", "features_std"]
        )
        self.target_mean = self._get_array(
            stats,
            ["target_mean", "y_mean", "Y_mean", "label_mean", "output_mean"]
        )
        self.target_std = self._get_array(
            stats,
            ["target_std", "y_std", "Y_std", "label_std", "output_std"]
        )

        self.input_features = stats.get("input_features", None)
        self.output_features = stats.get("output_features", None)

        if self.input_features is None:
            raise RuntimeError(
                "Stats JSON is missing 'input_features'. "
                "Cannot know which incoming features the model expects."
            )

        if self.output_features is None:
            raise RuntimeError(
                "Stats JSON is missing 'output_features'. "
                "Cannot name model outputs."
            )

        if self.input_mean is None or self.input_std is None:
            raise RuntimeError(
                "Input normalization stats were not found in stats JSON."
            )

        if self.target_mean is None or self.target_std is None:
            raise RuntimeError(
                "Target normalization stats were not found in stats JSON."
            )

        rospy.loginfo("Expected model input features: %s", self.input_features)
        rospy.loginfo("Expected model output features: %s", self.output_features)

    def _get_array(self, stats, keys):
        for key in keys:
            if key in stats:
                return np.array(stats[key], dtype=np.float32)
        return None

    def normalize_input(self, x):
        return (x - self.input_mean) / (self.input_std + 1.0e-8)

    def denormalize_target(self, y):
        return y * (self.target_std + 1.0e-8) + self.target_mean


class GRUGapFeatureNode:
    def __init__(self):
        rospy.init_node("gru_gap_feature_node")

        self.model_path = rospy.get_param("~model_path")
        self.stats_path = rospy.get_param("~stats_path")

        self.seq_len = int(rospy.get_param("~seq_len", 10))
        self.device_name = rospy.get_param("~device", "cpu")
        self.device = torch.device(self.device_name)

        self.observation_topic = rospy.get_param(
            "~observation_topic",
            "gap_feature_observation",
        )

        self.prediction_topic = rospy.get_param(
            "~prediction_topic",
            "gru_gap_feature_prediction",
        )

        self.max_buffer_time_gap = float(
            rospy.get_param("~max_buffer_time_gap", 0.5)
        )

        self.normalizer = Normalizer(self.stats_path)

        self.input_features = self.normalizer.input_features
        self.output_features = self.normalizer.output_features

        self.buffers = defaultdict(lambda: deque(maxlen=self.seq_len))
        self.last_stamp_by_key = {}

        self.model = self.load_model(self.model_path)

        self.pred_pub = rospy.Publisher(
            self.prediction_topic,
            GapFeaturePrediction,
            queue_size=10,
        )

        self.obs_sub = rospy.Subscriber(
            self.observation_topic,
            GapFeatureObservation,
            self.observation_cb,
            queue_size=100,
        )

        rospy.loginfo("GRU generic gap feature node ready.")
        rospy.loginfo("Subscribing to: %s", self.observation_topic)
        rospy.loginfo("Publishing predictions to: %s", self.prediction_topic)
        rospy.loginfo("seq_len: %d", self.seq_len)

    def load_model(self, model_path):
        # ////////////////////////////////////////////////////////////
        # Saved models from your training script are TorchScript.
        # ////////////////////////////////////////////////////////////

        try:
            model = torch.jit.load(
                model_path,
                map_location=self.device,
            )
            model.to(self.device)
            model.eval()

            rospy.loginfo("Loaded TorchScript GRU model from: %s", model_path)
            return model

        except Exception as e:
            raise RuntimeError(
                "Failed to load TorchScript model from '{}': {}".format(
                    model_path,
                    str(e),
                )
            )

    def make_key(self, msg):
        return "{}_{}".format(msg.model_id, msg.side)

    def maybe_clear_stale_buffer(self, key, stamp):
        if key not in self.last_stamp_by_key:
            self.last_stamp_by_key[key] = stamp
            return

        dt = (stamp - self.last_stamp_by_key[key]).to_sec()

        if dt < 0.0 or dt > self.max_buffer_time_gap:
            self.buffers[key].clear()

        self.last_stamp_by_key[key] = stamp

    def incoming_feature_dict(self, msg):
        if len(msg.feature_names) != len(msg.feature_values):
            raise RuntimeError(
                "Incoming feature_names and feature_values have different lengths."
            )

        return {
            name: float(value)
            for name, value in zip(msg.feature_names, msg.feature_values)
        }

    def build_model_input_sample(self, msg):
        feature_dict = self.incoming_feature_dict(msg)

        missing = [
            name
            for name in self.input_features
            if name not in feature_dict
        ]

        if len(missing) > 0:
            raise RuntimeError(
                "Incoming GapFeatureObservation is missing model-required "
                "features: {}".format(missing)
            )

        return np.array(
            [
                feature_dict[name]
                for name in self.input_features
            ],
            dtype=np.float32,
        )

    def observation_cb(self, msg):
        try:
            # //////////////////////////////////////////////////////
            # // The planner should already publish left only,
            # // but this avoids accidental duplicate use.
            # //////////////////////////////////////////////////////

            if msg.side != "left":
                return

            key = self.make_key(msg)
            self.maybe_clear_stale_buffer(key, msg.header.stamp)

            sample = self.build_model_input_sample(msg)
            self.buffers[key].append(sample)

            if len(self.buffers[key]) < self.seq_len:
                self.publish_invalid_prediction(
                    msg,
                    len(self.buffers[key]),
                )
                return

            seq = np.stack(
                list(self.buffers[key]),
                axis=0,
            ).astype(np.float32)

            seq = self.normalizer.normalize_input(seq)

            x = torch.from_numpy(seq).unsqueeze(0).to(self.device)

            with torch.no_grad():
                pred = self.model(x).cpu().numpy()[0]

            pred = self.normalizer.denormalize_target(pred)

            output_values = [
                float(value)
                for value in np.atleast_1d(pred)
            ]

            self.publish_prediction(
                msg,
                output_values,
                True,
                self.seq_len,
            )

        except Exception as e:
            rospy.logerr_throttle(
                1.0,
                "GRU feature node observation_cb error: %s",
                str(e),
            )

    def publish_invalid_prediction(self, obs_msg, seq_len_used):
        pred_msg = GapFeaturePrediction()
        pred_msg.header = obs_msg.header

        pred_msg.gap_index = obs_msg.gap_index
        pred_msg.model_id = obs_msg.model_id
        pred_msg.side = obs_msg.side

        pred_msg.output_names = list(self.output_features)
        pred_msg.output_values = [
            0.0 for _ in self.output_features
        ]

        pred_msg.valid = False
        pred_msg.seq_len_used = seq_len_used

        self.pred_pub.publish(pred_msg)

    def publish_prediction(
        self,
        obs_msg,
        output_values,
        valid,
        seq_len_used,
    ):
        pred_msg = GapFeaturePrediction()
        pred_msg.header = obs_msg.header

        pred_msg.gap_index = obs_msg.gap_index
        pred_msg.model_id = obs_msg.model_id
        pred_msg.side = obs_msg.side

        pred_msg.output_names = list(self.output_features)
        pred_msg.output_values = output_values

        pred_msg.valid = valid
        pred_msg.seq_len_used = seq_len_used

        self.pred_pub.publish(pred_msg)

        printable_outputs = ", ".join(
            [
                "{}={:.6f}".format(name, value)
                for name, value in zip(
                    self.output_features,
                    output_values,
                )
            ]
        )

        rospy.loginfo_throttle(
            0.5,
            "GRU feature prediction | gap_index=%d model_id=%d %s",
            obs_msg.gap_index,
            obs_msg.model_id,
            printable_outputs,
        )

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = GRUGapFeatureNode()
    node.spin()