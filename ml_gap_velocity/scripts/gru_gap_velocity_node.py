#!/usr/bin/env python3

import json
from collections import defaultdict, deque

import numpy as np
import rospy
import torch
import torch.nn as nn

from dynamic_gap.msg import GapPointObservation, GapVelocityPrediction


class GapGRU(nn.Module):
    def __init__(self, input_size=2, hidden_size=64, num_layers=2, output_size=2):
        super().__init__()
        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
        )
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        output, _ = self.gru(x)
        return self.fc(output[:, -1, :])


class Normalizer:
    def __init__(self, stats_path):
        self.input_mean = None
        self.input_std = None
        self.target_mean = None
        self.target_std = None

        if not stats_path:
            rospy.logwarn("No stats_path provided. Running without normalization.")
            return

        with open(stats_path, "r") as stats_file:
            stats = json.load(stats_file)

        self.input_mean = self._get_array(
            stats,
            ["input_mean", "x_mean", "X_mean", "feature_mean", "features_mean"],
        )
        self.input_std = self._get_array(
            stats,
            ["input_std", "x_std", "X_std", "feature_std", "features_std"],
        )
        self.target_mean = self._get_array(
            stats,
            ["target_mean", "y_mean", "Y_mean", "label_mean", "output_mean"],
        )
        self.target_std = self._get_array(
            stats,
            ["target_std", "y_std", "Y_std", "label_std", "output_std"],
        )

        if self.input_mean is None or self.input_std is None:
            rospy.logwarn("Input normalization stats not found; inputs will not be normalized.")
        if self.target_mean is None or self.target_std is None:
            rospy.logwarn("Target normalization stats not found; outputs will not be denormalized.")

    @staticmethod
    def _get_array(stats, keys):
        for key in keys:
            if key in stats:
                return np.asarray(stats[key], dtype=np.float32)
        return None

    def normalize_input(self, value):
        if self.input_mean is None or self.input_std is None:
            return value
        return (value - self.input_mean) / (self.input_std + 1.0e-8)

    def denormalize_target(self, value):
        if self.target_mean is None or self.target_std is None:
            return value
        return value * (self.target_std + 1.0e-8) + self.target_mean


class GRUGapVelocityNode:
    def __init__(self):
        rospy.init_node("gru_gap_velocity_node")

        self.model_path = rospy.get_param("~model_path")
        self.stats_path = rospy.get_param("~stats_path", "")

        self.seq_len = int(rospy.get_param("~seq_len", 10))
        self.input_size = int(rospy.get_param("~input_size", 2))
        self.hidden_size = int(rospy.get_param("~hidden_size", 64))
        self.num_layers = int(rospy.get_param("~num_layers", 2))
        self.output_size = int(rospy.get_param("~output_size", 2))

        self.device = torch.device(rospy.get_param("~device", "cpu"))

        self.observation_topic = rospy.get_param(
            "~observation_topic", "gap_point_observation"
        )
        self.prediction_topic = rospy.get_param(
            "~prediction_topic", "gru_gap_velocity_prediction"
        )
        self.max_buffer_time_gap = float(
            rospy.get_param("~max_buffer_time_gap", 0.5)
        )

        self.buffers = defaultdict(lambda: deque(maxlen=self.seq_len))
        self.last_stamp_by_key = {}
        self.normalizer = Normalizer(self.stats_path)

        self.model = GapGRU(
            input_size=self.input_size,
            hidden_size=self.hidden_size,
            num_layers=self.num_layers,
            output_size=self.output_size,
        ).to(self.device)
        self._load_model()

        self.prediction_publisher = rospy.Publisher(
            self.prediction_topic, GapVelocityPrediction, queue_size=10
        )
        self.observation_subscriber = rospy.Subscriber(
            self.observation_topic,
            GapPointObservation,
            self.observation_callback,
            queue_size=100,
        )

        rospy.loginfo("GRU gap velocity node ready")
        rospy.loginfo("Subscribing to: %s", self.observation_topic)
        rospy.loginfo("Publishing predictions to: %s", self.prediction_topic)

    def _load_model(self):
        try:
            self.model = torch.jit.load(self.model_path, map_location=self.device)
            self.model.to(self.device)
            self.model.eval()
            rospy.loginfo("Loaded TorchScript GRU model from: %s", self.model_path)
            return
        except Exception as error:
            rospy.logwarn(
                "Could not load model as TorchScript; trying state_dict: %s", error
            )

        checkpoint = torch.load(self.model_path, map_location=self.device)
        if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
            state_dict = checkpoint["model_state_dict"]
        elif isinstance(checkpoint, dict):
            state_dict = checkpoint
        else:
            raise RuntimeError(
                "Model file is neither a TorchScript model nor a state_dict checkpoint"
            )

        self.model.load_state_dict(state_dict)
        self.model.to(self.device)
        self.model.eval()
        rospy.loginfo("Loaded state_dict GRU model from: %s", self.model_path)

    @staticmethod
    def _key(message):
        return message.model_id, message.side

    def _clear_stale_buffer(self, key, stamp):
        previous_stamp = self.last_stamp_by_key.get(key)
        if previous_stamp is not None:
            time_gap = (stamp - previous_stamp).to_sec()
            if time_gap < 0.0 or time_gap > self.max_buffer_time_gap:
                self.buffers[key].clear()
        self.last_stamp_by_key[key] = stamp

    def observation_callback(self, message):
        key = self._key(message)
        self._clear_stale_buffer(key, message.header.stamp)
        self.buffers[key].append(
            np.asarray([message.gap_x, message.gap_y], dtype=np.float32)
        )

        if len(self.buffers[key]) < self.seq_len:
            self._publish_prediction(message, 0.0, 0.0, False, len(self.buffers[key]))
            return

        sequence = np.stack(self.buffers[key], axis=0).astype(np.float32)
        sequence = self.normalizer.normalize_input(sequence)
        tensor = torch.from_numpy(sequence).unsqueeze(0).to(self.device)

        with torch.no_grad():
            prediction = self.model(tensor).cpu().numpy()[0]
        prediction = self.normalizer.denormalize_target(prediction)

        pred_vx = float(prediction[0])
        pred_vy = float(prediction[1])
        self._publish_prediction(message, pred_vx, pred_vy, True, self.seq_len)

    def _publish_prediction(
        self, observation, pred_vx, pred_vy, valid, seq_len_used
    ):
        prediction = GapVelocityPrediction()
        prediction.header = observation.header
        prediction.gap_index = observation.gap_index
        prediction.model_id = observation.model_id
        prediction.side = observation.side
        prediction.gap_x = observation.gap_x
        prediction.gap_y = observation.gap_y
        prediction.pred_rel_vx = pred_vx
        prediction.pred_rel_vy = pred_vy
        prediction.valid = valid
        prediction.seq_len_used = seq_len_used
        self.prediction_publisher.publish(prediction)

    @staticmethod
    def spin():
        rospy.spin()


if __name__ == "__main__":
    GRUGapVelocityNode().spin()
