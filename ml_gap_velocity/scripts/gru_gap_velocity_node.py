#!/usr/bin/env python3

import json
import math
from collections import defaultdict, deque

import numpy as np
import rospy
import torch
import torch.nn as nn

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from dynamic_gap.msg import GapPointObservation
from dynamic_gap.msg import GapVelocityPrediction


class GapGRU(nn.Module):
    def __init__(self, input_size=2, hidden_size=64, num_layers=2, output_size=2):
        super().__init__()

        self.gru = nn.GRU(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True
        )

        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        out, _ = self.gru(x)
        last = out[:, -1, :]
        y = self.fc(last)
        return y


class Normalizer:
    def __init__(self, stats_path):
        self.input_mean = None
        self.input_std = None
        self.target_mean = None
        self.target_std = None

        if not stats_path:
            rospy.logwarn("No stats_path provided. Running without normalization.")
            return

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

        if self.input_mean is None or self.input_std is None:
            rospy.logwarn("Input normalization stats not found. Inputs will not be normalized.")

        if self.target_mean is None or self.target_std is None:
            rospy.logwarn("Target normalization stats not found. Outputs will not be denormalized.")

    def _get_array(self, stats, keys):
        for k in keys:
            if k in stats:
                arr = np.array(stats[k], dtype=np.float32)
                return arr
        return None

    def normalize_input(self, x):
        if self.input_mean is None or self.input_std is None:
            return x

        return (x - self.input_mean) / (self.input_std + 1.0e-8)

    def denormalize_target(self, y):
        if self.target_mean is None or self.target_std is None:
            return y

        return y * (self.target_std + 1.0e-8) + self.target_mean


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

        self.device_name = rospy.get_param("~device", "cpu")
        self.device = torch.device(self.device_name)

        self.observation_topic = rospy.get_param("~observation_topic", "gap_point_observation")
        self.prediction_topic = rospy.get_param("~prediction_topic", "gru_gap_velocity_prediction")
        self.marker_topic = rospy.get_param("~marker_topic", "gru_gap_velocity_markers")

        self.arrow_scale = float(rospy.get_param("~arrow_scale", 0.8))
        self.kalman_arrow_scale = float(rospy.get_param("~kalman_arrow_scale", 0.8))
        self.perfect_arrow_scale = float(rospy.get_param("~perfect_arrow_scale", 0.8))

        self.publish_kalman_marker = bool(rospy.get_param("~publish_kalman_marker", False))
        self.publish_perfect_marker = bool(rospy.get_param("~publish_perfect_marker", True))

        self.max_buffer_time_gap = float(rospy.get_param("~max_buffer_time_gap", 0.5))

        self.buffers = defaultdict(lambda: deque(maxlen=self.seq_len))
        self.last_stamp_by_key = {}

        self.normalizer = Normalizer(self.stats_path)

        self.model = GapGRU(
            input_size=self.input_size,
            hidden_size=self.hidden_size,
            num_layers=self.num_layers,
            output_size=self.output_size
        ).to(self.device)

        self.load_model(self.model_path)

        self.pred_pub = rospy.Publisher(
            self.prediction_topic,
            GapVelocityPrediction,
            queue_size=10
        )

        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=10
        )

        self.obs_sub = rospy.Subscriber(
            self.observation_topic,
            GapPointObservation,
            self.observation_cb,
            queue_size=100
        )

        rospy.loginfo("GRU gap velocity node ready.")
        rospy.loginfo("Subscribing to: %s", self.observation_topic)
        rospy.loginfo("Publishing predictions to: %s", self.prediction_topic)
        rospy.loginfo("Publishing markers to: %s", self.marker_topic)


    def load_model(self, model_path):
        # First try TorchScript, because your saved .pt is a TorchScript archive.
        try:
            self.model = torch.jit.load(model_path, map_location=self.device)
            self.model.to(self.device)
            self.model.eval()
            rospy.loginfo("Loaded TorchScript GRU model from: %s", model_path)
            return
        except Exception as e:
            rospy.logwarn("Could not load as TorchScript, trying state_dict format. Error was: %s", str(e))

        # Fallback for normal PyTorch state_dict checkpoints.
        checkpoint = torch.load(model_path, map_location=self.device)

        if isinstance(checkpoint, dict) and "model_state_dict" in checkpoint:
            state_dict = checkpoint["model_state_dict"]
        elif isinstance(checkpoint, dict):
            state_dict = checkpoint
        else:
            raise RuntimeError(
                "Model file is neither a TorchScript model nor a state_dict checkpoint."
            )

        self.model.load_state_dict(state_dict)
        self.model.to(self.device)
        self.model.eval()

        rospy.loginfo("Loaded state_dict GRU model from: %s", model_path)

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

    def observation_cb(self, msg):
        key = self.make_key(msg)
        self.maybe_clear_stale_buffer(key, msg.header.stamp)

        sample = np.array([msg.gap_x, msg.gap_y], dtype=np.float32)
        self.buffers[key].append(sample)

        if len(self.buffers[key]) < self.seq_len:
            self.publish_invalid_prediction(msg, len(self.buffers[key]))
            return

        seq = np.stack(list(self.buffers[key]), axis=0).astype(np.float32)
        seq = self.normalizer.normalize_input(seq)

        x = torch.from_numpy(seq).unsqueeze(0).to(self.device)

        with torch.no_grad():
            pred = self.model(x).cpu().numpy()[0]

        pred = self.normalizer.denormalize_target(pred)

        pred_vx = float(pred[0])
        pred_vy = float(pred[1])

        self.publish_prediction(msg, pred_vx, pred_vy, True, self.seq_len)
        self.publish_markers(msg, pred_vx, pred_vy)

    def publish_invalid_prediction(self, obs_msg, seq_len_used):
        pred_msg = GapVelocityPrediction()
        pred_msg.header = obs_msg.header

        pred_msg.gap_index = obs_msg.gap_index
        pred_msg.model_id = obs_msg.model_id
        pred_msg.side = obs_msg.side

        pred_msg.gap_x = obs_msg.gap_x
        pred_msg.gap_y = obs_msg.gap_y

        pred_msg.pred_rel_vx = 0.0
        pred_msg.pred_rel_vy = 0.0

        pred_msg.valid = False
        pred_msg.seq_len_used = seq_len_used

        self.pred_pub.publish(pred_msg)

    def publish_prediction(self, obs_msg, pred_vx, pred_vy, valid, seq_len_used):
        pred_msg = GapVelocityPrediction()
        pred_msg.header = obs_msg.header

        pred_msg.gap_index = obs_msg.gap_index
        pred_msg.model_id = obs_msg.model_id
        pred_msg.side = obs_msg.side

        pred_msg.gap_x = obs_msg.gap_x
        pred_msg.gap_y = obs_msg.gap_y

        pred_msg.pred_rel_vx = pred_vx
        pred_msg.pred_rel_vy = pred_vy

        pred_msg.valid = valid
        pred_msg.seq_len_used = seq_len_used

        self.pred_pub.publish(pred_msg)

    def stable_marker_id(self, msg, offset):
        base = abs(hash("{}_{}".format(msg.model_id, msg.side))) % 100000
        return base + offset

    def make_arrow_marker(self, obs_msg, vx, vy, ns, marker_id, color, scale):
        marker = Marker()
        marker.header = obs_msg.header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point()
        start.x = obs_msg.gap_x
        start.y = obs_msg.gap_y
        start.z = 0.05

        end = Point()
        end.x = obs_msg.gap_x + scale * vx
        end.y = obs_msg.gap_y + scale * vy
        end.z = 0.05

        marker.points.append(start)
        marker.points.append(end)

        marker.scale.x = 0.035
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        marker.color = color
        marker.lifetime = rospy.Duration(0.25)

        return marker

    def publish_markers(self, obs_msg, pred_vx, pred_vy):
        marker_array = MarkerArray()

        gru_color = ColorRGBA()
        gru_color.r = 0.0
        gru_color.g = 0.7
        gru_color.b = 0.0
        gru_color.a = 0.8

        marker_array.markers.append(
            self.make_arrow_marker(
                obs_msg,
                pred_vx,
                pred_vy,
                "gru_relative_velocity",
                self.stable_marker_id(obs_msg, 0),
                gru_color,
                self.arrow_scale
            )
        )

        # if self.publish_kalman_marker: #commented out because I want to use red for the ground truth
        #     kalman_color = ColorRGBA()
        #     kalman_color.r = 1.0
        #     kalman_color.g = 0.0
        #     kalman_color.b = 0.0
        #     kalman_color.a = 0.8

        #     marker_array.markers.append(
        #         self.make_arrow_marker(
        #             obs_msg,
        #             obs_msg.kalman_rel_vx,
        #             obs_msg.kalman_rel_vy,
        #             "kalman_relative_velocity",
        #             self.stable_marker_id(obs_msg, 100000),
        #             kalman_color,
        #             self.kalman_arrow_scale
        #         )
            # )

        if self.publish_perfect_marker:
            perfect_color = ColorRGBA()
            perfect_color.r = 1.0
            perfect_color.g = 0.0
            perfect_color.b = 0.0
            perfect_color.a = 0.8

            marker_array.markers.append(
                self.make_arrow_marker(
                    obs_msg,
                    obs_msg.perfect_rel_vx,
                    obs_msg.perfect_rel_vy,
                    "perfect_relative_velocity",
                    self.stable_marker_id(obs_msg, 200000),
                    perfect_color,
                    self.perfect_arrow_scale
                )
            )

        self.marker_pub.publish(marker_array)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = GRUGapVelocityNode()
    node.spin()