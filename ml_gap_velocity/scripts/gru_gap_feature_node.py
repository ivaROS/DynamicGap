#!/usr/bin/env python3

import json
from collections import defaultdict, deque

import numpy as np
import rospy
import torch

from dynamic_gap.msg import GapFeatureObservation
from dynamic_gap.msg import GapFeaturePrediction
from visualization_msgs.msg import Marker, MarkerArray


# ================================================================
# BASE_INPUT_FEATURES
#
# These are the features that must arrive in the incoming
# GapFeatureObservation ROS message (via feature_names /
# feature_values). They map 1-to-1 to what the planner publishes.
#
# Any feature NOT in this list is a DERIVED feature, meaning the
# node computes it locally from the base features or from buffer
# history. The node figures out which derived features to compute
# by reading `input_features` from the stats JSON — no manual
# configuration required.
# ================================================================
BASE_INPUT_FEATURES = [
    "sector_density",
    "sector_dynamic_raw_gap_point_count",
    "contained_raw_gap_point_count",
    "sector_area",
    "sector_angle_rad",
    "sector_radius",
]

# Minimum angle used when computing aspect_ratio to prevent
# division-by-zero or extreme values. Must match the training script.
MIN_SECTOR_ANGLE_RAD = 1e-4


class Normalizer:
    def __init__(self, stats_path):
        self.input_mean = None
        self.input_std  = None
        self.target_mean = None
        self.target_std  = None

        self.input_features  = None
        self.output_features = None

        if not stats_path:
            raise RuntimeError(
                "stats_path is required — it defines the expected "
                "input/output feature names and normalization constants."
            )

        with open(stats_path, "r") as f:
            stats = json.load(f)

        rospy.loginfo("Loaded normalization stats keys: %s", list(stats.keys()))

        self.input_mean  = self._get_array(stats, ["input_mean",  "x_mean",  "X_mean",  "feature_mean",  "features_mean"])
        self.input_std   = self._get_array(stats, ["input_std",   "x_std",   "X_std",   "feature_std",   "features_std"])
        self.target_mean = self._get_array(stats, ["target_mean", "y_mean",  "Y_mean",  "label_mean",    "output_mean"])
        self.target_std  = self._get_array(stats, ["target_std",  "y_std",   "Y_std",   "label_std",     "output_std"])

        self.input_features  = stats.get("input_features",  None)
        self.output_features = stats.get("output_features", None)

        if self.input_features is None:
            raise RuntimeError(
                "Stats JSON is missing 'input_features'. "
                "Cannot know which features the model expects."
            )
        if self.output_features is None:
            raise RuntimeError(
                "Stats JSON is missing 'output_features'. "
                "Cannot name model outputs."
            )
        if self.input_mean is None or self.input_std is None:
            raise RuntimeError(
                "Input normalization stats not found in stats JSON."
            )
        if self.target_mean is None or self.target_std is None:
            raise RuntimeError(
                "Target normalization stats not found in stats JSON."
            )

        rospy.loginfo("Model input features:  %s", self.input_features)
        rospy.loginfo("Model output features: %s", self.output_features)

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

        self.model_path  = rospy.get_param("~model_path")
        self.stats_path  = rospy.get_param("~stats_path")
        self.seq_len     = int(rospy.get_param("~seq_len", 10))
        self.device_name = rospy.get_param("~device", "cpu")
        self.device      = torch.device(self.device_name)

        self.observation_topic = rospy.get_param("~observation_topic", "gap_feature_observation")
        self.prediction_topic  = rospy.get_param("~prediction_topic",  "gru_gap_feature_prediction")
        self.marker_topic      = rospy.get_param("~marker_topic",      "gru_gap_feature_prediction_markers")

        self.text_marker_height = float(rospy.get_param("~text_marker_height", 0.28))
        self.text_marker_z      = float(rospy.get_param("~text_marker_z",      0.35))
        self.max_buffer_time_gap = float(rospy.get_param("~max_buffer_time_gap", 0.5))

        self.normalizer = Normalizer(self.stats_path)

        self.input_features  = self.normalizer.input_features
        self.output_features = self.normalizer.output_features

        # ============================================================
        # AUTO-DETECT DERIVED FEATURES
        #
        # We check which features in `input_features` are NOT in the
        # base set. Those must be computed locally rather than read
        # from the incoming ROS message.
        #
        # This means: whichever flags were set at training time are
        # automatically honoured here, with zero manual configuration.
        # If you trained with --use-density-rate, the stats JSON will
        # have "density_rate_of_change" in input_features, and the
        # node will compute it. If you didn't, it won't.
        # ============================================================
        self.use_density_rate = "density_rate_of_change" in self.input_features
        self.use_aspect_ratio = "aspect_ratio"           in self.input_features

        # Index of sector_density in the raw (unnormalized) buffer
        # entries, needed to compute density_rate_of_change at runtime.
        # We look it up once here to avoid repeated list searches.
        self._density_idx_in_base = BASE_INPUT_FEATURES.index("sector_density")

        # Index of sector_angle_rad in the base feature list, needed
        # to compute aspect_ratio at runtime.
        self._angle_idx_in_base = BASE_INPUT_FEATURES.index("sector_angle_rad")

        rospy.loginfo("use_density_rate: %s", self.use_density_rate)
        rospy.loginfo("use_aspect_ratio: %s", self.use_aspect_ratio)

        # Buffer stores raw (unnormalized) full feature vectors per
        # model_id+side key. Normalization happens on the full sequence
        # just before inference.
        self.buffers = defaultdict(lambda: deque(maxlen=self.seq_len))
        self.last_stamp_by_key = {}

        self.model = self.load_model(self.model_path)

        self.pred_pub   = rospy.Publisher(self.prediction_topic, GapFeaturePrediction, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic,     MarkerArray,          queue_size=10)

        self.obs_sub = rospy.Subscriber(
            self.observation_topic,
            GapFeatureObservation,
            self.observation_cb,
            queue_size=100,
        )

        rospy.loginfo("GRU gap feature node ready.")
        rospy.loginfo("Subscribing to:  %s", self.observation_topic)
        rospy.loginfo("Publishing to:   %s", self.prediction_topic)
        rospy.loginfo("seq_len:         %d", self.seq_len)
        rospy.loginfo("input_features:  %s", self.input_features)

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------

    def stable_marker_id(self, msg):
        return abs(hash("{}_{}".format(msg.model_id, msg.side))) % 1000000

    def get_named_output_value(self, output_values, desired_name):
        if desired_name not in self.output_features:
            return None
        idx = self.output_features.index(desired_name)
        if idx >= len(output_values):
            return None
        return float(output_values[idx])

    def get_named_target_value(self, obs_msg, desired_name):
        if len(obs_msg.target_names) != len(obs_msg.target_values):
            return None
        target_dict = {
            name: float(value)
            for name, value in zip(obs_msg.target_names, obs_msg.target_values)
        }
        return target_dict.get(desired_name, None)

    def load_model(self, model_path):
        try:
            model = torch.jit.load(model_path, map_location=self.device)
            model.to(self.device)
            model.eval()
            rospy.loginfo("Loaded TorchScript model from: %s", model_path)
            return model
        except Exception as e:
            raise RuntimeError(
                "Failed to load TorchScript model from '{}': {}".format(model_path, str(e))
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

    # ----------------------------------------------------------------
    # Core: build one model input sample from an incoming message.
    #
    # prev_sample is the last raw feature vector sitting in the buffer
    # (i.e. the previous timestep for this gap track). It is passed in
    # so density_rate_of_change can be computed without the buffer
    # having to be queried again.
    # ----------------------------------------------------------------

    def build_model_input_sample(self, msg, prev_sample=None):
        """
        Returns a 1-D float32 numpy array of length len(input_features),
        in the exact order the model was trained on.

        Base features come from the incoming message.
        Derived features are computed here:

        density_rate_of_change
            = current sector_density - previous sector_density
            If there is no previous sample (first observation for this
            gap track), we use 0.0, consistent with the training script
            which fills NaN diffs with 0.0 for the first row in a group.

        aspect_ratio
            = 1.0 / sector_angle_rad  (clipped to MIN_SECTOR_ANGLE_RAD)
            Purely geometric — requires no history.
        """
        feature_dict = self.incoming_feature_dict(msg)

        # Validate that all base features are present in the message.
        missing = [
            name for name in BASE_INPUT_FEATURES
            if name not in feature_dict
        ]
        if len(missing) > 0:
            raise RuntimeError(
                "Incoming GapFeatureObservation is missing base features: {}".format(missing)
            )

        # --------------------------------------------------------
        # Compute density_rate_of_change if this model uses it.
        #
        # prev_sample is the raw (unnormalized) full feature vector
        # from the previous timestep. We index into it using the
        # pre-computed _density_idx_in_base, which is the position
        # of sector_density in BASE_INPUT_FEATURES.
        #
        # Why this approach rather than recomputing from feature_dict?
        # The buffer already holds the full prior vector so we don't
        # need to re-parse the message. We just grab the relevant slot.
        # --------------------------------------------------------
        if self.use_density_rate:
            if prev_sample is not None:
                prev_density = float(prev_sample[self._density_idx_in_base])
                curr_density = feature_dict["sector_density"]
                feature_dict["density_rate_of_change"] = curr_density - prev_density
            else:
                # First observation for this gap track — no previous
                # density available. Use 0.0 (no change), consistent
                # with the training script's .fillna(0.0).
                feature_dict["density_rate_of_change"] = 0.0

        # --------------------------------------------------------
        # Compute aspect_ratio if this model uses it.
        #
        # aspect_ratio = 1 / sector_angle_rad.
        # Clipped to MIN_SECTOR_ANGLE_RAD to match training and to
        # prevent extreme values for nearly-zero-angle sectors.
        # --------------------------------------------------------
        if self.use_aspect_ratio:
            angle = max(
                feature_dict.get("sector_angle_rad", MIN_SECTOR_ANGLE_RAD),
                MIN_SECTOR_ANGLE_RAD,
            )
            feature_dict["aspect_ratio"] = 1.0 / angle

        # Build the array in the exact feature order the model expects.
        return np.array(
            [feature_dict[name] for name in self.input_features],
            dtype=np.float32,
        )

    # ----------------------------------------------------------------
    # ROS callback
    # ----------------------------------------------------------------

    def observation_cb(self, msg):
        try:
            # The planner should publish left only, but guard here too.
            if msg.side != "left":
                return

            key = self.make_key(msg)
            self.maybe_clear_stale_buffer(key, msg.header.stamp)

            # --------------------------------------------------------
            # Grab the previous sample BEFORE appending the new one.
            #
            # This is critical for density_rate_of_change: we need
            # ρ_{t-1} from the buffer, which is the last entry before
            # we push ρ_t. After append, buffer[-1] would be the
            # current sample, making the diff meaningless.
            # --------------------------------------------------------
            prev_sample = (
                self.buffers[key][-1]
                if len(self.buffers[key]) > 0
                else None
            )

            sample = self.build_model_input_sample(msg, prev_sample=prev_sample)
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

            output_values = [float(v) for v in np.atleast_1d(pred)]

            self.publish_prediction(msg, output_values, True, self.seq_len)

        except Exception as e:
            rospy.logerr_throttle(
                1.0,
                "GRU feature node observation_cb error: %s",
                str(e),
            )

    # ----------------------------------------------------------------
    # Publishers
    # ----------------------------------------------------------------

    def publish_prediction_marker(self, obs_msg, output_values):
        pred_density = self.get_named_output_value(output_values, "gt_sector_density")
        gt_density   = self.get_named_target_value(obs_msg, "gt_sector_density")

        if pred_density is None:
            return

        marker_array = MarkerArray()
        marker = Marker()
        marker.header = obs_msg.header
        marker.ns     = "gru_sector_density_text"
        marker.id     = self.stable_marker_id(obs_msg)
        marker.type   = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x  = obs_msg.vis_x
        marker.pose.position.y  = obs_msg.vis_y
        marker.pose.position.z  = self.text_marker_z
        marker.pose.orientation.w = 1.0

        marker.scale.z  = self.text_marker_height
        marker.color.g  = 1.0
        marker.color.a  = 1.0

        if gt_density is None:
            marker.text = "gap {} | GRU: {:.3f} | GT: N/A".format(
                obs_msg.gap_index, pred_density,
            )
        else:
            marker.text = "gap {} | GRU: {:.3f} | GT: {:.3f}".format(
                obs_msg.gap_index, pred_density, gt_density,
            )

        marker.lifetime = rospy.Duration(0.35)
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_invalid_prediction(self, obs_msg, seq_len_used):
        pred_msg = GapFeaturePrediction()
        pred_msg.header       = obs_msg.header
        pred_msg.gap_index    = obs_msg.gap_index
        pred_msg.model_id     = obs_msg.model_id
        pred_msg.side         = obs_msg.side
        pred_msg.output_names  = list(self.output_features)
        pred_msg.output_values = [0.0 for _ in self.output_features]
        pred_msg.valid         = False
        pred_msg.seq_len_used  = seq_len_used
        self.pred_pub.publish(pred_msg)

    def publish_prediction(self, obs_msg, output_values, valid, seq_len_used):
        pred_msg = GapFeaturePrediction()
        pred_msg.header       = obs_msg.header
        pred_msg.gap_index    = obs_msg.gap_index
        pred_msg.model_id     = obs_msg.model_id
        pred_msg.side         = obs_msg.side
        pred_msg.output_names  = list(self.output_features)
        pred_msg.output_values = output_values
        pred_msg.valid         = valid
        pred_msg.seq_len_used  = seq_len_used
        self.pred_pub.publish(pred_msg)
        self.publish_prediction_marker(obs_msg, output_values)

        printable_outputs = ", ".join(
            "{}={:.6f}".format(n, v)
            for n, v in zip(self.output_features, output_values)
        )
        gt_density = self.get_named_target_value(obs_msg, "gt_sector_density")
        gt_text = (
            "gt_sector_density=N/A" if gt_density is None
            else "gt_sector_density={:.6f}".format(gt_density)
        )
        rospy.loginfo_throttle(
            0.5,
            "GRU prediction | gap_index=%d model_id=%d %s %s",
            obs_msg.gap_index,
            obs_msg.model_id,
            printable_outputs,
            gt_text,
        )

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = GRUGapFeatureNode()
    node.spin()