#!/usr/bin/env python3

import math
import numpy as np
import rospy
import skfmm

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class FmmDistanceNode:
    def __init__(self):
        self.map_msg = None
        self.goal_msg = None

        self.occ_threshold = rospy.get_param("~occ_threshold", 50)
        self.treat_unknown_as_obstacle = rospy.get_param("~treat_unknown_as_obstacle", True)

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb, queue_size=1)
        self.goal_sub = rospy.Subscriber("/dgap/fmm_goal", PoseStamped, self.goal_cb, queue_size=1)

        self.fmm_pub = rospy.Publisher("/dgap/fmm_distance_map", Float32MultiArray, queue_size=1, latch=True)

    def map_cb(self, msg):
        self.map_msg = msg
        self.try_compute()

    def goal_cb(self, msg):
        self.goal_msg = msg
        self.try_compute()

    def world_to_grid(self, x, y):
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        res = self.map_msg.info.resolution

        col = int((x - origin_x) / res)
        row = int((y - origin_y) / res)

        h = self.map_msg.info.height
        w = self.map_msg.info.width

        if row < 0 or col < 0 or row >= h or col >= w:
            return None

        return row, col

    def occupancy_to_binary_grid(self):
        h = self.map_msg.info.height
        w = self.map_msg.info.width

        occ = np.array(self.map_msg.data, dtype=np.int16).reshape((h, w))

        if self.treat_unknown_as_obstacle:
            grid = np.where((occ < 0) | (occ >= self.occ_threshold), 1, 0)
        else:
            grid = np.where(occ >= self.occ_threshold, 1, 0)

        return grid.astype(np.uint8)

    def compute_fmm_distance_map(self, grid, goal_row, goal_col, resolution):
        height, width = grid.shape

        if not (0 <= goal_row < height and 0 <= goal_col < width):
            raise ValueError("Goal is out of bounds")

        if grid[goal_row, goal_col] == 1:
            raise ValueError("Goal is inside an obstacle")

        phi = np.ones((height, width), dtype=np.float64)
        phi[goal_row, goal_col] = -1.0

        phi_masked = np.ma.MaskedArray(phi, mask=grid.astype(bool))

        dists = skfmm.distance(phi_masked, dx=resolution)

        if isinstance(dists, np.ma.MaskedArray):
            dists = dists.filled(np.inf)

        return dists.astype(np.float32)

    def try_compute(self):
        if self.map_msg is None or self.goal_msg is None:
            return

        goal_x = self.goal_msg.pose.position.x
        goal_y = self.goal_msg.pose.position.y

        grid_rc = self.world_to_grid(goal_x, goal_y)
        if grid_rc is None:
            rospy.logwarn("FMM goal outside map")
            return

        goal_row, goal_col = grid_rc
        grid = self.occupancy_to_binary_grid()

        try:
            dists = self.compute_fmm_distance_map(
                grid,
                goal_row,
                goal_col,
                self.map_msg.info.resolution,
            )
        except Exception as e:
            rospy.logwarn(f"Could not compute FMM distance map: {e}")
            return

        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="height", size=self.map_msg.info.height, stride=self.map_msg.info.height * self.map_msg.info.width))
        msg.layout.dim.append(MultiArrayDimension(label="width", size=self.map_msg.info.width, stride=self.map_msg.info.width))
        msg.data = dists.reshape(-1).tolist()

        self.fmm_pub.publish(msg)

        rospy.loginfo(
            f"Published FMM distance map. goal row/col=({goal_row}, {goal_col}), "
            f"size={self.map_msg.info.width}x{self.map_msg.info.height}"
        )


if __name__ == "__main__":
    rospy.init_node("fmm_distance_node")
    FmmDistanceNode()
    rospy.spin()