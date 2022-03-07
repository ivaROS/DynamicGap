#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Vector3

import tf_conversions

import tf2_ros
import tf2_geometry_msgs

class Agent:
    def __init__(self, robot_namespace):
        self.robot_namespace = robot_namespace
        rospy.wait_for_service('/move_base_virtual/make_plan')
        self.get_plan = rospy.ServiceProxy('/move_base_virtual/make_plan', GetPlan)
        self.plan_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.plan = None
        self.agent_odoms = None
        self.odom_sub = rospy.Subscriber(robot_namespace + "/odom", Odometry, self.odom_CB, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher(robot_namespace + "/cmd_vel", Twist, queue_size=5)
        self.start = PoseStamped()
        self.start.header.frame_id = "known_map"
        self.goal = PoseStamped()
        self.goal.header.frame_id = "known_map"
        self.goal.pose.position.y = 10
        self.get_global_plan()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def odom_CB(self, msg):
        #print('odom msg header', msg.header)
        position = msg.pose.pose.position
        self.x = position.x
        self.y = position.y
        #print('agent x,y: ', self.x, ', ', self.y)
        desired_pose = self.plan.plan.poses[self.plan_idx]
        # ODOM: map_static
        # DESIRED: known_map
        # CMD_VEL: robot0 frame
        #print('desired pose: ', desired_pose)
        # first is target frame, second is source frame
        map_static_to_known_map_trans = self.tfBuffer.lookup_transform("known_map", "map_static", rospy.Time())
        # print(map_static_to_known_map_trans)
        # transforming from map_static to known_map
        odom_in_known_map = tf2_geometry_msgs.do_transform_pose(msg.pose, map_static_to_known_map_trans)
        x_diff = odom_in_known_map.pose.position.x - desired_pose.pose.position.x
        y_diff = odom_in_known_map.pose.position.y - desired_pose.pose.position.y
        # transforming from known_map to robot0
        #print('x_diff: ', x_diff, ', and y_diff: ', y_diff)
        known_map_to_robot_trans = self.tfBuffer.lookup_transform(robot_namespace, "known_map", rospy.Time())
        diff_vect = PoseStamped()
        diff_vect.pose.position.x = -x_diff
        diff_vect.pose.position.y = -y_diff
        diff_in_robot_0 = tf2_geometry_msgs.do_transform_pose(diff_vect, known_map_to_robot_trans)
        twist = Twist()
        twist.linear.x = self.clip_cmd_vel(diff_in_robot_0.pose.position.x)
        twist.linear.y = self.clip_cmd_vel(diff_in_robot_0.pose.position.y)
        # print('twist: ', twist)
        self.cmd_vel_pub.publish(twist)

        if np.sqrt(x_diff ** 2 + y_diff ** 2) < 0.05:
            self.plan_idx += 1
            if len(self.plan) < self.plan_idx:
                self.plan_idx = 0

        # cap agent velocity

        #    self.start = 0.0
        #    self.goal = 0.0
        #    self.get_global_plan()

    def get_global_plan(self):
        req = GetPlan()
        req.start = self.start
        req.goal = self.goal
        req.tolerance = 0.5
        self.plan = self.get_plan(req.start, req.goal, req.tolerance)
        # print("trying self.plan.respone: ", self.plan.response)
        # print("trying self.plan.plan: ", self.plan.plan)

    def clip_cmd_vel(self, original_val):
        if original_val > 1:
            return 1
        elif original_val < -1:
            return -1
        else:
            return original_val

if __name__ == '__main__':
    try:
        rospy.init_node("follow_global_path", anonymous=True)
        robot_namespace = rospy.get_param("~robot_namespace")
        #print("robot namespace: ", robot_namespace)
        Agent(robot_namespace)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
