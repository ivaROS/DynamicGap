#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Agent:
    def __init__(self):
        rospy.wait_for_service('/move_base/make_plan')
        self.get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.plan_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.plan = None
        self.agent_odoms = None
        self.odom_sub = rospy.Subscriber("robot0/odom", Odometry, self.odom_CB, queue_size=5)
        self.start = PoseStamped()
        self.start.header.frame_id = "known_map"
        self.goal = PoseStamped()
        self.goal.header.frame_id = "known_map"
        self.goal.pose.position.y = 10
        self.get_global_plan()

    def odom_CB(self, msg):
        position = msg.pose.pose.position
        self.x = position.x
        self.y = position.y

        #desired_pose = self.plan[self.plan_idx]
        #x_diff = self.x - desired_pose.position.x
        #y_diff = self.y - desired_pose.position.y
        #cmd_vel_x = -x_diff
        #cmd_vel_y = -y_diff
        #if np.sqrt(x_diff ** 2 + y_diff ** 2) < 0.05:
        #    self.plan_idx += 1

        #if len(self.plan) < self.plan_idx:
        #    self.start = 0.0
        #    self.goal = 0.0
        #    self.get_global_plan()

    def get_global_plan(self):
        req = GetPlan()
        req.start = self.start
        req.goal = self.goal
        req.tolerance = 0.5
        self.plan = self.get_plan(req.start, req.goal, req.tolerance)
        # print(self.plan)

if __name__ == '__main__':
    try:
        rospy.init_node("follow_global_path", anonymous=True)
        Agent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
