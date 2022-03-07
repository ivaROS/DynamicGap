#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3Stamped, PoseArray

import tf_conversions

import tf2_ros
import tf2_geometry_msgs

class Agent:
    def __init__(self, num_obsts, world):
        rospy.wait_for_service('/move_base_virtual/make_plan')
        self.get_plan = rospy.ServiceProxy('/move_base_virtual/make_plan', GetPlan)
        self.world = world
        self.plan_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.plan = None
        self.agent_odoms = None
        self.odom_subs = {}
        self.cmd_vel_pubs = {}
        self.need_plan = {}
        self.plans = {}
        self.plan_indices = {}
        self.plan_publishers = {}
        for i in range(0, num_obsts):
            robot_namespace = "robot" + str(i)
            self.odom_subs[robot_namespace] = rospy.Subscriber(robot_namespace + "/odom", Odometry, self.odom_CB, queue_size=5)
            self.cmd_vel_pubs[robot_namespace] = rospy.Publisher(robot_namespace + "/cmd_vel", Twist, queue_size=5)
            self.need_plan[robot_namespace] = True
            self.plan_indices[robot_namespace] = 0
            self.plan_publishers[robot_namespace] = rospy.Publisher(robot_namespace + "/global_path", PoseArray, queue_size=5)
        '''
        self.start = PoseStamped()
        self.start.header.frame_id = "known_map"
        self.goal = PoseStamped()
        self.goal.header.frame_id = "known_map"
        self.goal.pose.position.y = 10
        '''
        # self.get_global_plan()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.campus_goal_regions = [5, 16, 6, 13] # top left to bottom right

    def odom_CB(self, msg):
        robot_namespace = msg.child_frame_id
        print('plan index:', self.plan_indices[robot_namespace])
        if (robot_namespace not in self.plans) or len(self.plans[robot_namespace].plan.poses) <= self.plan_indices[robot_namespace]:
            #self.plan_indices[robot_namespace] = 0
            #self.plans[robot_namespace].plan.poses = np.flip(self.plans[robot_namespace].plan.poses)
            self.need_plan[robot_namespace] = True
        #print('odom msg header', msg.header)
        if self.plan_indices[robot_namespace] > 0 and len(self.plans[robot_namespace].plan.poses) <= self.plan_indices[robot_namespace]:
            self.plan_indices[robot_namespace] = 0
            self.plans[robot_namespace].plan.poses = np.flip(self.plans[robot_namespace].plan.poses)
        # ODOM: map_static
        # DESIRED: known_map
        # CMD_VEL: robot0 frame
        #print('desired pose: ', desired_pose)
        # first is target frame, second is source frame
        # print('odom in map static: ', msg.pose.pose.position.x, ", ", msg.pose.pose.position.y)
        map_static_to_known_map_trans = self.tfBuffer.lookup_transform("known_map", "map_static", rospy.Time(), rospy.Duration(3.0))

        # print(map_static_to_known_map_trans)
        # transforming from map_static to known_map
        odom_in_known_map = tf2_geometry_msgs.do_transform_pose(msg.pose, map_static_to_known_map_trans)
        # print('odom in known map: ', odom_in_known_map.pose.position.x, ", ", odom_in_known_map.pose.position.y)

        if self.need_plan[robot_namespace]:
            start = PoseStamped()
            start.header.frame_id = "known_map"
            start.header.stamp = rospy.Time.now()
            start.pose.position.x = odom_in_known_map.pose.position.x
            start.pose.position.y = odom_in_known_map.pose.position.y
            start.pose.position.z = 0.0
            self.get_global_plan(start, robot_namespace)
            self.need_plan[robot_namespace] = False
            return
        print('plan index:', self.plan_indices[robot_namespace], ' out of: ', len(self.plans[robot_namespace].plan.poses))

        #print('robot namespace: ', robot_namespace)
        #print('plan index: ', self.plan_indices[robot_namespace])
        #print('plan poses: ', self.plans[robot_namespace].plan.poses)
        desired_pose = self.plans[robot_namespace].plan.poses[self.plan_indices[robot_namespace]]
        # print('desired pose: ', desired_pose.pose.position.x, ", ", desired_pose.pose.position.y)
        x_diff = odom_in_known_map.pose.position.x - desired_pose.pose.position.x
        y_diff = odom_in_known_map.pose.position.y - desired_pose.pose.position.y
        # transforming from known_map to robot0
        # print('x_diff: ', x_diff, ', and y_diff: ', y_diff)
        known_map_to_robot_trans = self.tfBuffer.lookup_transform(robot_namespace, "known_map", rospy.Time())
        # print('known_map_to_robot_trans: ', known_map_to_robot_trans)
        diff_vect = Vector3Stamped()
        diff_vect.header.frame_id = "known_map"
        diff_vect.header.stamp = rospy.Time.now()
        diff_vect.vector.x = -x_diff
        diff_vect.vector.y = -y_diff
        # print('difference vector in known map: ', diff_vect.vector.x, ", ", diff_vect.vector.y)
        diff_in_robot_0 = tf2_geometry_msgs.do_transform_vector3(diff_vect, known_map_to_robot_trans)
        # print('difference vector in robot0: ', diff_in_robot_0.vector.x, ", ", diff_in_robot_0.vector.y)
        twist = Twist()
        [x_vel, y_vel] = self.clip_cmd_vel(diff_in_robot_0)
        # print('x_vel: ', x_vel, ', y_vel: ', y_vel)
        twist.linear.x = 5*x_vel
        twist.linear.y = 5*y_vel
        # print('twist: ', twist)
        self.cmd_vel_pubs[robot_namespace].publish(twist)

        if np.sqrt(np.square(x_diff) + np.square(y_diff)) < 0.05:
            self.plan_indices[robot_namespace] += 1

    def get_global_plan(self, start, robot_namespace):
        print('generating plan for ' + robot_namespace)
        goal = PoseStamped()
        goal.header.frame_id = "known_map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = -10 #np.random.randint(self.campus_goal_regions[0], self.campus_goal_regions[2])
        goal.pose.position.y = -10 #np.random.randint(self.campus_goal_regions[3], self.campus_goal_regions[1])
        goal.pose.position.z = 0.0
        req = GetPlan()
        req.start = start
        req.goal = goal
        req.tolerance = 0.5
        print('start of : ' + str(start.pose.position.x) + ', ' + str(start.pose.position.y)
              + ', goal of : ' + str(goal.pose.position.x) + ', ' + str(goal.pose.position.y))
        self.plans[robot_namespace] = self.get_plan(req.start, req.goal, req.tolerance)
        print('plan has length of: ', len(self.plans[robot_namespace].plan.poses))
        pub_pose_array = PoseArray()
        pub_pose_array.header.frame_id = "known_map"
        pub_pose_array.header.stamp = rospy.Time.now()
        for i in range(0, len(self.plans[robot_namespace].plan.poses)):
            new_pose = Pose()
            new_pose.position.x = self.plans[robot_namespace].plan.poses[i].pose.position.x
            new_pose.position.y = self.plans[robot_namespace].plan.poses[i].pose.position.y
            new_pose.position.z = 0.5
            pub_pose_array.poses.append(new_pose)
        # print('publishing this pose array: ', pub_pose_array)
        self.plan_publishers[robot_namespace].publish(pub_pose_array)

        # print('get plan return plan: ', plan)
        # print("trying self.plan.respone: ", self.plan.response)
        # print("trying self.plan.plan: ", self.plan.plan)

    def clip_cmd_vel(self, diff_in_robot_0):
        delta_x_norm = np.sqrt(np.square(diff_in_robot_0.vector.x) + np.square(diff_in_robot_0.vector.y))
        # print('delta x norm: ', delta_x_norm)
        thresh = 0.5
        if delta_x_norm > thresh:
            return ([diff_in_robot_0.vector.x, diff_in_robot_0.vector.y] / delta_x_norm) * thresh
        else:
            return [diff_in_robot_0.vector.x, diff_in_robot_0.vector.y]

if __name__ == '__main__':
    try:
        rospy.init_node("follow_global_path", anonymous=True)
        num_obsts = rospy.get_param("~num_obsts")
        world = rospy.get_param("~world")
        #print("robot namespace: ", robot_namespace)
        Agent(num_obsts, world)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
