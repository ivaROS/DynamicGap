#!/usr/bin/python3
"""Run one controlled goal-approach trial in an already running map_empty Arena.

Changes runtime scenario/logger settings, but does not build or edit Arena files.
Set /goal_radius to 0.18 BEFORE launching Arena; its robot manager caches it.
Use tm_robots:=scenario tm_obstacles:=scenario and a separate persistent roscore.
"""
import argparse
import collections
import json
import math
from pathlib import Path
import threading
import time

import dynamic_reconfigure.client
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from mbf_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry
from pedsim_msgs.msg import AgentStates
from roscpp.srv import GetLoggers, SetLoggerLevel
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty


class Trial:
    def __init__(self, goal):
        self.goal = goal
        self.lock = threading.RLock()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.state = {}
        self.points, self.scans, self.commands = [], [], []
        self.results, self.phases, self.pedestrians = [], [], []
        self.latest_goal = None
        self.subscribers = [
            rospy.Subscriber('/rto/odom', Odometry, self.odom, queue_size=1),
            rospy.Subscriber('/rto/scan', LaserScan, self.scan, queue_size=1),
            rospy.Subscriber('/rto/cmd_vel', Twist, self.command, queue_size=1),
            rospy.Subscriber('/rto/move_base_flex/move_base/result', MoveBaseActionResult,
                             self.result, queue_size=10),
            rospy.Subscriber('/rto/goal_approach_status', String, self.phase, queue_size=10),
            rospy.Subscriber('/rto/move_base_simple/goal', PoseStamped, self.goal_message, queue_size=1),
            rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.agents, queue_size=1),
        ]
        self.stop_publisher = rospy.Publisher('/rto/cmd_vel', Twist, queue_size=1)

    def map_position(self, position, frame):
        transform = self.buffer.lookup_transform('map', frame, rospy.Time(0), rospy.Duration(.05))
        q = transform.transform.rotation
        yaw = 2 * math.atan2(q.z, q.w)
        return (transform.transform.translation.x + math.cos(yaw)*position.x - math.sin(yaw)*position.y,
                transform.transform.translation.y + math.sin(yaw)*position.x + math.cos(yaw)*position.y,
                yaw)

    def odom(self, message):
        try:
            x, y, frame_yaw = self.map_position(message.pose.pose.position, message.header.frame_id)
        except tf2_ros.TransformException:
            return
        q = message.pose.pose.orientation
        yaw = frame_yaw + 2 * math.atan2(q.z, q.w)
        distance = math.hypot(x-self.goal[0], y-self.goal[1])
        with self.lock:
            self.state = dict(pose=[x, y, yaw], distance=distance, speed=message.twist.twist.linear.x)
            self.points.append((time.monotonic(), rospy.Time.now().to_sec(), x, y, yaw,
                                distance, message.twist.twist.linear.x))

    def scan(self, message):
        nearest = min((r for r in message.ranges if math.isfinite(r) and r >= 0), default=99.)
        with self.lock:
            self.scans.append((time.monotonic(), nearest))

    def command(self, message):
        with self.lock:
            self.commands.append((time.monotonic(), message.linear.x, message.angular.z))

    def result(self, message):
        with self.lock:
            self.results.append((time.monotonic(), message.status.status,
                                 message.result.outcome, message.status.text))

    def phase(self, message):
        with self.lock:
            self.phases.append((time.monotonic(), message.data))

    def goal_message(self, message):
        with self.lock:
            self.latest_goal = (message.pose.position.x, message.pose.position.y)

    def agents(self, message):
        for agent in message.agent_states:
            try:
                x, y, _ = self.map_position(agent.pose.position, agent.header.frame_id)
            except tf2_ros.TransformException:
                continue
            with self.lock:
                if 'pose' in self.state:
                    distance = math.hypot(x-self.state['pose'][0], y-self.state['pose'][1])
                    self.pedestrians.append((time.monotonic(), agent.id, x, y, distance))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True, type=Path)
    parser.add_argument('--label', required=True)
    parser.add_argument('--timeout', type=float, default=180.)
    parser.add_argument('--output-dir', type=Path, default=Path('/tmp/dynamic_gap_goal_tests'))
    parser.add_argument('--max-regression', type=float, default=.1)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    scenario = json.loads(args.scenario.read_text())
    start, goal = scenario['robots'][0]['start'], scenario['robots'][0]['goal']
    rospy.init_node('goal_approach_arena_trial', anonymous=True, disable_signals=True)
    tolerance = rospy.get_param('/rto/move_base_flex/DynamicGapPlannerROS/xy_goal_tolerance', .25)
    if rospy.get_param('/goal_radius', .75) >= tolerance:
        parser.error('set /goal_radius to 0.18 before launching Arena to prevent premature task resets')
    trial = Trial(goal)
    rospy.wait_for_service('/rto/move_base_flex/get_loggers', timeout=30)
    get_loggers = rospy.ServiceProxy('/rto/move_base_flex/get_loggers', GetLoggers)
    set_level = rospy.ServiceProxy('/rto/move_base_flex/set_logger_level', SetLoggerLevel)
    for logger in get_loggers().loggers:
        if logger.name.startswith('ros.dynamic_gap'):
            set_level(logger.name, 'warn')
    set_level('ros.dynamic_gap.GoalApproach', 'info')
    client = dynamic_reconfigure.client.Client('/task_generator_server', timeout=15)
    client.update_configuration(dict(SCENARIO_file=str(args.scenario.resolve()), timeout=args.timeout+30))
    # The task node receives dynamic-reconfigure updates asynchronously.
    time.sleep(1)
    rospy.wait_for_service('/reset_task', timeout=15)
    rospy.ServiceProxy('/reset_task', Empty)()
    time.sleep(.5)
    with trial.lock:
        if trial.latest_goal is None or math.hypot(trial.latest_goal[0]-goal[0], trial.latest_goal[1]-goal[1]) > .01:
            raise RuntimeError('Arena did not apply the requested goal')
        if 'pose' not in trial.state or math.hypot(trial.state['pose'][0]-start[0], trial.state['pose'][1]-start[1]) > .8:
            raise RuntimeError('Arena did not apply the requested clear start position')
    activation = rospy.get_param('/rto/move_base_flex/DynamicGapPlannerROS/goal_approach/activation_distance', 4.)
    began = time.monotonic()
    near, next_log, completion, final_state = None, began, None, None
    status = 'timeout'
    try:
        while time.monotonic()-began < args.timeout and not rospy.is_shutdown():
            now = time.monotonic()
            with trial.lock:
                state = dict(trial.state)
                last_command = trial.commands[-1][0] if trial.commands else began
            if state.get('distance', 99) <= activation and near is None:
                near = now
            if now >= next_log:
                print(json.dumps(dict(label=args.label, elapsed=round(now-began, 1), **state)), flush=True)
                next_log = now+10
            if state.get('distance', 99) < tolerance:
                completion, final_state, status = now, state, 'reached'
                break
            if now-began > 3 and now-last_command > 2:
                status = 'planner_stalled'
                break
            time.sleep(.05)
    finally:
        ended = completion or time.monotonic()
        if status != 'reached':
            trial.stop_publisher.publish(Twist())
    time.sleep(1)  # Allow Move Base to publish its success result.
    for subscriber in trial.subscribers:
        subscriber.unregister()
    with trial.lock:
        trace = {key: [sample for sample in getattr(trial, key) if began <= sample[0] <= ended]
                 for key in ('points', 'scans', 'commands', 'pedestrians')}
        near_points = [sample for sample in trace['points'] if near is not None and sample[0] >= near]
        least, regression = activation, 0.
        for sample in near_points:
            least = min(least, sample[5])
            regression = max(regression, sample[5]-least)
        results = [sample[1:] for sample in trial.results if sample[0] >= began]
        min_scan = min((sample[1] for sample in trace['scans']), default=None)
        min_person = min((sample[4] for sample in trace['pedestrians']), default=None)
        summary = dict(label=args.label, status=status, start=start, goal=goal,
                       elapsed_wall_s=ended-began, near_goal_seconds=ended-near if near else None,
                       final_distance_m=(final_state or trial.state).get('distance'),
                       min_scan_m=min_scan, min_pedestrian_center_distance_m=min_person,
                       max_near_goal_regression_m=regression, results=results,
                       goal_approach_phases=dict(collections.Counter(sample[1] for sample in trial.phases
                                                                    if began <= sample[0] <= ended)),
                       scenario=scenario,
                       mppi=rospy.get_param('/rto/move_base_flex/DynamicGapPlannerROS/mppi', {}))
    failures = []
    if status != 'reached': failures.append(status)
    if not any(result[0] == 3 and result[1] == 0 for result in results): failures.append('no_move_base_success')
    if min_scan is None or min_scan < .225: failures.append('missing_scan_or_footprint_contact')
    if regression > args.max_regression: failures.append('near_goal_detour')
    if scenario.get('obstacles', {}).get('dynamic') and min_person is None: failures.append('missing_pedestrian_observations')
    if min_person is not None and min_person < .525: failures.append('pedestrian_footprint_overlap')
    summary.update(success=not failures, failures=failures)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    label = ''.join(c if c.isalnum() or c in '-_' else '_' for c in args.label)
    (args.output_dir / (label+'_summary.json')).write_text(json.dumps(summary, indent=2)+'\n')
    (args.output_dir / (label+'_trace.json')).write_text(json.dumps(trace)+'\n')
    print(json.dumps(summary), flush=True)
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
