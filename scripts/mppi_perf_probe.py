#!/usr/bin/python3
"""Measure a running MPPI integration without changing ROS parameters or goals."""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import statistics
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        return None
    return dict(count=len(ordered), median=statistics.median(ordered),
                p95=ordered[math.ceil(.95 * len(ordered)) - 1], max=ordered[-1])


def process_cpu():
    result = {}
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            name = (directory / 'comm').read_text().strip()
            if name not in ('mbf_costmap_nav', 'rosout', 'rviz', 'flatland_server'):
                continue
            fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
            result[directory.name] = (name, (int(fields[11]) + int(fields[12])) /
                                     os.sysconf('SC_CLK_TCK'))
        except (OSError, ValueError):
            continue
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='full')
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    rospy.init_node('mppi_perf_probe', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    data = collections.defaultdict(lambda: collections.defaultdict(list))
    distance = [0.]
    previous_position = [None]
    previous_stamp = [None]
    reset_jumps = [0]

    def receive(message, topic):
        with lock:
            record = data[topic]
            record['arrival'].append(time.monotonic())
            markers = [m for m in message.markers if m.action == Marker.ADD] if isinstance(message, MarkerArray) else []
            header = markers[0].header if markers else getattr(message, 'header', None)
            if header is not None and header.stamp.to_sec() > 0:
                record['stamp_age_sim_ms'].append(1000 * (rospy.Time.now() - header.stamp).to_sec())
            if isinstance(message, MarkerArray):
                record['added_markers'].append(len(markers))
            if isinstance(message, Odometry):
                position = (message.pose.pose.position.x, message.pose.pose.position.y)
                if previous_position[0] is not None:
                    step = math.hypot(position[0] - previous_position[0][0],
                                      position[1] - previous_position[0][1])
                    dt = message.header.stamp.to_sec() - previous_stamp[0]
                    # Task resets can teleport the robot; exclude discontinuities.
                    if dt < 0 or step > max(1., 3. * dt):
                        reset_jumps[0] += 1
                    else:
                        distance[0] += step
                previous_position[0] = position
                previous_stamp[0] = message.header.stamp.to_sec()
                record['speed_m_s'].append(math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y))
            if isinstance(message, Twist):
                record['angular_velocity_rad_s'].append(message.angular.z)
                record['linear_velocity_m_s'].append(message.linear.x)

    topics = [('/rto/scan', LaserScan), ('/rto/odom', Odometry),
              ('/rto/ego_robot_marker', Marker), ('/rto/propagated_egocircle', MarkerArray),
              ('/rto/mppi_trajectories', MarkerArray), ('/rto/cmd_vel', Twist)]
    subscribers = [rospy.Subscriber(topic, kind, receive, callback_args=topic, queue_size=1)
                   for topic, kind in topics]
    # A newly started rospy node initially reports zero until its first /clock.
    if rospy.get_param('/use_sim_time', False):
        sim_start = rospy.wait_for_message('/clock', Clock, timeout=10).clock.to_sec()
    else:
        sim_start = rospy.Time.now().to_sec()
    with lock:
        data.clear()
        distance[0] = 0.
        previous_position[0] = None
        previous_stamp[0] = None
        reset_jumps[0] = 0
    initial_cpu = process_cpu()
    started = time.monotonic()
    while time.monotonic() - started < args.duration and not rospy.is_shutdown():
        time.sleep(min(.1, args.duration))
    elapsed = time.monotonic() - started
    final_cpu = process_cpu()
    for subscriber in subscribers:
        subscriber.unregister()
    with lock:
        report = dict(label=args.label, wall_seconds=elapsed,
                      sim_seconds=rospy.Time.now().to_sec() - sim_start,
                      mppi_parameters=rospy.get_param('/rto/move_base_flex/DynamicGapPlannerROS/mppi', {}),
                      robot_distance_m=distance[0], odometry_discontinuities=reset_jumps[0],
                      topics={}, cpu_percent_one_core={})
        for topic, _ in topics:
            record = data[topic]
            arrivals = record.pop('arrival', [])
            report['topics'][topic] = dict(messages=len(arrivals), wall_hz=len(arrivals)/elapsed,
                interval_wall_ms=distribution([1000*(b-a) for a,b in zip(arrivals, arrivals[1:])]),
                **{key: distribution(values) for key, values in record.items()})
        for pid, (name, seconds) in final_cpu.items():
            if pid in initial_cpu:
                report['cpu_percent_one_core'][name + ':' + pid] = 100*(seconds-initial_cpu[pid][1])/elapsed
    output = args.output or Path('/tmp') / ('mppi_perf_' + ''.join(c if c.isalnum() or c in '-_' else '_' for c in args.label) + '.json')
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    print('Saved:', output)
    if not report['topics']['/rto/mppi_trajectories']['messages']:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
