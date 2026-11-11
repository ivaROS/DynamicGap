"""Launching arena runs from il_tools drivers (il_goal_cursor_collect, il_eval_model) and
watching only the processes a driver started: every launch gets TAG=<driver pid> in its
environment, which the benchmark relaunch (delay_restart.py) inherits."""

import glob
import os
import subprocess
import time

LAUNCH_PATTERN = "roslaunch( -p [0-9]+)? arena_bringup"
PLANNER_PATTERN = "mbf_costmap_nav/mbf_costmap_nav"
TAG = "IL_GOAL_CURSOR_COLLECT"  # name kept from the first driver that used it


def master_port(env=os.environ):
    return env.get("ROS_MASTER_URI", "http://localhost:11311").rstrip("/").rsplit(":", 1)[-1]


def environ(pid):
    try:
        with open("/proc/{}/environ".format(pid), "rb") as f:
            entries = f.read().decode(errors="replace").split("\0")
    except OSError:  # gone, or not ours to read
        return None
    return dict(e.split("=", 1) for e in entries if "=" in e)


def process_port(pid, env):
    """Master port of a process: its -p/--port argument (roslaunch, rosmaster), else ROS_MASTER_URI."""
    try:
        with open("/proc/{}/cmdline".format(pid), "rb") as f:
            args = f.read().decode(errors="replace").split("\0")
    except OSError:
        args = []
    for flag in ("-p", "--port"):
        if flag in args[:-1]:
            return args[args.index(flag) + 1]
    return master_port(env)


def pids(pattern, keep=lambda pid, env: env.get(TAG) == str(os.getpid())):
    """Processes whose command line matches pattern and that keep(pid, environment)
    accepts (default: the ones this driver started)."""
    found = subprocess.run(["pgrep", "-f", pattern], stdout=subprocess.PIPE, text=True).stdout.split()
    return [pid for pid in found if (env := environ(pid)) is not None and keep(pid, env)]


def running(pattern):
    return bool(pids(pattern))


def port_in_use():
    """A master, launch or pending benchmark relaunch on this driver's port."""
    port = master_port()
    on_port = lambda pid, env: process_port(pid, env) == port
    return any(pids(p, on_port) for p in ("rosmaster", "roslaunch", "delay_restart.py"))


def stop_launch(timeout=60):
    """SIGINT the launch (and its relaunch), then SIGTERM what is left of it."""
    # a pending benchmark relaunch would start once the master is gone
    for pid in pids("delay_restart.py"):
        subprocess.run(["kill", "-TERM", pid])
    for pid in pids(LAUNCH_PATTERN):
        subprocess.run(["kill", "-INT", pid])
    end = time.time() + timeout
    while time.time() < end and running(LAUNCH_PATTERN):
        time.sleep(1)
    for pattern in (LAUNCH_PATTERN, "rosmaster", "flatland_server", PLANNER_PATTERN):
        for pid in pids(pattern):
            subprocess.run(["kill", "-TERM", pid])
    time.sleep(3)


def benchmark_runs(benchmark_dir, suite_file, since):
    """(run id, completed) of the benchmark runs of suite_file logged since since."""
    suite = os.path.splitext(suite_file)[0]
    runs = []
    for path in glob.glob(os.path.join(benchmark_dir, "logs", "*.log")):
        if os.path.getmtime(path) < since:
            continue
        with open(path) as f:
            lines = f.read().splitlines()
        if lines and lines[0].startswith("run ") and any(l.startswith("on suite {} with".format(suite)) for l in lines):
            runs.append((lines[0][len("run "):], any(l.endswith("BENCHMARK COMPLETED SUCCESSFULLY") for l in lines)))
    return runs


def launch_env(**extra):
    """Environment for a launch this driver starts (tagged as its own)."""
    return dict(os.environ, **{TAG: str(os.getpid())}, **extra)

