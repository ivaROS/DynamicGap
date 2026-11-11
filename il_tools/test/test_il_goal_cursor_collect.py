#!/usr/bin/env python3
"""il_goal_cursor_collect: it watches and stops only the launches it started, refuses
a port in use, and counts a stage as finished only when its own benchmark run logged
its completion."""

import importlib.machinery
import importlib.util
import os
import subprocess
import sys
import tempfile
import time
import unittest
import unittest.mock

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "il_goal_cursor_collect")


def load():
    loader = importlib.machinery.SourceFileLoader("il_goal_cursor_collect", SCRIPT)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class TestCollect(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.collect = load()

    def test_only_own_processes(self):
        c = self.collect
        marker = "il_goal_cursor_collect_test_{}".format(os.getpid())
        cmd = [sys.executable, "-c", "import time; time.sleep(30)", marker]
        own = subprocess.Popen(cmd, env=dict(os.environ, **{c.TAG: str(os.getpid())}))
        other = subprocess.Popen(cmd, env=dict(os.environ, **{c.TAG: "1"}))
        try:
            time.sleep(0.3)
            self.assertEqual([str(own.pid)], c.pids(marker))
        finally:
            own.kill()
            other.kill()

    def test_benchmark_runs(self):
        c = self.collect
        with tempfile.TemporaryDirectory() as bench:
            os.makedirs(os.path.join(bench, "logs"))
            since = time.time() - 1
            path = os.path.join(bench, "logs", "dgap_1.log")
            with open(path, "w") as f:
                f.write("run dgap_1\nof contest dgap with 1 contestants\non suite il_goal_cursor_collect_11312_a with 1 stages\n")
            with open(os.path.join(bench, "logs", "dgap_2.log"), "w") as f:  # the same stage on another port
                f.write("run dgap_2\non suite il_goal_cursor_collect_11311_a with 1 stages\n"
                        "1.0: BENCHMARK COMPLETED SUCCESSFULLY\n")
            self.assertEqual([("dgap_1", False)], c.benchmark_runs(bench, "il_goal_cursor_collect_11312_a.yaml", since))
            with open(path, "a") as f:
                f.write("1.0: BENCHMARK COMPLETED SUCCESSFULLY\n")
            self.assertEqual([("dgap_1", True)], c.benchmark_runs(bench, "il_goal_cursor_collect_11312_a.yaml", since))
            self.assertEqual([], c.benchmark_runs(bench, "il_goal_cursor_collect_11312_a_r2.yaml", since))
            self.assertEqual([], c.benchmark_runs(bench, "il_goal_cursor_collect_11312_a.yaml", time.time() + 10))

    def test_port_in_use(self):
        c = self.collect
        # a pending relaunch of another driver (its environment names the port)
        cmd = [sys.executable, "-c", "import time; time.sleep(30)", "delay_restart.py"]
        proc = subprocess.Popen(cmd, env=dict(os.environ, ROS_MASTER_URI="http://localhost:59871"))
        try:
            time.sleep(0.3)
            with unittest.mock.patch.dict(os.environ, {"ROS_MASTER_URI": "http://localhost:59871"}):
                self.assertTrue(c.port_in_use())
            with unittest.mock.patch.dict(os.environ, {"ROS_MASTER_URI": "http://localhost:59872"}):
                self.assertFalse(c.port_in_use())
        finally:
            proc.kill()

    def test_launch_with_a_port_is_found(self):
        c = self.collect
        # the driver's own first launch ("roslaunch -p PORT arena_bringup ...") and the
        # relaunch ("roslaunch arena_bringup ..."); a launch's -p wins over its environment
        script = "import time; time.sleep(30)"
        procs = [subprocess.Popen([sys.executable, "-c", script, "roslaunch", "-p", "59873", "arena_bringup"],
                                  env=dict(os.environ, ROS_MASTER_URI="http://localhost:59874",
                                           **{c.TAG: str(os.getpid())})),
                 subprocess.Popen([sys.executable, "-c", script, "roslaunch", "arena_bringup", "x"],
                                  env=dict(os.environ, **{c.TAG: str(os.getpid())}))]
        try:
            time.sleep(0.3)
            self.assertEqual(sorted(str(p.pid) for p in procs), sorted(c.pids(c.LAUNCH_PATTERN)))
            with unittest.mock.patch.dict(os.environ, {"ROS_MASTER_URI": "http://localhost:59873"}):
                self.assertTrue(c.port_in_use())
            with unittest.mock.patch.dict(os.environ, {"ROS_MASTER_URI": "http://localhost:59874"}):
                self.assertFalse(c.port_in_use())
        finally:
            for p in procs:
                p.kill()

if __name__ == "__main__":
    unittest.main()
