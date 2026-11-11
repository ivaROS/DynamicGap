#!/usr/bin/env python3
"""il_drive: uploads to the shared Drive folder. Runs real rclone against a local folder as the 'Drive'."""

import io
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import il_drive  # noqa: E402


def touch(path, data=b"x"):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)


class RemoteDirTest(unittest.TestCase):

    def test_mirrors_data_root_per_operator(self):
        self.assertEqual(il_drive.remote_dir("ildrive:", "ewan", "/d/evals/x/full", root="/d"),
                         "ildrive:ewan/evals/x/full")
        self.assertEqual(il_drive.remote_dir("gdrive:ilData/", "kim", "/d/il_gap_selection/s1", root="/d"),
                         "gdrive:ilData/kim/il_gap_selection/s1")

    def test_outside_root_uses_name(self):
        self.assertEqual(il_drive.remote_dir("ildrive:", "ewan", "/elsewhere/run7", root="/d"), "ildrive:ewan/run7")

    def test_operator_must_be_plain(self):
        for bad in ("", None, "a/b"):
            with self.assertRaises(ValueError):
                il_drive.remote_dir("ildrive:", bad, "/d/x", root="/d")


@unittest.skipIf(shutil.which("rclone") is None, "rclone not installed")
class UploadTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.root = os.path.join(self.tmp, "il_data")
        self.drive = os.path.join(self.tmp, "drive")
        os.makedirs(self.drive)
        os.environ["IL_DATA_ROOT"] = self.root
        self.log = io.StringIO()

    def tearDown(self):
        os.environ.pop("IL_DATA_ROOT", None)
        shutil.rmtree(self.tmp)

    def make_eval(self):
        run = os.path.join(self.root, "evals", "e1", "full")
        touch(os.path.join(run, "screen.mp4"), b"screen")
        touch(os.path.join(run, "videos", "ep01_a.mp4"), b"ep1")
        touch(os.path.join(run, "metrics.csv"), b"m")
        touch(os.path.join(run, "sessions", "s", "session_0.bag"), b"bag")
        return run, os.path.join(self.drive, "ewan", "evals", "e1", "full")

    def test_eval_uploaded_together_videos_deleted(self):
        run, dest = self.make_eval()
        self.assertEqual(il_drive.upload_eval(run, self.drive + "/", "ewan", log=self.log), 2)
        for rel, data in (("videos/ep01_a.mp4", b"ep1"), ("screen.mp4", b"screen"), ("metrics.csv", b"m"),
                          ("sessions/s/session_0.bag", b"bag")):
            with open(os.path.join(dest, rel), "rb") as f:
                self.assertEqual(f.read(), data, rel)
        self.assertEqual(il_drive.videos(run), [])
        self.assertTrue(os.path.exists(os.path.join(run, "metrics.csv")))
        self.assertTrue(os.path.exists(os.path.join(run, "sessions", "s", "session_0.bag")))
        self.assertFalse(os.path.exists(os.path.join(dest, il_drive.MARKER)))
        with open(os.path.join(run, il_drive.MARKER)) as f:
            self.assertIn("2 videos deleted locally", f.read())

    def test_eval_keep_videos(self):
        run, dest = self.make_eval()
        il_drive.upload_eval(run, self.drive + "/", "ewan", keep_videos=True, log=self.log)
        self.assertEqual(len(il_drive.videos(run)), 2)
        self.assertTrue(os.path.exists(os.path.join(dest, "screen.mp4")))

    def test_eval_upload_failure_deletes_nothing(self):
        run, _ = self.make_eval()
        self.assertEqual(il_drive.upload_eval(run, "nosuchremote:", "ewan", log=self.log), -1)
        self.assertEqual(len(il_drive.videos(run)), 2)

    def test_session_copied_and_kept(self):
        s = os.path.join(self.root, "il_gap_selection", "s1")
        touch(os.path.join(s, "session_0.bag"), b"bag")
        with open(os.path.join(s, "params.yaml"), "w") as f:
            f.write("operator: kim\n")
        self.assertEqual(il_drive.pending_sessions(os.path.dirname(s)), [s])
        self.assertEqual(il_drive.upload_session(s, self.drive + "/", log=self.log), 1)
        self.assertTrue(os.path.exists(os.path.join(self.drive, "kim", "il_gap_selection", "s1", "session_0.bag")))
        self.assertTrue(os.path.exists(os.path.join(s, "session_0.bag")))  # bags are never deleted
        self.assertEqual(il_drive.pending_sessions(os.path.dirname(s)), [])

    def test_operator_argument_wins(self):
        s = os.path.join(self.root, "il_gap_selection", "s2")
        touch(os.path.join(s, "session_0.bag"))
        with open(os.path.join(s, "params.yaml"), "w") as f:
            f.write("operator: kim\n")
        il_drive.upload_session(s, self.drive + "/", "ewan", log=self.log)
        self.assertTrue(os.path.isdir(os.path.join(self.drive, "ewan", "il_gap_selection", "s2")))

    def test_recording_session_skipped(self):
        s = os.path.join(self.root, "il_gap_selection", "s3")
        touch(os.path.join(s, "session_0.bag.active"))
        self.assertEqual(il_drive.upload_session(s, self.drive + "/", "ewan", log=self.log), 0)
        self.assertEqual(os.listdir(self.drive), [])

    def test_session_without_bag_skipped(self):
        s = os.path.join(self.root, "il_gap_selection", "s5")
        touch(os.path.join(s, "params.yaml"), b"operator: kim\n")
        touch(os.path.join(s, "episodes.csv"))
        self.assertEqual(il_drive.upload_session(s, self.drive + "/", log=self.log), 0)
        self.assertEqual(os.listdir(self.drive), [])

    def test_no_operator_skipped(self):
        s = os.path.join(self.root, "il_gap_selection", "s4")
        touch(os.path.join(s, "session_0.bag"))
        self.assertEqual(il_drive.upload_session(s, self.drive + "/", log=self.log), 0)
        self.assertEqual(os.listdir(self.drive), [])


if __name__ == "__main__":
    unittest.main()
