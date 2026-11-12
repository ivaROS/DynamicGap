"""Upload IL data to the shared Google Drive folder with rclone.

Layout on the Drive (REMOTE = $IL_DRIVE_REMOTE: `ildrive:`, an rclone remote rooted at the shared
folder ilData, Drive folder id 1cBxaI5KS-Ux-l3pW-MM9w-hOoD1V7upC; the shells set it when that remote exists), mirroring
~/il_data per person so a run's bags, metrics and videos stay together:
  REMOTE/<operator>/il_gap_selection/<session>/...   collected sessions: copied, the local copy stays
  REMOTE/<operator>/evals/<eval>/<model>/...         eval runs: all copied, then the local videos deleted
<operator> labels whose run it is ($IL_OPERATOR, else the session's operator in params.yaml).
Local paths are taken relative to ~/il_data ($IL_DATA_ROOT); a folder elsewhere keeps only its name.
rclone checks each upload's checksum; a video is deleted locally only once its Drive copy matches.
"""

import datetime
import os
import shutil
import subprocess
import sys

import yaml

MARKER = "on_drive.txt"
SKIPPED = "drive_skipped.txt"  # declined after the run (run_il_dataset.sh): not uploaded unless named


def data_root():
    return os.path.realpath(os.environ.get("IL_DATA_ROOT", os.path.expanduser("~/il_data")))


def check_operator(operator):
    """The operator is one folder name on the Drive: raises ValueError if it is not a plain name."""
    if not isinstance(operator, str) or not operator or "/" in operator or operator in (".", ".."):
        raise ValueError("operator must be a plain name, got {!r}".format(operator))


def remote_dir(remote, operator, local_dir, root=None):
    """REMOTE/<operator>/<local_dir relative to the data root> (its name if outside the root)."""
    check_operator(operator)
    local_dir = os.path.realpath(local_dir)
    root = os.path.realpath(root or data_root())
    if local_dir != root and os.path.commonpath([local_dir, root]) == root:
        rel = os.path.relpath(local_dir, root)
    else:
        rel = os.path.basename(local_dir)
    base = remote if remote.endswith(":") else remote.rstrip("/") + "/"
    return "{}{}/{}".format(base, operator, rel)


def session_operator(session_dir):
    """The operator recorded in the session's params.yaml, None if missing or not a plain name."""
    try:
        with open(os.path.join(session_dir, "params.yaml")) as f:
            params = yaml.safe_load(f)
        operator = params.get("operator") if isinstance(params, dict) else None
        check_operator(operator)
        return operator
    except (OSError, yaml.YAMLError, ValueError):
        return None


def files_below(local_dir, suffix):
    """Sorted paths of the files under local_dir (any depth) whose name ends with suffix."""
    return sorted(os.path.join(dirpath, f) for dirpath, _, files in os.walk(local_dir)
                  for f in files if f.endswith(suffix))


def videos(local_dir):
    return files_below(local_dir, ".mp4")


def has_episodes(session_dir):
    """True if the session's episodes.csv has at least one row: an episode began. The folder a
    benchmark writes before it relaunches itself can hold a short bag but no episode."""
    try:
        with open(os.path.join(session_dir, "episodes.csv")) as f:
            return sum(1 for line in f if line.strip()) > 1  # the header + a row
    except OSError:
        return False


def recording(local_dir):
    """True while rosbag is still writing (a .bag.active file anywhere below)."""
    return bool(files_below(local_dir, ".bag.active"))


def _note(local_dir, text):
    with open(os.path.join(local_dir, MARKER), "a") as f:
        f.write("{} {}\n".format(datetime.datetime.now().isoformat(timespec="seconds"), text))


def _rclone(args, dry_run):
    if shutil.which("rclone") is None:
        raise RuntimeError("rclone is not installed")
    cmd = ["rclone"] + args + ["--transfers", "4", "--stats-one-line", "--stats", "30s"]
    if dry_run:
        cmd.append("--dry-run")
    return subprocess.run(cmd).returncode


def upload_eval(local_dir, remote, operator, keep_videos=False, dry_run=False, log=sys.stdout):
    """Copy an eval run folder to the Drive, then delete its local videos (unless keep_videos).
    Returns the number of videos found, 0 if skipped, -1 on an rclone error."""
    if recording(local_dir):
        print("{}: still recording, skipped".format(local_dir), file=log)
        return 0
    found = videos(local_dir)
    dest = remote_dir(remote, operator, local_dir)
    size = sum(os.path.getsize(f) for f in found) / 1e9
    print("{}: copying ({} videos, {:.1f} GB) -> {}".format(local_dir, len(found), size, dest), file=log, flush=True)
    if _rclone(["copy", local_dir, dest, "--exclude", MARKER], dry_run) != 0:
        print("rclone failed; nothing local was deleted", file=log)
        return -1
    if keep_videos or not found:
        if not dry_run:
            _note(local_dir, "copied to {}".format(dest))
        return len(found)
    # the videos are on the Drive now: move (--checksum: compares MD5s, not size and time) deletes
    # only the local copies whose Drive copy matches
    rc = _rclone(["move", local_dir, dest, "--include", "*.mp4", "--checksum"], dry_run)
    if not dry_run:
        left = len(videos(local_dir))
        _note(local_dir, "copied to {}; {} videos deleted locally{}".format(
            dest, len(found) - left, " ({} kept: upload not verified)".format(left) if left else ""))
    if rc != 0:
        print("rclone failed while removing local videos; the ones not verified stay local", file=log)
        return -1
    return len(found)


def upload_session(session_dir, remote, operator=None, dry_run=False, log=sys.stdout):
    """Copy one session folder to the Drive (nothing local is deleted). Returns 1, 0 (skipped) or -1."""
    if recording(session_dir):
        print("{}: still recording, skipped".format(session_dir), file=log)
        return 0
    if not files_below(session_dir, ".bag"):
        # e.g. the short folder a benchmark writes before it relaunches itself: no driving in it
        print("{}: no bag (nothing recorded), skipped".format(session_dir), file=log)
        return 0
    if not has_episodes(session_dir):
        print("{}: no episode in episodes.csv (nothing driven), skipped".format(session_dir), file=log)
        return 0
    operator = operator or session_operator(session_dir)
    if not operator:
        print("{}: no operator (set IL_OPERATOR), skipped".format(session_dir), file=log)
        return 0
    dest = remote_dir(remote, operator, session_dir)
    print("{} -> {}".format(session_dir, dest), file=log, flush=True)
    if _rclone(["copy", session_dir, dest, "--exclude", MARKER], dry_run) != 0:
        print("rclone failed", file=log)
        return -1
    if not dry_run:
        _note(session_dir, "copied to {}".format(dest))
    return 1


def pending_sessions(sessions_root):
    """Session folders under sessions_root not uploaded yet and not declined (neither marker file)."""
    if not os.path.isdir(sessions_root):
        return []
    out = []
    for name in sorted(os.listdir(sessions_root)):
        d = os.path.join(sessions_root, name)
        if os.path.isdir(d) and not any(os.path.exists(os.path.join(d, m)) for m in (MARKER, SKIPPED)):
            out.append(d)
    return out
