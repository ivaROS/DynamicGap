#!/bin/bash
# IL data collection for the designed dataset (benchmark suite il_dataset_small_v1.yaml,
# gru_learn map): manual gap selection with rosbag recording.
# Usage: ~/run_il_dataset.sh <operator> [session_name]
#   operator: your name, saved with every session (e.g. ewan)
#   session:  default il_small_v1_<date>_<time>; a session name can be used only once
# Sessions land in /root/il_data/il_gap_selection/<contest>_<date_time>_<first stage>/.
# The benchmark relaunches itself once to load the map; RViz closing and reopening is expected.
# When the launch ends (Ctrl+C), new sessions are copied to the shared Drive folder if
# IL_DRIVE_REMOTE is set (Part 7 of the install guide); the local copy always stays.
if [ -z "$1" ]; then
    echo "usage: $0 <operator> [session_name]" >&2
    exit 1
fi
source /root/.bashrc_il
cd /root/arena_ws
export IL_OPERATOR="$1"
trap ':' INT  # Ctrl+C stops roslaunch but not this script (a handler, unlike '', is not inherited)
roslaunch arena_bringup start_trials.launch \
    man_gap:=true \
    tm_modules:=benchmark \
    benchmark_suite:=il_dataset_small_v1.yaml \
    record_il:=true \
    il_data_root:=/root/il_data \
    il_operator:="$1" \
    il_session:="${2:-il_small_v1_$(date +%Y-%m-%d_%H-%M-%S)}"
trap - INT
if [ -n "${IL_DRIVE_REMOTE:-}" ]; then
    echo; echo "Uploading new sessions to $IL_DRIVE_REMOTE/$IL_OPERATOR/il_gap_selection ..."
    il_upload_sessions || echo "Upload failed; retry later with: il_upload_sessions"
else
    echo; echo "IL_DRIVE_REMOTE is not set: sessions stay only in /root/il_data/il_gap_selection"
fi
