#!/bin/bash
# IL data collection for the designed dataset (benchmark suite il_dataset_small_v1.yaml,
# gru_learn map): manual gap selection with rosbag recording.
# Usage: ~/run_il_dataset.sh <operator> [session_name]
#   operator: your name, saved with every session and used as your folder on the Drive
#   session:  default il_small_v1_<date>_<time>; a session name can be used only once
# Sessions land in /root/il_data/il_gap_selection/<contest>_<date_time>_<first stage>/.
# The benchmark relaunches itself (RViz closes and reopens) to load the map; the relaunch runs
# detached from this terminal, so this script follows it: Ctrl+C here stops the run, and the script
# returns only when the run has ended. Then new sessions are copied to the shared Drive folder if
# IL_DRIVE_REMOTE is set (install guide Part 7); the local copy always stays.
case "${1:-}" in
    ""|.|..|*/*) echo "usage: $0 <operator> [session_name]  (operator: a plain name)" >&2; exit 1 ;;
esac
source /root/.bashrc_il
cd /root/arena_ws
export IL_OPERATOR="$1"

# the relaunch: delay_restart.py <master pid> ... waiting for the old master, then roslaunch ... benchmark_resume:=true
relaunch='delay_restart\.py [0-9]+ |roslaunch .*benchmark_resume:=true'
stop_run() {
    echo; echo "Stopping the run ..."
    pkill -INT -f "$relaunch"
}
trap stop_run INT  # a handler (unlike '') is not inherited: Ctrl+C still reaches roslaunch below

roslaunch arena_bringup start_trials.launch \
    man_gap:=true \
    tm_modules:=benchmark \
    benchmark_suite:=il_dataset_small_v1.yaml \
    record_il:=true \
    il_data_root:=/root/il_data \
    il_operator:="$1" \
    il_session:="${2:-il_small_v1_$(date +%Y-%m-%d_%H-%M-%S)}"

# Follow the relaunches until none has run for 20 s (a stage change relaunches again).
idle=0
while [ $idle -lt 20 ]; do
    if pgrep -f "$relaunch" >/dev/null; then idle=0; else idle=$((idle + 1)); fi
    sleep 1
done
trap - INT

# rosbag closes its file a moment after the launch ends
for _ in $(seq 30); do
    ls /root/il_data/il_gap_selection/*/*.bag.active >/dev/null 2>&1 || break
    sleep 1
done

if [ -n "${IL_DRIVE_REMOTE:-}" ]; then
    echo; echo "Uploading new sessions to $IL_DRIVE_REMOTE ($IL_OPERATOR/il_gap_selection) ..."
    il_upload_sessions || echo "Upload failed; retry later with: il_upload_sessions"
else
    echo; echo "IL_DRIVE_REMOTE is not set: sessions stay only in /root/il_data/il_gap_selection"
fi
