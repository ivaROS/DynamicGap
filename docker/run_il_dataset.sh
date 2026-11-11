#!/bin/bash
# IL data collection for the designed dataset (benchmark suite il_dataset_small_v1.yaml,
# gru_learn map): manual gap selection with rosbag recording.
# Usage: ~/run_il_dataset.sh <operator> [session_name]
#   operator: your name, saved with every session and used as your folder on the Drive
#   session:  default il_small_v1_<date>_<time>; a session name can be used only once
# Sessions land in $IL_DATA_ROOT/il_gap_selection/<contest>_<date_time>_<first stage>/.
# The benchmark relaunches itself (RViz closes and reopens) to load the map; the relaunch runs
# detached from this terminal, so this script follows it: Ctrl+C here stops the run, and the script
# returns only when the run has ended. Then each session of this run is checked (il_check_session)
# and, if IL_DRIVE_REMOTE is set, you are asked whether to upload it to the shared Drive folder.
# A session you decline stays local only (drive_skipped.txt): `il_upload_sessions <dir>` still sends
# it. The local copy always stays.
# One script for the container (defaults below) and the lab PC (~/run_il_dataset.sh there sets
# IL_SETUP, IL_DATA_ROOT and IL_RVIZ_PREFIX and runs this file).
# IL_LAUNCH_ARGS: extra start_trials.launch arguments, e.g. "ped_vmax:=0.8" or "il_goal_cursor:=true"
# (an automatic operator that holds the cursor toward the goal: for testing, not for the dataset).
case "${1:-}" in
    ""|.|..|*/*) echo "usage: $0 <operator> [session_name]  (operator: a plain name)" >&2; exit 1 ;;
esac
repo=$(cd "$(dirname "$(realpath "$0")")/.." && pwd)  # src/planners/dynamic_gap
source "${IL_SETUP:-/root/.bashrc_il}"
export IL_DATA_ROOT="${IL_DATA_ROOT:-/root/il_data}"
export IL_OPERATOR="$1"
export PATH="$repo/il_tools:$PATH"
sessions="$IL_DATA_ROOT/il_gap_selection"
cd "$repo/../../.."  # the workspace

# the relaunch: delay_restart.py <master pid> ... waiting for the old master, then roslaunch ... benchmark_resume:=true
relaunch='delay_restart\.py [0-9]+ |roslaunch .*benchmark_resume:=true'
stop_run() {
    echo; echo "Stopping the run ..."
    pkill -INT -f "$relaunch"
}
trap stop_run INT  # a handler (unlike '') is not inherited: Ctrl+C still reaches roslaunch below

before=$(ls "$sessions" 2>/dev/null)  # session folders that are not this run's
extra=()
[ -n "${IL_RVIZ_PREFIX:-}" ] && extra+=(rviz_prefix:="$IL_RVIZ_PREFIX")
read -ra more <<< "${IL_LAUNCH_ARGS:-}"
extra+=("${more[@]}")
roslaunch arena_bringup start_trials.launch \
    man_gap:=true \
    tm_modules:=benchmark \
    benchmark_suite:=il_dataset_small_v1.yaml \
    record_il:=true \
    il_data_root:="$IL_DATA_ROOT" \
    il_operator:="$1" \
    il_session:="${2:-il_small_v1_$(date +%Y-%m-%d_%H-%M-%S)}" \
    "${extra[@]}"

# Follow the relaunches until none has run for 20 s (a stage change relaunches again).
idle=0
while [ $idle -lt 20 ]; do
    if pgrep -f "$relaunch" >/dev/null; then idle=0; else idle=$((idle + 1)); fi
    sleep 1
done
trap - INT

# rosbag closes its file a moment after the launch ends
for _ in $(seq 30); do
    ls "$sessions"/*/*.bag.active >/dev/null 2>&1 || break
    sleep 1
done

new=()
for name in $(comm -13 <(echo "$before") <(ls "$sessions" 2>/dev/null)); do
    d="$sessions/$name"
    ls "$d"/*.bag >/dev/null 2>&1 && new+=("$d")  # no bag: the folder written before the relaunch
done
if [ ${#new[@]} -eq 0 ]; then
    echo; echo "No session recorded."
    exit 0
fi

for d in "${new[@]}"; do
    echo
    il_check_session "$d"
    if [ -z "${IL_DRIVE_REMOTE:-}" ]; then
        echo "IL_DRIVE_REMOTE is not set: the session stays only in $sessions"
        continue
    fi
    answer=y
    if [ -t 0 ]; then
        read -r -p "Upload $(basename "$d") to $IL_DRIVE_REMOTE$IL_OPERATOR/? [Y/n] " answer
    fi
    case "${answer:-y}" in
        [nN]*)
            echo "$(date -Iseconds) not uploaded (declined after the run)" >> "$d/drive_skipped.txt"
            echo "Kept local only. To upload it later: il_upload_sessions $d" ;;
        *)
            il_upload_sessions "$d" || echo "Upload failed; retry later with: il_upload_sessions $d" ;;
    esac
done
