#!/bin/bash
# Check the IL container: display, OpenGL, repositories and branches, build, data, Drive upload.
# Usage: il_check_install      (prints OK / WARN / FAIL per line)
ws=/root/arena_ws
source /root/.bashrc_il  # the workspace, also when run through docker exec (the base image sets ROS_DISTRO)
ok()   { printf '  OK    %s\n' "$*"; }
warn() { printf '  WARN  %s\n' "$*"; }
fail() { printf '  FAIL  %s\n' "$*"; bad=1; }
bad=0

echo "Display"
if xdpyinfo >/dev/null 2>&1; then ok "display $DISPLAY"; else fail "no display ($DISPLAY): container made without the display flags?"; fi
r=$(glxinfo -B 2>/dev/null | sed -n 's/.*OpenGL renderer string: //p')
if [ -z "$r" ]; then fail "OpenGL not available"
elif echo "$r" | grep -qi llvmpipe; then ok "OpenGL: software ($r)"
else ok "OpenGL: GPU ($r)"; fi

echo "Repositories"
check_repo() {  # path expected_remote expected_branch
    local d=$ws/src/$1
    [ -d "$d/.git" ] || { fail "$1 missing"; return; }
    local url br
    url=$(git -C "$d" remote get-url origin)
    br=$(git -C "$d" branch --show-current)
    [ "$url" = "$2" ] || { fail "$1: origin is $url, expected $2"; return; }
    [ "$br" = "$3" ] || { fail "$1: on '$br', expected $3"; return; }
    ok "$1  $br @ $(git -C "$d" rev-parse --short HEAD)  ($url)"
}
check_repo planners/dynamic_gap https://github.com/ivaROS/DynamicGap.git imitation_learning
check_repo arena/arena-rosnav https://github.com/ivaROS/arena-rosnav.git imitation_learning
check_repo arena/evaluation https://github.com/ivaROS/arena-evaluation.git imitation_learning
check_repo arena/simulation-setup https://github.com/ivaROS/arena-simulation-setup.git imitation_learning
check_repo arena/utils/pedsim_ros https://github.com/ivaROS/pedsim_ros.git global_path_custom_ped

echo "Build"
for p in dynamic_gap manual_gap_msgs rviz_plugins arena_bringup; do
    if rospack find "$p" >/dev/null 2>&1; then ok "$p"; else fail "$p not found (catkin build; source ~/.bashrc)"; fi
done
if [ -f "$ws/devel/lib/libdynamic_gap.so" ]; then ok "dynamic_gap compiled"; else fail "dynamic_gap not compiled"; fi

echo "Data"
for d in datasets models; do
    n=$(ls "/root/il_data/$d" 2>/dev/null | wc -l)
    if [ "$n" -gt 0 ]; then ok "/root/il_data/$d ($n entries, $(du -sh "/root/il_data/$d" | cut -f1))"; else fail "/root/il_data/$d empty"; fi
done
if v=$(python3 -c 'import torch; print(torch.__version__)' 2>/dev/null); then ok "torch $v"; else fail "torch missing"; fi

echo "Git and Drive"
if u=$(git config --global user.name); then ok "git user: $u"; else warn "git identity not set (Part 6)"; fi
if [ -z "${IL_DRIVE_REMOTE:-}" ]; then warn "Drive upload not set up (Part 7)"
elif rclone listremotes 2>/dev/null | grep -qx "${IL_DRIVE_REMOTE%%:*}:"; then ok "Drive upload to $IL_DRIVE_REMOTE"
else fail "IL_DRIVE_REMOTE=$IL_DRIVE_REMOTE: rclone has no such remote (rclone listremotes)"; fi

echo
if [ $bad = 0 ]; then echo "All checks passed."; else echo "Some checks FAILED: see the install guide's Troubleshooting."; fi
exit $bad
