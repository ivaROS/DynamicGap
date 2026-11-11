#!/bin/bash
# Check the IL container: display, OpenGL, repositories and branches, build, data, Drive upload.
# Usage: il_check_install      (prints OK / WARN / FAIL per line)
ws=/root/arena_ws
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
check_repo arena/arena-rosnav https://github.com/Udog-ILLINOIS/arena-rosnav.git imitation_learning
check_repo arena/evaluation https://github.com/Udog-ILLINOIS/arena-evaluation.git imitation_learning
check_repo arena/simulation-setup https://github.com/Udog-ILLINOIS/arena-simulation-setup.git imitation_learning
check_repo arena/utils/pedsim_ros https://github.com/ivaROS/pedsim_ros.git global_path_custom_ped

echo "Build"
for p in dynamic_gap manual_gap_msgs rviz_plugins arena_bringup; do
    if rospack find $p >/dev/null 2>&1; then ok "$p"; else fail "$p not found (catkin build; source ~/.bashrc)"; fi
done
if [ -f $ws/devel/lib/libdynamic_gap.so ]; then ok "dynamic_gap compiled"; else fail "dynamic_gap not compiled"; fi

echo "Data"
for d in datasets models; do
    n=$(ls /root/il_data/$d 2>/dev/null | wc -l)
    [ "$n" -gt 0 ] && ok "/root/il_data/$d ($n entries, $(du -sh /root/il_data/$d | cut -f1))" || fail "/root/il_data/$d empty"
done
python3 -c "import torch" 2>/dev/null && ok "torch $(python3 -c 'import torch;print(torch.__version__)')" || fail "torch missing"

echo "Git and Drive"
git config --global user.name >/dev/null && ok "git user: $(git config --global user.name)" || warn "git identity not set (Part 6)"
if [ -n "${IL_DRIVE_REMOTE:-}" ]; then ok "Drive upload to $IL_DRIVE_REMOTE"; else warn "Drive upload not set up (Part 7)"; fi

echo
[ $bad = 0 ] && echo "All checks passed." || echo "Some checks FAILED: see the install guide's Troubleshooting."
exit $bad
