#!/bin/bash
source /home/azaro/arena_ws/devel/setup.bash
cd /home/azaro/arena_ws
multiplier="$1"
trial="$2"
cleanup() {
 rosnode kill -a > /tmp/dynamic_gap_headon_30trials/stop.log 2>&1
 python3 - <<'PY'
import subprocess,os,signal
for line in subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines():
 p=line.strip().split(None,1)
 if len(p)==2 and p[1].startswith('/usr/bin/python3 /opt/ros/noetic/bin/roslaunch arena_bringup start_arena.launch'):
  try: os.kill(int(p[0]),signal.SIGINT)
  except ProcessLookupError: pass
PY
 cp /tmp/dynamic_gap_headon_30trials/original_planner.yaml /home/azaro/arena_ws/src/arena/simulation-setup/configs/mbf/local/dgap_local_planner_params.yaml
 sleep 5
}
trap cleanup EXIT
python3 - "$multiplier" <<'PY'
import sys,re
from pathlib import Path
original=Path('/tmp/dynamic_gap_headon_30trials/original_planner.yaml').read_text()
updated,n=re.subn(r'^  Q: .*$', '  Q: '+sys.argv[1]+'.0', original,flags=re.M)
assert n==1
Path('/home/azaro/arena_ws/src/arena/simulation-setup/configs/mbf/local/dgap_local_planner_params.yaml').write_text(updated)
PY
roslaunch arena_bringup start_arena.launch tm_modules:=benchmark > "/tmp/dynamic_gap_headon_30trials/launch_${multiplier}_${trial}.log" 2>&1 &
sleep 15
/usr/bin/python3 /tmp/dynamic_gap_headon_30trials/record.py "$multiplier" "$trial" > "/tmp/dynamic_gap_headon_30trials/record_${multiplier}_${trial}.log" 2>&1
result=$?
if [ "$result" -eq 0 ]; then
 python3 - "$multiplier" "$trial" <<'PY'
import json,sys
p='/tmp/dynamic_gap_headon_30trials/'+sys.argv[1]+'x_trial_'+sys.argv[2]+'.json'
s=json.load(open(p))['summary']
print({k:s[k] for k in ('multiplier','trial','status','started','elapsed_wall_s','min_person_center_distance','min_scan')})
PY
else
 tail -15 "/tmp/dynamic_gap_headon_30trials/record_${multiplier}_${trial}.log"
fi
exit "$result"
