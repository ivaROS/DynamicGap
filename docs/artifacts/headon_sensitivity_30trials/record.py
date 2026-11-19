import sys,time,json,math
sys.dont_write_bytecode = True
sys.path.insert(0,'/home/azaro/arena_ws/src/planners/dynamic_gap/test')
from run_goal_approach_arena import Trial
import rospy
from std_srvs.srv import Empty
rospy.init_node('headon_sensitivity_recorder',anonymous=True,disable_signals=True)
t=Trial((20,10))
rospy.wait_for_service('/reset_task',timeout=90)
time.sleep(3)
rospy.ServiceProxy('/reset_task',Empty)()
begin=time.monotonic(); status='timeout'; started=False
while time.monotonic()-begin<100:
    time.sleep(.05)
    with t.lock:
        s=dict(t.state)
    if not s: continue
    if not started:
        if math.hypot(s['pose'][0]-10,s['pose'][1]-10)<.8:
            started=True; begin=time.monotonic()
        continue
    if s['distance']<1.05:
        status='reached_task_tolerance'; break
    if time.monotonic()-begin>5 and math.hypot(s['pose'][0]-10,s['pose'][1]-10)<.2 and len(t.points)>2 and any(p[2]>12 for p in t.points):
        status='reset'; break
end=time.monotonic()
with t.lock:
    traces={k:[p for p in getattr(t,k) if begin<=p[0]<=end] for k in ('points','scans','commands','pedestrians','phases','results')}
    summary=dict(multiplier=float(sys.argv[1]),trial=int(sys.argv[2]),status=status,started=started,elapsed_wall_s=end-begin,final_state=dict(t.state),parameters=rospy.get_param('/rto/move_base_flex/DynamicGapPlannerROS'),min_person_center_distance=min((p[4] for p in traces['pedestrians']),default=None),min_scan=min((p[1] for p in traces['scans']),default=None),max_lateral_deviation=max((abs(p[3]-10) for p in traces['points']),default=None))
open('/tmp/dynamic_gap_headon_30trials/'+sys.argv[1]+'x_trial_'+sys.argv[2]+'.json','w').write(json.dumps(dict(summary=summary,traces=traces),indent=2))
assert summary['parameters']['Q'] == float(sys.argv[1]), 'Wrong obstacle weight'
print(json.dumps(summary),flush=True)
