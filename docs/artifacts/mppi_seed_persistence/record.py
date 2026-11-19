import sys,time,json,math,threading
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,'/home/azaro/arena_ws/src/planners/dynamic_gap/test')
from run_goal_approach_arena import Trial
import rospy
from std_srvs.srv import Empty
from rosgraph_msgs.msg import Log
from visualization_msgs.msg import MarkerArray
from roscpp.srv import GetLoggers,SetLoggerLevel
rospy.init_node('mppi_seed_validation',anonymous=True,disable_signals=True)
trial=Trial((20,10)); logs=[];frames=[];lock=threading.Lock()
def log_cb(msg):
 if 'initialized=1 warm_start=0 source=' in msg.msg:
  with lock: logs.append(dict(wall=time.monotonic(),stamp=msg.header.stamp.to_sec(),message=msg.msg))
def marker_cb(msg):
 markers=[]
 for m in msg.markers:
  if m.ns.endswith(('/seed','/nominal','/cost')):
   markers.append(dict(ns=m.ns,id=m.id,action=m.action,type=m.type,frame=m.header.frame_id,text=m.text,points=[[p.x,p.y,p.z] for p in m.points]))
 with lock: frames.append(dict(wall=time.monotonic(),markers=markers))
sub=rospy.Subscriber('/rosout',Log,log_cb,queue_size=1000)
markersub=rospy.Subscriber('/rto/mppi_trajectories',MarkerArray,marker_cb,queue_size=100)
rospy.wait_for_service('/reset_task',timeout=90)
rospy.wait_for_service('/rto/move_base_flex/get_loggers',timeout=30)
get=rospy.ServiceProxy('/rto/move_base_flex/get_loggers',GetLoggers)
setlevel=rospy.ServiceProxy('/rto/move_base_flex/set_logger_level',SetLoggerLevel)
for logger in get().loggers:
 if logger.name.startswith('ros.dynamic_gap'):
  setlevel(logger.name,'info' if logger.name.endswith('.MPPISeed') else 'warn')
setlevel('ros.dynamic_gap.MPPISeed','info')
time.sleep(1)
begin=time.monotonic()
rospy.ServiceProxy('/reset_task',Empty)()
status='timeout';started=False
while time.monotonic()-begin<90:
 time.sleep(.05)
 with trial.lock: state=dict(trial.state)
 if not state: continue
 if not started:
  if math.hypot(state['pose'][0]-10,state['pose'][1]-10)<.8: started=True
  continue
 if state['distance']<1.05:
  status='reached_recording_threshold';break
end=time.monotonic()
time.sleep(.3)
with lock:
 output=dict(status=status,logs=[x for x in logs if begin<=x['wall']<=end],frames=[x for x in frames if begin<=x['wall']<=end])
with trial.lock:
 output['phases']=[x for x in trial.phases if begin<=x[0]<=end]
 output['points']=[x for x in trial.points if begin<=x[0]<=end]
output['parameters']=rospy.get_param('/rto/move_base_flex/DynamicGapPlannerROS')
Path('/tmp/dynamic_gap_seed_diagnostics/run.json').write_text(json.dumps(output,indent=2))
print(status,'seed_logs',len(output['logs']),'marker_frames',len(output['frames']))
assert output['logs'] and any(m['ns'].endswith('/seed') and m['action']==0 for f in output['frames'] for m in f['markers'])
