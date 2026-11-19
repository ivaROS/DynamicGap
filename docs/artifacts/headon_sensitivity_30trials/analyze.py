"""Aggregate recorded trials and regenerate plots; run with /usr/bin/python3."""
import csv,gzip,json,math,statistics
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path(__file__).resolve().parent
source=Path('/tmp/dynamic_gap_headon_30trials')
rows=[];groups={}; reference=None
fig,axes=plt.subplots(1,2,figsize=(13,4.5),gridspec_kw={'width_ratios':[1.6,1]})
for color,m in zip(('tab:blue','tab:orange','tab:green'),(0,10,30)):
 groups[m]=[]
 for i in range(1,11):
  filename=f'{m}x_trial_{i}.json'; archive=out/(filename+'.gz')
  if archive.exists():
   with gzip.open(archive,'rt') as f: d=json.load(f)
  else:
   d=json.loads((source/filename).read_text())
   with gzip.open(archive,'wt') as f: json.dump(d,f)
  s=d['summary'];t=d['traces'];p=t['points'];a=t['pedestrians']
  assert s['multiplier']==m and s['trial']==i and s['parameters']['Q']==m
  params=dict(s['parameters']);params.pop('Q')
  if reference is None: reference=params
  assert params==reference, 'Other runtime parameters differ'
  assert p and a and t['scans'] and s['started']
  person_motion=max(math.hypot(v[2]-15,v[3]-11) for v in a)
  assert person_motion<.001, 'Person moved'
  row={k:s[k] for k in ('multiplier','trial','status','elapsed_wall_s','min_person_center_distance','min_scan','max_lateral_deviation')}
  row.update(elapsed_sim_s=p[-1][1]-p[0][1],start_x=p[0][2],start_y=p[0][3],start_yaw=p[0][4],person_max_displacement_m=person_motion,path_length_m=sum(math.hypot(v[2]-u[2],v[3]-u[3]) for u,v in zip(p,p[1:])))
  rows.append(row);groups[m].append(row)
  axes[0].plot([v[2] for v in p],[v[3] for v in p],color=color,alpha=.45,label=f'{m}× (10 trials)' if i==1 else None)
 values=[r['min_person_center_distance'] for r in groups[m]]
 axes[1].scatter([m+(i-4.5)*.12 for i in range(10)],values,color=color,alpha=.7)
 axes[1].errorbar(m,statistics.mean(values),yerr=statistics.stdev(values),fmt='ks',capsize=6)
axes[0].scatter([15],[11],color='black',marker='o',label='Person center')
axes[0].scatter([10,20],[10,10],color='black',marker='x')
axes[0].set(xlabel='Map x (m)',ylabel='Map y (m)',title='Measured robot paths');axes[0].axis('equal');axes[0].legend()
axes[1].set(xlabel='Obstacle weight multiplier',ylabel='Closest person-center distance (m)',title='Individual trials; black = mean ± sample SD',xticks=[0,10,30])
for ax in axes: ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(out/'comparison.png',dpi=180)
with (out/'trials.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
summary=[]
for m,g in groups.items():
 result={'multiplier':m,'trials':len(g),'reached_recording_threshold':sum(r['status']=='reached_task_tolerance' for r in g)}
 for key in ('min_person_center_distance','min_scan','elapsed_sim_s','elapsed_wall_s','path_length_m'):
  v=[r[key] for r in g];result[key]={'mean':statistics.mean(v),'sd':statistics.stdev(v),'min':min(v),'max':max(v)}
 summary.append(result)
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
print('start ranges', {k:(min(r[k] for r in rows),max(r[k] for r in rows)) for k in ('start_x','start_y','start_yaw')})
