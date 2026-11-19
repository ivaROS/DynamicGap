import json,math,gzip,collections
from pathlib import Path
out=Path(__file__).resolve().parent
with gzip.open(out/'run.json.gz','rt') as f: data=json.load(f)
# Exclude the in-flight pre-reset log/frame using the first observed reset pose.
start=next(p[0] for p in data['points'] if math.hypot(p[2]-10,p[3]-10)<.8)
logs=[l for l in data['logs'] if l['wall']>=start]
rows=[]
for l in logs:
 r=dict(t.split('=',1) for t in l['message'].split());r['stamp']=l['stamp'];rows.append(r)
frames=[f for f in data['frames'] if f['wall']>=start]
labels=[m for f in frames for m in f['markers'] if m['action']==0 and m['ns'].endswith('/cost')]
seeds=[m for f in frames for m in f['markers'] if m['action']==0 and m['ns'].endswith('/seed')]
nominals=[m for f in frames for m in f['markers'] if m['action']==0 and m['ns'].endswith('/nominal')]
assert len(rows)==len(labels)==len(seeds)==len(nominals)>0
assert all('warm_start=NO' in m['text'] for m in labels)
assert all(len(m['points'])==11 and m['frame'] for m in seeds+nominals)
assert all(int(r['initialized'])==1 and int(r['warm_start'])==0 for r in rows)
assert len({(r['left'],r['right']) for r in rows})==1
assert [int(r['pair_streak']) for r in rows]==list(range(1,len(rows)+1))
assert any(m['action']==2 and m['ns'].endswith('/seed') for f in frames for m in f['markers'])
summary=dict(status=data['status'],optimizer_calls=len(rows),fresh_initializations=len(rows),warm_starts=0,
             persistent_pair_resets=sum(int(r['pair_streak'])>1 for r in rows),
             endpoint_pairs=sorted({(r['left'],r['right']) for r in rows}),
             maximum_pair_streak=max(int(r['pair_streak']) for r in rows),
             first_stamp=rows[0]['stamp'],last_stamp=rows[-1]['stamp'],
             observed_span_s=rows[-1]['stamp']-rows[0]['stamp'],
             seed_sources=dict(collections.Counter(r['source'] for r in rows)),
             valid_outputs=sum(int(r['output_valid']) for r in rows),
             mean_abs_delta_w_range=[min(float(r['mean_abs_delta_w']) for r in rows),max(float(r['mean_abs_delta_w']) for r in rows)],
             verified_seed_frames=len(seeds),verified_label_frames=len(labels),seed_delete_verified=True,
             excluded_pre_reset_logs=len(data['logs'])-len(logs))
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(out/'seed.log').write_text(''.join(f"{l['stamp']:.3f} {l['message']}\n" for l in logs))
(out/'example_label.txt').write_text(labels[-1]['text']+'\n')
print(json.dumps(summary,indent=2))
