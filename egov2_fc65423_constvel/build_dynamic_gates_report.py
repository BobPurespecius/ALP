#!/usr/bin/env python3
import json,re,statistics
from pathlib import Path
ROOT=Path('/home/bob/ALP/egov2_fc65423_constvel'); D=ROOT/'dynamic_gates_repeatability_20260902'; M=json.load(open(D/'repeatability_metrics.json'))
groups={'Native':['native_1','native_2'],'Gradient':['gradient_1','gradient_2'],'ALP':['alp_1','alp_2','alp_3','alp_4']}
gates=[('A1','CROSSING',0),('B1','DWELL',1),('C1','LEFT_OPEN',2),('C2','RIGHT_OPEN',3),('A2','CROSSING',4)]
def pct(vals,q):
 vals=sorted(vals); x=(len(vals)-1)*q; lo=int(x); hi=min(lo+1,len(vals)-1); return vals[lo]+(vals[hi]-vals[lo])*(x-lo)
def fmt(x): return f'{x:.3f}' if isinstance(x,(int,float)) else str(x)
def status(v):
 if v['collision_episodes']>0:return 'COLLISION'
 if v['unsafe_episodes']>0:return 'UNSAFE'
 return 'SAFE'
def gate_line(r,oid):
 vals=[r['by_obstacle'][f'uav{u}_obstacle{oid}'] for u in (1,2,3)]
 s=[status(v) for v in vals]; st='COLLISION' if 'COLLISION' in s else 'UNSAFE' if 'UNSAFE' in s else 'SAFE'
 return st, min(v['min'] for v in vals), sum(v['collision_episodes'] for v in vals), sum(v['unsafe_episodes'] for v in vals), [u for u,x in zip((1,2,3),s) if x!='SAFE']
lines=['# Dynamic Gates Three-Version Repeatability Report','',f'Scene: `long_cylinder_forest_dynamic_gates.json`; target-facing yaw ON; RViz OFF; max_jer=22; serial runs; cutoff at target stop.','', 'Execution smoothness is odometry twist + finite difference (not MINCO analytic jerk).','']
lines += ['## Gate summary','', '| Gate | Native n=2 | Gradient n=2 | ALP n=4 |','|---|---|---|---|']
for name,typ,oid in gates:
 cells=[]
 for grp,names in groups.items():
  rs=[r for r in M['runs'] if r['run'] in names]; ss=[]
  for r in rs:ss.append(gate_line(r,oid)[0])
  cells.append(f"{', '.join(ss)} (safe {ss.count('SAFE')}/{len(ss)})")
 lines.append(f'| {name} {typ} | {cells[0]} | {cells[1]} | {cells[2]} |')
lines += ['', '## Full-run aggregates','', '| Metric | Native | Gradient | ALP |','|---|---:|---:|---:|']
keys=[('dynamic_collision_episodes','Dynamic collision episodes/run'),('dynamic_collision_samples','Dynamic collision samples/run'),('dynamic_unsafe_episodes','Unsafe episodes/run'),('min_dynamic','Min dynamic clearance'),('tracking_p95','Tracking P95'),('mean_path_length','Mean path length'),('static_collision_episodes','Static collision episodes'),('min_static','Min static clearance')]
for k,label in keys:
 row=[]
 for grp,names in groups.items():
  rs=[r for r in M['runs'] if r['run'] in names]; a=[r[k] for r in rs]; row.append(f'{statistics.mean(a):.3f} (min {min(a):.3f}, max {max(a):.3f})')
 lines.append(f'| {label} | '+' | '.join(row)+' |')
for k,label in [('jerk_p95','Executed jerk P95'),('jerk_rms','Executed jerk RMS'),('jerk_max','Executed jerk max'),('latency_p95','Planning latency P95')]:
 row=[]
 for grp,names in groups.items():
  rs=[r for r in M['runs'] if r['run'] in names]
  if k.startswith('jerk_'):
   a=[sum(r['per_uav'][str(u)]['smoothness'][k] for u in (1,2,3))/3 for r in rs]
  else:
   a=[r['latency']['p95'] for r in rs]
  row.append(f'{statistics.mean(a):.3f}')
 lines.append(f'| {label} | '+' | '.join(row)+' |')
lines += ['', '## Visibility (mean across runs)','', '| Metric | Native | Gradient | ALP |','|---|---:|---:|---:|']
for vk,label in [('uav1','UAV1'),('uav2','UAV2'),('uav3','UAV3'),('all3','All-3'),('atleast2','At-least-2'),('atleast1','At-least-1'),('none','None-visible')]:
 row=[]
 for grp,names in groups.items():
  rs=[r for r in M['runs'] if r['run'] in names]; row.append(f"{100*statistics.mean(r['visibility'][vk] for r in rs):.2f}%")
 lines.append(f'| {label} | '+' | '.join(row)+' |')
lines += ['', '## Gate detail by run','']
for grp,names in groups.items():
 lines += [f'### {grp}','', '| Run | Gate | Result | UAVs involved | Min clearance (m) | Collision episodes | Unsafe episodes |','|---|---|---|---|---:|---:|---:|']
 for rn in names:
  r=next(x for x in M['runs'] if x['run']==rn)
  for name,typ,oid in gates:
   st,mc,ce,ue,us=gate_line(r,oid); lines.append(f'| {rn} | {name} {typ} | {st} | {us or "-"} | {mc:.3f} | {ce} | {ue} |')
lines += ['', '## Collision contexts','', '| Run | UAV | Gate/obstacle | Window (s) | Min clearance (m) | Relative motion |','|---|---:|---|---|---:|---|']
for r in M['runs']:
 for c in r['contexts']:
  if int(c['obstacle']) < 5:
   lines.append(f"| {r['run']} | {c['uav']} | {gates[int(c['obstacle'])][0]} / {c['obstacle']} | {c['start_s']:.3f}–{c['end_s']:.3f} | {c['min_clearance']:.3f} | {gates[int(c['obstacle'])][1]} |")
lines += ['', '## ALP side/risk notes','', 'ALP risk-candidate logs were checked. For C1 (obstacle 2), observed non-nominal selections were SIDE_MINUS in runs alp_1 and alp_2; alp_3/alp_4 remained NOMINAL. For C2 (obstacle 3), non-nominal selections were SIDE_MINUS in all four runs. The artifacts do not encode a canonical LEFT/RIGHT label mapping, so C1/C2 geometric side correctness is not inferred from the sign alone.']
lines += ['', '## Required roll-up','',f'NATIVE_COLLISION_RUNS: {sum(any(gate_line(r,o)[0]=="COLLISION" for _,_,o in gates) for r in M["runs"] if r["run"].startswith("native"))}/2',f'GRADIENT_COLLISION_RUNS: {sum(any(gate_line(r,o)[0]=="COLLISION" for _,_,o in gates) for r in M["runs"] if r["run"].startswith("gradient"))}/2',f'ALP_COLLISION_RUNS: {sum(any(gate_line(r,o)[0]=="COLLISION" for _,_,o in gates) for r in M["runs"] if r["run"].startswith("alp"))}/4']
for name,typ,oid in gates:
 safe=sum(gate_line(r,oid)[0]=='SAFE' for r in M['runs'] if r['run'].startswith('alp')); lines.append(f'{name}_ALP_SAFE: {safe}/4')
lines += ['', 'Build: not required (no source changes). Runtime: all 8 runs reached target stop; no planner crash/dimension error observed.']
open(D/'DYNAMIC_GATES_THREE_VERSION_REPEATABILITY_REPORT.md','w').write('\n'.join(lines)+'\n')
print(D/'DYNAMIC_GATES_THREE_VERSION_REPEATABILITY_REPORT.md')
