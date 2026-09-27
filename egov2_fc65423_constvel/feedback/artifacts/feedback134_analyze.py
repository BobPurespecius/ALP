"""Read-only replay of retained ALP run data. Writes audit artifacts only."""
from pathlib import Path
import numpy as np, pandas as pd, json, gzip, re, collections, sys
ROOT=Path('/home/bob/ALP/egov2_fc65423_constvel')
OUT=ROOT/'feedback/artifacts/feedback134'; OUT.mkdir(exist_ok=True)
SCENE=json.loads((ROOT/'ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v6.json').read_text())
OBS=[(motion,int(k),x) for motion,field in [('STATIC','obstacleData'),('DYNAMIC','movingObstacleData')] for k,x in SCENE[field].items()]

def intervals(bad):
 bad=np.asarray(bad,dtype=bool)
 return list(zip(np.flatnonzero(bad & ~np.r_[False,bad[:-1]]),np.flatnonzero(bad & ~np.r_[bad[1:],False])+1))

def center(obs,ts):
 motion,i,o=obs;ts=np.asarray(ts)
 p=np.tile(np.array(o['centerENU'][:2]),(len(ts),1)).astype(float)
 if motion=='DYNAMIC':
  axis=np.array(o.get('axisENU',[1.,0.]),float);axis/=np.linalg.norm(axis)
  p += np.sin(2*np.pi*ts/max(o['period'],1e-6)+o.get('phase',0))[:,None]*o.get('amplitude',0)*axis[None,:]
 return p

def clearances(pos,target,ts,margin=.08):
 """Exact closest XY point after clipping sight segment to cylinder z slab."""
 pos=np.asarray(pos);target=np.asarray(target);d=target-pos
 result=[]
 for obs in OBS:
  o=obs[2];c=center(obs,ts);zmin=o.get('zMin',0);zmax=zmin+o.get('height',3.)
  dz=d[:,2]; flat=np.abs(dz)<1e-9
  inv=np.divide(1.,dz,out=np.zeros_like(dz),where=~flat)
  za=(zmin-pos[:,2])*inv;zb=(zmax-pos[:,2])*inv
  lo=np.where(flat,0,np.maximum(0,np.minimum(za,zb)))
  hi=np.where(flat,1,np.minimum(1,np.maximum(za,zb)))
  eligible=(lo<=hi)&(~flat | ((pos[:,2]>=zmin)&(pos[:,2]<=zmax)))
  den=np.sum(d[:,:2]**2,axis=1)
  alpha=np.divide(np.sum((c-pos[:,:2])*d[:,:2],axis=1),den,out=np.zeros(len(ts)),where=den>1e-12)
  alpha=np.minimum(hi,np.maximum(lo,alpha))
  dist=np.linalg.norm(pos[:,:2]+alpha[:,None]*d[:,:2]-c,axis=1)-o.get('radius',.5)-margin
  result.append(np.where(eligible,dist,np.inf))
 return np.array(result).T

def interp(df,t,cols):
 return np.column_stack([np.interp(t,df.time_s,df[c]) for c in cols])

def loadrun(rid):
 d=ROOT/'runs'/rid
 v=pd.read_csv(d/'visibility.csv');tr=pd.read_csv(d/'visibility_trajectory.csv')
 epoch=float(np.median(tr.timestamp-tr.time_s))
 return v,{u:tr[tr.uav_id==u].sort_values('time_s').drop_duplicates('time_s') for u in (1,2,3)},epoch

def audit_run(rid):
 v,dfs,epoch=loadrun(rid);ts=v.time_s.to_numpy();dt=np.r_[np.diff(ts),np.median(np.diff(ts))]
 rows=[];details=[];validation=[]
 for u,df in dfs.items():
  p=interp(df,ts,['x','y','z']);target=interp(df,ts,['target_x','target_y','target_z'])
  cl=clearances(p,target,ts);ns=len(SCENE['obstacleData']);sm=cl[:,:ns].min(axis=1);dm=cl[:,ns:].min(axis=1)
  validation.append(dict(uav=u,static_disagreement=int(((sm>0)!=v[f'uav{u}_static_los_clear']).sum()),dynamic_disagreement=int(((dm>0)!=v[f'uav{u}_dynamic_los_clear']).sum()),n=len(ts)))
  for j,(a,b) in enumerate(intervals(v[f'visible_uav{u}'].eq(0)),1):
   counts=collections.Counter()
   for idx in range(a,b):
    for k in np.flatnonzero(cl[idx]<=0):counts[(OBS[k][0],OBS[k][1])]+=1
   ranked=counts.most_common();labels=[f'{x[0]}:{x[1]}' for x,n in ranked]
   primary=ranked[0][0] if ranked else None
   row=dict(run=rid,episode=f'u{u}e{j}',uav=u,loss_begin=float(ts[a]),loss_end=float(ts[b] if b<len(ts) else ts[-1]+dt[-1]),loss_seconds=float(dt[a:b].sum()),world_begin=float(epoch+ts[a]),world_end=float(epoch+(ts[b] if b<len(ts) else ts[-1]+dt[-1])),blockers=','.join(labels),blocker_counts=str(ranked),static_loss_samples=int(v.iloc[a:b][f'uav{u}_static_los_clear'].eq(0).sum()),dynamic_loss_samples=int(v.iloc[a:b][f'uav{u}_dynamic_los_clear'].eq(0).sum()),fov_loss_samples=int(v.iloc[a:b][f'uav{u}_camera_fov_valid'].eq(0).sum()))
   if primary:
    k=next(k for k,x in enumerate(OBS) if x[:2]==primary);c=center(OBS[k],ts[a:b]);e=c-target[a:b,:2];e/=np.maximum(np.linalg.norm(e,axis=1)[:,None],1e-9);right=np.column_stack([e[:,1],-e[:,0]]);side=np.sum(right*(p[a:b,:2]-c),axis=1)
    row.update(actual_geometric_side=f'{side[0]:+.4f}->{side[-1]:+.4f}',side_crossings=int(np.sum(side[:-1]*side[1:]<0)),min_shadow_clearance=float(cl[a:b,k].min()))
   rows.append(row)
  # Sampled executed geometry for all visibility loss neighborhoods.
  for a,b in intervals(v[f'visible_uav{u}'].eq(0)):
   for idx in range(max(0,a-30),min(len(ts),b+15)):
    blockers=np.flatnonzero(cl[idx]<=0)
    details.append(dict(run=rid,uav=u,time_s=ts[idx],x=p[idx,0],y=p[idx,1],target_x=target[idx,0],target_y=target[idx,1],static_clearance=sm[idx],dynamic_clearance=dm[idx],visible=int(v.iloc[idx][f'visible_uav{u}']),blockers='|'.join(f'{OBS[k][0]}:{OBS[k][1]}'for k in blockers)))
 union=[dict(begin=float(ts[a]),end=float(ts[b]if b<len(ts)else ts[-1]+dt[-1]),seconds=float(dt[a:b].sum())) for a,b in intervals(v.visible_count.lt(3))]
 summary=dict(run=rid,epoch=epoch,samples=len(v),K3_sample_fraction=float(v.visible_count.eq(3).mean()),K3_time_fraction=float(np.sum(dt*v.visible_count.eq(3))/sum(dt)),K3_loss_seconds=float(sum(x['seconds']for x in union)),K2_sample_fraction=float(v.visible_count.ge(2).mean()),uav_loss_events=len(rows),union_loss_events=len(union),union_episodes=union,oracle_validation=validation)
 pd.DataFrame(rows).to_csv(OUT/f'{rid}_episodes.csv',index=False)
 pd.DataFrame(details).drop_duplicates().to_csv(OUT/f'{rid}_executed_geometry.csv',index=False)
 (OUT/f'{rid}_summary.json').write_text(json.dumps(summary,indent=2))
 print(json.dumps(summary));print(pd.DataFrame(rows)[['episode','loss_begin','loss_end','blockers','actual_geometric_side','min_shadow_clearance']].to_string(index=False))
 return rows

if __name__=='__main__':
 for rid in sys.argv[1:] or ['20260927_055748_1104145','20260927_061256_1157439']:
  audit_run(rid)
