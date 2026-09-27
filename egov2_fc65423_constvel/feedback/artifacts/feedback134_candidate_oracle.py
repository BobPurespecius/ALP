from feedback134_analyze import *
import hashlib
RID='20260927_103759_1243248'
DIAG=OUT/'diagnostic_full'
KINDS={0:'NOMINAL',1:'SIDE_PLUS',2:'SIDE_MINUS'}

def read_jsonl(path):
 rows=[]
 for line in path.open():
  try:rows.append(json.loads(line))
  except json.JSONDecodeError:pass # live trailing partial line only
 return rows

def eval_poly(r,world,derivative=0):
 pieces=np.asarray(r['pieces'],float);duration=pieces[:,0];coeff=pieces[:,1:].reshape(-1,3,6)
 starts=np.r_[0,np.cumsum(duration)[:-1]];t=np.asarray(world)-r['activation']
 ix=np.clip(np.searchsorted(np.cumsum(duration),t,side='right'),0,len(pieces)-1)
 local=t-starts[ix];out=np.zeros((len(t),3))
 for i in np.unique(ix):
  mask=ix==i
  for axis in range(3):out[mask,axis]=np.polyval(np.polyder(coeff[i,axis],derivative),local[mask])
 return out

MAXCACHE={}
def max_derivatives(r):
 key=hashlib.sha256(np.asarray(r['pieces'],float).tobytes()).hexdigest()
 if key in MAXCACHE:return MAXCACHE[key]
 vals=[]
 for order in (1,2,3):
  mx=0.
  for piece in r['pieces']:
   T=piece[0];c=np.array(piece[1:]).reshape(3,6)*T**np.arange(5,-1,-1)
   dc=np.array([np.polyder(x,order)/T**order for x in c]);sq=sum(np.polymul(x,x)for x in dc)
   derivative=np.trim_zeros(np.polyder(sq),'f');roots=np.roots(derivative) if len(derivative)>1 else []
   us=[0.,1.]+[float(x.real)for x in roots if abs(x.imag)<1e-7 and 0<x.real<1]
   mx=max(mx,max(np.linalg.norm([np.polyval(x,u)for x in dc])for u in us))
  vals.append(float(mx))
 MAXCACHE[key]=vals;return vals

def yaw_step(yaw,rate,desired,dt):
 if dt<=1e-6:return yaw,rate
 diff=(desired-yaw+np.pi)%(2*np.pi)-np.pi;sign=1 if diff>=0 else -1;acc=sign*5*np.pi;limit=sign*2*np.pi
 if abs(rate+dt*acc)<=2*np.pi:step=rate*dt+.5*acc*dt*dt;endrate=rate+dt*acc
 else:
  at=(limit-rate)/acc;step=rate*dt+((dt-at)+dt)*(limit-rate)/2;endrate=limit
 reached=abs(diff)<=abs(step)
 return ((yaw+(diff if reached else step)+np.pi)%(2*np.pi)-np.pi,0. if reached else endrate)

def first_persistent(vis,t,anchor,dwell=.3):
 mask=t>=anchor-1e-8
 for a,b in intervals(np.asarray(vis)&mask):
  if t[b-1]-t[a]>=dwell-1e-8:return float(t[a])
 return None

def oracle(r,context,ep,dfs,v,epoch,dt=.01):
 if not r.get('pieces'):return None
 start=r['activation']-epoch;duration=sum(p[0]for p in r['pieces']);end=min(start+duration,float(ep.loss_end)+.6,dfs[int(ep.uav)].time_s.max())
 if end<=start+.02:return None
 ts=np.arange(start,end+1e-8,dt);world=ts+epoch;p=eval_poly(r,world)
 target=interp(dfs[int(ep.uav)],ts,['target_x','target_y','target_z'])
 cl=clearances(p,target,ts);los=cl.min(axis=1)>0
 loslabel=str(ep.blockers).split(',')[0];k=next((i for i,o in enumerate(OBS)if f'{o[0]}:{o[1]}'==loslabel),None)
 if k is None:return None
 c=center(OBS[k],ts);e=c-target[:,:2];e/=np.maximum(np.linalg.norm(e,axis=1)[:,None],1e-9);right=np.column_stack([e[:,1],-e[:,0]])
 s=np.sum(right*(p[:,:2]-c),axis=1)
 prodright=np.array(context.get('frame_right',[0,0,0]),float)[:2];prodside=(p[:,:2]-c)@prodright
 d=target-p;dist=np.linalg.norm(d,axis=1);bearing=np.arctan2(d[:,1],d[:,0])
 yaw=float(context.get('self_yaw',np.interp(start,dfs[int(ep.uav)].time_s,dfs[int(ep.uav)].actual_yaw_rad)));rate=float(context.get('self_yaw_rate',0));previous=float(context.get('self_odom_stamp',r['activation']))-epoch
 fov=[]
 vfov=2*np.arctan(np.tan(np.deg2rad(42.5))*720/1280)
 for t,des,dd in zip(ts,bearing,d):
  yaw,rate=yaw_step(yaw,rate,des,t-previous);previous=t
  horizontal=abs((des-yaw+np.pi)%(2*np.pi)-np.pi);vertical=abs(np.arctan2(dd[2],np.linalg.norm(dd[:2])))
  fov.append(horizontal<=np.deg2rad(42.5) and vertical<=vfov/2)
 vis=los&(dist>=.2)&(dist<=8)&np.array(fov)
 team=vis.copy()
 for peer in (1,2,3):
  if peer==int(ep.uav):continue
  ix=np.clip(np.searchsorted(v.time_s.to_numpy(),ts,side='right')-1,0,len(v)-1)
  team &= v[f'visible_uav{peer}'].to_numpy()[ix].astype(bool)
 anchor=max(start,float(ep.loss_begin));mask=ts>=anchor-1e-8
 binary_idx=np.flatnonzero(vis&mask);shadow_idx=np.flatnonzero((cl[:,k]>0)&mask)
 eligible=mask&(ts<float(ep.loss_end))
 side_at_anchor=float(np.interp(anchor,ts,s))
 # Sustained exit: after the LAST hidden sample through the common episode end + 0.3 s.
 # This prevents a transient visible prefix followed by another shadow crossing
 # from being counted as successful persistent recovery.
 stable_end=float(ep.loss_end)+.3
 def sustained(maskvis):
  m=(ts>=anchor-1e-8)&(ts<=stable_end+1e-8)
  if end<stable_end-1e-6 or not np.any(m):return None
  ids=np.flatnonzero(m&~maskvis)
  a=(ids[-1]+1)if len(ids)else np.flatnonzero(m)[0]
  if a>=len(ts)or stable_end-ts[a]<.3-.011:return None
  return float(ts[a])
 sustained_self=sustained(vis);sustained_team=sustained(team)
 vmax,amax,jmax=max_derivatives(r)
 # Body-clearance is an independent analytic-primitive check, not an occupancy certificate.
 body_static=np.inf;body_dynamic=np.inf
 for o in OBS:
  rad=o[2].get('radius',.5);bc=np.linalg.norm(p[:,:2]-center(o,ts),axis=1)-rad
  if o[0]=='STATIC':body_static=min(body_static,float(bc.min()))
  else:body_dynamic=min(body_dynamic,float(bc.min()))
 return dict(common_K3_loss_seconds=float(np.sum((ts<=stable_end+1e-8)&~team)*dt),common_self_loss_seconds=float(np.sum((ts<=stable_end+1e-8)&~vis)*dt),sustained_recovery_time=sustained_self,sustained_K3_recovery_time=sustained_team,common_window_covered=end>=stable_end-1e-6,begin=start,end=end,duration=duration,actual_geometric_side=f'{s[0]:+.6f}->{s[-1]:+.6f}',side_start=float(s[0]),side_at_loss=side_at_anchor,side_end=float(s[-1]),production_frame_side_start=float(prodside[0]),production_frame_side_end=float(prodside[-1]),axis_crossings=int(np.sum(s[:-1]*s[1:]<0)),min_shadow_clearance=float(cl[:,k].min()),min_all_los_clearance=float(cl.min()),shadow_exit_time=float(ts[shadow_idx[0]])if len(shadow_idx)else None,first_binary_recovery_time=float(ts[binary_idx[0]])if len(binary_idx)else None,first_persistent_recovery_time=first_persistent(vis,ts,anchor),persistent_02=first_persistent(vis,ts,anchor,.2),persistent_05=first_persistent(vis,ts,anchor,.5),first_persistent_K3_recovery_time=first_persistent(team,ts,anchor),K3_loss_seconds=float(np.sum(eligible&~team)*dt),self_loss_seconds=float(np.sum(eligible&~vis)*dt),LOS_loss_seconds=float(np.sum(eligible&~los)*dt),fov_loss_seconds=float(np.sum(eligible&~np.array(fov))*dt),max_v=vmax,max_a=amax,max_j=jmax,pvaj_strict_ok=vmax<=3.00001 and amax<=6.00001 and jmax<=20.00001,body_static_clearance=body_static,body_dynamic_clearance=body_dynamic)

def main(rid=RID):
 v,dfs,epoch=loadrun(rid);ep=pd.read_csv(OUT/f'{rid}_episodes.csv')
 records=sum([read_jsonl(p)for p in DIAG.glob('candidate_drone_*.jsonl')],[])
 records.sort(key=lambda r:r['world']);bycand=collections.defaultdict(list)
 for r in records:
  if r.get('candidate',-1)>=0 and r['stage']!='SIDE_BUDGET_SKIP':bycand[(r['drone'],r['candidate'])].append(r)
 los_records=sum([read_jsonl(p)for p in DIAG.glob('los_drone_*.jsonl')],[])
 unmatched=[]
 for r in los_records:
  kinds={'NOMINAL':0,'SIDE_PLUS':1,'SIDE_MINUS':2};kind=kinds.get(r['kind'])
  matches=[(key,rows)for key,rows in bycand.items()if key[0]==r['drone'] and rows[0].get('kind')==kind and any(x['stage']in ('SOLVER_OK','SOLVER_FAILED') and 0<=x['world']-r['world']<2. and abs(x['activation']-r['activation'])<1e-5 for x in rows)]
  if len(matches)==1:
   key,rows=matches[0];r['candidate']=key[1];r['kind']=kind;rows.append(r)
  else:unmatched.append(r)
 output=[];fates=[]
 for _,e in ep.iterrows():
  if not isinstance(e.blockers,str)or not e.blockers:continue
  dr=int(e.uav)-1
  for (d,cid),rows in bycand.items():
   if d!=dr:continue
   rows.sort(key=lambda x:x['world']);ct=next((x for x in rows if x['stage']=='FINAL_INPUT'),rows[0]);batch=ct.get('batch',rows[0].get('batch',0))
   t=batch-epoch
   if not e.loss_begin-2.5<=t<e.loss_end:continue
   latest={}
   for r in rows:latest[r['stage']]=r
   context=latest.get('COMPARATOR_LOSS',latest.get('COMPARATOR_WIN',ct))
   hard=any(s in latest for s in ['COMPARATOR_WIN','COMPARATOR_LOSS'])
   commit=latest.get('COMMITTED');traj_id=commit.get('related_id')if commit else None
   executed=dfs[int(e.uav)]
   executed_match=executed[executed.trajectory_id==traj_id]if traj_id is not None else executed.iloc[0:0]
   fate=dict(run=rid,episode=e.episode,drone=d,candidate=cid,kind=KINDS[ct['kind']],batch=batch,batch_s=t,activation=ct.get('activation'),hard_safe=hard,seed_eligible=latest.get('SEED_ELIGIBILITY',{}).get('related_id')==1,selected='SELECTED'in latest,committed=commit is not None,trajectory_id=traj_id,executed=len(executed_match)>0,execution_begin=float(executed_match.time_s.min())if len(executed_match)else None,execution_end=float(executed_match.time_s.max())if len(executed_match)else None,stages='|'.join(r['stage']for r in rows),first_reject='',reject_reason='',solver_reason=latest.get('SOLVER_FAILED',{}).get('reason',''),comparator_reason=context.get('reason',''),K2=context.get('K2'),C3=context.get('C3'),D3=context.get('D3'),comparison_begin=context.get('comparison_begin',0)-epoch,comparison_end=context.get('comparison_end',0)-epoch,checked_until=context.get('checked_until',0)-epoch,fallback=ct.get('fallback',0))
   for stage in ['PREINIT_REJECT','FINAL_INPUT_FAILED','HANDOFF_REJECT','STATIONARY_REJECT','PREFLIGHT_REJECT','CERTIFICATE_REJECT','FINALIZE_DEADLINE_STOP','COMMIT_FAILED']:
    if stage in latest:fate['first_reject']=stage;fate['reject_reason']=latest[stage].get('reason','');break
   fates.append(fate)
   for stage in ['RAW_SEED_ATTEMPT','PRE_TIMING_SEED','TIMED_SEED','JOINT_SEED','PRE_LOS','POST_LOS','SOLVER_OK','SOLVER_FAILED','FINAL_INPUT']:
    r=latest.get(stage)
    if r and r.get('pieces'):
     met=oracle(r,context,e,dfs,v,epoch)
     if met:output.append(dict(fate,stage=stage,**met))
 pd.DataFrame(output).to_csv(OUT/f'{rid}_candidate_oracle.csv',index=False)
 pd.DataFrame(fates).to_csv(OUT/f'{rid}_candidate_fates.csv',index=False)
 (OUT/f'{rid}_unmatched_los.json').write_text(json.dumps(unmatched,indent=2))
 print('records',len(records),'candidates',len(bycand),'oracle_stage_rows',len(output),'candidate_episode_rows',len(fates),'unmatched_los',len(unmatched))
 print(pd.DataFrame(fates).groupby(['episode','first_reject']).size().to_string())

if __name__=='__main__':main(sys.argv[1]if len(sys.argv)>1 else RID)
