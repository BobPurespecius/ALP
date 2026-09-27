from feedback134_candidate_oracle import *
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
v,dfs,epoch=loadrun(RID);rows=sum([read_jsonl(p)for p in DIAG.glob('candidate_drone_*.jsonl')],[]);los=sum([read_jsonl(p)for p in DIAG.glob('los_drone_*.jsonl')],[])
def record(dr,cid,stage):
 if stage in ['PRE_LOS','POST_LOS']:
  solver=next(x for x in rows if x['drone']==dr and x.get('candidate')==cid and x['stage']=='SOLVER_OK')
  return next(x for x in los if x['drone']==dr and x['stage']==stage and abs(x['activation']-solver['activation'])<1e-5 and x['kind']==KINDS[solver['kind']])
 return next(x for x in rows if x['drone']==dr and x.get('candidate')==cid and x['stage']==stage)
fig,axs=plt.subplots(3,1,figsize=(10,9))
for cid,stage,label in [(35,'TIMED_SEED','35 seed'),(35,'FINAL_INPUT','35 final'),(34,'FINAL_INPUT','34 selected')]:
 r=record(0,cid,stage);ts=np.arange(7.15,9.12,.005);p=eval_poly(r,ts+epoch);tar=interp(dfs[1],ts,['target_x','target_y','target_z']);cl=clearances(p,tar,ts).min(axis=1);axs[0].plot(ts,cl,label=label)
axs[0].axhline(0,color='k',lw=.8);axs[0].set_title('A: valid seed loses its recovery advantage in main optimization');axs[0].set_ylabel('LOS clearance (m)');axs[0].legend()
for st in ['PRE_LOS','POST_LOS']:
 r=record(0,80,st);ts=np.arange(r['activation']-epoch,r['activation']-epoch+sum(p[0]for p in r['pieces']),.005);j=np.linalg.norm(eval_poly(r,ts+epoch,3),axis=1);axs[1].plot(ts,j,label=st)
axs[1].axhline(20,color='r',ls='--',label='jerk limit');axs[1].set_title('B: post-solve LOS correction breaks a PVA-safe solution');axs[1].set_ylabel('Jerk (m/s^3)');axs[1].legend()
for cid in [150,151]:
 r=record(2,cid,'FINAL_INPUT');ts=np.arange(34.85,36.69,.005);p=eval_poly(r,ts+epoch);tar=interp(dfs[3],ts,['target_x','target_y','target_z']);cl=clearances(p,tar,ts).min(axis=1);axs[2].plot(ts,cl,label=f'{cid} '+('selected'if cid==150 else'hard-safe alternative'))
axs[2].axhline(0,color='k',lw=.8);axs[2].axvspan(34.849324,36.227077,color='gray',alpha=.15,label='production comparison window');axs[2].set_title('C: shorter comparison window ranks the slower recovery higher');axs[2].set_ylabel('LOS clearance (m)');axs[2].legend();axs[2].set_xlabel('Seconds since target motion began')
for a in axs:a.grid(alpha=.2)
fig.tight_layout();fig.savefig(OUT/'causal_evidence.png',dpi=150);fig.savefig(OUT/'causal_evidence.pdf')
