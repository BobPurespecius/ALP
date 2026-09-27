from feedback134_candidate_oracle import *
v,dfs,epoch=loadrun(RID);rs=read_jsonl(DIAG/'candidate_drone_2.jsonl');out=[]
for cid in [146,147,148,149,150,151]:
 r=next(x for x in rs if x.get('candidate')==cid and x['stage']=='FINAL_INPUT');ctx=next(x for x in rs if x.get('candidate')==cid and x['stage']in ['COMPARATOR_WIN','COMPARATOR_LOSS']);t0=ctx['comparison_begin'];t1=ctx['comparison_end'];world=np.arange(np.ceil(t0/.1)*.1,t1+1e-8,.1);ts=world-epoch;p=eval_poly(r,world);target=interp(dfs[3],ts,['target_x','target_y','target_z']);pred=np.array(ctx['target'])+(world-ctx['target_epoch'])[:,None]*np.array(ctx['target_v']);cl=clearances(p,target,ts);b=cl.min(axis=1)>0
 world2=np.arange(r['activation'],epoch+36.7022,.005);t2=world2-epoch;p2=eval_poly(r,world2);tar2=interp(dfs[3],t2,['target_x','target_y','target_z']);cl2=clearances(p2,tar2,t2);b2=cl2.min(axis=1)>0
 print(cid,'window',t0-epoch,t1-epoch,'prodC3',ctx['C3'],'truth_window_LOS',b.mean(),'target_pred_error',np.linalg.norm(pred-target,axis=1).max(),'LOS_FALSE',[(round(t2[a],6),round(t2[b-1]+.005,6))for a,b in intervals(~b2)])
 out.append(dict(candidate=cid,window_begin=t0-epoch,window_end=t1-epoch,C3=ctx['C3'],truth_LOS_fraction=b.mean(),target_error=float(np.linalg.norm(pred-target,axis=1).max())))
pd.DataFrame(out).to_csv(OUT/'window_replay.csv',index=False)
