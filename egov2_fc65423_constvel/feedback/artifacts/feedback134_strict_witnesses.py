from feedback134_candidate_oracle import *
q=pd.read_csv(OUT/'qualified_seed_fates.csv');q=q[q.within_batch_support]
p=pd.read_csv(OUT/'same_batch_comparisons.csv');o=pd.read_csv(OUT/f'{RID}_candidate_oracle.csv');f=pd.read_csv(OUT/f'{RID}_candidate_fates.csv');rows=[]
for x in q.itertuples():
 a=p[(p.episode==x.episode)&(p.drone==x.drone)&(p.candidate==x.candidate)].iloc[0]
 z=o[(o.episode==x.episode)&(o.drone==x.drone)&(o.candidate==x.candidate)&(o.stage=='FINAL_INPUT')].iloc[0]
 w=f[(f.episode==x.episode)&(f.drone==x.drone)&(f.candidate==a.reference)].iloc[0]
 pre=o[(o.episode==x.episode)&(o.drone==x.drone)&(o.candidate==x.candidate)&(o.stage=='PRE_LOS')]
 if a.better_than_selected and a.certified_recovery:primary='K';terminal='C3_LOSS'
 elif z.reject_reason=='DYNAMICS_FAIL':
  assert len(pre)==1
  # Verify loss of seed advantage precedes the final dynamics rejection.
  assert pre.iloc[0].common_K3_loss_seconds>=a.refloss-.025
  primary='B';terminal='DYNAMICS_FAIL'
 else:primary='B';terminal='QUALITY_LOSS_OR_DEGRADED_SELECTED'
 rows.append(dict(episode=x.episode,drone=x.drone,candidate=x.candidate,kind=x.kind,batch_s=x.batch_s,primary=primary,seed_recovery=x.sustained_recovery_time,final_recovery=z.first_persistent_recovery_time,seed_loss=x.common_K3_loss_seconds,final_loss=z.common_K3_loss_seconds,seed_j=x.max_j,final_j=z.max_j,winner=int(a.reference),winner_kind=a.refkind,winner_executed=w.executed,winner_loss=a.refloss,selected=z.selected,reason=z.comparator_reason,terminal_gate=terminal))
r=pd.DataFrame(rows);r.to_csv(OUT/'strict_witness_funnel.csv',index=False)
print(len(r),r.primary.value_counts().to_dict(),r.terminal_gate.value_counts().to_dict())
