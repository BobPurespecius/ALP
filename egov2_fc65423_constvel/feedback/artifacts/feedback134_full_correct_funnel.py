from feedback134_candidate_oracle import *
v,dfs,epoch=loadrun(RID);o=pd.read_csv(OUT/f'{RID}_candidate_oracle.csv');e=pd.read_csv(OUT/f'{RID}_episodes.csv').set_index('episode')
# Independent reference: executed K3 over EXACT candidate common window.
for ix,r in o.iterrows():
 ts=np.arange(r.begin,float(e.loc[r.episode,'loss_end'])+.3+1e-8,.01);idx=np.clip(np.searchsorted(v.time_s.to_numpy(),ts,side='right')-1,0,len(v)-1)
 o.loc[ix,'executed_common_loss']=float((v.visible_count.to_numpy()[idx]<3).sum()*.01)
o['faster_than_executed']=o.common_window_covered & (o.sustained_K3_recovery_time<e.loc[o.episode,'loss_end'].to_numpy()-.05)
o['no_total_loss_regression']=o.common_K3_loss_seconds<=o.executed_common_loss+.025
# Counts include N and S, selected and unselected; no selection-conditioned filter.
z=o[(o.stage=='FINAL_INPUT')&o.faster_than_executed&o.no_total_loss_regression].copy()
z['certified']=z.hard_safe&(z.sustained_recovery_time+.3<=z.checked_until+1e-6)
z['truth_body_ok']=(z.body_static_clearance>.099)&(z.body_dynamic_clearance>.384)
# Whole-polynomial primitive body bound is conservative; full preflight still required.
z.to_csv(OUT/'execution_referenced_final_funnel.csv',index=False)
y=z[z.certified&z.truth_body_ok].drop_duplicates(['drone','candidate'])
print('FINAL_GEOMETRIC_RECOVERY_NO_TOTAL_LOSS_REGRESSION',len(z.drop_duplicates(['drone','candidate'])),'PRECHECK_HARD',len(z[z.hard_safe].drop_duplicates(['drone','candidate'])),'CERT_AND_TRUTH_BODY',len(y))
print(y[['episode','drone','candidate','kind','selected','committed','executed','sustained_recovery_time','common_K3_loss_seconds','executed_common_loss','execution_begin','execution_end']].to_string(index=False))
print('SELECTED/COMMITTED/EXECUTED',int(y.selected.sum()),int(y.committed.sum()),int(y.executed.sum()))
print('EXECUTED_TO_RECOVERY',int((y.executed&(y.execution_end>=y.sustained_recovery_time)).sum()))
print('EXECUTED_RECOVERY_AND_DWELL',int((y.executed&(y.execution_end>=y.sustained_recovery_time+.3)).sum()))
print('EP',y.episode.nunique(),y.episode.unique())
seed=o[o.stage=='TIMED_SEED'];g=seed[seed.faster_than_executed&seed.no_total_loss_regression&seed.seed_eligible].drop_duplicates(['drone','candidate']);print('SEED_GENERATED_EXEC_REFERENCE',len(g))
