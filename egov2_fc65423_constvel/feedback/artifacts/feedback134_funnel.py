from feedback134_candidate_oracle import *
o=pd.read_csv(OUT/f'{RID}_candidate_oracle.csv');f=pd.read_csv(OUT/f'{RID}_candidate_fates.csv');eps=pd.read_csv(OUT/f'{RID}_episodes.csv').set_index('episode')
final=o[o.stage=='FINAL_INPUT'].copy();final['certified_recovery']=final.hard_safe & (final.sustained_recovery_time+.3<=final.checked_until+1e-6)
final['episode_gain']=eps.loc[final.episode,'loss_end'].to_numpy()-final.sustained_recovery_time
rows=[]
for (ep,dr,b),g in final.groupby(['episode','drone','batch']):
 selected=g[g.selected];nom=g[g.kind=='NOMINAL'];reference=selected.iloc[0] if len(selected) else (nom.iloc[0]if len(nom) else None)
 if reference is None:continue
 for _,c in g.iterrows():
  r=dict(episode=ep,drone=dr,batch=b,batch_s=c.batch_s,candidate=c.candidate,kind=c.kind,reference=reference.candidate,refkind=reference.kind,selected=c.selected,committed=c.committed,executed=c.executed,hard_safe=c.hard_safe,certified_recovery=c.certified_recovery,recovery=c.sustained_recovery_time,refrecovery=reference.sustained_recovery_time,loss=c.common_K3_loss_seconds,refloss=reference.common_K3_loss_seconds,recovery_gain=reference.sustained_recovery_time-c.sustained_recovery_time,loss_gain=reference.common_K3_loss_seconds-c.common_K3_loss_seconds,checked_until=c.checked_until,reason=c.comparator_reason,K2=c.K2,C3=c.C3,D3=c.D3,refK2=reference.K2,refC3=reference.C3,refD3=reference.D3,comparison_begin=c.comparison_begin,comparison_end=c.comparison_end,first_reject=c.first_reject,reject_reason=c.reject_reason)
  r['better_than_selected']=len(selected)>0 and c.candidate!=reference.candidate and np.isfinite(c.sustained_recovery_time) and ((r['recovery_gain']>.025 and r['loss_gain']>.025) or (not np.isfinite(reference.sustained_recovery_time) and r['loss_gain']>.025))
  rows.append(r)
p=pd.DataFrame(rows);p.to_csv(OUT/'same_batch_comparisons.csv',index=False)
print('CERTIFIED_FASTER_THAN_SELECTED');print(p[p.certified_recovery&p.better_than_selected].to_string(index=False))
print('ALL_HARD_FASTER_THAN_SELECTED');print(p[p.hard_safe&p.better_than_selected][['episode','batch_s','candidate','kind','reference','refkind','recovery_gain','loss_gain','certified_recovery','reason']].to_string(index=False))
print('CERT_EP_SUMMARY');print(final.groupby('episode').apply(lambda g:pd.Series(dict(final=len(g),hard=g.hard_safe.sum(),certified=g.certified_recovery.sum(),certified_earlier=(g.certified_recovery&(g.episode_gain>.025)).sum(),selected_certified_earlier=(g.certified_recovery&(g.episode_gain>.025)&g.selected).sum()))).to_string())
# Geometry preservation: compare only same candidate pair and report safe-vs-unchecked explicitly.
seed=o[o.stage=='TIMED_SEED'];pre=o[o.stage=='PRE_LOS'];post=o[o.stage=='POST_LOS']
pp=pre.merge(post,on=['episode','drone','candidate'],suffixes=('_pre','_post'))
print('LOS_UNIQUE',len(pp.drop_duplicates(['drone','candidate'])),'PVA_BROKEN',len(pp[pp.pvaj_strict_ok_pre&~pp.pvaj_strict_ok_post].drop_duplicates(['drone','candidate'])))
for stage in ['TIMED_SEED','PRE_LOS']:
 x=o[o.stage==stage].merge(p[['episode','drone','candidate','refrecovery','refloss']],on=['episode','drone','candidate'])
 x['faster']=((x.refrecovery-x.sustained_recovery_time>.025)|(~np.isfinite(x.refrecovery)&np.isfinite(x.sustained_recovery_time)))&(x.refloss-x.common_K3_loss_seconds>.025)
 x=x.merge(final[['episode','drone','candidate','sustained_recovery_time','common_K3_loss_seconds']],on=['episode','drone','candidate'],suffixes=('','_final'))
 x['collapsed']=x.faster&((x.sustained_recovery_time_final-x.sustained_recovery_time>.025)|~np.isfinite(x.sustained_recovery_time_final))&(x.common_K3_loss_seconds_final-x.common_K3_loss_seconds>.025)
 x.to_csv(OUT/f'{stage}_preservation.csv',index=False)
 print(stage,'faster',len(x[x.faster]),'faster_pva',len(x[x.faster&x.pvaj_strict_ok]),'collapsed',len(x[x.collapsed]),'collapsed_safe_seed',len(x[x.collapsed&x.seed_eligible]))
 print(x[x.faster&x.pvaj_strict_ok][['episode','candidate','kind','seed_eligible','hard_safe','first_reject','reject_reason','sustained_recovery_time','sustained_recovery_time_final','common_K3_loss_seconds','common_K3_loss_seconds_final','refrecovery','refloss','collapsed']].to_string(index=False))
