from feedback134_candidate_oracle import *
o=pd.read_csv(OUT/f'{RID}_candidate_oracle.csv');f=pd.read_csv(OUT/f'{RID}_candidate_fates.csv');v,dfs,epoch=loadrun(RID)
records=sum([read_jsonl(p)for p in DIAG.glob('candidate_drone_*.jsonl')],[]); by=collections.defaultdict(list)
for r in records:by[(r['drone'],r.get('candidate',-1))].append(r)
keys=set(zip(f.drone,f.candidate)); ff=f.drop_duplicates(['drone','candidate']); oo=o.drop_duplicates(['drone','candidate','stage'])
print('WINDOW_UNIQUE',len(ff),'NOMINAL',sum(ff.kind=='NOMINAL'),'SIDE',sum(ff.kind!='NOMINAL'))
print('STAGES_UNIQUE',{s:len({(r['drone'],r.get('candidate'))for r in records if (r['drone'],r.get('candidate'))in keys and r['stage']==s})for s in sorted(set(r['stage']for r in records))})
print('FIRST_REJECT',ff.groupby(['first_reject'],dropna=False).size().to_dict());print('REJECT_REASON',ff.groupby(['reject_reason'],dropna=False).size().to_dict())
print('SOLVER_REASON',ff.groupby(['solver_reason'],dropna=False).size().to_dict())
print('COMPARATOR_REASON',ff[ff.hard_safe].groupby(['comparator_reason'],dropna=False).size().to_dict())
for dr in range(3):
 rr=[r for r in records if r['drone']==dr];print('WRITER',dr,max(r.get('writer_cumulative_us',0)for r in rr)/1e6,'RECORDS',len(rr))
identity_bad=[]
for key,rs in by.items():
 hs={r['poly_hash']for r in rs if r['stage'] in ('SOLVER_OK','FINAL_INPUT','SELECTED','COMMITTED') and r.get('poly_hash')}
 if len(hs)>1 and not any(r.get('fallback')for r in rs):identity_bad.append(key)
print('IDENTITY_CHANGED_WITHOUT_FALLBACK',identity_bad)
# per-window budget skips not candidate payloads, do not count nominal id as SIDE identity
es=pd.read_csv(OUT/f'{RID}_episodes.csv');skips=[]
for r in records:
 if r['stage']!='SIDE_BUDGET_SKIP':continue
 e=es[(es.uav==r['drone']+1)&(es.loss_begin-2.5<=r['batch']-epoch)&(es.loss_end>r['batch']-epoch)]
 if len(e):skips.append(dict(drone=r['drone'],batch=r['batch'],side=r['reason'],episodes=','.join(e.episode),time=r['batch']-epoch))
pd.DataFrame(skips).drop_duplicates(['drone','batch','side']).to_csv(OUT/'budget_skips.csv',index=False)
print('BUDGET_SKIPS_FULL',sum(r['stage']=='SIDE_BUDGET_SKIP'for r in records),'EP_UNIQUE',len(pd.DataFrame(skips).drop_duplicates(['drone','batch','side']))if skips else 0)
pre=oo[oo.stage=='PRE_LOS'];post=oo[oo.stage=='POST_LOS'];pp=pre.merge(post,on=['drone','candidate'],suffixes=('_pre','_post'));broken=pp[pp.pvaj_strict_ok_pre&~pp.pvaj_strict_ok_post]
print('LOS_PAIR',len(pp),'PVA_PRE_SAFE',int(pp.pvaj_strict_ok_pre.sum()),'BROKEN',len(broken),'FATES',broken.groupby('first_reject_post',dropna=False).size().to_dict())
q=pd.read_csv(OUT/'TIMED_SEED_preservation.csv');q=q[q.faster&q.seed_eligible].drop_duplicates(['drone','candidate']);print('QUALIFIED_SEED',len(q),'COLLAPSED',int(q.collapsed.sum()))
# Conservative recovery inside the actual batch certificate of a final comparator candidate,
# plus initializer ABSOLUTE_SAFE; still not a final certificate for the seed itself.
cert=ff.groupby(['drone','batch']).checked_until.max()
q['batch_cert_end']=[cert.loc[(r.drone,r.batch)]for r in q.itertuples()];q['within_batch_support']=q.sustained_recovery_time+.3<=q.batch_cert_end+1e-6
print('QUALIFIED_SEED_WITHIN_SUPPORT',q[q.within_batch_support][['episode','candidate','batch_s','sustained_recovery_time','batch_cert_end','collapsed','first_reject']].to_string(index=False))
q.to_csv(OUT/'qualified_seed_fates.csv',index=False)
# All stronger geometry witnesses final vs actual episode end, production preflight separately.
e=es.set_index('episode');z=o[o.stage=='FINAL_INPUT'].copy();z['gain']=e.loc[z.episode,'loss_end'].to_numpy()-z.sustained_recovery_time
z['geometry_good']=(z.gain>.025)&np.isfinite(z.sustained_K3_recovery_time)&z.common_window_covered
z['cert_recovery']=z.hard_safe&(z.sustained_recovery_time+.3<=z.checked_until+1e-6)
print('GEOM_FINAL',len(z[z.geometry_good]),'PVA',len(z[z.geometry_good&z.pvaj_strict_ok]),'HARD',len(z[z.geometry_good&z.hard_safe]),'CERT',len(z[z.geometry_good&z.cert_recovery]))
print('CERT_SIDE_GOOD',z[z.geometry_good&z.cert_recovery&(z.kind!='NOMINAL')][['episode','candidate','kind','selected','committed','executed','trajectory_id','execution_begin','execution_end','sustained_recovery_time','checked_until','body_static_clearance','body_dynamic_clearance']].to_string(index=False))
z.to_csv(OUT/'final_geometry_fates.csv',index=False)
# overhead & liveness
bs={}
for r in records:bs[(r['drone'],r['batch'])]=min(bs.get((r['drone'],r['batch']),r['world']),r['world'])
for dr in range(3):
 times=sorted(b for d,b in bs if d==dr);d=np.diff(times);print('BATCH_GAP',dr,len(times),'max',max(d),'p99',np.quantile(d,.99))
