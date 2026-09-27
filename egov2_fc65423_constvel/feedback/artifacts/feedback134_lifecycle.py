from feedback134_candidate_oracle import *
f=pd.read_csv(OUT/f'{RID}_candidate_fates.csv').drop_duplicates(['drone','candidate']);records=sum([read_jsonl(p)for p in DIAG.glob('candidate_drone_*.jsonl')],[]);by=collections.defaultdict(list)
for r in records:by[(r['drone'],r.get('candidate'))].append(r)
rows=[];solverms=[];finalms=[]
for a in f.to_dict('records'):
 rs=by[a['drone'],a['candidate']];d={r['stage']:r for r in rs};ct=d.get('FINAL_INPUT',d.get('FINAL_INPUT_FAILED',rs[0]));a.update(astar=ct.get('astar'),sfc=ct.get('sfc'),blocker=ct.get('los_blocker'),geometry_mask=ct.get('mask'),activation_world=ct.get('activation'),optimized_hash=d.get('SOLVER_OK',{}).get('poly_hash'),final_hash=d.get('FINAL_INPUT',{}).get('poly_hash'),validated_hash=d.get('COMPARATOR_WIN',d.get('COMPARATOR_LOSS',{})).get('hash'),revision=d.get('COMPARATOR_WIN',d.get('COMPARATOR_LOSS',{})).get('revision'))
 a['solver_failed_observed']='SOLVER_FAILED'in d;a['fallback_used']=ct.get('fallback',0)
 if not isinstance(a['first_reject'],str) and 'SOLVER_FAILED'in d and 'FINAL_INPUT'not in d:a['first_reject']='SOLVER_FAILED_NO_FINAL_INPUT';a['reject_reason']=d['SOLVER_FAILED'].get('reason','')or'OPTIMIZER_FALSE_WITHOUT_REASON'
 for stage in sorted(d):a['time_'+stage]=d[stage]['world']
 rows.append(a)
 if 'SEED_ELIGIBILITY'in d and ('SOLVER_OK'in d or'SOLVER_FAILED'in d):solverms.append(1000*((d.get('SOLVER_OK',d.get('SOLVER_FAILED')))['world']-d['SEED_ELIGIBILITY']['world']))
 if 'FINAL_INPUT'in d and ('COMPARATOR_WIN'in d or'COMPARATOR_LOSS'in d):finalms.append(1000*(d.get('COMPARATOR_LOSS',d.get('COMPARATOR_WIN'))['world']-d['FINAL_INPUT']['world']))
z=pd.DataFrame(rows);z.to_csv(OUT/'candidate_lifecycle.csv',index=False)
print('Lifecycle',len(z),'astar final marked',int(z.astar.sum()),'SFC marked',int(z.sfc.sum()),'fallback',int(z.fallback_used.sum()))
for label,ms in [('SIDE_SOLVE',solverms),('FINAL_INPUT_TO_COMPARATOR',finalms)]:print(label,len(ms),'median,p95,max',np.quantile(ms,[.5,.95,1]).tolist())
print(z.groupby('first_reject',dropna=False).size().to_string())
