from feedback134_analyze import *
TAGS={'candidate-batch-activation','FROZEN_CANDIDATE','local-geometry-candidate','moving-rehead-reject','moving-candidate-reject','side-minco','candidate-final-status','candidate-preinit-failure','side-budget-skip','K3_SIDE_CANDIDATES','topology-post-check','planner-traj-commit','execution-handoff','SIDE_TOPOLOGY_INTENT','raw-los-truth','conflict-descriptor','side-feasible-initializer','side-feasible-fallback','K3_EVENT_RESULT','K3_LOCAL_EVENT_CREATED','COMMIT_SAME_REVISION','LOS_SOFT_PLANE','side-bilateral','risk-candidate','execution-reserve','side-construction'}
PAT=re.compile(r'\[(?:INFO|WARN|ERROR|DEBUG)\]\s*\[(\d+\.\d+)\]: \[([A-Za-z][A-Za-z0-9_-]*)\]([^\n\x1b]*)')
KV=re.compile(r'\b([A-Za-z_][A-Za-z0-9_]*)=([^\s{}]+)')

def main(rid):
 v,dfs,epoch=loadrun(rid);eps=pd.read_csv(OUT/f'{rid}_episodes.csv')
 records=[];counts=collections.Counter();p=ROOT/'runs'/rid/'roslaunch_stdout.log.gz'
 with gzip.open(p,'rt',errors='replace')as f:
  for ln,line in enumerate(f,1):
   for m in PAT.finditer(line):
    counts[m[2]]+=1
    if m[2]not in TAGS:continue
    d=dict(KV.findall(m[3]));d.update(tag=m[2],world=float(m[1]),time_s=float(m[1])-epoch,line=ln,raw=m[3].strip())
    dr=d.get('drone',d.get('drone_id'))
    d['drone']=int(dr) if dr is not None and dr.isdigit() else None
    records.append(d)
 records.sort(key=lambda x:x['world'])
 # Assign unlabelled minco records only when a unique nearby final-status drone exists.
 final=[r for r in records if r['tag']=='candidate-final-status'and r['drone'] is not None]
 for r in records:
  if r['tag']=='side-minco':
   matches=[f for f in final if f.get('candidate_type')==r.get('candidate_type') and 0<=f['world']-r['world']<.015]
   drones={f['drone']for f in matches}
   if len(drones)==1:r['drone']=next(iter(drones));r['drone_inference']='unique_following_final_status_within_15ms'
 with gzip.open(OUT/f'{rid}_events.jsonl.gz','wt')as f:
  for r in records:f.write(json.dumps(r)+'\n')
 batches=[r for r in records if r['tag']=='candidate-batch-activation'and r['drone']is not None]
 summary=[];funnel=[]
 for _,ep in eps.iterrows():
  dr=int(ep.uav)-1;begin=ep.loss_begin;end=ep.loss_end
  rr=[r for r in records if r['drone']==dr and begin-2<=r['time_s']<=end]
  bb=[b for b in batches if b['drone']==dr and begin-2<=b['time_s']<=end]
  summary.append(dict(episode=ep.episode,uav=int(ep.uav),begin=begin,end=end,batches=len(bb),minco_success=sum(r['tag']=='side-minco'and r.get('success')=='1'for r in rr),minco_deadline=sum(r['tag']=='side-minco'and r.get('status')=='EXECUTION_DEADLINE'for r in rr),construction_deadline_rows=sum(r['tag']=='side-construction'and 'EXECUTION_DEADLINE'in r['raw']for r in rr),budget_skip=sum(r['tag']=='side-budget-skip'for r in rr),handoff_reject=sum(r['tag']=='moving-rehead-reject'for r in rr),preflight_reject=sum(r['tag']=='local-geometry-candidate'and r.get('safe')=='0'for r in rr),preinit_reject=sum(r['tag']=='candidate-preinit-failure'for r in rr),commit_kinds=dict(collections.Counter(r.get('kind')for r in rr if r['tag']=='topology-post-check'))))
  for b in bb:
   bt=float(b.get('plan_start',b['world']));finish=b['world'];post=finish+.025
   linked=[r for r in rr if bt<=r['world']<=post]
   for kind in ['NOMINAL','SIDE_PLUS','SIDE_MINUS']:
    kr=[r for r in linked if r.get('candidate_type',r.get('kind',r.get('side')))==kind]
    funnel.append(dict(run=rid,episode=ep.episode,drone=dr,batch=bt,activation=b.get('batch_activation'),kind=kind,observed_tags='|'.join(r['tag']for r in kr),statuses='|'.join(str(r.get('status',r.get('reason','')))for r in kr),candidate_payload='UNRESOLVED_DUE_TO_MISSING_CANDIDATE_PAYLOAD',association='same_drone_plan_start_to_finalize_plus_25ms_NOT_EXACT_ID'))
 pd.DataFrame(funnel).to_csv(OUT/f'{rid}_historical_batch_funnel.csv',index=False)
 (OUT/f'{rid}_log_summary.json').write_text(json.dumps(dict(counts=dict(counts),episodes=summary,unattributed_side_minco=sum(r['tag']=='side-minco'and r['drone']is None for r in records)),indent=2))
 print(rid,'COUNTS',{k:counts[k]for k in ['side-minco','side-budget-skip','FROZEN_CANDIDATE','K3_SIDE_CANDIDATES','POST_OPT_PAYLOAD_MUTATED','candidate-batch-activation']});print(pd.DataFrame(summary).to_string(index=False))
 # Motion liveness per drone per 10 second interval, never a global aggregate proxy.
 live=[]
 for dr in range(3):
  for a in range(0,80,10):live.append(dict(drone=dr,begin=a,batches=sum(b['drone']==dr and a<=b['time_s']<a+10 for b in batches)))
 pd.DataFrame(live).to_csv(OUT/f'{rid}_liveness.csv',index=False)

if __name__=='__main__':
 for rid in sys.argv[1:]or ['20260927_055748_1104145','20260927_061256_1157439']:main(rid)
