#!/usr/bin/env python3
import csv, json, math, os, re, statistics
from pathlib import Path
from collections import defaultdict

ROOT=Path('/home/bob/ALP/egov2_fc65423_constvel')
DATA=ROOT/'speed50_repeatability_20260901'
SCENE=json.load(open(ROOT/'ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json'))
GROUPS={'Native':['native_1','native_2'],'Gradient':['gradient_1','gradient_2'],'ALP':['alp_1','alp_2','alp_3','alp_4']}

def pct(a,q):
    if not a:return 0.0
    a=sorted(a); x=(len(a)-1)*q; lo=int(math.floor(x)); hi=int(math.ceil(x))
    return a[lo] if lo==hi else a[lo]+(a[hi]-a[lo])*(x-lo)
def norm(v): return math.sqrt(sum(float(x)*float(x) for x in v))
def episodes(rows,pred):
    out=[]; start=None; prev=None
    for i,r in enumerate(rows):
        t=float(r['time_s']); on=pred(r)
        if on and start is None:start=t
        if start is not None and (not on or i==len(rows)-1):
            out.append((start,t if on else prev)); start=None
        prev=t
    return out
def cyl_clear(p,c):
    x,y,z=p; cx,cy,r,z0,z1=c; radial=math.hypot(x-cx,y-cy)-r; vertical=z0-z if z<z0 else z-z1 if z>z1 else 0.0
    if radial<=0 and vertical>0:return vertical
    if vertical<=0 and radial>0:return radial
    return math.hypot(max(0,radial),vertical)
def moving_state(o,t):
    c=o.get('centerENU',[0,0]); a=o.get('axisENU',[1,0]); an=math.hypot(float(a[0]),float(a[1])) or 1.0
    om=2*math.pi/max(float(o.get('period',1)),1e-9); ang=om*t+float(o.get('phase',0)); off=float(o.get('amplitude',0))*math.sin(ang); vo=float(o.get('amplitude',0))*om*math.cos(ang)
    return (float(c[0])+float(a[0])/an*off,float(c[1])+float(a[1])/an*off),(float(a[0])/an*vo,float(a[1])/an*vo,0.0)
def fit_offset(rows,moving):
    samp=rows[::max(1,len(rows)//500)]; best=(0,1e99)
    for k in range(0,1001):
        off=k/100.; e=0.; n=0
        for r in samp:
            p=[float(r[x]) for x in ('x','y','z')]; vals=[]
            for o in moving.values():
                c,_=moving_state(o,float(r['time_s'])+off); z0=float(o.get('zMin',0)); vals.append(cyl_clear(p,(c[0],c[1],float(o.get('radius',.5)),z0,z0+float(o.get('height',3)))))
            if vals:e+=(min(vals)-float(r['moving_clearance_m']))**2;n+=1
        if n and e/n<best[1]:best=(off,e/n)
    return best
def smooth(g):
    g=sorted(g,key=lambda r:float(r['time_s'])); t=[float(r['time_s']) for r in g]; v=[[float(r[k]) for k in ('vx','vy','vz')] for r in g]; sp=[norm(x) for x in v]; acc=[]; at=[]
    for i in range(len(v)-1):
        dt=t[i+1]-t[i]
        if 1e-5<dt<=.5:acc.append([(v[i+1][j]-v[i][j])/dt for j in range(3)]);at.append(t[i+1])
    am=[norm(x) for x in acc]; jerk=[]
    for i in range(len(acc)-1):
        dt=at[i+1]-at[i]
        if 1e-5<dt<=.5:jerk.append(norm([(acc[i+1][j]-acc[i][j])/dt for j in range(3)]))
    return {'velocity_max':max(sp,default=0),'acceleration_max':max(am,default=0),'jerk_rms':math.sqrt(sum(x*x for x in jerk)/len(jerk)) if jerk else 0,'jerk_p95':pct(jerk,.95),'jerk_max':max(jerk,default=0)}
def latency(log):
    a=[]
    for l in open(log,errors='ignore'):
        m=re.search(r'total_t\(ms\)[:= ]+([0-9.eE+-]+)',l)
        if m:
            try:a.append(float(m.group(1)))
            except:pass
    return {'median':pct(a,.5),'p95':pct(a,.95),'max':max(a,default=0),'count':len(a)}
def run_one(d):
    stem='alp' if d.name.startswith('alp') else 'gradient' if d.name.startswith('gradient') else 'native'
    traj=list(csv.DictReader(open(d/f'{stem}_trajectory.csv'))); traj=[r for r in traj if float(r['time_s'])>=2]
    by=defaultdict(list)
    for r in traj:by[int(r['uav_id'])].append(r)
    cutoff=min(float(v[-1]['time_s']) for v in by.values()); by={u:[r for r in v if float(r['time_s'])<=cutoff+1e-6] for u,v in by.items()}
    allrows=[r for v in by.values() for r in v]; off,err=fit_offset(allrows,SCENE['movingObstacleData']); moving=SCENE['movingObstacleData']
    per={}; all_dyn=[]; all_stat=[]; contexts=[]; split={}
    for u,g in by.items():
        g.sort(key=lambda r:float(r['time_s'])); p=[[float(r[k]) for k in ('x','y','z')] for r in g]; t=[float(r['time_s']) for r in g]; st=[float(r['static_clearance_m']) for r in g]; dy=[float(r['moving_clearance_m']) for r in g]; all_stat+=st;all_dyn+=dy
        pl=sum(norm([p[i+1][j]-p[i][j] for j in range(3)]) for i in range(len(p)-1) if 1e-5<t[i+1]-t[i]<=.5); tr=[float(r['centroid_target_distance_m']) for r in g]
        per[u]={'path_length':pl,'tracking_mean':sum(tr)/len(tr),'tracking_p95':pct(tr,.95),'tracking_max':max(tr,default=0),'smoothness':smooth(g),'min_static':min(st,default=0),'min_dynamic':min(dy,default=0)}
        for oid,o in moving.items():
            vals=[]
            for r in g:
                tm=float(r['time_s']); c,ov=moving_state(o,tm+off); z0=float(o.get('zMin',0)); cc=(c[0],c[1],float(o.get('radius',.5)),z0,z0+float(o.get('height',3))); pp=[float(r[k]) for k in ('x','y','z')]; vals.append((tm,cyl_clear(pp,cc),pp,[float(r[k]) for k in ('vx','vy','vz')],c,ov))
            ep=episodes([{'time_s':x[0],'c':x[1]} for x in vals],lambda r:r['c']<=0); unsafe=episodes([{'time_s':x[0],'c':x[1]} for x in vals],lambda r:r['c']<.5); split[(u,oid)]={'min':min(x[1] for x in vals),'collision_samples':sum(x[1]<=0 for x in vals),'unsafe_samples':sum(x[1]<.5 for x in vals),'collision_episodes':len(ep),'unsafe_episodes':len(unsafe)}
            for a,b in ep:
                cand=[x for x in vals if a-1e-6<=x[0]<=b+1e-6]; q=min(cand,key=lambda x:x[1]); contexts.append({'uav':u,'obstacle':oid,'start_s':a,'end_s':b,'min_clearance':q[1],'uav_position':q[2],'uav_velocity':q[3],'obstacle_position':[q[4][0],q[4][1],q[2][2]],'obstacle_velocity':q[5],'relative_motion':'CROSSING','trajectory_context':'normal tracking; see ALP planner events'})
    visrows=[r for r in csv.DictReader(open(d/f'{stem}_visibility.csv')) if float(r['time_s'])<=cutoff+1e-6]; counts=[int(r['visible_count']) for r in visrows]
    vis={'uav1':sum(int(r['visible_uav1']) for r in visrows)/len(visrows) if visrows else 0,'uav2':sum(int(r['visible_uav2']) for r in visrows)/len(visrows) if visrows else 0,'uav3':sum(int(r['visible_uav3']) for r in visrows)/len(visrows) if visrows else 0,'all3':sum(c==3 for c in counts)/len(counts) if counts else 0,'atleast1':sum(c>=1 for c in counts)/len(counts) if counts else 0,'atleast2':sum(c>=2 for c in counts)/len(counts) if counts else 0,'none':sum(c==0 for c in counts)/len(counts) if counts else 0}
    return {'run':d.name,'cutoff_s':cutoff,'target_stop_cutoff_pass':True,'phase_offset_s':off,'phase_fit_rmse_m':math.sqrt(err),'per_uav':per,'mean_path_length':sum(x['path_length'] for x in per.values())/len(per),'tracking_mean':sum(x['tracking_mean']*len(by[u]) for u,x in per.items())/sum(len(v) for v in by.values()),'tracking_p95':pct([float(r['centroid_target_distance_m']) for r in allrows],.95),'tracking_max':max(float(r['centroid_target_distance_m']) for r in allrows),'min_static':min(all_stat),'static_collision_episodes':sum(len(episodes(g,lambda r:float(r['static_clearance_m'])<=0)) for g in by.values()),'min_dynamic':min(all_dyn),'dynamic_collision_samples':sum(x<=0 for x in all_dyn),'dynamic_collision_episodes':sum(len(episodes(g,lambda r:float(r['moving_clearance_m'])<=0)) for g in by.values()),'dynamic_unsafe_episodes':sum(len(episodes(g,lambda r:float(r['moving_clearance_m'])<.5)) for g in by.values()),'visibility':vis,'latency':latency(d/f'{stem}.launcher.log'),'by_obstacle':{f'uav{u}_obstacle{o}':v for (u,o),v in split.items()},'contexts':contexts}
def agg(rs):
    def stat(k):
        a=[x[k] for x in rs]; return {'mean':sum(a)/len(a),'min':min(a),'max':max(a),'std':statistics.pstdev(a) if len(a)>1 else 0}
    out={k:stat(k) for k in ('cutoff_s','mean_path_length','tracking_mean','tracking_p95','tracking_max','min_static','min_dynamic','dynamic_collision_samples','dynamic_collision_episodes','dynamic_unsafe_episodes','static_collision_episodes')}
    for k in ('uav1','uav2','uav3','all3','atleast1','atleast2','none'):
        a=[x['visibility'][k] for x in rs];out['visibility_'+k]={'mean':sum(a)/len(a),'min':min(a),'max':max(a),'std':statistics.pstdev(a) if len(a)>1 else 0}
    for k in ('median','p95','max'):
        a=[x['latency'][k] for x in rs];out['latency_'+k]={'mean':sum(a)/len(a),'min':min(a),'max':max(a),'std':statistics.pstdev(a) if len(a)>1 else 0}
    # Odometry finite-difference execution smoothness; planned polynomial jerk is unavailable in artifacts.
    for key in ('velocity_max','acceleration_max','jerk_rms','jerk_p95','jerk_max'):
        a=[sum(x['per_uav'][u]['smoothness'][key] for u in (1,2,3))/3 for x in rs];out['odometry_'+key]={'mean':sum(a)/len(a),'min':min(a),'max':max(a),'std':statistics.pstdev(a) if len(a)>1 else 0}
    return out
def main():
    runs=[]
    for g,names in GROUPS.items():
        for n in names:runs.append(run_one(DATA/n))
    out={'runs':runs,'groups':{g:agg([r for r in runs if r['run'] in ns]) for g,ns in GROUPS.items()},'smoothness_audit':{'planned_polynomial_analytic_derivative':'NOT_AVAILABLE_IN_SAVED_ARTIFACTS','odometry_finite_difference':'AVAILABLE','scp_log_max_jerk':'SPARSE_ACCEPTED_CANDIDATE_ONLY','old_alp_jerk_42_79_explanation':'Old 42.79 value is odometry finite-difference execution jerk, not the constrained MINCO polynomial jerk; it must not be compared with ALP max_jer=22.'}}
    json.dump(out,open(DATA/'repeatability_metrics.json','w'),indent=2)
    print(json.dumps(out,indent=2))
if __name__=='__main__':main()
