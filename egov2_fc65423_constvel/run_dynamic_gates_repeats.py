#!/usr/bin/env python3
import csv, os, signal, subprocess, time, json
from pathlib import Path

ROOT=Path('/home/bob/ALP/egov2_fc65423_constvel')
SCENE=ROOT/'ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates.json'
OUTROOT=ROOT/'dynamic_gates_repeatability_20260902'
RUNS=[('native',2,False,False,False,11401),('gradient',2,True,False,False,11402),('alp',4,True,True,True,11403)]

def stop_seen(path):
    if not path.exists(): return False
    try:
        rows=list(csv.DictReader(path.open()))
    except Exception: return False
    by={}
    for r in rows:
        try: by[int(r['uav_id'])]=r
        except Exception: pass
    if len(by)!=3:return False
    return all(abs(float(r['target_x'])-36.0)<.03 and abs(float(r['target_y'])-.25)<.03 and
               abs(float(r['target_vx']))<.01 and abs(float(r['target_vy']))<.01 for r in by.values())

def run_one(name,idx,moving,risk,hard,port):
    out=OUTROOT/f'{name}_{idx}'; out.mkdir(parents=True,exist_ok=True)
    log=out/f'{name}.launcher.log'; vis=out/f'{name}_visibility.csv'; traj=out/f'{name}_trajectory.csv'
    env=os.environ.copy(); env.update({'NATIVE_EGOV2_ROS_MASTER_URI':f'http://127.0.0.1:{port}','NATIVE_EGOV2_EXPERIMENT_DIR':str(ROOT),'NATIVE_EGOV2_LOG_FILE':str(log),'NATIVE_EGOV2_VISIBILITY_CSV':str(vis),'NATIVE_EGOV2_TRAJECTORY_CSV':str(traj),'NATIVE_EGOV2_MOVING_COST':'true' if moving else 'false','NATIVE_EGOV2_RISK_CANDIDATES':'true' if risk else 'false','NATIVE_EGOV2_HARD_CORRIDOR_SCP':'true' if hard else 'false','NATIVE_EGOV2_TARGET_FACING_YAW':'true'})
    args=['scene_file:='+str(SCENE),'rviz:=false','visibility_csv:='+str(vis),'trajectory_csv:='+str(traj),'enable_target_facing_yaw:=true','max_jer:=22','early_avoidance_enabled:=false','moving_obstacle_time_aware_cost_enabled:='+('true' if moving else 'false'),'enable_risk_triggered_candidates:='+('true' if risk else 'false'),'enable_candidate_hard_corridor_scp:='+('true' if hard else 'false'),'enable_warm_start_counterfactual:=false']
    cmd='source /opt/ros/noetic/setup.bash; source /home/bob/ALP/guidance/ros_ws/devel/setup.bash; source '+str(ROOT/'ros_ws/devel/setup.bash')+'; exec '+str(ROOT/'run_constvel_gradient_rviz.sh')+' '+' '.join(args)
    print(f'START {name}_{idx}',flush=True)
    p=subprocess.Popen(['bash','-lc',cmd],cwd=ROOT,env=env,start_new_session=True)
    stable=0; started=time.time(); stop_wall=None
    while True:
        time.sleep(.5)
        if stop_seen(traj):
            stable+=1
            if stable>=6:
                stop_wall=time.time(); print(f'TARGET_STOP {name}_{idx} wall={stop_wall-started:.1f}s',flush=True); os.killpg(p.pid,signal.SIGINT); break
        else: stable=0
        if p.poll() is not None: break
        if time.time()-started>240:
            print(f'TIMEOUT {name}_{idx}',flush=True); os.killpg(p.pid,signal.SIGINT); break
    try: p.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=8)
    json.dump({'name':name,'run':idx,'target_stop_detected':stop_wall is not None,'wall_runtime_s':(stop_wall-started if stop_wall else time.time()-started),'exit_code':p.returncode,'scene':str(SCENE)},(out/'run_meta.json').open('w'),indent=2)
    time.sleep(2)

if __name__=='__main__':
    OUTROOT.mkdir(exist_ok=True)
    for name,count,moving,risk,hard,port in RUNS:
        for idx in range(1,count+1): run_one(name,idx,moving,risk,hard,port)
    print('ALL_RUNS_DONE',flush=True)
