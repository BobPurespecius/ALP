#!/usr/bin/env python3
"""Exercise the production coordinator's one-shot pending identities over ROS."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rospy
from traj_utils.msg import (PolyTraj, TeamTrajectorySolution, TopologyCandidate,
                            TopologyCandidateBundle, TopologyCoordination)

root = Path(__file__).resolve().parents[3]
rospy.init_node('team_bundle_lifecycle_contract', anonymous=True)
received = []
solutions = []
subscriber = rospy.Subscriber('/topology_coordination/result', TopologyCoordination,
                              lambda m: received.append(m), queue_size=20)
solution_subscriber = rospy.Subscriber(
    '/topology_coordination/team_solution', TeamTrajectorySolution,
    lambda m: solutions.append(m), queue_size=20)
publishers = [rospy.Publisher('/topology_coordination/uav%d/candidate_bundle' % i,
                             TopologyCandidateBundle, queue_size=4) for i in range(3)]
execution_publishers = [rospy.Publisher(
    '/drone_%d_planning/trajectory' % i, PolyTraj, queue_size=4)
    for i in range(3)]
node = subprocess.Popen([os.environ.get('ALP_TEST_COORDINATOR_EXECUTABLE', str(root/'devel/lib/multi_uav_formation/multi_uav_topology_coordinator')),
                         '__name:=bundle_lifecycle_coordinator'],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True)
def wait_for(test, timeout=3.0):
    end=time.monotonic()+timeout
    while not test() and time.monotonic()<end:
        time.sleep(.01)
    assert test(), 'production coordinator did not satisfy expected callback contract'

def bundle(i, generation, now):
    b=TopologyCandidateBundle(drone_id=i, planning_generation=generation,
        transaction_id=generation,
        planning_epoch=now,bundle_stamp=now,evaluation_start_time=now,
        requested_evaluation_horizon=1.5,local_selected_candidate_id=i+1,
        expected_execution_generation=generation,
        expected_trajectory_id=generation,
        boundary_source_revision=generation,
        boundary_source_trajectory_id=generation,
        committed_coverage_start=now, committed_safety_validated=True,
        earliest_activation=now+rospy.Duration(.2),
        validated_end=now+rospy.Duration(3.0))
    c=TopologyCandidate(candidate_id=i+1, success=True,static_valid=True,
        dynamics_valid=True,swarm_valid=True,safety_class=TopologyCandidate.SAFETY_ABSOLUTE_SAFE,
        trajectory_start_time=now,duration=3.0,evaluation_horizon=3.0,
        binary_visibility_valid=True,visibility_sample_offsets=[0.0,3.0],self_visibility_samples=[1,1])
    p=c.trajectory;p.drone_id=i;p.order=5;p.start_time=now;p.duration=[1.5,1.5]
    pos=[2*math.cos(i*2*math.pi/3),2*math.sin(i*2*math.pi/3),1.5]
    p.start_p=pos;p.end_p=pos;p.start_v=p.end_v=p.start_a=p.end_a=[0.,0.,0.]
    p.inner_x=[pos[0]];p.inner_y=[pos[1]];p.inner_z=[pos[2]]
    b.committed_trajectory=p
    b.candidates=[c];return b

def execution(i, generation, now):
    pos=[2*math.cos(i*2*math.pi/3),2*math.sin(i*2*math.pi/3),1.5]
    return PolyTraj(
        drone_id=i, traj_id=generation, start_time=now, order=5,
        coef_x=[0.,0.,0.,0.,0.,pos[0]],
        coef_y=[0.,0.,0.,0.,0.,pos[1]],
        coef_z=[0.,0.,0.,0.,0.,pos[2]], duration=[3.0],
        trajectory_source='NOMINAL', generation=generation,
        safety_validated=True, lifecycle_state='CURRENTLY_VALID')
try:
    wait_for(lambda: all(p.get_num_connections()>0 for p in publishers) and
             all(p.get_num_connections()>0 for p in execution_publishers))
    now=rospy.Time.now()
    for i,p in enumerate(execution_publishers):p.publish(execution(i,1,now))
    time.sleep(.05)
    for i,p in enumerate(publishers):p.publish(bundle(i,1,now))
    time.sleep(.15)
    assert not received and not solutions, \
        'non-constructible Joint input gained fallback execution authority'
    # Consumed generation-1 identities must not be recycled with one fresh
    # bundle. Joint failure remains a Local no-op and publishes no fallback.
    now=rospy.Time.now();publishers[0].publish(bundle(0,2,now))
    time.sleep(.08)
    assert not received and not solutions
    # A delayed duplicate cannot overwrite the fresh unconsumed identity.
    publishers[0].publish(bundle(0,1,now))
    for i,p in enumerate(execution_publishers):p.publish(execution(i,2,now))
    for i in (1,2):publishers[i].publish(bundle(i,2,now))
    time.sleep(.15)
    assert node.poll() is None and not received and not solutions
    print('TEAM_BUNDLE_LIFECYCLE_ROS_PASS committed_prefix=1 '
          'joint_failure_local_noop=1 delayed_duplicate_rejected=1')
finally:
    node.send_signal(signal.SIGINT)
    try:node.wait(timeout=5)
    except subprocess.TimeoutExpired:node.kill();node.wait()
    coordinator_output = node.stdout.read()
    if coordinator_output:
        print(coordinator_output, end='')
