"""Bounded fleet reset with actual pose, cancellation and settling feedback.

Preserves world time. Occupied destinations fail before requesting teleports;
this does not pretend a service acknowledgement establishes a physical reset.
"""
import argparse
import json
import math
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def reset(output, timeout=35.):
    node=Node('robot_lab_measured_fleet_reset')
    clocks=[];regressions=[]
    state={};operator='batch-reset-'+uuid.uuid4().hex
    received=[-math.inf]
    def observe(msg):
        state.clear();state.update(json.loads(msg.data));received[0]=time.monotonic()
        stamp=state.get('sim_time')
        if isinstance(stamp,(int,float)) and math.isfinite(stamp) and (not clocks or stamp!=clocks[-1]):
            if clocks and stamp<clocks[-1]:regressions.append(dict(before=clocks[-1],after=stamp))
            clocks.append(stamp)
    subscription=node.create_subscription(String,'/fleet/state',observe,10)
    heartbeat=node.create_publisher(String,'/fleet/operator_heartbeat',10)
    command=node.create_publisher(String,'/fleet/command',10)
    started=time.monotonic();session=None;phase='waiting';stable=None;result=None
    trace=[]
    try:
        while time.monotonic()-started<timeout:
            rclpy.spin_once(node,timeout_sec=.05)
            if not state:continue
            if session is not None and session!=state['session_id']:raise ValueError('Fleet session changed during reset')
            session=state['session_id']
            if regressions:raise ValueError('Actual world clock regressed during batch reset')
            if not 1<=len(state.get('members',[]))<=12:raise ValueError('Actual bounded fleet members required')
            if time.monotonic()-received[0]>1.:raise ValueError('Actual reset feedback lost')
            common=dict(session_id=session,operator_id=operator)
            heartbeat.publish(String(data=json.dumps(common)))
            if state['operator_id']!=operator:continue
            members=state['members']
            trace.append(dict(sim_time=state['sim_time'],phase=phase,members=[dict(id=m['id'],truth=m['truth'],
                mode=m['mode'],reason=m['reason'],nav2_pending=m['nav2_pending'],cancel_pending=m['cancel_pending']) for m in members]))
            if phase=='waiting':
                if not all(m.get('reset_pose') and m['truth'] and m['ready'] for m in members):continue
                for member in members:
                    target=member['reset_pose']
                    for other in members:
                        if other is member:continue
                        if math.dist(target[:2],[other['truth']['x'],other['truth']['y']])<member['radius_m']+other['radius_m']+.15:
                            raise ValueError('Reset destination occupied: '+member['id']+' by '+other['id']+'; move the peer clear first')
                command.publish(String(data=json.dumps(dict(common,op='stop',targets=[m['id'] for m in members]))))
                phase='stopping'
            elif phase=='stopping':
                if any(m['nav2_pending'] or m['cancel_pending'] for m in members):continue
                command.publish(String(data=json.dumps(dict(common,op='reset',targets=[m['id'] for m in members]))))
                phase='resetting'
            elif phase=='resetting':
                valid=True
                for m in members:
                    p,t=m['reset_pose'],m['truth']
                    error=abs(math.atan2(math.sin(t['yaw']-p[2]),math.cos(t['yaw']-p[2]))) if t else math.inf
                    valid=valid and bool(t and m['ages']['truth'] is not None and m['ages']['truth']<.5
                        and m['mode']=='idle' and not m['nav2_pending'] and not m['cancel_pending']
                        and math.dist(p[:2],[t['x'],t['y']])<=.03 and error<=.03
                        and t['linear'] is not None and abs(t['linear'])<=.02
                        and t['angular'] is not None and abs(t['angular'])<=.04
                        and (not state['navigation'] or m['navigation_ready']))
                if valid:
                    if stable is None:stable=state['sim_time']
                    if state['sim_time']-stable>=1. and len(clocks)>=2:
                        result=dict(success=True,reason='Every native member physically reset, localized and settled',
                            session_id=session,sim_time=state['sim_time'],settled_sim_s=state['sim_time']-stable);break
                else:stable=None
        if result is None:raise TimeoutError('Fleet reset did not physically settle within its deadline')
    except (ValueError,KeyError,TypeError,TimeoutError) as exc:
        result=dict(success=False,reason=str(exc),session_id=session)
    finally:
        if session and state.get('operator_id')==operator:
            command.publish(String(data=json.dumps(dict(session_id=session,operator_id=operator,op='stop',targets=[m['id'] for m in state['members']]))))
        node.destroy_node()
    result.update(schema='robot_lab.measured_fleet_reset.v1',world_clock_reset=bool(regressions) if len(clocks)>=2 else None,
                  clock_before=clocks[0] if clocks else None,clock_after=clocks[-1] if clocks else None,
                  clock_samples=len(clocks),clock_regressions=regressions,trace=trace,
                  wall_duration_s=time.monotonic()-started)
    Path(output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True);options=parser.parse_args(args)
    path=Path(options.output).resolve();path.parent.mkdir(parents=True,exist_ok=True)
    if path.parent.stat().st_dev==Path('/').stat().st_dev:
        parser.error('Reset receipt must stay on SSD')
    rclpy.init(args=[])
    try:result=reset(path)
    finally:rclpy.shutdown()
    print(json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
