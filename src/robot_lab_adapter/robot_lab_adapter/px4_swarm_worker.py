"""One isolated ROS/MAVLink/TF owner for one actual PX4 FCU in a shared world."""
import argparse
import json
import math
import os
import time
from collections import deque
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .px4_ros_controller import PX4Controller
from .px4_sitl_runtime import NativeStateRelay
from .px4_planning import PX4Planning
from .px4_frames import ned_to_enu
from robot_lab_utils.aerial_takeoff import x500_envelope


def main():
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    parser=argparse.ArgumentParser();parser.add_argument('--session',required=True);parser.add_argument('--member',required=True)
    args=parser.parse_args();session=json.loads(Path(args.session).read_text())
    item=next(m for m in session['members'] if m['id']==args.member);prefix=item['namespace']
    remaps=['/key_vel','/px4/odometry','/px4/status','/px4/goal','/px4/takeoff','/px4/land','/px4/hold',
        '/px4/planning/status','/px4/planned_path','/px4/plan_goal','/px4/plan_execute','/px4/plan_cancel','/px4/takeoff_check']
    ros=['--ros-args','-r','__ns:='+prefix]
    for topic in remaps:ros+=['-r',topic+':='+prefix+topic]
    ros+=['-p','port:=udpout:127.0.0.1:'+str(item['port']),'-p','expected_system_id:='+str(item['system_id']),
          '-p','frame_prefix:='+prefix.strip('/')+'/',
          *sum((['-p','origin_'+axis+':='+str(value)] for axis,value in zip('xyz',item['origin'])),[])]
    rclpy.init(args=ros);controller=PX4Controller();relay=NativeStateRelay(dict(os.environ),prefix,prefix+'/px4',prefix.strip('/')+'/')
    upstream=Path(session['px4_root'])/'Tools/simulation/gz'
    envelope=x500_envelope(upstream/'models',model_file=Path(item['model_file']))
    planner=PX4Planning(controller,session['source_world'],envelope)
    executor=SingleThreadedExecutor()
    for node in (controller,relay,planner):executor.add_node(node)
    lease={'at':-float('inf'),'generation':None,'operator':''}
    checks=[]
    stable_positions=deque(maxlen=12)
    alignment={'committed':False}
    measured_position=controller.position
    def position():
        # While still disarmed at the known, settled spawn, the FCU's
        # barometric origin can move during initialization. Keep that explicit
        # startup prior until the first accepted Takeoff, then freeze the
        # transform. No simulator pose is read here or during flight.
        if not alignment['committed'] and controller.takeoff_ready():
            raw=controller.flight.local_position
            local=ned_to_enu(raw.x,raw.y,raw.z)
            controller.origin=[a-b for a,b in zip(item['origin'],local)]
            controller.ground_z=item['origin'][2]
        return measured_position()
    controller.position=position
    def invalidate():
        planner.canceled.set();planner.generation+=1;planner.takeoff_plan=None
        planner.takeoff_state=dict(state='invalidated',error='Aerial group command generation changed')
        controller.clear_path('Aerial group command generation changed')
        if controller.phase in ('priming','arming'):
            if controller.flight.armed():controller.command_mode(4,6);controller.transition('landing')
            else:controller.command_mode(4,3);controller.transition('idle')
    def heartbeat(msg):
        try:
            data=json.loads(msg.data)
            if data['session_id']==session['session_id']:
                if lease['generation'] is not None and data['generation']<lease['generation']:return
                if lease['generation'] is not None and data['generation']!=lease['generation']:invalidate()
                lease['at']=time.monotonic();lease['generation']=data['generation'];lease['operator']=data['operator_id']
        except (ValueError,KeyError,TypeError):pass
    controller.create_subscription(String,'/aerial_swarm/vehicle_lease',heartbeat,10)
    reply=controller.create_publisher(String,prefix+'/command_reply',10)
    def request(msg):
        try:
            data=json.loads(msg.data)
            if (data['session_id']!=session['session_id'] or data['operator_id']!=lease['operator']
                    or lease['generation'] is None or data['generation']<lease['generation'] or time.monotonic()-lease['at']>.8):return
            if data['generation']>lease['generation']:invalidate();lease['generation']=data['generation']
            callbacks={'check':planner.check_takeoff,'takeoff':controller.takeoff,'land':controller.land,'hold':controller.hold}
            if data['op']=='check':
                if checks:raise ValueError('A takeoff check is already pending')
                checks.append(dict(request_id=data['request_id'],generation=data['generation'],
                    started=time.monotonic(),stage='stabilizing',attempts=0))
                return  # Wait for a stable landed estimate, then actual geometry.
            response=callbacks[data['op']](Trigger.Request(),Trigger.Response())
            reply.publish(String(data=json.dumps(dict(request_id=data['request_id'],generation=data['generation'],
                accepted=response.success,message=response.message))))
        except (ValueError,KeyError,TypeError):return
    controller.create_subscription(String,prefix+'/command_request',request,10)
    guard=planner.consume_takeoff
    def consume():
        if time.monotonic()-lease['at']>.8:raise ValueError('A live aerial group operator lease is required before Takeoff')
        try:
            target=guard()
            if not alignment['committed']:
                raw=controller.flight.local_position
                alignment.update(committed=True,reference_world=item['origin'],
                    local_enu=list(ned_to_enu(raw.x,raw.y,raw.z)),offset=list(controller.origin),
                    source='Known landed spawn prior until first accepted Takeoff; frozen FCU odometry thereafter')
                (Path(session['directory'])/(item['id']+'-origin-alignment.json')).write_text(json.dumps(alignment,indent=2)+'\n')
            return target
        except ValueError:
            controller.get_logger().warning('Takeoff seal rejected: '+json.dumps(dict(
                state=planner.takeoff_state,position=controller.position(),generation=lease['generation'])))
            raise
    planner.consume_takeoff=consume
    def watchdog():
        now=time.monotonic()
        # Gate calibration on measured velocity while landed and level. Raw
        # barometric-position drift during EKF startup is precisely the origin
        # error being calibrated, so it is recorded rather than treated as
        # physical body travel. Never use the fixed prior as a motion sample.
        if controller.takeoff_ready():
            raw=controller.flight.local_position
            stable_positions.append((now,list(ned_to_enu(raw.x,raw.y,raw.z)),
                                     math.sqrt(raw.vx**2+raw.vy**2+raw.vz**2)))
        else:stable_positions.clear()
        stable=(len(stable_positions)>=6 and now-stable_positions[0][0]>=1.
            and max(speed for _,_,speed in stable_positions)<=.08)
        for request in list(checks):
            invalid=(request['generation']!=lease['generation'] or now-lease['at']>.8)
            if request['stage']=='stabilizing' and stable and not invalid:
                response=planner.check_takeoff(Trigger.Request(),Trigger.Response())
                if response.success:
                    request['stage']='checking';request['attempts']+=1
                else:
                    request['stage']='failed';request['error']=response.message
            state=planner.takeoff_state
            finished=request['stage']=='checking' and state.get('state') in ('clear','failed','invalidated','expired','canceled')
            if finished and state.get('state') in ('invalidated','expired') and request['attempts']<2 and not invalid:
                request['stage']='stabilizing';stable_positions.clear();continue
            if invalid or finished or request['stage']=='failed' or now-request['started']>22.:
                checks.remove(request)
                accepted=not invalid and request['stage']=='checking' and state.get('state')=='clear' and planner.takeoff_plan is not None
                message=('Actual selected-world takeoff corridor checked; fresh single-use result ready' if accepted
                    else request.get('error') or state.get('error') or 'Landed estimate did not stabilize or the check timed out')
                reply.publish(String(data=json.dumps(dict(request_id=request['request_id'],generation=request['generation'],
                    accepted=accepted,message=message,check_position=state.get('start_enu') if accepted else None))))
        if controller.flight.armed() and (time.monotonic()-lease['at']>1. or controller.phase in ('idle','waiting_for_fcu','telemetry_lost')):
            controller.command_mode(4,6);controller.transition('landing')
    controller.create_timer(.2,watchdog)
    try:
        while rclpy.ok():executor.spin_once(timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        if controller.flight.armed():
            try:controller.command_mode(4,6)
            except Exception:pass
        relay.close();planner.close()
        for node in (planner,relay,controller):node.destroy_node()
        controller.flight.master.close()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
