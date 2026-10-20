"""One isolated ROS/MAVLink/TF owner for one actual PX4 FCU in a shared world."""
import argparse
import json
import os
import time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .px4_ros_controller import PX4Controller
from .px4_sitl_runtime import NativeStateRelay
from .px4_planning import PX4Planning
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
            response=callbacks[data['op']](Trigger.Request(),Trigger.Response())
            reply.publish(String(data=json.dumps(dict(request_id=data['request_id'],generation=data['generation'],
                accepted=response.success,message=response.message))))
        except (ValueError,KeyError,TypeError):return
    controller.create_subscription(String,prefix+'/command_request',request,10)
    guard=planner.consume_takeoff
    def consume():
        if time.monotonic()-lease['at']>.8:raise ValueError('A live aerial group operator lease is required before Takeoff')
        return guard()
    planner.consume_takeoff=consume
    def watchdog():
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
