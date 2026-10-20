#!/usr/bin/env python3
"""Native Tethys propeller/fins; explicit experimental cruise-only guidance.

Reuse the existing marine actual-pose observer and operator protocol. This
underactuated source AUV cannot strafe, pivot or hold depth at zero flow.
Only native bounded propulsion/fin targets are commanded; no body pose writes.
"""
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace
import uuid
import numpy as np
import rclpy
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import FluidPressure,JointState
from std_msgs.msg import Float64,String
from tf2_ros import TransformBroadcaster
from robot_lab_utils.marine_controller import MarineController,angle
from robot_lab_utils.marine_model import rotation


class AuvController(MarineController):
    def __init__(self):
        Node.__init__(self,'tethys_auv_controller')
        for key,value in dict(calibration_file='',surface_z=0.,maximum_depth=5.4,control_allowed=True).items():self.declare_parameter(key,value)
        self.calibration=json.loads(Path(self.get_parameter('calibration_file').value).read_text())
        self.model=SimpleNamespace(density=self.calibration['fluid_density_kg_m3'])
        self.surface=float(self.get_parameter('surface_z').value);self.maximum_depth=float(self.get_parameter('maximum_depth').value)
        self.robot_id='tethys_auv';self.session_id=uuid.uuid4().hex
        self.enabled=False;self.manual_seen=False;self.reason='AUV disarmed; passive water forces remain physical'
        self.input_owner='manual'
        self.operator='';self.hb_at=self.pose_at=self.cmd_at=self.clock_at=self.joints_at=-math.inf
        self.clock_value=self.pose=self.velocity=self.rates=self.orientation=None
        self.target_depth=self.target_heading=None;self.command=(0.,0.,0.,0.)
        self.forces=np.zeros(1);self.fin_targets=np.zeros(2);self.joints={}
        self.last_sim=None;self.last_state=0.;self.count=0;self.joint_count=0
        self.pub=[self.create_publisher(Float64,'/marine/auv/propeller_force',10)]
        self.fin_pub=[self.create_publisher(Float64,'/marine/auv/'+axis+'_fin',10) for axis in ('horizontal','vertical')]
        self.state_pub=self.create_publisher(String,'/marine/state',10)
        self.pressure_pub=self.create_publisher(FluidPressure,'/marine/pressure_ideal',10);self.tf=TransformBroadcaster(self)
        self.create_subscription(Odometry,'/marine/odometry',self._pose,10)
        self.create_subscription(JointState,'/joint_states',self.observe_joints,10)
        self.create_subscription(Twist,'/marine/key_vel',self._manual_input,10)
        self.create_subscription(Twist,'/marine/mission_vel',self._mission_input,10)
        self.create_subscription(String,'/marine/command',self._command,10)
        self.create_subscription(String,'/marine/operator_heartbeat',self._heartbeat,10)
        self.create_subscription(ClockMessage,'/clock',self._observe_clock,qos_profile_sensor_data)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME);self.create_timer(.02,self._tick,clock=self.steady)

    def observe_joints(self,msg):
        joints={name.split('::')[-1]:float(value) for name,value in zip(msg.name,msg.position)}
        needed={'propeller_joint','horizontal_fins_joint','vertical_fins_joint'}
        if not needed<=joints.keys() or not all(math.isfinite(joints[k]) for k in needed):return
        self.joints={k:joints[k] for k in needed};self.joints_at=time.monotonic();self.joint_count+=1

    def _fresh(self):return MarineController._fresh(self) and time.monotonic()-self.joints_at<.6

    def _drive(self,msg):
        if not self.enabled:return
        values=[msg.linear.x,msg.linear.z,msg.angular.z,msg.linear.y,msg.angular.x,msg.angular.y]
        if not all(math.isfinite(v) for v in values) or sum(abs(v) for v in values[3:])>1e-9:
            self.stop('Unsupported AUV strafe/roll/pitch command');return
        self.command=(max(-.7,min(.7,values[0])),0.,max(-.2,min(.2,values[1])),max(-.15,min(.15,values[2])))
        self.cmd_at=time.monotonic()
        if any(abs(v)>1e-6 for v in values[:3]):self.manual_seen=True

    def _command(self,msg):
        MarineController._command(self,msg)
        try:op=json.loads(msg.data).get('op')
        except (ValueError,TypeError):return
        if op=='arm' and self.enabled:
            self.manual_seen=False;self.reason='AUV armed neutral; depth/heading guidance requires forward flow'
        if self.target_depth is not None:self.target_depth=max(.6,min(self.maximum_depth,self.target_depth))

    def stop(self,reason):
        self.input_owner='manual'
        self.enabled=False;self.command=(0.,0.,0.,0.);self.cmd_at=-math.inf
        self.target_depth=self.target_heading=None;self.reason=reason;self.forces=np.zeros(1)
        self.pub[0].publish(Float64(data=0.))
        self.fin_targets[:]=0.;self.manual_seen=False
        for pub in self.fin_pub:pub.publish(Float64(data=0.))

    def _tick(self):
        now=time.monotonic();fresh=self._fresh()
        if self.enabled and (not fresh or now-self.hb_at>.8):self.stop('AUV feedback/operator loss; propulsion zero, fins neutral')
        if self.enabled and self.input_owner=='mission' and now-self.cmd_at>.5:self.stop('AUV mission input lost; propulsion zero, fins neutral')
        dt=0. if self.last_sim is None or self.clock_value is None else max(0.,min(.05,self.clock_value-self.last_sim))
        self.last_sim=self.clock_value;surge=0.;guidance=False
        if self.orientation is not None and self.velocity is not None:
            forward=rotation((0.,0.,self.orientation[2]))@np.array([-1.,0.,0.])
            surge=float(forward@self.velocity)
        if self.enabled:
            roll,pitch,heading=self.orientation;depth=self.surface-self.pose.pose.pose.position.z
            if max(abs(roll),abs(pitch))>.5 or not .2<depth<self.maximum_depth+.2:
                self.stop('AUV attitude/depth envelope exceeded')
            else:
                vx,_vy,vz,wz=self.command if now-self.cmd_at<.35 else (0.,0.,0.,0.)
                self.target_depth=max(.6,min(self.maximum_depth,self.target_depth-vz*dt))
                self.target_heading=angle(self.target_heading+wz*dt)
                # Source front is -X. Native thrust is a signed force on +X.
                forward=rotation((0.,0.,heading))@np.array([-1.,0.,0.]);surge=float(forward@self.velocity)
                self.forces[0]=max(-35.,min(35.,-(80.*(vx-surge)+6.2282*vx*abs(vx)))) if self.manual_seen else 0.
                desired=np.zeros(2);guidance=surge>.15 and vx>.05
                if guidance:
                    desired_down=max(-.1,min(.1,.6*(self.target_depth-depth)+.7*self.velocity[2]))
                    target_pitch=max(-.2,min(.2,-math.atan2(desired_down,max(.2,surge))))
                    desired[0]=-1.8*(target_pitch-pitch)+3.2*self.rates[1]
                    # Native source rudder has negative yaw effectiveness
                    # with the actual -X forward flow; retain its joint axis.
                    desired[1]=-3.*(wz-self.rates[2])-.8*angle(self.target_heading-heading)
                    desired=np.clip(desired,-.261799,.261799)
                self.fin_targets+=np.clip(desired-self.fin_targets,-.2*dt,.2*dt)
        self.pub[0].publish(Float64(data=float(self.forces[0])))
        for pub,target in zip(self.fin_pub,self.fin_targets):pub.publish(Float64(data=float(target)))
        if now-self.last_state>.1:
            self.last_state=now;p=self.pose.pose.pose.position if self.pose else None
            state=dict(schema_version=1,session_id=self.session_id,robot_id=self.robot_id,enabled=self.enabled,ready=fresh,
                control_allowed=self.get_parameter('control_allowed').value,reason=self.reason,
                input_owner=self.input_owner,
                operator_id=self.operator,operator_fresh=now-self.hb_at<.8,
                depth=self.surface-p.z if p else None,target_depth=self.target_depth,
                position=dict(x=p.x,y=p.y,z=p.z) if p else None,rpy=list(self.orientation) if self.orientation else None,
                velocity_world=self.velocity.tolist() if self.velocity is not None else None,
                commanded_thruster_forces_N=self.forces.tolist(),fin_targets_rad=self.fin_targets.tolist(),actual_joints_rad=self.joints,
                measured_forward_surge_m_s=surge,cruise_guidance_active=guidance,body_samples=self.count,joint_samples=self.joint_count,
                sim_time=self.clock_value,limits=dict(surge=.7,sway=0.,yaw=.15),
                propulsion_kind='propeller_fins',calibration=self.calibration,
                observer='Actual simulator pose finite difference and joint observations; pressure is explicitly ideal pose-derived emulation. Independent estimation/SLAM unimplemented.',
                scope='Pinned native Tethys AUV + declared trim ballast and authored bounded cruise guidance; no hover/strafe/pivot/depth hold at zero flow or mission qualification')
            self.state_pub.publish(String(data=json.dumps(state,allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=AuvController()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RuntimeError:
        # Humble can raise a pybind conversion error while SIGINT destroys
        # a subscription. Preserve genuine runtime failures in a live context.
        if rclpy.ok():raise
    finally:
        if rclpy.ok():node.stop('AUV shutdown; propulsion zero, fins neutral')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
