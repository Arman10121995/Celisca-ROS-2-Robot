#!/usr/bin/env python3
"""Differential thrust for a free, buoyant catamaran; no pose/velocity writes."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64, String
from robot_lab_utils.marine_controller import MarineController, angle
from robot_lab_utils.marine_model import SurfaceModel, rotation


class SurfaceController(MarineController):
    def load_model(self, description, density):
        return SurfaceModel(description, density)

    def _drive(self, msg):
        if not self.enabled:
            return
        values=[msg.linear.x,msg.angular.z,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y]
        if not all(map(math.isfinite,values)) or any(abs(v)>1e-9 for v in values[2:]):
            self.stop('Surface craft supports surge/yaw; depth, strafe, roll and pitch are passive')
            return
        self.command=(max(-.4,min(.4,values[0])),0.,0.,max(-.3,min(.3,values[1])))
        self.cmd_at=time.monotonic()

    def _command(self, msg):
        try:
            data=json.loads(msg.data)
            if data.get('session_id')==self.session_id and data.get('op') in ('depth_up','depth_down','set_depth','surface'):
                self.reason='Surface draft is physical buoyancy; depth commands are unavailable'
                return
        except (ValueError,TypeError):
            pass
        super()._command(msg)
        if self.enabled:
            self.target_depth=None
            if isinstance(data,dict) and data.get('op')=='arm':self.reason='Surface thrust armed; surge/yaw neutral until input'

    def _tick(self):
        now=time.monotonic();fresh=self._fresh()
        if self.enabled and (not fresh or now-self.hb_at>.8):
            self.stop('Surface feedback/operator lost; thrust removed')
        if self.enabled and self.input_owner=='mission' and now-self.cmd_at>.5:
            self.stop('Surface mission input lost; thrust removed')
        dt=0. if self.last_sim is None or self.clock_value is None else max(0.,min(.05,self.clock_value-self.last_sim))
        self.last_sim=self.clock_value;scale=1.
        if self.enabled:
            roll,pitch,heading=self.orientation
            if max(abs(roll),abs(pitch))>.5 or abs(self.pose.pose.pose.position.z-self.surface)>.4:
                self.stop('Surface attitude/draft envelope exceeded')
            else:
                vx,_,_,wz=self.command if now-self.cmd_at<.35 else (0.,0.,0.,0.)
                self.target_heading=angle(self.target_heading+wz*dt)
                surge=float((rotation((0.,0.,heading)).T@self.velocity)[0])
                desired_yaw=max(-.3,min(.3,wz+.8*angle(self.target_heading-heading)))
                force=110.*(vx-surge)+18.*vx+22.*vx*abs(vx)
                torque=12.*(desired_yaw-self.rates[2])
                self.forces,scale=self.model.allocate([force,0.,0.,0.,0.,torque])
        for publisher,force in zip(self.pub,self.forces):
            publisher.publish(Float64(data=float(force)))
        if now-self.last_state>.1:
            self.last_state=now;p=self.pose.pose.pose.position if self.pose else None
            self.state_pub.publish(String(data=json.dumps(dict(schema_version=1,session_id=self.session_id,
                robot_id=self.robot_id,enabled=self.enabled,ready=fresh,reason=self.reason,
                control_allowed=self.get_parameter('control_allowed').value,input_owner=self.input_owner,
                operator_id=self.operator,operator_fresh=now-self.hb_at<.8,
                position=dict(x=p.x,y=p.y,z=p.z) if p else None,
                depth=self.surface-p.z if p else None,target_depth=None,
                rpy=list(self.orientation) if self.orientation else None,
                velocity_world=self.velocity.tolist() if self.velocity is not None else None,
                commanded_thruster_forces_N=self.forces.tolist(),force_allocation_scale=scale,
                limits=dict(surge=.4,sway=0.,heave=0.,yaw=.3),body_samples=self.count,sim_time=self.clock_value,
                observer='Actual body finite differences for inner thrust control; independent noisy marine odometry for missions',
                scope='Authored catamaran and native graded buoyancy/hydrodynamics; wave and hardware calibration unassessed'),allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=SurfaceController()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.stop('Surface controller shutdown')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
