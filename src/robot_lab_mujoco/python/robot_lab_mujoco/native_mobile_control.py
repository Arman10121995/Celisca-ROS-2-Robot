"""Native Stretch wheel actuation with exclusive base/articulation ownership.

The original model's wheel radius, axle, signs and transmissions determine
kinematics. Original Stretch uses bounded tendon motor feedback; Stretch 3
uses its authored velocity servos. Neither controller writes body poses.
"""
import hashlib
import json
import time
from pathlib import Path

import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class NativeStretchControl:
    def __init__(self, node, configuration):
        self.node, self.model, self.data, self.mj = node, node.model, node.data, node.mujoco
        config = json.loads(Path(configuration).read_text())
        path = Path(node.get_parameter('native_mjcf').value).resolve()
        if (config.get('schema') != 'robot_lab.native_stretch.v1'
                or path != Path(config['native_model']).resolve()
                or hashlib.sha256(path.read_bytes()).hexdigest() != config['native_sha256']):
            raise ValueError('Native mobile controller requires the exact installed Stretch source contract')
        self.variant = config['variant']
        self.limits = config['limits']
        self.joints = np.asarray([self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_JOINT, name)
            for name in ('joint_left_wheel', 'joint_right_wheel')])
        self.base = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_BODY, 'base_link')
        if self.base < 0 or np.any(self.joints < 0):
            raise ValueError('Native Stretch base/wheel joints missing')
        base_rotation = self.data.xmat[self.base].reshape(3, 3)
        locations = (self.data.xpos[self.model.jnt_bodyid[self.joints]]-self.data.xpos[self.base]) @ base_rotation
        self.axle = float(locations[0, 1]-locations[1, 1])
        self.signs = np.sign(self.data.xaxis[self.joints] @ base_rotation[:, 1])
        radii = []
        for joint in self.joints:
            matches = [g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] == self.model.jnt_bodyid[joint]
                and self.model.geom_type[g] == self.mj.mjtGeom.mjGEOM_CYLINDER and self.model.geom_contype[g]]
            if len(matches) != 1:
                raise ValueError('Expected one source collision cylinder per Stretch wheel')
            radii.append(float(self.model.geom_size[matches[0], 0]))
        if self.axle <= 0 or min(radii) <= 0 or np.any(self.signs == 0):
            raise ValueError('Invalid source native mobile geometry')
        self.radii = np.asarray(radii)
        names = ['forward', 'turn'] if self.variant == 'stretch' else ['left_wheel_vel', 'right_wheel_vel']
        self.actuators = np.asarray([self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_ACTUATOR, name) for name in names])
        if np.any(self.actuators < 0):
            raise ValueError('Native mobile actuators missing')
        if self.variant == 'stretch':
            if np.any(self.model.actuator_trntype[self.actuators] != self.mj.mjtTrn.mjTRN_TENDON):
                raise ValueError('Original Stretch requires forward/turn tendon motors')
        elif self.variant != 'stretch_3' or np.any(self.model.actuator_trntype[self.actuators] != self.mj.mjtTrn.mjTRN_JOINT):
            raise ValueError('Unsupported native mobile transmission')
        self.dofs = self.model.jnt_dofadr[self.joints]
        self.received = -float('inf')
        self.command = np.zeros(2)
        self.ramped = np.zeros(2)
        self.fault, self.status = '', 'neutral; waiting for operator velocity'
        self.publisher = node.create_publisher(String, '/robot_lab/mobile/state', 10)
        node.create_subscription(Twist, '/robot_lab_controller/cmd_vel_unstamped', self.receive, 1)

    def moving(self):
        velocity = np.zeros(6)
        self.mj.mj_objectVelocity(self.model, self.data, self.mj.mjtObj.mjOBJ_BODY, self.base, velocity, 1)
        return bool(np.linalg.norm(velocity[3:5]) > .025 or abs(velocity[2]) > .05 or np.any(abs(self.ramped) > .005))

    def manipulator_busy(self):
        control = self.node.articulation
        return bool(control and (control.status in ('jogging', 'experimental Cartesian servo') or
                                 control.sequence is not None or
                                 (control.servo and control.servo.active()) or
                                 np.any(np.abs(self.data.actuator_velocity[control.indices]) > .02)))

    def stowed(self):
        extend = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_TENDON, 'extend')
        return extend >= 0 and self.data.ten_length[extend] <= self.limits['max_drive_extension_m']

    def receive(self, message):
        values = np.array([message.linear.x, message.angular.z])
        if not np.all(np.isfinite(values)):
            self.command[:] = 0
            self.status = 'Rejected non-finite velocity'
            return
        if self.fault or self.manipulator_busy() or (np.any(values) and not self.stowed()):
            self.command[:] = 0
            self.status = 'Stop articulation and retract the arm before driving'
            return
        self.command = np.clip(values, [-self.limits['max_linear_m_s'], -self.limits['max_yaw_rad_s']],
            [self.limits['max_linear_m_s'], self.limits['max_yaw_rad_s']])
        self.received = time.monotonic()
        self.status = 'driving' if np.any(self.command) else 'zero velocity'

    def before_step(self):
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            self.fault = 'Non-finite native state; reset required'
        if self.fault:
            self.data.ctrl[self.actuators] = 0
            return
        target = self.command if time.monotonic()-self.received <= .5 and not self.manipulator_busy() and self.stowed() else np.zeros(2)
        if not np.any(target) and np.any(self.command):
            self.status = 'watchdog/ownership stop'
        rates = np.array([self.limits['linear_acceleration_m_s2'], self.limits['yaw_acceleration_rad_s2']])
        self.ramped += np.clip(target-self.ramped, -rates*self.model.opt.timestep, rates*self.model.opt.timestep)
        vx, yaw = self.ramped
        wheel_rates = np.array([vx-yaw*self.axle/2, vx+yaw*self.axle/2])/self.radii*self.signs
        if self.variant == 'stretch_3':
            gear = self.model.actuator_gear[self.actuators, 0]
            controls = wheel_rates*gear
        else:
            # Authored tendon moments map forward/turn motor controls into
            # the two wheel generalized forces, including gear and signs.
            desired_torque = self.limits['wheel_velocity_gain']*(wheel_rates-self.data.qvel[self.dofs])
            if self.data.actuator_moment.ndim == 2:
                transmission = self.data.actuator_moment[self.actuators][:, self.dofs].T
            else:
                transmission = np.zeros((2, 2))
                for column, actuator in enumerate(self.actuators):
                    begin = int(self.data.moment_rowadr[actuator])
                    end = begin+int(self.data.moment_rownnz[actuator])
                    entries = dict(zip(self.data.moment_colind[begin:end], self.data.actuator_moment[begin:end]))
                    transmission[:, column] = [entries.get(int(dof), 0.) for dof in self.dofs]
            try:
                controls = np.linalg.solve(transmission, desired_torque)
            except np.linalg.LinAlgError:
                self.fault = 'Singular source wheel transmission; reset required'
                self.data.ctrl[self.actuators] = 0
                return
        if not np.all(np.isfinite(controls)):
            self.fault = 'Non-finite wheel effort; reset required'
            self.data.ctrl[self.actuators] = 0
            return
        if self.variant == 'stretch_3':
            caps = np.min(np.abs(self.model.actuator_ctrlrange[self.actuators]), axis=1)
            controls /= max(1., float(np.max(np.abs(controls)/caps)))
        self.data.ctrl[self.actuators] = np.clip(controls, *self.model.actuator_ctrlrange[self.actuators].T)

    def reset(self):
        self.command[:], self.ramped[:] = 0, 0
        self.received = -float('inf')
        self.fault, self.status = '', 'reset; neutral'

    def publish(self):
        self.publisher.publish(String(data=json.dumps(dict(variant=self.variant, status=self.status,
            command=self.ramped.tolist(), measured_wheel_rates=self.data.qvel[self.dofs].tolist(),
            arm_stowed=bool(self.stowed()), articulation_busy=self.manipulator_busy(), fault=self.fault,
            axle_m=self.axle, wheel_radii_m=self.radii.tolist(), limits=self.limits,
            qualification='implemented native mobile manipulation; runtime validation pending'))))
