"""Display installed native MJCF robots in the normal Robot Lab launch.

Native model geometry, joint state and body transforms come from MuJoCo. A
passive display keeps the model's authored starting pose; optional explicit
controllers run native actuators. Experimental locomotion is separate from
measured walking support.
"""
import json
import math
import os
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, Twist
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from robot_lab_utils.native_mjcf_assets import body_frame
from robot_lab_utils.process_lifetime import exit_with_parent


class NativeAssetDisplay(Node):
    def __init__(self):
        super().__init__('native_asset_display')
        for name, value in [('native_mjcf', ''), ('model', ''), ('world_xml', ''),
                            ('gui', True), ('hold_position', True), ('arm_control', 'none'),
                            ('grasp_fixture', False), ('locomotion_policy_config', ''),
                            ('mobile_control_config', ''),
                            ('native_task', 'display'),
                            ('articulation_control', False), ('spawn_x', 0.),
                            ('spawn_y', 0.), ('spawn_z', 0.), ('spawn_yaw', 0.)]:
            self.declare_parameter(name, value)
        # EGL avoids the Jetson GLX passive viewer's shutdown crash.
        os.environ.setdefault('MUJOCO_GL', 'egl')
        import mujoco
        self.mujoco = mujoco
        spec = mujoco.MjSpec.from_file(self.get_parameter('native_mjcf').value)
        original = spec.compile()
        self.robot_extent = original.stat.extent
        self.robot_bodies = original.nbody
        self.robot_joints = original.njnt
        spawn = np.array([self.get_parameter('spawn_'+axis).value for axis in ('x', 'y', 'z')])
        yaw = self.get_parameter('spawn_yaw').value
        yaw_quat = np.array([math.cos(yaw/2), 0., 0., math.sin(yaw/2)])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.],
                             [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
        for body in spec.worldbody.bodies:
            body.pos = rotation @ body.pos + spawn
            result = np.zeros(4)
            mujoco.mju_mulQuat(result, yaw_quat, body.quat)
            body.quat = result
        world_path = self.get_parameter('world_xml').value
        if world_path:
            # Same real mesh converter as the established MuJoCo bridge.
            from robot_lab_mujoco.mujoco_spawner import MuJoCoSpawner, _stage_world_meshes
            world_text = MuJoCoSpawner._absolutize_asset_paths(
                Path(world_path).read_text(), str(Path(world_path).parent))
            world_text = _stage_world_meshes(world_text, logger=self.get_logger())
            world = mujoco.MjSpec.from_string(world_text)
            spec.attach(world, frame=spec.worldbody.add_frame(), prefix='environment_')
        elif not any(int(g.type) == int(mujoco.mjtGeom.mjGEOM_PLANE) for g in spec.worldbody.geoms):
            spec.worldbody.add_geom(name='environment_ground', type=mujoco.mjtGeom.mjGEOM_PLANE,
                size=[50., 50., .1], rgba=[.3, .33, .35, 1.], friction=[1., .005, .0001])
        if self.get_parameter('grasp_fixture').value:
            if self.get_parameter('arm_control').value != 'panda':
                raise ValueError('The grasp fixture requires native Panda control')
            # Measure the source home FK to position a supported object between
            # the pads. Its free joint is used only by the physics engine.
            preview = spec.compile()
            home_data = mujoco.MjData(preview)
            mujoco.mj_resetDataKeyframe(preview, home_data, 0)
            mujoco.mj_forward(preview, home_data)
            fingers = [mujoco.mj_name2id(preview, mujoco.mjtObj.mjOBJ_BODY, name)
                       for name in ('left_finger', 'right_finger')]
            pads = [index for index in range(preview.ngeom)
                    if int(preview.geom_bodyid[index]) in fingers
                    and int(preview.geom_type[index]) == int(mujoco.mjtGeom.mjGEOM_BOX)
                    and np.allclose(preview.geom_size[index], [.0085, .004, .0085])]
            if len(pads) != 2:
                raise ValueError('The Panda source fingertip geometry differs')
            center = np.mean(home_data.geom_xpos[pads], axis=0)
            if center[2] <= .03:
                raise ValueError('The grasp fixture requires a positive support height')
            support_height = center[2]-.015
            pedestal = spec.worldbody.add_body(name='manipulation_pedestal',
                pos=[center[0], center[1], support_height/2])
            pedestal.add_geom(name='manipulation_pedestal_geom', type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[.011, .011, support_height/2], rgba=[.35, .4, .45, 1])
            cube = spec.worldbody.add_body(name='manipulation_object', pos=center)
            cube.add_freejoint(name='manipulation_object_joint')
            cube.add_geom(name='manipulation_object_geom', type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[.015, .015, .015], mass=.05, friction=[1., .005, .0001],
                rgba=[1., .45, .05, 1])
            # MjSpec extends existing keyframes with zeros for a new free
            # joint. Author its initial pose so home/reset retain the support.
            for key in spec.keys:
                key.qpos = np.concatenate((key.qpos, center, [1., 0., 0., 0.]))
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        if self.model.nkey:
            home = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, 'home')
            mujoco.mj_resetDataKeyframe(self.model, self.data, max(home, 0))
            for joint in range(self.robot_joints):
                if int(self.model.jnt_type[joint]) == int(mujoco.mjtJoint.mjJNT_FREE):
                    address = int(self.model.jnt_qposadr[joint])
                    self.data.qpos[address:address+3] = rotation @ self.data.qpos[address:address+3] + spawn
                    result = np.zeros(4)
                    mujoco.mju_mulQuat(result, yaw_quat, self.data.qpos[address+3:address+7])
                    self.data.qpos[address+3:address+7] = result
        mujoco.mj_forward(self.model, self.data)
        self.object_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'manipulation_object')
        self.hold = self.get_parameter('hold_position').value
        self.arm = None
        self.gripper = None
        self.locomotion = None
        self.articulation = None
        self.mobile = None
        self.sensors = None
        controller = self.get_parameter('arm_control').value
        policy_config = self.get_parameter('locomotion_policy_config').value
        mobile_config = self.get_parameter('mobile_control_config').value
        if mobile_config:
            if controller != 'none' or policy_config or self.get_parameter('grasp_fixture').value:
                raise ValueError('Native mobile control requires exclusive actuator ownership')
            from robot_lab_mujoco.native_mobile_control import NativeStretchControl
            self.mobile = NativeStretchControl(self, mobile_config)
            from robot_lab_mujoco.native_articulation_control import NativeArticulationControl
            self.articulation = NativeArticulationControl(self)
            from robot_lab_mujoco.native_mobile_sensors import NativeMobileSensors
            self.sensors = NativeMobileSensors(self, self.get_parameter('native_task').value)
            self._odom_pub = self.sensors.odom_pub
            from robot_lab_utils.reset_notifications import ResetNotifications
            self.reset_notifications = ResetNotifications(self)
            self.create_service(Trigger, '/robot_lab/reset', self.reset)
            self.hold = False
        if self.get_parameter('articulation_control').value:
            if mobile_config:
                raise ValueError('Native mobile control already owns articulation')
            if controller != 'none' or policy_config or self.get_parameter('grasp_fixture').value:
                raise ValueError('Generic articulation requires exclusive actuator ownership')
            from robot_lab_mujoco.native_articulation_control import NativeArticulationControl
            self.articulation = NativeArticulationControl(self)
            self.create_service(Trigger, '/robot_lab/reset', self.reset)
            self.hold = False
        if policy_config:
            if controller != 'none' or self.get_parameter('grasp_fixture').value:
                raise ValueError('Locomotion and native arm/fixture controllers cannot share actuators')
            from robot_lab_mujoco.unitree_policy import NativeUnitreePolicy
            self.locomotion = NativeUnitreePolicy(self.model, self.data, policy_config,
                self.get_parameter('native_mjcf').value)
            # Initial/reset pose only: subsequent movement is motor torque.
            self.data.qpos[self.locomotion.qpos] = self.locomotion.nominal
            mujoco.mj_forward(self.model, self.data)
            self.create_subscription(Twist, '/robot_lab_controller/cmd_vel_unstamped',
                lambda msg: self.locomotion.receive(msg.linear.x, msg.linear.y, msg.angular.z), 1)
            self.create_service(Trigger, '/robot_lab/reset', self.reset)
            self.locomotion_pub = self.create_publisher(String, '/robot_lab/locomotion/state', 10)
            self.hold = False
        if controller == 'panda':
            from robot_lab_mujoco.native_arm_control import NativePandaControl
            self.arm = NativePandaControl(self)
            from robot_lab_mujoco.native_gripper_control import NativePandaGripper
            self.gripper = NativePandaGripper(self)
            self.create_service(Trigger, '/robot_lab/reset', self.reset)
            self.hold = False
        elif controller != 'none':
            raise ValueError('Unknown native arm controller: '+controller)
        self.initial_qpos, self.initial_ctrl = self.data.qpos.copy(), self.data.ctrl.copy()
        self.clock_pub = self.create_publisher(ClockMsg, '/clock', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.object_pub = self.create_publisher(String, '/manipulation/object_state', 10)
        self.description_pub = self.create_publisher(String, '/robot_description',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.description = String(data=Path(self.get_parameter('model').value).read_text())
        self.tf = TransformBroadcaster(self)
        self.last_description = -math.inf
        self.dt = .02
        self.create_timer(self.dt, self.tick, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info('Installed native MJCF: %s; %d robot bodies / %d joints; %s' %
            (self.get_parameter('native_mjcf').value, self.robot_bodies, self.robot_joints,
             'experimental position articulation' if self.articulation else
             'experimental Unitree policy dynamics' if self.locomotion else
             'controlled Panda dynamics' if self.arm else
             ('passive authored-pose display' if self.hold else 'native dynamics')))

    def tick(self):
        mj = self.mujoco
        if self.hold:
            self.data.time += self.dt
            mj.mj_forward(self.model, self.data)
        else:
            steps = max(1, round(self.dt / self.model.opt.timestep))
            if self.articulation:
                for _ in range(steps):
                    if self.mobile:
                        self.mobile.before_step()
                    self.articulation.before_step()
                    mj.mj_step(self.model, self.data)
                    self.articulation.after_step()
                self.articulation.publish()
                if self.mobile:
                    self.mobile.publish()
            elif self.locomotion:
                for _ in range(steps):
                    self.locomotion.before_step()
                    mj.mj_step(self.model, self.data)
                    self.locomotion.after_step()
                self.locomotion_pub.publish(String(data=json.dumps(self.locomotion.state())))
            elif self.arm:
                for _ in range(steps):
                    self.arm.before_step()
                    self.gripper.before_step()
                    mj.mj_step(self.model, self.data)
                    self.arm.after_step()
                    self.gripper.after_step()
                self.arm.publish()
                self.gripper.publish()
            else:
                mj.mj_step(self.model, self.data, nstep=steps)
        if self.object_body >= 0:
            body = self.object_body
            self.object_pub.publish(String(data=json.dumps(dict(time=self.data.time,
                id='manipulation_object', frame='native_world', size=[.03, .03, .03],
                finger_links=['native_body_'+str(int(b)) for b in self.gripper.finger_bodies],
                pedestal=dict(type='box', size=(2*self.model.geom_size[mj.mj_name2id(self.model,
                        mj.mjtObj.mjOBJ_GEOM, 'manipulation_pedestal_geom')]).tolist(),
                    position=self.data.xpos[mj.mj_name2id(self.model,
                        mj.mjtObj.mjOBJ_BODY, 'manipulation_pedestal')].tolist(),
                    orientation=[1., 0., 0., 0.]),
                position=self.data.xpos[body].tolist(), orientation_wxyz=self.data.xquat[body].tolist(),
                spatial_velocity=self.data.cvel[body].tolist()))))
        if not np.all(np.isfinite(self.data.qpos)):
            raise RuntimeError('Native model produced non-finite state')
        clock = ClockMsg()
        clock.clock.sec = int(self.data.time)
        clock.clock.nanosec = int((self.data.time - int(self.data.time)) * 1e9)
        self.clock_pub.publish(clock)
        if self.sensors:
            self.sensors.publish(clock.clock)
        state = JointState()
        state.header.stamp = clock.clock
        for joint in range(self.robot_joints):
            if int(self.model.jnt_type[joint]) not in (int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE)):
                continue
            state.name.append(mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, joint) or ('joint_'+str(joint)))
            state.position.append(float(self.data.qpos[int(self.model.jnt_qposadr[joint])]))
            state.velocity.append(float(self.data.qvel[int(self.model.jnt_dofadr[joint])]))
            state.effort.append(float(self.data.qfrc_actuator[int(self.model.jnt_dofadr[joint])]))
        self.joint_pub.publish(state)
        transforms = []
        for body in range(1, self.robot_bodies):
            if self.sensors and self.sensors.task != 'display' and body == self.sensors.base:
                continue  # The estimator exclusively owns odom -> base TF.
            parent = int(self.model.body_parentid[body])
            inverse = self.data.xquat[parent] * np.array([1., -1., -1., -1.])
            position, quat = np.zeros(3), np.zeros(4)
            mj.mju_rotVecQuat(position, self.data.xpos[body]-self.data.xpos[parent], inverse)
            mj.mju_mulQuat(quat, inverse, self.data.xquat[body])
            transform = TransformStamped()
            transform.header.stamp = clock.clock
            transform.header.frame_id = body_frame(parent)
            transform.child_frame_id = body_frame(body)
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, position)
            transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z = map(float, quat)
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        if self.data.time - self.last_description >= 1.:
            self.description_pub.publish(self.description)
            self.last_description = self.data.time

    def reset(self, _request, response):
        if self.arm:
            self.arm.finish('aborted', -4, 'reset; holding source home position')
            self.gripper.finish('aborted', 'reset; holding source home gap')
        sim_time = self.data.time
        self.mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.initial_qpos
        self.data.ctrl[:] = self.initial_ctrl
        self.data.time = sim_time
        self.mujoco.mj_forward(self.model, self.data)
        if self.arm:
            self.arm.target = self.data.qpos[self.arm.qpos].copy()
            self.gripper.target = self.gripper.commanded = self.gripper.opening()
        if self.locomotion:
            self.locomotion.reset()
        if self.articulation:
            self.articulation.reset()
        if self.mobile:
            self.mobile.reset()
            self.reset_notifications.notify()
        response.success, response.message = True, 'Native robot/object reset; simulation clock preserved'
        return response

    def run_viewer(self):
        import tkinter as tk
        from PIL import Image, ImageTk
        mj = self.mujoco
        self.model.vis.global_.offwidth = max(800, self.model.vis.global_.offwidth)
        self.model.vis.global_.offheight = max(600, self.model.vis.global_.offheight)
        renderer = mj.Renderer(self.model, width=800, height=600)
        camera = mj.MjvCamera()
        camera.lookat[:] = self.data.xpos[1] if self.robot_bodies > 1 else self.model.stat.center
        camera.distance = max(1., min(8., 2*self.robot_extent))
        camera.azimuth, camera.elevation = 125., -25.
        root = tk.Tk()
        root.title('Robot Lab — ' + Path(self.get_parameter('native_mjcf').value).parent.name)
        label = tk.Label(root)
        label.pack()
        controls = tk.Frame(root); controls.pack(fill='x')
        def change(attribute, value, multiply=False):
            old = getattr(camera, attribute)
            setattr(camera, attribute, old*value if multiply else old+value)
        for title, attribute, value, multiply in [('Rotate Left', 'azimuth', -15., False),
                ('Rotate Right', 'azimuth', 15., False), ('Zoom In', 'distance', .8, True),
                ('Zoom Out', 'distance', 1.25, True)]:
            tk.Button(controls, text=title, command=lambda a=attribute,v=value,m=multiply:change(a,v,m)).pack(side='left')
        running = True
        def close():
            nonlocal running
            running = False
            root.destroy()
        root.protocol('WM_DELETE_WINDOW', close)
        try:
            last_render = 0.
            while running and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=.001)
                if time.monotonic()-last_render >= .04:
                    renderer.update_scene(self.data, camera=camera)
                    frame = ImageTk.PhotoImage(Image.fromarray(renderer.render()))
                    label.configure(image=frame); label.image = frame
                    last_render = time.monotonic()
                root.update()
        finally:
            # GUI Stop signals the owned group; ros2 launch also forwards
            # SIGINT. Finish EGL/Tk cleanup once even under that second signal.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            renderer.close()
            if running:
                root.destroy()


def main(args=None):
    exit_with_parent()
    rclpy.init(args=args)
    node = None
    try:
        node = NativeAssetDisplay()
        if node.get_parameter('gui').value:
            node.run_viewer()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            if node.sensors:
                node.sensors.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
