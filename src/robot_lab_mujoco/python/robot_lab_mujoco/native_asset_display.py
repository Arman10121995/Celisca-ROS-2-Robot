"""Display installed native MJCF robots in the normal Robot Lab launch.

Native model geometry, joint state and body transforms come from MuJoCo. A
passive display keeps the model's authored starting pose; hold=false runs its
unmodified dynamics. This node does not claim walking or manipulation control.
"""
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
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from robot_lab_utils.native_mjcf_assets import body_frame
from robot_lab_utils.process_lifetime import exit_with_parent


class NativeAssetDisplay(Node):
    def __init__(self):
        super().__init__('native_asset_display')
        for name, value in [('native_mjcf', ''), ('model', ''), ('world_xml', ''),
                            ('gui', True), ('hold_position', True), ('arm_control', 'none'), ('spawn_x', 0.),
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
        self.hold = self.get_parameter('hold_position').value
        self.arm = None
        controller = self.get_parameter('arm_control').value
        if controller == 'panda':
            from robot_lab_mujoco.native_arm_control import NativePandaControl
            self.arm = NativePandaControl(self)
            self.hold = False
        elif controller != 'none':
            raise ValueError('Unknown native arm controller: '+controller)
        self.clock_pub = self.create_publisher(ClockMsg, '/clock', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.description_pub = self.create_publisher(String, '/robot_description',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.description = String(data=Path(self.get_parameter('model').value).read_text())
        self.tf = TransformBroadcaster(self)
        self.last_description = -math.inf
        self.dt = .02
        self.create_timer(self.dt, self.tick, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info('Installed native MJCF: %s; %d robot bodies / %d joints; %s' %
            (self.get_parameter('native_mjcf').value, self.robot_bodies, self.robot_joints,
             'controlled Panda dynamics' if self.arm else
             ('passive authored-pose display' if self.hold else 'native dynamics')))

    def tick(self):
        mj = self.mujoco
        if self.hold:
            self.data.time += self.dt
            mj.mj_forward(self.model, self.data)
        else:
            steps = max(1, round(self.dt / self.model.opt.timestep))
            if self.arm:
                for _ in range(steps):
                    self.arm.before_step()
                    mj.mj_step(self.model, self.data)
                    self.arm.after_step()
                self.arm.publish()
            else:
                mj.mj_step(self.model, self.data, nstep=steps)
        if not np.all(np.isfinite(self.data.qpos)):
            raise RuntimeError('Native model produced non-finite state')
        clock = ClockMsg()
        clock.clock.sec = int(self.data.time)
        clock.clock.nanosec = int((self.data.time - int(self.data.time)) * 1e9)
        self.clock_pub.publish(clock)
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
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
