"""Measured Panda fixture geometry and acknowledged MoveIt attachment diffs.

Attachment changes collision planning only. The simulated object remains free
and its motion remains determined by original contact physics.
"""
import copy
import json
import math
import time

import numpy as np
from geometry_msgs.msg import Point, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


def rotation(q):
    w, x, y, z = np.asarray(q, dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion_product(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw]


class MeasuredObjects:
    def __init__(self, node, collision_object, enabled):
        self.node, self.convert, self.enabled = node, collision_object, enabled
        self.object, self.object_received = None, -math.inf
        self.gripper, self.gripper_received = {}, -math.inf
        self.joints, self.joints_received = {}, -math.inf
        self.arm = {}
        self.future, self.pending = None, None
        self.applied, self.attached, self.revision = None, False, 0
        self.command, self.error = None, ''
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.stop = node.create_client(Trigger, '/arm/stop')
        node.create_subscription(String, '/manipulation/object_state', self.receive_object, 10)
        node.create_subscription(String, '/gripper/status', self.receive_gripper, 10)
        node.create_subscription(String, '/arm/status', self.receive_arm, 10)
        node.create_subscription(JointState, '/joint_states', self.receive_joints, 10)
        node.create_subscription(String, '/arm/planning_scene_command', self.receive_command, 10)

    def receive_object(self, message):
        try:
            value = json.loads(message.data)
            if value.get('frame') != 'native_world' or value.get('id') != 'manipulation_object':
                return
            values = [*value['position'], *value['orientation_wxyz'], *value['spatial_velocity'], *value['size']]
            if not all(math.isfinite(float(v)) for v in values) or np.linalg.norm(value['orientation_wxyz']) < .9:
                raise ValueError('Invalid measured object geometry/state')
            if (len(value['position']) != 3 or len(value['size']) != 3
                    or len(value['orientation_wxyz']) != 4 or len(value['spatial_velocity']) != 6
                    or min(value['size']) <= 0):
                raise ValueError('Invalid measured object dimensions')
            self.object, self.object_received = value, time.monotonic()
        except (KeyError, TypeError, ValueError) as exc:
            self.error = str(exc)

    def receive_gripper(self, message):
        try:
            value = json.loads(message.data)
            if value.get('controller') == 'panda_gripper_native':
                self.gripper, self.gripper_received = value, time.monotonic()
        except (TypeError, ValueError):
            pass

    def receive_arm(self, message):
        try:
            self.arm = json.loads(message.data)
        except (TypeError, ValueError):
            pass

    def receive_joints(self, message):
        values = dict(zip(message.name, message.position))
        if all('joint'+str(i) in values for i in range(1, 8)) and all(map(math.isfinite, values.values())):
            self.joints, self.joints_received = values, time.monotonic()

    def receive_command(self, message):
        try:
            command = json.loads(message.data)['operation']
            if command not in ('attach', 'detach'):
                raise ValueError('Use attach or detach')
            if not self.enabled or self.future is not None or self.arm.get('busy'):
                raise ValueError('Wait for an idle owned fixture scene')
            if self.object_received < time.monotonic()-.8:
                raise ValueError('Fresh measured object state required')
            if command == 'attach' and (not self.grasped() or self.joints_received < time.monotonic()-.5):
                raise ValueError('Attach requires fresh joints and measured contact on both source fingers')
            self.command, self.error = command, ''
        except (ValueError, TypeError, KeyError) as exc:
            self.error = str(exc)

    def grasped(self):
        return (time.monotonic()-self.gripper_received < .8 and all(
            sum(float(c['normal_force_n']) for c in self.gripper.get('contacts', {}).get(name, [])
                if c.get('body') == 'manipulation_object') > .02
            for name in ('finger_joint1', 'finger_joint2')))

    def world_object(self):
        return self.convert(dict(type='box', size=self.object['size'], position=self.object['position'],
            orientation=self.object['orientation_wxyz']), 'manipulation_object')

    def scene(self):
        message = PlanningScene(is_diff=True)
        message.robot_state.is_diff = True
        return message

    def apply(self, scene, operation):
        request = ApplyPlanningScene.Request(scene=scene)
        self.pending, self.pending_at = operation, time.monotonic()
        self.future = self.node.client.call_async(request)
        self.pending_object = copy.deepcopy(self.object)

    def detach(self):
        scene = self.scene()
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name='panda_tcp', object=CollisionObject(id='manipulation_object', operation=CollisionObject.REMOVE))]
        scene.world.collision_objects = [self.world_object()]
        self.apply(scene, 'detach')

    def poll(self, status):
        if not self.enabled:
            return
        status.update(scope='Selected-world scene plus measured fixture; attachment affects planning only',
            scene_revision=self.revision, object_attached=self.attached, object_error=self.error)
        now = time.monotonic()
        fresh = now-self.object_received < .8
        if not fresh:
            status.update(ready=False, object_error='Waiting for fresh measured object geometry')
            return
        if self.future is not None:
            status['ready'] = False
            if now-self.pending_at > 10:
                self.error = 'Object scene acknowledgement timed out'
                # Keep the scene unready; an unknown outstanding apply must
                # not be overwritten by an unrelated diff.
                return
            if not self.future.done():
                return
            try:
                answer = self.future.result()
                if self.pending == 'fk':
                    if answer.error_code.val != 1 or not answer.pose_stamped or not self.grasped():
                        raise ValueError('Measured FK/contact unavailable for object attachment')
                    if (now-self.joints_received > .5 or any(abs(self.joints.get(name, math.inf)-value) > .002
                            for name, value in self.fk_joints.items())
                            or math.dist(self.fk_object['position'], self.object['position']) > .001
                            or abs(float(np.dot(self.fk_object['orientation_wxyz'],
                                self.object['orientation_wxyz']))) < .99999):
                        raise ValueError('Robot/object moved during FK; stop and request attachment again')
                    tool = answer.pose_stamped[0].pose
                    tq = [tool.orientation.w, tool.orientation.x, tool.orientation.y, tool.orientation.z]
                    offset = rotation(tq).T @ (np.asarray(self.object['position'])-
                        [tool.position.x, tool.position.y, tool.position.z])
                    q = quaternion_product([tq[0], -tq[1], -tq[2], -tq[3]], self.object['orientation_wxyz'])
                    obj = self.world_object()
                    obj.header.frame_id = 'panda_tcp'
                    obj.primitive_poses[0].position = Point(x=float(offset[0]), y=float(offset[1]), z=float(offset[2]))
                    obj.primitive_poses[0].orientation = Quaternion(w=float(q[0]), x=float(q[1]), y=float(q[2]), z=float(q[3]))
                    scene = self.scene()
                    scene.world.collision_objects = [CollisionObject(id=obj.id, operation=CollisionObject.REMOVE)]
                    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                        link_name='panda_tcp', object=obj, touch_links=self.object['finger_links'])]
                    self.future = None
                    self.apply(scene, 'attach')
                    return
                if not answer.success:
                    raise ValueError('MoveIt rejected the measured object scene diff')
                self.attached = self.pending == 'attach' or (self.attached and self.pending != 'detach')
                self.applied = self.pending_object
                self.revision += 1
                self.error = ''
            except Exception as exc:
                self.error = str(exc)
            self.future, self.pending = None, None
        if self.attached and not self.grasped():
            # Lost contact/reset removes the planning attachment and stops an
            # executing arm path; the physical object is never pose-welded.
            if self.arm.get('busy') and self.stop.service_is_ready():
                self.stop.call_async(Trigger.Request())
            self.detach()
        elif self.command == 'detach':
            self.command = None
            self.detach()
        elif self.command == 'attach':
            self.command = None
            if not self.fk.service_is_ready():
                self.error = 'MoveIt FK service unavailable'
            else:
                request = GetPositionFK.Request()
                request.header.frame_id, request.fk_link_names = 'native_world', ['panda_tcp']
                request.robot_state.joint_state.name = list(self.joints)
                request.robot_state.joint_state.position = list(self.joints.values())
                self.pending, self.pending_at = 'fk', now
                self.fk_joints, self.fk_object = self.joints.copy(), copy.deepcopy(self.object)
                self.future = self.fk.call_async(request)
        elif not self.attached and (self.applied is None or math.dist(
                self.applied['position'], self.object['position']) > .001 or min(np.linalg.norm(
                np.asarray(self.applied['orientation_wxyz'])-self.object['orientation_wxyz']), np.linalg.norm(
                np.asarray(self.applied['orientation_wxyz'])+self.object['orientation_wxyz'])) > .005):
            scene = self.scene()
            scene.world.collision_objects = [self.world_object()]
            pedestal = self.object.get('pedestal')
            if pedestal:
                scene.world.collision_objects.append(self.convert(pedestal, 'manipulation_pedestal'))
            self.apply(scene, 'update')
        moving = not self.attached and (np.linalg.norm(self.object['spatial_velocity'][3:]) > .02 or
                                        np.linalg.norm(self.object['spatial_velocity'][:3]) > .1)
        status.update(ready=self.future is None and not moving and not self.error,
            object_attached=self.attached, scene_revision=self.revision, object_error=self.error,
            object_moving=bool(moving), grasp_contacts_ready=bool(self.grasped()))
