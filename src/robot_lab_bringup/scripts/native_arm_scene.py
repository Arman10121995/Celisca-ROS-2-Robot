#!/usr/bin/env python3
"""Publish the selected canonical world's real static collision geometry.

An incomplete/unsupported world stays unready; the GUI must wait for the
ApplyPlanningScene acknowledgement before planning. This is a static scene,
not an implementation of dynamic-obstacle prediction or object attachment.
"""
import hashlib
import json
import time
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, LinkPadding, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import String

from robot_lab_utils.sdf_world import extract_static_shapes, uri_resolver


def collision_object(shape, name):
    message = CollisionObject(id=name)
    message.header.frame_id = 'native_world'
    message.operation = CollisionObject.ADD
    pose = Pose()
    pose.position = Point(x=shape['position'][0], y=shape['position'][1], z=shape['position'][2])
    w, x, y, z = shape['orientation']
    pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
    kind = shape['type']
    if kind in ('box', 'sphere', 'cylinder', 'plane'):
        primitive = SolidPrimitive()
        primitive.type = {'box': SolidPrimitive.BOX, 'sphere': SolidPrimitive.SPHERE,
                          'cylinder': SolidPrimitive.CYLINDER, 'plane': SolidPrimitive.BOX}[kind]
        dimensions = list(shape['size'])
        if kind == 'cylinder':
            dimensions.reverse()  # SolidPrimitive expects height, radius.
        elif kind == 'plane':
            dimensions.append(.1)
            pose.position.z -= .05
        primitive.dimensions = dimensions
        message.primitives, message.primitive_poses = [primitive], [pose]
    elif kind == 'mesh':
        from robot_lab_utils.mesh_assets import _load_indexed_mesh
        vertices, faces = _load_indexed_mesh(shape['mesh'])
        mesh = Mesh()
        mesh.vertices = [Point(x=float(v[0]*shape['scale'][0]),
                               y=float(v[1]*shape['scale'][1]),
                               z=float(v[2]*shape['scale'][2])) for v in vertices]
        mesh.triangles = [MeshTriangle(vertex_indices=[int(i) for i in f]) for f in faces]
        message.meshes, message.mesh_poses = [mesh], [pose]
    else:
        raise ValueError('Planning-scene shape needs conversion: '+kind)
    return message


class NativeArmScene(Node):
    def __init__(self):
        super().__init__('native_arm_scene')
        self.declare_parameter('world_path', '')
        self.declare_parameter('spawn_z', 0.)
        self.publisher = self.create_publisher(String, '/arm/planning_scene_status', 10)
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.read_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.status = dict(ready=False, scope='Static selected-world collision scene', geometry_count=0)
        self.future = None
        self.matrix_loaded = False
        self.request = ApplyPlanningScene.Request()
        self.request.scene = PlanningScene(is_diff=True)
        self.request.scene.robot_state.is_diff = True
        self.request.scene.link_padding = [LinkPadding(link_name='native_body_'+str(i), padding=.01)
                                            for i in range(2, 12)]
        self.start = time.monotonic()
        try:
            path = self.get_parameter('world_path').value
            if path:
                shapes, skipped = extract_static_shapes(path, uri_resolver(package_share=get_package_share_directory), collision_only=True)
                if skipped:
                    raise ValueError('Incomplete planning scene: '+'; '.join(skipped))
                self.status['world_path'] = str(Path(path).resolve())
                self.status['world_sha256'] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
            else:
                shapes = [dict(type='plane', model='ground_plane', position=[0., 0., 0.],
                               orientation=[1., 0., 0., 0.], size=[100., 100.])]
            self.request.scene.world.collision_objects = [collision_object(s, 'robot_lab_world_'+str(i)) for i, s in enumerate(shapes)]
            self.status['geometry_count'] = len(shapes)
            # The native fixed base may rest on the floor; all moving arm
            # links retain their floor collision checks.
            self.floors = ['robot_lab_world_'+str(i) for i, s in enumerate(shapes)
                      if s['type'] == 'plane' or (s['type'] == 'box'
                      and any(t in s['model'].lower() for t in ('floor', 'ground'))
                      and s['position'][2]+s['size'][2]/2 <= self.get_parameter('spawn_z').value+.005)]
        except Exception as exc:
            self.status['error'] = str(exc)
            self.get_logger().error(str(exc))
        self.create_timer(.5, self.poll, clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    def poll(self):
        if 'error' not in self.status and not self.status['ready']:
            if not self.matrix_loaded and self.future is None and self.read_client.service_is_ready():
                request = GetPlanningScene.Request()
                request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                self.future = self.read_client.call_async(request)
            elif not self.matrix_loaded and self.future is not None and self.future.done():
                # Retain the native semantic exclusions. A new partial ACM
                # would otherwise replace the planner's self-collision rules.
                matrix = self.future.result().scene.allowed_collision_matrix
                for name in ['native_body_1']+self.floors:
                    if name not in matrix.entry_names:
                        matrix.entry_names.append(name)
                        for entry in matrix.entry_values:
                            entry.enabled.append(False)
                        matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
                base = matrix.entry_names.index('native_body_1')
                for name in self.floors:
                    floor = matrix.entry_names.index(name)
                    matrix.entry_values[base].enabled[floor] = True
                    matrix.entry_values[floor].enabled[base] = True
                self.request.scene.allowed_collision_matrix = matrix
                self.matrix_loaded, self.future = True, None
            elif self.matrix_loaded and self.future is None and self.client.service_is_ready():
                self.future = self.client.call_async(self.request)
            elif self.future is not None and self.future.done():
                self.status['ready'] = bool(self.future.result().success)
                if not self.status['ready']:
                    self.status['error'] = 'MoveIt rejected the selected-world collision scene'
            elif time.monotonic()-self.start > 20:
                self.status['error'] = 'MoveIt planning-scene acknowledgement timed out'
        self.publisher.publish(String(data=json.dumps(self.status)))


def main(args=None):
    rclpy.init(args=args)
    node = NativeArmScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
