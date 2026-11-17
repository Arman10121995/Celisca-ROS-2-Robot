"""Top-down pick-and-place planning for a fixed URDF arm (ROADMAP R5.7).

Backend-neutral: MuJoCo is used here only as a private kinematics and
collision engine. It holds the robot's URDF import (the bridges' own
``_build_mjcf_from_urdf``), the world's static boxes and the object as a
mocap box. The live plant can be PyBullet, MuJoCo or Isaac. The node
(``robot_lab_bringup/scripts/urdf_pick_place.py``) executes the returned
joint waypoints through the shared articulation contract and judges grasp
and release from simulator truth. Nothing here commands or reads a plant.

Stages: open -> above the object -> down to the grasp -> close -> lift ->
over the support -> down to the place height -> open -> retreat up.

- Vertical moves are Cartesian straight lines: IK every ``CART_STEP_M`` along
  the line, each solve seeded from the previous one.
- The transfer is a joint-space segment.
- Every sampled configuration is collision-checked, with the object attached
  to the tool after the grasp.
- Finger/object contacts are allowed while grasping or holding, and
  object/support contacts while the object rests or is set down. Everything
  else is a collision.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

CART_STEP_M = 0.01          # spacing of IK solves along straight Cartesian moves
JOINT_STEP_RAD = 0.02       # collision samples along joint-space segments
POSITION_TOLERANCE_M = 0.002
ANGLE_TOLERANCE_RAD = 0.02
CONTACT_MARGIN_M = 0.002    # penetration below which a contact counts (dist < -margin)


@dataclass
class ArmSpec:
    """The arm's planning description (robot profile ``pick_place``)."""
    arm_joints: List[str]
    finger_joints: List[str]
    tool_body: str
    tcp_offset: Sequence[float] = (0.0, 0.0, 0.1034)
    finger_open: float = 0.04
    finger_closed: float = 0.0
    max_object_width: float = 0.075
    tool_yaw_offset: float = -math.pi / 4      # Panda: hand x axis is 45 deg from link8

    @classmethod
    def from_config(cls, config):
        return cls(arm_joints=list(config['arm_joints']), finger_joints=list(config['finger_joints']),
                   tool_body=str(config['tool_body']), tcp_offset=tuple(config.get('tcp_offset', (0., 0., .1034))),
                   finger_open=float(config.get('finger_open', .04)), finger_closed=float(config.get('finger_closed', 0.)),
                   max_object_width=float(config.get('max_object_width', .075)),
                   tool_yaw_offset=float(config.get('tool_yaw_offset', -math.pi / 4)))


@dataclass
class Box:
    name: str
    position: Sequence[float]
    quaternion: Sequence[float]      # w, x, y, z
    size: Sequence[float]            # full extents

    @property
    def top(self):
        return float(self.position[2]) + float(self.size[2]) / 2.0


def yaw_of(quaternion):
    w, x, y, z = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def top_down_rotation(yaw):
    """Rotation matrix of a tool pointing straight down (tool z = -world z), turned by yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    # tool x = (c, s, 0); tool z = (0, 0, -1); tool y = z cross x
    x = np.array([c, s, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class PlanningError(RuntimeError):
    pass


class ArmScene:
    """Private MuJoCo model: the arm, static boxes and one object box (mocap)."""

    def __init__(self, robot_mjcf, spec: ArmSpec, obstacles: Sequence[Box], obj: Box,
                 base=(0.0, 0.0, 0.0, 0.0)):
        import mujoco
        self.mj, self.spec = mujoco, spec
        root = ET.fromstring(robot_mjcf)
        world = root.find('worldbody')
        tool = next((b for b in world.iter('body') if b.get('name') == spec.tool_body), None)
        if tool is None:
            raise ValueError('tool body %s not in the robot model' % spec.tool_body)
        ET.SubElement(tool, 'site', name='pick_place_tcp', size='0.005',
                      pos='%g %g %g' % tuple(spec.tcp_offset))
        if any(abs(v) > 1e-12 for v in base):
            # The URDF arm is welded at the model origin; move the whole tree.
            wrapper = ET.Element('body', name='pick_place_base', pos='%g %g %g' % tuple(base[:3]),
                                 quat='%g 0 0 %g' % (math.cos(base[3] / 2), math.sin(base[3] / 2)))
            for child in [c for c in world if c.tag in ('body', 'geom')]:
                world.remove(child)
                wrapper.append(child)
            world.append(wrapper)
        for box in obstacles:
            ET.SubElement(world, 'geom', name='obstacle_' + box.name, type='box',
                          size='%g %g %g' % tuple(max(v, 1e-6) / 2 for v in box.size),
                          pos='%g %g %g' % tuple(box.position), quat='%g %g %g %g' % tuple(box.quaternion))
        holder = ET.SubElement(world, 'body', name='pick_place_object', mocap='true',
                               pos='%g %g %g' % tuple(obj.position), quat='%g %g %g %g' % tuple(obj.quaternion))
        ET.SubElement(holder, 'geom', name='object', type='box',
                      size='%g %g %g' % tuple(max(v, 1e-6) / 2 for v in obj.size), contype='1', conaffinity='1')
        option = root.find('option')
        if option is None:
            option = ET.SubElement(root, 'option')
        option.set('gravity', '0 0 0')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'pick_place_tcp')
        self.object_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'object')
        self.object_mocap = m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'pick_place_object')]
        self.obstacle_geoms = {box.name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'obstacle_' + box.name)
                               for box in obstacles}
        self.joint_qpos, self.joint_dof, self.limits = {}, {}, {}
        for name in spec.arm_joints + spec.finger_joints:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j < 0:
                raise ValueError('joint %s not in the robot model' % name)
            self.joint_qpos[name] = int(m.jnt_qposadr[j])
            self.joint_dof[name] = int(m.jnt_dofadr[j])
            self.limits[name] = (float(m.jnt_range[j][0]), float(m.jnt_range[j][1])) if m.jnt_limited[j] \
                else (-math.pi, math.pi)
        object_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'pick_place_object')
        robot_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] != 0 and m.geom_bodyid[g] != object_body
                       and (m.geom_contype[g] or m.geom_conaffinity[g])]
        finger_bodies = {int(m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f)]) for f in spec.finger_joints}
        self.robot_geoms = set(robot_geoms)
        self.finger_geoms = {g for g in robot_geoms if int(m.geom_bodyid[g]) in finger_bodies}
        self.attached = None        # object pose in the TCP frame while held

    # --- state ---------------------------------------------------------------
    def set_joints(self, q: Dict[str, float]):
        for name, value in q.items():
            self.data.qpos[self.joint_qpos[name]] = value
        self.mj.mj_kinematics(self.model, self.data)
        if self.attached is not None:
            position, rotation = self.tcp()
            self._place_object(position + rotation @ self.attached[0], rotation @ self.attached[1])
        self.mj.mj_forward(self.model, self.data)

    def tcp(self):
        return (self.data.site_xpos[self.site].copy(), self.data.site_xmat[self.site].reshape(3, 3).copy())

    def _place_object(self, position, rotation):
        quat = np.zeros(4)
        self.mj.mju_mat2Quat(quat, np.asarray(rotation, dtype=float).ravel())
        self.data.mocap_pos[self.object_mocap] = position
        self.data.mocap_quat[self.object_mocap] = quat

    def move_object(self, position, quaternion):
        rotation = np.zeros(9)
        self.mj.mju_quat2Mat(rotation, np.asarray(quaternion, dtype=float))
        self._place_object(np.asarray(position, dtype=float), rotation.reshape(3, 3))
        self.mj.mj_forward(self.model, self.data)

    def attach(self):
        """Hold the object at its current pose relative to the TCP."""
        position, rotation = self.tcp()
        obj_rotation = np.zeros(9)
        self.mj.mju_quat2Mat(obj_rotation, self.data.mocap_quat[self.object_mocap])
        self.attached = (rotation.T @ (self.data.mocap_pos[self.object_mocap] - position),
                         rotation.T @ obj_rotation.reshape(3, 3))

    def detach(self):
        self.attached = None

    # --- collision -------------------------------------------------------------
    def collisions(self, allow_fingers=False, allow_supports=()):
        """Contacts deeper than the margin, excluding allowed pairs: [(geom name, geom name, depth)]."""
        m, d, found = self.model, self.data, []
        allowed_supports = {self.obstacle_geoms[name] for name in allow_supports}
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist > -CONTACT_MARGIN_M:
                continue
            g1, g2 = int(c.geom1), int(c.geom2)
            pair = {g1, g2}
            if self.object_geom in pair:
                other = (pair - {self.object_geom}).pop() if len(pair) > 1 else self.object_geom
                if allow_fingers and other in self.finger_geoms:
                    continue
                if other in allowed_supports:
                    continue
            elif not (pair & self.robot_geoms):
                continue                # scenery against scenery
            found.append((self.mj.mj_id2name(m, self.mj.mjtObj.mjOBJ_GEOM, g1) or str(g1),
                          self.mj.mj_id2name(m, self.mj.mjtObj.mjOBJ_GEOM, g2) or str(g2), round(float(c.dist), 4)))
        return found

    # --- inverse kinematics ------------------------------------------------------
    def solve(self, position, rotation, seed: Dict[str, float], fingers: Dict[str, float], iterations=300):
        """Arm joints placing the TCP at position/rotation (damped least squares), or None."""
        m, d, names = self.model, self.data, self.spec.arm_joints
        q = np.array([seed[n] for n in names], dtype=float)
        low = np.array([self.limits[n][0] for n in names])
        high = np.array([self.limits[n][1] for n in names])
        dofs = [self.joint_dof[n] for n in names]
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        target_rotation = np.asarray(rotation, dtype=float)
        for _ in range(iterations):
            self.set_joints(dict(zip(names, q), **fingers))
            tcp_position, tcp_rotation = self.tcp()
            error_position = np.asarray(position) - tcp_position
            error_rotation = 0.5 * sum(np.cross(tcp_rotation[:, k], target_rotation[:, k]) for k in range(3))
            if np.linalg.norm(error_position) < POSITION_TOLERANCE_M and np.linalg.norm(error_rotation) < ANGLE_TOLERANCE_RAD:
                return dict(zip(names, map(float, q)))
            self.mj.mj_jacSite(m, d, jacp, jacr, self.site)
            jacobian = np.vstack([jacp[:, dofs], jacr[:, dofs]])
            error = np.concatenate([error_position, error_rotation])
            damping = 1e-3
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(6), error)
            q = np.clip(q + np.clip(step, -0.2, 0.2), low, high)
        return None

    def solve_seeded(self, position, rotation, seeds: Sequence[Dict[str, float]], fingers):
        """First IK solution over several seeds (a straight-up start is near singular)."""
        for seed in seeds:
            solution = self.solve(position, rotation, seed, fingers)
            if solution is not None:
                return solution
        return None

    # --- paths -----------------------------------------------------------------
    def straight_line(self, start: Dict[str, float], end_position, rotation, fingers, **allow):
        """IK waypoints along the straight TCP line from the current TCP position to end_position."""
        self.set_joints(dict(start, **fingers))
        begin = self.tcp()[0]
        length = float(np.linalg.norm(np.asarray(end_position) - begin))
        steps = max(1, int(math.ceil(length / CART_STEP_M)))
        q, path = dict(start), []
        for k in range(1, steps + 1):
            point = begin + (np.asarray(end_position) - begin) * k / steps
            solution = self.solve(point, rotation, q, fingers)
            if solution is None:
                raise PlanningError('no IK %.0f%% along a straight move' % (100 * k / steps))
            if max(abs(solution[n] - q[n]) for n in q) > 0.25:
                raise PlanningError('IK jumped %.2f rad along a straight move' % max(abs(solution[n] - q[n]) for n in q))
            self.set_joints(dict(solution, **fingers))
            hits = self.collisions(**allow)
            if hits:
                raise PlanningError('collision along a straight move: %s' % hits[:3])
            q = solution
            path.append(dict(solution))
        return path

    def joint_route(self, start, end, vias, fingers, **allow):
        """A joint segment, or through the first via pose (e.g. Home) whose two segments are clear."""
        try:
            return self.joint_segment(start, end, fingers, **allow)
        except PlanningError as direct:
            for via in vias:
                try:
                    return self.joint_segment(start, via, fingers, **allow) + self.joint_segment(via, end, fingers, **allow)
                except PlanningError:
                    continue
            raise direct

    def joint_segment(self, start: Dict[str, float], end: Dict[str, float], fingers, **allow):
        """Collision-checked linear joint interpolation; returns [end]."""
        a = np.array([start[n] for n in self.spec.arm_joints])
        b = np.array([end[n] for n in self.spec.arm_joints])
        steps = max(1, int(math.ceil(np.abs(b - a).max() / JOINT_STEP_RAD)))
        for k in range(1, steps + 1):
            q = dict(zip(self.spec.arm_joints, a + (b - a) * k / steps))
            self.set_joints(dict(q, **fingers))
            hits = self.collisions(**allow)
            if hits:
                raise PlanningError('collision on a joint move: %s' % hits[:3])
        return [dict(end)]


def plan_pick_place(scene: ArmScene, current: Dict[str, float], obj: Box, support: Box, *,
                    approach=0.10, lift=0.10, seeds: Sequence[Dict[str, float]] = ()):
    """Stage list [{stage, waypoints (arm joints), fingers}] for one pick and place.

    The object is grasped top-down at its centre, fingers across the
    object's x or y extent (whichever is narrower, within the gripper). It
    is placed at the support's top centre with the same orientation
    relative to the tool.
    """
    spec = scene.spec
    open_f = {n: spec.finger_open for n in spec.finger_joints}
    sizes = sorted(((float(obj.size[0]), 0.0), (float(obj.size[1]), math.pi / 2)))
    width, turn = sizes[0]
    if width > spec.max_object_width:
        raise PlanningError('object %.3f m wider than the gripper (%.3f m)' % (width, spec.max_object_width))
    object_yaw = yaw_of(obj.quaternion)
    # Fingers close along tool y: point tool x along the object's longer side.
    tool_yaw = object_yaw + turn + math.pi / 2 + spec.tool_yaw_offset
    # Two equivalent grasps 180 deg apart: take the one nearer the current wrist.
    scene.set_joints(dict(current, **open_f))
    current_rotation = scene.tcp()[1]
    current_yaw = math.atan2(current_rotation[1, 0], current_rotation[0, 0])
    tool_yaw = min((tool_yaw, tool_yaw + math.pi), key=lambda y: abs(math.atan2(math.sin(y - current_yaw), math.cos(y - current_yaw))))
    rotation = top_down_rotation(tool_yaw)
    centre = np.asarray(obj.position, dtype=float)
    grasp_point = centre.copy()            # TCP at the object's centre
    above_object = centre + [0, 0, approach]
    stages = []
    scene.detach()
    scene.move_object(obj.position, obj.quaternion)
    # Like MoveIt, never plan from a start that is already in collision (the
    # Panda's all-zero pose has link5 inside a finger): move Home first.
    scene.set_joints(dict(current, **open_f))
    start_hits = scene.collisions()
    if start_hits:
        raise PlanningError('start pose in collision (%s); move the arm Home first' % start_hits[:2])
    # Pre-grasp: any collision-free joint path to the solve above the object.
    # Seeds: the current pose, then the given ones (e.g. Home) and two
    # perturbed copies of them.
    rng = np.random.default_rng(0)
    seeds = [dict(current)] + [dict(s) for s in seeds] + [
        {n: v + float(rng.uniform(-0.3, 0.3)) for n, v in s.items()} for s in seeds for _ in range(2)]
    above = scene.solve_seeded(above_object, rotation, seeds, open_f)
    if above is None:
        raise PlanningError('no IK above the object')
    stages.append(dict(stage='open', waypoints=[dict(current)], fingers=open_f))
    vias = [{n: s[n] for n in spec.arm_joints} for s in seeds[1:]]
    stages.append(dict(stage='above_object', waypoints=scene.joint_route(current, above, vias, open_f), fingers=open_f))
    down = scene.straight_line(above, grasp_point, rotation, open_f, allow_fingers=True)
    stages.append(dict(stage='down_to_grasp', waypoints=down, fingers=open_f))
    grasp = down[-1]
    closed = {n: spec.finger_closed for n in spec.finger_joints}
    stages.append(dict(stage='close', waypoints=[dict(grasp)], fingers=closed))
    scene.set_joints(dict(grasp, **open_f))
    scene.attach()
    up = scene.straight_line(grasp, grasp_point + [0, 0, lift], rotation, open_f, allow_fingers=True,
                             allow_supports=tuple(scene.obstacle_geoms))
    stages.append(dict(stage='lift', waypoints=up, fingers=closed))
    place_centre = np.array([support.position[0], support.position[1], support.top + float(obj.size[2]) / 2.0])
    place_point = place_centre + (grasp_point - centre)
    above_place = place_point + [0, 0, lift]
    over = scene.solve_seeded(above_place, rotation, [up[-1]] + seeds[1:], open_f)
    if over is None:
        raise PlanningError('no IK above the support')
    stages.append(dict(stage='over_support', waypoints=scene.joint_route(up[-1], over, vias, open_f, allow_fingers=True),
                       fingers=closed))
    lower = scene.straight_line(over, place_point, rotation, open_f, allow_fingers=True,
                                allow_supports=(support.name,))
    stages.append(dict(stage='lower', waypoints=lower, fingers=closed))
    stages.append(dict(stage='release', waypoints=[dict(lower[-1])], fingers=open_f))
    scene.detach()
    scene.move_object(place_centre, obj.quaternion)
    retreat = scene.straight_line(lower[-1], place_point + [0, 0, approach], rotation, open_f, allow_fingers=True)
    stages.append(dict(stage='retreat', waypoints=retreat, fingers=open_f))
    return dict(stages=stages, grasp_width_m=width, tool_yaw=tool_yaw, place_centre=place_centre.tolist(),
                grasp_point=grasp_point.tolist())


def synchronised(waypoints: List[Dict[str, float]], start: Dict[str, float], rate: float, period: float):
    """Densify a waypoint list so no joint moves faster than rate (rad/s) between period-spaced targets."""
    out, previous = [], dict(start)
    for target in waypoints:
        delta = max(abs(target[n] - previous[n]) for n in target)
        steps = max(1, int(math.ceil(delta / (rate * period))))
        for k in range(1, steps + 1):
            out.append({n: previous[n] + (target[n] - previous[n]) * k / steps for n in target})
        previous = dict(target)
    return out
