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
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

CART_STEP_M = 0.01          # spacing of IK solves along straight Cartesian moves
JOINT_STEP_RAD = 0.02       # collision samples along joint-space segments
POSITION_TOLERANCE_M = 0.002
ANGLE_TOLERANCE_RAD = 0.02
CONTACT_MARGIN_M = 0.002    # penetration below which a contact counts (dist < -margin)
SEARCH_SECONDS = 15.0       # RRT-Connect budget when direct and via routes collide
GRASP_RAISE_STEP_M = 0.006  # grip this much higher when the fingertips would hit the support


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
                 base=(0.0, 0.0, 0.0, 0.0), held: Optional[Dict[str, float]] = None):
        import mujoco
        self.mj, self.spec = mujoco, spec
        root = ET.fromstring(robot_mjcf)
        world = root.find('worldbody')
        tool = next((b for b in world.iter('body') if b.get('name') == spec.tool_body), None)
        if tool is None:
            raise ValueError('tool body %s not in the robot model' % spec.tool_body)
        # A mobile robot's import floats its root; planning welds it at the
        # measured base pose instead.
        for body in [c for c in world if c.tag == 'body']:
            for joint in [j for j in body if j.tag == 'freejoint' or (j.tag == 'joint' and j.get('type') == 'free')]:
                body.remove(joint)
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
        # Geoms the plan moves: bodies at or below a planned joint. Contacts
        # between two robot geoms that never move (a mobile base's wheels in
        # its chassis) are fixed by the source model and ignored.
        planned = {int(m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)])
                   for n in spec.arm_joints + spec.finger_joints}

        def moved(body):
            while body > 0:
                if body in planned:
                    return True
                body = int(m.body_parentid[body])
            return False
        self.moving_geoms = {g for g in robot_geoms if moved(int(m.geom_bodyid[g]))}
        self.finger_geoms = {g for g in robot_geoms if int(m.geom_bodyid[g]) in finger_bodies}
        self.attached = None        # object pose in the TCP frame while held
        # Joints the plan never moves (a mobile manipulator's torso and head)
        # stay at their measured values.
        for name, value in (held or {}).items():
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j < 0:
                raise ValueError('held joint %s not in the robot model' % name)
            self.data.qpos[int(m.jnt_qposadr[j])] = float(value)
        mujoco.mj_forward(m, self.data)

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
            elif pair <= self.robot_geoms and not (pair & self.moving_geoms):
                continue                # fixed robot parts against each other
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
        """A joint segment; else through the first via pose (e.g. Home) whose two
        segments are clear; else a bounded RRT-Connect search
        (``robot_lab_utils.bounded_control_planner``)."""
        try:
            return self.joint_segment(start, end, fingers, **allow)
        except PlanningError as direct:
            for via in vias:
                try:
                    return self.joint_segment(start, via, fingers, **allow) + self.joint_segment(via, end, fingers, **allow)
                except PlanningError:
                    continue
            from robot_lab_utils.bounded_control_planner import connect_controls
            names = self.spec.arm_joints

            def clear(a, b):
                try:
                    self.joint_segment(dict(zip(names, a)), dict(zip(names, b)), fingers, **allow)
                    return True
                except PlanningError:
                    return False
            try:
                found = connect_controls([start[n] for n in names], [end[n] for n in names],
                                         [self.limits[n] for n in names], clear,
                                         deadline=time.monotonic() + SEARCH_SECONDS, max_nodes=600, max_edges=4000)
            except ValueError as exc:
                raise PlanningError('%s; search: %s' % (direct, exc)) from None
            return [dict(zip(names, map(float, point))) for point in found['path'][1:]]

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


def plan_pick(scene: ArmScene, current: Dict[str, float], obj: Box, *, approach=0.10, lift=0.10,
              seeds: Sequence[Dict[str, float]] = ()):
    """Stages open -> above the object -> down to the grasp -> close -> lift.

    The object is grasped top-down at its centre, fingers across the
    object's x or y extent (whichever is narrower, within the gripper). On
    return the scene holds the object at the lifted pose.
    """
    spec = scene.spec
    open_f = {n: spec.finger_open for n in spec.finger_joints}
    # Fingers close along tool y, across one horizontal extent of the object.
    # Grip across each extent that fits (narrower first), facing either way.
    sides = sorted(((float(obj.size[0]), 0.0), (float(obj.size[1]), math.pi / 2)))
    sides = [side for side in sides if side[0] <= spec.max_object_width]
    if not sides:
        raise PlanningError('object %.3f m wider than the gripper (%.3f m)' % (min(obj.size[:2]), spec.max_object_width))
    object_yaw = yaw_of(obj.quaternion)
    scene.set_joints(dict(current, **open_f))
    current_rotation = scene.tcp()[1]
    current_yaw = math.atan2(current_rotation[1, 0], current_rotation[0, 0])

    def turn_from_wrist(yaw):
        return abs(math.atan2(math.sin(yaw - current_yaw), math.cos(yaw - current_yaw)))
    candidates = []
    for rank, (width, turn) in enumerate(sides):
        base_yaw = object_yaw + turn + math.pi / 2 + spec.tool_yaw_offset
        candidates += sorted(((rank, width, yaw) for yaw in (base_yaw, base_yaw + math.pi)),
                             key=lambda c: turn_from_wrist(c[2]))
    errors = []
    for _rank, width, candidate in candidates:
        try:
            return _plan_pick_at(scene, current, obj, candidate, width, approach, lift, seeds)
        except PlanningError as exc:
            errors.append(str(exc))
    raise PlanningError(' / '.join(errors))


def _plan_pick_at(scene, current, obj, tool_yaw, width, approach, lift, seeds):
    spec = scene.spec
    open_f = {n: spec.finger_open for n in spec.finger_joints}
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
    seeds = _seeds(current, seeds)
    above = scene.solve_seeded(above_object, rotation, seeds, open_f)
    if above is None:
        raise PlanningError('no IK above the object')
    stages.append(dict(stage='open', waypoints=[dict(current)], fingers=open_f))
    vias = _high_vias(scene, above_object, rotation, above, open_f) + [{n: s[n] for n in spec.arm_joints} for s in seeds[1:]]
    stages.append(dict(stage='above_object', waypoints=scene.joint_route(current, above, vias, open_f), fingers=open_f))
    # Fingertips may reach below the TCP: grip higher on the object (in
    # GRASP_RAISE_STEP_M steps, keeping that much of its upper half) until
    # they clear what the object stands on.
    half = float(obj.size[2]) / 2.0
    raise_m = 0.0
    while True:
        try:
            down = scene.straight_line(above, grasp_point, rotation, open_f, allow_fingers=True)
            break
        except PlanningError:
            raise_m += GRASP_RAISE_STEP_M
            if raise_m > half - GRASP_RAISE_STEP_M:
                raise
            grasp_point = centre + [0, 0, raise_m]
    stages.append(dict(stage='down_to_grasp', waypoints=down, fingers=open_f))
    grasp = down[-1]
    closed = {n: spec.finger_closed for n in spec.finger_joints}
    stages.append(dict(stage='close', waypoints=[dict(grasp)], fingers=closed))
    scene.set_joints(dict(grasp, **open_f))
    scene.attach()
    up = scene.straight_line(grasp, grasp_point + [0, 0, lift], rotation, open_f, allow_fingers=True,
                             allow_supports=tuple(scene.obstacle_geoms))
    stages.append(dict(stage='lift', waypoints=up, fingers=closed))
    return dict(stages=stages, grasp_width_m=width, tool_yaw=tool_yaw, grasp_point=grasp_point.tolist())


def plan_place(scene: ArmScene, current: Dict[str, float], obj: Box, support: Box, *, place_xy=None,
               approach=0.10, lift=0.10, seeds: Sequence[Dict[str, float]] = ()):
    """Stages over the support -> down to the place height -> open -> retreat up.

    The scene must already hold the object (``ArmScene.attach``) at the arm
    pose ``current``. The object is set down at ``place_xy`` (default: the
    support's centre) on the support's top, keeping its pose relative to the
    tool; the tool stays pointing straight down.
    """
    spec = scene.spec
    if scene.attached is None:
        raise PlanningError('the scene does not hold the object')
    open_f = {n: spec.finger_open for n in spec.finger_joints}
    closed = {n: spec.finger_closed for n in spec.finger_joints}
    scene.set_joints(dict(current, **open_f))
    tool_position, tool_rotation = scene.tcp()
    tool_yaw = math.atan2(tool_rotation[1, 0], tool_rotation[0, 0])
    rotation = top_down_rotation(tool_yaw)
    held_centre = tool_position + tool_rotation @ scene.attached[0]
    xy = support.position[:2] if place_xy is None else place_xy
    place_centre = np.array([float(xy[0]), float(xy[1]), support.top + float(obj.size[2]) / 2.0])
    place_point = place_centre + (tool_position - held_centre)
    above_place = place_point + [0, 0, lift]
    seeds = _seeds(current, seeds)
    over = scene.solve_seeded(above_place, rotation, seeds, open_f)
    if over is None:
        raise PlanningError('no IK above the support')
    vias = _high_vias(scene, above_place, rotation, over, open_f) + [{n: s[n] for n in spec.arm_joints} for s in seeds[1:]]
    stages = [dict(stage='over_support', waypoints=scene.joint_route(current, over, vias, open_f, allow_fingers=True),
                   fingers=closed)]
    lower = scene.straight_line(over, place_point, rotation, open_f, allow_fingers=True,
                                allow_supports=(support.name,))
    stages.append(dict(stage='lower', waypoints=lower, fingers=closed))
    stages.append(dict(stage='release', waypoints=[dict(lower[-1])], fingers=open_f))
    scene.detach()
    scene.move_object(place_centre, obj.quaternion)
    retreat = scene.straight_line(lower[-1], place_point + [0, 0, approach], rotation, open_f, allow_fingers=True)
    stages.append(dict(stage='retreat', waypoints=retreat, fingers=open_f))
    return dict(stages=stages, place_centre=place_centre.tolist())


def _high_vias(scene, point, rotation, seed, fingers, heights=(0.15, 0.30)):
    """Arm poses with the tool straight above ``point``: detours that clear a table edge."""
    vias = []
    for height in heights:
        solution = scene.solve(np.asarray(point) + [0, 0, height], rotation, seed, fingers)
        if solution is not None:
            vias.append(solution)
    return vias


def _seeds(current, seeds):
    rng = np.random.default_rng(0)
    return [dict(current)] + [dict(s) for s in seeds] + [
        {n: v + float(rng.uniform(-0.3, 0.3)) for n, v in s.items()} for s in seeds for _ in range(2)]


def plan_pick_place(scene: ArmScene, current: Dict[str, float], obj: Box, support: Box, *,
                    approach=0.10, lift=0.10, seeds: Sequence[Dict[str, float]] = ()):
    """Stage list [{stage, waypoints (arm joints), fingers}] for one pick and place
    from a fixed base: ``plan_pick`` then ``plan_place`` at the support's centre."""
    pick = plan_pick(scene, current, obj, approach=approach, lift=lift, seeds=seeds)
    place = plan_place(scene, pick['stages'][-1]['waypoints'][-1], obj, support, approach=approach, lift=lift,
                       seeds=seeds)
    return dict(stages=pick['stages'] + place['stages'], grasp_width_m=pick['grasp_width_m'],
                tool_yaw=pick['tool_yaw'], place_centre=place['place_centre'], grasp_point=pick['grasp_point'])


# --- mobile manipulators: where the base stands -------------------------------
def _local(box: Box, point):
    yaw = yaw_of(box.quaternion)
    dx, dy = float(point[0]) - float(box.position[0]), float(point[1]) - float(box.position[1])
    return np.array([math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy])


def _world(box: Box, local):
    yaw = yaw_of(box.quaternion)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([box.position[0] + c * local[0] - s * local[1], box.position[1] + s * local[0] + c * local[1]])


def _edges(box: Box):
    """(outward unit normal in the box frame, half extent across, half extent along) per side."""
    hx, hy = float(box.size[0]) / 2.0, float(box.size[1]) / 2.0
    return [(np.array([1.0, 0.0]), hx, hy), (np.array([-1.0, 0.0]), hx, hy),
            (np.array([0.0, 1.0]), hy, hx), (np.array([0.0, -1.0]), hy, hx)]


def box_distance(box: Box, point):
    """2D distance from a point to a box footprint (0 inside)."""
    local = np.abs(_local(box, point)) - np.array([float(box.size[0]), float(box.size[1])]) / 2.0
    return float(np.linalg.norm(np.maximum(local, 0.0)))


def standoff(box: Box, target_xy, reach: float, robot_xy=None):
    """Base pose (x, y, yaw) facing the box side nearest ``target_xy``, ``reach`` m short of it.

    The side is the one with the target closest to its edge; on a tie, the
    side facing ``robot_xy``. Returns (x, y, yaw, depth): depth is how far the
    target lies inside that edge.
    """
    local = _local(box, target_xy)
    robot = _local(box, robot_xy) if robot_xy is not None else np.zeros(2)
    best = None
    for normal, across, _along in _edges(box):
        depth = across - float(normal @ local)
        key = (round(depth, 3), -float(normal @ robot))
        if best is None or key < best[0]:
            best = (key, normal, depth)
    _key, normal, depth = best
    base_local = local + normal * reach
    yaw = yaw_of(box.quaternion) + math.atan2(-normal[1], -normal[0])
    xy = _world(box, base_local)
    return float(xy[0]), float(xy[1]), math.atan2(math.sin(yaw), math.cos(yaw)), float(depth)


def place_standoff(box: Box, robot_xy, inset: float, reach: float, margin: float = 0.05):
    """Where to set an object down on ``box`` from the side facing ``robot_xy``, and where the base stands.

    The place point is ``inset`` m inside that edge, level with the robot
    along it (kept ``margin`` from the corners). The base faces the edge,
    ``reach`` m from the place point. Returns ((x, y, yaw), (place x, place y)).
    """
    robot = _local(box, robot_xy)
    best = None
    for normal, across, along in _edges(box):
        outside = float(normal @ robot) - across
        if best is None or outside > best[0]:
            best = (outside, normal, across, along)
    _outside, normal, across, along = best
    tangent = np.array([-normal[1], normal[0]])
    offset = max(-(along - margin), min(along - margin, float(tangent @ robot)))
    local = normal * (across - inset) + tangent * offset
    place = _world(box, local)
    base = _world(box, local + normal * reach)
    yaw = yaw_of(box.quaternion) + math.atan2(-normal[1], -normal[0])
    return (float(base[0]), float(base[1]), math.atan2(math.sin(yaw), math.cos(yaw))), (float(place[0]), float(place[1]))


def segment_clearance(boxes: Sequence[Box], start, end, step: float = 0.02):
    """Smallest 2D distance from the straight segment start -> end to any box, and that box's name."""
    start, end = np.asarray(start[:2], dtype=float), np.asarray(end[:2], dtype=float)
    steps = max(1, int(math.ceil(float(np.linalg.norm(end - start)) / step)))
    best = (math.inf, None)
    for k in range(steps + 1):
        point = start + (end - start) * k / steps
        for box in boxes:
            distance = box_distance(box, point)
            if distance < best[0]:
                best = (distance, box.name)
    return best


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
