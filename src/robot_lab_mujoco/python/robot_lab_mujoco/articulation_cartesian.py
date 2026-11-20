"""Source-frame Cartesian IK on a private rigid articulation snapshot.

Uses original URDF fixed mounts and SciPy bounded least squares. Candidate
joint goals still require the existing collision planner and explicit Execute.
No inverse-kinematics result is written to the live plant.
"""
import math
import time
import warnings
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from robot_lab_utils.sim_frames import compose, rigid_sensor_offsets
from robot_lab_utils.bounded_control_planner import PlanningBudgetExceeded

SCHEMA = 'robot_lab.cartesian_goal.v1'


def rotation(quaternion):
    return Rotation.from_quat(np.asarray(quaternion)[[1, 2, 3, 0]])


def quaternion(rot):
    return rot.as_quat()[[3, 0, 1, 2]]


def vector(value, label, bound):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) or abs(v) > bound for v in value)):
        raise ValueError('Finite bounded three-component '+label+' required')
    return np.asarray(value, dtype=float)


class SourceTool:
    """Declared fixed TCP mount, surviving carrier and reviewed source chain."""
    def __init__(self, model, root_body, urdf, specs, configuration):
        if not isinstance(configuration, dict) or configuration.get('schema') != SCHEMA:
            raise ValueError('Unknown source Cartesian configuration')
        document = ET.fromstring(urdf)
        parents = {j.find('child').get('link'): j for j in document.findall('joint')}
        roots = {l.get('name') for l in document.findall('link')}-set(parents)
        self.tip_frame = configuration.get('tip_frame')
        self.reference_frame = configuration.get('reference_frame')
        names = configuration.get('joints')
        if not isinstance(names, (list, tuple)) or any(not isinstance(name, str) for name in names):
            raise ValueError('Cartesian configuration needs an ordered source joint list')
        self.names = tuple(names)
        if (roots != {self.reference_frame} or not self.names or len(self.names) > 8
                or len(set(self.names)) != len(self.names) or set(self.names)-set(specs)):
            raise ValueError('Cartesian source needs the URDF root and <=8 independent arm/torso joints')
        ancestors, visited, frame = set(), set(), self.tip_frame
        while frame in parents:
            if frame in visited:
                raise ValueError('Cartesian source chain contains a cycle')
            visited.add(frame)
            joint = parents[frame]
            if joint.get('type') != 'fixed':
                ancestors.add(joint.get('name'))
            frame = joint.find('parent').get('link')
        if frame != self.reference_frame or not set(self.names) <= ancestors:
            raise ValueError('Cartesian joints must belong to the declared TCP source chain')
        self.root_body = int(root_body)
        self.body = -1
        for carrier, offset in rigid_sensor_offsets(urdf, self.tip_frame).items():
            body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, carrier)
            if body >= 0:
                self.body, self.offset = body, offset
                break
        if self.body < 0:
            raise ValueError('The compiled plant lost the moving TCP carrier')

    def world_pose(self, data):
        position, orient = compose((data.xpos[self.body], data.xquat[self.body]), self.offset)
        return np.asarray(position), np.asarray(orient)

    def relative_pose(self, data):
        position, orient = self.world_pose(data)
        base = rotation(data.xquat[self.root_body])
        local = base.inv().apply(position-data.xpos[self.root_body])
        local_rot = base.inv()*rotation(orient)
        # A vertical tool has non-unique Euler angles. The control/solver keeps
        # quaternions; these degrees are only an operator-editable display.
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            rpy = local_rot.as_euler('xyz', degrees=True)
        return dict(schema=SCHEMA, frame=self.reference_frame, tip_frame=self.tip_frame,
                    position=local.tolist(), orientation_wxyz=quaternion(local_rot).tolist(),
                    rpy_deg=rpy.tolist())

    def normalize(self, request):
        if not isinstance(request, dict) or set(request)-{'frame', 'position', 'orientation', 'rpy_deg'}:
            raise ValueError('Cartesian goal must contain only its frame, position and orientation choice')
        if request.get('frame') != self.reference_frame:
            raise ValueError('Cartesian coordinates must use '+self.reference_frame)
        result = dict(frame=self.reference_frame, position=vector(request.get('position'), 'XYZ metres', 10.).tolist(),
                      orientation=request.get('orientation', 'keep'))
        if result['orientation'] not in ('keep', 'free', 'rpy'):
            raise ValueError('Choose keep, free or explicit RPY endpoint orientation')
        if result['orientation'] == 'rpy':
            result['rpy_deg'] = vector(request.get('rpy_deg'), 'RPY degrees', 180.).tolist()
        return result

    def world_goal(self, request, data):
        request = self.normalize(request)
        current = self.relative_pose(data)
        if np.linalg.norm(np.asarray(request['position'])-current['position']) > .35:
            raise ValueError('TCP goal is over 0.35 m from measured tool; use a smaller step or reposition the base')
        base = rotation(data.xquat[self.root_body])
        position = data.xpos[self.root_body]+base.apply(request['position'])
        orient = None
        if request['orientation'] == 'keep':
            orient = self.world_pose(data)[1]
        elif request['orientation'] == 'rpy':
            orient = quaternion(base*Rotation.from_euler('xyz', request['rpy_deg'], degrees=True))
        return request, dict(position=position.tolist(), orientation_wxyz=None if orient is None else orient.tolist())


def pose_error(tool, data, goal):
    position, orient = tool.world_pose(data)
    distance = float(np.linalg.norm(position-goal['position']))
    angle = None if goal['orientation_wxyz'] is None else float(
        (rotation(goal['orientation_wxyz'])*rotation(orient).inv()).magnitude())
    return dict(position_m=distance, orientation_rad=angle)


def inverse_candidates(scene, tool, goal, *, deadline, home=None):
    """At most eight deterministic seeds; collision-valid source joint goals."""
    lower = np.array([scene.specs[n]['lower'] for n in scene.names])
    upper = np.array([scene.specs[n]['upper'] for n in scene.names])
    span = upper-lower
    start = np.clip(np.array([scene.snapshot[scene.addresses[n]] for n in scene.names]), lower, upper)
    seeds = [start, (lower+upper)/2]
    if home and all(n in home for n in scene.names):
        seeds.insert(1, np.clip([home[n] for n in scene.names], lower, upper))
    random = np.random.default_rng(0)
    seeds.extend(lower+random.random(len(start))*span for _ in range(8-len(seeds)))
    wanted_rotation = None if goal['orientation_wxyz'] is None else rotation(goal['orientation_wxyz'])
    attempts, candidates = [], []
    best = None
    def forward(values):
        if time.monotonic() >= deadline:
            raise PlanningBudgetExceeded('Cartesian IK deadline reached')
        scene.data.qpos[:] = scene.snapshot
        for name, value in zip(scene.names, values):
            scene.data.qpos[scene.addresses[name]] = value
        mj.mj_kinematics(scene.model, scene.data)
        return tool.world_pose(scene.data)
    def residual(values):
        position, orient = forward(values)
        terms = [position-goal['position']]
        if wanted_rotation is not None:
            terms.append(.15*(wanted_rotation*rotation(orient).inv()).as_rotvec())
        terms.append(.0001*(values-start)/span)
        return np.concatenate(terms)
    for index, seed in enumerate(seeds):
        try:
            answer = least_squares(residual, np.clip(seed, lower+1e-9, upper-1e-9),
                bounds=(lower, upper), max_nfev=100, xtol=1e-7, ftol=1e-7, gtol=1e-7)
            forward(answer.x)
            error = pose_error(tool, scene.data, goal)
            best = error if best is None or error['position_m'] < best['position_m'] else best
            valid = error['position_m'] <= .003 and (error['orientation_rad'] is None or error['orientation_rad'] <= .025)
            collision_free = valid and scene.collision_free(answer.x)
            duplicate = any(np.max(np.abs((answer.x-prior)/span)) < .001 for prior in candidates)
            attempts.append(dict(seed=index, **error, collision_free=bool(collision_free), duplicate=duplicate))
            if collision_free and not duplicate:
                candidates.append(answer.x.copy())
                if len(candidates) == 4:
                    break
        except PlanningBudgetExceeded:
            break
    if not candidates:
        detail = '' if best is None else ' Best position error %.1f mm%s.' % (
            best['position_m']*1000., '' if best['orientation_rad'] is None else ', angle %.1f deg' % math.degrees(best['orientation_rad']))
        raise ValueError('No collision-free source IK candidate within bounds/budget.'+detail+
                         ' Try Use measured, a small XYZ step, Position only, or reposition the base.')
    candidates.sort(key=lambda q: float(np.linalg.norm((q-start)/span)))
    return [dict(zip(scene.names, map(float, q))) for q in candidates], attempts
