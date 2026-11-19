"""Tripod grasp synthesis for the DEX-EE three-finger hand (ROADMAP R5.8).

The Curl and Pinch templates close every finger along a fixed path and meet
a box at whatever points the path happens to cross; on 2026-11-17 those
points lay along a tilted line and the box rolled out of the fingertips as
its support dropped. This module chooses the contacts for the object first
and then the finger joints that reach them:

- **contacts:** the thumb (F0) on the centre of the box face toward it, and
  F1/F2 on the opposite face at ±``spread`` of its half width, all three
  in the horizontal plane through the box centre (the hand points up);
- **joints:** per finger, bounded least squares on the executed model's own
  kinematics puts the distal tip pad's contact patch on its point, with the
  pad facing the face. F1/F2 cannot face it squarely within their J0
  range; their rounded tips still press along the face normal, so a pad
  normal within 35° is accepted;
- **object placement:** a grid over the box centre in the hand frame keeps
  the best Ferrari-Canny force-closure quality among the reachable fits;
- **targets:** ``grasp`` aims ``squeeze`` metres inside each face, so the
  authored PID position servos press with bounded effort; ``approach`` is
  ``backoff`` metres outside, a clear pre-grasp opening that is also the
  pose the hand opens to.

Pure kinematics on a copy of the executed model's data: no simulation
state is written and nothing is welded or attached.
"""
import numpy as np

FINGERS = ('F0', 'F1', 'F2')
PAD_NORMAL_TOLERANCE_DEG = 35.
FIT_TOLERANCE_MM = 1.5
FRICTION_FOR_QUALITY = .5     # below the source fingertip friction (1.0), for margin
# Box-centre search in the hand_base frame (m). DEX-EE fingers point along +z;
# the thumb sits on +y.
CENTRE_Y = np.arange(0., .0301, .005)
CENTRE_Z = np.arange(.29, .3301, .005)


def _ids(model):
    import mujoco as mj
    joint = lambda name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
    ids = {}
    for finger in FINGERS:
        joints = [joint('%s/J%d' % (finger, i)) for i in range(4)]
        site = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, finger+'/distal_site')
        tip = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, finger+'/distal_geom_tip_col')
        if min(joints+[site, tip]) < 0 or int(model.geom_type[tip]) != int(mj.mjtGeom.mjGEOM_MESH):
            raise ValueError('DEX-EE finger %s joints/distal site/tip pad missing in executed model' % finger)
        ids[finger] = dict(qpos=[int(model.jnt_qposadr[j]) for j in joints], site=site, tip=tip,
            low=model.jnt_range[joints, 0].copy(), high=model.jnt_range[joints, 1].copy())
    base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'hand_base')
    if base < 0:
        raise ValueError('DEX-EE hand_base missing in executed model')
    return ids, base


class _Kinematics:
    def __init__(self, model, data):
        import mujoco as mj
        self.mj, self.model = mj, model
        self.data = mj.MjData(model)
        self.data.qpos[:] = data.qpos
        self.ids, base = _ids(model)
        mj.mj_kinematics(model, self.data)
        self.base_pos = self.data.xpos[base].copy()
        self.base_rot = self.data.xmat[base].reshape(3, 3).copy()
        self.reference = self.data.qpos.copy()
        self.vertices = {}
        for finger, entry in self.ids.items():
            mesh = int(model.geom_dataid[entry['tip']])
            first = int(model.mesh_vertadr[mesh])
            self.vertices[finger] = model.mesh_vert[first:first+int(model.mesh_vertnum[mesh])].copy()

    def pad(self, finger, q):
        """World contact-patch centre of the distal tip pad and its outward normal."""
        entry = self.ids[finger]
        self.data.qpos[:] = self.reference
        self.data.qpos[entry['qpos']] = q
        self.mj.mj_kinematics(self.model, self.data)
        normal = self.data.site_xmat[entry['site']].reshape(3, 3)[:, 1].copy()
        geom = entry['tip']
        world = self.vertices[finger]@self.data.geom_xmat[geom].reshape(3, 3).T+self.data.geom_xpos[geom]
        forward = (world-self.data.site_xpos[entry['site']])@normal
        return world[forward >= forward.max()-.002].mean(axis=0), normal

    def reach(self, finger, point, normal, seed):
        """Joints putting the pad on ``point`` facing ``normal``; with fit errors (mm, degrees)."""
        from scipy.optimize import least_squares
        entry = self.ids[finger]
        def residual(q):
            p, n = self.pad(finger, q)
            return np.concatenate([(p-point)*100., n-normal])
        result = least_squares(residual, seed, bounds=(entry['low']+1e-3, entry['high']-1e-3))
        p, n = self.pad(finger, result.x)
        return (result.x, float(np.linalg.norm(p-point))*1e3,
                float(np.degrees(np.arccos(np.clip(n@normal, -1., 1.)))))


def epsilon_quality(points, normals, centre, mu=FRICTION_FOR_QUALITY, edges=8, torque_scale=.02):
    """Ferrari-Canny epsilon of point contacts with friction cones; > 0 is force closure.

    ``normals`` point into the object; torques are scaled by ``torque_scale``
    (m) so forces and torques are comparable.
    """
    from scipy.spatial import ConvexHull
    from scipy.spatial import QhullError
    wrenches = []
    for p, n in zip(points, normals):
        n = np.asarray(n, dtype=float)/np.linalg.norm(n)
        a = np.cross(n, [1., 0., 0.]) if abs(n[0]) < .9 else np.cross(n, [0., 1., 0.])
        a /= np.linalg.norm(a)
        b = np.cross(n, a)
        for k in range(edges):
            t = 2*np.pi*k/edges
            f = n+mu*(np.cos(t)*a+np.sin(t)*b)
            f /= np.linalg.norm(f)
            wrenches.append(np.concatenate([f, np.cross(np.asarray(p)-centre, f)/torque_scale]))
    try:
        hull = ConvexHull(np.array(wrenches))
    except QhullError:
        return -1.
    return float(-hull.equations[:, -1].max())


def synthesize_tripod(model, data, contract, half=.02, spread=.6, squeeze=.004, backoff=.015):
    """Choose a box placement and per-finger joint targets for a three-finger tripod.

    ``contract`` is the DEX-EE hand contract (its channels give the actuator
    names and bounds). Returns world centre, half size, box orientation
    (wxyz, the hand_base orientation), ``approach`` and ``grasp`` targets by
    channel name, the force-closure quality and the per-finger fit; raises
    when no placement is reachable.
    """
    kin = _Kinematics(model, data)
    R, origin = kin.base_rot, kin.base_pos
    seed = np.array([0., -.6, .8, .3])
    best = None
    for cy in CENTRE_Y:
        for cz in CENTRE_Z:
            local = np.array([0., cy, cz])
            contacts = {'F0': (local+[0., half, 0.], [0., -1., 0.]),
                        'F1': (local+[spread*half, -half, 0.], [0., 1., 0.]),
                        'F2': (local+[-spread*half, -half, 0.], [0., 1., 0.])}
            fit, worst = {}, 0.
            for finger in FINGERS:
                point, inward = R@contacts[finger][0]+origin, R@np.array(contacts[finger][1])
                q, error_mm, normal_deg = kin.reach(finger, point, inward, seed)
                fit[finger] = dict(q=q, point=point, inward=inward, error_mm=error_mm, normal_deg=normal_deg)
                worst = max(worst, error_mm+max(0., normal_deg-PAD_NORMAL_TOLERANCE_DEG)/5.)
            if worst > FIT_TOLERANCE_MM:
                continue
            centre = R@local+origin
            # Rounded tips meet a flat face along its normal: quality from the
            # reached pad points and the inward face normals.
            reached = [kin.pad(finger, fit[finger]['q'])[0] for finger in FINGERS]
            quality = epsilon_quality(reached, [fit[finger]['inward'] for finger in FINGERS], centre)
            score = quality-.05*worst
            if best is None or score > best['score']:
                best = dict(score=score, local=local, centre=centre, fit=fit, quality=quality, worst=worst)
    if best is None or best['quality'] <= 0:
        raise ValueError('No reachable force-closure tripod for this box on the executed hand')
    targets = dict(approach={}, grasp={})
    channels = {channel['name']: channel for channel in contract['channels']}
    for finger in FINGERS:
        entry = best['fit'][finger]
        grasp = kin.reach(finger, entry['point']+entry['inward']*squeeze, entry['inward'], entry['q'])[0]
        approach = kin.reach(finger, entry['point']-entry['inward']*backoff, entry['inward'], entry['q'])[0]
        for number in range(4):
            name = '%s/J%d_actuator' % (finger, number)
            low, high = channels[name]['limits']
            targets['grasp'][name] = float(np.clip(grasp[number], low, high))
            targets['approach'][name] = float(np.clip(approach[number], low, high))
    quat = np.zeros(4)
    kin.mj.mju_mat2Quat(quat, R.reshape(-1))
    return dict(centre=best['centre'].tolist(), half_size=[float(half)]*3, quat_wxyz=[float(v) for v in quat],
        centre_in_hand=best['local'].tolist(), approach=targets['approach'], grasp=targets['grasp'],
        force_closure_epsilon=round(best['quality'], 4), squeeze_m=squeeze, backoff_m=backoff, spread=spread,
        fit={finger: dict(error_mm=round(best['fit'][finger]['error_mm'], 3),
                          pad_normal_deg=round(best['fit'][finger]['normal_deg'], 1)) for finger in FINGERS},
        scope='Synthesized from the executed kinematics: thumb against F1+F2 on opposite faces at the box mid-plane; '
              'quality is a model estimate, not a measured hold')
