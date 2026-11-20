"""Tripod grasp synthesis for native dexterous hands (ROADMAP R5.8).

Reviewed hands (``HANDS``): DEX-EE (2026-11-19) and the Wonik Allegro right
hand (2026-11-20). Each names a thumb, two opposing fingers, the joints and
channels per finger, its fingertip pad and the hand frame axes.

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

FINGERS = ('F0', 'F1', 'F2')   # DEX-EE thumb, then its two opposing fingers
PAD_NORMAL_TOLERANCE_DEG = 35.
FIT_TOLERANCE_MM = 1.5
FRICTION_FOR_QUALITY = .5     # below the source fingertip friction (1.0), for margin
# Box-centre search in the hand_base frame (m). DEX-EE fingers point along +z;
# the thumb sits on +y.
CENTRE_Y = np.arange(0., .0301, .005)
CENTRE_Z = np.arange(.29, .3301, .005)

# Per reviewed hand, in its base body frame: ``grasp_axis`` joins the thumb
# face (on the ``thumb_side`` of the box) to the opposing face; the two
# opponents sit at +-spread along ``spread_axis``; ``grid`` searches the box
# centre per axis (an absent axis is 0). ``pad`` is how the fingertip meets
# the face: DEX-EE's mesh pad has a site normal that must face the face;
# Allegro's tips are capsules, so the contact is the capsule point furthest
# along the face's inward normal and no pad orientation is imposed.
HANDS = {
    'shadow_dexee': dict(thumb='F0', opponents=('F1', 'F2'), base='hand_base',
        joint='%s/J%d', channel='%s/J%d_actuator', pad='mesh_site',
        grasp_axis=1, thumb_side=1., spread_axis=0, grid={1: CENTRE_Y, 2: CENTRE_Z},
        seed={finger: (0., -.6, .8, .3) for finger in FINGERS}),
    # Palm faces up with the fingers along world +x as authored (palm frame:
    # fingers +z, palm face +x). The thumb comes across the palm and meets
    # index+middle along the finger axis; the box is held above the palm.
    'wonik_allegro': dict(thumb='th', opponents=('ff', 'mf'), base='palm',
        joint='%sj%d', channel='%sa%d', pad='capsule',
        grasp_axis=2, thumb_side=-1., spread_axis=1,
        grid={0: np.arange(.04, .0901, .01), 1: np.arange(-.02, .0401, .01), 2: np.arange(-.03, .0101, .01)},
        seed={'th': (1.2, .6, .5, .5), 'ff': (0., 1., .8, .6), 'mf': (0., 1., .8, .6)},
        # Tip-only IK leaves the links free: reject placements where another
        # collision geom of the hand enters the box (sampled box/capsule points).
        clearance_m=.002,
        # The authored servos are kp 1 N m/rad, so a 4 mm squeeze carries almost
        # nothing, and a deeper geometric squeeze drags each tip along its curl
        # arc and twisted the box (8.6 deg once unsupported, 2026-11-20).
        # Instead approach, contact and press lie on one joint line
        # d = K^-1 J^T n: the tip meets the face at the planned point, and at
        # the press target q + F d the servo torques push it along the face
        # normal with F newtons. Separately solved approach poses touched
        # millimetres off plan; the thumb then pressed 3 mm below the fingers
        # and the 0.006 N m couple pitched the box past the 0.15 rad slip
        # limit on the compliant tips (2026-11-20).
        squeeze_force_n=2.),
}


def _geom_points(model, data, geom):
    """World sample points on a box or capsule collision geom (corners, faces, axis caps)."""
    import mujoco as mj
    size, kind = model.geom_size[geom], int(model.geom_type[geom])
    if kind == int(mj.mjtGeom.mjGEOM_BOX):
        local = np.array([[x, y, z] for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)])*size
    elif kind == int(mj.mjtGeom.mjGEOM_CAPSULE):
        radius, half = size[:2]
        rim = [[radius, 0, 0], [-radius, 0, 0], [0, radius, 0], [0, -radius, 0], [0, 0, 0]]
        local = np.array([[x, y, z+h] for h in (-half, 0., half) for x, y, z in rim]
                         + [[0, 0, half+radius], [0, 0, -half-radius]])
    else:
        return np.zeros((0, 3))
    return local@data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom]


def _penetrates(kin, fit, R, centre, half, margin):
    """Whether any non-tip hand collision geom enters the box at the fitted contact pose."""
    kin.data.qpos[:] = kin.reference
    for finger, entry in fit.items():
        kin.data.qpos[kin.ids[finger]['qpos']] = entry['q']
    kin.mj.mj_kinematics(kin.model, kin.data)
    tips = {entry['tip'] for entry in kin.ids.values()}
    for geom in range(kin.model.ngeom):
        if geom in tips or not (kin.model.geom_contype[geom] or kin.model.geom_conaffinity[geom]):
            continue
        if int(kin.model.geom_bodyid[geom]) == 0 or int(kin.model.body_rootid[kin.model.geom_bodyid[geom]]) != kin.root:
            continue
        local = (_geom_points(kin.model, kin.data, geom)-centre)@R
        if np.any(np.all(np.abs(local) < half+margin, axis=1)):
            return True
    return False


def _ids(model, hand):
    import mujoco as mj
    joint = lambda name: mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
    ids = {}
    for finger in (hand['thumb'], *hand['opponents']):
        joints = [joint(hand['joint'] % (finger, i)) for i in range(4)]
        if hand['pad'] == 'mesh_site':
            site = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, finger+'/distal_site')
            tip = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, finger+'/distal_geom_tip_col')
            kind = mj.mjtGeom.mjGEOM_MESH
        else:
            body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, finger+'_tip')
            tips = [g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == body
                    and int(model.geom_type[g]) == int(mj.mjtGeom.mjGEOM_CAPSULE) and model.geom_contype[g]]
            site, tip, kind = 0, tips[0] if body >= 0 and len(tips) == 1 else -1, mj.mjtGeom.mjGEOM_CAPSULE
        if min(joints+[site, tip]) < 0 or int(model.geom_type[tip]) != int(kind):
            raise ValueError('Hand finger %s joints/fingertip pad missing in executed model' % finger)
        ids[finger] = dict(qpos=[int(model.jnt_qposadr[j]) for j in joints], site=site, tip=tip,
            low=model.jnt_range[joints, 0].copy(), high=model.jnt_range[joints, 1].copy())
    base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, hand['base'])
    if base < 0:
        raise ValueError('Hand base body %s missing in executed model' % hand['base'])
    return ids, base


class _Kinematics:
    def __init__(self, model, data, hand=None):
        import mujoco as mj
        self.mj, self.model = mj, model
        self.hand = hand or HANDS['shadow_dexee']
        self.data = mj.MjData(model)
        self.data.qpos[:] = data.qpos
        self.ids, base = _ids(model, self.hand)
        mj.mj_kinematics(model, self.data)
        self.root = int(model.body_rootid[base])
        self.base_pos = self.data.xpos[base].copy()
        self.base_rot = self.data.xmat[base].reshape(3, 3).copy()
        self.reference = self.data.qpos.copy()
        self.vertices = {}
        if self.hand['pad'] == 'mesh_site':
            for finger, entry in self.ids.items():
                mesh = int(model.geom_dataid[entry['tip']])
                first = int(model.mesh_vertadr[mesh])
                self.vertices[finger] = model.mesh_vert[first:first+int(model.mesh_vertnum[mesh])].copy()

    def pad(self, finger, q, toward=None):
        """World contact-patch centre of the distal tip pad and its outward normal.

        For a capsule tip ``toward`` (the face's inward normal) selects the
        surface point that meets the face; its normal is ``toward`` itself.
        """
        entry = self.ids[finger]
        self.data.qpos[:] = self.reference
        self.data.qpos[entry['qpos']] = q
        self.mj.mj_kinematics(self.model, self.data)
        geom = entry['tip']
        if self.hand['pad'] == 'capsule':
            radius, half = self.model.geom_size[geom, :2]
            axis = self.data.geom_xmat[geom].reshape(3, 3)[:, 2]
            ends = self.data.geom_xpos[geom]+np.outer([-half, half], axis)
            n = np.asarray(toward, dtype=float)
            return ends[int(np.argmax(ends@n))]+radius*n, n.copy()
        normal = self.data.site_xmat[entry['site']].reshape(3, 3)[:, 1].copy()
        world = self.vertices[finger]@self.data.geom_xmat[geom].reshape(3, 3).T+self.data.geom_xpos[geom]
        forward = (world-self.data.site_xpos[entry['site']])@normal
        return world[forward >= forward.max()-.002].mean(axis=0), normal

    def normal_press(self, finger, q, inward, actuators):
        """Per-newton joint offset d = K^-1 J^T n of a position-servo press, and the pad's normal travel per unit d.

        At the contact pose a servo target ``q + F d`` pushes the pad along
        ``inward`` with ``F`` N; ``q - s d`` backs it off ``s`` times the travel.
        """
        point = self.pad(finger, q, inward)[0]
        self.mj.mj_comPos(self.model, self.data)   # mj_jac needs the com-based dofs
        entry = self.ids[finger]
        jacp = np.zeros((3, self.model.nv))
        body = int(self.model.geom_bodyid[entry['tip']])
        self.mj.mj_jac(self.model, self.data, jacp, None, point, body)
        dofs = [int(self.model.jnt_dofadr[int(np.flatnonzero(self.model.jnt_qposadr == adr)[0])]) for adr in entry['qpos']]
        jacobian = jacp[:, dofs]
        gains = np.array([float(self.model.actuator_gainprm[index, 0]) for index in actuators])
        offset = jacobian.T@np.asarray(inward)/gains
        return offset, float(np.asarray(inward)@(jacobian@offset))

    def reach(self, finger, point, normal, seed):
        """Joints putting the pad on ``point`` facing ``normal``; with fit errors (mm, degrees)."""
        from scipy.optimize import least_squares
        entry = self.ids[finger]
        def residual(q):
            p, n = self.pad(finger, q, normal)
            return np.concatenate([(p-point)*100., n-normal])
        result = least_squares(residual, seed, bounds=(entry['low']+1e-3, entry['high']-1e-3))
        p, n = self.pad(finger, result.x, normal)
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

    ``contract`` is the hand contract (its adapter selects the reviewed hand
    in ``HANDS``; its channels give the actuator names and bounds). Returns
    world centre, half size, box orientation (wxyz, the hand base
    orientation), ``approach`` and ``grasp`` targets by channel name, the
    channel joints, the force-closure quality and the per-finger fit; raises
    when no placement is reachable.
    """
    hand = HANDS.get(contract['adapter'])
    if hand is None:
        raise ValueError('No reviewed tripod synthesis for hand adapter: '+str(contract['adapter']))
    fingers = (hand['thumb'], *hand['opponents'])
    kin = _Kinematics(model, data, hand)
    R, origin = kin.base_rot, kin.base_pos
    axis, side, lateral = hand['grasp_axis'], hand['thumb_side'], hand['spread_axis']
    unit = np.eye(3)
    grid = [hand['grid'].get(i, np.zeros(1)) for i in range(3)]
    best = None
    for local in (np.array([x, y, z]) for x in grid[0] for y in grid[1] for z in grid[2]):
        contacts = {hand['thumb']: (local+side*half*unit[axis], -side*unit[axis]),
                    hand['opponents'][0]: (local-side*half*unit[axis]+spread*half*unit[lateral], side*unit[axis]),
                    hand['opponents'][1]: (local-side*half*unit[axis]-spread*half*unit[lateral], side*unit[axis])}
        fit, worst = {}, 0.
        for finger in fingers:
            point, inward = R@contacts[finger][0]+origin, R@np.array(contacts[finger][1])
            q, error_mm, normal_deg = kin.reach(finger, point, inward, np.array(hand['seed'][finger]))
            fit[finger] = dict(q=q, point=point, inward=inward, error_mm=error_mm, normal_deg=normal_deg)
            worst = max(worst, error_mm+max(0., normal_deg-PAD_NORMAL_TOLERANCE_DEG)/5.)
            if worst > FIT_TOLERANCE_MM:
                break
        if worst > FIT_TOLERANCE_MM:
            continue
        centre = R@local+origin
        if hand.get('clearance_m') is not None and _penetrates(kin, fit, R, centre, half, hand['clearance_m']):
            continue
        # Rounded tips meet a flat face along its normal: quality from the
        # reached pad points and the inward face normals.
        reached = [kin.pad(finger, fit[finger]['q'], fit[finger]['inward'])[0] for finger in fingers]
        quality = epsilon_quality(reached, [fit[finger]['inward'] for finger in fingers], centre)
        score = quality-.05*worst
        if best is None or score > best['score']:
            best = dict(score=score, local=local, centre=centre, fit=fit, quality=quality, worst=worst)
    if best is None or best['quality'] <= 0:
        raise ValueError('No reachable force-closure tripod for this box on the executed hand')
    targets = dict(approach={}, grasp={}, joints={})
    channels = {channel['name']: channel for channel in contract['channels']}
    for finger in fingers:
        entry = best['fit'][finger]
        if hand.get('squeeze_force_n'):
            offset, travel = kin.normal_press(finger, entry['q'], entry['inward'],
                [channels[hand['channel'] % (finger, n)]['index'] for n in range(4)])
            if travel <= 0:
                raise ValueError('Finger %s cannot press along the face normal' % finger)
            grasp = entry['q']+hand['squeeze_force_n']*offset
            approach = entry['q']-backoff/travel*offset
        else:
            grasp = kin.reach(finger, entry['point']+entry['inward']*squeeze, entry['inward'], entry['q'])[0]
            approach = kin.reach(finger, entry['point']-entry['inward']*backoff, entry['inward'], entry['q'])[0]
        for number in range(4):
            name = hand['channel'] % (finger, number)
            low, high = channels[name]['limits']
            targets['grasp'][name] = float(np.clip(grasp[number], low, high))
            targets['approach'][name] = float(np.clip(approach[number], low, high))
            targets['joints'][name] = hand['joint'] % (finger, number)
    # Fingers outside the tripod (Allegro's ring) stay at their reference pose.
    for name, channel in channels.items():
        if name not in targets['grasp'] and channel.get('joint'):
            joint = kin.mj.mj_name2id(model, kin.mj.mjtObj.mjOBJ_JOINT, channel['joint'])
            value = float(np.clip(kin.reference[int(model.jnt_qposadr[joint])], *channel['limits']))
            targets['grasp'][name] = targets['approach'][name] = value
            targets['joints'][name] = channel['joint']
    quat = np.zeros(4)
    kin.mj.mju_mat2Quat(quat, R.reshape(-1))
    if abs(R[2, 2]) < .999:
        # The cube's faces lie on the hand-frame axes. When the hand frame is
        # not z-up (Allegro's fingers point along world x), give the same cube
        # a yaw-only orientation so its support stage stays vertical.
        level = [R[:, i] for i in range(3) if abs(R[2, i]) < 1e-6]
        if len(level) != 2 or max(abs(R[2, i]) for i in range(3)) < 1-1e-6:
            raise ValueError('Tripod box needs a hand frame with one vertical axis')
        yaw = np.arctan2(level[0][1], level[0][0])
        quat = np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)])
    return dict(centre=best['centre'].tolist(), half_size=[float(half)]*3, quat_wxyz=[float(v) for v in quat],
        centre_in_hand=best['local'].tolist(), approach=targets['approach'], grasp=targets['grasp'],
        joints=targets['joints'],
        force_closure_epsilon=round(best['quality'], 4), backoff_m=backoff, spread=spread,
        **(dict(squeeze_force_n=hand['squeeze_force_n']) if hand.get('squeeze_force_n') else dict(squeeze_m=squeeze)),
        fit={finger: dict(error_mm=round(best['fit'][finger]['error_mm'], 3),
                          pad_normal_deg=round(best['fit'][finger]['normal_deg'], 1)) for finger in fingers},
        scope='Synthesized from the executed kinematics: thumb against %s+%s on opposite faces at the box mid-plane; '
              'quality is a model estimate, not a measured hold' % hand['opponents'])
