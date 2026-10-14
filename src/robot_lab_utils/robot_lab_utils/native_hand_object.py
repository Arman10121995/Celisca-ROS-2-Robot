"""Source-faithful native-hand object geometry and measured grasp evaluation.

This derives a documented free object for the installed native hands and
evaluates a *measured* grasp from contact evidence. It never writes an object
pose, welds a body, or claims a stable grasp. The physical object stays free
and its motion is owned by the original contact physics; this module only

  * computes where a supported object should be authored between the opposing
    grasp surfaces of each hand, from measured fingertip/pad geometry, and
  * classifies measured two-sided contact, support and slip into a truthful
    grasp state.

Panda keeps its own separately measured gripper cube workflow; this module is
only for the source-authored dexterous/coupled-tendon hands.
"""
import numpy as np


# Opposing grasp surfaces per adapter, by body-name substring. A grasp needs
# measured contact on both sides of the object; these name the two sides.
GRASP_SIDES = {
    'shadow_dexee': (('F0/',), ('F1/', 'F2/')),      # thumb vs index+middle pinch
    'robotiq_2f85': (('left_',), ('right_',)),       # coupled finger pair
    'robotiq_2f85_v4': (('left_',), ('right_',)),
    'umi_gripper': (('left_finger',), ('right_finger')),
}

# The distal contact *pads* used to place the object, by exact body name. These
# are the surfaces that actually meet the object, so the object is centered and
# sized from them rather than from the whole finger linkage. The broader
# GRASP_SIDES sets remain in charge of measured two-sided contact detection.
GRASP_PADS = {
    'shadow_dexee': (('F0/finger_distal',), ('F1/finger_distal', 'F2/finger_distal')),
    'robotiq_2f85': (('left_pad',), ('right_pad',)),
    'robotiq_2f85_v4': (('left_pad',), ('right_pad',)),
    'umi_gripper': (('left_finger_holder',), ('right_finger_holder',)),
}


def _bodies_named(model, names, prefixes):
    """Body ids whose name starts with any prefix, in a stable order."""
    import mujoco as mj
    found = []
    for index in range(model.nbody):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, index) or ''
        if name in names or any(name.startswith(prefix) for prefix in prefixes):
            found.append(index)
    return found


def grasp_sides(adapter):
    """The two opposing body-name prefix groups used for two-sided contact."""
    sides = GRASP_SIDES.get(adapter)
    if sides is None:
        raise ValueError('No reviewed opposing grasp surfaces for hand adapter: '+str(adapter))
    return sides


def grasp_pads(model, adapter):
    """The two opposing distal-pad body-id sets used to place the object."""
    pads = GRASP_PADS.get(adapter)
    if pads is None:
        raise ValueError('No reviewed distal grasp pads for hand adapter: '+str(adapter))
    left = _bodies_named(model, set(), pads[0])
    right = _bodies_named(model, set(), pads[1])
    if not left or not right:
        raise ValueError('Native hand distal grasp pads missing in executed model')
    return [int(b) for b in left], [int(b) for b in right]


def grasp_surface_bodies(model, adapter, contract):
    """The two opposing body-id sets used for two-sided contact, by name.

    These ids are static for an executed model, so they can be resolved once
    without depending on the current finger pose.
    """
    left_prefixes, right_prefixes = grasp_sides(adapter)
    names = {channel['joint'] for channel in contract['channels']}
    left = _bodies_named(model, names, left_prefixes)
    right = _bodies_named(model, names, right_prefixes)
    if not left or not right:
        raise ValueError('Native hand grasp surfaces missing in executed model')
    return [int(b) for b in left], [int(b) for b in right]


def object_geometry(model, data, adapter, contract):
    """Derive a documented free object between the opposing grasp pads.

    Uses the measured distal-pad positions at the *open* source pose to center
    a small cube on the closing axis, sized to fit inside the open gap so the
    source actuators must actually close on it. Returns a dict with the authored
    center, half-size and the two opposing body-id sets, or raises when the
    executed geometry does not match the reviewed adapter.

    No pose is written and no object is added here; the caller authors the free
    body with these measured values.
    """
    import mujoco as mj
    left, right = grasp_pads(model, adapter)
    mj.mj_forward(model, data)
    def collision_points(bodies):
        points = []
        for geom in range(model.ngeom):
            if (int(model.geom_bodyid[geom]) not in bodies
                    or not (model.geom_contype[geom] or model.geom_conaffinity[geom])):
                continue
            kind, half = int(model.geom_type[geom]), model.geom_size[geom]
            if kind == int(mj.mjtGeom.mjGEOM_MESH):
                mesh = int(model.geom_dataid[geom])
                first, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
                local = model.mesh_vert[first:first+count]
            elif kind == int(mj.mjtGeom.mjGEOM_BOX):
                local = np.array([[x, y, z] for x in (-half[0], half[0])
                    for y in (-half[1], half[1]) for z in (-half[2], half[2])])
            else:
                raise ValueError('Hand object placement needs reviewed mesh/box collision pads')
            points.extend(local@data.geom_xmat[geom].reshape(3, 3).T+data.geom_xpos[geom])
        if not points:
            raise ValueError('Hand grasp pad has no authored collision surface')
        return np.asarray(points)
    left_points, right_points = collision_points(left), collision_points(right)
    left_center, right_center = left_points.mean(axis=0), right_points.mean(axis=0)
    axis = right_center-left_center
    span = float(np.linalg.norm(axis))
    if not np.isfinite(span) or span <= 1e-4:
        raise ValueError('Native hand opposing grasp surfaces are coincident')
    direction = axis/span
    center = (left_center+right_center)/2.
    left_face = float(np.max(left_points@direction))
    right_face = float(np.min(right_points@direction))
    gap = right_face-left_face
    if gap <= .002 or not np.isfinite(gap):
        raise ValueError('Hand has no clear collision-pad opening for a free object')
    center += ((left_face+right_face)/2.-float(center@direction))*direction
    # A cube that fits inside the open pad gap on the closing axis, so the
    # fingers must travel to meet and pass its faces. Keep the closing-axis
    # half-size well inside the open span, and cap the transverse size to the
    # smaller pad extent so the object stays between the pads, not beside them.
    # Axis-aligned cube projections remain inside the measured opening even
    # after an arbitrary source/global mount rotation. Keep it small enough
    # for the actual pad, rather than centering on a visual child body origin.
    half_along = min(.015, .3*gap/float(np.sum(np.abs(direction))))
    transverse = half_along
    half_size = np.array([transverse, transverse, transverse])
    closing = int(np.argmax(np.abs(direction)))
    half_size[closing] = half_along
    return dict(center=center.tolist(), half_size=half_size.tolist(),
        left_bodies=left, right_bodies=right,
        closing_axis=closing, direction=direction.tolist(), open_span=span, clear_gap=gap,
        placement_scope='Authored collision pad vertices; no visual-only pad origins')


def classify_grasp(left_force_n, right_force_n, object_supported, object_slip_m,
                   contact_threshold_n=.2, slip_threshold_m=.01):
    """Classify measured contact into a truthful grasp state.

    Inputs are measured: the summed external normal force on each opposing side
    (N), whether the object is still supported from below (bool), and how far the
    object has moved since the grasp was asserted (m). A grasp is only reported
    when both sides carry external contact; loss of either side, loss of support
    or measured slip downgrades the state honestly. This never asserts stability.
    """
    left_force_n, right_force_n = float(left_force_n), float(right_force_n)
    if (not np.all(np.isfinite([left_force_n, right_force_n, object_slip_m,
            contact_threshold_n, slip_threshold_m])) or min(left_force_n, right_force_n,
            object_slip_m) < 0 or min(contact_threshold_n, slip_threshold_m) <= 0):
        return dict(grasp='invalid_measurement', scope='Finite nonnegative contact/slip evidence required',
            two_sided_contact=False, supported=False, left_force_n=left_force_n,
            right_force_n=right_force_n, slip_m=float(object_slip_m))
    both = left_force_n >= contact_threshold_n and right_force_n >= contact_threshold_n
    state = dict(left_force_n=left_force_n, right_force_n=right_force_n,
        two_sided_contact=bool(both), supported=bool(object_supported),
        slip_m=float(object_slip_m))
    if not both:
        state['grasp'] = 'no_contact'
        state['scope'] = 'Measured contact below threshold on at least one opposing side'
    elif not object_supported:
        state['grasp'] = 'unsupported'
        state['scope'] = 'Two-sided contact but the object is not supported from below'
    elif object_slip_m > slip_threshold_m:
        state['grasp'] = 'slipping'
        state['scope'] = 'Two-sided contact but the object moved beyond the slip threshold'
    else:
        state['grasp'] = 'contact_hold'
        state['scope'] = 'Two-sided measured contact with support and no measured slip; stability/lift unassessed'
    return state
