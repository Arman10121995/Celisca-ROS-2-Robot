"""Conservative vertical takeoff sweeps over the actual selected static scene.

The native collision envelope has no extra margin beneath its feet, and the
lowest SUPPORT_BAND_M of it is not swept (the vehicle rises away from its
support). Side/top margins and rotating propeller bounds are explicit. This
is geometry checking, not flight/tracking qualification.
"""
import hashlib
import itertools
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(values):
    roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def x500_envelope(models, model_file=None):
    """Derive bounds from the pinned X500 boxes, including rotor sweep.

    Reject changed/unknown model semantics instead of reusing guessed bounds.
    Values are relative to base_link; the runtime's FCU map origin shares it.
    """
    models = Path(models)
    root = Path(model_file) if model_file else models/'x500/model.sdf'
    base = models/'x500_base/model.sdf'
    root_model, base_model = ET.parse(root).getroot().find('model'), ET.parse(base).getroot().find('model')
    includes = root_model.findall('include')
    if (len(includes) != 1 or includes[0].findtext('uri') != 'model://x500_base'
            or root_model.findall('link') or includes[0].find('pose') is not None):
        raise ValueError('Takeoff checker requires the installed, unmodified X500/base include contract.')
    points = []
    for link in base_model.findall('link'):
        link_pose = link.find('pose')
        if link_pose is not None and link_pose.get('relative_to'):
            raise ValueError('Unsupported relative native X500 link pose.')
        pose = [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]
        if len(pose) != 6:
            raise ValueError('Native X500 poses must have six coordinates.')
        lr = rpy_matrix(pose[3:])
        for collision in link.findall('collision'):
            if collision.find('pose') is not None and collision.find('pose').get('relative_to'):
                raise ValueError('Unsupported relative native X500 collision pose.')
            cp = [float(v) for v in collision.findtext('pose', '0 0 0 0 0 0').split()]
            size = [float(v) for v in collision.findtext('geometry/box/size', '').split()]
            if len(cp) != 6 or len(size) != 3 or min(size) <= 0:
                raise ValueError('Takeoff envelope requires all source X500 collisions to be valid boxes.')
            local = np.array(list(itertools.product((-1., 1.), repeat=3)))*np.asarray(size)/2
            local = local @ rpy_matrix(cp[3:]).T+cp[:3]
            if link.get('name', '').startswith('rotor_'):
                # A spinning propeller fills its radial disk, not just the
                # narrow box at the imported joint angle.
                radial = float(np.max(np.linalg.norm(local[:, :2], axis=1)))
                lower_z, upper_z = float(local[:, 2].min()), float(local[:, 2].max())
                local = np.array(list(itertools.product((-radial, radial), (-radial, radial), (lower_z, upper_z))))
            points.extend(local @ lr.T+pose[:3])
    if not points:
        raise ValueError('Native X500 collision envelope is empty.')
    points = np.asarray(points)
    # Keep yaw independent and retain .06 m horizontal/top geometric margin.
    radius = float(np.max(np.linalg.norm(points[:, :2], axis=1)))+.06
    lower = [-radius, -radius, float(points[:, 2].min())]
    upper = [radius, radius, float(points[:, 2].max())+.06]
    stamps = {str(path.resolve()): (path.stat().st_mtime_ns, path.stat().st_size) for path in (root, base)}
    return dict(lower=lower, upper=upper, horizontal_margin_m=.06, top_margin_m=.06,
        bottom_margin_m=0., rotor_sweep=True, source_sha256=hashlib.sha256(root.read_bytes()+base.read_bytes()).hexdigest(),
        resources=stamps)


def extend_envelope(envelope, points, source_file=None, margin=.06):
    """Grow an envelope to cover extra body points (e.g. a mounted camera).

    Uses the same yaw-independent radius and side/top margins as
    x500_envelope; the bottom keeps no margin.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ValueError('Envelope extension needs finite 3D points.')
    radius = max(envelope['upper'][0], float(np.max(np.linalg.norm(points[:, :2], axis=1)))+margin)
    extended = dict(envelope, lower=[-radius, -radius, min(envelope['lower'][2], float(points[:, 2].min()))],
                    upper=[radius, radius, max(envelope['upper'][2], float(points[:, 2].max())+margin)])
    if source_file is not None:
        source_file = Path(source_file)
        extended['source_sha256'] = hashlib.sha256(
            envelope['source_sha256'].encode()+source_file.read_bytes()).hexdigest()
        extended['resources'] = dict(envelope.get('resources', {}),
            **{str(source_file.resolve()): (source_file.stat().st_mtime_ns, source_file.stat().st_size)})
    return extended


def triangle_box_intersection(triangles, center, half):
    """Vectorized triangle/AABB separating axes, including open mesh walls."""
    vertices = np.asarray(triangles)-center
    candidates = np.all(vertices.min(axis=1) <= half+1e-9, axis=1)&np.all(vertices.max(axis=1) >= -half-1e-9, axis=1)
    vertices = vertices[candidates]
    if not len(vertices):
        return False
    edges = np.roll(vertices, -1, axis=1)-vertices
    axes = [np.cross(edges[:, 0], edges[:, 1])]
    for edge in range(3):
        for basis in np.eye(3):
            axes.append(np.cross(edges[:, edge], basis))
    possible = np.ones(len(vertices), dtype=bool)
    for axis in axes:
        projection = np.einsum('tvi,ti->tv', vertices, axis)
        radius = np.abs(axis) @ half
        possible &= (projection.min(axis=1) <= radius+1e-9)&(projection.max(axis=1) >= -radius-1e-9)
        if not np.any(possible):
            return False
    return bool(np.any(possible))


def obb_intersection(center, half, other_center, other_half, rotation):
    delta = other_center-center
    axes = [*np.eye(3), *rotation.T]
    axes.extend(np.cross(a, b) for a in np.eye(3) for b in rotation.T)
    for axis in axes:
        if abs(delta @ axis) > half @ abs(axis)+other_half @ abs(rotation.T @ axis)+1e-9:
            return False
    return True


def intersects(scene, lower, upper, deadline, canceled):
    center, half = (lower+upper)/2, (upper-lower)/2
    corners = np.array(list(itertools.product((-1., 1.), repeat=3)))*half+center
    for shape in scene.shapes:
        if canceled() or time.monotonic() > deadline:
            raise TimeoutError('Takeoff geometry check canceled/deadline exceeded.')
        rotation, position = shape['rotation'], np.asarray(shape['position'])
        kind = shape['type']
        if kind == 'box':
            if obb_intersection(center, half, position, np.asarray(shape['size'])/2, rotation):
                return True
        elif kind == 'sphere':
            if np.linalg.norm(np.maximum(abs(position-center)-half, 0)) <= shape['size'][0]:
                return True
        elif kind == 'cylinder':
            radius, height = shape['size']
            # The enclosing authored oriented box is conservative for a
            # cylinder; it can reject space beside a curved corner.
            if obb_intersection(center, half, position, np.array([radius, radius, height/2]), rotation):
                return True
        elif kind == 'plane':
            plane = np.array([[-1., -1., 0.], [1., -1., 0.], [1., 1., 0.], [-1., 1., 0.]])
            plane[:, :2] *= np.asarray(shape['size'])/2
            vertices = plane @ rotation.T+position
            if triangle_box_intersection(vertices[[[0, 1, 2], [0, 2, 3]]], center, half):
                return True
        else:
            surface = shape['surface']
            local_corners = (corners-position) @ rotation
            local_min, local_max = local_corners.min(axis=0), local_corners.max(axis=0)
            indices = list(surface.triangles_tree.intersection(tuple(np.r_[local_min, local_max])))
            for start in range(0, len(indices), 2048):
                if canceled() or time.monotonic() > deadline:
                    raise TimeoutError('Takeoff mesh check canceled/deadline exceeded.')
                triangles = surface.triangles[indices[start:start+2048]] @ rotation.T+position
                if triangle_box_intersection(triangles, center, half):
                    return True
            local_center = (center-position) @ rotation
            if shape['solid'] and surface.contains([local_center])[0]:
                return True
            if kind == 'heightmap':
                # Reject a wholly underground box too; open heightfields
                # enclose terrain below their upper surface.
                lo, hi = shape['bounds']
                if np.all(local_center[:2] >= lo[:2]) and np.all(local_center[:2] <= hi[:2]):
                    hits, _, _ = surface.ray.intersects_location(
                        [[local_center[0], local_center[1], hi[2]+1]], [[0., 0., -1.]], multiple_hits=False)
                    if len(hits) and local_center[2] <= hits[0, 2]:
                        return True
    return False


# The lowest band of the starting envelope is not swept.  A vertical rise
# moves away from whatever the landed vehicle stands on, and the FCU height
# is aligned once at spawn + 0.24 m while the feet sit 0.227 m below
# base_link: 13 mm of nominal clearance that estimator noise or contact
# penetration erases, after which the support floor itself rejected every
# check.  Anything taller than the band beneath the vehicle is still found.
SUPPORT_BAND_M = .05


def check_takeoff(scene, start, envelope, desired_rise=3., minimum_rise=.8,
                  timeout=15., canceled=lambda: False, support_band=SUPPORT_BAND_M):
    start = np.asarray(start, dtype=float)
    low, high = np.asarray(envelope['lower'], dtype=float), np.asarray(envelope['upper'], dtype=float)
    if not 0 <= support_band < high[2]-low[2]:
        raise ValueError('Support band must lie within the vehicle envelope.')
    low = low+[0., 0., support_band]
    if (start.shape != (3,) or low.shape != (3,) or high.shape != (3,)
            or not np.all(np.isfinite([start, low, high])) or np.any(high <= low)
            or not math.isfinite(desired_rise) or not minimum_rise <= desired_rise <= 10):
        raise ValueError('Finite source envelope and desired rise within 0.8–10 m required.')
    deadline = time.monotonic()+timeout
    def blocked(rise):
        return intersects(scene, start+low, start+high+[0., 0., rise], deadline, canceled)
    if blocked(minimum_rise):
        raise ValueError('Selected world blocks the minimum 0.8 m takeoff corridor; choose a different spawn.')
    if not blocked(desired_rise):
        rise = desired_rise
    else:
        lower_rise, upper_rise = minimum_rise, desired_rise
        for _ in range(12):
            middle = (lower_rise+upper_rise)/2
            if blocked(middle):
                upper_rise = middle
            else:
                lower_rise = middle
        # Preserve an additional vertical margin when adapting to a ceiling.
        rise = max(minimum_rise, lower_rise-.1)
        if blocked(rise):
            raise ValueError('Could not establish a clear bounded takeoff corridor.')
    return dict(start_enu=start.tolist(), target_enu=(start+[0., 0., rise]).tolist(),
        rise_m=rise, requested_rise_m=desired_rise, ceiling_limited=rise < desired_rise-1e-6,
        envelope={key: value for key, value in envelope.items() if key != 'resources'},
        unchecked_support_band_m=support_band,
        geometry_scope='Static selected-world vertical envelope; support, actors and flight tracking require physical validation')
