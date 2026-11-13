"""Conservative XY hull of measured native collision bounds, including payload.

Geometry stays in the engine. This projects its authored local AABB corners
through current geometry/base poses; no object or robot state is changed.
"""
import numpy as np
from shapely.geometry import MultiPoint


CORNERS = np.array([[x, y, z] for x in (-1., 1.) for y in (-1., 1.) for z in (-1., 1.)])


def collision_footprint(model, data, base, robot_bodies, payload=None, padding=.03):
    if not np.isfinite(padding) or padding < 0:
        raise ValueError('Footprint padding must be finite and nonnegative')
    if not 0 < base < robot_bodies <= model.nbody:
        raise ValueError('Invalid native robot body range')
    bodies = set(range(1, robot_bodies))
    if payload is not None:
        if not robot_bodies <= payload < model.nbody:
            raise ValueError('Payload must be an independent world body')
        joint = int(model.body_jntadr[payload])
        # MuJoCo mjJNT_FREE = 0; no engine import is needed for projection.
        if joint < 0 or int(model.jnt_type[joint]) != 0:
            raise ValueError('Payload must have an independent free joint')
        children = {payload}
        for body in range(payload+1, model.nbody):
            if int(model.body_parentid[body]) in children:
                children.add(body)
        bodies.update(children)
    points, payload_geoms = [], 0
    rotation = data.xmat[base].reshape(3, 3)
    for geom in range(model.ngeom):
        body = int(model.geom_bodyid[geom])
        if body not in bodies or not (model.geom_contype[geom] or model.geom_conaffinity[geom]):
            continue
        if body >= robot_bodies:
            payload_geoms += 1
        center, half = model.geom_aabb[geom, :3], model.geom_aabb[geom, 3:]
        world = ((center+CORNERS*half) @ data.geom_xmat[geom].reshape(3, 3).T
                 + data.geom_xpos[geom])
        points.extend(((world-data.xpos[base]) @ rotation)[:, :2])
    if payload is not None and not payload_geoms:
        raise ValueError('Payload has no physical collision geometry')
    if not points or not np.all(np.isfinite(points)):
        raise ValueError('Missing/non-finite native footprint geometry')
    hull = MultiPoint(np.asarray(points)).convex_hull.buffer(padding, resolution=2)
    if hull.is_empty or hull.geom_type != 'Polygon' or hull.area <= 0:
        raise ValueError('Degenerate native collision footprint')
    return [[float(x), float(y)] for x, y in list(hull.exterior.coords)[:-1]]
