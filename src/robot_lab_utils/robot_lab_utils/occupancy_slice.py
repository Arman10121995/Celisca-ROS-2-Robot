"""Conservative primitive height-slice cells, computed once per static shape."""
import itertools

import numpy as np


def primitive_slice_cells(shape, sizes, resolution, height):
    kind = shape['type']
    size = shape['size']
    if kind not in ('box', 'sphere', 'cylinder'):
        raise ValueError('Unsupported primitive: '+kind)
    extent = np.asarray(size)/2 if kind == 'box' else np.full(3, size[0])
    if kind == 'cylinder': extent[2] = size[1]/2
    quat = np.asarray(shape['orientation'], dtype=float)
    quat /= np.linalg.norm(quat)
    w,x,y,z = quat
    rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    corners = np.array([shape['position']+rotation@(extent*np.array(signs))
                        for signs in itertools.product((-1,1),repeat=3)])
    lower,upper = corners.min(axis=0),corners.max(axis=0)
    if not lower[2] <= height <= upper[2]: return None
    # Cell centres are (i+.5)*resolution-size/2; a cell occupies a square.
    first = np.ceil((lower[:2]+np.asarray(sizes)/2)/resolution-1).astype(int)
    last = np.floor((upper[:2]+np.asarray(sizes)/2)/resolution).astype(int)
    return tuple(int(v) for v in (*first,*last))


def terrain_slice_polygons(vertices, faces, height):
    """Project the solid part of each terrain triangle above a world z slice.

    Vertices already have the composed world transform. Clip against z >=
    height before projecting to XY: joining high samples in raster order
    invents barriers between disconnected hills and omits their interiors.
    Separate clipped triangles preserve both gaps and sloping intersections.
    """
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=int)
    if (vertices.ndim != 2 or vertices.shape[1] != 3
            or faces.ndim != 2 or faces.shape[1] != 3
            or not np.isfinite(vertices).all() or not np.isfinite(height)
            or (faces.size and (faces.min() < 0 or faces.max() >= len(vertices)))):
        raise ValueError('Terrain requires finite XYZ vertices and triangle indices')
    polygons = []
    for triangle in vertices[faces]:
        if triangle[:, 2].max() < height:
            continue
        clipped = []
        previous = triangle[-1]
        for current in triangle:
            previous_inside = previous[2] >= height
            current_inside = current[2] >= height
            if previous_inside != current_inside:
                fraction = (height - previous[2]) / (current[2] - previous[2])
                clipped.append(previous + fraction * (current - previous))
            if current_inside:
                clipped.append(current)
            previous = current
        if len(clipped) >= 3:
            polygons.append(np.asarray(clipped)[:, :2])
    return polygons
