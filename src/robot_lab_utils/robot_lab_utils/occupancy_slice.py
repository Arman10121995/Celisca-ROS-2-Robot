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
