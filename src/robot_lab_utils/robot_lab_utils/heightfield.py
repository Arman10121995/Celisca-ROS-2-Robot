"""SDF heightfield raster -> metres, shared by every non-Gazebo backend.

Gazebo reads ``<heightmap>`` natively; MuJoCo, PyBullet and Isaac cannot, and
each used to drop the terrain entirely.  This module is the one place that
turns the heightmap image plus the SDF ``<size>`` into geometry in metres, so
all three backends describe the same surface as Gazebo.

The conversion is not guesswork.  The layout below was measured against the
Gazebo on this host (gz-sim 8 / Harmonic, ODE) by dropping 0.2 m spheres onto
deliberately asymmetric rasters and reading the settled poses back from
``/world/<name>/dynamic_pose/info``; see
``docs/status/evidence/r6-7-heightfield-probe-2026-10-05/``.  Three details in
particular are easy to get wrong and are pinned by that evidence:

* **Elevation is normalised by the image's own maximum, not by 255.**  A
  uniform raster of value 100 (``size`` z = 4) produced a surface at 4.0 m,
  and bands of 50/150 produced 1.333 m and 4.0 m - i.e. ``v/max * size_z``.
  Normalising by 255 would have given 1.57 m and 2.35 m.
* **Image row 0 is maximum +y**, and column 0 is minimum x.  A raster whose
  top half was 0 and bottom half 255 put y=+8 m at 0.0 m and y=-8 m at 4.0 m.
* **The raster is centred on the collision pose** and spans ``(n-1)/2 *
  size/n`` in each direction, not ``size/2``.  Gazebo's own heightfield AABB
  measured +-9.69697 m for a 20 m / 33-sample raster (predicted 9.69697), and
  +-9.84615 m / +-9.41177 m for 65 / 17 samples.  The sample spacing is
  ``size/n`` with samples at cell centres, so the outer half-cell is missing.

Elevation is measured from the pose origin, so a terrain whose lowest sample
is 0 sits exactly at the collision's z rather than being centred on it.
"""

import os

import numpy as np


def load_heightmap(path):
    """Return the raster as a float array shaped (rows, columns), row 0 first.

    Grayscale images are read through Pillow; a plain ``.pgm``/``.pnm`` file
    is parsed directly so the conversion still works where Pillow is absent.
    """
    extension = os.path.splitext(path)[1].lower()
    if extension not in (".pgm", ".pnm", ".ppm"):
        from PIL import Image  # imported lazily: only raster formats need it
        with Image.open(path) as image:
            return np.asarray(image.convert("L"), dtype=float)
    return _read_pnm(path)


def _read_pnm(path):
    """Minimal binary/ASCII PNM reader (P2/P5/P3/P6) for heightmap rasters."""
    with open(path, "rb") as handle:
        data = handle.read()
    tokens, index = [], 0
    while len(tokens) < 4:
        while index < len(data) and data[index:index + 1].isspace():
            index += 1
        if data[index:index + 1] == b"#":
            while index < len(data) and data[index:index + 1] not in (b"\n", b"\r"):
                index += 1
            continue
        start = index
        while index < len(data) and not data[index:index + 1].isspace():
            index += 1
        tokens.append(data[start:index])
    magic = tokens[0].decode("ascii")
    width, height, maximum = int(tokens[1]), int(tokens[2]), int(tokens[3])
    body = data[index + 1:]
    if magic in ("P2", "P3"):
        flat = np.array(body.split()[:width * height], dtype=float)
    else:
        depth = 3 if magic == "P6" else 1
        dtype = np.uint8 if maximum < 256 else ">u2"
        flat = np.frombuffer(body[:width * height * depth],
                             dtype=dtype).astype(float)
        if depth == 3:  # collapse colour to luminance
            flat = flat.reshape(-1, 3).mean(axis=1)
    return flat.reshape(height, width)


def heightfield_grid(shape, samples=None):
    """Vertex grid for one ``sdf_world`` heightmap shape, in shape-local metres.

    Returns ``(vertices, faces)`` with ``vertices`` an (N, 3) float array and
    ``faces`` an (M, 3) int array of triangles wound counter-clockwise seen
    from above, matching the measured Gazebo layout.

    *samples* optionally decimates the raster by an integer factor per axis,
    which keeps very large heightmaps inside the triangle budgets the
    backends impose on static terrain.  Decimation takes the mean of the
    block, so the surface stays a smoothed version of the source.
    """
    elevations = load_heightmap(shape["heightmap"])
    size = [float(value) for value in shape["size"]]
    rows, columns = elevations.shape
    if samples and samples > 1:
        elevations = _decimate(elevations, samples)
        rows, columns = elevations.shape
    peak = float(elevations.max())
    if peak <= 0.0:
        raise ValueError("heightmap %s has no positive elevation"
                         % os.path.basename(shape["heightmap"]))
    # Measured: elevation is normalised by the raster's own maximum.
    heights = elevations * (size[2] / peak)

    cell_x, cell_y = size[0] / columns, size[1] / rows
    # Measured: cell-centred samples, so the grid spans (n-1)*size/n.
    xs = (np.arange(columns) + 0.5) * cell_x - size[0] / 2.0
    # Measured: raster row 0 is maximum +y, so the row axis is reversed.
    ys = size[1] / 2.0 - (np.arange(rows) + 0.5) * cell_y
    grid_x, grid_y = np.meshgrid(xs, ys)
    vertices = np.column_stack([grid_x.ravel(), grid_y.ravel(), heights.ravel()])

    index = np.arange(rows * columns).reshape(rows, columns)
    top_left = index[:-1, :-1].ravel()
    top_right = index[:-1, 1:].ravel()
    bottom_left = index[1:, :-1].ravel()
    bottom_right = index[1:, 1:].ravel()
    faces = np.vstack([
        np.column_stack([top_left, bottom_left, bottom_right]),
        np.column_stack([top_left, bottom_right, top_right])])
    return vertices, faces


def _decimate(elevations, factor):
    """Block-mean decimation, dropping a ragged trailing edge."""
    rows, columns = elevations.shape
    used_rows = rows - rows % factor
    used_columns = columns - columns % factor
    trimmed = elevations[:used_rows, :used_columns]
    return trimmed.reshape(used_rows // factor, factor,
                           used_columns // factor, factor).mean(axis=(1, 3))


def support_height(shape, x, y, samples=None):
    """Terrain elevation at world point (x, y), or None outside the raster.

    Used to check that a robot's spawn or waypoint actually stands on the
    converted surface instead of floating over or sinking into it.
    """
    vertices, _ = heightfield_grid(shape, samples=samples)
    position = np.asarray(shape.get("position", [0.0, 0.0, 0.0]), dtype=float)
    orientation = np.asarray(shape.get("orientation", [1.0, 0.0, 0.0, 0.0]),
                             dtype=float)
    local = _inverse_rotate([x - position[0], y - position[1], 0.0], orientation)
    offsets = vertices[:, :2] - np.asarray(local[:2])[None, :]
    closest = int(np.argmin((offsets ** 2).sum(axis=1)))
    return float(vertices[closest, 2] + position[2])


def _inverse_rotate(vector, quaternion):
    w, x, y, z = quaternion
    return _rotate_vector(vector, (w, -x, -y, -z))


def _rotate_vector(vector, quaternion):
    w, x, y, z = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def write_obj(vertices, faces, path):
    """Write a triangle mesh as OBJ, for the backends that stage world meshes."""
    with open(path, "w") as handle:
        for vertex in vertices:
            handle.write("v %.6g %.6g %.6g\n" % tuple(vertex))
        for face in faces:
            handle.write("f %d %d %d\n" % (face[0] + 1, face[1] + 1, face[2] + 1))
    return path

    return _read_pnm(path)
