#!/usr/bin/env python3
"""Render a known scene with the MuJoCo and PyBullet camera paths, check depth.

The R7.2 depth replays left MuJoCo and PyBullet floors 1.5 and 2 mm per
metre high after the principal-point fix, where Gazebo and Isaac read 0.
That could be the camera pose the robot pipeline gives the renderer, or the
renderer itself.  This places the lab's OAK-D camera model (320 x 240, its
field of view and clip planes) exactly 0.2 m above a floor, looking along
+x at a wall, renders through each backend's own path (MuJoCo's Renderer
with the spawner's clip settings; pybullet_spawner._render_rgbd) and
reports, against the exact geometry:

- the wall's depth error at the image centre, for walls 0.5-4 m away;
- the floor height recovered with the backend's published pinhole
  (camera_model.intrinsics: pixel centres, or TinyRenderer's axis for
  PyBullet), per depth bin, and the principal-point row that would zero it.

MuJoCo renders depth without multisampling (robot_lab_mujoco.camera_render);
``--multisampled`` shows the default buffer's 3/8 px offset.

  python3 scripts/depth_render_check.py [--backend mujoco|pybullet|both]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for package in ('robot_lab_utils', 'robot_lab_pybullet/python', 'robot_lab_mujoco/python'):
    sys.path.insert(0, os.path.join(HERE, '..', 'src', package))
os.environ.setdefault('MUJOCO_GL', 'egl')

from robot_lab_utils import camera_model  # noqa: E402

HEIGHT = 0.2
SINGLE_SAMPLE = True


def camera():
    oakd = camera_model.OAKD
    return dict(width=int(oakd.get('width', 320)), height=int(oakd.get('height', 240)),
                horizontal_fov=float(oakd['horizontal_fov']), near=float(oakd['near']), far=float(oakd['far']))


def render_mujoco(wall, cam):
    import mujoco
    fovy = math.degrees(camera_model.vertical_fov(cam['horizontal_fov'], cam['width'], cam['height']))
    xml = f"""<mujoco><worldbody>
      <geom type="plane" size="20 20 0.1" rgba="0.5 0.5 0.5 1"/>
      <geom type="box" pos="{wall + 0.05} 0 1" size="0.05 10 2" rgba="0.6 0.6 0.6 1"/>
      <camera name="c" pos="0 0 {HEIGHT}" xyaxes="0 -1 0 0 0 1" fovy="{fovy}"/>
    </worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    extent = max(float(model.stat.extent), 1e-6)
    model.vis.map.znear = cam['near'] / extent
    model.vis.map.zfar = cam['far'] / extent
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    from robot_lab_mujoco.camera_render import single_sample_renderer
    renderer = single_sample_renderer(mujoco, model, cam['height'], cam['width']) if SINGLE_SAMPLE \
        else mujoco.Renderer(model, cam['height'], cam['width'])
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera='c')
    depth = renderer.render().astype(np.float64)
    renderer.close()
    return depth


def render_pybullet(wall, cam):
    import pybullet as p
    from robot_lab_pybullet.pybullet_spawner import _render_rgbd
    client = p.connect(p.DIRECT)
    plane = p.createCollisionShape(p.GEOM_BOX, halfExtents=[20, 20, 0.005], physicsClientId=client)
    p.createMultiBody(0, plane, p.createVisualShape(p.GEOM_BOX, halfExtents=[20, 20, 0.005],
                                                    physicsClientId=client),
                      basePosition=[0, 0, -0.005], physicsClientId=client)
    box = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 10, 2], physicsClientId=client)
    p.createMultiBody(0, -1, box, basePosition=[wall + 0.05, 0, 1], physicsClientId=client)
    _, depth = _render_rgbd([0, 0, HEIGHT], [0, 0, 0, 1], cam, client=client)
    p.disconnect(client)
    return depth.astype(np.float64)


def analyse(depth, wall, cam, convention):
    w, h = cam['width'], cam['height']
    _, fy, _, cy = camera_model.intrinsics(w, h, cam['horizontal_fov'], convention)
    centre = depth[h // 2 - 1:h // 2 + 1, w // 2 - 1:w // 2 + 1].mean()
    rows = np.arange(h)[:, None] * np.ones((1, w))
    finite = np.isfinite(depth)
    # Floor pixels: below the horizon and short of the wall.
    floor = finite & (rows > cy + 2) & (depth < wall - 0.05)
    z = depth[floor]
    height = HEIGHT - (rows[floor] - cy) * z / fy
    bins = {}
    for lo, hi in ((0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0)):
        sel = (z >= lo) & (z < hi)
        if sel.sum() > 20:
            bins['%g-%g m' % (lo, hi)] = round(float(np.median(height[sel])) * 1000, 2)
    # The row offset that would zero the floor: height = H - (v - cy') z / fy.
    if len(z) > 50:
        offset = float(np.median((height) * fy / z))
    else:
        offset = None
    return {'wall_m': wall, 'centre_depth_error_mm': round((centre - wall) * 1000, 2),
            'floor_height_mm_by_depth': bins,
            'cy_correction_px': round(offset, 3) if offset is not None else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='both', choices=('mujoco', 'pybullet', 'both'))
    parser.add_argument('--multisampled', action='store_true',
                        help="MuJoCo's default multisampled depth (before the fix)")
    args = parser.parse_args()
    global SINGLE_SAMPLE
    SINGLE_SAMPLE = not args.multisampled
    cam = camera()
    report = {'camera': cam}
    backends = ('mujoco', 'pybullet') if args.backend == 'both' else (args.backend,)
    for backend in backends:
        render = render_mujoco if backend == 'mujoco' else render_pybullet
        convention = camera_model.TINY_RENDERER if backend == 'pybullet' else camera_model.PIXEL_CENTRES
        report[backend] = [analyse(render(wall, cam), wall, cam, convention) for wall in (0.5, 1.0, 2.0, 3.0, 4.0)]
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    sys.exit(main())
