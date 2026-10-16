"""MuJoCo depth images are centred on the pixels (2026-10-16).

A multisampled off-screen buffer gives depth from one sample, 3/8 px from
the pixel centre: the R7.2 floor replays rose 1.1-5.2 mm with range.  The
single-sample depth renderer puts the floor of a camera 0.2 m up at
principal-point row (h - 1) / 2.
"""
import math
import os

import numpy as np
import pytest


def _floor_row_fit(single_sample):
    os.environ.setdefault('MUJOCO_GL', 'egl')
    mujoco = pytest.importorskip('mujoco')
    from robot_lab_mujoco.camera_render import single_sample_renderer
    height, width, hfov, camera_height = 240, 320, 1.25, 0.2
    fovy = math.degrees(2 * math.atan(math.tan(hfov / 2) * height / width))
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="plane" size="30 30 0.1"/>'
        '<camera name="c" pos="0 0 %g" xyaxes="0 -1 0 0 0 1" fovy="%g"/></worldbody></mujoco>'
        % (camera_height, fovy))
    extent = model.stat.extent
    model.vis.map.znear, model.vis.map.zfar = 0.3 / extent, 100.0 / extent
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    try:
        renderer = (single_sample_renderer(mujoco, model, height, width) if single_sample
                    else mujoco.Renderer(model, height, width))
    except Exception as exc:   # no OpenGL context on this host
        pytest.skip(str(exc))
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera='c')
    depth = renderer.render().astype(float)
    renderer.close()
    rows, inverse = [], []
    for row in range(125, height):
        z = depth[row, 150:170]
        if np.all(np.isfinite(z)) and z.mean() < 25:
            rows.append(row)
            inverse.append(1.0 / np.median(z))
    (cy, fy), *_ = np.linalg.lstsq(np.column_stack([np.ones(len(rows)), np.array(inverse) * camera_height]),
                                   np.array(rows, dtype=float), rcond=None)
    return cy, fy, model


def test_single_sample_depth_is_centred_on_the_pixels():
    cy, fy, model = _floor_row_fit(True)
    assert cy == pytest.approx(119.5, abs=0.02)
    assert fy == pytest.approx(160.0 / math.tan(0.625), rel=1e-4)
    assert model.vis.quality.offsamples > 0   # restored for the colour renderer
