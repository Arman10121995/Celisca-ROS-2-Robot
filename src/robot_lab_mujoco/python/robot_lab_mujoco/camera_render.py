"""MuJoCo renderer for depth images.

MuJoCo renders off-screen into a multisampled buffer (vis.quality.offsamples,
4 by default).  Colour is resolved by averaging the samples, but depth comes
from one sample, 3/8 of a pixel from the pixel centre: a floor rendered
0.2 m below the lab's OAK-D camera fitted a principal-point row of 119.125
instead of 119.5, and the R7.2 depth replays saw the MuJoCo floor rise
1.1-5.2 mm with range.  Without multisampling the row fits 119.5 exactly
(scripts/depth_render_check.py, 2026-10-16).
"""


def single_sample_renderer(mujoco, model, height, width):
    """A ``mujoco.Renderer`` whose context has no multisampling."""
    samples = int(model.vis.quality.offsamples)
    model.vis.quality.offsamples = 0
    try:
        return mujoco.Renderer(model, height, width)
    finally:
        model.vis.quality.offsamples = samples
