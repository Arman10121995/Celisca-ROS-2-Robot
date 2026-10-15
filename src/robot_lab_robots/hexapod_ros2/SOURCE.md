# Source hexapod adapter

Original CAD model and KDL conversion:
[ariegweomamerie/hexapod_ros2](https://github.com/ariegweomamerie/hexapod_ros2),
commit `b146f84d58123caabfcea8fe2a5e2a353acc33de`, MIT License retained here.
The original `description/urdf` and `description/meshes` are preserved. The
unchanged KDL conversion and notice live in `robot_lab_utils/vendor`.
Upstream runtime claims are not Robot Lab mission evidence.

`scripts/generate_gait_assets.py` expands the source into the normal installed
URDF. It replaces simulator/ros2_control declarations, resolves package assets
and reverses the identity base/footprint joint so massive `base_link` is root:
MuJoCo otherwise loses trunk inertia by fusing the massless root into world.
Original link mass/inertia, joint axes/limits and collision shapes remain.
The head holds source neutral. The whole plant is rotated at spawn to align
the original −Y front with the requested heading, preserving source axes.

This is an **authored experimental adapter**, not the upstream Jazzy/Harmonic
runtime. Source FK/IK, conservative strides, tripod/ripple/wave phases and
MuJoCo in-step PD drive joints (160 Nm/rad, 5 Nms/rad, torque capped at 20 Nm
below the source 100 Nm declarations; authored reflected inertia 0.01 kg m²).
Geometric IK is not a learned walking policy. Actual world contact forces,
body tilt and joint feedback are separate from planned support phases.
Joint servos maintain posture; the body stays free in physics.

MuJoCo Display/Gait is the initial scope. Independent estimation, saved
SLAM/navigation, other backends, terrain/recovery and broad gait/contact/
loss/reset qualification remain pending.
