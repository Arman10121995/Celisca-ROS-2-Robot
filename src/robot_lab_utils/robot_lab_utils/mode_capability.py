"""Which bringup modes a robot can really run in a given simulator.

Shared by the bringup launch (which refuses an impossible selection) and the
GUI (which greys it out with the reason), so both give the same answer.

A mode needs sensors and, for anything that moves the robot, a base that
follows /cmd_vel:

* ``lidar_2d`` - Gazebo renders the LiDAR declared in the description; the
  PyBullet, MuJoCo and Isaac bridges ray-cast a scan for every robot (from
  its laser link, else 0.12 m above its root link).
* ``rgbd_camera`` - declared camera link; every backend renders it.
* ``velocity_base`` - the robot follows /cmd_vel: the wheeled bases, and a
  legged or humanoid robot whose model-matched policy walks it (below).
  Without one, a legged robot can localize where it stands but not map by
  moving or navigate.
* ``stands`` - the robot stays upright without a controller.  Wheeled bases
  do; the PyBullet and MuJoCo bridges hold a legged or humanoid robot's
  joints at its spawn pose.  Isaac's hold does not keep them in place
  (measured 2026-09-23: dogs drifted up to 1.9 m, one robot was thrown
  across the map, H1-2 produced NaN poses), so there they are display-only
  unless they have a learned policy.
* ``legged_policy`` - a model-matched learned locomotion policy is
  provisioned (robot_lab_adapter.legged_policy).  Its PD law runs inside
  the MuJoCo and Isaac physics steps, so in Isaac it provides the stance
  the pose hold cannot; in Gazebo an effort joint_trajectory_controller
  carries it for descriptions with a ``ros2_control`` block (implemented
  2026-10-12; validation per robot/backend pending).  The policy walks on
  /cmd_vel and leg kinematic odometry replaces wheel odometry, so such a
  robot also counts as a ``velocity_base`` there (2026-10-13, experimental).

Robot features come from robot_lab_robots/config/robots.yaml.
"""

# Features the simulator bridges provide for any robot.
BRIDGE_FEATURES = {
    "gazebo": (),
    "pybullet": ("lidar_2d", "stands"),
    "mujoco": ("lidar_2d", "stands"),
    "isaac": ("lidar_2d",),
}

# Backends whose in-step PD bridge runs a robot's learned policy; there a
# robot with ``legged_policy`` stands on it.
POLICY_STANCE_SIMULATORS = ("mujoco", "isaac")

# What each mode needs from the robot (sim_modes.yaml may add more).
MODE_ROBOT_FEATURES = {
    "display": (),
    "loc": ("lidar_2d", "stands"),
    "slam": ("lidar_2d", "velocity_base"),
    "3d_slam": ("rgbd_camera", "velocity_base"),
    "nav": ("lidar_2d", "velocity_base"),
    "flight": ("flight_controller",),
    "marine": ("marine_dynamics",),
    "gait": ("source_gait",),
}

FEATURE_LABELS = {
    "lidar_2d": "a 2D LiDAR (Gazebo uses the one in the description; the "
                "PyBullet, MuJoCo and Isaac bridges cast one for any robot)",
    "rgbd_camera": "an RGB-D camera link",
    "velocity_base": "a base that follows /cmd_vel (wheels, or a provisioned "
                     "walking policy in MuJoCo, Isaac or Gazebo)",
    "stands": "a stable stance without a controller (the PyBullet and "
              "MuJoCo bridges hold its joints; Isaac's hold does not)",
    "car_steering": "a car-like base that cannot turn on the spot",
    "holonomic_base": "a base that translates sideways as well as forward",
    "flight_controller": "a live PX4 flight controller",
    "marine_dynamics": "a native water-force and bounded thruster controller",
    "source_gait": "a source-bound multi-leg IK/gait and bounded joint controller",
    "legged_policy": "a provisioned model-matched learned locomotion policy",
}


def robot_features(robot_config, simulator=None):
    """Features of a robot profile, plus what *simulator* adds for any robot."""
    features = set((robot_config or {}).get("features", []))
    if "velocity_base" in features:
        features.add("stands")  # a wheeled base rests on its wheels
    if simulator:
        features.update(BRIDGE_FEATURES.get(simulator, ()))
        if "legged_policy" in features and (
                simulator in POLICY_STANCE_SIMULATORS
                or (simulator == "gazebo" and "ros2_control" in features)):
            features.add("stands")
            features.add("velocity_base")  # the policy walks on /cmd_vel
    return features


def required_features(mode, mode_config=None):
    """Robot features *mode* needs: the built-in table plus sim_modes.yaml."""
    needed = list(MODE_ROBOT_FEATURES.get(mode, ()))
    for feature in (mode_config or {}).get("required_features", []) or []:
        if feature not in needed:
            needed.append(feature)
    return needed


def missing_features(robot_config, mode, simulator=None, mode_config=None):
    """Required features the robot lacks in *simulator* (all simulators if None)."""
    have = robot_features(robot_config, simulator)
    missing = [feature for feature in required_features(mode, mode_config)
               if feature not in have]
    supported = (robot_config or {}).get('supported_simulators', [])
    if simulator and supported and simulator not in supported:
        missing.append('support for simulator '+simulator+' (available: '+', '.join(supported)+')')
    # Imported robots may qualify one backend before another. Do not expose
    # unmeasured mode/backend combinations merely because sensors exist.
    mode_matrix = (robot_config or {}).get('supported_modes_by_simulator')
    if simulator and mode_matrix is not None and mode not in mode_matrix.get(simulator, []):
        missing.append('qualified '+mode+' support in '+simulator)
    return missing


def describe_missing(missing):
    """Human-readable reason for a list of missing features."""
    return "robot lacks " + "; ".join(FEATURE_LABELS.get(f, f) for f in missing)
