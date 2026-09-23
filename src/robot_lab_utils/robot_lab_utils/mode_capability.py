"""Which bringup modes a robot can really run in a given simulator.

Shared by the bringup launch (which refuses an impossible selection) and the
GUI (which greys it out with the reason), so both give the same answer.

A mode needs sensors and, for anything that moves the robot, a base that
follows /cmd_vel:

* ``lidar_2d`` - Gazebo renders the LiDAR declared in the description; the
  PyBullet, MuJoCo and Isaac bridges ray-cast a scan for every robot (from
  its laser link, else 0.12 m above its root link).
* ``rgbd_camera`` - declared camera link; every backend renders it.
* ``velocity_base`` - the robot drives on /cmd_vel (the differential-drive
  bases).  Legged and humanoid robots have no velocity-tracking gait in this
  lab yet, so they can localize where they stand but not map by moving or
  navigate.

Robot features come from robot_lab_robots/config/robots.yaml.
"""

# Features the simulator bridges provide for any robot.
BRIDGE_FEATURES = {
    "gazebo": (),
    "pybullet": ("lidar_2d",),
    "mujoco": ("lidar_2d",),
    "isaac": ("lidar_2d",),
}

# What each mode needs from the robot (sim_modes.yaml may add more).
MODE_ROBOT_FEATURES = {
    "display": (),
    "loc": ("lidar_2d",),
    "slam": ("lidar_2d", "velocity_base"),
    "3d_slam": ("rgbd_camera", "velocity_base"),
    "nav": ("lidar_2d", "velocity_base"),
}

FEATURE_LABELS = {
    "lidar_2d": "a 2D LiDAR (Gazebo uses the one in the description; the "
                "PyBullet, MuJoCo and Isaac bridges cast one for any robot)",
    "rgbd_camera": "an RGB-D camera link",
    "velocity_base": "a base that drives on /cmd_vel (no walking gait yet)",
}


def robot_features(robot_config, simulator=None):
    """Features of a robot profile, plus what *simulator* adds for any robot."""
    features = set((robot_config or {}).get("features", []))
    if simulator:
        features.update(BRIDGE_FEATURES.get(simulator, ()))
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
    return [feature for feature in required_features(mode, mode_config)
            if feature not in have]


def describe_missing(missing):
    """Human-readable reason for a list of missing features."""
    return "robot lacks " + "; ".join(FEATURE_LABELS.get(f, f) for f in missing)
