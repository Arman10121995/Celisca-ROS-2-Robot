"""Frame conventions shared by the PyBullet, MuJoCo and Isaac Sim bridges.

Gazebo publishes odometry, IMU and scan through its own plugins; the other
three backends publish them from simulator state, and each has to get the
same conventions right:

* quaternion order - ROS messages are (x, y, z, w); MuJoCo and Isaac Sim are
  scalar-first (w, x, y, z);
* the odometry twist is expressed in the child frame (``base_footprint``),
  which is how robot_localization fuses vx / vy / vyaw, while simulators
  report world-frame velocities;
* a sensor on a fixed link sits at that link's pose, with that link's
  heading, not at a guessed height above the base.

Quaternions here are scalar-first (w, x, y, z), like ``sdf_world``; frames
are ``(position, quaternion)`` pairs.  Pure Python, no ROS or simulator
imports.
"""
import math
import xml.etree.ElementTree as ET

from .sdf_world import compose, quaternion_from_rpy

IDENTITY = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))


def xyzw_from_wxyz(quaternion):
    """Scalar-first quaternion as ROS (x, y, z, w)."""
    w, x, y, z = (float(v) for v in list(quaternion)[:4])
    return [x, y, z, w]


def wxyz_from_xyzw(quaternion):
    """ROS (x, y, z, w) quaternion as scalar-first."""
    x, y, z, w = (float(v) for v in list(quaternion)[:4])
    return (w, x, y, z)


def conjugate(quaternion):
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def rotate(vector, quaternion):
    """Rotate *vector* by a unit scalar-first *quaternion*."""
    w, x, y, z = quaternion
    vx, vy, vz = (float(v) for v in vector)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def world_to_body(vector, quaternion):
    """Express a world-frame *vector* in the frame oriented by *quaternion*."""
    return rotate(vector, conjugate(quaternion))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def body_odometry(position, orientation_xyzw, linear_world, angular_world,
                  reported_offset=None):
    """Odometry of the base from a simulator's world-frame body state.

    Returns ``(position, orientation_xyzw, linear_body, angular_body)``.
    *reported_offset* is the pose of the body the simulator reports in the
    base's frame, when that body is not the base itself (Isaac Sim reports
    the articulation's root body, e.g. ``base_link`` rather than the URDF
    root ``base_footprint``).  Velocities come back in the base frame.
    """
    quaternion = wxyz_from_xyzw(orientation_xyzw)
    position = tuple(float(v) for v in position)
    linear_world = tuple(float(v) for v in linear_world)
    angular_world = tuple(float(v) for v in angular_world)
    if reported_offset is not None:
        base_position, quaternion = compose((position, quaternion),
                                            invert(reported_offset))
        lever = tuple(b - p for b, p in zip(base_position, position))
        linear_world = tuple(v + c for v, c in
                             zip(linear_world, cross(angular_world, lever)))
        position = base_position
    return (list(position), xyzw_from_wxyz(quaternion),
            list(world_to_body(linear_world, quaternion)),
            list(world_to_body(angular_world, quaternion)))


def yaw_of(quaternion):
    """Heading in radians of a scalar-first quaternion."""
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def invert(frame):
    position, quaternion = frame
    inverse = conjugate(quaternion)
    return (tuple(-v for v in rotate(position, inverse)), inverse)


def _floats(text, count):
    values = [float(v) for v in (text or "").split()]
    return (values + [0.0] * count)[:count]


def urdf_link_frames(urdf_text):
    """Pose of every link in the URDF root link's frame at zero joint angles.

    Returns ``(frames, root_link)``; *frames* maps link name to
    ``(position, quaternion)``.  Joint ``<origin>`` roll/pitch/yaw uses the
    same fixed-axis convention as SDF.
    """
    root = ET.fromstring(urdf_text)
    links = [link.get("name") for link in root.findall("link")]
    parent_of = {}
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            continue
        origin = joint.find("origin")
        xyz = _floats(origin.get("xyz") if origin is not None else "", 3)
        rpy = _floats(origin.get("rpy") if origin is not None else "", 3)
        parent_of[child.get("link")] = (
            parent.get("link"), (tuple(xyz), quaternion_from_rpy(*rpy)))

    frames = {}

    def frame_of(link, depth):
        if link in frames:
            return frames[link]
        if link not in parent_of or depth > len(parent_of):
            result = IDENTITY  # a root, or a malformed joint cycle
        else:
            parent, origin = parent_of[link]
            result = compose(frame_of(parent, depth + 1), origin)
        frames[link] = result
        return result

    for link in links:
        frame_of(link, 0)
    roots = [link for link in links if link not in parent_of]
    return frames, (roots[0] if roots else None)


def relative_frame(frames, link, base):
    """Pose of *link* in *base*'s frame, or None if either link is unknown."""
    if link not in frames or base not in frames:
        return None
    return compose(invert(frames[base]), frames[link])


def offset_from_root(urdf_text, link):
    """Pose of *link* in the URDF root link's frame, or None."""
    try:
        frames, root = urdf_link_frames(urdf_text)
    except ET.ParseError:
        return None
    return relative_frame(frames, link, root) if root else None


def mounted_sensor_offsets(urdf_text, sensor_link):
    """Offset of *sensor_link* from each link it is rigidly carried by.

    A backend reports the pose of whichever link it treats as the base (the
    URDF root for MuJoCo and PyBullet; the articulation's root body for
    Isaac Sim, which may be a different link).  Offsets are returned for the
    sensor link's ancestors so the backend can pick the one matching its
    base.  Empty when the link is not in the description.
    """
    try:
        frames, _root = urdf_link_frames(urdf_text)
    except ET.ParseError:
        return {}
    if sensor_link not in frames:
        return {}
    return {base: relative_frame(frames, sensor_link, base) for base in frames
            if relative_frame(frames, sensor_link, base) is not None}


def mounted_pose(base_position, base_quaternion, offset):
    """World pose of a sensor rigidly mounted at *offset* from the base."""
    return compose((tuple(float(v) for v in base_position),
                    tuple(float(v) for v in base_quaternion)), offset)
