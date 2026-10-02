"""PX4 NED/FRD to ROS ENU/FLU conversions, independent of ROS/MAVLink."""
import math


def ned_to_enu(x, y, z):
    return y, x, -z


def enu_to_ned(x, y, z):
    return y, x, -z


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def attitude_enu_flu(roll, pitch, yaw):
    """Apply R_ENU_NED * R_NED_FRD * R_FRD_FLU; return xyzw."""
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    q = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
         cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    root_half = math.sqrt(0.5)
    return _multiply(_multiply((root_half, root_half, 0, 0), q), (1, 0, 0, 0))


def yaw_enu(yaw_ned):
    return math.atan2(math.cos(yaw_ned), math.sin(yaw_ned))
