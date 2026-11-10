"""sensor_fusion.ComplementaryImu axis convention (REP-103: x forward, y left, z up)."""
import math

from robot_lab_algorithms.sensor_fusion import ComplementaryImu

G = 9.81


def gravity_in_body(roll, pitch):
    """What a resting accelerometer reads (specific force) at roll/pitch."""
    return (-G * math.sin(pitch), G * math.sin(roll) * math.cos(pitch), G * math.cos(roll) * math.cos(pitch))


def test_static_tilt_converges_to_the_accelerometer():
    f = ComplementaryImu(alpha=0.9)
    ax, ay, az = gravity_in_body(0.3, -0.2)
    for _ in range(500):
        pitch, roll = f.update(ax, ay, az, 0.0, 0.0, 0.01)
    assert abs(roll - 0.3) < 0.01 and abs(pitch + 0.2) < 0.01


def test_roll_follows_gx_and_pitch_follows_gy():
    # Rotating about x (roll rate gx) while the accelerometer still reads level:
    # over a short time the gyro dominates, so roll rises and pitch stays put.
    f = ComplementaryImu(alpha=0.999)
    for _ in range(50):
        pitch, roll = f.update(0.0, 0.0, G, 0.5, 0.0, 0.01)
    assert roll > 0.2 and abs(pitch) < 0.01
    g = ComplementaryImu(alpha=0.999)
    for _ in range(50):
        pitch, roll = g.update(0.0, 0.0, G, 0.0, 0.5, 0.01)
    assert pitch > 0.2 and abs(roll) < 0.01
