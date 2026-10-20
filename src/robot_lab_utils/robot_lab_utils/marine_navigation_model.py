"""Marine sensor equations and pose-free dead reckoning.

The simulator sensor model may observe the physical plant; the estimator takes
only timestamped velocity, pressure and IMU observations plus an initial prior.
Neither pose truth nor ideal controller pressure is an estimator correction.
"""
import math
import numpy as np
from .marine_model import rotation


JANUS = np.array([[.5, 0., -math.sqrt(.75)], [-.5, 0., -math.sqrt(.75)],
                  [0., .5, -math.sqrt(.75)], [0., -.5, -math.sqrt(.75)]])


def bottom_ranges(scene, position, orientation, maximum=10.):
    """Trace four bottom-lock beams against selected-world collision surfaces."""
    origin = np.asarray(position, dtype=float) + rotation(orientation) @ np.array([0., 0., -.2])
    result = []
    for beam in JANUS @ rotation(orientation).T:
        distance = .1
        for _ in range(256):
            clearance = scene.clearance(origin + distance * beam)
            if clearance <= .01:
                break
            distance += max(.005, min(.5, .8 * clearance))
            if distance > maximum:
                distance = math.inf
                break
        else:
            distance = math.inf
        result.append(float(distance))
    return result


def dvl_velocity(body_velocity, rng, standard_deviation=.005):
    beams = JANUS @ np.asarray(body_velocity) + rng.normal(0., standard_deviation, 4)
    value = np.linalg.lstsq(JANUS, beams, rcond=None)[0]
    covariance = standard_deviation ** 2 * np.linalg.inv(JANUS.T @ JANUS)
    return value, covariance, beams


class MarineOdometry:
    def __init__(self, position, density=1025.):
        self.position = np.asarray(position, dtype=float).copy()
        if self.position.shape != (3,) or not np.isfinite(self.position).all():
            raise ValueError('A finite initial position prior is required')
        self.density = float(density)
        if not math.isfinite(self.density) or self.density <= 0:
            raise ValueError('Positive water density required')
        self.covariance = np.diag([.02 ** 2, .02 ** 2, .01 ** 2])
        self.last_stamp = None
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)

    def update(self, stamp, body_velocity, velocity_covariance, orientation, pressure, pressure_variance, surface=0.):
        velocity, covariance, orientation = np.asarray(body_velocity), np.asarray(velocity_covariance), np.asarray(orientation)
        if (velocity.shape != (3,) or covariance.shape != (3, 3) or orientation.shape != (3,)
                or not np.isfinite([stamp, pressure, pressure_variance, surface]).all()
                or not np.isfinite(velocity).all() or not np.isfinite(covariance).all()
                or not np.isfinite(orientation).all() or pressure_variance <= 0
                or np.min(np.linalg.eigvalsh(covariance)) < -1e-10):
            raise ValueError('Finite velocity/IMU/pressure and nonnegative covariance required')
        if self.last_stamp is not None and not 0 < stamp - self.last_stamp <= .5:
            raise ValueError('Sensor timestamps repeated, rewound or missing; reinitialize explicitly')
        dt = 0. if self.last_stamp is None else stamp - self.last_stamp
        world_rotation = rotation(orientation)
        world_velocity = world_rotation @ velocity
        self.position += .5 * (self.velocity + world_velocity) * dt
        self.covariance += world_rotation @ covariance @ world_rotation.T * dt ** 2 + np.eye(3) * .0001 * dt
        z = surface - (pressure - 101325.) / (self.density * 9.81)
        variance = pressure_variance / (self.density * 9.81) ** 2
        gain = self.covariance[:, 2] / (self.covariance[2, 2] + variance)
        self.position += gain * (z - self.position[2])
        self.covariance -= np.outer(gain, self.covariance[2, :])
        self.velocity, self.orientation, self.last_stamp = world_velocity, orientation.copy(), stamp
        return self.position.copy()


def waypoint_command(position, orientation, velocity, target, auv=False):
    """Bounded estimated-pose guidance; actual forces remain in native controllers."""
    error = np.asarray(target) - np.asarray(position)
    distance = float(np.linalg.norm(error))
    yaw = float(orientation[2])
    if auv:
        heading = yaw + math.pi  # Source Tethys forward is -X.
        desired = math.atan2(error[1], error[0])
        yaw_error = math.atan2(math.sin(desired - heading), math.cos(desired - heading))
        return (.3, 0., max(-.08, min(.08, error[2] * .4)), max(-.12, min(.12, yaw_error * .5))), distance
    world = np.clip(error * .5, -.2, .2)
    body = rotation((0., 0., yaw)).T @ world
    return (float(body[0]), float(body[1]), float(world[2]), 0.), distance
