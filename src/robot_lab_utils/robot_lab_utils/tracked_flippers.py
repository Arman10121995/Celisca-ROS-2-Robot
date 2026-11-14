"""Flipper-aware terrain policy for the articulated tracked vehicle (ROADMAP R10.7).

Sensing is the robot's own: the forward RGB-D depth image gives the terrain
height ahead in the track corridor, and the IMU gives pitch and roll. No
world geometry or simulator pose is read. The classic step sequence is:

- travel:   both flipper pairs at their source rest (29° above horizontal);
- approach: a rise ahead; front flippers lift (at least to rest) so their
            tips meet the edge and act as a ramp; rear flippers already press
            down (the source flippers slew at 0.2 rad/s, so the rear pair must
            be in place before the hull rears up);
- climb:    the hull pitches up; front flippers stay up so the tracks ride
            the edge, rear flippers press down to lift the tail, push and
            limit the rear-up (the source flipper PID is capped at 4 Nm, too
            weak to lift the hull with the front pair: measured 2026-11-14);
- descend:  a drop ahead or nose-down pitch; front flippers reach down first,
            rear flippers lift;
- hold:     roll beyond 0.35 rad; flippers hold and the drive is stopped.

Flipper elevation θ (above horizontal, toward the flipper's own end) maps
to the source joints as q_front = 0.5 − θ and q_rear = θ − 0.5 (measured
from the source SDF poses and axes), clipped to the ±0.8 rad joint limits.
"""
import math

import numpy as np

FLIPPER_LENGTH_M = .33
PIVOT_HEIGHT_M = .09          # front/rear wheel centre above ground
REST_RAD = .5                 # joint 0: 29° above horizontal
PRESS_RAD = -.15              # tip below the pivot: presses on the ground/tread
RISE_M, DROP_M = .05, .06
CLIMB_PITCH, LEVEL_PITCH = .12, .05
MAX_ROLL = .35
LEVEL_HOLD_S = 1.


def joints(front, rear):
    """Elevations (front, rear) to source joint targets in FLIPPER_JOINTS order (fl, rl, fr, rr)."""
    qf = max(-.8, min(.8, .5 - front))
    qr = max(-.8, min(.8, rear - .5))
    return [qf, qr, qf, qr]


def terrain_ahead(depth, hfov, camera_xyz, ground_z, corridor=.25, near=.55, far=1.4, stride=2):
    """Height of the terrain ahead (base frame) from a level forward depth camera.

    Returns rise (90th-percentile height above the ground line), drop (True
    when the corridor sees ground clearly below the wheels' line) and the
    number of supporting points, or None without enough points.
    """
    depth = np.asarray(depth, dtype=float)
    if depth.ndim != 2:
        raise ValueError('A single-channel depth image is required')
    h, w = depth.shape
    fx = (w / 2) / math.tan(hfov / 2)
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z > .05)
    du, dv = (u - (w - 1) / 2) / fx, (v - (h - 1) / 2) / fx
    x_c, y_c = du * z, dv * z
    X, Y, Z = z + camera_xyz[0], -x_c + camera_xyz[1], camera_xyz[2] - y_c
    band = valid & (np.abs(Y) < corridor) & (X > near) & (X < far)
    # Where each downward ray would meet flat ground: a drop shows as floor that
    # should be in the band but reads farther (or lower) than that.
    with np.errstate(divide='ignore', invalid='ignore'):
        flat = np.where(dv > 1e-6, (camera_xyz[2] - ground_z) / dv, np.inf)
    expected = (dv > 1e-6) & (np.abs(-du * flat) < corridor) & (flat + camera_xyz[0] > near) & (flat + camera_xyz[0] < far)
    missing = expected & (~valid | (z > flat + .3))
    drop = bool(expected.sum() >= 30 and missing.sum() > .3 * expected.sum())
    if band.sum() < 30:
        return dict(rise_m=0., drop=drop, points=int(band.sum()), nearest_rise_m=None) if drop else None
    heights = Z[band] - ground_z
    drop = drop or bool(np.percentile(heights, 10) < -DROP_M)
    return dict(rise_m=float(np.percentile(heights, 90)), drop=drop,
                points=int(band.sum()), nearest_rise_m=float(X[band][heights > RISE_M].min()) if (heights > RISE_M).any() else None)


def edge_elevation(rise):
    """Front flipper elevation that puts the tip just above a step edge."""
    lift = (rise + .04 - PIVOT_HEIGHT_M) / FLIPPER_LENGTH_M
    return max(0., min(1.2, math.asin(max(-1., min(1., lift)))))


class FlipperPolicy:
    def __init__(self):
        self.state, self.since, self.level_since = 'travel', 0., None
        self.elevations = (REST_RAD, REST_RAD)

    def update(self, terrain, pitch, roll, now):
        """terrain: terrain_ahead() result or None; pitch > 0 is nose up."""
        if not all(map(math.isfinite, (pitch, roll))):
            raise ValueError('Finite IMU attitude required')
        previous = self.state
        rise = terrain['rise_m'] if terrain else 0.
        drop = bool(terrain and terrain['drop'])
        if abs(roll) > MAX_ROLL:
            self.state = 'hold'
        elif self.state == 'hold':
            if abs(roll) < MAX_ROLL * .7:
                self.state = 'travel'
        elif pitch < -CLIMB_PITCH or (drop and abs(pitch) < LEVEL_PITCH and self.state in ('travel', 'descend')):
            self.state = 'descend'
        elif pitch > CLIMB_PITCH:
            self.state = 'climb'
        elif self.state in ('climb', 'descend'):
            level = abs(pitch) < LEVEL_PITCH and rise < RISE_M and not drop
            self.level_since = (self.level_since or now) if level else None
            if self.level_since is not None and now - self.level_since >= LEVEL_HOLD_S:
                self.state = 'approach' if rise >= RISE_M else 'travel'
        elif rise >= RISE_M:
            self.state = 'approach'
        elif self.state == 'approach' and rise < RISE_M * .6:
            self.state = 'travel'
        if self.state != previous:
            self.since, self.level_since = now, None
        if self.state == 'approach':
            self.elevations = (max(REST_RAD, edge_elevation(rise)), PRESS_RAD)
        elif self.state == 'climb':
            self.elevations = (REST_RAD, PRESS_RAD)
        elif self.state == 'descend':
            self.elevations = (PRESS_RAD, REST_RAD)
        elif self.state == 'travel':
            self.elevations = (REST_RAD, REST_RAD)
        return dict(state=self.state, elevations=self.elevations, targets=joints(*self.elevations),
                    stop_drive=self.state == 'hold', rise_m=round(rise, 3), drop=drop)
