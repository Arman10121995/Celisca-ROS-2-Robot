#!/usr/bin/env python3
"""Figures for the algorithm tutorials (docs/tutorials), drawn by running the
project's own implementations in robot_lab_algorithms on small synthetic
problems.

    python3 docs/learn/figures/algorithm_figures.py              # all
    python3 docs/learn/figures/algorithm_figures.py planners pid  # some
"""

import math
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import patches  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'src' / 'robot_lab_algorithms'))

from robot_lab_algorithms import control, global_planning, local_planning, perception  # noqa: E402
from robot_lab_algorithms import sensor_fusion, state_estimation  # noqa: E402

plt.rcParams.update({'font.size': 10, 'axes.titlesize': 11, 'figure.dpi': 110, 'savefig.bbox': 'tight'})
COL = {'path': '#e8590c', 'truth': '#212529', 'meas': '#adb5bd', 'a': '#1c7ed6', 'b': '#2f9e44', 'c': '#be4bdb',
       'obst': '#495057', 'tree': '#a5d8ff'}


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)
    print('wrote', name)


# ---------------------------------------------------------------------------
# Global planning
# ---------------------------------------------------------------------------

OBSTACLES = [(2.0, 0.0, 3.0, 6.5), (5.0, 3.5, 6.0, 10.0), (7.0, 1.0, 9.5, 2.0), (7.5, 5.5, 8.5, 8.5)]


def is_free(x, y, margin=0.15):
    return 0 <= x <= 10 and 0 <= y <= 10 and not any(
        x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin for x0, y0, x1, y1 in OBSTACLES)


def path_length(path):
    return sum(math.dist(p, q) for p, q in zip(path, path[1:])) if path and len(path) > 1 else float('nan')


def fig_planners():
    start, goal, bounds = (1.0, 1.0), (9.0, 9.0), (0.0, 0.0, 10.0, 10.0)
    planners = [('Dijkstra', global_planning.DijkstraPlanner(resolution=0.2)),
                ('A*', global_planning.AStarPlanner(resolution=0.2)),
                ('PRM (300 samples)', global_planning.PRMPlanner(num_samples=300, max_connection_distance=2.0)),
                ('RRT', global_planning.RRTPlanner(step=0.5, max_iter=4000, goal_tol=0.5)),
                ('RRT*', global_planning.RRTStarPlanner(step=0.5, max_iter=3000, goal_tol=0.5, radius=1.5))]
    fig, axes = plt.subplots(1, len(planners), figsize=(17, 3.9))
    for ax, (name, planner) in zip(axes, planners):
        t0 = time.time()
        try:
            path = planner.plan(start, goal, is_free, bounds)
        except Exception as exc:  # show the failure instead of hiding it
            path, name = None, '%s (error: %s)' % (name, type(exc).__name__)
        dt = time.time() - t0
        for x0, y0, x1, y1 in OBSTACLES:
            ax.add_patch(patches.Rectangle((x0, y0), x1 - x0, y1 - y0, color=COL['obst']))
        if path:
            ax.plot(*zip(*path), color=COL['path'], lw=2.5)
        ax.plot(*start, 'o', color=COL['a'], ms=9)
        ax.plot(*goal, '*', color='#f59f00', ms=14, mec='k')
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title('%s\nlength %.1f m, %.2f s' % (name, path_length(path), dt) if path else name + '\nno path')
    fig.suptitle('Five global planners from robot_lab_algorithms.global_planning on the same problem', y=1.04)
    save(fig, 'planners_compare.png')


# ---------------------------------------------------------------------------
# Local planning
# ---------------------------------------------------------------------------

def fig_dwa():
    """Dynamic window: sample (v, w), roll each forward, score it."""
    obstacles = np.array([[1.4, 0.15], [1.4, 0.35], [1.4, -0.05], [1.0, -0.9], [2.4, 0.9]])
    goal = np.array([3.0, 0.4])
    fig, ax = plt.subplots(figsize=(8, 4.2))
    best, best_score = None, -math.inf
    for v in np.linspace(0.1, 0.5, 5):
        for w in np.linspace(-1.0, 1.0, 11):
            x = y = th = 0.0
            pts = [(x, y)]
            for _ in range(20):
                x += v * math.cos(th) * 0.1
                y += v * math.sin(th) * 0.1
                th += w * 0.1
                pts.append((x, y))
            pts = np.array(pts)
            clearance = np.min(np.hypot(*(pts[:, None, :] - obstacles[None]).transpose(2, 0, 1)))
            if clearance < 0.15:
                ax.plot(*pts.T, color='#ffc9c9', lw=1)
                continue
            score = -np.linalg.norm(pts[-1] - goal) + 0.5 * min(clearance, 0.6) + 0.3 * v
            ax.plot(*pts.T, color=plt.cm.viridis(min(1, max(0, (score + 3) / 2.5))), lw=1)
            if score > best_score:
                best, best_score = pts, score
    ax.plot(*best.T, color=COL['path'], lw=3, label='chosen trajectory')
    ax.plot(*obstacles.T, 's', color=COL['obst'], ms=10, label='obstacles (from the LiDAR)')
    ax.plot(*goal, '*', color='#f59f00', ms=15, mec='k', label='goal')
    ax.add_patch(patches.Circle((0, 0), 0.12, color=COL['a']))
    ax.plot([], [], color='#ffc9c9', label='would collide: rejected')
    ax.set_aspect('equal')
    ax.legend(fontsize=8, loc='lower left')
    lp = local_planning.DWBLocalPlanner()
    ranges = [3.0] * 360
    v, w = lp.compute_velocity_commands(-math.pi, math.radians(1), ranges, 0.2, 0.0, 0.0, 3.0, 0.4, 0.0)
    ax.set_title('Dynamic Window Approach: try many (v, w), simulate 2 s ahead, score each\n'
                 '(colour = score; DWBLocalPlanner on an open scan chose v = %.2f m/s, w = %.2f rad/s)' % (v, w))
    save(fig, 'dwa_trajectories.png')


# ---------------------------------------------------------------------------
# State estimation and fusion
# ---------------------------------------------------------------------------

def fig_kalman():
    rng = np.random.default_rng(1)
    dt, n = 0.1, 120
    t = np.arange(n) * dt
    truth = np.column_stack((0.5 * t, 1.5 * np.sin(0.4 * t)))
    meas = truth + rng.normal(0, 0.35, truth.shape)
    kf = state_estimation.LinearKalmanFilter(process_noise=0.05, measurement_noise=0.35)
    pf = state_estimation.ParticleFilter(num_particles=300, motion_noise=0.05, measurement_noise=0.35)
    est_kf, est_pf = [], []
    for z in meas:
        kf.predict(dt)
        kf.update(z[0], z[1])
        est_kf.append(np.array(kf.get_state()).ravel()[:2])
        pf.predict(dt)
        pf.update(z[0], z[1])
        pf.resample()
        est_pf.append(np.array(pf.estimate_state()).ravel()[:2])
    est_kf, est_pf = np.array(est_kf), np.array(est_pf)
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 4))
    a.plot(*meas.T, '.', color=COL['meas'], ms=4, label='noisy position measurements (σ = 0.35 m)')
    a.plot(*truth.T, color=COL['truth'], lw=2, label='true path')
    a.plot(*est_kf.T, color=COL['a'], lw=2, label='Kalman filter')
    a.plot(*est_pf.T, color=COL['c'], lw=1.5, ls='--', label='particle filter (300)')
    a.set_aspect('equal')
    a.legend(fontsize=8)
    a.set_title('Tracking a moving robot from noisy fixes')
    err = lambda e: np.hypot(*(e - truth).T)  # noqa: E731
    b.plot(t, err(meas), color=COL['meas'], label='measurement error (mean %.2f m)' % err(meas).mean())
    b.plot(t, err(est_kf), color=COL['a'], lw=2, label='Kalman error (mean %.2f m)' % err(est_kf).mean())
    b.plot(t, err(est_pf), color=COL['c'], label='particle error (mean %.2f m)' % err(est_pf).mean())
    b.set_xlabel('time (s)')
    b.set_ylabel('position error (m)')
    b.legend(fontsize=8)
    b.set_title('Filtering removes most of the noise')
    save(fig, 'kalman_tracking.png')


def fig_imu_fusion():
    rng = np.random.default_rng(2)
    dt, n = 0.01, 3000
    t = np.arange(n) * dt
    roll = 0.4 * np.sin(0.5 * t) + 0.2 * (t > 15)                     # true tilt (rad)
    gx = np.gradient(roll, dt) + 0.02 + rng.normal(0, 0.01, n)          # gyro: biased (drifts)
    ay = 9.81 * np.sin(roll) + rng.normal(0, 1.2, n)                     # accelerometer: noisy
    az = 9.81 * np.cos(roll) + rng.normal(0, 1.2, n)
    acc_roll = np.arctan2(ay, az)
    gyro_roll = np.cumsum(gx) * dt
    comp = sensor_fusion.ComplementaryImu(alpha=0.98)
    madg = sensor_fusion.MadgwickFilter(sample_rate=100.0, beta=0.05)
    out_c, out_m = [], []
    for k in range(n):
        _, r = comp.update(0.0, ay[k], az[k], gx[k], 0.0, dt)   # returns (pitch, roll)
        out_c.append(r)
        madg.update(0.0, ay[k], az[k], gx[k], 0.0, 0.0, dt)
        out_m.append(madg.get_euler_angles()[0])
    fig, ax = plt.subplots(figsize=(11, 3.8))
    ax.plot(t, np.degrees(acc_roll), color=COL['meas'], lw=0.5, label='accelerometer only: noisy')
    ax.plot(t, np.degrees(gyro_roll), color='#f08c00', lw=1.5, label='gyro only: smooth but drifts')
    ax.plot(t, np.degrees(out_c), color=COL['a'], lw=2, label='complementary filter (alpha 0.98)')
    ax.plot(t, np.degrees(out_m), color=COL['c'], lw=1.5, ls='--', label='Madgwick filter')
    ax.plot(t, np.degrees(roll), color=COL['truth'], lw=1, label='truth')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('roll (deg)')
    ax.legend(fontsize=8, ncol=3, loc='upper left')
    ax.set_title('IMU attitude: fuse the gyro (short term) with the accelerometer (long term)')
    save(fig, 'imu_fusion.png')


# ---------------------------------------------------------------------------
# Control
# ---------------------------------------------------------------------------

def fig_pid():
    dt, n = 0.02, 600
    t = np.arange(n) * dt
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for label, gains, colour in [('P only (kp 4): settles short', (4.0, 0.0, 0.0), '#f08c00'),
                                 ('PI (kp 4, ki 1): reaches it, overshoots', (4.0, 1.0, 0.0), COL['b']),
                                 ('PID (kp 4, ki 1, kd 2.5): reaches it, little overshoot', (4.0, 1.0, 2.5), COL['a'])]:
        pid = control.PIDController(*gains, max_output=8.0, min_output=-8.0, max_integral=50.0, min_integral=-50.0)
        x = v = 0.0
        xs = []
        for _ in t:
            u = pid.update(1.0 - x, dt)
            # A cart with friction and a constant disturbance (like a slope).
            a_ = u - 0.8 * v - 0.6
            v += a_ * dt
            x += v * dt
            xs.append(x)
        ax.plot(t, xs, color=colour, lw=2, label=label)
    ax.axhline(1.0, color=COL['truth'], ls=':', label='setpoint')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position')
    ax.legend(fontsize=8)
    ax.set_title('PID step response (control.PIDController) on a cart with friction and a push:\n'
                 'P alone settles short, I removes the offset, D damps the overshoot')
    save(fig, 'pid_response.png')


# ---------------------------------------------------------------------------
# Perception
# ---------------------------------------------------------------------------

def fig_perception():
    rng = np.random.default_rng(4)
    ground = np.column_stack((rng.uniform(-3, 3, 900), rng.uniform(-3, 3, 900), rng.normal(0, 0.01, 900)))
    objects = []
    for cx, cy, h in ((1.2, 0.8, 0.6), (-1.5, 1.6, 1.0), (0.3, -1.7, 0.4)):
        objects.append(np.column_stack((rng.normal(cx, 0.12, 120), rng.normal(cy, 0.12, 120), rng.uniform(0.05, h, 120))))
    noise = np.column_stack((rng.uniform(-3, 3, 15), rng.uniform(-3, 3, 15), rng.uniform(0.1, 1.2, 15)))
    cloud = np.vstack([ground] + objects + [noise])
    rng.shuffle(cloud)
    ground_pts, obstacle_pts = perception.RANSACGroundRemoval(distance_threshold=0.05).segment(cloud.tolist())
    obstacle_pts = np.array(obstacle_pts)
    clusters, noise_pts = perception.DBSCANClusterer(eps=0.25, min_samples=8).cluster(obstacle_pts.tolist())
    fig = plt.figure(figsize=(14, 4.2))
    a = fig.add_subplot(1, 3, 1, projection='3d')
    a.scatter(*cloud.T, s=2, color=COL['meas'])
    a.set_title('1. Raw 3D point cloud (%d points)' % len(cloud))
    b = fig.add_subplot(1, 3, 2, projection='3d')
    g = np.array(ground_pts)
    b.scatter(*g.T, s=2, color='#ced4da')
    b.scatter(*obstacle_pts.T, s=3, color=COL['path'])
    b.set_title('2. RANSAC: fit the floor plane (grey),\nthe rest are obstacles (orange)')
    c = fig.add_subplot(1, 3, 3)
    for k, cl in enumerate(clusters):
        cl = np.array(cl)
        c.scatter(cl[:, 0], cl[:, 1], s=6, color=plt.cm.tab10(k), label='object %d (%d pts)' % (k + 1, len(cl)))
    if len(noise_pts):
        nz = np.array(noise_pts)
        c.scatter(nz[:, 0], nz[:, 1], s=10, marker='x', color='k', label='noise')
    c.set_aspect('equal')
    c.legend(fontsize=7)
    c.set_title('3. DBSCAN: group nearby points into objects\n(top view)')
    for ax in (a, b):
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])
    save(fig, 'perception_pipeline.png')


FIGURES = {'planners': fig_planners, 'dwa': fig_dwa, 'kalman': fig_kalman, 'imu': fig_imu_fusion,
           'pid': fig_pid, 'perception': fig_perception}

if __name__ == '__main__':
    for name in (sys.argv[1:] or FIGURES):
        try:
            FIGURES[name]()
        except Exception as exc:
            print('FAILED', name, type(exc).__name__, exc)
