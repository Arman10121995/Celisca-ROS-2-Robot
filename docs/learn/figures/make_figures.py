#!/usr/bin/env python3
"""Generate the figures and animations of the beginner course (docs/learn).

Every figure is drawn from the project's own code and data where it can be:
the small_house map, robot_lab_vacuum_cleaning's planner, room division and
mission, and the support-matrix JSON.  Run from the workspace root:

    python3 docs/learn/figures/make_figures.py            # all figures
    python3 docs/learn/figures/make_figures.py astar mcl  # some of them

Needs numpy, scipy, matplotlib and Pillow (all shipped with the ROS install).
"""

import json
import math
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib import animation, patches  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'src' / 'robot_lab_vacuum_cleaning'))
sys.path.insert(0, str(ROOT / 'src' / 'robot_lab_vacuum_cleaning' / 'test'))

from robot_lab_vacuum_cleaning import planner, rooms  # noqa: E402
from robot_lab_vacuum_cleaning.grid import Grid  # noqa: E402
from robot_lab_vacuum_cleaning.mission import Mission, PathFollower, RobotConfig, StallDetector  # noqa: E402
from robot_lab_vacuum_cleaning.store import Store  # noqa: E402

# One palette for every figure.
C = {'free': '#f7f7f2', 'wall': '#2b2d31', 'unknown': '#c9ccd1', 'path': '#e8590c', 'robot': '#1c7ed6',
     'clean': '#51cf66', 'zone': '#339af0', 'nogo': '#fa5252', 'goal': '#f59f00', 'muted': '#868e96',
     'particle': '#be4bdb'}
ROOM_COLOURS = ['#a5d8ff', '#ffd8a8', '#b2f2bb', '#eebefa', '#ffec99', '#99e9f2', '#ffc9c9', '#d0bfff']
plt.rcParams.update({'font.size': 10, 'axes.titlesize': 11, 'figure.dpi': 110, 'savefig.bbox': 'tight'})


def small_house():
    """The small_house occupancy grid (rows go up, as in nav_msgs)."""
    path = ROOT / 'src/robot_lab_maps/maps/small_house/maps/map.pgm'
    img = np.array(Image.open(path))[::-1]
    data = np.where(img < 100, 100, np.where(img > 250, 0, -1)).astype(np.int16)
    return Grid(0.05, (-12.5, -12.5), data)


def grid_rgb(grid, unknown=C['unknown']):
    rgb = np.zeros(grid.data.shape + (3,))
    rgb[:] = matplotlib.colors.to_rgb(unknown)
    rgb[grid.free_mask()] = matplotlib.colors.to_rgb(C['free'])
    rgb[grid.occupied_mask()] = matplotlib.colors.to_rgb(C['wall'])
    return rgb


def extent(grid):
    x0, y0 = grid.origin
    return [x0, x0 + grid.width * grid.resolution, y0, y0 + grid.height * grid.resolution]


def house_axes(ax, grid, title=None, crop=(-9.6, 9.8, -5.8, 5.8)):
    ax.set_xlim(crop[0], crop[1])
    ax.set_ylim(crop[2], crop[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    if title:
        ax.set_title(title)


def draw_robot(ax, x, y, yaw, r=0.165, colour=C['robot'], zorder=5):
    ax.add_patch(patches.Circle((x, y), r, color=colour, zorder=zorder))
    ax.plot([x, x + 1.6 * r * math.cos(yaw)], [y, y + 1.6 * r * math.sin(yaw)], color='white', lw=2, zorder=zorder + 1)


def save(fig, name):
    fig.savefig(OUT / name)
    plt.close(fig)
    print('wrote', name)


# ---------------------------------------------------------------------------
# Maps and sensing
# ---------------------------------------------------------------------------

def fig_occupancy_grid():
    g = small_house()
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={'width_ratios': [1.6, 1]})
    a.imshow(grid_rgb(g), origin='lower', extent=extent(g), interpolation='nearest')
    house_axes(a, g, 'The small_house map: 500 x 500 cells of 5 cm')
    a.add_patch(patches.Rectangle((2.0, 1.4), 1.2, 0.9, fill=False, ec=C['path'], lw=2))
    a.legend(handles=[patches.Patch(color=C['free'], ec='k', label='free (0)'),
                      patches.Patch(color=C['wall'], label='occupied (100)'),
                      patches.Patch(color=C['unknown'], label='unknown (-1)')], loc='lower left', fontsize=8)
    ix0, iy0 = g.world_to_cell(2.0, 1.4)
    sub = g.data[iy0:iy0 + 18, ix0:ix0 + 24]
    show = np.where(sub < 0, 50, np.where(sub >= 65, 100, 0))
    b.imshow(show, origin='lower', cmap='Greys', vmin=0, vmax=100)
    for (r, c), v in np.ndenumerate(sub):
        if c % 2 == 0 and r % 2 == 0:
            b.text(c, r, str(int(v)), ha='center', va='center', fontsize=6, color='white' if v >= 65 else C['muted'])
    b.set_title('Zoom (orange box): every cell holds a number')
    b.set_xticks([])
    b.set_yticks([])
    save(fig, 'occupancy_grid.png')


def raycast(grid, x, y, angle, max_range=6.0, step=0.025):
    for d in np.arange(0.12, max_range, step):
        ix, iy = grid.world_to_cell(x + d * math.cos(angle), y + d * math.sin(angle))
        if not grid.inside(ix, iy) or grid.data[iy, ix] >= 65:
            return d
    return max_range


def fig_lidar():
    g = small_house()
    x, y, yaw = 0.0, 0.0, 0.3
    fig, ax = plt.subplots(figsize=(7.5, 5))
    ax.imshow(grid_rgb(g), origin='lower', extent=extent(g), interpolation='nearest')
    for k, a in enumerate(np.linspace(-math.pi, math.pi, 120, endpoint=False)):
        d = raycast(g, x, y, yaw + a)
        ex, ey = x + d * math.cos(yaw + a), y + d * math.sin(yaw + a)
        ax.plot([x, ex], [y, ey], color=C['zone'], lw=0.5, alpha=0.5)
        ax.plot(ex, ey, '.', color=C['nogo'], ms=3)
    draw_robot(ax, x, y, yaw)
    house_axes(ax, g, '2D LiDAR: 360 beams, each returns the distance to the first wall', (-6.5, 6.5, -4.5, 4.5))
    save(fig, 'lidar_scan.png')


def fig_mcl():
    """Monte Carlo localisation (the idea behind AMCL) on the real map."""
    g = small_house()
    rng = np.random.default_rng(3)
    truth = (1.0, 0.5, 0.4)
    n = 2000
    free = np.argwhere(g.free_mask())
    pick = free[rng.choice(len(free), n)]
    parts = np.column_stack(((pick[:, 1] + rng.random(n)) * 0.05 - 12.5,
                             (pick[:, 0] + rng.random(n)) * 0.05 - 12.5, rng.uniform(-math.pi, math.pi, n)))
    beams = np.linspace(-math.pi, math.pi, 24, endpoint=False)
    z = np.array([raycast(g, truth[0], truth[1], truth[2] + b) for b in beams])

    def weights(ps):
        logw = np.zeros(len(ps))
        for i, p in enumerate(ps):
            pred = np.array([raycast(g, p[0], p[1], p[2] + b, step=0.08) for b in beams])
            logw[i] = -0.5 * np.sum(np.minimum(((pred - z) / 0.7) ** 2, 9.0))   # robust: cap each beam
        w = np.exp(logw - logw.max())
        return w / w.sum()

    def resample(ps, w, jitter):
        idx = rng.choice(len(ps), len(ps), p=w)
        return ps[idx] + rng.normal(0, jitter, ps.shape)

    stages = [('1. Start: %d guesses spread over the house' % n, parts, None)]
    w = weights(parts)
    stages.append(('2. First scan: weight = how well each guess explains it', parts, w))
    ps = parts
    for jitter in ([0.15, 0.15, 0.1], [0.08, 0.08, 0.05], [0.04, 0.04, 0.03]):
        ps = resample(ps, w, jitter)
        w = weights(ps)
    stages.append(('3. After three scans and resamples (zoomed)', ps, w))
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))
    for k, (ax, (title, pts, ww)) in enumerate(zip(axes, stages)):
        ax.imshow(grid_rgb(g), origin='lower', extent=extent(g), interpolation='nearest')
        ax.plot(truth[0], truth[1], '*', color=C['goal'], ms=16 if k < 2 else 30, mec='k', zorder=3)
        size = 4 if ww is None else (1 + 60 * ww / ww.max() if k < 2 else 8)
        ax.scatter(pts[:, 0], pts[:, 1], s=size, color=C['particle'], alpha=0.5, lw=0, zorder=4)
        crop = (truth[0] - 1.5, truth[0] + 1.5, truth[1] - 1.2, truth[1] + 1.2) if k == 2 else (-9.6, 9.8, -5.8, 5.8)
        house_axes(ax, g, title, crop)
    est = ps[:, :2].mean(axis=0)
    fig.suptitle('Particle filter localisation (the idea behind AMCL): star = true robot; purple = pose guesses. '
                 'Final estimate %.2f m from the truth' % math.dist(est, truth[:2]), y=1.02)
    save(fig, 'particle_filter.png')


def fig_tf_frames():
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.set_xlim(-0.5, 7)
    ax.set_ylim(-0.5, 4)
    ax.set_aspect('equal')
    ax.axis('off')

    def frame(x, y, yaw, name, colour):
        for dx, dy, lab in ((math.cos(yaw), math.sin(yaw), 'x'), (-math.sin(yaw), math.cos(yaw), 'y')):
            ax.annotate('', (x + 0.8 * dx, y + 0.8 * dy), (x, y), arrowprops=dict(arrowstyle='->', color=colour, lw=2))
            ax.text(x + 0.9 * dx, y + 0.9 * dy, lab, color=colour)
        ax.text(x - 0.2, y - 0.35, name, color=colour, fontsize=11, weight='bold')
    frame(0, 0, 0, 'map', '#212529')
    frame(1.2, 0.9, 0.12, 'odom', '#7048e8')
    frame(4.6, 2.2, 0.5, 'base_footprint', C['robot'])
    draw_robot(ax, 4.6, 2.2, 0.5, r=0.35)
    frame(4.95, 2.4, 0.5, '  laser_link', C['path'])
    ax.annotate('', (1.2, 0.9), (0, 0), arrowprops=dict(arrowstyle='->', color='#7048e8', ls='--'))
    ax.text(0.1, 0.65, 'map→odom\n(AMCL / SLAM:\ncorrects drift)', fontsize=8, color='#7048e8')
    ax.annotate('', (4.6, 2.2), (1.2, 0.9), arrowprops=dict(arrowstyle='->', color=C['robot'], ls='--'))
    ax.text(2.4, 1.15, 'odom→base_footprint\n(wheel odometry / EKF:\nsmooth, drifts)', fontsize=8, color=C['robot'])
    ax.text(5.3, 3.0, 'base→laser_link\n(fixed, from the URDF)', fontsize=8, color=C['path'])
    ax.set_title('TF: every coordinate frame is a node in a tree, every arrow a transform')
    save(fig, 'tf_frames.png')


# ---------------------------------------------------------------------------
# Planning
# ---------------------------------------------------------------------------

def fig_inflation():
    g = small_house()
    pm = planner.PlanningMap(g, 0.165)
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 4.4))
    im = a.imshow(np.minimum(pm.obstacle_clearance_m, 1.5), origin='lower', extent=extent(g), cmap='viridis')
    house_axes(a, g, 'Distance to the nearest obstacle (m)')
    fig.colorbar(im, ax=a, shrink=0.8)
    rgb = grid_rgb(g)
    blocked = ~pm.traversable & g.free_mask()
    rgb[blocked] = matplotlib.colors.to_rgb('#ffa8a8')
    b.imshow(rgb, origin='lower', extent=extent(g), interpolation='nearest')
    house_axes(b, g, 'Inflation: where the robot CENTRE may go (white)')
    b.legend(handles=[patches.Patch(color='#ffa8a8', label='free floor, but too close for a\n0.33 m robot (radius + 4 cm margin)')],
             loc='lower left', fontsize=8)
    save(fig, 'inflation.png')


def astar_steps(trav, start, goal):
    """A plain 8-connected A*, recording the expansion order (for teaching)."""
    import heapq
    h = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])  # noqa: E731
    best = {start: 0.0}
    parent = {}
    heap = [(h(start), 0.0, start)]
    order = []
    while heap:
        _, gcost, cell = heapq.heappop(heap)
        if gcost > best.get(cell, math.inf):
            continue
        order.append(cell)
        if cell == goal:
            break
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == dy == 0:
                    continue
                n = (cell[0] + dx, cell[1] + dy)
                if not (0 <= n[0] < trav.shape[1] and 0 <= n[1] < trav.shape[0]) or not trav[n[1], n[0]]:
                    continue
                ng = gcost + math.hypot(dx, dy)
                if ng < best.get(n, math.inf):
                    best[n] = ng
                    parent[n] = cell
                    heapq.heappush(heap, (ng + h(n), ng, n))
    path = [goal]
    while path[-1] in parent:
        path.append(parent[path[-1]])
    return order, path[::-1]


def fig_astar():
    g = small_house()
    pm = planner.PlanningMap(g, 0.165)
    # Plan on a 4x coarser grid so the animation is readable.
    k = 4
    trav = pm.traversable[:g.height // k * k, :g.width // k * k].reshape(g.height // k, k, g.width // k, k).all(axis=(1, 3))
    to_c = lambda x, y: (int((x + 12.5) / (0.05 * k)), int((y + 12.5) / (0.05 * k)))  # noqa: E731
    free_cells = np.argwhere(trav)

    def snap(x, y):
        c = to_c(x, y)
        r = free_cells[np.argmin((free_cells[:, 1] - c[0]) ** 2 + (free_cells[:, 0] - c[1]) ** 2)]
        return int(r[1]), int(r[0])
    start, goal = snap(-6.5, -4.0), snap(8.6, 0.6)
    order, path = astar_steps(trav, start, goal)
    assert order[-1] == goal, 'goal not reached'
    base = np.ones(trav.shape + (3,))
    base[~trav] = matplotlib.colors.to_rgb('#495057')
    fig, ax = plt.subplots(figsize=(8, 4.8))
    img = ax.imshow(base, origin='lower', interpolation='nearest')
    ax.set_xlim(start[0] - 15, goal[0] + 15)
    ax.set_ylim(min(start[1], goal[1]) - 20, max(start[1], goal[1]) + 25)
    ax.plot(*start, 'o', color=C['robot'], ms=9)
    ax.plot(*goal, '*', color=C['goal'], ms=15)
    line, = ax.plot([], [], color=C['path'], lw=3)
    title = ax.set_title('')
    ax.set_xticks([])
    ax.set_yticks([])
    frames = 60
    per = max(1, len(order) // frames)

    def draw(i):
        shown = base.copy()
        for c in order[:min(len(order), (i + 1) * per)]:
            shown[c[1], c[0]] = matplotlib.colors.to_rgb('#a5d8ff')
        img.set_data(shown)
        if (i + 1) * per >= len(order):
            line.set_data([p[0] for p in path], [p[1] for p in path])
            title.set_text('A*: shortest path found after %d cells' % len(order))
        else:
            title.set_text('A* explores cells nearest the goal first (%d)' % min(len(order), (i + 1) * per))
        return img, line, title
    anim = animation.FuncAnimation(fig, draw, frames=frames + 12, blit=False)
    anim.save(OUT / 'astar.gif', writer=animation.PillowWriter(fps=8))
    plt.close(fig)
    print('wrote astar.gif')


def fig_pure_pursuit():
    path = [(x, 0.6 * math.sin(x / 1.3)) for x in np.linspace(0, 6, 60)]
    f = PathFollower(path, 0.3, 1.6, lookahead=0.8)
    pose = [0.2, -0.5, 0.6]
    trail = [tuple(pose)]
    for _ in range(400):
        v, w, status = f.step(tuple(pose), math.inf)
        if status == 'done':
            break
        pose = [pose[0] + v * math.cos(pose[2]) * 0.05, pose[1] + v * math.sin(pose[2]) * 0.05, pose[2] + w * 0.05]
        trail.append(tuple(pose))
    snap = trail[25]
    g = PathFollower(path, 0.3, 1.6, lookahead=0.8)
    g.step(snap, math.inf)
    target = g.point_at(g.s + g.lookahead)
    fig, ax = plt.subplots(figsize=(9, 3.6))
    ax.plot(*zip(*path), '--', color=C['muted'], label='path to follow')
    ax.plot([p[0] for p in trail], [p[1] for p in trail], color=C['robot'], lw=2, label='robot trajectory')
    draw_robot(ax, *snap, r=0.12)
    ax.add_patch(patches.Circle(snap[:2], 0.8, fill=False, ls=':', color=C['path']))
    ax.plot(*target, 'o', color=C['path'], ms=9, label='lookahead point (0.8 m ahead on the path)')
    ax.annotate('', target, snap[:2], arrowprops=dict(arrowstyle='->', color=C['path'], lw=1.5))
    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('Pure pursuit: steer towards a point a fixed distance ahead on the path')
    save(fig, 'pure_pursuit.png')


def fig_diff_drive():
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))
    cases = [('v = 0.3 m/s, w = 0\nboth wheels equal: straight', 0.3, 0.0),
             ('v = 0.3 m/s, w = 0.8 rad/s\nright wheel faster: left arc', 0.3, 0.8),
             ('v = 0, w = 1.5 rad/s\nwheels opposite: spin on the spot', 0.0, 1.5)]
    for ax, (title, v, w) in zip(axes, cases):
        x = y = yaw = 0.0
        xs, ys = [x], [y]
        for _ in range(60):
            x += v * math.cos(yaw) * 0.05
            y += v * math.sin(yaw) * 0.05
            yaw += w * 0.05
            xs.append(x)
            ys.append(y)
        ax.plot(xs, ys, color=C['path'], lw=2)
        draw_robot(ax, 0, 0, 0, r=0.11)
        sep = 0.22
        vl, vr = v - w * sep / 2, v + w * sep / 2
        ax.annotate('', (0.0 + 0.6 * vl, 0.11), (0, 0.11), arrowprops=dict(arrowstyle='->', color='#2f9e44', lw=2))
        ax.annotate('', (0.0 + 0.6 * vr, -0.11), (0, -0.11), arrowprops=dict(arrowstyle='->', color='#2f9e44', lw=2))
        ax.text(-0.35, 0.25, 'left %.2f' % vl, fontsize=8, color='#2f9e44')
        ax.text(-0.35, -0.33, 'right %.2f' % vr, fontsize=8, color='#2f9e44')
        ax.set_xlim(-0.5, 1.2)
        ax.set_ylim(-0.6, 1.0)
        ax.set_aspect('equal')
        ax.set_title(title, fontsize=9)
    fig.suptitle('Differential drive: /cmd_vel (v, w) becomes two wheel speeds  v ∓ w·separation/2', y=1.04)
    save(fig, 'diff_drive.png')


# ---------------------------------------------------------------------------
# The robot vacuum
# ---------------------------------------------------------------------------

def fig_room_division():
    from scipy import ndimage
    g = small_house()
    free = g.free_mask()
    dist = ndimage.distance_transform_edt(free) * g.resolution
    labels = rooms.segment_rooms(g)
    cores = rooms.hierarchical_cores(free, dist, [0.45, 0.6, 0.75, 0.9, 1.05, 1.25, 1.5], int(0.6 / 0.0025))
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5))
    ax = axes[0, 0]
    ax.imshow(grid_rgb(g), origin='lower', extent=extent(g), interpolation='nearest')
    house_axes(ax, g, '1. The map: one connected free space')
    ax = axes[0, 1]
    im = ax.imshow(np.where(free, dist, np.nan), origin='lower', extent=extent(g), cmap='magma')
    house_axes(ax, g, '2. Clearance: doorways are narrow (dark)')
    fig.colorbar(im, ax=ax, shrink=0.75, label='m to the nearest wall')
    ax = axes[1, 0]
    show = np.ma.masked_where(cores == 0, cores)
    ax.imshow(grid_rgb(g), origin='lower', extent=extent(g), interpolation='nearest')
    ax.imshow(show, origin='lower', extent=extent(g), cmap='tab10', interpolation='nearest')
    house_axes(ax, g, '3. Room "cores": the wide middles of rooms')
    ax = axes[1, 1]
    rgb = grid_rgb(g)
    for rid in range(1, labels.max() + 1):
        rgb[labels == rid] = matplotlib.colors.to_rgb(ROOM_COLOURS[(rid - 1) % len(ROOM_COLOURS)])
    ax.imshow(rgb, origin='lower', extent=extent(g), interpolation='nearest')
    for rid, info in rooms.room_info(g, labels).items():
        ax.text(*info['anchor'], 'Room %d\n%.0f m²' % (rid, info['area_m2']), ha='center', fontsize=8, weight='bold')
    house_axes(ax, g, '4. Grow cores back (widest first): borders land in doorways')
    save(fig, 'room_division.png')


def fig_coverage_plan():
    g = small_house()
    pm = planner.PlanningMap(g, 0.165)
    labels = rooms.segment_rooms(g)
    region = pm.traversable & (labels == 4) & pm.reachable_from((0.0, 0.0))
    segs = planner.order_segments(planner.lane_segments(g, region, 0.24, planner.best_axis(g, region, 0.24), 0.13), (5.0, 2.8))
    edges = planner.order_polylines(planner.edge_contours(pm, region), (5.0, 2.8))
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    crop = (4.4, 9.6, -1.4, 3.6)
    for ax in axes:
        rgb = grid_rgb(g)
        rgb[region] = matplotlib.colors.to_rgb('#e7f5ff')
        ax.imshow(rgb, origin='lower', extent=extent(g), interpolation='nearest')
        house_axes(ax, g, None, crop)
    axes[0].set_title('1. Edges first: follow walls and furniture')
    for line in edges:
        axes[0].plot(*zip(*line), color='#7048e8', lw=2)
    axes[1].set_title('2. Lanes 0.24 m apart (cleaning width 0.26 m)')
    for i, (p, q) in enumerate(segs):
        axes[1].plot([p[0], q[0]], [p[1], q[1]], color=C['path'], lw=1.5)
        if i < len(segs) - 1:
            n = segs[i + 1][0]
            axes[1].plot([q[0], n[0]], [q[1], n[1]], color=C['muted'], lw=0.8, ls=':')
    covered = planner.sweep_mask(g, segs, 0.13) | planner.sweep_mask(g, [(a, b) for line in edges for a, b in zip(line, line[1:])], 0.195)
    rgb = grid_rgb(g)
    rgb[covered & g.free_mask()] = matplotlib.colors.to_rgb(C['clean'])
    axes[2].imshow(rgb, origin='lower', extent=extent(g), interpolation='nearest')
    house_axes(axes[2], g, '3. Swept (green): %.0f%% of where its centre can go' % (
        100 * (covered & region).sum() / region.sum()), crop)
    save(fig, 'coverage_plan.png')


class _World:
    """A kinematic robot in the map (the unit tests' harness, simplified)."""

    def __init__(self, grid, pose, radius=0.165):
        ys, xs = np.nonzero(grid.occupied_mask())
        self.pts = np.column_stack(((xs + 0.5) * 0.05 + grid.origin[0], (ys + 0.5) * 0.05 + grid.origin[1]))
        self.pose = list(pose)
        self.radius = radius

    def sense(self):
        from robot_lab_vacuum_cleaning.mission import Sense
        x, y, yaw = self.pose
        d = np.hypot(self.pts[:, 0] - x, self.pts[:, 1] - y)
        near = self.pts[d < 1.5]
        ang = np.arctan2(near[:, 1] - y, near[:, 0] - x) - yaw
        ahead = np.abs(np.arctan2(np.sin(ang), np.cos(ang))) < math.radians(35)
        front = float(d[d < 1.5][ahead].min()) - self.radius if ahead.any() else math.inf
        return Sense(front_clear=front, points=near)

    def move(self, v, w, dt):
        x, y, yaw = self.pose
        nx, ny = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
        if np.hypot(self.pts[:, 0] - nx, self.pts[:, 1] - ny).min() > self.radius - 0.01:
            x, y = nx, ny
        self.pose = [x, y, yaw + w * dt]


def run_job(request, seconds=3000, record_every=2.0, battery_scale=None):
    g = small_house()
    m = Mission(RobotConfig(robot_radius=0.165), Store(Path(tempfile.mkdtemp())), map_key='figures')
    if battery_scale:
        m.settings.update(battery_time_scale=battery_scale, resume_battery_pct=60.0)
    m.update_map(g)
    w = _World(g, (0.0, 0.0, 0.0))
    m.step(0.0, tuple(w.pose), w.sense())
    print(m.command(dict(request, cmd='start')))
    t, frames, timeline = 0.0, [], []
    while t < seconds and (m.job is not None or m.state in ('returning', 'docking')):
        v, wz = m.step(t, tuple(w.pose), w.sense())
        w.move(v, wz, 0.1)
        if not frames or t - frames[-1][0] >= record_every:
            frames.append((t, list(w.pose), m.coverage.copy(), m.state, m.battery))
        timeline.append((t, m.state, m.battery))
        t += 0.1
    return g, m, frames, timeline


def fig_coverage_animation():
    g, m, frames, _ = run_job({'mode': 'rooms', 'rooms': [4], 'edge': True}, record_every=4.0)
    crop = (4.3, 9.7, -1.5, 3.7)
    fig, ax = plt.subplots(figsize=(6.2, 6))
    base = grid_rgb(g)
    img = ax.imshow(base, origin='lower', extent=extent(g), interpolation='nearest')
    house_axes(ax, g, None, crop)
    robot = patches.Circle((0, 0), 0.165, color=C['robot'], zorder=5)
    ax.add_patch(robot)
    title = ax.set_title('')
    trail_x, trail_y = [], []
    line, = ax.plot([], [], color=C['robot'], lw=0.8, alpha=0.6)

    def draw(i):
        t, pose, cov, state, battery = frames[min(i, len(frames) - 1)]
        rgb = base.copy()
        rgb[cov & g.free_mask()] = matplotlib.colors.to_rgb(C['clean'])
        img.set_data(rgb)
        robot.center = pose[:2]
        trail_x.append(pose[0])
        trail_y.append(pose[1])
        line.set_data(trail_x, trail_y)
        title.set_text('Office clean  t = %3d s  state: %s  battery %.0f%%' % (t, state, battery))
        return img, robot, line, title
    anim = animation.FuncAnimation(fig, draw, frames=len(frames) + 8)
    anim.save(OUT / 'vacuum_cleaning.gif', writer=animation.PillowWriter(fps=10))
    plt.close(fig)
    print('wrote vacuum_cleaning.gif (%s, %.0f%%)' % (m.last_record['result'], m.last_record['coverage_pct']))


def fig_battery():
    _, m, _, timeline = run_job({'mode': 'rooms', 'rooms': [4], 'edge': True}, seconds=6000, record_every=30,
                                battery_scale=40.0)
    t = np.array([x[0] for x in timeline]) / 60.0
    b = np.array([x[2] for x in timeline])
    states = [x[1] for x in timeline]
    fig, ax = plt.subplots(figsize=(10, 3.6))
    colours = {'cleaning': C['clean'], 'returning': C['goal'], 'docking': C['goal'], 'charging': C['zone'],
               'docked': C['muted']}
    start = 0
    for i in range(1, len(states) + 1):
        if i == len(states) or states[i] != states[start]:
            ax.axvspan(t[start], t[i - 1], color=colours.get(states[start], '#eee'), alpha=0.25, lw=0)
            start = i
    ax.plot(t, b, color='#212529', lw=2)
    ax.axhline(20, ls=':', color=C['nogo'])
    ax.text(t[-1], 21, 'return at 20 %', ha='right', fontsize=8, color=C['nogo'])
    ax.axhline(60, ls=':', color=C['zone'])
    ax.text(t[-1], 61, 'resume at 60 %', ha='right', fontsize=8, color=C['zone'])
    ax.set_xlabel('time (min, battery model 40x faster than real)')
    ax.set_ylabel('battery %')
    ax.legend(handles=[patches.Patch(color=c, alpha=0.4, label=s) for s, c in colours.items()], ncol=5, fontsize=8,
              loc='upper center', bbox_to_anchor=(0.5, -0.25))
    ax.set_title('Recharge and resume: the job pauses at the dock and continues (%d recharges)' % m.last_record['recharges'])
    save(fig, 'battery_recharge.png')


def fig_stall():
    walls = []
    for xs, ys in ((np.linspace(0, 4, 81), np.zeros(81)), (np.linspace(0, 4, 81), np.full(81, 3.0)),
                   (np.zeros(61), np.linspace(0, 3, 61)), (np.full(61, 4.0), np.linspace(0, 3, 61))):
        walls.append(np.column_stack((xs, ys)))
    walls = np.concatenate(walls)

    def scan(pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        d = walls - [x, y]
        return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))
    start, claimed = (1.0, 1.5, 0.0), (1.4, 1.5, 0.0)
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    for ax, (title, truth) in zip(axes, [('Moving: the new scan matches "moved 0.4 m"', claimed),
                                         ('Stalled: the new scan matches "did not move"', start)]):
        old_in_new_if_moved = scan(start) + [start[0] - claimed[0], 0]
        new = scan(truth)
        ax.plot(*new.T, '.', color='#212529', ms=3, label='latest scan')
        ax.plot(*old_in_new_if_moved.T, '.', color=C['clean'], ms=2, label='old scan, if it moved as odometry says')
        ax.plot(*scan(start).T, '.', color=C['nogo'], ms=2, alpha=0.6, label='old scan, if it did not move')
        draw_robot(ax, 0, 0, 0, r=0.15)
        ax.set_aspect('equal')
        ax.set_xlim(-1.8, 3.2)
        ax.set_ylim(-1.8, 1.8)
        ax.set_title(title, fontsize=10)
    axes[0].legend(fontsize=7, loc='lower right')
    fig.suptitle('Stall detector: odometry says "I moved"; the LiDAR decides whether that is true', y=1.03)
    save(fig, 'stall_detector.png')


def fig_spot():
    g = small_house()
    pm = planner.PlanningMap(g, 0.165)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.4))
    for ax, (centre, title) in zip(axes, [((0.0, 0.0), 'Open floor: a square spiral'),
                                          ((2.0, 2.0), 'Beside furniture: outline + lanes')]):
        ax.imshow(grid_rgb(g), origin='lower', extent=extent(g), interpolation='nearest')
        runs = planner.spot_spiral(pm, centre, 1.6, 0.234)
        h = 0.8
        square = g.polygon_mask([(centre[0] - h, centre[1] - h), (centre[0] + h, centre[1] - h),
                                 (centre[0] + h, centre[1] + h), (centre[0] - h, centre[1] + h)])
        region = pm.traversable & square
        segs = [(p, q) for r in runs for p, q in zip(r, r[1:])]
        swept = planner.sweep_mask(g, segs, 0.13) & region
        if swept.sum() >= 0.9 * region.sum():
            for r in runs:
                ax.plot(*zip(*r), color=C['path'], lw=1.5)
        else:
            for line in planner.edge_contours(pm, region, boundary=True):
                ax.plot(*zip(*line), color='#7048e8', lw=1.5)
            for p, q in planner.lane_segments(g, region, 0.24, 'x', 0.13):
                ax.plot([p[0], q[0]], [p[1], q[1]], color=C['path'], lw=1.5)
        ax.add_patch(patches.Rectangle((centre[0] - h, centre[1] - h), 2 * h, 2 * h, fill=False, ec=C['zone'], ls='--'))
        house_axes(ax, g, title, (centre[0] - 1.4, centre[0] + 1.4, centre[1] - 1.4, centre[1] + 1.4))
    save(fig, 'spot_clean.png')


# ---------------------------------------------------------------------------
# Evidence
# ---------------------------------------------------------------------------

def fig_matrix():
    folders = sorted((ROOT / 'docs/status/evidence').glob('support-matrix-2026-*'))
    data = json.load(open(folders[-1] / 'support-matrix-current.json'))['support_cells']
    sims = ['gazebo', 'mujoco', 'pybullet', 'isaac']
    passed = [sum(1 for c in data if c['simulator'] == s and str(c['test_result']).startswith('passed')) for s in sims]
    failed = [sum(1 for c in data if c['simulator'] == s and str(c['test_result']).startswith('failed')) for s in sims]
    fig, ax = plt.subplots(figsize=(7, 3.4))
    ax.bar(sims, passed, color=C['clean'], label='passed screen')
    ax.bar(sims, failed, bottom=passed, color=C['nogo'], label='failed screen')
    for i, (p, f) in enumerate(zip(passed, failed)):
        ax.text(i, p + f + 2, '%d / %d' % (p, p + f), ha='center', fontsize=9)
    ax.set_ylabel('measured cells')
    ax.set_title('Support matrix (%s): one cell = robot x world x simulator x task' % folders[-1].name[-10:])
    ax.legend(fontsize=8)
    save(fig, 'support_matrix.png')


FIGURES = {'grid': fig_occupancy_grid, 'lidar': fig_lidar, 'mcl': fig_mcl, 'tf': fig_tf_frames,
           'inflation': fig_inflation, 'astar': fig_astar, 'pursuit': fig_pure_pursuit, 'diff': fig_diff_drive,
           'rooms': fig_room_division, 'coverage': fig_coverage_plan, 'spot': fig_spot, 'stall': fig_stall,
           'cleaning': fig_coverage_animation, 'battery': fig_battery, 'matrix': fig_matrix}

if __name__ == '__main__':
    for name in (sys.argv[1:] or FIGURES):
        FIGURES[name]()
