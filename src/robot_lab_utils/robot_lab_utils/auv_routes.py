"""Source-constrained Tethys Dubins routes and sensor-estimated path following.

Numerical Dubins geometry is pinned MIT PythonRobotics code under _vendor.
The vehicle travels along source -X; no sideways, reverse or pivot commands.
"""
import math
import time
import numpy as np
from ._vendor.pythonrobotics_dubins import plan_dubins_path


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def plan_auv(scene, start, body_yaw, goal, radius, bottom, surface, canceled,
             goal_heading=None, turn_radius=12., timeout=15.):
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if (start.shape != (3,) or goal.shape != (3,) or not np.isfinite([start, goal]).all()
            or not all(map(math.isfinite, [body_yaw, radius, turn_radius, bottom, surface]))
            or radius <= 0 or not 10. <= turn_radius <= 30.):
        raise ValueError('Finite AUV poses, footprint and a 10–30 m cruise turn radius required')
    if not bottom+radius+.1 <= min(start[2], goal[2]) <= max(start[2], goal[2]) <= surface-radius-.1:
        raise ValueError('AUV route violates source-inflated depth clearance')
    if np.linalg.norm(goal[:2]-start[:2]) < 2.:
        raise ValueError('AUV needs at least 2 m of horizontal cruise; it cannot pivot/hover')
    heading = angle(body_yaw+math.pi)
    initial=start.copy()
    # Native fins have no authority at rest. A mandatory source-checked metre
    # of straight run-up establishes flow before asking for curved guidance.
    start=start.copy();start[:2]+=np.array([math.cos(heading),math.sin(heading)])
    bearing = math.atan2(goal[1]-start[1], goal[0]-start[0])
    if goal_heading is not None and not math.isfinite(goal_heading):
        raise ValueError('Finite AUV exit course required')
    endings = [float(goal_heading)] if goal_heading is not None else [bearing, heading, *np.linspace(-math.pi, math.pi, 9)[:-1]]
    deadline, candidates = time.monotonic()+timeout, []
    for ending in endings:
        # Exact circle tangencies can make an upstream zero-length straight
        # segment slightly negative through roundoff, selecting a needless
        # full loop. Admit the analytic single-arc degeneracy, checked through
        # the identical depth/footprint/collision/braking barriers below.
        turn = angle(ending-heading)
        if abs(turn)>1e-9:
            centre=start[:2]+math.copysign(turn_radius,turn)*np.array([-math.sin(heading),math.cos(heading)])
            offset=start[:2]-centre
            end=centre+np.array([[math.cos(turn),-math.sin(turn)],[math.sin(turn),math.cos(turn)]])@offset
            length=abs(turn)*turn_radius
            if np.linalg.norm(end-goal[:2])<1e-7 and length<=45. and abs(goal[2]-start[2])/length<=.1:
                angles=np.linspace(0.,turn,max(2,math.ceil(length/.15)+1))
                xyz=np.column_stack([centre[0]+np.cos(angles)*offset[0]-np.sin(angles)*offset[1],
                    centre[1]+np.sin(angles)*offset[0]+np.cos(angles)*offset[1],np.linspace(start[2],goal[2],len(angles))])
                candidates.append((length,xyz,heading+angles,['L' if turn>0 else 'R']))
        for word in ('LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL'):
            if canceled() or time.monotonic() > deadline:
                raise TimeoutError('AUV planning canceled or deadline exceeded')
            try:
                x, y, yaw, modes, lengths = plan_dubins_path(*start[:2], heading, *goal[:2], ending,
                    1./turn_radius, step_size=.15/turn_radius, selected_types=[word])
            except (TypeError, ValueError):
                continue  # The selected Dubins word has no real solution.
            length = float(sum(abs(value) for value in lengths))
            if length > 45. or abs(goal[2]-start[2])/max(length, 1e-9) > .1:
                continue
            xy = np.column_stack([x, y])
            distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
            xyz = np.column_stack([xy, start[2]+(goal[2]-start[2])*distances/max(distances[-1], 1e-9)])
            candidates.append((length, xyz, np.asarray(yaw), modes))
    for length, xyz, yaw, modes in sorted(candidates, key=lambda row: row[0]):
        lead=np.linspace(initial,start,8)
        xyz=np.vstack([lead[:-1],xyz]);yaw=np.r_[np.full(7,heading),yaw]
        length+=1.
        if length>45.:continue
        valid = True
        for a, b in zip(xyz[:-1], xyz[1:]):
            if not scene.segment(a, b, radius+.01, deadline, canceled):
                valid = False
                break
        if valid:
            brake = xyz[-1]+3.*np.array([math.cos(yaw[-1]), math.sin(yaw[-1]), 0.])
            if not scene.segment(xyz[-1], brake, radius, deadline, canceled):
                continue
            return xyz.tolist(), dict(kind='auv_dubins', length_m=length, turn_radius_m=turn_radius,
                headings_rad=yaw.tolist(), source_forward_axis='-X', modes=list(modes),
                terminal_course_rad=float(yaw[-1]), braking_corridor_m=3., max_depth_slope=.1,
                straight_runup_m=1.,
                geometry='Pinned MIT PythonRobotics Dubins, selected-world/observed collision checks')
    raise ValueError('No source-clear curvature-constrained AUV route and 3 m terminal braking corridor')


class AuvPathFollower:
    def __init__(self, points, headings):
        self.points = np.asarray(points, dtype=float)
        self.headings = np.unwrap(np.asarray(headings, dtype=float))
        if (self.points.ndim != 2 or self.points.shape[1] != 3 or len(self.points) < 2
                or self.headings.shape != (len(self.points),) or not np.isfinite(self.points).all()
                or not np.isfinite(self.headings).all()):
            raise ValueError('A checked sampled AUV path and matching headings are required')
        segments = np.linalg.norm(np.diff(self.points[:, :2], axis=0), axis=1)
        self.arc = np.r_[0., np.cumsum(segments)]
        self.progress = 0
        self.cross_track = None

    def command(self, position, body_yaw, velocity):
        position, velocity = np.asarray(position), np.asarray(velocity)
        if not np.isfinite([position, velocity]).all() or not math.isfinite(body_yaw):
            raise ValueError('Finite sensor-estimated AUV feedback required')
        # Local monotone nearest point prevents jumping across a Dubins loop.
        stop = min(len(self.points), self.progress+25)
        nearest = self.progress+int(np.argmin(np.linalg.norm(self.points[self.progress:stop, :2]-position[:2], axis=1)))
        self.progress = nearest
        self.cross_track = float(np.linalg.norm(self.points[nearest, :2]-position[:2]))
        if self.cross_track > 1.:
            raise ValueError('AUV left the 1 m estimated tracking envelope; explicit replan required')
        look = min(len(self.points)-1, int(np.searchsorted(self.arc, self.arc[nearest]+.7)))
        target = self.points[look]
        tangent = np.array([math.cos(self.headings[nearest]), math.sin(self.headings[nearest])])
        offset = position[:2]-self.points[nearest, :2]
        signed_error = tangent[0]*offset[1]-tangent[1]*offset[0]
        speed=float(np.linalg.norm(velocity[:2]))
        course=math.atan2(velocity[1],velocity[0]) if speed>.15 else body_yaw+math.pi
        sideslip=angle(course-body_yaw-math.pi)
        desired = self.headings[nearest]-sideslip-math.atan2(.3*signed_error, .3)
        error = angle(desired-body_yaw-math.pi)
        if abs(error) > 1.:
            raise ValueError('AUV course error exceeds flow-guidance envelope; explicit replan required')
        delta_arc = max(self.arc[look]-self.arc[nearest], .01)
        curvature = (self.headings[look]-self.headings[nearest])/delta_arc
        cruise = .3
        yaw_rate = max(-.12, min(.12, max(.15,speed)*curvature+.6*error))
        slope = (target[2]-self.points[nearest, 2])/delta_arc
        vertical = max(-.03, min(.03, cruise*slope+.3*(target[2]-position[2])))
        remaining = float(self.arc[-1]-self.arc[nearest])
        terminal_error = abs(angle(self.headings[-1]-course))
        brake = (remaining < max(.3, 1.5*float(np.linalg.norm(velocity)))
                 and np.linalg.norm(position-self.points[-1]) < .85 and terminal_error < .3)
        return (cruise, 0., vertical, yaw_rate), target.tolist(), nearest, brake
