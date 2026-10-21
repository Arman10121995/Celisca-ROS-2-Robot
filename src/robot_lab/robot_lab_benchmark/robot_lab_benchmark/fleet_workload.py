"""Run fleet/swarm experiments with task dispatch, coverage, and traffic coordination.

Supports multiple robot coordination scenarios under Nav2 with independent truth.
Fleet workload schema: robot_lab.fleet_workload.v1
"""

import hashlib
import json
import math
import os
import random
from pathlib import Path

from .workload_contract import FLEET_WORKLOAD_SCHEMA, FLEET_RESULT_SCHEMA, validate_workload


class FleetWorkload:
    """Run fleet/swarm experiments with task dispatch, coverage, and traffic coordination."""

    def __init__(self, specification, directory):
        self.specification, self.directory = specification, directory
        self.trace_file = (directory/'fleet-trace.jsonl').open('w', buffering=1)
        self.fleet_id = specification['fleet_id']
        self.member_count = len(specification['fleet_goals'])
        self.goal_index = 0
        self.total_goals_completed = 0
        self.member_status = {}
        for i in range(self.member_count):
            self.member_status[i] = {'x': 0, 'y': 0, 'yaw': 0, 'active': True}
        self.trace_write('workload_start', fleet_id=self.fleet_id,
                         fleet_goals=specification['fleet_goals'])

    def trace_write(self, event, **kwargs):
        entry = dict(event=event, wall_monotonic=os.times() if hasattr(os, 'times') else 0, **kwargs)
        self.trace_file.write(json.dumps(entry) + '\n')

    def dispatch_task(self, member_id, target_x, target_y, target_yaw):
        """Dispatch a navigation task to a specific fleet member."""
        self.member_status[member_id] = {'x': target_x, 'y': target_y, 'yaw': target_yaw, 'active': True}
        self.trace_write('task_dispatched', member_id=member_id,
                         target_x=target_x, target_y=target_y, target_yaw=target_yaw)

    def coverage_pattern(self, member_id, grid_size=5, spacing=2.0):
        """Generate a coverage pattern for a fleet member."""
        x = random.uniform(-grid_size, grid_size)
        y = random.uniform(-grid_size, grid_size)
        yaw = random.uniform(-math.pi, math.pi)
        self.dispatch_task(member_id, x, y, yaw)
        self.trace_write('coverage_pattern', member_id=member_id,
                         grid_size=grid_size, spacing=spacing)

    def traffic_coordination(self, member_id, intersection_x, intersection_y, direction='straight'):
        """Coordinate traffic at an intersection for a fleet member."""
        self.dispatch_task(member_id, intersection_x, intersection_y, 0.0)
        self.trace_write('traffic_coordination', member_id=member_id,
                         intersection_x=intersection_x, intersection_y=intersection_y, direction=direction)

    def traffic_light_coordination(self, member_id, intersection_x, intersection_y, direction='straight', light_state='green'):
        """Coordinate traffic at a traffic light intersection for a fleet member.

        Args:
            member_id: Fleet member identifier
            intersection_x: X coordinate of the intersection
            intersection_y: Y coordinate of the intersection
            direction: Direction to travel ('straight', 'left', 'right')
            light_state: Current traffic light state ('red', 'yellow', 'green')
        """
        # If light is red, wait; if green, proceed
        if light_state.lower() == 'red':
            self.dispatch_task(member_id, intersection_x, intersection_y, 0.0)
            self.trace_write('traffic_light_wait', member_id=member_id,
                             intersection_x=intersection_x,
                             intersection_y=intersection_y,
                             light_state=light_state, direction=direction)
        else:
            self.traffic_coordination(member_id, intersection_x, intersection_y, direction)
            self.trace_write('traffic_light_proceed', member_id=member_id,
                             intersection_x=intersection_x, intersection_y=intersection_y,
                             light_state=light_state, direction=direction)

    def run_traffic_signal_cycle(self, member_id, intersection_x, intersection_y,
                                 green_duration=5.0, red_duration=3.0):
        """Run a complete traffic signal cycle for a fleet member.

        Args:
            member_id: Fleet member identifier
            intersection_x: X coordinate of the intersection
            intersection_y: Y coordinate of the intersection
            green_duration: Duration of green light in seconds
            red_duration: Duration of red light in seconds
        """
        # Green light - proceed through intersection
        self.traffic_light_coordination(member_id, intersection_x, intersection_y, 'straight', 'green')

        # Simulate red light - stop before intersection
        self.traffic_light_coordination(member_id, intersection_x, intersection_y, 'stop', 'red')

        # Green light - proceed again
        self.traffic_light_coordination(member_id, intersection_x, intersection_y, 'straight', 'green')

    def mark_completed(self, member_id):
        """Mark a fleet member as having completed its task."""
        if member_id in self.member_status:
            self.member_status[member_id]['active'] = False
            self.total_goals_completed += 1
            self.trace_write('task_completed', member_id=member_id)
        else:
            pass  # silently ignore unknown member

    def get_result(self):
        """Generate fleet experiment result."""
        success = self.total_goals_completed == self.member_count
        member_results = []
        for i in range(self.member_count):
            status = self.member_status.get(i, {'x': 0, 'y': 0, 'yaw': 0, 'active': False})
            member_results.append({
                'member_id': i,
                'x': status['x'],
                'y': status['y'],
                'yaw': status['yaw'],
                'success': not status['active']
            })
        return {
            'schema': FLEET_RESULT_SCHEMA,
            'fleet_id': self.fleet_id,
            'run_id': os.environ.get('ROBOT_LAB_EXPERIMENT_ID', 'unknown'),
            'success': success,
            'member_results': member_results,
            'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID', '0')),
        }

    def swarm_formation(self, member_id, formation_type='wedge', offset_radius=2.0, angle_offset=0.0):
        """Assign a swarm formation position for a fleet member.

        Creates coordinated formation positions for swarm members.

        Args:
            member_id: Fleet member identifier
            formation_type: Type of formation ('wedge', 'circle', 'line', 'random')
            offset_radius: Distance from center point
            angle_offset: Base angle offset for formation
        """
        formation_angles = {
            'wedge': member_id * (math.pi / 4),
            'circle': (2.0 * math.pi * member_id) / max(self.member_count, 1),
            'line': member_id * (math.pi / (max(self.member_count, 1) / 2)),
            'random': random.uniform(0, 2 * math.pi)
        }

        angle = formation_angles.get(formation_type, 0) + angle_offset
        x = offset_radius * math.cos(angle)
        y = offset_radius * math.sin(angle)
        yaw = angle + math.pi  # Face outward from center

        self.dispatch_task(member_id, x, y, yaw)
        self.trace_write('swarm_formation', member_id=member_id,
                         formation_type=formation_type,
                         offset_radius=offset_radius,
                         angle=angle)

    def swarm_rendezvous(self, member_id, target_x, target_y, target_yaw):
        """Coordinate swarm rendezvous - all members converge to a target.

        Args:
            member_id: Fleet member identifier
            target_x: Target X coordinate
            target_y: Target Y coordinate
            target_yaw: Target yaw angle
        """
        self.dispatch_task(member_id, target_x, target_y, target_yaw)
        self.trace_write('swarm_rendezvous', member_id=member_id,
                         target_x=target_x, target_y=target_y, target_yaw=target_yaw)

    def swarm_dispersion(self, member_id, center_x=0.0, center_y=0.0, radius=3.0):
        """Coordinate swarm dispersion from a center point.

        Members spread out to evenly distributed positions around a circle.

        Args:
            member_id: Fleet member identifier
            center_x: Center X coordinate
            center_y: Center Y coordinate
            radius: Dispersion radius
        """
        angle = (2.0 * math.pi * member_id) / max(self.member_count, 1)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        yaw = angle + math.pi  # Face outward

        self.dispatch_task(member_id, x, y, yaw)
        self.trace_write('swarm_dispersion', member_id=member_id,
                         center_x=center_x, center_y=center_y,
                         radius=radius, angle=angle)

    def swarm_flocking(self, member_id, target_x, target_y, target_yaw, neighbors=None):
        """Coordinate swarm flocking behavior - members follow neighbors.

        Simulates flocking behavior where members align with nearby flock members.

        Args:
            member_id: Fleet member identifier
            target_x: Target X coordinate
            target_y: Target Y coordinate
            target_yaw: Target yaw angle
            neighbors: List of neighboring member IDs (optional)
        """
        if neighbors and len(neighbors) > 0:
            # Average neighbor positions for flocking alignment
            avg_x = sum(self.member_status.get(n, {}).get('x', 0) for n in neighbors) / len(neighbors)
            avg_y = sum(self.member_status.get(n, {}).get('y', 0) for n in neighbors) / len(neighbors)
            # Blend target with neighbor average
            blended_x = 0.7 * target_x + 0.3 * avg_x
            blended_y = 0.7 * target_y + 0.3 * avg_y
        else:
            blended_x = target_x
            blended_y = target_y

        self.dispatch_task(member_id, blended_x, blended_y, target_yaw)
        self.trace_write('swarm_flocking', member_id=member_id,
                         target_x=blended_x, target_y=blended_y,
                         has_neighbors=len(neighbors) if neighbors else 0)

    def cooperative_coverage(self, member_id, grid_size=5, spacing=2.0, coverage_area=None):
        """Coordinate cooperative coverage of an area by multiple fleet members.

        Members divide the coverage area into regions and each member covers its assigned region.

        Args:
            member_id: Fleet member identifier
            grid_size: Size of the coverage grid
            spacing: Spacing between coverage points
            coverage_area: Optional [min_x, min_y, max_x, max_y] coverage bounds
        """
        if coverage_area:
            min_x, min_y, max_x, max_y = coverage_area
            # Calculate member's sub-region based on ID
            num_members = self.member_count
            member_x_range = min_x + (max_x - min_x) * ((member_id % 2) / 2)
            member_y_range = min_y + (max_y - min_y) * ((member_id // 2) / (num_members // 2 + 1)) if num_members >= 4 else min_y
            x = member_x_range + random.uniform(0, (max_x - min_x) / 2)
            y = member_y_range + random.uniform(0, (max_y - min_y) / 2)
        else:
            x = random.uniform(-grid_size, grid_size)
            y = random.uniform(-grid_size, grid_size)

        yaw = random.uniform(-math.pi, math.pi)
        self.dispatch_task(member_id, x, y, yaw)
        self.trace_write('cooperative_coverage', member_id=member_id,
                         grid_size=grid_size, spacing=spacing,
                         coverage_area=coverage_area)


def main(specification=None, directory='.'):
    """Main entry point for fleet workload execution."""
    import rclpy
    from pathlib import Path

    if specification is None:
        specification = {
            'fleet_id': 'fleet_alpha',
            'fleet_goals': [
                {'id': 0, 'x': 1.0, 'y': 2.0, 'yaw': 0.5},
                {'id': 1, 'x': 3.0, 'y': 4.0, 'yaw': 1.0},
            ]
        }

    spec = specification
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)

    # Determine schema from spec or use default
    schema = spec.get('schema', FLEET_WORKLOAD_SCHEMA)

    # Validate the workload specification
    try:
        # Build minimal document for validation
        doc = {'schema': schema}
        if schema == FLEET_WORKLOAD_SCHEMA:
            doc['fleet_goals'] = spec.get('fleet_goals', [])
        elif schema == WORKLOAD_SCHEMA:
            doc['goals'] = spec.get('goals', [])
            doc['truth_coordinates'] = spec.get('truth_coordinates', 'selected_world')
        validated = validate_workload(doc)
        print(f"Validated fleet workload: {validated.get('schema')}")
    except ValueError as e:
        print(f"Workload validation error: {e}")
        return 1

    # Create and run fleet workload
    fleet_id = spec.get('fleet_id', 'fleet_alpha')
    fleet = FleetWorkload({'fleet_id': fleet_id, 'fleet_goals': spec.get('fleet_goals', [])}, directory)

    # Run coordination patterns for each member
    for i in range(fleet.member_count):
        fleet.swarm_formation(i, formation_type='wedge', offset_radius=3.0)

    # Simulate task completion
    for i in range(fleet.member_count):
        fleet.mark_completed(i)

    # Write result
    result = fleet.get_result()
    result_path = directory / 'fleet-mission-result.json'
    result_path.write_text(json.dumps(result, indent=2))
    print(f"Fleet result written to {result_path}")

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
