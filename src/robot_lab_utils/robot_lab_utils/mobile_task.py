"""Source-bound mobile task documents shared by the GUI and native executor."""
import copy
import math


SCHEMA = 'robot_lab.native_mobile_task.v1'


def validate_task(document, source_sha256, scene_id, channels, allow_navigation):
    if not isinstance(document, dict) or document.get('schema') != SCHEMA:
        raise ValueError('Choose a Robot Lab mobile task document.')
    if document.get('source_sha256') != source_sha256 or document.get('scene_id') != scene_id:
        raise ValueError('Task must match the running native robot and selected collision scene.')
    steps = document.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= 32:
        raise ValueError('A task must contain 1–32 navigation or articulation steps.')
    known = {channel['name']: channel for channel in channels}
    output = []
    for original in steps:
        if not isinstance(original, dict):
            raise ValueError('Each task step must be an object.')
        step = copy.deepcopy(original)
        step['name'] = str(step.get('name', step.get('kind', 'Step')))[:80]
        if step.get('kind') == 'navigate':
            if not allow_navigation:
                raise ValueError('Navigation steps require the explicit native nav workflow.')
            pose = step.get('pose')
            if not isinstance(pose, list) or len(pose) != 3:
                raise ValueError('Navigation pose is [map X, map Y, yaw radians].')
            pose = [float(value) for value in pose]
            if not all(map(math.isfinite, pose)) or max(abs(v) for v in pose[:2]) > 10000:
                raise ValueError('Finite, bounded navigation coordinates required.')
            pose[2] = math.atan2(math.sin(pose[2]), math.cos(pose[2]))
            step['pose'] = pose
        elif step.get('kind') == 'articulate':
            points = step.get('waypoints')
            if not isinstance(points, list) or not 1 <= len(points) <= 64:
                raise ValueError('Articulation steps require 1–64 taught waypoints.')
            for point in points:
                if not isinstance(point, dict) or not isinstance(point.get('targets'), dict):
                    raise ValueError('Articulation waypoints require named source targets.')
                if set(point['targets']) != set(known):
                    raise ValueError('Taught targets must include every running actuator channel.')
                dwell = float(point.get('settle_s', .5))
                if not math.isfinite(dwell) or not .2 <= dwell <= 5:
                    raise ValueError('Settling duration must be 0.2–5 simulated seconds.')
                point['settle_s'] = dwell
                for name, value in point['targets'].items():
                    value = float(value)
                    if not math.isfinite(value) or not known[name]['limits'][0] <= value <= known[name]['limits'][1]:
                        raise ValueError('Taught target exceeds source bounds: '+name)
                    point['targets'][name] = value
        else:
            raise ValueError('Implemented steps are navigate and articulate.')
        output.append(step)
    return output
