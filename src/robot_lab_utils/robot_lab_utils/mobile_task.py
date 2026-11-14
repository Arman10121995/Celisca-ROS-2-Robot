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
        raise ValueError('A task must contain 1–32 navigation, articulation or object checkpoints.')
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
        elif step.get('kind') in ('object_check', 'reach_object', 'grip', 'stow_arm'):
            if step['kind']=='object_check' and step.get('condition') not in ('grasped', 'released'):
                raise ValueError('Object condition must be grasped or released.')
            for key in ('object', 'left_body', 'right_body'):
                value = step.get(key)
                if not isinstance(value, str) or not value.strip() or len(value) > 200:
                    raise ValueError('Choose an exact model body name for '+key)
            if len({step[key] for key in ('object', 'left_body', 'right_body')}) != 3:
                raise ValueError('Object and opposing gripper bodies must be distinct.')
            if step['kind']=='reach_object':
                if step.get('pose_source','native_truth') not in ('native_truth','aruco_pnp','rgbd_box_top'):
                    raise ValueError('Choose native geometry, camera markers or RGB-D boxes')
                if step.get('pose_source') in ('aruco_pnp','rgbd_box_top') and 'target_xyz' in step:
                    raise ValueError('Camera-relative object reach cannot also override world XYZ')
                offset = step.get('offset', [0., 0., 0.])
                if not isinstance(offset, list) or len(offset)!=3:
                    raise ValueError('Reach offset must be native-world XYZ metres')
                offset = list(map(float, offset))
                if not all(map(math.isfinite, offset)) or max(map(abs, offset)) > .5:
                    raise ValueError('Reach offsets must be finite and within 0.5 m')
                step['offset'] = offset
                if not isinstance(step.get('keep_orientation',False),bool):
                    raise ValueError('Keep-orientation must be a boolean')
                if 'orientation_wxyz' in step:
                    orientation=step['orientation_wxyz']
                    if not isinstance(orientation,list) or len(orientation)!=4:
                        raise ValueError('Pad orientation must be a world wxyz quaternion')
                    orientation=list(map(float,orientation))
                    norm=math.sqrt(sum(v*v for v in orientation))
                    if not all(map(math.isfinite,orientation)) or not .9<=norm<=1.1:
                        raise ValueError('Finite unit world pad orientation required')
                    if step.get('keep_orientation'):
                        raise ValueError('Choose measured or explicit orientation, not both')
                    step['orientation_wxyz']=[v/norm for v in orientation]
                if 'target_xyz' in step:
                    target = step['target_xyz']
                    if not isinstance(target,list) or len(target)!=3:
                        raise ValueError('World reach target must be XYZ metres')
                    target = list(map(float,target))
                    if not all(map(math.isfinite,target)) or max(map(abs,target)) > 10000:
                        raise ValueError('Finite bounded world reach target required')
                    step['target_xyz'] = target
            if step['kind']=='grip' and step.get('operation') not in ('open', 'close'):
                raise ValueError('Gripper operation must be open or close')
        else:
            raise ValueError('Implemented steps are navigate, articulate, object_check, reach_object, grip and stow_arm.')
        output.append(step)
    return output
