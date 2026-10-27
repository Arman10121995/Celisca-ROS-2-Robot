"""Physical contracts for native marine, gait and multi-FCU experiments."""
import math

SCHEMA = 'robot_lab.extension_workload.v1'
RESULT_SCHEMA = 'robot_lab.measured_extension_result.v1'
CONTRACTS = {
    'marine': dict(position_m=.2, linear_m_s=.05, settle_sim_s=1.),
    'auv': dict(position_m=.6, linear_m_s=.05, settle_sim_s=1.),
    'gait': dict(position_m=.15, heading_rad=.15, linear_m_s=.03, angular_rad_s=.05, settle_sim_s=1.),
    'aerial_swarm': dict(position_m=.35, linear_m_s=.15, settle_sim_s=2.),
}


def validate(document):
    workflow, task = document.get('workflow'), document.get('task')
    if workflow not in ('marine', 'gait', 'aerial_swarm'):
        raise ValueError('Choose native marine, gait or aerial_swarm workflow.')
    if task not in ({'formation'} if workflow == 'aerial_swarm' else {'navigation', 'mapping'}):
        raise ValueError('Choose formation, navigation or mapping for this workflow.')
    if document.get('truth_coordinates') != 'selected_world':
        raise ValueError('Extension truth must explicitly use selected_world coordinates.')
    timeout = float(document.get('timeout_s', 300.))
    if not math.isfinite(timeout) or not 30 <= timeout <= 900:
        raise ValueError('Extension workload timeout must be 30–900 wall seconds.')
    result = dict(document, timeout_s=timeout)
    if task == 'navigation':
        goals = document.get('goals')
        if not isinstance(goals, list) or not 1 <= len(goals) <= 32:
            raise ValueError('Provide 1–32 world XYZ marine or world XY/yaw gait goals.')
        result['goals'] = []
        for goal in goals:
            if not isinstance(goal, list) or len(goal) != 3:
                raise ValueError('Each goal has three coordinates.')
            values = list(map(float, goal))
            if not all(map(math.isfinite, values)) or max(map(abs, values)) > 10000:
                raise ValueError('Finite bounded goal coordinates required.')
            if workflow == 'gait':
                values[2] = math.atan2(math.sin(values[2]), math.cos(values[2]))
            result['goals'].append(values)
    if task == 'mapping':
        distance = float(document.get('distance_m', .2))
        if not math.isfinite(distance) or not .15 <= distance <= 5.:
            raise ValueError('Map acquisition requires .15–5 m of actual physical travel.')
        result['distance_m'] = distance
    return result


def endpoint_ok(pose, target, speed, yaw_rate, contract):
    if pose is None or speed is None:
        return False
    values = [*pose, *target, speed]
    if not all(map(math.isfinite, values)) or speed < 0 or speed > contract['linear_m_s']:
        return False
    if 'heading_rad' in contract:
        error = abs(math.atan2(math.sin(pose[2]-target[2]), math.cos(pose[2]-target[2])))
        return (math.dist(pose[:2], target[:2]) <= contract['position_m']
                and error <= contract['heading_rad'] and yaw_rate is not None
                and math.isfinite(yaw_rate) and abs(yaw_rate) <= contract['angular_rad_s'])
    return math.dist(pose, target) <= contract['position_m']
