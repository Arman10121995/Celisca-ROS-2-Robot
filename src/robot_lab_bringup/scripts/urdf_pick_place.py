#!/usr/bin/env python3
"""Pick and place for a fixed URDF arm on any backend (ROADMAP R5.7).

The plan comes from robot_lab_mujoco.arm_pick_place: a private MuJoCo
kinematics and collision model of the robot URDF and the world's static
boxes. It executes through the shared articulation contract
(``/molar/articulation/{state,command,heartbeat,stop}``). Object poses come
from simulator truth on ``/simulation/free_bodies`` (MuJoCo, PyBullet);
there is no perception here.

**Interface**

- ``/pick_place/request`` (String JSON ``{object, support, lift, approach}``)
  starts one task. A task is only ever started by an explicit request.
- ``/pick_place/stop`` (Trigger) stops the task and the articulation.
- ``/pick_place/state`` (String JSON, 2 Hz) reports the phase, stage,
  message, the objects and supports, and the last result.

**Execution**

- Arm and finger targets go out every ``PERIOD`` s, densified so that no
  joint is asked for more than ``RATE`` rad/s (below the contract's caps).
  Each stage then waits until the measured joints settle.

**Judged by truth**

- **Grasp:** after closing, the fingers stop on the object.
- **Lift:** the object rises with the tool and keeps its offset to it.
- **Release:** after the retreat, the object rests on the selected support:
  at its top, inside its footprint, not moving.

**Stopping and receipts**

- Stop, a stale articulation state, a lost plant or a failed check ends the
  task with Stop.
- Each task writes a JSON receipt under ``$ROBOT_LAB_RUNTIME_ROOT/pick-place/``.
"""
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

PERIOD = 0.1            # s between targets while moving
RATE = 0.3              # rad/s, below the arm channels' 0.35 cap
SETTLE_TOLERANCE = 0.01  # rad
STAGE_TIMEOUT = 30.0    # s per stage
STATE_STALE = 1.0       # s


def _now():
    return time.monotonic()


def _plain(value):
    """JSON fallback for numpy scalars and arrays in check results."""
    return value.tolist() if hasattr(value, 'tolist') else str(value)


class PickPlace(Node):
    def __init__(self):
        super().__init__('urdf_pick_place')
        for name, default in (('model_path', ''), ('articulation_config', ''), ('pick_place_config', ''),
                              ('world_path', ''), ('base_x', 0.0), ('base_y', 0.0), ('base_z', 0.0),
                              ('base_yaw', 0.0)):
            self.declare_parameter(name, default)
        self.config = json.loads(self.get_parameter('pick_place_config').value)
        articulation = json.loads(self.get_parameter('articulation_config').value)
        from robot_lab_mujoco.arm_pick_place import ArmSpec
        self.spec = ArmSpec.from_config(self.config)
        self.home = {name: float(articulation['home'][name]) for name in self.spec.arm_joints}
        # Coupled followers (the second Panda finger) are measured, not commanded.
        self.finger_masters = [n for n in self.spec.finger_joints if n in articulation['joints']]
        self.base = tuple(float(self.get_parameter(k).value) for k in ('base_x', 'base_y', 'base_z', 'base_yaw'))
        self.supports, self.object_sizes = self._world_boxes()
        self.robot_mjcf = None
        self.truth, self.truth_at = {}, -math.inf
        self.state, self.state_at = {}, -math.inf
        self.task = None
        self.last_result = None
        self.message = 'Select an object and a support, then request a pick and place.'
        self.create_subscription(String, '/simulation/free_bodies', self._on_truth, 10)
        self.create_subscription(String, '/molar/articulation/state', self._on_state, 10)
        self.create_subscription(String, '/pick_place/request', self._on_request, 1)
        self.command_pub = self.create_publisher(String, '/molar/articulation/command', 1)
        self.heartbeat_pub = self.create_publisher(Empty, '/molar/articulation/heartbeat', 1)
        self.state_pub = self.create_publisher(String, '/pick_place/state', 1)
        self.stop_client = self.create_client(Trigger, '/molar/articulation/stop')
        self.create_service(Trigger, '/pick_place/stop', self._on_stop)
        self.create_timer(PERIOD, self._tick)
        self.create_timer(0.5, self._publish_state)
        root = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'pick-place'
        root.mkdir(parents=True, exist_ok=True)
        self.receipts = root

    # --- inputs ---------------------------------------------------------------
    def _world_boxes(self):
        """Supports (static boxes) and loose box sizes from the world file."""
        from robot_lab_mujoco.arm_pick_place import Box
        from robot_lab_utils.sdf_world import extract_dynamic_bodies, extract_static_shapes
        path = self.get_parameter('world_path').value
        if not path or not os.path.isfile(path):
            return {}, {}
        sizes = {body['name']: list(body['shape']['size']) for body in extract_dynamic_bodies(path)
                 if body['shape']['type'] == 'box'}
        shapes, _ = extract_static_shapes(path, lambda uri, base: '', exclude_dynamic=True)
        boxes = {}
        for shape in shapes:
            # A support is a box a robot could set things on: under 1.2 m tall,
            # not the floor (wider than 2 m).
            if shape['type'] == 'box' and max(shape['size'][:2]) < 2.0 and shape['size'][2] < 1.2:
                boxes[shape['model']] = Box(shape['model'], shape['position'], shape['orientation'], shape['size'])
        return boxes, sizes

    def _on_truth(self, message):
        self.truth = json.loads(message.data)
        self.truth_at = _now()

    def _on_state(self, message):
        self.state = json.loads(message.data)
        self.state_at = _now()

    def measured(self):
        return {c['name']: c['measured'] for c in self.state.get('channels', []) if c.get('measured') is not None}

    def _publish_state(self):
        task = self.task or {}
        self.state_pub.publish(String(data=json.dumps(dict(
            schema='robot_lab.pick_place_state.v1', phase=task.get('phase', 'idle'), stage=task.get('stage'),
            message=self.message, objects=sorted(self.truth), supports=sorted(self.supports),
            last_result=self.last_result, truth_age_s=round(_now() - self.truth_at, 2) if self.truth else None),
            default=_plain)))

    # --- requests ---------------------------------------------------------------
    def _on_stop(self, request, response):
        self._finish('stopped', 'Stopped by request')
        response.success, response.message = True, 'stopped'
        return response

    def _on_request(self, message):
        if self.task is not None:
            self.message = 'A task is already running; Stop it first.'
            return
        try:
            request = json.loads(message.data)
            self.task = self._plan(request)
        except Exception as exc:  # noqa: BLE001 - reported to the operator
            self.task = None
            self.message = 'Not started: %s' % exc
            self.last_result = dict(passed=False, error=str(exc), request=message.data)

    def _plan(self, request):
        from robot_lab_mujoco.arm_pick_place import ArmScene, Box, plan_pick_place, synchronised
        name, support_name = request.get('object'), request.get('support')
        if name not in self.truth or _now() - self.truth_at > STATE_STALE:
            raise ValueError('no fresh truth for object %r' % name)
        if support_name not in self.supports:
            raise ValueError('unknown support %r (have %s)' % (support_name, sorted(self.supports)))
        if not self.state.get('initialized') or self.state.get('fault') or _now() - self.state_at > STATE_STALE:
            raise ValueError('articulation not ready: %s' % self.state.get('status'))
        measured = self.measured()
        missing = [n for n in self.spec.arm_joints + self.spec.finger_joints if n not in measured]
        if missing:
            raise ValueError('no measurement for %s' % missing)
        pose = self.truth[name]
        size = self.object_sizes.get(name)
        if not size:
            raise ValueError('object %r is not a loose box of the world file' % name)
        obj = Box(name, pose[:3], pose[3:], size)
        if self.robot_mjcf is None:
            from robot_lab_adapter.legged_training import _package_map
            from robot_lab_mujoco.mujoco_spawner import _build_mjcf_from_urdf
            urdf = Path(self.get_parameter('model_path').value).read_text()
            self.robot_mjcf = _build_mjcf_from_urdf(urdf, _package_map(urdf), robot_name='pick_place_robot')
        obstacles = list(self.supports.values())
        scene = ArmScene(self.robot_mjcf, self.spec, obstacles, obj, base=self.base)
        start = {n: float(measured[n]) for n in self.spec.arm_joints}
        lift = float(request.get('lift', 0.10))
        approach = float(request.get('approach', 0.10))
        if not 0.03 <= lift <= 0.25 or not 0.03 <= approach <= 0.25:
            raise ValueError('lift and approach must be 0.03-0.25 m')
        planned = plan_pick_place(scene, start, obj, self.supports[support_name], approach=approach, lift=lift,
                                  seeds=[self.home])
        stages, previous = [], dict(start)
        for stage in planned['stages']:
            targets = synchronised(stage['waypoints'], previous, RATE, PERIOD)
            previous = dict(stage['waypoints'][-1])
            stages.append(dict(name=stage['stage'], targets=targets,
                               fingers={n: v for n, v in stage['fingers'].items() if n in self.finger_masters},
                               finger_goal=stage['fingers']))
        self.message = 'Planned %d stages; executing.' % len(stages)
        return dict(phase='executing', stage=stages[0]['name'], stages=stages, index=0, step=0, request=request,
                    object=name, support=support_name, scene=scene, obj=obj, plan=dict(
                        grasp_width_m=planned['grasp_width_m'], place_centre=planned['place_centre']),
                    started=_now(), stage_started=_now(), log=[], object_start=list(pose[:3]))

    # --- execution ----------------------------------------------------------------
    def _send(self, targets):
        self.command_pub.publish(String(data=json.dumps(dict(plant_id=self.state.get('plant_id'), targets=targets))))

    def _tcp(self, task):
        scene = task['scene']
        measured = self.measured()
        scene.detach()
        scene.set_joints({n: measured[n] for n in self.spec.arm_joints + self.spec.finger_joints})
        return scene.tcp()

    def _tick(self):
        try:
            self._advance()
        except Exception as exc:  # noqa: BLE001 - a task error must stop the arm, not the node
            self.get_logger().error('pick and place error: %r' % exc)
            self._finish('failed', 'internal error: %s' % exc)

    def _advance(self):
        task = self.task
        if task is None:
            return
        self.heartbeat_pub.publish(Empty())
        if _now() - self.state_at > STATE_STALE or self.state.get('fault'):
            return self._finish('failed', 'articulation state stale or faulted: %s' % self.state.get('status'))
        stage = task['stages'][task['index']]
        if _now() - task['stage_started'] > STAGE_TIMEOUT:
            return self._finish('failed', 'stage %s timed out' % stage['name'])
        if task['step'] < len(stage['targets']):
            self._send(dict(stage['targets'][task['step']], **stage['fingers']))
            task['step'] += 1
            return
        if not self._settled(stage):
            self._send(dict(stage['targets'][-1], **stage['fingers']))
            return
        check = self._check(task, stage['name'])
        if check is None:
            return                              # still waiting on a timed observation
        tcp = self._tcp(task)[0]
        task['log'].append(dict(stage=stage['name'], seconds=round(_now() - task['stage_started'], 2),
                                object_truth=[round(v, 4) for v in self.truth[task['object']][:3]],
                                tcp_estimate=[round(float(v), 4) for v in tcp], **check))
        if not check.get('passed', True):
            return self._finish('failed', '%s: %s' % (stage['name'], check.get('reason')))
        task['index'] += 1
        task['step'] = 0
        task['stage_started'] = _now()
        task.pop('wait_until', None)
        if task['index'] >= len(task['stages']):
            return self._finish('succeeded', 'Placed %s on %s' % (task['object'], task['support']))
        task['stage'] = task['stages'][task['index']]['name']
        self.message = 'Stage %s' % task['stage']

    def _settled(self, stage):
        measured = self.measured()
        final = stage['targets'][-1]
        if any(abs(measured[n] - final[n]) > SETTLE_TOLERANCE for n in final):
            return False
        # Fingers settle on target, or once stopped (held by the object) after
        # they had time to start moving.
        rates = {c['name']: c.get('measured_velocity') or 0.0 for c in self.state.get('channels', [])}
        on_target = all(abs(measured[n] - stage['finger_goal'][n]) < 0.002 for n in self.spec.finger_joints)
        stopped = all(abs(rates.get(n, 0.0)) < 0.002 for n in self.spec.finger_joints)
        return on_target or (stopped and _now() - self.task['stage_started'] > 1.0)

    def _check(self, task, stage):
        """Truth checks after a stage; None while a timed observation is still running."""
        name = task['object']
        if _now() - self.truth_at > STATE_STALE:
            return dict(passed=False, reason='object truth stale')
        pose = np.asarray(self.truth[name][:3])
        if stage == 'close':
            measured = self.measured()
            gaps = [measured[n] for n in self.spec.finger_joints]
            width = task['plan']['grasp_width_m']
            # Each finger stopped against the object, not at its closed limit.
            gripping = all(g > 0.25 * width / 2 for g in gaps)
            return dict(passed=gripping, finger_positions=[round(g, 4) for g in gaps],
                        reason='fingers closed past the object' if not gripping else '')
        if stage == 'lift':
            if 'wait_until' not in task:
                task['wait_until'] = _now() + 1.0       # hold one second unsupported
                position, rotation = self._tcp(task)
                task['held_offset'] = (rotation.T @ (pose - position)).tolist()
                return None
            if _now() < task['wait_until']:
                return None
            position, rotation = self._tcp(task)
            offset = rotation.T @ (pose - position)
            drift = float(np.linalg.norm(offset - np.asarray(task['held_offset'])))
            rise = float(pose[2] - task['object_start'][2])
            lift = float(task['request'].get('lift', 0.10))
            passed = rise > 0.8 * lift and drift < 0.01
            return dict(passed=passed, rise_m=round(rise, 4), hold_drift_m=round(drift, 4),
                        reason='object did not rise with the tool' if not passed else '')
        if stage == 'retreat':
            if 'wait_until' not in task:
                task['wait_until'] = _now() + 1.0
                task['rest_sample'] = pose.tolist()
                return None
            if _now() < task['wait_until']:
                return None
            support = self.supports[task['support']]
            half = float(task['obj'].size[2]) / 2.0
            local = self._in_support(support, pose)
            speed = float(np.linalg.norm(pose - np.asarray(task['rest_sample'])))
            on_top = abs(pose[2] - (support.top + half)) < 0.01
            inside = abs(local[0]) <= support.size[0] / 2.0 and abs(local[1]) <= support.size[1] / 2.0
            passed = on_top and inside and speed < 0.005
            return dict(passed=passed, final_pose=[round(v, 4) for v in pose], height_error_m=round(pose[2] - support.top - half, 4),
                        inside_support=inside, moved_last_s_m=round(speed, 4),
                        reason='' if passed else 'object not resting on %s' % support.name)
        return dict(passed=True)

    @staticmethod
    def _in_support(support, point):
        from robot_lab_mujoco.arm_pick_place import yaw_of
        yaw = yaw_of(support.quaternion)
        dx, dy = point[0] - support.position[0], point[1] - support.position[1]
        return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy)

    def _finish(self, outcome, message):
        task, self.task = self.task, None
        if outcome != 'succeeded' and self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
        self.message = message
        if task is None:
            return
        result = dict(schema='robot_lab.pick_place_result.v1', outcome=outcome, passed=outcome == 'succeeded',
                      message=message, request=task['request'], object=task['object'], support=task['support'],
                      object_start=task['object_start'], plan=task['plan'], stages=task['log'],
                      seconds=round(_now() - task['started'], 2), plant_id=self.state.get('plant_id'),
                      model_sha256=self.state.get('model_sha256'), finished=time.strftime('%Y-%m-%dT%H:%M:%S'))
        path = self.receipts / ('%s_%s.json' % (time.strftime('%Y%m%dT%H%M%S'), task['object']))
        path.write_text(json.dumps(result, indent=1, default=_plain) + '\n')
        result['receipt'] = str(path)
        self.last_result = result
        self.get_logger().info('pick and place %s: %s (%s)' % (outcome, message, path))


def main():
    rclpy.init()
    node = PickPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
