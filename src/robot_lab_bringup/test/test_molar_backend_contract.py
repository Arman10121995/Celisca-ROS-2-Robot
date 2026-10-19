"""Execute real backend contract functions with SDK/ROS-free recording stubs.

These source regressions do not launch a plant or establish physical motion.
"""
import ast
import math
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import xml.etree.ElementTree as ET

import pytest


SRC = Path(__file__).resolve().parents[2]
SPAWNERS = {
    backend: SRC/('robot_lab_'+backend)/'python'/('robot_lab_'+backend)/(backend+'_spawner.py')
    for backend in ('mujoco', 'pybullet', 'isaac')}
RUNTIME = SRC/'robot_lab_isaac/python/robot_lab_isaac/isaac_runtime.py'
ARTICULATION = SRC/'robot_lab_utils/robot_lab_utils/urdf_articulation.py'


def source_function(path, name):
    tree = ast.parse(path.read_text())
    matches = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == name]
    assert len(matches) == 1, (path, name)
    return matches[0]


def load_nodes(nodes, namespace):
    module = ast.Module(body=nodes, type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), '<isolated-backend-contract>', 'exec'), namespace)
    return namespace


class Twist:
    def __init__(self, forward=0., yaw=0.):
        self.linear = SimpleNamespace(x=forward, y=0., z=0.)
        self.angular = SimpleNamespace(x=0., y=0., z=yaw)


def callback_subject(backend):
    clock = SimpleNamespace(now=100.)
    names = ['_on_cmd', '_on_mux_cmd'] + (['_accept_cmd'] if backend == 'isaac' else [])
    methods = [source_function(SPAWNERS[backend], name) for name in names]
    wrapper = ast.ClassDef(name='Callbacks', bases=[], keywords=[], body=methods, decorator_list=[])
    scope = load_nodes([wrapper], dict(math=math, Twist=Twist,
        time=SimpleNamespace(monotonic=lambda: clock.now)))
    subject = scope['Callbacks']()
    subject._articulation = SimpleNamespace(base_inhibited=True)
    subject._twist_lock = threading.Lock()
    subject._twist = Twist(.2, .3)
    subject._last_cmd_time = 99.9
    subject._last_mux_cmd_time = -math.inf
    return subject, clock


@pytest.mark.parametrize('backend', tuple(SPAWNERS))
@pytest.mark.parametrize('callback', ('_on_cmd', '_on_mux_cmd'))
def test_blocked_drive_is_discarded_and_requires_new_input_after_retraction(backend, callback):
    subject, clock = callback_subject(backend)
    rejected = Twist(.25, .5)
    getattr(subject, callback)(rejected)
    assert subject._twist is not rejected
    assert subject._twist.linear.x == subject._twist.angular.z == 0.
    assert subject._last_cmd_time == -math.inf
    subject._articulation.base_inhibited = False
    # Retraction alone cannot revive a message received while inhibited.
    assert subject._twist.linear.x == 0. and clock.now-subject._last_cmd_time > .5
    clock.now += 1.1  # also expire the existing mux-priority window
    fresh = Twist(.1, -.2)
    getattr(subject, callback)(fresh)
    assert subject._twist is fresh
    assert subject._last_cmd_time == clock.now


@pytest.mark.parametrize('backend', tuple(SPAWNERS))
def test_ordinary_profiles_without_articulation_keep_drive_callback(backend):
    subject, clock = callback_subject(backend)
    subject._articulation = None
    message = Twist(.1, .2)
    subject._on_mux_cmd(message)
    assert subject._twist is message and subject._last_cmd_time == clock.now


@pytest.mark.parametrize('backend,method', (
    ('mujoco', '_step_physics'), ('pybullet', '_loop'), ('isaac', '_send_drive_targets')))
def test_physics_inhibit_also_discards_previously_accepted_command(backend, method):
    function = source_function(SPAWNERS[backend], method)
    guards = [node for node in ast.walk(function) if isinstance(node, ast.If)
              and any(isinstance(part, ast.Attribute) and part.attr == 'base_inhibited'
                      for part in ast.walk(node.test))]
    assert len(guards) == 1
    wrapper = ast.parse('def inhibit(self):\n    pass').body[0]
    wrapper.body = [guards[0]]
    scope = load_nodes([wrapper], dict(math=math, Twist=Twist))
    reset_calls = []
    subject = SimpleNamespace(_twist=Twist(.25, .5), _last_cmd_time=100.,
        _articulation=SimpleNamespace(base_inhibited=True), _twist_lock=threading.Lock(),
        _drive=SimpleNamespace(reset=lambda: reset_calls.append(True)))
    scope['inhibit'](subject)
    assert subject._twist.linear.x == subject._twist.angular.z == 0.
    assert subject._last_cmd_time == -math.inf and reset_calls == [True]


class Attribute:
    def __init__(self, value=None):
        self.value = value

    def Set(self, value):
        self.value = value


class Drive:
    def __init__(self):
        # Exercise existing SDK attributes and the Create fallback.
        self.attributes = {'Stiffness': Attribute(100.), 'TargetPosition': Attribute(2.)}

    def __getattr__(self, name):
        if name.startswith('Get') and name.endswith('Attr'):
            key = name[3:-4]
            return lambda: self.attributes.get(key)
        if name.startswith('Create') and name.endswith('Attr'):
            key = name[6:-4]
            return lambda: self.attributes.setdefault(key, Attribute())
        raise AttributeError(name)


class Prim:
    def __init__(self, name, kind, proxy=False):
        self.name, self.kind, self.proxy = name, kind, proxy
        self.drive = Drive()

    def GetName(self):
        return self.name

    def IsInstanceProxy(self):
        return self.proxy

    def IsA(self, kind):
        return self.kind == kind


def caster_author(monkeypatch):
    revolute = object()
    applied = []
    def apply(prim, axis):
        applied.append((prim.name, axis))
        return prim.drive
    pxr = ModuleType('pxr')
    pxr.Usd = SimpleNamespace(TraverseInstanceProxies=lambda: object())
    pxr.UsdPhysics = SimpleNamespace(RevoluteJoint=revolute,
        DriveAPI=SimpleNamespace(Apply=apply))
    monkeypatch.setitem(sys.modules, 'pxr', pxr)
    scope = load_nodes([source_function(RUNTIME, '_author_passive_caster_drives')], dict(math=math))
    return scope['_author_passive_caster_drives'], revolute, applied


def source_casters(profile):
    root = ET.parse(SRC/'robot_lab_robots'/profile/'urdf'/(profile+'.urdf.xacro')).getroot()
    return {joint.get('name'): dict(damping=float(joint.find('dynamics').get('damping')),
        effort=float(joint.find('limit').get('effort')))
        for joint in root.findall('joint') if joint.get('name', '').startswith('caster_')}


@pytest.mark.parametrize('profile', ('probot_l', 'molar_hri'))
def test_native_caster_drives_remove_centering_and_keep_source_bearing_limits(profile, monkeypatch):
    author, revolute, applied = caster_author(monkeypatch)
    specifications = source_casters(profile)
    assert len(specifications) == 8
    prims = [Prim(name, revolute) for name in specifications]
    unrelated = [Prim(name, revolute) for name in ('drive_left_joint', 'conveyor_lift_joint', 'transfer_roller_joint')]
    stage = SimpleNamespace(Traverse=lambda _: prims+unrelated)
    assert author(stage, specifications) == sorted(specifications)
    assert sorted(applied) == sorted((name, 'angular') for name in specifications)
    for prim in prims:
        values = {name: attribute.value for name, attribute in prim.drive.attributes.items()}
        assert values['Type'] == 'force'
        assert values['Stiffness'] == values['TargetPosition'] == values['TargetVelocity'] == 0.
        assert values['Damping'] == pytest.approx(specifications[prim.name]['damping']*math.pi/180.)
        assert values['MaxForce'] == specifications[prim.name]['effort']
    assert all(prim.drive.attributes['Stiffness'].value == 100. for prim in unrelated)


@pytest.mark.parametrize('defect', ('missing', 'proxy', 'wrong_type'))
def test_missing_native_caster_joint_fails_instead_of_silently_pinning(defect, monkeypatch):
    author, revolute, _ = caster_author(monkeypatch)
    specifications = {'caster_front_left_spin_joint': dict(damping=.01, effort=1.)}
    prims = [] if defect == 'missing' else [Prim('caster_front_left_spin_joint',
        object() if defect == 'wrong_type' else revolute, proxy=defect == 'proxy')]
    with pytest.raises(ValueError, match='Passive caster joints missing'):
        author(SimpleNamespace(Traverse=lambda _: prims), specifications)


def test_bullet_roller_uses_native_velocity_motor_with_source_force_cap():
    loop = source_function(SPAWNERS['pybullet'], '_loop')
    guards = [node for node in ast.walk(loop) if isinstance(node, ast.If)
              and isinstance(node.test, ast.Compare) and isinstance(node.test.left, ast.Name)
              and node.test.left.id == 'kind' and any(isinstance(value, ast.Constant)
                  and value.value == 'velocity' for value in node.test.comparators)]
    assert len(guards) == 1
    wrapper = ast.parse('def apply_roller(self):\n    for once in (None,):\n        pass').body[0]
    wrapper.body[0].body = [guards[0]]
    calls = []
    bullet = SimpleNamespace(VELOCITY_CONTROL=7,
        setJointMotorControl2=lambda *args, **kwargs: calls.append((args, kwargs)))
    scope = load_nodes([wrapper], dict(p=bullet, kind='velocity', name='transfer_roller_joint',
        target=8., spec=dict(effort=2.)))
    scope['apply_roller'](SimpleNamespace(_robot_id=3, _joint_idx={'transfer_roller_joint': 4}))
    assert calls == [((3, 4, bullet.VELOCITY_CONTROL),
        dict(targetVelocity=8., force=2., velocityGain=1.))]


@pytest.mark.parametrize('force', (139.302, -139.302, 3000., -3000., 0.))
def test_mujoco_servo_and_gravity_sum_respects_source_effort(force):
    import numpy as np
    scope = load_nodes([source_function(ARTICULATION, 'effort_budget'),
        source_function(SPAWNERS['mujoco'], '_apply_articulation_gravity')], dict(math=math))
    joint = 'conveyor_lift_joint'
    ranges = np.array([[-2000., 2000.], [-2., 2.], [-30., 30.]])
    applied = np.array([0., 7., 8.])
    subject = SimpleNamespace(
        _articulation=SimpleNamespace(contract=SimpleNamespace(joints={
            joint: dict(mode='position', effort=2000.),
            'transfer_roller_joint': dict(mode='velocity', effort=2.)})),
        _joint_name2dofadr={joint: 0, 'transfer_roller_joint': 1},
        _actuator_ids={joint+'_articulation': 0},
        _model=SimpleNamespace(actuator_forcerange=ranges),
        _data=SimpleNamespace(qfrc_applied=applied, qfrc_bias=np.array([force, 0., 0.])))
    scope['_apply_articulation_gravity'](subject)
    ff, cap = scope['effort_budget'](force, 2000.)
    assert applied[0] == ff and ranges[0] == pytest.approx((-cap, cap))
    assert max(abs(ff+servo) for servo in ranges[0]) <= 2000.+1e-9
    assert applied[1:] == pytest.approx((7., 8.))
    assert ranges[1:] == pytest.approx(np.array([[-2., 2.], [-30., 30.]]))


@pytest.mark.parametrize('force', (139.302, -139.302, 3000., -3000., 0.))
def test_isaac_live_drive_budget_precedes_feedforward_and_preserves_other_joints(force):
    scope = load_nodes([source_function(ARTICULATION, 'effort_budget'),
        source_function(RUNTIME, '_apply_articulation_effort_budget')], dict(math=math))
    calls = []
    controller = SimpleNamespace(set_max_efforts=lambda values, joint_indices:
        calls.append(('limit', values.tolist(), joint_indices.tolist())))
    robot = SimpleNamespace(get_articulation_controller=lambda: controller,
        set_joint_efforts=lambda values, joint_indices:
            calls.append(('force', values.tolist(), joint_indices.tolist())))
    joints = {'conveyor_lift_joint': dict(mode='position', effort=2000.),
              'transfer_roller_joint': dict(mode='velocity', effort=2.)}
    names = ['drive_left_joint', 'transfer_roller_joint', 'conveyor_lift_joint', 'caster_spin_joint']
    scope['_apply_articulation_effort_budget'](robot, names, joints,
        {'conveyor_lift_joint': force, 'drive_left_joint': 100000.})
    ff, cap = scope['effort_budget'](force, 2000.)
    assert calls == [('limit', [cap], [2]), ('force', [ff], [2])]


def test_isaac_missing_feedforward_clears_force_and_restores_budget():
    scope = load_nodes([source_function(RUNTIME, '_apply_articulation_effort_budget')], dict(math=math))
    calls = []
    controller = SimpleNamespace(set_max_efforts=lambda values, joint_indices:
        calls.append(('limit', values.tolist())))
    robot = SimpleNamespace(get_articulation_controller=lambda: controller,
        set_joint_efforts=lambda values, joint_indices: calls.append(('force', values.tolist())))
    scope['_apply_articulation_effort_budget'](robot, ['lift'],
        {'lift': dict(mode='position', effort=2000.)}, {})
    assert calls == [('limit', [2000.]), ('force', [0.])]


def test_isaac_watchdog_hold_retains_gravity_budget():
    tree = ast.parse(RUNTIME.read_text())
    holds = [node for node in ast.walk(tree) if isinstance(node, ast.If)
             and any(isinstance(part, ast.Name) and part.id == 'articulation_watchdog_hold'
                     for part in ast.walk(node.test))]
    # Execute the actual stale-target branch with an already captured hold.
    branch = next(node for node in holds if isinstance(node.test, ast.Compare)
                  and any(isinstance(part, ast.Constant) and part.value is None
                          for part in ast.walk(node.test)))
    parent = next(node for node in ast.walk(tree) if isinstance(node, ast.If) and branch in node.body)
    wrapper = ast.parse('def watchdog(articulation_watchdog_hold, cfg, reader):\n    pass').body[0]
    wrapper.body = parent.body
    wrapper.body.append(ast.Return(value=ast.Name(id='targets', ctx=ast.Load())))
    scope = load_nodes([wrapper], {})
    result = scope['watchdog']({'position': {'lift': .2}, 'velocity': {'roller': 0.}},
        {'drive_wheel_joints': ['wheel']},
        SimpleNamespace(joint_targets={'effort': {'lift': 139.302}}))
    assert result['effort'] == {'lift': 139.302}
    assert result['velocity'] == {'wheel': 0., 'roller': 0.}


@pytest.mark.parametrize('motion', ('x', 'yaw'))
def test_isaac_reversing_base_motion_still_blocks_articulation(motion):
    scope = load_nodes([source_function(SPAWNERS['isaac'], '_base_moving')], dict(math=math))
    subject = SimpleNamespace(_base_history=[])
    for index in range(17):
        t = index/32
        wave = .01*math.sin(2*math.pi*t/.25)
        yaw = wave*3 if motion == 'yaw' else 0.
        measured = dict(t=t, pos=[wave if motion == 'x' else 0., 0., 0.],
            orn=[0., 0., math.sin(yaw/2), math.cos(yaw/2)], lin=[0., 0., 0.], ang=[0., 0., 0.])
        moving = scope['_base_moving'](subject, measured)
    assert moving


def test_isaac_rest_noise_and_yaw_wrap_do_not_report_a_reversal():
    scope = load_nodes([source_function(SPAWNERS['isaac'], '_base_moving')], dict(math=math))
    subject = SimpleNamespace(_base_history=[])
    for index in range(33):
        wave = .0001*math.sin(index*1.7)
        yaw = math.pi+wave
        measured = dict(t=index/32, pos=[wave, -wave, 0.],
            orn=[0., 0., math.sin(yaw/2), math.cos(yaw/2)], lin=[.03, 0., 0.], ang=[0., 0., .06])
        moving = scope['_base_moving'](subject, measured)
    assert not moving
