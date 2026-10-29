import math
import time
import unittest.mock as mock

import pytest
from geometry_msgs.msg import Twist

from robot_lab_adapter.aerial_swarm_controller import AerialSwarmController


def _fresh(row, now=5.0):
    return (now - row['status_at'] < 2.0 and
            now - row['pose_at'] < 1.5 and
            row['status'] and row['status'].get('position_ready'))


def _member(member_id, phase='flying', status_age=0.0, pose_age=0.0, position_ready=True):
    now = 5.0
    st = dict(phase=phase, position_ready=position_ready,
              status_at=now - status_age, pose_at=now - pose_age)
    return dict(id=member_id, namespace='/swarm/air' + member_id, status=st,
                status_at=now - status_age, pose_at=now - pose_age,
                pose=[0., 0., 0.], yaw=0., truth=None,
                truth_at=float('-inf'), result=None)


@pytest.mark.parametrize('status_age,pose_age,phase,position_ready,expected', [
    (0.0, 0.0, 'flying', True, True),
    (2.1, 0.0, 'flying', True, False),
    (0.0, 1.6, 'flying', True, False),
    (0.0, 0.0, 'holding', True, True),
    (0.0, 0.0, 'flying', False, False),
    (0.0, 0.0, None, True, False),
    (0.0, 0.0, 'flying', True, True),
])
def test_fresh_predicate(status_age, pose_age, phase, position_ready, expected):
    now = 5.0
    st = dict(phase=phase, position_ready=position_ready) if phase is not None else None
    row = dict(status_at=now - status_age, pose_at=now - pose_age, status=st)
    assert bool(_fresh(row)) is expected


def test_member_loss_computation():
    now = 5.0
    member_fresh = _member('air1', phase='flying', status_age=0.0, pose_age=0.0)
    member_stale = _member('air2', phase='flying', status_age=2.1, pose_age=0.0)
    member_holding = _member('air3', phase='holding', status_age=0.0, pose_age=0.0)
    members = [member_fresh, member_stale, member_holding]
    lost = [m for m in members if not _fresh(m, now=now) or m['status']['phase'] != 'flying']
    assert [m['id'] for m in lost] == ['air2', 'air3']


def test_member_loss_computation_all_fresh():
    members = [_member('air1', phase='flying', status_age=0.0, pose_age=0.0),
               _member('air2', phase='flying', status_age=0.0, pose_age=0.0)]
    lost = [m for m in members if not _fresh(m, now=5.0) or m['status']['phase'] != 'flying']
    assert lost == []


def test_fresh_status_age_boundary():
    now = 5.0
    row = dict(status_at=5.0 - 1.99, pose_at=5.0, status=dict(phase='flying', position_ready=True))
    assert _fresh(row, now=5.0) is True
    row = dict(status_at=5.0 - 2.01, pose_at=5.0, status=dict(phase='flying', position_ready=True))
    assert _fresh(row, now=5.0) is False


def test_fresh_pose_age_boundary():
    now = 5.0
    row = dict(status_at=5.0, pose_at=5.0 - 1.49, status=dict(phase='flying', position_ready=True))
    assert _fresh(row, now=5.0) is True
    row = dict(status_at=5.0, pose_at=5.0 - 1.51, status=dict(phase='flying', position_ready=True))
    assert _fresh(row, now=5.0) is False


def _formation_ctrl(now=None):
    if now is None:
        now = time.monotonic()
    ctrl = object.__new__(AerialSwarmController)
    ctrl.mode = 'formation'
    ctrl.anchor = [0.0, 0.0, 0.95]
    ctrl.targets = ['air1', 'air2']
    ctrl.spacing = 3.0
    ctrl.hb_at = now
    ctrl.operator = ''
    ctrl.generation = 0
    ctrl.command_at = -math.inf
    ctrl.pending = []
    ctrl.member_lost = {}
    ctrl.member_lost_at = {}
    ctrl.pub = mock.MagicMock()
    ctrl.lease = mock.MagicMock()
    ctrl.scene = mock.MagicMock()
    ctrl.scene.segment.return_value = True
    ctrl.event = mock.MagicMock()
    ctrl.formation_errors = []
    ctrl.altitude_errors = []
    ctrl.last_status = 0.0
    ctrl.reason = 'idle'
    ctrl.session = {'session_id': 'test-session', 'map': 'nav_empty', 'directory': '/tmp/test-session'}
    return ctrl


def _formation_member(member_id, index, target_y, pose_z, now, origin_z=0.5):
    return dict(
        id=member_id, instance=index, system_id=index + 1, port=14580 + index,
        namespace='/swarm/' + member_id,
        status=dict(phase='flying', position_ready=True),
        status_at=now, pose_at=now,
        pose=[0.0, target_y, pose_z], yaw=0.0,
        truth=[0.0, target_y, pose_z], truth_at=now, result=None,
        origin=[0.0, 0.0, origin_z],
        velocity_pub=mock.MagicMock())


def test_formation_altitude_trim_climbs_below_target():
    now = time.monotonic()
    ctrl = _formation_ctrl(now)
    ctrl.members = {
        'air1': _formation_member('air1', 0, -1.5, 0.85, now),
        'air2': _formation_member('air2', 1, 1.5, 1.05, now),
    }
    ctrl.tick()
    v1 = ctrl.members['air1']['velocity_pub'].publish.call_args[0][0]
    v2 = ctrl.members['air2']['velocity_pub'].publish.call_args[0][0]
    assert isinstance(v1, Twist) and isinstance(v2, Twist)
    # Velocities() already supplies half the z error; the trim adds a bounded
    # nudge toward the shared anchor height.
    assert v1.linear.z == pytest.approx(0.075, abs=1e-6)
    assert v2.linear.z == pytest.approx(-0.075, abs=1e-6)
    assert ctrl.members['air1']['last_altitude_trim_m_s'] == pytest.approx(0.025)
    assert ctrl.members['air2']['last_altitude_trim_m_s'] == pytest.approx(-0.025)


def test_formation_altitude_trim_deadband():
    now = time.monotonic()
    ctrl = _formation_ctrl(now)
    ctrl.members = {
        'air1': _formation_member('air1', 0, -1.5, 0.91, now),
        'air2': _formation_member('air2', 1, 1.5, 0.99, now),
    }
    ctrl.tick()
    v1 = ctrl.members['air1']['velocity_pub'].publish.call_args[0][0]
    v2 = ctrl.members['air2']['velocity_pub'].publish.call_args[0][0]
    assert v1.linear.z == pytest.approx(0.02, abs=1e-6)
    assert v2.linear.z == pytest.approx(-0.02, abs=1e-6)
    assert 'last_altitude_trim_m_s' not in ctrl.members['air1']
    assert 'last_altitude_trim_m_s' not in ctrl.members['air2']


def test_only_formation_mode_applies_altitude_trim():
    now = time.monotonic()
    ctrl = _formation_ctrl(now)
    ctrl.mode = 'rendezvous'
    ctrl.anchor = None
    ctrl.members = {
        'air1': _formation_member('air1', 0, -1.5, 0.85, now),
        'air2': _formation_member('air2', 1, 1.5, 1.05, now),
    }
    ctrl.tick()
    for member in ctrl.members.values():
        member['velocity_pub'].publish.assert_called_once()
        assert 'last_altitude_trim_m_s' not in member
