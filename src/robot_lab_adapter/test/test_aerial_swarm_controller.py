import unittest.mock as mock

import pytest

from robot_lab_adapter.aerial_swarm_controller import AerialSwarmController


def _fresh(row, now=5.0):
    return (now - row['status_at'] < 1.2 and
            now - row['pose_at'] < 0.6 and
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
    (2.0, 0.0, 'flying', True, False),
    (0.0, 1.2, 'flying', True, False),
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
    member_stale = _member('air2', phase='flying', status_age=2.0, pose_age=0.0)
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
    row = dict(status_at=5.0 - 1.19, pose_at=5.0, status=dict(phase='flying', position_ready=True))
    assert _fresh(row, now=5.0) is True
    row = dict(status_at=5.0 - 1.21, pose_at=5.0, status=dict(phase='flying', position_ready=True))
    assert _fresh(row, now=5.0) is False


def test_fresh_pose_age_boundary():
    now = 5.0
    row = dict(status_at=5.0, pose_at=5.0 - 0.59, status=dict(phase='flying', position_ready=True))
    assert _fresh(row, now=5.0) is True
    row = dict(status_at=5.0, pose_at=5.0 - 0.61, status=dict(phase='flying', position_ready=True))
    assert _fresh(row, now=5.0) is False
