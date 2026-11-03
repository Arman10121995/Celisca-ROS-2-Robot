"""Floor-aware traffic and physically acknowledged exit-parking contracts."""
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/robot_lab_utils'))
from robot_lab_utils.fleet_controller import FleetController
from robot_lab_utils.fleet_transport import LiftTransport
from robot_lab_utils.fleet_transport_queue import TransportQueue
from test_fleet_transport import CONFIG,context


def test_exit_parking_needs_a_real_owned_route_and_settled_body():
    goal=dict(x=1.55,y=-.55,yaw=0.)
    task=LiftTransport('alpha','op','L0','L1',CONFIG,0.,parking=goal)
    task.phase='exiting';task.route_id=task.prefix+'-exiting'
    state=context(task,(2.2,0.,3.),10.);state.update(floor='L1',map_ready=True)
    state['history']=[dict(source='route:'+task.route_id,state='succeeded')]
    assert task.tick(state,1.)==[]
    state['clock']=11.1
    action=task.tick(state,2.)
    assert action[0]['kind']=='route' and action[0]['goal']['x']==goal['x']
    assert task.phase=='parking' and task.active
    state['history']=[dict(source='route:'+task.route_id,state='succeeded')]
    assert task.tick(state,3.)==[]  # Parking Nav2 success at the landing is insufficient.
    state['body']['xyz']=[1.55,-.55,3.]
    state['clock']=12.
    assert task.tick(state,4.)==[]
    state['clock']=13.1
    assert task.tick(state,5.)==[dict(kind='release',hold=False)]
    assert task.phase=='succeeded'


def test_parking_request_admission_is_atomic_and_rejects_bad_coordinates():
    queue=TransportQueue()
    with pytest.raises(ValueError,match='Parking'):
        queue.append([dict(robot='alpha',floor='L1',parking=dict(x=1.,y=0.,yaw=float('nan')))],'op',dict(alpha='L0'),['L0','L1'],0.)
    assert not queue.pending


def test_explicit_slow_host_budgets_remain_finite_and_bounded():
    task=LiftTransport('alpha','op','L0','L1',dict(CONFIG,total_timeout_s=900.,phase_timeout_s=300.,route_timeout_s=180.),0.)
    assert task.tick(context(task),1.)[0]['goal']['timeout']==180.
    assert task.tick(context(task),302.)[-1]==dict(kind='release',hold=True)
    with pytest.raises(ValueError,match='bounded transport'):
        LiftTransport('alpha','op','L0','L1',dict(CONFIG,total_timeout_s=float('inf')),0.)


def test_floor_filter_needs_fresh_observed_unheld_floors(monkeypatch):
    monkeypatch.setattr('robot_lab_utils.fleet_controller.time.monotonic',lambda:100.)
    node=SimpleNamespace(session=dict(infrastructure_file='facility.yaml'),infrastructure_at=100.,
        infrastructure_floors=dict(alpha='L0',beta='L1'),infrastructure_holds=set())
    assert not FleetController._same_floor(node,'alpha','beta')
    node.infrastructure_at=98.
    assert FleetController._same_floor(node,'alpha','beta')
    node.infrastructure_at=100.;node.infrastructure_holds={'alpha'}
    assert FleetController._same_floor(node,'alpha','beta')
    node.infrastructure_holds.clear();node.infrastructure_floors.pop('beta')
    assert FleetController._same_floor(node,'alpha','beta')


def test_overlapping_xy_on_separate_floors_is_not_reported_as_contact(monkeypatch):
    from robot_lab_utils.fleet_motion import Neighbor
    controller=object.__new__(FleetController)
    controller.members={name:SimpleNamespace(name=name) for name in ('alpha','beta')}
    controller._neighbor=lambda member:Neighbor(member.name,0.,0.,0.,0.,.2)
    controller._same_floor=lambda a,b:False
    controller._stop_member=Mock();controller._event=Mock();controller._contact_events=[]
    controller._contact_monitor(100.,dict(alpha=True,beta=True))
    controller._stop_member.assert_not_called();controller._event.assert_not_called()
