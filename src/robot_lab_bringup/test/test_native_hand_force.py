"""Position-target force feedback guards, not a physical hand qualification."""
import pytest
from robot_lab_utils.native_hand_force import ContactForceHold


def test_existing_contact_required_before_any_target_update():
    hold=ContactForceHold(1.,.5)
    with pytest.raises(ValueError,match='contact lost'):
        hold.update(0.,0.,1.)
    assert hold.closure==.5


def test_feedback_closes_for_weak_side_and_opens_for_excess_force_at_bounded_rate():
    hold=ContactForceHold(1.,.5)
    assert hold.update(0.,.2,.4)==.5
    value=hold.update(.1,.2,.4)
    assert .5<value<=.503
    value2=hold.update(.2,1.6,1.5)
    assert value2<value and value-value2<=.003


def test_deadband_frozen_clock_and_closure_limits():
    hold=ContactForceHold(1.,.5)
    hold.update(0.,1.,1.)
    assert hold.update(.1,1.02,.98)==.5
    assert hold.update(.1,.2,.2)==.5
    held=ContactForceHold(1.,1.)
    held.update(0.,.2,.2)
    assert held.update(.1,.2,.2)==1.
    assert held.status=='saturated'


@pytest.mark.parametrize('forces',[(0.,1.),(1.,3.),(float('nan'),1.),(-1.,1.)])
def test_loss_overload_and_bad_measurements_fail_closed(forces):
    hold=ContactForceHold(1.,.5)
    hold.update(0.,1.,1.)
    with pytest.raises(ValueError):
        hold.update(.01,*forces)
    assert hold.closure==.5


def test_clock_jump_and_deadline_are_not_mistaken_for_a_force_error():
    hold=ContactForceHold(1.,.5);hold.update(0.,1.,1.)
    with pytest.raises(ValueError,match='cadence'):
        hold.update(.5,1.,1.)
    hold=ContactForceHold(1.,.5);hold.update(1.,1.,1.)
    with pytest.raises(ValueError,match='clock'):
        hold.update(0.,1.,1.)
    hold=ContactForceHold(1.,.5);hold.update(0.,1.,1.);hold.previous=30.
    with pytest.raises(ValueError,match='deadline'):
        hold.update(30.01,1.,1.)


def test_native_controller_rejects_force_mode_without_an_object_and_keeps_source_limits(tmp_path):
    import numpy as np
    pytest.importorskip('mujoco')
    pytest.importorskip('std_msgs.msg')
    import json
    from std_msgs.msg import String
    from test_native_hand_object import _pad_hand
    from robot_lab_utils.native_hand import hand_contract
    from robot_lab_mujoco.native_hand_control import NativeHandControl
    source,model,data,node,messages=_pad_hand(tmp_path)
    control=NativeHandControl(node,hand_contract(model,source));control.heartbeat(None)
    gain=model.actuator_gainprm.copy();bounds=model.actuator_forcerange.copy()
    control.receive(String(data=json.dumps(dict(operation='hand_force_hold',plant_id=control.plant_id,
        source_sha256=control.source_sha256,contract_sha256=control.contract['contract_sha256'],
        pose='Close',target_force_n=1.))))
    assert control.force_hold is None
    assert 'Establish a measured' in control.status
    np.testing.assert_array_equal(model.actuator_gainprm,gain)
    np.testing.assert_array_equal(model.actuator_forcerange,bounds)
    control.publish()
    state=json.loads(messages[-1].data)
    assert state['hand']['force_regulation']['state']=='interrupted'


def test_existing_overforce_unloads_but_any_rising_overload_still_stops():
    hold=ContactForceHold(5.,.5)
    assert hold.update(0.,12.,12.)==.5
    assert hold.status=='unloading'
    assert hold.update(.1,11.5,11.5)<.5
    with pytest.raises(ValueError,match='overload increased'):
        hold.update(.2,14.,14.)
    hold=ContactForceHold(5.,.5);hold.update(0.,12.,12.)
    hold.update(.1,9.,9.);assert not hold.unloading
    with pytest.raises(ValueError,match='entry bound'):
        ContactForceHold(1.,.5).update(0.,51.,51.)
