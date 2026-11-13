"""Object gates require measured contact/support, sustained time and low slip."""
import pytest
from robot_lab_utils.object_custody import ObjectCustody
from robot_lab_utils.mobile_task import SCHEMA, validate_task


def observe(gate, t, **kw):
    values = dict(stamp=t, relative=[0., 0., 0.], left_force=1., right_force=1., supported=False, speed=0.)
    values.update(kw)
    return gate.observe(**values)


def test_grasp_requires_opposing_contact_without_external_support():
    gate = ObjectCustody()
    for t in range(4):
        assert not observe(gate, t, right_force=0.)
    assert not observe(gate, 4, supported=True)
    assert not observe(gate, 5)
    assert observe(gate, 6)


def test_slip_and_clock_reset_restart_dwell_and_frozen_time_never_passes():
    gate = ObjectCustody()
    assert not observe(gate, 4)
    assert not observe(gate, 5, relative=[.02, 0., 0.])
    assert not observe(gate, 1, relative=[.02, 0., 0.])
    for _ in range(10):
        assert not observe(gate, 1, relative=[.02, 0., 0.])
    assert observe(gate, 2, relative=[.02, 0., 0.])


def test_release_requires_support_no_fingers_and_stopped_object():
    gate = ObjectCustody()
    assert not observe(gate, 0, released=True)
    assert not observe(gate, 1, released=True, supported=True)
    args = dict(released=True, supported=True, left_force=0., right_force=0.)
    assert not observe(gate, 2, speed=.2, **args)
    assert not observe(gate, 3, **args)
    assert observe(gate, 4, **args)


def test_bad_measurement_fails_closed():
    with pytest.raises(ValueError):
        observe(ObjectCustody(), float('nan'))


def test_task_body_names_and_conditions_are_explicit_and_distinct():
    step = dict(kind='object_check', condition='grasped', object='cube', left_body='finger_l', right_body='finger_r')
    document = dict(schema=SCHEMA, source_sha256='source', scene_id='scene', steps=[step])
    assert validate_task(document, 'source', 'scene', [], False)[0]['object']=='cube'
    step['right_body'] = 'finger_l'
    with pytest.raises(ValueError, match='distinct'):
        validate_task(document, 'source', 'scene', [], False)


def test_native_object_recipe_steps_validate_bounds_and_gripper_operation():
    common=dict(object='cube',left_body='finger_l',right_body='finger_r')
    steps=[dict(kind='reach_object',offset=[0.,0.,.1],**common),
           dict(kind='grip',operation='close',**common),dict(kind='stow_arm',**common)]
    document=dict(schema=SCHEMA,source_sha256='source',scene_id='scene',steps=steps)
    assert len(validate_task(document,'source','scene',[],False))==3
    steps[0]['offset']=[float('nan'),0.,0.]
    with pytest.raises(ValueError,match='finite'):
        validate_task(document,'source','scene',[],False)
