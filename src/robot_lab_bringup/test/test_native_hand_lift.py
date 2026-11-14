"""Source-axis object transport guards; component checks, not GUI certificates."""
import json
import time

import numpy as np
import pytest

mj=pytest.importorskip('mujoco')
pytest.importorskip('std_msgs.msg')
from std_msgs.msg import String
from test_native_hand_object import _object_hand


def request(control,operation,**values):
    control.receive(String(data=json.dumps(dict(operation=operation,plant_id=control.plant_id,
        source_sha256=control.source_sha256,contract_sha256=control.contract['contract_sha256'],**values))))


def test_fixed_source_hand_refuses_transport_without_changing_motor_contract():
    control,model,data,*_=_object_hand()
    gains=model.actuator_gainprm.copy();limits=model.actuator_ctrlrange.copy();pose=data.qpos.copy()
    # The axis check itself must refuse a fixed source, even with an object.
    with pytest.raises(ValueError,match='arm transport adapter'):control.lift_channel()
    request(control,'hand_lift_object',height_m=.05)
    assert control.lift_run is None
    np.testing.assert_array_equal(data.qpos,pose)
    np.testing.assert_array_equal(model.actuator_gainprm,gains)
    np.testing.assert_array_equal(model.actuator_ctrlrange,limits)


def test_umi_source_axis_preload_and_interruption_contract():
    control,model,data,node,*_=_object_hand('umi_gripper','umi_gripper.xml')
    slot,joint,axis_z=control.lift_channel()
    assert abs(axis_z)>.995 and control.channels[slot]['max_rate']<=.05
    # Protocol-only setup: these declared observations are not a grasp claim.
    control.grasp.update(state='holding',supported=True)
    before=control.target.copy();pose=data.qpos.copy()
    request(control,'hand_lift_object',height_m=.05)
    assert control.lift_state['state']=='lifting' and control.lift_run
    np.testing.assert_allclose(control.target[slot],before[slot]+.05/axis_z)
    np.testing.assert_array_equal(np.delete(control.target,slot),np.delete(before,slot))
    np.testing.assert_array_equal(data.qpos,pose)
    control.last_heartbeat=time.monotonic()-1.
    control.update_lift()
    assert control.lift_run is None and control.lift_state['state']=='interrupted'
    assert 'heartbeat' in control.lift_state['reason']


@pytest.mark.parametrize('height',[float('nan'),-.05,.5])
def test_umi_rejects_invalid_height_before_motion(height):
    control,model,data,*_=_object_hand('umi_gripper','umi_gripper.xml')
    control.grasp.update(state='holding',supported=True)
    pose=data.qpos.copy()
    request(control,'hand_lift_object',height_m=height)
    assert control.lift_run is None and 'height' in control.status
    np.testing.assert_array_equal(data.qpos,pose)


def test_real_umi_grasp_lift_return_and_supported_release_keep_source_motors():
    control,model,data,node,*_=_object_hand('umi_gripper','umi_gripper.xml')
    gain=model.actuator_gainprm.copy();bounds=model.actuator_ctrlrange.copy()
    force_bounds=model.actuator_forcerange.copy();damping=model.dof_damping.copy()
    assert control.contract['open_targets']['fingers_actuator']==0.
    assert control.contract['curled_targets']['fingers_actuator']==.05
    def step(seconds):
        for i in range(round(seconds/model.opt.timestep)):
            if i%100==0:control.heartbeat(None)
            control.before_step();mj.mj_step(model,data);control.after_step()
    request(control,'hand_pose',pose='Open');step(2.)
    assert abs(data.xpos[min(control.object_sides[0]),2]-.11)<.003
    request(control,'hand_grasp',pose='Close',closure=1.,effort_fraction=.8,contact_limit_n=1.)
    step(4.)
    assert control.grasp['state']=='holding'
    before=float(data.xpos[control.object_body,2])
    request(control,'hand_lift_object',height_m=.05);step(5.)
    assert control.lift_state['state']=='holding'
    assert not control.grasp['supported']
    assert abs(float(data.xpos[control.object_body,2])-before-.05)<.005
    assert min(control.grasp['left_force_n'],control.grasp['right_force_n'])>.2
    request(control,'hand_lower_object');step(5.)
    assert control.lift_state['state']=='supported' and control.grasp['supported']
    assert 0.<=control.lift_state['support_seating_m']<=.003+1e-9
    request(control,'hand_release');step(3.)
    assert control.grasp['state']=='supported_release' and control.grasp['contact_count']==0
    np.testing.assert_array_equal(model.actuator_gainprm,gain)
    np.testing.assert_array_equal(model.actuator_ctrlrange,bounds)
    np.testing.assert_array_equal(model.actuator_forcerange,force_bounds)
    np.testing.assert_array_equal(model.dof_damping,damping)
