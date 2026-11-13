"""Explicit placement requests: fresh source binding, attachment and gap gates."""
import math
import os
import time
from types import SimpleNamespace

import pytest
pytest.importorskip('geometry_msgs.msg')

from robot_lab_gui.grasp_target_controls import GraspTargetControls


def test_support_placement_only_plans_with_matching_fresh_grasp_and_finite_gap():
    plans=[];messages=[]
    panel=GraspTargetControls.__new__(GraspTargetControls)
    panel.catalog=dict(plant_id='plant',source_sha256='source',supports={'bench':dict(id='bench',
        frame='native_world',type='box',position=[.4,0.,.4],size=[.5,.5,.2],orientation_wxyz=[1.,0.,0.,0.])})
    panel.object=dict(id='cube',plant_id='plant',source_sha256='source',frame='native_world',
        type='box',position=[.3,0.,.5],size=[.03]*3,orientation_wxyz=[1.,0.,0.,0.])
    panel.received=panel.catalog_received=time.monotonic()
    panel.support_choice=SimpleNamespace(get=lambda:'bench')
    panel.release_gap=SimpleNamespace(get=lambda:.03)
    panel.feedback=SimpleNamespace(set=messages.append)
    panel.cartesian=SimpleNamespace(ready=lambda:True,plan_pose=lambda *args:plans.append(args),scene=dict(
        object_id='cube',object_attached=True,grasp_contacts_ready=True,object_in_tool=dict(object_id='cube',
            plant_id='plant',source_sha256='source',position=[0.,0.,.1],orientation_wxyz=[1.,0.,0.,0.])))
    panel.plan_placement('above');panel.plan_placement('lower')
    assert len(plans)==2
    assert plans[0][0].position.z-plans[1][0].position.z==pytest.approx(.08)
    assert plans[1][0].position.z==pytest.approx(.445)
    panel.release_gap.get=lambda:math.nan;panel.plan_placement('lower')
    assert len(plans)==2 and 'finite release gap' in messages[-1]
    panel.release_gap.get=lambda:.03
    panel.cartesian.scene['grasp_contacts_ready']=False;panel.plan_placement('lower')
    assert len(plans)==2 and 'measured two-finger' in messages[-1]
    panel.cartesian.scene['grasp_contacts_ready']=True
    panel.object['plant_id']='old';panel.plan_placement('above');assert len(plans)==2
    panel.object['plant_id']='plant';panel.catalog_received=-math.inf
    panel.plan_placement('above');assert len(plans)==2


@pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='Actual Tk requires a display or Xvfb')
def test_actual_widget_edits_no_motion_and_dispatches_only_explicit_placement_plans():
    import json
    import tkinter as tk
    from std_msgs.msg import String
    root=tk.Tk();plans=[]
    cartesian=SimpleNamespace(ready=lambda:True,owned=lambda:True,planning=False,reading_pose=False,
        arm=SimpleNamespace(ready=lambda:True),plan_pose=lambda *args:plans.append(args),scene=dict(
            object_id='cube',object_attached=True,grasp_contacts_ready=True,link_padding_m=.01,
            object_in_tool=dict(object_id='cube',plant_id='plant',source_sha256='source',
                position=[0.,0.,.1],orientation_wxyz=[1.,0.,0.,0.])))
    panel=GraspTargetControls(root,cartesian);panel.pack()
    try:
        support=dict(id='bench',frame='native_world',type='box',size=[.5,.5,.2],
            position=[.4,0.,.4],orientation_wxyz=[1.,0.,0.,0.])
        panel.receive_catalog(String(data=json.dumps(dict(plant_id='plant',source_sha256='source',
            objects=['cube'],selected='cube',supports={'bench':support}))))
        panel.receive(String(data=json.dumps(dict(id='cube',plant_id='plant',source_sha256='source',
            frame='native_world',type='box',size=[.03]*3,position=[.2,0.,.7],orientation_wxyz=[1.,0.,0.,0.]))))
        root.update();panel.poll()
        assert panel.support_selector['values']==('bench',)
        panel.release_gap.set(.04);root.update();assert not plans
        panel.place_buttons[0].invoke();panel.place_buttons[1].invoke()
        assert len(plans)==2 and plans[1][0].position.z==pytest.approx(.455)
        cartesian.scene['object_attached']=False;panel.poll()
        assert panel.place_buttons[0].instate(['disabled'])
        panel.place_buttons[0].invoke();assert len(plans)==2
    finally:root.destroy()
