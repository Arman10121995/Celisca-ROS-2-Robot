"""Actual side-strategy, gap editing and gated retreat through the Tk controls."""
import json
import os
from types import SimpleNamespace

import pytest
pytest.importorskip('geometry_msgs.msg')
pytestmark=pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='Actual Tk requires a display or Xvfb')


def test_side_candidates_only_plan_and_retreat_requires_open_detached_fingers():
    import tkinter as tk
    from std_msgs.msg import String
    from robot_lab_gui.grasp_target_controls import GraspTargetControls
    root=tk.Tk();plans=[];invalidations=[]
    hand=SimpleNamespace(gap_var=tk.DoubleVar(root,value=0.))
    parent=SimpleNamespace(ready=lambda:True,owned=lambda:True,planning=False,reading_pose=False,
        invalidate=lambda:invalidations.append(1),plan_pose=lambda *args:plans.append(args),
        arm=SimpleNamespace(ready=lambda:True,app=SimpleNamespace(hand_tab=hand)),
        joints={'finger_joint1':.04,'finger_joint2':.04},
        scene=dict(object_id='cube',object_attached=False,grasp_contacts_ready=False,link_padding_m=.01))
    panel=GraspTargetControls(root,parent);panel.pack()
    try:
        panel.receive_catalog(String(data=json.dumps(dict(plant_id='plant',source_sha256='source',
            objects=['cube'],selected='cube',supports={}))))
        panel.receive(String(data=json.dumps(dict(id='cube',plant_id='plant',source_sha256='source',
            frame='native_world',type='box',size=[.03]*3,position=[.4,0.,.3],orientation_wxyz=[1.,0.,0.,0.]))))
        panel.strategy.set('Side');panel.strategy_selector.event_generate('<<ComboboxSelected>>');root.update()
        assert invalidations and not plans
        panel.generate();panel.poll()
        assert panel.candidates and all(c['id'].startswith('side_') for c in panel.candidates)
        panel.set_gap();assert hand.gap_var.get()==pytest.approx(54.) and not plans
        panel.buttons[0].invoke();panel.buttons[3].invoke();assert len(plans)==2
        parent.scene['object_attached']=True;parent.scene['grasp_contacts_ready']=True;panel.poll()
        assert panel.buttons[3].instate(['disabled'])
        panel.plan('retreat');assert len(plans)==2
        parent.scene['object_attached']=False;parent.scene['grasp_contacts_ready']=False
        parent.joints={'finger_joint1':0.,'finger_joint2':0.}
        panel.plan('retreat');assert len(plans)==2
        assert 'Open Hand' in panel.feedback.get()
    finally:root.destroy()
