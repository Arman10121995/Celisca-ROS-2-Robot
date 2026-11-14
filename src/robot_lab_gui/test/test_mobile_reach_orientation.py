"""Task builder orientation edits are source-labelled and never dispatch motion."""
import math
import os
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot_lab_gui.mobile_task_controls import MobileTaskControls


def test_recipe_steps_capture_the_selected_orientation_mode_without_commands():
    panel=MobileTaskControls.__new__(MobileTaskControls)
    panel.object_fields={key:SimpleNamespace(get=lambda name=name:name)
                         for key,name in [('object','cube'),('left_body','left'),('right_body','right')]}
    panel.orientation_mode=SimpleNamespace(get=lambda:'Position only')
    point=panel.object_step('reach_object',target_xyz=[.2,0.,.5])
    assert 'orientation_wxyz' not in point and 'keep_orientation' not in point
    panel.pose_source=SimpleNamespace(get=lambda:'RGB-D colored boxes')
    assert panel.object_step('reach_object')['pose_source']=='rgbd_box_top'
    assert 'pose_source' not in panel.object_step('grip',operation='open')
    panel.orientation_mode.get=lambda:'Keep measured pad'
    assert panel.object_step('reach_object')['keep_orientation']
    panel.orientation_mode.get=lambda:'World pad RPY'
    panel.orientation_rpy=[SimpleNamespace(get=lambda value=value:value) for value in (10.,20.,30.)]
    result=panel.object_step('reach_object')
    rotation=Rotation.from_quat(np.array(result['orientation_wxyz'])[[1,2,3,0]])
    np.testing.assert_allclose(rotation.as_euler('xyz',degrees=True),[10.,20.,30.])
    panel.orientation_rpy[0].get=lambda:math.nan
    with pytest.raises(ValueError,match='finite pad'):panel.object_step('reach_object')


@pytest.mark.skipif(not os.environ.get('DISPLAY'),reason='Actual Tk requires a display or Xvfb')
def test_real_widget_builds_source_bound_recipe_without_dispatch():
    import tkinter as tk
    root=tk.Tk()
    app=SimpleNamespace(robot_var=tk.StringVar(root,value='stretch'),map_var=tk.StringVar(root,value='bench'))
    control=SimpleNamespace(app=app,state={'channels':[dict(name='lift',limits=[0.,1.])]})
    panel=MobileTaskControls(root,control);panel.pack()
    try:
        panel.state=dict(source_sha256='source',scene_id='scene',navigation_available=False)
        panel.editable=lambda:True
        for key,name in [('object','cube'),('left_body','left'),('right_body','right')]:panel.object_fields[key].set(name)
        panel.orientation_mode.set('Keep measured pad');root.update()
        assert not panel.steps
        panel.add_pick()
        assert len(panel.steps)==7
        assert all(step['keep_orientation'] for step in panel.steps if step['kind']=='reach_object')
        panel.orientation_mode.set('World pad RPY');panel.orientation_rpy[2].set(45.)
        panel.approach_mode.set('From +X');panel.approach_distance.set(.12)
        panel.add_place()
        assert len(panel.steps)==13
        assert all('orientation_wxyz' in step for step in panel.steps[7:] if step['kind']=='reach_object')
        np.testing.assert_allclose(panel.steps[7]['target_xyz'],[.12,0.,.5])
        np.testing.assert_allclose(panel.steps[11]['target_xyz'],[.12,0.,.5])
        panel.approach_distance.set(math.nan);panel.add_pick()
        assert len(panel.steps)==13 and 'finite' in panel.status.get()
        assert panel.node is None  # No ROS publisher/action has been connected.
        panel.approach_distance.set(.08);panel.pose_source.set('Camera fiducials');panel.add_pick()
        assert panel.steps[14]['pose_source']=='aruco_pnp' and panel.steps[15]['pose_source']=='aruco_pnp'
        assert panel.steps[17]['pose_source']=='native_truth'  # Instrumented lift after physical contact.
        panel.pose_source.set('RGB-D colored boxes');panel.add_pick()
        assert panel.steps[21]['pose_source']==panel.steps[22]['pose_source']=='rgbd_box_top'
        assert panel.steps[24]['pose_source']=='native_truth'
        panel.ready=lambda:True
        panel.state.update(perception_sources={'aruco_pnp':['cube']},
                           perception_errors={'rgbd_box_top':'ambiguous color'})
        panel.update_perception_status()
        assert 'ambiguous color' in panel.perception_status.get()
        panel.pose_source.set('Camera fiducials');panel.update_perception_status()
        assert panel.perception_status.get()=='Camera targets: cube'
        panel.ready=lambda:False;panel.update_perception_status()
        assert 'Waiting' in panel.perception_status.get() and panel.node is None
    finally:root.destroy()
