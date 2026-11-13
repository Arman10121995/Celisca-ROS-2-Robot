"""Placement uses the measured rigid grasp relation, not a guessed TCP offset."""
import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot_lab_utils.grasp_candidates import support_placement


def inputs():
    obj=dict(id='cube',frame='native_world',type='box',size=[.04,.06,.08],
        position=[.1,.2,.5],orientation_wxyz=[1.,0.,0.,0.],plant_id='plant',source_sha256='source')
    attachment=dict(object_id='cube',position=[.01,.02,.12],orientation_wxyz=[1.,0.,0.,0.],
        plant_id='plant',source_sha256='source')
    support=dict(id='bench',frame='native_world',type='box',size=[.5,.4,.1],
        position=[.5,.3,.45],orientation_wxyz=[1.,0.,0.,0.])
    return obj,attachment,support


def rotation(q):return Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix()


def test_rotated_grasp_offset_reconstructs_the_requested_object_pose_on_support():
    obj,attached,support=inputs()
    obj['orientation_wxyz']=Rotation.from_euler('xyz',[.2,.3,.7]).as_quat()[[3,0,1,2]].tolist()
    attached['orientation_wxyz']=Rotation.from_euler('xyz',[-.5,.1,.2]).as_quat()[[3,0,1,2]].tolist()
    before=copy.deepcopy((obj,attached,support))
    placement=support_placement(obj,attached,support,clearance=.02)
    pose=placement['poses']['lower'];tool=rotation(pose['orientation_wxyz'])
    center=np.asarray(pose['position'])+tool@attached['position']
    np.testing.assert_allclose(center,placement['object_center'],atol=1e-12)
    np.testing.assert_allclose(tool@rotation(attached['orientation_wxyz']),rotation(obj['orientation_wxyz']),atol=1e-12)
    half_height=(abs(rotation(obj['orientation_wxyz']))@obj['size'])[2]/2
    assert center[2]-half_height==pytest.approx(.5+.02)
    np.testing.assert_allclose(np.asarray(placement['poses']['above']['position'])-pose['position'],[0.,0.,.08])
    assert (obj,attached,support)==before


@pytest.mark.parametrize('bad',['wrong_object','stale_plant','wrong_source','no_attachment','sloped','too_small','nan'])
def test_unbound_or_incompatible_placement_geometry_is_refused(bad):
    obj,attached,support=inputs()
    if bad=='wrong_object':attached['object_id']='another'
    elif bad=='stale_plant':attached['plant_id']='old'
    elif bad=='wrong_source':attached['source_sha256']='old'
    elif bad=='no_attachment':attached=None
    elif bad=='sloped':support['orientation_wxyz']=Rotation.from_euler('x',.1).as_quat()[[3,0,1,2]].tolist()
    elif bad=='too_small':support['size']=[.03,.03,.1]
    else:support['position'][0]=float('nan')
    with pytest.raises(ValueError):support_placement(obj,attached,support)
