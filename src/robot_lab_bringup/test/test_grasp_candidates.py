"""Geometry and actual MoveIt request contracts; no grasp-success shortcuts."""
import math
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from robot_lab_utils.grasp_candidates import top_grasps


def obj(**values):
    return dict(dict(frame='native_world',id='cube',type='box',size=[.03,.03,.03],
                     position=[.4,0.,.3],orientation_wxyz=[1.,0.,0.,0.]),**values)


def test_box_grasps_fit_actual_opening_and_form_right_handed_tool_frames():
    candidates=top_grasps(obj(size=[.03,.12,.04]))
    assert len(candidates)==2
    for candidate in candidates:
        assert candidate['opening_m']==pytest.approx(.034)
        np.testing.assert_allclose(candidate['poses']['approach']['position'],[.4,0.,.38])
        np.testing.assert_allclose(candidate['poses']['lift']['position'],[.4,0.,.4])
        q=np.array(candidate['poses']['grasp']['orientation_wxyz'])
        matrix=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        np.testing.assert_allclose(matrix[:,1],candidate['closing_axis_world'],atol=1e-12)
        np.testing.assert_allclose(matrix[:,2],[0.,0.,-1.],atol=1e-12)
        assert np.linalg.det(matrix)==pytest.approx(1.)


def test_rotated_box_sphere_and_cylinder_use_projected_physical_widths():
    quat=Rotation.from_euler('z',.7).as_quat()[[3,0,1,2]].tolist()
    assert len(top_grasps(obj(size=[.03,.12,.04],orientation_wxyz=quat)))==2
    sphere=top_grasps(obj(type='sphere',size=[.03]))
    assert len(sphere)==4 and all(c['opening_m']==pytest.approx(.064) for c in sphere)
    cylinder=top_grasps(obj(type='cylinder',size=[.02,.2]))
    assert len(cylinder)==4 and all(c['opening_m']==pytest.approx(.044) for c in cylinder)
    assert top_grasps(obj(size=[.2,.2,.2]))==[]


@pytest.mark.parametrize('values',[dict(frame='camera'),dict(size=[-.01,.02,.03]),
    dict(orientation_wxyz=[0.,0.,0.,0.]),dict(position=[float('nan'),0.,0.]),dict(type='mesh')])
def test_invalid_unframed_or_unreviewed_geometry_is_refused(values):
    with pytest.raises(ValueError):top_grasps(obj(**values))
