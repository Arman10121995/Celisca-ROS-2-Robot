"""Projected RGB-D geometry checks; no simulated mission success flags."""
import copy
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot_lab_utils.rgbd_boxes import bindings,estimate_box
from robot_lab_utils.object_estimates import ObjectEstimates


def configuration():
    return dict(schema='robot_lab.rgbd_boxes.v1',boxes=[dict(object='cube',size_m=[.06,.04,.06],
        hsv_low=[5,100,50],hsv_high=[30,255,255],depth_noise_m=.001)])


def projected_face(yaw=.35):
    # Independent pinhole ray intersection with a known horizontal top face.
    width,height=640,480;k=np.array([[550.,0.,320.],[0.,550.,240.],[0.,0.,1.]])
    world=np.eye(4);world[:3,:3]=np.diag([1.,-1.,-1.]);world[:3,3]=[0.,0.,.8]
    y,x=np.mgrid[:height,:width];z=.8-.06
    points=np.stack(((x-320.)/550.*z,-(y-240.)/550.*z),axis=-1)
    local=(points-[.02,.01])@Rotation.from_euler('z',yaw).as_matrix()[:2,:2]
    inside=(np.abs(local[:,:,0])<=.03)&(np.abs(local[:,:,1])<=.02)
    rgb=np.full((height,width,3),80,dtype=np.uint8);rgb[inside]=[255,115,13]
    depth=np.full((height,width),2.);depth[inside]=z
    return rgb,depth,k,world


def test_color_depth_and_camera_transform_recover_unmarked_box_center_and_symmetric_yaw():
    rgb,depth,k,world=projected_face();binding=bindings(configuration())['cube']
    result=estimate_box(rgb,depth,k,[],world,binding)
    np.testing.assert_allclose(result['position'],[.02,.01,.03],atol=.002)
    yaw=Rotation.from_quat(np.array(result['orientation_wxyz'])[[1,2,3,0]]).as_euler('xyz')[2]
    assert abs(yaw-.35)<.03 and result['yaw_symmetry_order']==2
    assert result['position_std_bound_m']<.02 and result['pose_reference']=='collision_box_center'


@pytest.mark.parametrize('case',['missing_depth','wrong_size','two_objects','no_color','occluded','hole'])
def test_missing_ambiguous_or_incomplete_objects_are_refused(case):
    rgb,depth,k,world=projected_face(0.);binding=bindings(configuration())['cube']
    if case=='missing_depth':depth[:]=np.nan
    elif case=='wrong_size':binding['size_m']=[.02,.02,.06]
    elif case=='two_objects':
        rgb[:,150:240]=np.roll(rgb,-150,axis=1)[:,150:240]
    elif case=='no_color':rgb[:]=80
    elif case=='occluded':rgb[:,320:]=80
    elif case=='hole':
        colored=rgb[:,:,0]>200;y,x=np.nonzero(colored)
        rgb[y.min()+3:y.max()-2,x.min()+3:x.max()-2]=80
    with pytest.raises(ValueError):estimate_box(rgb,depth,k,[],world,binding)


def test_binding_rejects_duplicate_names_bad_color_and_nonfinite_dimensions():
    for key,value in [('size_m',[math.nan,.03,.03]),('hsv_high',[200,255,255]),('depth_noise_m',.1)]:
        doc=configuration();doc['boxes'][0][key]=value
        with pytest.raises(ValueError):bindings(doc)
    doc=configuration();doc['boxes']*=2
    with pytest.raises(ValueError):bindings(doc)
    doc=configuration();other=copy.deepcopy(doc['boxes'][0]);other['object']='other';doc['boxes'].append(other)
    with pytest.raises(ValueError,match='overlap'):bindings(doc)


def test_rgbd_cache_cannot_replace_marker_targets_or_override_identity_and_reference():
    identity=dict(plant_id='p',source_sha256='s',scene_id='w');rgb,depth,k,world=projected_face()
    target=estimate_box(rgb,depth,k,[],world,bindings(configuration())['cube']);target['observation_id']='rgb/depth'
    message=dict(schema='robot_lab.object_estimates.v1',**identity,frame='native_world',pose_source='rgbd_box_top',stamp_s=10.,targets={'cube':target})
    rgbd=ObjectEstimates('rgbd_box_top');rgbd.receive(message,identity,10.)
    assert rgbd.get('cube',identity,10.1)['object']=='cube'
    with pytest.raises(ValueError):ObjectEstimates().receive(message,identity,10.)
    invalid=copy.deepcopy(message);invalid['targets']['cube']['pose_reference']='body_origin'
    with pytest.raises(ValueError):rgbd.receive(invalid,identity,10.)
    with pytest.raises(ValueError):rgbd.get('cube',dict(identity,plant_id='other'),10.)
