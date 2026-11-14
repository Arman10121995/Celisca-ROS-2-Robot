"""Independent projected camera poses and stale/source-bound target rejection."""
import math
import time
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from robot_lab_utils.fiducial_objects import bindings,marker_pose,object_pose
from robot_lab_utils.object_estimates import ObjectEstimates


def configuration():
    return dict(schema='robot_lab.object_markers.v1',markers=[dict(id=7,object='cube',marker_size_m=.024,
        object_size_m=[.03,.03,.03],object_from_marker=[0.,0.,.015,1.,0.,0.,0.])])


def test_camera_pnp_and_known_mount_recover_full_object_pose_without_truth_input():
    cv2=pytest.importorskip('cv2')
    half=.012;points=np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]])
    k=np.array([[700.,0.,320.],[0.,710.,240.],[0.,0.,1.]])
    r=np.array([2.8,.12,.15]);t=np.array([.02,-.015,.3])
    image=cv2.projectPoints(points,r,t,k,None)[0]
    estimate=marker_pose(image,.024,k,[])
    np.testing.assert_allclose(estimate['position'],t,atol=1e-8)
    q=np.array(estimate['orientation_wxyz'])
    assert (Rotation.from_quat(q[[1,2,3,0]]).inv()*Rotation.from_rotvec(r)).magnitude()<1e-7
    world=np.eye(4);world[:3,:3]=Rotation.from_euler('xyz',[.2,.1,-.3]).as_matrix();world[:3,3]=[1.,2.,.5]
    result=object_pose(estimate,bindings(configuration())[7],world)
    expected=world[:3,:3]@(t-Rotation.from_rotvec(r).apply([0.,0.,.015]))+world[:3,3]
    np.testing.assert_allclose(result['position'],expected,atol=1e-8)
    with pytest.raises(ValueError,match='small'):marker_pose(image*.1,.024,k,[])


def test_marker_bindings_refuse_duplicates_and_invalid_physical_geometry():
    doc=configuration();doc['markers'].append(dict(doc['markers'][0]))
    with pytest.raises(ValueError,match='unique'):bindings(doc)
    for key,value in [('marker_size_m',math.nan),('object_size_m',[0.,.03,.03]),('object_from_marker',[0.]*7)]:
        doc=configuration();doc['markers'][0][key]=value
        with pytest.raises(ValueError):bindings(doc)


IDENTITY=dict(plant_id='plant',source_sha256='source',scene_id='scene')


def record():
    return dict(schema='robot_lab.object_estimates.v1',**IDENTITY,frame='native_world',pose_source='aruco_pnp',stamp_s=10.,
        targets=dict(cube=dict(object='cube',position=[.1,.2,.3],orientation_wxyz=[1.,0.,0.,0.],
                              observation_id='image/corners',position_std_bound_m=.005)))


def test_estimated_pose_never_falls_back_to_truth_when_missing_stale_or_wrong_source():
    store=ObjectEstimates()
    with pytest.raises(ValueError):store.get('cube',IDENTITY,10.)
    store.receive(record(),IDENTITY,10.1)
    assert store.get('cube',IDENTITY,10.2)['position']==[.1,.2,.3]
    with pytest.raises(ValueError):store.get('cube',dict(IDENTITY,plant_id='other'),10.2)
    with pytest.raises(ValueError):store.get('other',IDENTITY,10.2)
    with pytest.raises(ValueError):store.get('cube',IDENTITY,11.)
    store.received=time.monotonic()-2.
    with pytest.raises(ValueError):store.get('cube',IDENTITY,10.2)
    empty=record();empty['targets']={};store.receive(empty,IDENTITY,10.2)
    with pytest.raises(ValueError):store.get('cube',IDENTITY,10.2)


@pytest.mark.parametrize('key,value',[('plant_id','other'),('stamp_s',11.),('stamp_s',8.),('frame','camera'),('pose_source','native_truth')])
def test_stale_future_untransformed_or_unbound_observations_are_refused(key,value):
    data=record();data[key]=value
    with pytest.raises(ValueError):ObjectEstimates().receive(data,IDENTITY,10.)
