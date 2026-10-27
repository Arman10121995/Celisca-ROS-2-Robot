"""Physical sensor equations, no-GPS dead reckoning and source camera frames."""
import math
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.marine_navigation_model import MarineOdometry, bottom_ranges, dvl_velocity, waypoint_command


def test_janus_recovers_all_axes_with_declared_uncertainty():
    value = np.array([.2, -.1, .05])
    measured, covariance, beams = dvl_velocity(value, np.random.default_rng(4), standard_deviation=0.)
    assert np.allclose(measured, value)
    assert beams.shape == (4,) and covariance.shape == (3, 3)
    noisy, covariance, _ = dvl_velocity(value, np.random.default_rng(4))
    assert not np.array_equal(noisy, value)
    assert np.linalg.eigvalsh(covariance).min() > 0


def test_bottom_lock_requires_actual_intersections_not_depth_only():
    class Floor:
        def clearance(self, p):return p[2] + 6.
    ranges = bottom_ranges(Floor(), [0., 0., -1.], [0., 0., 0.])
    assert all(abs(v - 4.8 / math.sqrt(.75)) < .03 for v in ranges)
    assert all(math.isinf(v) for v in bottom_ranges(Floor(), [0., 0., 20.], [0., 0., 0.]))


def test_sensor_only_translation_and_pressure_correction_with_rotated_body():
    filter = MarineOdometry([2., 3., -1.])
    pressure = 101325. + 1025.*9.81*1.2
    for i in range(11):
        p = filter.update(i*.1, [.2, 0., 0.], np.eye(3)*.001, [0.,0.,math.pi/2], pressure, 75.**2)
    assert p[0] == pytest.approx(2.)
    assert p[1] == pytest.approx(3.2)
    assert p[2] == pytest.approx(-1.2, abs=.002)
    assert filter.covariance[0,0] > .02**2  # No invented absolute x/y correction.
    for stamp in (1., .5, 2.):
        with pytest.raises(ValueError, match='timestamps'):
            filter.update(stamp,[0.,0.,0.],np.eye(3),[0.,0.,0.],pressure,1.)


def test_underactuated_and_holonomic_commands_stay_distinct():
    rov, _ = waypoint_command([0.,0.,-1.],[0.,0.,0.],[0.,0.,0.],[0.,1.,-2.])
    assert rov[0] == 0 and rov[1] > 0 and rov[2] < 0
    auv, _ = waypoint_command([0.,0.,-1.],[0.,0.,math.pi],[0.,0.,0.],[2.,1.,-1.],auv=True)
    assert auv[0] > 0 and auv[1] == 0 and abs(auv[3]) <= .12


def test_camera_chain_uses_fixed_source_geometry_not_map_pose():
    from robot_lab_utils.marine_camera_relay import fixed_extrinsic
    source = '''<robot name="test"><link name="base_link"/><link name="camera"/><link name="optical"/>
      <joint name="camera_mount" type="fixed"><parent link="base_link"/><child link="camera"/><origin xyz=".2 0 .1"/></joint>
      <joint name="optical_mount" type="fixed"><parent link="camera"/><child link="optical"/><origin rpy="0 0 1.5707963267948966"/></joint></robot>'''
    transform = fixed_extrinsic(source,'optical')
    assert np.allclose(transform[:3,3],[.2,0.,.1])
    assert np.allclose(transform[:3,:3] @ [1.,0.,0.],[0.,1.,0.],atol=1e-12)
    with pytest.raises(ValueError,match='fixed source'):
        fixed_extrinsic(source,'missing')


def test_actual_rtabmap_padded_cloud_layout_is_consumed_without_a_node():
    from types import SimpleNamespace
    from sensor_msgs.msg import PointCloud2,PointField
    from robot_lab_utils.marine_navigation import MarineNavigation
    message=PointCloud2();message.header.frame_id='map';message.height=1;message.width=4
    message.fields=[PointField(name=key,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,key in enumerate('xyz')]
    message.point_step=32;message.row_step=128;message.is_dense=True
    array=np.zeros((4,8),dtype='<f4');array[:,:3]=[[1,2,3],[2,2,3],[3,2,3],[4,2,3]];message.data=array.tobytes()
    receiver=SimpleNamespace(cloud_at=-math.inf,observed=None,reason='')
    MarineNavigation.observe_cloud(receiver,message)
    assert receiver.observed.cell_count==4


def test_authored_auv_camera_keeps_source_masses_collisions_and_actuators():
    import xml.etree.ElementTree as ET
    from robot_lab_utils.marine_pipeline import auv_camera_kit
    from robot_lab_utils.marine_camera_relay import fixed_extrinsic
    folder=ROOT/'src/robot_lab_robots/tethys_auv'
    native=ET.parse(folder/'model.sdf').getroot().find('model')
    physical=lambda model:[ET.tostring(x) for x in model.iter() if x.tag in ('inertial','collision','joint','plugin')]
    before=physical(native)
    description=auv_camera_kit(native,(folder/'urdf/tethys_auv.urdf').read_text())
    assert physical(native)==before
    camera=native.find("link[@name='base_link']/sensor[@name='lab_rgbd']")
    assert camera.get('type')=='rgbd_camera'
    transform=fixed_extrinsic(description,'oakd_rgb_camera_optical_frame')
    assert np.allclose(transform[:3,3],[-1.17,0.,.06])
    # Optical +Z points along the source AUV's -X cruising direction.
    assert np.allclose(transform[:3,:3] @ [0.,0.,1.],[-1.,0.,0.],atol=1e-9)


def test_auv_zero_velocity_brakes_with_signed_thrust_but_inactive_propulsion_is_zero():
    from robot_lab_utils.auv_controller import surge_thrust
    assert surge_thrust(.3,0.)<0.  # Native source forward is -X.
    assert surge_thrust(0.,.3)>0.  # Positive force opposes forward travel.
    assert surge_thrust(0.,.3,False)==0.
    assert abs(surge_thrust(-.7,.7))<=35.
    with pytest.raises(ValueError):surge_thrust(float('nan'),0.)
