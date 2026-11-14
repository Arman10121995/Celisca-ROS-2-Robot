"""Actual ROS image/CameraInfo/TF to camera-only object estimate transport.

The camera image is explicitly synthesized by perspective projection. This
test does not claim a robot grasp or a rendered physical mission.
"""
import json
import math
import time
import numpy as np
import pytest
import yaml


def test_ros_images_calibration_tf_and_task_identity_produce_a_fresh_estimate(tmp_path):
    cv2=pytest.importorskip('cv2')
    if not hasattr(cv2,'aruco'):pytest.skip('OpenCV aruco module unavailable')
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from scipy.spatial.transform import Rotation
    from robot_lab_utils.fiducial_object_pose import FiducialObjects
    binding=dict(schema='robot_lab.object_markers.v1',markers=[dict(id=7,object='camera_fixture_cube',marker_size_m=.024,
        object_size_m=[.03,.03,.03],object_from_marker=[0.,0.,.015,1.,0.,0.,0.])])
    path=tmp_path/'markers.yaml';path.write_text(yaml.safe_dump(binding))
    initialized=not rclpy.ok()
    if initialized:rclpy.init()
    detector=FiducialObjects(parameter_overrides=[Parameter('bindings',value=str(path))])
    probe=rclpy.create_node('object_pose_camera_probe')
    executor=SingleThreadedExecutor();executor.add_node(detector);executor.add_node(probe)
    output=[];probe.create_subscription(String,'/robot_lab/object_estimates',lambda m:output.append(json.loads(m.data)),10)
    images=probe.create_publisher(Image,'/oakd/rgb/image_raw',qos_profile_sensor_data)
    infos=probe.create_publisher(CameraInfo,'/oakd/rgb/camera_info',qos_profile_sensor_data)
    contexts=probe.create_publisher(String,'/mobile_task/state',10)
    tf=StaticTransformBroadcaster(probe)
    transform=TransformStamped();transform.header.frame_id='native_world';transform.child_frame_id='fixture_optical'
    transform.transform.rotation.w=1.;tf.sendTransform(transform)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker=cv2.aruco.generateImageMarker(dictionary,7,200) if hasattr(cv2.aruco,'generateImageMarker') else cv2.aruco.drawMarker(dictionary,7,200)
    half=.012;points=np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]])
    k=np.array([[700.,0.,320.],[0.,700.,240.],[0.,0.,1.]])
    r=np.array([2.8,.1,.15]);t=np.array([.01,-.02,.25])
    corners=cv2.projectPoints(points,r,t,k,None)[0].reshape(4,2)
    warp=cv2.getPerspectiveTransform(np.array([[0,0],[199,0],[199,199],[0,199]],dtype=np.float32),corners.astype(np.float32))
    image=cv2.warpPerspective(marker,warp,(640,480),borderValue=255)
    identity=dict(plant_id='interface_fixture',source_sha256='fixture_source',scene_id='fixture_scene')
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and not any(o['targets'] for o in output):
            context=dict(identity,object_candidates=['camera_fixture_cube'])
            contexts.publish(String(data=json.dumps(context)))
            info=CameraInfo();info.header.frame_id='fixture_optical';info.header.stamp=probe.get_clock().now().to_msg()
            info.width=640;info.height=480;info.k=k.ravel().tolist();info.distortion_model='plumb_bob';info.d=[0.]*5
            msg=Image();msg.header=info.header;msg.width=640;msg.height=480;msg.encoding='mono8';msg.step=640;msg.data=image.tobytes()
            # The actual native camera sends RGB before CameraInfo. Exercise
            # that delivery order instead of assuming calibration arrives first.
            images.publish(msg)
            for _ in range(3):executor.spin_once(timeout_sec=.005)
            infos.publish(info)
            for _ in range(12):executor.spin_once(timeout_sec=.01)
        valid=[o for o in output if o['targets']]
        assert valid,output
        result=valid[-1];assert all(result[key]==value for key,value in identity.items())
        target=result['targets']['camera_fixture_cube']
        expected=t-Rotation.from_rotvec(r).apply([0.,0.,.015])
        np.testing.assert_allclose(target['position'],expected,atol=.004)
        assert result['pose_source']=='aruco_pnp' and result['frame']=='native_world'
        assert target['observation_id'] and target['position_std_bound_m']<.02
    finally:
        executor.shutdown();detector.destroy_node();probe.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
