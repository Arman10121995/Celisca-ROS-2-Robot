"""Real ROS RGB/depth/CameraInfo/TF transport with explicitly projected pixels."""
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest
import yaml


def test_metric_depth_decoding_preserves_endian_and_row_padding():
    from robot_lab_utils.rgbd_object_pose import pixels
    rows=np.full((16,18),800,dtype='>u2')
    message=SimpleNamespace(width=16,height=16,step=36,encoding='16UC1',is_bigendian=True,data=rows.tobytes())
    np.testing.assert_allclose(pixels(message,True),.8)
    message.step=30
    with pytest.raises(ValueError,match='stride'):pixels(message,True)


def test_ros_rgb_depth_calibration_and_tf_produce_pose_then_clear_on_frame_mismatch(tmp_path,monkeypatch):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from robot_lab_utils.rgbd_object_pose import RGBDObjects
    config=dict(schema='robot_lab.rgbd_boxes.v1',boxes=[dict(object='box',size_m=[.06,.04,.06],
        hsv_low=[5,100,60],hsv_high=[30,255,255],depth_noise_m=.001)])
    path=tmp_path/'boxes.yaml';path.write_text(yaml.safe_dump(config))
    initialized=not rclpy.ok()
    if initialized:rclpy.init()
    detector=RGBDObjects(parameter_overrides=[Parameter('bindings',value=str(path))])
    from tf2_ros import LookupException
    lookup=detector.tf.lookup_transform;lookup_stamps=[]
    def briefly_delayed_tf(target,source,when):
        lookup_stamps.append(when.nanoseconds)
        if len(lookup_stamps)==1:raise LookupException('Declared delayed TF test')
        return lookup(target,source,when)
    monkeypatch.setattr(detector.tf,'lookup_transform',briefly_delayed_tf)
    probe=rclpy.create_node('rgbd_box_interface_probe');executor=SingleThreadedExecutor()
    executor.add_node(detector);executor.add_node(probe)
    results=[];probe.create_subscription(String,'/robot_lab/object_estimates',lambda m:results.append(json.loads(m.data)),10)
    color_pub=probe.create_publisher(Image,'/oakd/rgb/image_raw',qos_profile_sensor_data)
    depth_pub=probe.create_publisher(Image,'/oakd/depth/image_raw',qos_profile_sensor_data)
    info_pub=probe.create_publisher(CameraInfo,'/oakd/rgb/camera_info',qos_profile_sensor_data)
    context_pub=probe.create_publisher(String,'/mobile_task/state',10)
    tf=StaticTransformBroadcaster(probe)
    transform=TransformStamped();transform.header.frame_id='native_world';transform.child_frame_id='rgbd_fixture_optical'
    transform.transform.translation.z=.8;transform.transform.rotation.x=1.;transform.transform.rotation.w=0.;tf.sendTransform(transform)
    # Perspective ray intersection with an unmarked top face; synthetic input,
    # actual ROS serialization, subscriptions, time matching and TF lookup.
    yy,xx=np.mgrid[:480,:640];z=.74
    inside=(np.abs((xx-320)/550*z-.02)<=.03)&(np.abs(-(yy-240)/550*z-.01)<=.02)
    color=np.full((480,640,3),80,dtype=np.uint8);color[inside]=[255,115,13]
    distance=np.full((480,640),2.,dtype='<f4');distance[inside]=z
    identity=dict(plant_id='rgbd_fixture',source_sha256='fixture_source',scene_id='fixture_scene')
    def publish(mismatch=False):
        context_pub.publish(String(data=json.dumps(dict(identity,object_candidates=['box'],physical_pose=['ignored']))))
        info=CameraInfo();info.header.frame_id='rgbd_fixture_optical';info.header.stamp=probe.get_clock().now().to_msg()
        info.width=640;info.height=480;info.k=[550.,0.,320.,0.,550.,240.,0.,0.,1.];info.d=[0.]*5;info.distortion_model='plumb_bob'
        rgb=Image();rgb.header=info.header;rgb.width=640;rgb.height=480;rgb.step=1920;rgb.encoding='rgb8';rgb.data=color.tobytes()
        depth=Image();depth.header.stamp=info.header.stamp;depth.header.frame_id='other_camera' if mismatch else info.header.frame_id
        depth.width=640;depth.height=480;depth.step=2560;depth.encoding='32FC1';depth.data=distance.tobytes()
        color_pub.publish(rgb);depth_pub.publish(depth)
        for _ in range(3):executor.spin_once(timeout_sec=.005)
        info_pub.publish(info)
        for _ in range(15):executor.spin_once(timeout_sec=.01)
    try:
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline and not any(r['targets'] for r in results):publish()
        valid=[r for r in results if r['targets']]
        assert valid,[(r.get('reason'),r.get('rejected')) for r in results]
        result=valid[-1];assert all(result[k]==v for k,v in identity.items())
        np.testing.assert_allclose(result['targets']['box']['position'],[.02,.01,.03],atol=.002)
        assert result['pose_source']=='rgbd_box_top'
        assert len(lookup_stamps)>=2 and lookup_stamps[0]==lookup_stamps[1]
        before=len(results);deadline=time.monotonic()+3.
        while time.monotonic()<deadline and not any('aligned' in r.get('reason','') for r in results[before:]):publish(True)
        rejected=[r for r in results[before:] if 'aligned' in r.get('reason','')]
        assert rejected and rejected[-1]['targets']=={}
    finally:
        executor.shutdown();detector.destroy_node();probe.destroy_node()
        if initialized and rclpy.ok():rclpy.shutdown()
