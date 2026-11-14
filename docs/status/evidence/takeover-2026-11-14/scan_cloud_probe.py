import json, math, os, signal, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from ament_index_python.packages import get_package_prefix
root=Path(os.environ['ROBOT_LAB_RUNTIME_ROOT'])/'takeover-20261114'
executable=Path(get_package_prefix('pointcloud_to_laserscan'))/'lib/pointcloud_to_laserscan/laserscan_to_pointcloud_node'
result=None
with (root/'converter-node.log').open('w') as log:
    process=subprocess.Popen([str(executable),'--ros-args','-r','scan_in:=/takeover/scan','-r','cloud:=/takeover/cloud'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('takeover_converter_probe')
    messages=[]
    sub=node.create_subscription(PointCloud2,'/takeover/cloud',messages.append,qos_profile_sensor_data)
    pub=node.create_publisher(LaserScan,'/takeover/scan',qos_profile_sensor_data)
    try:
        deadline=time.monotonic()+15
        while time.monotonic()<deadline and not messages:
            if process.poll() is not None:raise RuntimeError('Converter exited: '+str(process.returncode))
            scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp=node.get_clock().now().to_msg()
            scan.angle_min=-math.pi/2;scan.angle_max=math.pi/2;scan.angle_increment=math.pi/2
            scan.range_min=.1;scan.range_max=10.;scan.ranges=[1.,2.,3.]
            pub.publish(scan);rclpy.spin_once(node,timeout_sec=.1)
        if not messages:raise RuntimeError('No converted point cloud in 15 seconds')
        cloud=messages[-1];points=np.array([tuple(v) for v in read_points(cloud,field_names=('x','y','z'),skip_nans=True)])
        expected=np.array([[0.,-1.,0.],[2.,0.,0.],[0.,3.,0.]])
        np.testing.assert_allclose(points,expected,atol=1e-6)
        assert cloud.header.frame_id=='laser'
        result=dict(executable=str(executable),input='Published synthetic three-ray LaserScan; no robot/simulator',
            output_frame=cloud.header.frame_id,points=points.tolist(),expected=expected.tolist(),
            maximum_error_m=float(np.max(abs(points-expected))),passed=True)
        (root/'converter-result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
