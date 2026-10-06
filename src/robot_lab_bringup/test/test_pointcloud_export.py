"""Verify PCD export against ROS messages, including packed float RGB."""
import importlib.util
import struct
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip('sensor_msgs_py')
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header

path = Path(__file__).resolve().parents[1] / 'scripts/export_3d_map.py'
spec = importlib.util.spec_from_file_location('export_3d_map', path)
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


@pytest.mark.parametrize('rgb',[False,True])
def test_pcd_uses_real_points_and_preserves_packed_rgb(tmp_path, rgb):
    fields = [PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
              for i,name in enumerate(('x','y','z'))]
    points = [(1.0,2.0,3.0),(float('nan'),2.0,3.0)]
    if rgb:
        fields.append(PointField(name='rgb',offset=12,datatype=PointField.FLOAT32,count=1))
        color = struct.unpack('f',struct.pack('I',0x123456))[0]
        points = [(*point,color) for point in points]
    message = create_cloud(Header(frame_id='map'),fields,points)
    output = tmp_path/'map.pcd'
    exporter.save_pcd_from_msg(message,output)
    text = output.read_text()
    assert 'POINTS 1\n' in text
    assert text.split('DATA ascii\n')[1].strip() == ('1.0 2.0 3.0 1193046' if rgb else '1.0 2.0 3.0')


def test_empty_or_nonfinite_cloud_does_not_create_artifact(tmp_path):
    fields = [PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
              for i,name in enumerate(('x','y','z'))]
    message = create_cloud(Header(),fields,[(float('inf'),0.0,0.0)])
    output = tmp_path/'map.pcd'
    with pytest.raises(ValueError,match='no finite XYZ'):
        exporter.save_pcd_from_msg(message,output)
    assert not output.exists()


@pytest.mark.integration
def test_live_latched_cloud_export_exits_after_writing(tmp_path, monkeypatch):
    """A valid artifact must not leave a callback blocked in DDS shutdown."""
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    monkeypatch.setenv('ROS_DOMAIN_ID', '227')
    context = Context()
    context.init()
    node = Node('export_test_publisher', context=context)
    publisher = node.create_publisher(PointCloud2, '/export_test_cloud',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z'))]
    message = create_cloud(Header(frame_id='map'), fields, [(1., 2., 3.)])
    output = tmp_path/'live.pcd'
    child = subprocess.Popen([sys.executable, str(path), '--cloud-topic',
        '/export_test_cloud', '--output', str(output)], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, env=os.environ.copy())
    try:
        deadline = time.monotonic()+8
        while child.poll() is None and time.monotonic()<deadline:
            publisher.publish(message)
            time.sleep(.05)
        logs, _ = child.communicate(timeout=2)
        assert child.returncode == 0, logs
        assert 'POINTS 1\n' in output.read_text()
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=2)
        node.destroy_node()
        context.shutdown()


@pytest.mark.integration
def test_live_export_missing_topic_exits_with_failure(tmp_path):
    output = tmp_path/'missing.pcd'
    code = ('import importlib.util,sys; '
            's=importlib.util.spec_from_file_location("exporter",sys.argv[1]); '
            'm=importlib.util.module_from_spec(s); s.loader.exec_module(m); '
            'sys.exit(0 if m.export_pcd_live("/no_export_source",sys.argv[2],.4) else 1)')
    result = subprocess.run([sys.executable, '-c', code, str(path), str(output)],
        capture_output=True, text=True, timeout=5, env=dict(os.environ, ROS_DOMAIN_ID='228'))
    assert result.returncode == 1, result.stdout+result.stderr
    assert not output.exists()
