"""Verify PCD export against ROS messages, including packed float RGB."""
import importlib.util
import struct
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
