"""Use actual native state to check ROS joint feedback and keyed free-base spawn."""
import math
from pathlib import Path

import pytest

pytest.importorskip('mujoco')
rclpy = pytest.importorskip('rclpy')
from robot_lab_mujoco.native_asset_display import NativeAssetDisplay
from robot_lab_utils.native_mjcf_assets import export_display_urdf
import mujoco


def test_keyed_native_joint_feedback_and_spawn(tmp_path):
    model = tmp_path/'robot.xml'
    model.write_text('''<mujoco><worldbody><body name="base" pos="1 0 1">
        <freejoint name="floating"/><geom type="sphere" size=".1"/>
        <body name="arm" pos="0 0 .2"><joint name="elbow" type="hinge"/>
        <geom type="capsule" size=".05 .2"/></body></body></worldbody>
        <keyframe><key name="home" qpos="1 0 1 1 0 0 0 .3"/></keyframe></mujoco>''')
    description = tmp_path/'display.urdf'
    export_display_urdf(mujoco.MjModel.from_xml_path(str(model)),description)
    rclpy.init(args=['--ros-args','-p','native_mjcf:='+str(model),'-p','model:='+str(description),
                    '-p','gui:=false','-p','spawn_x:=2.0','-p','spawn_y:=3.0','-p',
                    'spawn_yaw:='+str(math.pi/2)])
    node = None
    try:
        node = NativeAssetDisplay(); messages=[]
        node.joint_pub.publish = messages.append
        node.tick()
        assert messages[0].name == ['elbow']
        assert list(messages[0].position) == pytest.approx([.3])
        assert list(messages[0].velocity) == pytest.approx([0.])
        assert node.data.xpos[1] == pytest.approx([2.,4.,1.])
        assert node.data.time == pytest.approx(.02)
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()
