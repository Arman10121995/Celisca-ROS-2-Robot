"""Authored sensor-source simulation contracts for pressure, DVL and sonar."""

import math
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Range
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))

from robot_lab_utils.marine_sensors import MarineSensors  # noqa: E402
from robot_lab_utils.marine_model import MarineModel  # noqa: E402


@pytest.fixture(scope='module', autouse=True)
def _ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


class _SpyPublisher:
    """Capture messages published through a ROS publisher for test inspection."""

    def __init__(self, publisher):
        self._publisher = publisher
        self.count = 0
        self.published_message = None
        self._original_publish = publisher.publish

    def __getattr__(self, name):
        return getattr(self._publisher, name)

    def publish(self, msg):
        self.count += 1
        self.published_message = msg
        self._original_publish(msg)


def _make_scene(world_file):
    from robot_lab_utils.aerial_planning import StaticScene
    return StaticScene(world_file)


def _make_sensor(scene_file, **kwargs):
    node = MarineSensors()
    node.scene = _make_scene(scene_file)
    node.density = 1025.
    node.surface = 0.
    node.rng = np.random.default_rng(41)
    node.sonar_previous = None
    node.sonar_last_output = -math.inf
    for key, value in kwargs.items():
        setattr(node, key, value)
    return node


def _capture_sonar(node):
    node.sonar_pub = _SpyPublisher(node.sonar_pub)


def test_sonar_publishes_bottom_range():
    scene = _make_scene(ROOT / 'src/robot_lab_maps/maps/marine_tank/worlds/marine_tank.world')
    node = MarineSensors()
    node.scene = scene
    node.density = 1025.
    node.surface = 0.
    node.rng = np.random.default_rng(41)
    node.sonar_previous = None
    node.sonar_last_output = -math.inf
    _capture_sonar(node)

    msg = SimpleNamespace(header=None, pose=None)
    msg.header = Header(frame_id='map', stamp=rclpy.time.Time(seconds=0).to_msg())
    msg.pose = SimpleNamespace(pose=None)
    msg.pose.pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=-1.), orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))

    node.sonar_observe(msg)

    assert node.sonar_pub.count == 1
    published = node.sonar_pub.published_message
    assert published.header.frame_id == 'base_link'
    assert published.radiation_type == Range.ULTRASOUND


def test_sonar_calculates_range_to_seabed():
    scene = _make_scene(ROOT / 'src/robot_lab_maps/maps/marine_tank/worlds/marine_tank.world')
    node = MarineSensors()
    node.scene = scene
    node.density = 1025.
    node.surface = 0.
    node.rng = np.random.default_rng(41)
    node.sonar_previous = None
    node.sonar_last_output = -math.inf
    _capture_sonar(node)

    msg = SimpleNamespace(header=None, pose=None)
    msg.header = Header(frame_id='map', stamp=rclpy.time.Time(seconds=0).to_msg())
    msg.pose = SimpleNamespace(pose=None)
    msg.pose.pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=-1.), orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))

    node.sonar_observe(msg)

    published = node.sonar_pub.published_message
    assert 0.1 < published.range <= 10.
    assert math.isfinite(published.range)


def test_sonar_empty_when_beyond_surface():
    scene = _make_scene(ROOT / 'src/robot_lab_maps/maps/marine_tank/worlds/marine_tank.world')
    node = MarineSensors()
    node.scene = scene
    node.density = 1025.
    node.surface = 0.
    node.rng = np.random.default_rng(41)
    node.sonar_previous = None
    node.sonar_last_output = -math.inf
    _capture_sonar(node)

    msg = SimpleNamespace(header=None, pose=None)
    msg.header = Header(frame_id='map', stamp=rclpy.time.Time(seconds=0).to_msg())
    msg.pose = SimpleNamespace(pose=None)
    msg.pose.pose = SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=1.), orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))

    node.sonar_observe(msg)

    published = node.sonar_pub.published_message
    assert math.isinf(published.range) or published.range > 10.


def test_marine_sensors_without_world_file():
    """MarineSensors() must instantiate with an empty world_file (no map)."""
    node = MarineSensors()
    assert node.scene is None


def test_range_has_no_sonar_constant():
    """Range message has no SONAR radiation_type constant."""
    assert not hasattr(Range, 'SONAR')


def test_range_has_ultrasound_constant():
    """Range message has the ULTRASOUND radiation_type constant."""
    assert hasattr(Range, 'ULTRASOUND')
    assert Range.ULTRASOUND == 0


def test_sonar_does_not_replace_motion_dvl_or_duplicate_pressure():
    from nav_msgs.msg import Odometry
    node=MarineSensors()
    node.scene=SimpleNamespace(clearance=lambda p: p[2]+6.)
    node.velocity_pub=_SpyPublisher(node.velocity_pub)
    node.pressure_pub=_SpyPublisher(node.pressure_pub)
    try:
        for stamp,x in [(0.,0.),(.1,.02)]:
            msg=Odometry();msg.header.frame_id='map'
            msg.header.stamp=rclpy.time.Time(seconds=stamp).to_msg()
            msg.pose.pose.position.x=x;msg.pose.pose.position.z=-1.
            msg.pose.pose.orientation.w=1.
            node.observe(msg)
        assert node.velocity_pub.count==1 and node.pressure_pub.count==2
        assert node.velocity_pub.published_message.twist.twist.linear.x==pytest.approx(.2,abs=.03)
        node.sonar_observe(msg)
        assert node.velocity_pub.count==1 and node.pressure_pub.count==2
    finally:node.destroy_node()


def test_sonar_traces_downward_instead_of_reporting_nearest_side_wall():
    from robot_lab_utils.marine_navigation_model import sonar_range
    scene=SimpleNamespace(clearance=lambda p: min(abs(p[0]-1.),p[2]+6.))
    assert sonar_range(scene,[0.,0.,-1.],[0.,0.,0.])==pytest.approx(5.,abs=.01)
