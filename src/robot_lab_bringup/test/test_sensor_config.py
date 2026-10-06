"""Source sensor contracts fail loudly instead of producing a fallback sensor."""
import math

import pytest

from robot_lab_utils.sensor_config import sensor_parameters


def test_absent_configuration_preserves_legacy_defaults():
    assert sensor_parameters('') == {}
    assert sensor_parameters('{}') == {}
    assert sensor_parameters(None) == {}


def test_original_sensor_frame_and_calibration_survive_serialization():
    params = sensor_parameters('{"laser_link_name":"rplidar_link", "scan_samples":640,'
                               '"camera_horizontal_fov":1.047, "camera_near":0.3,'
                               '"camera_far":100, "camera_rate":5}',
                               '<robot name="t4"><link name="rplidar_link"/></robot>')
    assert params['scan_samples'] == 640
    assert params['laser_link_name'] == 'rplidar_link'
    assert params['camera_horizontal_fov'] == pytest.approx(1.047)
    assert isinstance(params['camera_far'], float)


@pytest.mark.parametrize('config', [
    [], {'laser_link_name': 'missing'}, {'laser_link_name': 'bad frame'},
    {'camera_optical_frame': ''}, {'scan_samples': 640.5}, {'scan_samples': True},
    {'camera_width': 1}, {'camera_rate': -1}, {'camera_rate': math.inf},
    {'camera_horizontal_fov': math.pi}, {'camera_horizontal_fov': 0},
    {'camera_near': 1, 'camera_far': 0.5},
    {'scan_range_min': 0, 'scan_range_max': 12}, {'unknown_sensor': 1},
])
def test_invalid_import_cannot_silently_advertise_a_different_sensor(config):
    with pytest.raises(ValueError):
        sensor_parameters(config, '<robot name="fixture"><link name="base"/></robot>')
