"""The gait launch exposes the MuJoCo base force pulse, off by default (R10.6 push recovery, 2026-11-20)."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
pytest.importorskip('launch')


def test_perturbation_arguments_default_to_no_push():
    spec = importlib.util.spec_from_file_location('gait_launch', ROOT/'src/robot_lab_bringup/launch/gait_simulator.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from launch.actions import DeclareLaunchArgument
    declared = {action.name: action.default_value for action in module.generate_launch_description().entities
                if isinstance(action, DeclareLaunchArgument)}
    for key in ('force_n', 'start_s', 'duration_s', 'axis'):
        assert 'perturbation_'+key in declared
    text = lambda name: ''.join(part.text for part in declared[name])
    assert text('perturbation_force_n') == '0.0' and text('perturbation_axis') == '1'
