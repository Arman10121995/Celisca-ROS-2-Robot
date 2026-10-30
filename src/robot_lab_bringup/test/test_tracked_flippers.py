"""Preserved source versus explicit bounded derivative; not terrain evidence."""
from pathlib import Path
import json,sys,time
import xml.etree.ElementTree as ET
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.tracked_control import FLIPPER_JOINTS,TrackedControl


def test_original_geometry_stays_fixed_and_derivative_has_real_force_pids():
    base=ET.parse(ROOT/'src/robot_lab_robots/lab_tracked/model.sdf').getroot().find('model')
    derivative=ET.parse(ROOT/'src/robot_lab_robots/lab_tracked_articulated/model.sdf').getroot().find('model')
    assert [ET.tostring(link) for link in base.findall('link')]==[ET.tostring(link) for link in derivative.findall('link')]
    for name in FLIPPER_JOINTS:
        original=base.find("joint[@name='%s']/axis/limit"%name)
        derived=derivative.find("joint[@name='%s']/axis/limit"%name)
        assert original.findtext('lower')==original.findtext('upper')=='0'
        assert float(derived.findtext('lower'))==-.8 and float(derived.findtext('upper'))==.8
        plugin=next(p for p in derivative.findall('plugin') if p.findtext('joint_name')==name)
        assert plugin.findtext('use_velocity_commands')=='false'
        assert float(plugin.findtext('cmd_max'))==4 and float(plugin.findtext('cmd_min'))==-4


@pytest.mark.parametrize('values',[[0,0,0], [0,0,0,1.], [float('nan'),0,0,0],[True,0,0,0]])
def test_unbounded_or_malformed_targets_cannot_reach_native_servos(values):
    node=SimpleNamespace(session='actual',robot_id='lab_tracked_articulated',flipper_at=time.monotonic(),
        clock_at=time.monotonic(),flipper_requested=[0.]*4,flipper_output=[.1]*4)
    msg=SimpleNamespace(data=json.dumps(dict(session_id='actual',robot_id=node.robot_id,op='target',positions=values)))
    TrackedControl.flipper_command(node,msg)
    assert node.flipper_requested==[0.]*4
