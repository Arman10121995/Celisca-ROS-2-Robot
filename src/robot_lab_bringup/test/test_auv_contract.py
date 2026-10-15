"""Pinned model and actuator/frame contracts, not AUV mission certificates."""
from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
FOLDER=ROOT/'src/robot_lab_robots/tethys_auv'
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))


def test_retained_native_model_is_exact_pinned_source():
    source=FOLDER/'source/model.sdf'
    assert hashlib.sha256(source.read_bytes()).hexdigest()=='a9e6b39850139ce93cbe7b3198cc5e094541a28b81b14553b4db62ccca98ae16'
    metadata=json.loads((FOLDER/'source/fuel-metadata.json').read_text())
    assert metadata['version']==3
    assert metadata['license_name']=='Creative Commons Attribution 4.0 International'
    original=ET.parse(source).getroot().find('model');derived=ET.parse(FOLDER/'model.sdf').getroot().find('model')
    for link in original.findall('link'):
        other=derived.find("link[@name='%s']"%link.get('name'))
        def signature(item):return item.tag,tuple(sorted(item.attrib.items())),(item.text or '').strip(),tuple(signature(child) for child in item)
        assert signature(link.find('inertial'))==signature(other.find('inertial'))
        # Serialization indentation can differ outside the retained source.
        for collision in link.findall('collision'):
            assert collision.findtext('geometry/box/size')==other.find("collision[@name='%s']"%collision.get('name')).findtext('geometry/box/size')


def test_internal_trim_matches_actual_collision_displacement():
    model=ET.parse(FOLDER/'model.sdf').getroot().find('model');c=json.loads((FOLDER/'calibration.json').read_text())
    volume=sum(np.prod(list(map(float,collision.findtext('geometry/box/size').split()))) for link in model.findall('link') for collision in link.findall('collision'))
    mass=sum(float(link.findtext('inertial/mass')) for link in model.findall('link'))
    assert mass==pytest.approx(volume*1025.,abs=1e-8)
    assert c['source_mass_kg']==pytest.approx(148.3571)
    assert c['ballast_mass_kg']==pytest.approx(3.7089275)
    assert model.find("link[@name='lab_trim_ballast']/collision") is None


def test_native_fins_are_bounded_force_pid_not_position_writes():
    model=ET.parse(FOLDER/'model.sdf').getroot().find('model')
    fins=[p for p in model.findall('plugin') if p.get('name').endswith('::JointPositionController')]
    assert {p.findtext('joint_name') for p in fins}=={'horizontal_fins_joint','vertical_fins_joint'}
    for p in fins:
        assert p.findtext('use_velocity_commands')=='false'
        assert float(p.findtext('cmd_min'))==-2.
        assert float(p.findtext('cmd_max'))==2.
    prop=next(p for p in model.findall('plugin') if p.get('name').endswith('::Thruster'))
    assert float(prop.findtext('min_thrust_cmd'))==-35.
    assert float(prop.findtext('max_thrust_cmd'))==35.
    assert float(model.findtext("joint[@name='propeller_joint']/axis/limit/effort"))==2.
    hydro=next(p for p in model.findall('plugin') if p.get('name').endswith('::Hydrodynamics'))
    assert float(hydro.findtext('zDotW'))>0.,'Installed Fortress uses -Ma*acceleration; a negative mass amplifies acceleration'


@pytest.mark.parametrize('linear_y,angular_x,value',[(.1,0.,.1),(0.,.1,.1),(0.,0.,float('nan'))])
def test_unsupported_or_nonfinite_drive_cannot_reach_propulsion(linear_y,angular_x,value):
    from types import SimpleNamespace
    from geometry_msgs.msg import Twist
    from robot_lab_utils.auv_controller import AuvController
    result=[];dummy=SimpleNamespace(enabled=True,stop=lambda reason:result.append(reason))
    msg=Twist();msg.linear.x=value;msg.linear.y=linear_y;msg.angular.x=angular_x
    AuvController._drive(dummy,msg)
    assert result
