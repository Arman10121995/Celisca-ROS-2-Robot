"""Sensor approvals must never admit extra collision/visual geometry."""
import hashlib
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_lab_utils.aerial_takeoff import x500_envelope


def model(tmp_path,geometry=''):
    (tmp_path/'x500').mkdir();(tmp_path/'x500_base').mkdir()
    (tmp_path/'x500_base/model.sdf').write_text('<sdf><model><link name="base_link"><collision><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></sdf>')
    path=tmp_path/'x500/model.sdf'
    path.write_text('<sdf><model><include><uri>model://x500_base</uri></include><link name="range_sensor">'+geometry+'</link></model></sdf>')
    link=ET.parse(path).getroot().find('model/link')
    return {'range_sensor':hashlib.sha256(ET.tostring(link)).hexdigest()}


def test_only_explicit_identical_noncolliding_sensor_is_admitted(tmp_path):
    approved=model(tmp_path)
    with pytest.raises(ValueError):x500_envelope(tmp_path)
    result=x500_envelope(tmp_path,sensor_link_hashes=approved)
    assert result['lower'][2]==-.5
    assert result['upper'][2]==.56
    with pytest.raises(ValueError):x500_envelope(tmp_path,sensor_link_hashes={'range_sensor':'changed'})


@pytest.mark.parametrize('tag',['collision','visual'])
def test_even_matching_hash_cannot_approve_added_geometry(tmp_path,tag):
    approved=model(tmp_path,'<'+tag+'><geometry><box><size>8 8 8</size></box></geometry></'+tag+'>')
    with pytest.raises(ValueError):x500_envelope(tmp_path,sensor_link_hashes=approved)
