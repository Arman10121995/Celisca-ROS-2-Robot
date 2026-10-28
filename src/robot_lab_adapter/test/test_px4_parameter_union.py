"""PX4 integer parameters use MAVLink's bytewise float union."""
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import pytest

pytest.importorskip('pymavlink')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_lab_adapter.px4_mavlink import Flight


@pytest.mark.parametrize('value',[-20,0,1,4,500])
def test_native_integer_wire_union_round_trip(value):
    wire=Flight.encode_parameter(value,6)
    assert struct.unpack('<i',struct.pack('<f',wire))[0]==value
    flight=Flight.__new__(Flight);flight.expected_system=2;flight.parameters={}
    message=SimpleNamespace(get_srcSystem=lambda:2,get_type=lambda:'PARAM_VALUE',
        param_id=b'COM_OBL_RC_ACT\x00',param_type=6,param_value=wire)
    flight._absorb(message)
    assert flight.parameters['COM_OBL_RC_ACT']==value
    assert Flight.encode_parameter(.5,9)==.5


def test_another_fcu_cannot_confirm_this_members_parameters():
    flight=Flight.__new__(Flight);flight.expected_system=2;flight.parameters={}
    message=SimpleNamespace(get_srcSystem=lambda:1,get_type=lambda:'PARAM_VALUE',
        param_id='EKF2_HGT_REF',param_type=6,param_value=Flight.encode_parameter(1,6))
    flight._absorb(message)
    assert flight.parameters=={}
