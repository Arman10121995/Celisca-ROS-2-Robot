"""Real consumer message contracts: shape, generation, frame and loss gates."""
import numpy as np
from robot_lab_utils.footprint_receipts import FootprintReceipts,matching_shape


POINTS=[[-.2,-.3],[.3,-.3],[.5,.2],[-.2,.3]]


def oriented(points):
    angle=.73
    rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    return (np.asarray(points)@rotation.T+[3.,-2.]).tolist()


def test_current_local_and_global_matching_padded_shapes_required():
    gate=FootprintReceipts();gate.expect(POINTS,10.,7)
    shape=oriented(gate.expected)
    assert gate.observe('local','odom',10.1,shape,10.2,wall=20.)
    assert not gate.ready(10.2,wall=20.)
    assert gate.observe('global','map',10.1,shape,10.2,wall=20.)
    assert gate.ready(10.2,wall=20.)
    assert not gate.ready(13.,wall=20.)
    assert not gate.ready(10.2,wall=24.)


def test_unpadded_wrong_frame_scaled_or_old_polygon_cannot_acknowledge():
    gate=FootprintReceipts();gate.expect(POINTS,10.,7)
    assert not gate.observe('global','odom',10.1,oriented(gate.expected),10.2,wall=20.)
    assert not gate.observe('global','map',9.9,oriented(gate.expected),10.2,wall=20.)
    assert not gate.observe('global','map',10.1,oriented(POINTS),10.2,wall=20.)
    assert not gate.observe('global','map',10.1,oriented(gate.expected*2),10.2,wall=20.)
    assert not gate.observe('global','map',11.,oriented(gate.expected),10.2,wall=20.)


def test_shape_change_new_payload_and_clock_reset_require_new_receipts():
    gate=FootprintReceipts();gate.expect(POINTS,10.,7)
    for name,frame in [('local','odom'),('global','map')]:
        assert gate.observe(name,frame,10.1,oriented(gate.expected),10.2,wall=20.)
    gate.expect(POINTS,10.3,7)
    assert gate.ready(10.3,wall=20.1)  # repeated publication is not a new geometry epoch
    gate.expect(np.roll(POINTS,2,axis=0)[::-1],10.35,7)
    assert gate.ready(10.35,wall=20.1)  # ring ordering is not a geometric change
    gate.expect(POINTS,10.4,8)
    assert not gate.ready(10.4,wall=20.1)
    gate.expect(POINTS,0.,8)
    assert gate.generation==0. and gate.received=={}


def test_ring_order_is_irrelevant_but_malformed_measurements_fail_closed():
    assert matching_shape(POINTS,np.roll(oriented(POINTS),2,axis=0)[::-1])
    assert not matching_shape(POINTS,[[0,0]])
    assert not matching_shape(POINTS,[[float('nan'),0]]*4)
