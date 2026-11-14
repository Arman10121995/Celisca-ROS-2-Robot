"""Real receipts, source guards and deadlines, without simulated mission claims."""
from types import SimpleNamespace
import pytest
from robot_lab_utils.guarded_sequence import GuardedSequence


def fixture():
    calls=[];stops=[];state={'receipt':('pending','waiting')}
    port=SimpleNamespace(guard=lambda:None,begin=lambda step:calls.append(step) or len(calls),
        observe=lambda step,token:state['receipt'],stop=stops.append,record=lambda **data:None)
    return GuardedSequence(port),calls,stops,state


def test_starting_a_bounded_sequence_does_not_dispatch_or_infer_completion():
    engine,calls,stops,state=fixture();engine.start(['plan','move','release'])
    assert not calls
    engine.tick();assert calls==['plan'] and engine.index==0
    for _ in range(5):engine.tick()
    assert calls==['plan'] and engine.index==0 and engine.active
    state['receipt']=('succeeded','measured result');engine.tick();assert engine.index==1
    engine.tick();assert calls==['plan','move']
    engine.cancel();assert len(stops)==1 and not engine.active
    engine.tick();engine.cancel();assert len(stops)==1


def test_failed_guard_or_receipt_stops_without_next_stage():
    engine,calls,stops,state=fixture();engine.start(['move','release']);engine.tick()
    state['receipt']=('failed','source action rejected');engine.tick()
    assert calls==['move'] and stops and engine.state=='failed'
    engine,calls,stops,state=fixture()
    def lost():raise ValueError('Source changed')
    engine.port.guard=lost;engine.start(['move']);engine.tick()
    assert not calls and stops and engine.state=='failed'


def test_deadline_is_terminal_and_measured_completion_requires_every_receipt():
    engine,calls,stops,state=fixture();engine.start(['move']);engine.stage_started-=61;engine.tick()
    assert not calls and stops and engine.state=='failed'
    engine,calls,stops,state=fixture();engine.start(['pick','place']*3)
    state['receipt']=('succeeded','actual interface receipt fixture')
    for _ in range(12):engine.tick()
    assert len(calls)==6 and not engine.active and engine.state=='measured_stages_reached'
    with pytest.raises(ValueError):engine.start(['unbounded']*129)
