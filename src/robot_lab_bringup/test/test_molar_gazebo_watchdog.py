"""Run source callbacks without ROS or an engine; live plant checks are separate."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

ROOT=Path(__file__).resolve().parents[3]

class Twist:
    def __init__(self):
        self.linear=NS(x=0.)
        self.angular=NS(z=0.)

class Array:
    def __init__(self,data): self.data=data

class Publisher:
    def __init__(self): self.messages=[]
    def publish(self,msg): self.messages.append(msg)

def source_methods():
    path=ROOT/'src/robot_lab_bringup/scripts/molar_gazebo_control.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef))
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('tick','feedback','drive')]
    namespace=dict(math=math,time=NS(monotonic=lambda:10.),Twist=Twist,Float64MultiArray=Array)
    exec(compile(ast.Module(body=methods,type_ignores=[]),str(path),'exec'),namespace)
    return namespace

def fixture():
    names=['conveyor_lift_joint','telescope_stage1_joint','telescope_stage2_joint']
    contract=NS(joints=dict.fromkeys(names+['transfer_roller_joint']), references=dict(zip(names,[.6,.14,.14])), stops=0)
    def stop(reason):
        contract.stops+=1
        contract.references=dict(zip(names,[.2,.03,.04]))
    contract.stop=stop
    obj=NS(articulation=NS(contract=contract,base_inhibited=False),last_sim=None,
           get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000)),
           feedback_at=9.,feedback_stale=False,command_at=9.9,command=Twist(),
           pos_names=names,position_pub=Publisher(),roller_pub=Publisher(),drive_pub=Publisher(),
           positions={'keep':1.},velocities={'keep':0.})
    obj.command.linear.x=.25
    return obj

def test_stale_feedback_revokes_old_goal_and_holds_once():
    obj=fixture(); tick=source_methods()['tick']
    tick(obj)
    assert obj.position_pub.messages[-1].data==[.2,.03,.04]
    assert obj.roller_pub.messages[-1].data==[0.]
    assert obj.drive_pub.messages[-1].linear.x==0.
    assert obj.command_at==-math.inf
    tick(obj)
    assert obj.articulation.contract.stops==1

def test_invalid_feedback_preserves_last_finite_values():
    obj=fixture(); feedback=source_methods()['feedback']
    feedback(obj,NS(name=['conveyor_lift_joint'],position=[math.nan],velocity=[0.]))
    assert obj.positions=={'keep':1.} and obj.velocities=={'keep':0.}
    assert obj.feedback_at==-math.inf

def test_complete_finite_feedback_admitted_atomically():
    obj=fixture(); feedback=source_methods()['feedback']
    names=[*obj.articulation.contract.joints,'drive_left_joint','drive_right_joint']
    feedback(obj,NS(name=names,position=[0.]*len(names),velocity=[0.]*len(names)))
    assert obj.feedback_at==10.
    assert set(obj.positions)==set(names) and set(obj.velocities)==set(names)
