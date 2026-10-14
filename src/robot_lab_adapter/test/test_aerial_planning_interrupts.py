"""Real planner event/control contracts with fixtures, never flight evidence."""
from types import SimpleNamespace
import time

import pytest

pytest.importorskip('pymavlink', reason='PX4 SDK: run these checks in the existing SSD PX4 venv')
import rclpy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from robot_lab_adapter.px4_planning import PX4Planning
from robot_lab_utils.observed_aerial_scene import VoxelObstacles


@pytest.fixture
def planner(tmp_path,monkeypatch):
    world = tmp_path/'fixture.world'
    world.write_text('<sdf version="1.7"><world name="fixture"/></sdf>')
    static = SimpleNamespace(clearance=lambda _position:20.,resources=[],resource_stamps={})
    from robot_lab_utils.aerial_planning import StaticScene
    class FixtureSceneFactory:
        segment=StaticScene.segment
        def __new__(cls,_path):
            return static
    monkeypatch.setattr('robot_lab_utils.aerial_planning.StaticScene',FixtureSceneFactory)
    # Direct path planning asks the base scene to check a segment. The tests
    # here exercise command/event ownership; geometry has separate checks.
    static.segment=lambda *_a,**_k:True
    controller=SimpleNamespace(phase='flying',route=[],manual=(0.,0.,0.,0.),ground_z=0.,
        command_generation=0,route_guard=None,ready=lambda:True,position=lambda:[0.,0.,2.],
        takeoff_ready=lambda:False,route_status='fixture',target=[0.,0.,2.])
    calls=[]
    controller.set_path=lambda route:calls.append(route)
    rclpy.init()
    node=PX4Planning(controller,str(world))
    try:
        yield node,controller,calls,static
    finally:
        node.close();node.destroy_node();rclpy.shutdown()


def observations(point):
    layer=VoxelObstacles([point])
    return SimpleNamespace(snapshot=lambda:((layer,),dict(scope='explicit synthetic fixture')),
        ready=lambda:True,poll=lambda:None,state=lambda:dict(enabled=True),close=lambda:None)


def arm_execution(node,controller,static):
    node.route=[[0.,0.,2.],[2.,0.,2.]]
    node.start=controller.position()
    node.scene=static
    node.resources={}
    node.planned_at=time.monotonic()
    node.plan_command_generation=controller.command_generation


def test_operator_stop_invalidates_a_completed_background_plan(planner):
    node,controller,calls,_=planner
    goal=PoseStamped()
    goal.header.frame_id='map'
    goal.pose.position.x,goal.pose.position.z=2.,2.
    node.goal(goal)
    node.worker.join(timeout=2.)
    assert not node.worker.is_alive()
    controller.command_generation+=1  # Same epoch change made by Drive/Stop.
    node.poll()
    assert node.state['state']=='invalidated' and not node.route and not calls


def test_new_obstacle_rejects_async_execution_recheck_without_starting_motion(planner):
    node,controller,calls,static=planner
    # ObservedScene uses the production stepping method, not the admission
    # fixture's permissive direct segment helper.
    node.obstacles=observations([1.,0.,2.])
    arm_execution(node,controller,static)
    response=node.execute(Trigger.Request(),Trigger.Response())
    assert response.success and node.state['state']=='checking_execution' and not calls
    node.worker.join(timeout=2.)
    node.poll()
    assert node.state['state']=='rejected' and not calls


def test_operator_stop_during_clear_recheck_prevents_late_execution(planner):
    node,controller,calls,static=planner
    node.obstacles=observations([50.,0.,2.])
    arm_execution(node,controller,static)
    assert node.execute(Trigger.Request(),Trigger.Response()).success
    node.worker.join(timeout=2.)
    controller.command_generation+=1
    node.poll()
    assert node.state['state']=='invalidated' and not calls
