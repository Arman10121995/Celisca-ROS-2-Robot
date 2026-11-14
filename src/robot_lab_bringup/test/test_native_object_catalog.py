"""Exact free-box selection/state and selected-world planning scene contracts."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

mj=pytest.importorskip('mujoco')
pytest.importorskip('std_msgs.msg')
from std_msgs.msg import String
from robot_lab_mujoco.native_object_catalog import NativeObjectCatalog


@pytest.fixture
def catalog(tmp_path):
    source=tmp_path/'fixture.xml';source.write_text('''<mujoco><worldbody>
      <body name="base"><geom type="box" size=".1 .1 .1"/>
      <body name="left"><geom type="box" size=".01 .01 .01"/></body>
      <body name="right"><geom type="box" size=".01 .01 .01"/></body></body>
      <body name="environment_cube" pos="1 0 1"><freejoint/>
        <geom type="box" size=".02 .03 .04" pos=".1 0 0"/></body>
      <body name="environment_ball" pos="2 0 1"><freejoint/>
        <geom type="sphere" size=".04"/>
        <body pos="0 0 .2"><geom name="moving_shelf" type="box" size=".1 .1 .02"/></body></body>
      <body name="fixed_box" pos="0 0 .2"><geom name="bench_top" type="box" size=".1 .1 .02"/></body>
      </worldbody></mujoco>''')
    model=mj.MjModel.from_xml_path(str(source));data=mj.MjData(model);mj.mj_forward(model,data)
    published={};node=SimpleNamespace(model=model,data=data,mujoco=mj,robot_bodies=4,object_body=-1,
        arm=SimpleNamespace(reserved=False,dofs=np.array([],dtype=int),servo=SimpleNamespace(active=lambda:False)),
        gripper=SimpleNamespace(reserved=False,finger_bodies=[2,3],contacts=lambda:{}),
        create_publisher=lambda kind,topic,*a:SimpleNamespace(publish=published.setdefault(topic,[]).append),
        create_subscription=lambda *a:None,get_parameter=lambda name:SimpleNamespace(value=str(source)),
        object_pub=SimpleNamespace(publish=published.setdefault('/manipulation/object_state',[]).append))
    return NativeObjectCatalog(node),node,published


def request(catalog,**values):
    return String(data=json.dumps(dict(dict(plant_id=catalog.plant_id,source_sha256=catalog.source_sha,id='environment_cube'),**values)))


def test_selection_is_explicit_and_state_comes_from_actual_collision_geometry(catalog):
    catalog,node,published=catalog;before=node.data.qpos.copy()
    assert catalog.snapshot() is None
    catalog.publish();assert published['/manipulation/object_state']==[]
    catalog.select(request(catalog,plant_id='stale'));assert catalog.selected is None
    catalog.select(request(catalog));assert catalog.selected=='environment_cube' and node.object_body==-1
    catalog.receive_scene(String(data=json.dumps(dict(object_id='environment_cube',ready=True))))
    assert node.object_body==4
    state=catalog.snapshot()
    assert state['size']==[.04,.06,.08]
    np.testing.assert_allclose(state['position'],[1.1,0.,1.])
    assert state['world_model']=='cube' and state['selected_by_operator']
    np.testing.assert_array_equal(node.data.qpos,before)
    assert 'environment_ball' not in catalog.objects and catalog.unsupported
    assert list(catalog.supports)==['bench_top']
    np.testing.assert_allclose(catalog.supports['bench_top']['size'],[.2,.2,.04])
    catalog.publish()
    assert json.loads(published['/manipulation/object_catalog'][-1].data)['supports']==catalog.supports


def test_busy_or_attached_plant_cannot_switch_selection(catalog):
    catalog,node,_=catalog
    node.arm.reserved=True;catalog.select(request(catalog));assert catalog.selected is None
    node.arm.reserved=False;catalog.scene=dict(ready=True,object_attached=True)
    catalog.select(request(catalog));assert catalog.selected is None
    assert 'detach' in catalog.error


def test_world_snapshot_is_retired_once_even_when_the_measured_box_moves():
    pytest.importorskip('moveit_msgs.msg')
    scripts=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(scripts))
    from native_scene_objects import MeasuredObjects
    from native_arm_scene import collision_object
    calls=[]
    future=SimpleNamespace(done=lambda:False,result=lambda:SimpleNamespace(success=True))
    node=SimpleNamespace(create_client=lambda *a:None,create_subscription=lambda *a:None,
        world_model_ids={'cube':['world_original']},client=SimpleNamespace(call_async=lambda r:calls.append(r) or future))
    objects=MeasuredObjects(node,collision_object,False)
    payload=dict(id='environment_cube',frame='native_world',size=[.03]*3,position=[.4,0.,.5],
        orientation_wxyz=[1.,0.,0.,0.],spatial_velocity=[0.]*6,selected_by_operator=True,
        finger_links=['native_body_10','native_body_11'],world_model='cube')
    objects.receive_object(String(data=json.dumps(payload)));objects.poll({})
    from moveit_msgs.msg import CollisionObject
    assert any(c.id=='world_original' and c.operation==CollisionObject.REMOVE for c in calls[0].scene.world.collision_objects)
    future.done=lambda:True;objects.poll({});assert 'world_original' in objects.retired_ids
    payload['position'][0]+=.01;objects.receive_object(String(data=json.dumps(payload)));objects.poll({})
    assert len(calls)==2
    assert all(c.id!='world_original' for c in calls[-1].scene.world.collision_objects)


def test_appended_free_world_body_keeps_authored_pose_in_robot_home_key():
    from robot_lab_utils.native_mjcf_assets import seed_appended_world_keys
    robot=mj.MjSpec.from_string('<mujoco><worldbody><body><joint name="arm"/>'
        '<geom type="sphere" size=".1"/></body></worldbody><keyframe><key name="home" qpos=".3"/></keyframe></mujoco>')
    before=robot.compile()
    world=mj.MjSpec.from_string('<mujoco><worldbody><body name="cube" pos="1 2 .515"><freejoint/>'
        '<geom type="box" size=".015 .015 .015" mass=".035"/></body></worldbody></mujoco>')
    robot.attach(world,frame=robot.worldbody.add_frame(),prefix='environment_')
    seed_appended_world_keys(robot,before.nbody)
    model=robot.compile();data=mj.MjData(model);mj.mj_resetDataKeyframe(model,data,0);mj.mj_forward(model,data)
    np.testing.assert_allclose(data.qpos[0],.3)
    body=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,'environment_cube')
    np.testing.assert_allclose(data.xpos[body],[1.,2.,.515]);np.testing.assert_allclose(data.xquat[body],[1.,0.,0.,0.])


def test_light_cube_support_aggregates_all_four_real_contact_forces(tmp_path):
    source=tmp_path/'supported.xml';source.write_text('''<mujoco><worldbody>
      <geom type="plane" size="3 3 .1"/>
      <body name="base" pos="0 0 1"><geom type="box" size=".1 .1 .1"/>
        <body name="left"><geom type="sphere" size=".01"/></body>
        <body name="right"><geom type="sphere" size=".01"/></body></body>
      <body name="environment_cube" pos="1 0 .015"><freejoint/>
        <geom type="box" size=".015 .015 .015" mass=".035"/></body>
      </worldbody></mujoco>''')
    model=mj.MjModel.from_xml_path(str(source));data=mj.MjData(model)
    for _ in range(1000):mj.mj_step(model,data)
    node=SimpleNamespace(model=model,data=data,mujoco=mj,robot_bodies=4,object_body=4,
        gripper=SimpleNamespace(finger_bodies=[2,3]),create_publisher=lambda *args:None,
        create_subscription=lambda *args:None,get_parameter=lambda _:SimpleNamespace(value=str(source)))
    forces=[]
    for i in range(data.ncon):
        force=np.zeros(6);mj.mj_contactForce(model,data,i,force);forces.append(abs(force[0]))
    assert len(forces)==4 and max(forces)<.1 and sum(forces)>.3
    observation=NativeObjectCatalog(node).snapshot()['custody']
    assert observation['supported'] and observation['speed']<.001


def test_selected_box_retires_its_static_snapshot_and_requires_real_scene_ack():
    pytest.importorskip('moveit_msgs.msg')
    scripts=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(scripts))
    from native_scene_objects import MeasuredObjects
    from native_arm_scene import collision_object
    calls=[]
    node=SimpleNamespace(create_client=lambda *a:None,create_subscription=lambda *a:None,
        world_model_ids={'cube':['robot_lab_world_3']},
        client=SimpleNamespace(call_async=lambda request:(calls.append(request) or SimpleNamespace(done=lambda:False))))
    objects=MeasuredObjects(node,collision_object,False)
    payload=dict(id='environment_cube',frame='native_world',size=[.03,.03,.03],position=[1.,0.,.5],
        orientation_wxyz=[1.,0.,0.,0.],spatial_velocity=[0.]*6,selected_by_operator=True,
        finger_links=['native_body_10','native_body_11'],world_model='cube')
    objects.receive_object(String(data=json.dumps(payload)))
    status={};objects.poll(status)
    assert objects.enabled and status['object_id']=='environment_cube' and not status['ready']
    operations={item.id:item.operation for item in calls[0].scene.world.collision_objects}
    from moveit_msgs.msg import CollisionObject
    assert operations['environment_cube']==CollisionObject.ADD
    assert operations['robot_lab_world_3']==CollisionObject.REMOVE


def test_unbound_world_object_does_not_silently_leave_a_static_planning_duplicate():
    pytest.importorskip('moveit_msgs.msg')
    scripts=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(scripts))
    from native_scene_objects import MeasuredObjects
    from native_arm_scene import collision_object
    node=SimpleNamespace(create_client=lambda *a:None,create_subscription=lambda *a:None,world_model_ids={})
    objects=MeasuredObjects(node,collision_object,False)
    payload=dict(id='environment_unknown',frame='native_world',size=[.03]*3,position=[0.,0.,.5],
        orientation_wxyz=[1.,0.,0.,0.],spatial_velocity=[0.]*6,selected_by_operator=True,
        finger_links=['native_body_10','native_body_11'],world_model='unknown')
    objects.receive_object(String(data=json.dumps(payload)));status={};objects.poll(status)
    assert objects.enabled and not status['ready'] and 'canonical world' in status['object_error']


def test_measured_attachment_transform_is_published_only_after_scene_ack_and_cleared_on_detach():
    pytest.importorskip('moveit_msgs.msg')
    import time
    from concurrent.futures import Future
    from geometry_msgs.msg import PoseStamped
    scripts=Path(__file__).resolve().parents[1]/'scripts';sys.path.insert(0,str(scripts))
    from native_scene_objects import MeasuredObjects
    from native_arm_scene import collision_object
    updates=[]
    def apply(request):
        future=Future();updates.append((request,future));return future
    node=SimpleNamespace(create_client=lambda *a:None,create_subscription=lambda *a:None,
        world_model_ids={'cube':['robot_lab_world_3']},client=SimpleNamespace(call_async=apply))
    objects=MeasuredObjects(node,collision_object,True)
    payload=dict(id='environment_cube',frame='native_world',size=[.03]*3,position=[1.,0.,.5],
        orientation_wxyz=[1.,0.,0.,0.],spatial_velocity=[0.]*6,plant_id='current',source_sha256='source',
        finger_links=['native_body_10','native_body_11'],world_model='cube')
    objects.receive_object(String(data=json.dumps(payload)))
    objects.applied=payload.copy()
    objects.gripper={'contacts':{name:[dict(body='environment_cube',normal_force_n=1.)]
                                for name in ['finger_joint1','finger_joint2']}}
    objects.gripper_received=objects.joints_received=time.monotonic()
    objects.joints={'joint1':.1}
    fk=Future();objects.fk=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda request:fk)
    objects.command='attach';objects.poll({})
    tool=PoseStamped();tool.pose.position.x=.9;tool.pose.position.y=.1;tool.pose.position.z=.4
    tool.pose.orientation.w=1.
    fk.set_result(SimpleNamespace(error_code=SimpleNamespace(val=1),pose_stamped=[tool]))
    pending={};objects.poll(pending)
    assert not objects.attached and objects.attachment is None and pending['object_in_tool'] is None
    # Attached ADD performs MoveIt's world-to-robot transfer. A duplicate
    # world REMOVE rejects the diff after that transfer has already happened.
    scene=updates[-1][0].scene
    assert not scene.world.collision_objects
    assert scene.robot_state.attached_collision_objects[0].object.id=='environment_cube'
    assert scene.robot_state.attached_collision_objects[0].object.header.frame_id=='panda_tcp'
    updates[-1][1].set_result(SimpleNamespace(success=True))
    acknowledged={};objects.poll(acknowledged)
    assert acknowledged['object_attached'] and acknowledged['ready']
    np.testing.assert_allclose(acknowledged['object_in_tool']['position'],[.1,-.1,.1])
    assert acknowledged['object_in_tool']['plant_id']=='current'
    objects.command='detach';objects.poll({})
    updates[-1][1].set_result(SimpleNamespace(success=True))
    detached={};objects.poll(detached)
    assert not detached['object_attached'] and detached['object_in_tool'] is None
