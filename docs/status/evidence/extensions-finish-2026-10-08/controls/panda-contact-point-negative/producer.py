"""Owned real Tk Hand/Arm control and independent native joint/object state."""
import hashlib,json,math,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from robot_lab_gui.launcher import SimulationLauncherGui
root=Path.cwd();out=Path(os.environ['PANDA_HAND_OUT']);out.mkdir(parents=True,exist_ok=False)
(out/'producer.py').write_bytes(Path(__file__).read_bytes())
app=SimulationLauncherGui();app.update();observer=None;samples=[];objects=[];states=[]
report=dict(passed=False,scope='Native Panda / MuJoCo / nav_empty / actual Tk Hand and Arm controls, physical cube and independent state; no other backend or dexterous-hand claim')
def pump():
 app.update()
 if app.process:(out/'live-gui-output.log').write_text(app.output.get('1.0','end'))
 if observer:rclpy.spin_once(observer,timeout_sec=0.)
 time.sleep(.005)
def until(predicate,timeout=30):
 end=time.monotonic()+timeout
 while not predicate() and time.monotonic()<end:pump()
 assert predicate(),'Measured condition did not occur'
def wait_sim(duration):
 start=objects[-1]['time']
 until(lambda:objects[-1]['time']>=start+duration,max(30,duration*5))
def phase(name,function):
 first=len(samples);begin=time.monotonic();function();wait_sim(.25)
 report[name]=dict(wall_s=time.monotonic()-begin,joints=samples[first:],objects=objects[-10:],status=dict(app.hand_tab.state))
try:
 app.robot_var.set('menagerie_franka_emika_panda');app.simulator_var.set('mujoco');app.map_var.set('nav_empty');app.mode_var.set('display');app.gui_var.set('false');app._update_from_selection();app.update()
 app.hand_tab.fixture_var.set(True);app._update_validation_and_command();app.update()
 report['command']=app.command_var.get();assert 'grasp_fixture:=true' in report['command'];assert app.start_button.instate(['!disabled'])
 profile=app.robot_profiles[app.robot_var.get()];files={};installed={}
 for package,names in (('robot_lab_mujoco',('native_asset_display.py','native_arm_control.py','native_gripper_control.py')),('robot_lab_gui',('launcher.py','lab_tabs.py','hand_tab.py'))):
  for name in names:
   path=root/'install'/package/'lib/python3.10/site-packages'/package/name
   installed[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
 for package in ('robot_lab_gui','robot_lab_mujoco','robot_lab_bringup','robot_lab_utils'):
  for path in (root/'src'/package).rglob('*'):
   if path.is_file() and path.suffix in ('.py','.yaml','.xml'):
    files[str(path.relative_to(root))]=hashlib.sha256(path.read_bytes()).hexdigest()
 (out/'source-manifest.json').write_text(json.dumps(dict(profile=profile,source_sha256=files,installed_sha256=installed,native_xml_sha256=hashlib.sha256(Path(profile['native_mjcf']).read_bytes()).hexdigest(),producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
 app._start_launch();app._ensure_ros_publisher();observer=Node('panda_gripper_truth_observer')
 def joints(m):samples.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,position=dict(zip(m.name,m.position)),velocity=dict(zip(m.name,m.velocity)),effort=dict(zip(m.name,m.effort))))
 observer.create_subscription(JointState,'/joint_states',joints,10)
 observer.create_subscription(String,'/manipulation/object_state',lambda m:objects.append(json.loads(m.data)),10)
 observer.create_subscription(String,'/gripper/status',lambda m:states.append(json.loads(m.data)),10)
 until(lambda:app.arm_tab.ready() and app.hand_tab.ready() and len(samples)>5 and len(objects)>5,60)
 wait_sim(1);initial=objects[-1]['position'];report['initial_object']=initial
 assert initial[2]>.2 and math.dist(initial,objects[-5]['position'])<.003
 assert abs(samples[-1]['position']['finger_joint1']-samples[-1]['position']['finger_joint2'])<.001
 def close():
  app.hand_tab.force_var.set(.5);app.hand_tab.close_button.invoke()
  until(lambda:app.hand_tab.handle is not None);until(lambda:not app.hand_tab.state['busy'] and app.hand_tab.handle is None)
  assert 'contact stall' in app.hand_tab.state['status'],app.hand_tab.state
  assert .015<app.hand_tab.state['opening_m']<.04
  assert app.hand_tab.state['actuator_effort_per_finger_n']<=.5+1e-6
  for contacts in app.hand_tab.state['contacts'].values():assert any(c['body']=='manipulation_object' and c['normal_force_n']>.05 for c in contacts)
 phase('close',close)
 def lift():
  app.arm_tab.step_var.set(.15);app.arm_tab.jog_buttons[2].invoke()
  until(lambda:app.arm_tab.handle is not None);until(lambda:not app.arm_tab.state['busy'] and app.arm_tab.handle is None)
  assert 'target reached' in app.arm_tab.state['status'],app.arm_tab.state
  wait_sim(2)
  assert objects[-1]['position'][2]>initial[2]+.04,(initial,objects[-1])
 phase('lift_and_hold',lift)
 before_release=objects[-1]['position'][2]
 def release():
  app.hand_tab.open_button.invoke();until(lambda:app.hand_tab.handle is not None)
  until(lambda:not app.hand_tab.state['busy'] and app.hand_tab.handle is None)
  assert app.hand_tab.state['opening_m']>.075
  wait_sim(2);assert objects[-1]['position'][2]<before_release-.03
 phase('release',release)
 client=observer.create_client(Trigger,'/robot_lab/reset');assert client.wait_for_service(timeout_sec=5)
 before=objects[-1]['time'];future=client.call_async(Trigger.Request());until(future.done);assert future.result().success
 wait_sim(1);assert objects[-1]['time']>before and math.dist(objects[-1]['position'],initial)<.004
 report['reset']=dict(response=future.result().message,object=objects[-1])
 from control_msgs.action import GripperCommand
 from rclpy.action import ActionClient
 action=ActionClient(observer,GripperCommand,'/gripper/gripper_action');assert action.server_is_ready()
 rejected=[]
 for gap,force in ((.081,.5),(.04,21.),(float('nan'),.5)):
  goal=GripperCommand.Goal();goal.command.position=gap;goal.command.max_effort=force
  request=action.send_goal_async(goal);until(request.done);assert not request.result().accepted
  rejected.append(dict(gap=str(gap),force=force,accepted=request.result().accepted))
 report['rejected_commands']=rejected
 def running_close():
  app.hand_tab.close_button.invoke();until(lambda:app.hand_tab.handle is not None and app.hand_tab.state['busy'])
  wait_sim(.25)
 def cancel():
  running_close();app.hand_tab.cancel_button.invoke()
  until(lambda:app.hand_tab.handle is None and not app.hand_tab.state['busy'])
  assert 'canceled' in app.hand_tab.state['status']
  opening=app.hand_tab.state['opening_m'];wait_sim(1)
  assert abs(app.hand_tab.state['opening_m']-opening)<.002
 phase('cancel',cancel)
 def reopen():
  app.hand_tab.open_button.invoke();until(lambda:app.hand_tab.handle is not None)
  until(lambda:app.hand_tab.handle is None and not app.hand_tab.state['busy'])
  assert app.hand_tab.state['opening_m']>.075
 reopen()
 def stop():
  running_close();app.hand_tab.stop_button.invoke()
  until(lambda:app.hand_tab.handle is None and not app.hand_tab.state['busy'])
  assert 'stopped' in app.hand_tab.state['status']
  opening=app.hand_tab.state['opening_m'];wait_sim(1)
  assert abs(app.hand_tab.state['opening_m']-opening)<.002
 phase('stop',stop);reopen()
 def loss():
  running_close();publish=app.hand_tab.heartbeat_pub.publish
  app.hand_tab.heartbeat_pub.publish=lambda _:None;begin=time.monotonic()
  try:
   until(lambda:app.hand_tab.handle is None and not app.hand_tab.state['busy'],3)
   report['heartbeat_loss_wall_s']=time.monotonic()-begin
   assert report['heartbeat_loss_wall_s']<1.3
   assert 'heartbeat lost' in app.hand_tab.state['status']
   opening=app.hand_tab.state['opening_m'];wait_sim(1)
   assert abs(app.hand_tab.state['opening_m']-opening)<.002
  finally:app.hand_tab.heartbeat_pub.publish=publish
 phase('publisher_loss',loss);reopen()
 app._update_reset_button();assert app.reset_robot_button.instate(['!disabled'])
 running_close();before=objects[-1]['time'];app.reset_robot_button.invoke()
 until(lambda:app.hand_tab.handle is None and not app.hand_tab.state['busy'])
 wait_sim(1);assert objects[-1]['time']>before and math.dist(objects[-1]['position'],initial)<.004
 assert app.hand_tab.state['opening_m']>.075
 report['gui_reset']=dict(object=objects[-1],gripper_status=app.hand_tab.state)
 app._stop_launch();until(lambda:not app._launch_running,30);assert app.process.poll()==0
 report['launch_returncode']=app.process.poll();report['passed']=True
finally:
 if app.process and app.process.poll() is None:
  app._stop_launch();end=time.monotonic()+30
  while app._launch_running and time.monotonic()<end:pump()
 report['launch_returncode']=app.process.poll() if app.process else None
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
 (out/'truth.trace.json').write_text(json.dumps(dict(joints=samples,objects=objects,gripper_status=states),indent=2)+'\n')
 (out/'gui-output.log').write_text(app.output.get('1.0','end'))
 if observer:observer.destroy_node()
 app.destroy()
