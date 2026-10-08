import pybullet as p, math, json, hashlib
from pathlib import Path
model=Path('/workspace/molar/robot_lab_runtime/external_assets/models/asset_husky/729f8aa45ccd86fa33a05e07ef698c52c451cd9c/drive.urdf')
records=[]
for force in (25,50,100):
 client=p.connect(p.DIRECT)
 p.setGravity(0,0,-9.81);p.setTimeStep(1/240)
 plane=p.createCollisionShape(p.GEOM_PLANE);p.createMultiBody(0,plane)
 robot=p.loadURDF(str(model),[0,0,.2],flags=p.URDF_USE_INERTIA_FROM_FILE)
 wheels={p.getJointInfo(robot,j)[1].decode():j for j in range(p.getNumJoints(robot)) if p.getJointInfo(robot,j)[2]!=p.JOINT_FIXED}
 for joint in wheels.values():p.changeDynamics(robot,joint,lateralFriction=1)
 for _ in range(480):
  for joint in wheels.values():p.setJointMotorControl2(robot,joint,p.VELOCITY_CONTROL,targetVelocity=0,force=force)
  p.stepSimulation()
 initial=p.getBasePositionAndOrientation(robot)[0]
 for _ in range(1200):
  for name,joint in wheels.items():p.setJointMotorControl2(robot,joint,p.VELOCITY_CONTROL,targetVelocity=(-1 if '_left_' in name else 1)*.5708/2/.1651,force=force)
  p.stepSimulation()
 state=p.getBasePositionAndOrientation(robot);velocity=p.getBaseVelocity(robot)
 records.append(dict(force=force,initial=initial,final=state,velocity=velocity,wheels={n:p.getJointState(robot,j) for n,j in wheels.items()}))
 p.disconnect(client)
out=Path(__file__).parent/'report.json';out.write_text(json.dumps(dict(model=str(model),sha256=hashlib.sha256(model.read_bytes()).hexdigest(),records=records),indent=2)+'\n')
for r in records:print('force',r['force'],'linear',r['velocity'][0],'angular',r['velocity'][1],'motor',[(n,round(s[1],3),round(s[3],3)) for n,s in r['wheels'].items()])
