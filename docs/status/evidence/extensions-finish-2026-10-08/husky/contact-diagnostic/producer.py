import pybullet as p,math,json,hashlib,time
from pathlib import Path
model=Path('/workspace/molar/robot_lab_runtime/external_assets/models/asset_husky/729f8aa45ccd86fa33a05e07ef698c52c451cd9c/drive.urdf')
rows=[]
for cone,iterations in [(1,50),(0,50),(1,200),(0,200)]:
 for direction in [1,-1]:
  client=p.connect(p.DIRECT);p.setGravity(0,0,-9.81);p.setTimeStep(1/240)
  p.setPhysicsEngineParameter(enableConeFriction=cone,numSolverIterations=iterations)
  plane=p.createCollisionShape(p.GEOM_PLANE);ground=p.createMultiBody(0,plane)
  p.changeDynamics(ground,-1,lateralFriction=1)
  robot=p.loadURDF(str(model),[0,0,.2],flags=p.URDF_USE_INERTIA_FROM_FILE)
  wheels={p.getJointInfo(robot,j)[1].decode():j for j in range(p.getNumJoints(robot)) if p.getJointInfo(robot,j)[2]!=p.JOINT_FIXED}
  for joint in wheels.values():p.changeDynamics(robot,joint,lateralFriction=1,anisotropicFriction=[1,.5,1])
  for _ in range(480):
   for joint in wheels.values():p.setJointMotorControl2(robot,joint,p.VELOCITY_CONTROL,targetVelocity=0,force=50)
   p.stepSimulation()
  samples=[]
  for i in range(2400):
   desired=direction*min(1,i/240*1.5)
   for name,joint in wheels.items():p.setJointMotorControl2(robot,joint,p.VELOCITY_CONTROL,targetVelocity=(-1 if '_left_' in name else 1)*desired*.5708/2/.1651,force=50)
   p.stepSimulation()
   if i%12==0:
    pose=p.getBasePositionAndOrientation(robot);v=p.getBaseVelocity(robot);q=pose[1];yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    samples.append(dict(t=i/240,position=pose[0],yaw=yaw,vx=v[0][0]*math.cos(yaw)+v[0][1]*math.sin(yaw),wz=v[1][2],contacts=len(p.getContactPoints(bodyA=robot))))
  tail=samples[-40:]
  row=dict(cone=cone,iterations=iterations,direction=direction,tail_vx=sum(s['vx'] for s in tail)/len(tail),tail_wz=sum(s['wz'] for s in tail)/len(tail),tail_wz_min=min(s['wz'] for s in tail),tail_wz_max=max(s['wz'] for s in tail),samples=samples)
  rows.append(row);print({k:v for k,v in row.items() if k!='samples'},flush=True);p.disconnect(client)
Path(__file__).with_name('report.json').write_text(json.dumps(dict(model=str(model),model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),rows=rows),indent=2)+'\n')
