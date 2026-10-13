#!/usr/bin/env python3
"""Replay a bag's /joint_states and /imu/out through LegOdometry and score it.

Compares the integrated leg odometry with /odom/ground_truth from the same
bag: displacement drift (m and % of the distance walked) and forward-speed
bias/RMS while moving.  Used to tune stance detection offline.

  python3 scripts/leg_odometry_replay.py <bag dir> <expanded urdf>
"""
import os
import sqlite3, glob, sys, math, numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'robot_lab_algorithms'))
from robot_lab_algorithms.leg_odometry import LegKinematics, LegOdometry, quaternion_matrix
bag=sys.argv[1]; urdf=open(sys.argv[2]).read()
db=glob.glob(bag+'/*.db3')[0]; c=sqlite3.connect(db)
ids={n:i for i,n in c.execute('select id,name from topics')}
def load(t,T): return [(ts,deserialize_message(d,T)) for ts,d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ids[t],))]
st=lambda h:h.stamp.sec+h.stamp.nanosec*1e-9
js=load('/joint_states',JointState); imu=load('/imu/out',Imu); tr=load('/odom/ground_truth',Odometry)
events=sorted([(ts,'j',m) for ts,m in js]+[(ts,'i',m) for ts,m in imu], key=lambda e:e[0])
odo=LegOdometry(LegKinematics(urdf)); last_imu=None; out=[]
for ts,k,m in events:
    if k=='i':
        q=m.orientation; last_imu=(quaternion_matrix(q.x,q.y,q.z,q.w),(m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z))
    elif last_imu is not None:
        odo.update(st(m.header), dict(zip(m.name,m.position)), dict(zip(m.name,m.velocity)), *last_imu)
        out.append((st(m.header), odo.x, odo.y, odo.velocity[0]))
tt=np.array([st(m.header) for _,m in tr]); tp=np.array([(m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x) for _,m in tr])
o=np.array(out)
i0=np.searchsorted(tt,o[0,0]); i1=min(np.searchsorted(tt,o[-1,0]),len(tt)-1)
dtruth=tp[i1,:2]-tp[i0,:2]; dleg=o[-1,1:3]-o[0,1:3]
walked=np.sum(np.hypot(*np.diff(tp[i0:i1+1,:2],axis=0).T))
vt=np.interp(o[:,0],tt,tp[:,2]); moving=vt>0.1
print('walked %.2f m, drift %.3f m (%.1f%%), vx bias %.3f rms %.3f'%(walked, np.linalg.norm(dleg-dtruth), 100*np.linalg.norm(dleg-dtruth)/walked, (o[moving,3]-vt[moving]).mean(), np.sqrt(((o[moving,3]-vt[moving])**2).mean())))
