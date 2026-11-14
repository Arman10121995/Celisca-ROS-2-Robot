"""Start the shared-world launch in its own session; print its PID."""
import os, subprocess, sys
log = open(sys.argv[2], 'w')
cmd = ['ros2', 'launch', 'robot_lab_bringup', 'shared_world.launch.py', 'session:=' + sys.argv[1],
       'navigation:=true', 'gui:=false', 'rviz:=false'] + sys.argv[3:]
p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
print(p.pid)
