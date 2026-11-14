import subprocess, sys
log = open(sys.argv[1], 'w')
cmd = ['ros2', 'launch', 'robot_lab_bringup', 'marine_simulator.launch.py', 'gui:=false', 'rviz:=false'] + sys.argv[2:]
p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True); print(p.pid)
