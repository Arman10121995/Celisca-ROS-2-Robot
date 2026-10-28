"""One Harmonic world with distinct, leased PX4 X500 FCUs and ROS workers."""
import argparse
from contextlib import suppress
import fcntl
import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from .aerial_swarm_model import identities
from .px4_sitl_runtime import prepare_world,prepare_model,x500_urdf,HARMONIC_DISCOVERY_PORTS,stop_group


def close_owned(processes):
    """Bounded cleanup of every owned group, even if a ROS destructor fails."""
    for signum,budget in ((signal.SIGINT,3.),(signal.SIGTERM,2.),(signal.SIGKILL,1.)):
        for process in reversed(processes):
            with suppress(ProcessLookupError):os.killpg(process.pid,signum)
        deadline=time.monotonic()+budget
        for process in reversed(processes):
            with suppress(subprocess.TimeoutExpired):
                process.wait(timeout=max(.01,deadline-time.monotonic()))
    # Parent exit is not proof that its Gazebo/transport descendants exited.
    for process in processes:
        with suppress(ProcessLookupError):os.killpg(process.pid,signal.SIGKILL)


def actual_origin(topic,env):
    reader=subprocess.Popen(['gz','topic','-e','-t',topic,'--json-output'],env=env,
        stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,start_new_session=True)
    selector=selectors.DefaultSelector();selector.register(reader.stdout,selectors.EVENT_READ)
    try:
        deadline=time.monotonic()+8.
        while time.monotonic()<deadline:
            if selector.select(.3):
                line=reader.stdout.readline()
                try:
                    p=json.loads(line)['pose']['position'];values=[float(p.get(k,0.)) for k in 'xyz']
                    if all(math.isfinite(v) for v in values):return values
                except (ValueError,KeyError,TypeError):pass
        raise RuntimeError('No actual settled Gazebo body-origin observation: '+topic)
    finally:
        selector.close();stop_group(reader)


def main():
    from robot_lab_utils.process_lifetime import exit_with_parent
    from .aerial_swarm_controller import AerialSwarmController
    exit_with_parent()
    parser=argparse.ArgumentParser();parser.add_argument('--session',required=True)
    parser.add_argument('--px4-root',default=os.environ.get('PX4_ROOT','/workspace/molar/px4/PX4-Autopilot'))
    parser.add_argument('--gui',choices=('true','false'),default='true')
    args=parser.parse_args();session=json.loads(Path(args.session).read_text());members=identities(len(session['members']))
    if [m['id'] for m in members]!=[m['id'] for m in session['members']]:raise ValueError('Unexpected FCU identity manifest')
    root=Path(args.px4_root).resolve();build=root/'build/px4_sitl_default';upstream=root/'Tools/simulation/gz'
    if not (build/'bin/px4').is_file():raise RuntimeError('Provision the existing native PX4 runtime first')
    folder=Path(session['directory']);runtime=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))
    if not str(folder.resolve()).startswith('/workspace/') or folder.stat().st_dev!=Path('/workspace').stat().st_dev:
        raise RuntimeError('All aerial swarm runtime, logs and leases must be on SSD')
    leases=[];processes=[];logs=[];node=None
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        for item in members:
            lock=(runtime/'px4'/('instance-'+str(item['instance'])+'.lock'));lock.parent.mkdir(exist_ok=True)
            lease=lock.open('a');fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB);leases.append(lease)
        (folder/'models').mkdir();(folder/'worlds').mkdir()
        world=folder/'worlds/robot_lab_flight.sdf';prepare_world(session['source_world'],world,upstream)
        env=dict(os.environ,PYTHONUNBUFFERED='1',**HARMONIC_DISCOVERY_PORTS)
        env.update(PX4_GZ_WORLD='robot_lab_flight',PX4_GZ_STANDALONE='1',PX4_SIM_MODEL='gz_x500',PX4_SYS_AUTOSTART='4001',
            PX4_GZ_MODELS=str(folder/'models'),PX4_GZ_WORLDS=str(folder/'worlds'),
            PX4_GZ_PLUGINS=str(build/'src/modules/simulation/gz_plugins'),
            GZ_PARTITION='robot_lab_aerial_swarm_'+session['session_id'],
            GZ_SIM_SERVER_CONFIG_PATH=str(root/'src/modules/simulation/gz_bridge/server.config'),
            PATH=str(build/'bin')+os.pathsep+os.environ['PATH'])
        env['GZ_SIM_RESOURCE_PATH']=os.pathsep.join([str(folder/'models'),str(upstream/'models'),str(Path(session['source_world']).parent),
            str(Path(session['source_world']).parent.parent/'models'),env.get('GZ_SIM_RESOURCE_PATH','')])
        env['GZ_SIM_SYSTEM_PLUGIN_PATH']=env['PX4_GZ_PLUGINS']+os.pathsep+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
        if args.gui=='false':env['HEADLESS']='1'
        description=x500_urdf(upstream)
        import yaml
        for item,requested in zip(members,session['members']):
            item['spawn']=requested['spawn'];model=folder/'models'/item['entity'];prepare_model(model,upstream)
            tree=ET.parse(model/'model.sdf')
            tree.getroot().find('model').set('name',item['entity'])
            if session.get('estimator',{}).get('px4_height_reference')==2:
                from .px4_range_sensor import add_sensor
                item['range_topic']=item['namespace']+'/range_native'
                add_sensor(tree.getroot().find('model'),item['range_topic'])
            for plugin in tree.getroot().findall('model/plugin'):
                for key in ('odom_topic','tf_topic','topic'):
                    entry=plugin.find(key)
                    if entry is not None and entry.text.startswith('/px4/'):
                        entry.text=item['namespace']+entry.text
            tree.write(model/'model.sdf',encoding='unicode',xml_declaration=True)
            item['model_file']=str(model/'model.sdf');(model/'model.config').write_text('<model><name>'+item['entity']+'</name><sdf version="1.9">model.sdf</sdf></model>')
        def start(command,name,environment=env,**kwargs):
            stream=(folder/(name+'.log')).open('w');logs.append(stream)
            proc=subprocess.Popen(command,env=environment,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,**kwargs)
            processes.append(proc);return proc
        start(['gz','sim','-r',*(['-s','--headless-rendering'] if args.gui=='false' else []),str(world),'--force-version','8'],'harmonic')
        for item in members:
            request='sdf_filename:'+json.dumps(item['model_file'])+' name:'+json.dumps(item['entity'])+' allow_renaming:false pose:{position:{'+ ' '.join(k+':'+str(item['spawn'][k]) for k in 'xyz')+'}}'
            spawned=False
            for _ in range(6):
                result=subprocess.run(['gz','service','-s','/world/robot_lab_flight/create','--reqtype','gz.msgs.EntityFactory',
                    '--reptype','gz.msgs.Boolean','--timeout','3000','--req',request],env=env,capture_output=True,text=True,timeout=5)
                if result.returncode==0 and 'data: true' in result.stdout:spawned=True;break
            if not spawned:raise RuntimeError('Actual X500 spawn failed: '+item['id'])
            time.sleep(.4)
            item['origin']=actual_origin(item['namespace']+'/px4/odometry_truth_native',env)
            rootfs=folder/('fcu-'+item['id']);rootfs.mkdir()
            vehicle_env=dict(env,PX4_GZ_MODEL_NAME=item['entity'],
                PX4_PARAM_EKF2_HGT_REF=str(session.get('estimator',{}).get('px4_height_reference',0)),
                PX4_PARAM_EKF2_GPS_CTRL='7' if session.get('estimator',{}).get('px4_height_reference',1)==1 else '5')
            if session.get('estimator',{}).get('px4_height_reference')==2:
                vehicle_env.update(PX4_PARAM_EKF2_RNG_CTRL='2',PX4_PARAM_EKF2_RNG_POS_Z='.05')
            start([str(build/'bin/px4'),'-d','-i',str(item['instance']),str(build/'etc'),'-w',str(rootfs)],
                  item['id']+'-px4',vehicle_env,pass_fds=(leases[item['instance']].fileno(),))
            config=folder/(item['id']+'-description.yaml');config.write_text(yaml.safe_dump({'/**':{'ros__parameters':dict(robot_description=description,
                frame_prefix=item['namespace'].strip('/')+'/',use_sim_time=False)}}))
            start(['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','-r','__ns:='+item['namespace'],'--params-file',str(config)],item['id']+'-description')
        session.update(members=members,px4_root=str(root));Path(args.session).write_text(json.dumps(session,indent=2)+'\n')
        for item in members:
            start([sys.executable,'-m','robot_lab_adapter.px4_swarm_worker','--session',args.session,'--member',item['id']],item['id']+'-worker')
        rclpy.init(args=[]);node=AerialSwarmController(session)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            if any(p.poll() is not None for p in processes):raise RuntimeError('An owned aerial/FCU/ROS process exited; closing shared session')
            if any(Path(stream.name).stat().st_size>128*1024**2 for stream in logs):raise RuntimeError('Aerial session log budget exceeded')
    except KeyboardInterrupt:pass
    finally:
        # rclpy can shut its context down before Node.destroy_node() on SIGINT.
        # Cleanup exceptions must never strand FCUs holding the port leases.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        try:
            if node and rclpy.ok():
                # Keep the actual FCUs/plant alive for bounded landing cleanup.
                # Worker lease loss also requests Land; no pose/disarm shortcut.
                node.mode='idle'
                for member in node.members.values():
                    if member.get('status') and member['status'].get('armed'):
                        try:node.service(member,'land')
                        except ValueError:pass
                deadline=time.monotonic()+12.
                while time.monotonic()<deadline and any(m.get('status') and m['status'].get('armed') for m in node.members.values()):
                    rclpy.spin_once(node,timeout_sec=.05)
            if node:
                with suppress(Exception):node.close()
                with suppress(Exception):node.destroy_node()
        finally:
            close_owned(processes)
            for stream in logs:stream.close()
            for lease in leases:lease.close()
            if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
