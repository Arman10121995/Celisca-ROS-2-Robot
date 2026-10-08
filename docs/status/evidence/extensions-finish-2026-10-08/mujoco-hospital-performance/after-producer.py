#!/usr/bin/env python3
"""Read the same bounded clock/scan/depth window after byte-array packing."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
args.output.parent.mkdir(parents=True, exist_ok=False)
rclpy.init()
node = Node('read_only_matched_clock_window_after')
samples=[]
counts={'scan':0, 'depth':0}
last_depth=None

def clock(message):
    samples.append([time.monotonic(),message.clock.sec+message.clock.nanosec*1e-9])

def scan(message):
    counts['scan']+=1

def depth(message):
    global last_depth
    counts['depth']+=1
    last_depth=message

subscriptions=[node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data),
               node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data),
               node.create_subscription(Image,'/oakd/depth/image_raw',depth,qos_profile_sensor_data)]
deadline=time.monotonic()+60
while not samples and time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.1)
start=time.monotonic()
while time.monotonic()-start<15:
    rclpy.spin_once(node,timeout_sec=.1)
report=dict(scope='Read-only same first/last message-arrival clock algorithm and 15 s window on normal Labbot/MuJoCo/hospital v4 navigation after typed-array packing. Single host/window, not sustained speed or general qualification.',
    domain=os.environ['ROS_DOMAIN_ID'],samples=samples,message_counts=counts,
    producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
try:
    assert len(samples)>2 and counts['scan'] and last_depth is not None
    report.update(wall_s=samples[-1][0]-samples[0][0],simulation_s=samples[-1][1]-samples[0][1])
    report['real_time_factor']=report['simulation_s']/report['wall_s']
    assert last_depth.encoding=='32FC1' and last_depth.is_bigendian==0
    assert len(last_depth.data)==last_depth.height*last_depth.step
    pixels=np.frombuffer(last_depth.data,dtype='<f4')
    valid=pixels[np.isfinite(pixels)]
    assert len(valid)>0 and last_depth.header.frame_id
    report['depth']=dict(width=last_depth.width,height=last_depth.height,step=last_depth.step,
        encoding=last_depth.encoding,frame=last_depth.header.frame_id,finite_pixels=len(valid),
        minimum_m=float(valid.min()),maximum_m=float(valid.max()),
        data_sha256=hashlib.sha256(bytes(last_depth.data)).hexdigest())
finally:
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
print({k:v for k,v in report.items() if k!='samples'})
