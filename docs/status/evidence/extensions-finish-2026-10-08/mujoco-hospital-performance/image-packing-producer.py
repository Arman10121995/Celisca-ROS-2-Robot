#!/usr/bin/env python3
"""Measure real ROS image construction and verify lossless wire serialization.

Run twice with sourced ROS/installed packages, before/after the implementation.
This microbenchmark is separate from simulator real-time factor.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Image
import robot_lab_utils.camera_msgs as builders

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', required=True, type=Path)
args = parser.parse_args()
results=[]
for encoding, payload in [('rgb8',np.arange(240*320*3,dtype=np.uint8).reshape(240,320,3)),
                          ('32FC1',np.linspace(0,12,240*320,dtype=np.float32).reshape(240,320))]:
    message=builders.image_msg(Time(sec=7,nanosec=123), 'camera_optical', payload, encoding)
    wire=serialize_message(message)
    decoded=deserialize_message(wire,Image)
    assert bytes(decoded.data)==payload.tobytes()
    timings=[]
    for _ in range(20):
        start=time.perf_counter()
        builders.image_msg(Time(sec=7,nanosec=123),'camera_optical',payload,encoding)
        timings.append(time.perf_counter()-start)
    results.append(dict(encoding=encoding,shape=list(payload.shape),iterations=len(timings),
        samples_wall_s=timings,median_wall_s=float(np.median(timings)),
        wire_sha256=hashlib.sha256(wire).hexdigest(),data_sha256=hashlib.sha256(bytes(decoded.data)).hexdigest()))
source=Path(builders.__file__)
report=dict(scope='Actual installed ROS image construction and lossless serialization microbenchmark; no simulator RTF or mission qualification.',
    source_path=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),records=results)
args.output.write_text(json.dumps(report,indent=2)+'\n')
print([(r['encoding'],r['median_wall_s']) for r in results])
