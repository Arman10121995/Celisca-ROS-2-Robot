#!/usr/bin/env python3
"""Aligned RGB/depth/calibration/TF to source-bound unmarked box targets."""
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from scipy.spatial.transform import Rotation

from robot_lab_utils.rgbd_boxes import bindings,estimate_box


def stamp(message):return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def pixels(message,depth=False):
    if not 16<=message.width<=1280 or not 16<=message.height<=960:
        raise ValueError('Image dimensions exceed bounded RGB-D input')
    if depth:
        formats={'32FC1':('f4',1.),'16UC1':('u2',.001)}
        if message.encoding not in formats:raise ValueError('Depth must be 32FC1 metres or 16UC1 millimetres')
        kind,scale=formats[message.encoding];dtype=np.dtype(('>' if message.is_bigendian else '<')+kind)
        count=1
    else:
        if message.encoding not in ('rgb8','bgr8'):raise ValueError('RGB-D requires RGB8 or BGR8 color')
        dtype=np.dtype('u1');count=3;scale=1.
    row=message.width*count*dtype.itemsize
    if message.step<row or message.step>row+4096 or len(message.data)!=message.step*message.height:
        raise ValueError('Malformed image stride/data')
    raw=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)[:,:row].copy()
    result=raw.view(dtype).reshape(message.height,message.width) if depth else raw.reshape(message.height,message.width,3)
    if depth:return result.astype(float)*scale
    return result if message.encoding=='rgb8' else result[:,:,::-1].copy()


class RGBDObjects(Node):
    def __init__(self,**kwargs):
        super().__init__('rgbd_object_pose',**kwargs)
        for name,value in dict(bindings='',image_topic='/oakd/rgb/image_raw',depth_topic='/oakd/depth/image_raw',
                camera_info_topic='/oakd/rgb/camera_info',context_topic='/mobile_task/state',
                output_topic='/robot_lab/object_estimates').items():self.declare_parameter(name,value)
        import yaml
        path=Path(self.get_parameter('bindings').value)
        self.boxes=bindings(yaml.safe_load(path.read_text()));self.bindings_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        self.task_context={};self.names=set();self.context_at=-math.inf;self.frames={};self.last_frame=-math.inf
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.output=self.create_publisher(String,self.get_parameter('output_topic').value,10)
        self.create_subscription(String,self.get_parameter('context_topic').value,self.receive_context,10)
        for name,topic,kind in [('rgb','image_topic',Image),('depth','depth_topic',Image),('info','camera_info_topic',CameraInfo)]:
            self.create_subscription(kind,self.get_parameter(topic).value,
                lambda message,name=name:self.receive(name,message),qos_profile_sensor_data)
        self.create_timer(.05,self.process_pending,clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    def receive_context(self,message):
        try:
            if len(message.data)>2*1024*1024:return
            data=json.loads(message.data)
            if not isinstance(data,dict):return
            identity={key:data[key] for key in ('plant_id','source_sha256','scene_id')}
            if any(not isinstance(value,str) or not 1<=len(value)<=256 for value in identity.values()):return
            names=data.get('object_candidates')
            if not isinstance(names,list) or len(names)>256 or any(not isinstance(name,str) for name in names):return
            if identity!=self.task_context:self.frames={}
            self.task_context=identity;self.names=set(names);self.context_at=time.monotonic()
            self.process_pending()
        except (ValueError,TypeError,KeyError):pass

    def receive(self,name,message):
        self.frames[name]=(message,time.monotonic());self.process_pending()

    def process_pending(self):
        wall=time.monotonic()
        if (wall-self.last_frame<.1 or not self.task_context or wall-self.context_at>1.
                or not all(key in self.frames for key in ('rgb','depth','info'))):return
        rgb,depth,info=[self.frames[name][0] for name in ('rgb','depth','info')]
        if max(stamp(rgb),stamp(depth),stamp(info))-min(stamp(rgb),stamp(depth),stamp(info))>.02:return
        if any(wall-value[1]>1. for value in self.frames.values()):return
        paired=self.frames;prior_last=self.last_frame
        self.frames={};self.last_frame=wall
        output=dict(schema='robot_lab.object_estimates.v1',**self.task_context,frame='native_world',
            pose_source='rgbd_box_top',stamp_s=stamp(rgb),targets={},bindings_sha256=self.bindings_sha256,
            camera_frame=rgb.header.frame_id,scope='Aligned RGB-D known-size upright color boxes; no simulator object pose input')
        try:
            if not -.05<=self.get_clock().now().nanoseconds*1e-9-stamp(rgb)<=.5:
                raise ValueError('Stale/future RGB-D sample')
            if (not rgb.header.frame_id or rgb.header.frame_id!=depth.header.frame_id or rgb.header.frame_id!=info.header.frame_id
                    or (rgb.width,rgb.height)!=(depth.width,depth.height) or (rgb.width,rgb.height)!=(info.width,info.height)
                    or info.distortion_model not in ('','plumb_bob','rational_polynomial')
                    or info.roi.x_offset or info.roi.y_offset or info.binning_x>1 or info.binning_y>1
                    or info.roi.width not in (0,info.width) or info.roi.height not in (0,info.height)):
                raise ValueError('RGB/depth must be aligned in the same calibrated optical frame and image size')
            color=pixels(rgb);distance=pixels(depth,True)
            transform=self.tf.lookup_transform('native_world',rgb.header.frame_id,rclpy.time.Time.from_msg(rgb.header.stamp)).transform
            p,q=transform.translation,transform.rotation
            world=np.eye(4);world[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();world[:3,3]=[p.x,p.y,p.z]
            digest=hashlib.sha256(bytes(rgb.data)+bytes(depth.data)+str(stamp(rgb)).encode()+self.bindings_sha256.encode()).hexdigest()
            for name,binding in self.boxes.items():
                if name not in self.names:continue
                try:
                    result=estimate_box(color,distance,np.asarray(info.k).reshape(3,3),info.d,world,binding)
                    result['observation_id']=hashlib.sha256((digest+name).encode()).hexdigest()
                    output['targets'][name]=result
                except (ValueError,cv2.error,np.linalg.LinAlgError) as exc:output.setdefault('rejected',{})[name]=str(exc)
        except TransformException as exc:
            # Robot-state TF can arrive just after the image. Retry one
            # bounded pair without blocking the executor or accepting latest TF.
            if wall-min(value[1] for value in paired.values())<.2:
                self.frames=paired;self.last_frame=prior_last;return
            output['reason']=type(exc).__name__+': image-time TF unavailable'
        except Exception as exc:output['reason']=type(exc).__name__+': '+str(exc)
        self.output.publish(String(data=json.dumps(output,allow_nan=False)))


def main(args=None):
    rclpy.init(args=args);node=RGBDObjects()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
