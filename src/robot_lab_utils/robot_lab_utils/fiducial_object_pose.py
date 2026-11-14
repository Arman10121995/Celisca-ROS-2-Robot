#!/usr/bin/env python3
"""ArUco image/camera/TF measurements for source-bound manipulation targets."""
import hashlib
import json
import math
from pathlib import Path
import time
from collections import Counter

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from scipy.spatial.transform import Rotation

from robot_lab_utils.fiducial_objects import bindings,marker_pose,object_pose


class FiducialObjects(Node):
    def __init__(self,**kwargs):
        super().__init__('fiducial_object_pose',**kwargs)
        for name,value in dict(bindings='',image_topic='/oakd/rgb/image_raw',camera_info_topic='/oakd/rgb/camera_info',
                               context_topic='/mobile_task/state',output_topic='/robot_lab/object_estimates').items():
            self.declare_parameter(name,value)
        import yaml
        path=Path(self.get_parameter('bindings').value)
        self.markers=bindings(yaml.safe_load(path.read_text()))
        self.bindings_sha256=hashlib.sha256(path.read_bytes()).hexdigest()
        if not hasattr(cv2,'aruco'):raise RuntimeError('Install OpenCV with the aruco module')
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters=cv2.aruco.DetectorParameters() if hasattr(cv2.aruco,'DetectorParameters') else cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector=cv2.aruco.ArucoDetector(dictionary,parameters) if hasattr(cv2.aruco,'ArucoDetector') else None
        self.dictionary,self.parameters=dictionary,parameters
        self.task_context={};self.task_context_at=-math.inf;self.info=None;self.info_at=-math.inf;self.last_frame=-math.inf
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.output=self.create_publisher(String,self.get_parameter('output_topic').value,10)
        self.create_subscription(String,self.get_parameter('context_topic').value,self.receive_context,10)
        self.create_subscription(CameraInfo,self.get_parameter('camera_info_topic').value,self.receive_info,qos_profile_sensor_data)
        self.create_subscription(Image,self.get_parameter('image_topic').value,self.receive_image,qos_profile_sensor_data)

    def receive_context(self,message):
        try:
            if len(message.data)>2*1024*1024:return
            data=json.loads(message.data)
            if not isinstance(data,dict):return
            if not all(isinstance(data.get(k),str) and data[k] for k in ('plant_id','source_sha256','scene_id')):return
            names=data.get('object_candidates')
            if not isinstance(names,list) or len(names)>256 or any(not isinstance(name,str) for name in names):return
            self.task_context={key:data[key] for key in ('plant_id','source_sha256','scene_id')}
            self.names=set(names);self.task_context_at=time.monotonic()
        except (ValueError,TypeError):pass

    def receive_info(self,message):self.info=message;self.info_at=time.monotonic()

    def receive_image(self,message):
        wall=time.monotonic()
        if wall-self.last_frame<.1:return
        self.last_frame=wall
        if not self.task_context or wall-self.task_context_at>1.:return
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        output=dict(schema='robot_lab.object_estimates.v1',**self.task_context,frame='native_world',pose_source='aruco_pnp',
                    stamp_s=stamp,targets={},bindings_sha256=self.bindings_sha256,
                    camera_frame=message.header.frame_id,scope='Camera PnP targets; source geometry/TF remain declared inputs')
        try:
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if not -.05<=age<=.5:raise ValueError('Stale/future image')
            info=self.info
            if info is None or wall-self.info_at>1.:raise ValueError('Fresh CameraInfo required')
            info_stamp=info.header.stamp.sec+info.header.stamp.nanosec*1e-9
            if (info.header.frame_id!=message.header.frame_id or abs(info_stamp-stamp)>.05
                    or info.width!=message.width or info.height!=message.height
                    or info.distortion_model not in ('','plumb_bob','rational_polynomial')
                    or info.roi.x_offset or info.roi.y_offset or info.binning_x>1 or info.binning_y>1):
                raise ValueError('Image and camera calibration/frame/size do not match')
            count={'rgb8':3,'bgr8':3,'mono8':1}.get(message.encoding)
            if not count or not 16<=message.width<=4096 or not 16<=message.height<=4096:
                raise ValueError('Supported RGB/BGR/mono 8-bit image required')
            if message.step<message.width*count or len(message.data)!=message.step*message.height:
                raise ValueError('Malformed camera stride/data')
            pixels=np.frombuffer(message.data,dtype=np.uint8).reshape(message.height,message.step)
            pixels=pixels[:,:message.width*count].reshape(message.height,message.width,count)
            gray=pixels[:,:,0] if count==1 else cv2.cvtColor(pixels,cv2.COLOR_RGB2GRAY if message.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
            corners,ids,_=(self.detector.detectMarkers(gray) if self.detector else
                          cv2.aruco.detectMarkers(gray,self.dictionary,parameters=self.parameters))
            transform=self.tf.lookup_transform('native_world',message.header.frame_id,rclpy.time.Time.from_msg(message.header.stamp)).transform
            p,q=transform.translation,transform.rotation
            world=np.eye(4);world[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();world[:3,3]=[p.x,p.y,p.z]
            detected=[] if ids is None else list(zip(ids.ravel().tolist(),corners))
            if len(detected)>64:raise ValueError('Too many detected markers for a bounded target frame')
            duplicates={number for number,count in Counter(i for i,_ in detected).items() if count>1}
            for number,c in detected[:64]:
                if number not in self.markers or number in duplicates:continue
                binding=self.markers[number]
                if binding['object'] not in self.names:continue
                try:
                    if np.any(c[...,0]<2) or np.any(c[...,1]<2) or np.any(c[...,0]>message.width-3) or np.any(c[...,1]>message.height-3):
                        raise ValueError('Marker clipped by image boundary')
                    estimate=marker_pose(c,binding['marker_size_m'],info.k,info.d)
                    target=object_pose(estimate,binding,world)
                    target['observation_id']=hashlib.sha256((str(stamp)+str(number)+str(c.tolist())+self.bindings_sha256).encode()).hexdigest()
                    target['marker_id']=number;output['targets'][binding['object']]=target
                except (ValueError,cv2.error,np.linalg.LinAlgError) as exc:output.setdefault('rejected',{})[str(number)]=str(exc)
        except Exception as exc:output['reason']=type(exc).__name__+': '+str(exc)
        self.output.publish(String(data=json.dumps(output,allow_nan=False)))


def main(args=None):
    rclpy.init(args=args);node=FiducialObjects()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
