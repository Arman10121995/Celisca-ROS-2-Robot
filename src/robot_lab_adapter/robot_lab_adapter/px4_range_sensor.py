"""An actual Gazebo downward lidar, delivered through MAVLink DISTANCE_SENSOR.

No simulator body position enters this channel. Range includes authored noise
and real selected-world collision geometry. The fixed mount points body down.
"""
from collections import deque
import hashlib
import json
import math
import subprocess
import threading
import time
import xml.etree.ElementTree as ET


def add_sensor(model, topic):
    # X500 merges its base include. SDF relative_to resolves the existing
    # base_link frame; this explicit source kit adds no hull/actuator changes.
    link=ET.SubElement(model,'link',name='lab_range_link')
    ET.SubElement(link,'pose',relative_to='base_link').text='0 0 -.05 0 1.5707963267948966 0'
    inertia=ET.SubElement(link,'inertial');ET.SubElement(inertia,'mass').text='.001'
    tensor=ET.SubElement(inertia,'inertia')
    for axis in ('ixx','iyy','izz'):ET.SubElement(tensor,axis).text='.000001'
    joint=ET.SubElement(model,'joint',name='lab_range_mount',type='fixed')
    ET.SubElement(joint,'parent').text='base_link';ET.SubElement(joint,'child').text='lab_range_link'
    sensor=ET.SubElement(link,'sensor',name='lab_range',type='gpu_lidar')
    ET.SubElement(sensor,'topic').text=topic;ET.SubElement(sensor,'update_rate').text='30'
    ET.SubElement(sensor,'always_on').text='true'
    lidar=ET.SubElement(sensor,'lidar');scan=ET.SubElement(lidar,'scan');horizontal=ET.SubElement(scan,'horizontal')
    for key,value in dict(samples='1',resolution='1',min_angle='0',max_angle='0').items():ET.SubElement(horizontal,key).text=value
    ranges=ET.SubElement(lidar,'range')
    for key,value in dict(min='.02',max='15',resolution='.01').items():ET.SubElement(ranges,key).text=value
    noise=ET.SubElement(lidar,'noise',type='gaussian');ET.SubElement(noise,'mean').text='0';ET.SubElement(noise,'stddev').text='.01'


def verified_sensor_links(model_file,topic):
    """Approve only the exact authored link and its fixed source mount."""
    expected=ET.Element('model');add_sensor(expected,topic)
    actual=ET.parse(model_file).getroot().find('model')
    for tag,name in (('link','lab_range_link'),('joint','lab_range_mount')):
        item=actual.find(tag+"[@name='"+name+"']")
        template=expected.find(tag+"[@name='"+name+"']")
        if item is None or ET.tostring(item)!=ET.tostring(template):
            raise ValueError('Downward sensor geometry/mount differs from the explicit kit')
    return {'lab_range_link':hashlib.sha256(ET.tostring(actual.find("link[@name='lab_range_link']"))).hexdigest()}


class RangeSensor:
    def __init__(self, controller, topic, environment):
        self.controller=controller;self.queue=deque(maxlen=1)
        self.process=subprocess.Popen(['gz','topic','-e','-t',topic,'--json-output'],env=environment,
            stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        self.thread=threading.Thread(target=self.read,daemon=True);self.thread.start()
        self.timer=controller.create_timer(.03,self.publish)

    def read(self):
        for line in self.process.stdout:
            try:self.queue.append((time.monotonic(),json.loads(line)))
            except json.JSONDecodeError:pass

    def publish(self):
        if not self.queue:return
        received,raw=self.queue.pop()
        if time.monotonic()-received>.2:return
        try:
            if len(raw['ranges'])!=1:return
            value=float(raw['ranges'][0]);minimum=float(raw.get('rangeMin',raw.get('range_min')));maximum=float(raw.get('rangeMax',raw.get('range_max')))
            if not all(map(math.isfinite,(value,minimum,maximum))) or not minimum<=value<=maximum:return
            flight=self.controller.flight
            if flight.state is None or flight.local_position is None:return
            # MAVLink uses cm, covariance cm^2. Orientation 25 is body down;
            # PX4 consumes this actual sensor via its standard receiver/EKF.
            flight.master.mav.distance_sensor_send(int(flight.local_position.time_boot_ms)&0xffffffff,
                round(minimum*100),round(maximum*100),round(value*100),0,1,25,1)
            self.controller.range_observation=dict(range_m=value,received_monotonic=received,stamp=raw.get('header',{}).get('stamp'),
                noise_std_m=.01,source='Actual Gazebo collision lidar; MAVLink downward range sensor')
        except (ValueError,KeyError,TypeError,OverflowError):return

    def close(self):
        self.controller.destroy_timer(self.timer)
        self.process.terminate()
        try:self.process.wait(timeout=2.)
        except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
