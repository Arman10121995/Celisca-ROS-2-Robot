"""Native Stretch sensors and dynamic full-body navigation footprint.

RGB-D uses the authored camera and its fovy; lidar originates at the authored
laser body with an explicit lab mount offset. Odometry/IMU are simulator-state
measurements, the same truth-bootstrap contract as alternate wheel backends.
They do not establish independent state-estimator accuracy.
"""
import math

import numpy as np
from geometry_msgs.msg import Point32, Polygon, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import Bool

from robot_lab_utils.camera_msgs import camera_info_msg, image_msg
from robot_lab_utils.native_mjcf_assets import body_frame


class NativeMobileSensors:
    def __init__(self, node, task):
        self.node, self.model, self.data, self.mj = node, node.model, node.data, node.mujoco
        self.task = task
        self.base = node.mobile.base
        self.laser = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_BODY, 'laser')
        self.camera = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_CAMERA,
            'd435i_camera_rgb' if node.mobile.variant == 'stretch_3' else 'camera_rgb')
        if self.laser < 0 or self.camera < 0:
            raise ValueError('Native Stretch sensor mounting/camera missing')
        self.frame = body_frame(self.base)
        self.scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = node.create_publisher(Odometry, '/odom/ground_truth', 10)
        self.imu_pub = node.create_publisher(Imu, '/imu/out', 10)
        self.rgb_pub = node.create_publisher(Image, '/oakd/rgb/image_raw', qos_profile_sensor_data)
        self.depth_pub = node.create_publisher(Image, '/oakd/depth/image_raw', qos_profile_sensor_data)
        self.info_pub = node.create_publisher(CameraInfo, '/oakd/rgb/camera_info', qos_profile_sensor_data)
        self.ready_pub = node.create_publisher(Bool, '/robot_lab/native_sensors_ready',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.footprints = [node.create_publisher(Polygon, topic, 10) for topic in
                          ('/local_costmap/footprint', '/global_costmap/footprint')]
        self.last_scan, self.last_camera, self.last_footprint = -math.inf, -math.inf, -math.inf
        self.scan_count, self.camera_count = 0, 0
        self.width, self.height = 320, 240
        self.renderer = None
        self.camera_error = ''
        angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        self.directions = np.stack((np.cos(angles), np.sin(angles), np.zeros(360)), axis=1)

    def static_mounts(self, stamp):
        laser = TransformStamped()
        laser.header.stamp, laser.header.frame_id = stamp, body_frame(self.laser)
        laser.child_frame_id = 'native_laser_frame'
        laser.transform.translation.z = .035
        laser.transform.rotation.w = 1.
        camera = TransformStamped()
        camera.header.stamp, camera.header.frame_id = stamp, body_frame(int(self.model.cam_bodyid[self.camera]))
        camera.child_frame_id = 'native_camera_optical_frame'
        camera.transform.translation.x, camera.transform.translation.y, camera.transform.translation.z = map(float, self.model.cam_pos[self.camera])
        quaternion = np.zeros(4)
        self.mj.mju_mulQuat(quaternion, self.model.cam_quat[self.camera], np.array([0., 1., 0., 0.]))
        camera.transform.rotation.w, camera.transform.rotation.x, camera.transform.rotation.y, camera.transform.rotation.z = map(float, quaternion)
        self.node.tf.sendTransform([laser, camera])

    def publish(self, stamp):
        self.static_mounts(stamp)
        velocity = np.zeros(6)
        self.mj.mj_objectVelocity(self.model, self.data, self.mj.mjtObj.mjOBJ_BODY, self.base, velocity, 1)
        odometry = Odometry()
        odometry.header.stamp, odometry.header.frame_id, odometry.child_frame_id = stamp, 'odom', self.frame
        p, q = odometry.pose.pose.position, odometry.pose.pose.orientation
        p.x, p.y, p.z = map(float, self.data.xpos[self.base])
        q.w, q.x, q.y, q.z = map(float, self.data.xquat[self.base])
        odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z = map(float, velocity[:3])
        odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z = map(float, velocity[3:])
        for index in (0, 7, 14, 21, 28, 35):
            odometry.pose.covariance[index] = 1e-4
            odometry.twist.covariance[index] = 1e-3
        self.odom_pub.publish(odometry)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, self.frame
        imu.orientation, imu.angular_velocity = odometry.pose.pose.orientation, odometry.twist.twist.angular
        imu.orientation_covariance = [1e-4, 0., 0., 0., 1e-4, 0., 0., 0., 1e-4]
        imu.angular_velocity_covariance = [1e-3, 0., 0., 0., 1e-3, 0., 0., 0., 1e-3]
        imu.linear_acceleration_covariance[0] = -1.
        self.imu_pub.publish(imu)
        if self.data.time-self.last_scan >= .2-1e-9:
            self.last_scan = self.data.time
            self.scan(stamp)
        if self.data.time-self.last_camera >= .2-1e-9:
            self.last_camera = self.data.time
            try:
                self.rgbd(stamp)
                self.camera_error = ''
            except Exception as exc:
                # Keep lidar/Drive alive while a camera installation fault is
                # visible and the 3D workflow remains unready.
                self.camera_error = str(exc)
                self.node.get_logger().error('Native camera unavailable: '+str(exc), throttle_duration_sec=5.)
        if self.data.time-self.last_footprint >= .5-1e-9:
            self.last_footprint = self.data.time
            self.footprint()
        self.ready_pub.publish(Bool(data=self.scan_count >= 2 and
            (self.task != '3d_slam' or self.camera_count >= 1 and not self.camera_error)))

    def scan(self, stamp):
        rotation = self.data.xmat[self.laser].reshape(3, 3)
        origin = self.data.xpos[self.laser]+rotation @ [0., 0., .035]
        directions = np.ascontiguousarray(self.directions @ rotation.T).ravel()
        hits, distances = np.empty(360, dtype=np.int32), np.empty(360)
        self.mj.mj_multiRay(self.model, self.data, origin, directions,
            np.array([1, 1, 0, 1, 0, 0], dtype=np.uint8), True, self.laser,
            hits, distances, None, 360, 15.)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'native_laser_frame'
        scan.angle_min, scan.angle_increment = -math.pi, 2*math.pi/360
        scan.angle_max = scan.angle_min+359*scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = .08, 15., .2
        scan.ranges = [float(v) if .08 <= v <= 15 else math.inf for v in distances]
        self.scan_pub.publish(scan)
        self.scan_count += 1

    def rgbd(self, stamp):
        if self.renderer is None:
            self.renderer = self.mj.Renderer(self.model, height=self.height, width=self.width)
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera)
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().astype(np.float32)
        self.renderer.disable_depth_rendering()
        depth[~np.isfinite(depth) | (depth <= 0.) | (depth > 30.)] = np.nan
        fov = 2*math.atan(math.tan(math.radians(float(self.model.cam_fovy[self.camera]))/2)*self.width/self.height)
        frame = 'native_camera_optical_frame'
        self.rgb_pub.publish(image_msg(stamp, frame, rgb, 'rgb8'))
        self.depth_pub.publish(image_msg(stamp, frame, depth, '32FC1'))
        self.info_pub.publish(camera_info_msg(stamp, frame, self.width, self.height, fov))
        self.camera_count += 1

    def footprint(self):
        from shapely.geometry import MultiPoint
        rotation = self.data.xmat[self.base].reshape(3, 3)
        points = []
        corners = np.array([[x, y, z] for x in (-1., 1.) for y in (-1., 1.) for z in (-1., 1.)])
        for geom in range(self.model.ngeom):
            if (not 1 <= int(self.model.geom_bodyid[geom]) < self.node.robot_bodies
                    or not (self.model.geom_contype[geom] or self.model.geom_conaffinity[geom])):
                continue
            center, half = self.model.geom_aabb[geom, :3], self.model.geom_aabb[geom, 3:]
            world = (center+corners*half) @ self.data.geom_xmat[geom].reshape(3, 3).T+self.data.geom_xpos[geom]
            points.extend(((world-self.data.xpos[self.base]) @ rotation)[:, :2].tolist())
        hull = MultiPoint(points).convex_hull.buffer(.03, resolution=2)
        polygon = Polygon(points=[Point32(x=float(x), y=float(y), z=0.) for x, y in list(hull.exterior.coords)[:-1]])
        for publisher in self.footprints:
            publisher.publish(polygon)

    def close(self):
        if self.renderer:
            self.renderer.close()
