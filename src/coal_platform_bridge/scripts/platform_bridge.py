#!/usr/bin/env python3
"""Add run/scene context to existing zero-noise truth and planar scans.

This baseline supports only coincident map/world. p3d velocities already use
world axes; copying them is correct here and must be reviewed for other sources.
"""
import math
import uuid
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster
from coal_platform_interfaces.msg import GroundTruth, LidarFrameRaw, RunContext


class PlatformBridge(Node):
    def __init__(self):
        super().__init__('coal_platform_bridge')
        defaults = {'scene_id': 'coal_face_base', 'scene_version': '0.2.0',
                    'robot_id': 'shearer_01', 'sensor_id': 'lidar_01',
                    'experiment_id': 'coal_face_base_development'}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        if not all(isinstance(v, str) and v for v in self.cfg.values()):
            raise ValueError('Scene/run metadata parameters must be nonempty strings')
        self.last_clock = None
        self.reported = set()
        self.new_run()
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.truth_pub = self.create_publisher(GroundTruth, '/platform/v1/ground_truth', reliable)
        self.scan_pub = self.create_publisher(LidarFrameRaw, '/platform/v1/scan_raw', sensor)
        self.run_pub = self.create_publisher(RunContext, '/platform/v1/run_context',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'world'
        tf.transform.rotation.w = 1.0
        self.static.sendTransform(tf)
        self.create_subscription(Odometry, '/ground_truth', self.on_truth, reliable)
        self.create_subscription(LaserScan, '/scan_raw', self.on_scan, sensor)
        self.create_subscription(Clock, '/clock', self.on_clock, sensor)
        self.get_logger().info('Adapter active: ' + self.cfg['scene_id'] + ' identity map->world; run_id=' + self.run_id)

    def new_run(self):
        self.run_id = str(uuid.uuid4())
        self.sequence = {'truth': 0, 'scan': 0}
        self.last_stamp = {'truth': -1, 'scan': -1}

    def on_clock(self, msg):
        stamp = msg.clock.sec * 1000000000 + msg.clock.nanosec
        if self.last_clock is not None and stamp < self.last_clock:
            self.new_run()
            self.get_logger().info('Simulation time reset; new run_id=' + self.run_id)
        self.last_clock = stamp

    def reject(self, reason):
        if reason not in self.reported:
            self.get_logger().error(reason)
            self.reported.add(reason)

    def context(self, header, stream, frame):
        ns = header.stamp.sec * 1000000000 + header.stamp.nanosec
        if ns <= self.last_stamp[stream]:
            self.reject('Duplicate/out-of-order ' + stream + ' stamp dropped')
            return None
        self.last_stamp[stream] = ns
        context = RunContext()
        context.header.stamp = deepcopy(header.stamp)
        context.header.frame_id = header.frame_id
        context.header.frame_id = frame
        context.schema_version = '0.2.0'
        context.header.experiment_id = self.cfg['experiment_id']
        context.header.run_id = self.run_id
        context.header.scene_id = self.cfg['scene_id']
        context.header.scene_version = self.cfg['scene_version']
        context.robot_id = self.cfg['robot_id']
        context.sequence = self.sequence[stream]
        self.sequence[stream] += 1
        return context

    def on_truth(self, msg):
        if msg.header.frame_id != 'world' or msg.child_frame_id != 'base_link':
            self.reject('Expected world/base_link Odometry; identity adapter refuses other frames')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  linear.x, linear.y, linear.z, angular.x, angular.y, angular.z]
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in [q.x,q.y,q.z,q.w])-1) > 1e-5:
            self.reject('Invalid ground-truth pose/twist')
            return
        context = self.context(msg.header, 'truth', 'map')
        if context is None:
            return
        out = GroundTruth()
        out.context = context
        out.observation_id = '/'.join([self.run_id, self.cfg['robot_id'],
                                       'ground_truth', str(context.sequence)])
        out.child_frame_id = msg.child_frame_id
        out.pose = deepcopy(msg.pose.pose)
        out.twist = deepcopy(msg.twist.twist)
        out.roll_rad = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        out.pitch_rad = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
        out.yaw_rad = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.truth_pub.publish(out)
        self.run_pub.publish(context)

    def on_scan(self, msg):
        n = len(msg.ranges)
        if msg.header.frame_id != 'laser_link' or n != 720 or len(msg.intensities) not in (0, n):
            self.reject('Expected laser_link, 720 beams and empty-or-720 intensities')
            return
        context = self.context(msg.header, 'scan', msg.header.frame_id)
        if context is None:
            return
        out = LidarFrameRaw()
        out.context = context
        out.observation_id = '/'.join([self.run_id, self.cfg['robot_id'],
                                       self.cfg['sensor_id'], 'frame', str(context.sequence)])
        out.sensor_id = self.cfg['sensor_id']
        for new, old in [('angle_min_rad','angle_min'), ('angle_max_rad','angle_max'),
                         ('angle_increment_rad','angle_increment'), ('time_increment_s','time_increment'),
                         ('scan_time_s','scan_time'), ('range_min_m','range_min'), ('range_max_m','range_max')]:
            setattr(out, new, getattr(msg, old))
        out.beam_count = n
        out.truth_ranges_m = list(msg.ranges)
        out.raw_ranges_m = list(msg.ranges)
        out.intensities = list(msg.intensities)
        valid, status = [], []
        for distance in msg.ranges:
            hit = math.isfinite(distance) and msg.range_min <= distance <= msg.range_max
            valid.append(hit)
            status.append(0 if hit else 1 if distance == math.inf else
                          3 if math.isfinite(distance) and distance < msg.range_min else 2)
        out.raw_valid = valid
        out.raw_status = status
        self.scan_pub.publish(out)


def main():
    rclpy.init()
    node = PlatformBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
