#!/usr/bin/env python3
"""Ideal kinematic IMU at base_link origin; not an independent sensor model."""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

def rotation(q):
    a = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(a)
    if not np.isfinite(norm) or norm < 1e-10:
        raise ValueError('Invalid truth quaternion')
    x, y, z, w = a/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

class IdealImu(Node):
    def __init__(self):
        super().__init__('shearer_ideal_imu')
        self.declare_parameter('gravity_ms2', 9.81)
        self.gravity = float(self.get_parameter('gravity_ms2').value)
        if not math.isfinite(self.gravity) or self.gravity <= 0:
            raise ValueError('gravity_ms2 must be positive')
        self.history = deque(maxlen=5)
        self.pub = self.create_publisher(Imu, '/imu_raw', 100)
        self.static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'imu_link'
        tf.transform.rotation.w = 1.0
        self.static.sendTransform(tf)
        self.sub = self.create_subscription(Odometry, '/ground_truth', self.on_truth,
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info('Ideal IMU: 5-sample velocity fit; center timestamp (~40 ms latency at 50 Hz); orientation unavailable')

    def on_truth(self, msg):
        if msg.header.frame_id != 'world' or msg.child_frame_id != 'base_link':
            self.get_logger().error('Expected world/base_link ground truth frames')
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if self.history:
            dt = t-self.history[-1][0]
            if dt == 0:
                return
            if dt < 0 or dt > .1:
                self.history.clear()
        try:
            R = rotation(msg.pose.pose.orientation)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self.history.clear()
            return
        v = msg.twist.twist.linear
        # nav_msgs/Odometry twist is expressed in child_frame_id.
        world_velocity = R @ np.array([v.x, v.y, v.z])
        if not np.all(np.isfinite(world_velocity)):
            self.history.clear()
            return
        self.history.append((t, world_velocity, R, msg))
        if len(self.history) < 5:
            return
        middle = self.history[2]
        times = np.array([h[0]-middle[0] for h in self.history])
        times -= times.mean()
        velocity = np.array([h[1] for h in self.history])
        acceleration_world = (times[:, None]*velocity).sum(axis=0)/(times @ times)
        specific_force = middle[2].T @ (acceleration_world + [0, 0, self.gravity])
        out = Imu()
        out.header.stamp = middle[3].header.stamp
        out.header.frame_id = 'imu_link'
        # No independent orientation estimate is supplied by a raw IMU.
        out.orientation.w = 1.0
        out.orientation_covariance[0] = -1.0
        out.angular_velocity = middle[3].twist.twist.angular
        out.linear_acceleration.x = float(specific_force[0])
        out.linear_acceleration.y = float(specific_force[1])
        out.linear_acceleration.z = float(specific_force[2])
        # Zero covariance arrays mean unknown, not zero measurement error.
        self.pub.publish(out)

def main():
    rclpy.init()
    node = IdealImu()
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
