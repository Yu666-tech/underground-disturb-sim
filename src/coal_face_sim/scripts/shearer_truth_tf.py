#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class TruthTF(Node):
    def __init__(self):
        super().__init__('shearer_truth_tf')
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        fixed = TransformStamped()
        fixed.header.stamp = self.get_clock().now().to_msg()
        fixed.header.frame_id = 'base_link'
        fixed.child_frame_id = 'laser_link'
        fixed.transform.translation.z = 0.55
        fixed.transform.rotation.w = 1.0
        self.static.sendTransform(fixed)
        self.subscription = self.create_subscription(
            Odometry, '/ground_truth', self.on_truth, qos_profile_sensor_data)

    def on_truth(self, msg):
        if msg.header.frame_id != 'world' or msg.child_frame_id != 'base_link':
            self.get_logger().error('Expected ground truth frames world -> base_link')
            return
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.dynamic.sendTransform(tf)

def main():
    rclpy.init()
    node = TruthTF()
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
