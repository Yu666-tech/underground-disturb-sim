#!/usr/bin/env python3
"""Record five UWB CSV tables; no classification or error algorithm in this node."""
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict

from coal_platform_interfaces.msg import UwbDecision, UwbQueryTrace, UwbRangeObserved, UwbRangeRaw
from uwb_csv import UwbCsvWriter


class UwbCsvRecorder(Node):
    def __init__(self):
        super().__init__('uwb_csv_recorder')
        self.declare_parameter('output_dir', '')
        output_dir = self.get_parameter('output_dir').value
        if not output_dir:
            raise ValueError('Set output_dir to a NEW experiment directory')
        templates = Path(get_package_share_directory('coal_platform_bridge')) / 'csv_templates'
        self.writer = UwbCsvWriter(output_dir, templates)
        sensor = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        reliable = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions_ = []
        for kind, cls, topic, qos in (
                ('raw', UwbRangeRaw, '/platform/v1/uwb_ranges_raw', sensor),
                ('decision', UwbDecision, '/disturbance_decision/uwb', reliable),
                ('observed', UwbRangeObserved, '/uwb_ranges', sensor),
                ('query', UwbQueryTrace, '/disturbance_query/uwb', reliable)):
            self.subscriptions_.append(self.create_subscription(
                cls, topic, lambda msg, k=kind: self.writer.ingest(k, message_to_ordereddict(msg)), qos))
        self.create_timer(1.0, self.writer.refresh_transitions)
        self.get_logger().info(f'Writing five UWB CSV tables to {output_dir}')


def main():
    rclpy.init()
    node = None
    try:
        node = UwbCsvRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            report = node.writer.close()
            node.get_logger().info(f'CSV recording summary: {report}')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
