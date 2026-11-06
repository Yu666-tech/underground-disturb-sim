#!/usr/bin/env python3
"""Synthetic facts only: never predicts B/C decisions or injected observations."""
import math
import json
import time
import uuid
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Vector3, TransformStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from coal_platform_interfaces.msg import (RunContext, GroundTruth, RobotState,
    UwbRangeRaw, LidarFrameRaw, Intersection, ReflectorCandidate)
from coal_platform_interfaces.srv import QueryRay

CASES = {'A00': 'EMPTY', 'A01': 'INTERSECTION', 'A02': 'REFLECTOR',
         'A03': 'BOTH', 'A04': 'QUERY_ERROR', 'A05': 'INPUT_INVALID',
         'A06': 'SENSOR_UNAVAILABLE', 'A07': 'OUT_OF_RANGE', 'A08': 'EMPTY'}


class FixtureInputs(Node):
    def __init__(self):
        super().__init__('coal_mock_fixtures')
        self.declare_parameter('phase_duration_s', 4.0)
        self.phase_s = float(self.get_parameter('phase_duration_s').value)
        if not math.isfinite(self.phase_s) or self.phase_s < .2:
            raise ValueError('phase_duration_s must be >= 0.2')
        self.fixture = json.loads((Path(get_package_share_directory('coal_platform_devtools'))/'config/fixture_cases.json').read_text())
        global CASES
        CASES = {k:v['case'] for k,v in self.fixture['uwb'].items()}
        self.run_id = str(uuid.uuid4())
        self.tick = 0
        self.batch = 0
        self.latest_ns = 0
        self.observations = {}
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.truth_pub = self.create_publisher(GroundTruth, '/platform/v1/ground_truth', reliable)
        self.state_pub = self.create_publisher(RobotState, '/platform/v1/robot_state', reliable)
        self.scan_pub = self.create_publisher(LidarFrameRaw, '/platform/v1/scan_raw', sensor)
        self.uwb_pub = self.create_publisher(UwbRangeRaw, '/platform/v1/uwb_ranges_raw',
                                          QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.context_pub = self.create_publisher(RunContext, '/platform/v1/run_context', latched)
        self.info_pub = self.create_publisher(String, '/platform/v1/fixture_info', latched)
        self.clock_pub = self.create_publisher(Clock, '/clock', sensor)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child, xyz in [('map', 'world', (0., 0., 0.)),
                                   ('base_link', 'laser_link', (0., 0., .55))]:
            tr = TransformStamped()
            tr.header.frame_id = parent
            tr.child_frame_id = child
            tr.transform.translation = Vector3(x=xyz[0], y=xyz[1], z=xyz[2])
            tr.transform.rotation.w = 1.
            transforms.append(tr)
        self.static.sendTransform(transforms)
        self.create_service(QueryRay, '/mock/scene/query_ray', self.query)
        self.info_pub.publish(String(data=json.dumps({
            'source': 'MOCK_FIXTURES_ONLY', 'is_mock': True,
            'scene_id': 'mock_bc_fixture', 'scene_version': '0.1.0',
            'query_service': '/mock/scene/query_ray', 'uwb_cases': CASES,
            'scan_special_indices': {'0': 'NO_RETURN', '1': 'INPUT_INVALID_NAN',
                                     '2': 'BELOW_MIN', '3': 'ABOVE_MAX'},
            'phase_duration_s': self.phase_s,
            'phase_work_modes': ['TRAVELLING', 'CUTTING', 'TRAVELLING', 'IDLE'],
            'phase_path_y_m': [5., 25., 35., 5., 5.],
            'zone_y_interval_m': [20., 40.],
            'warning': 'Synthetic inputs, no Gazebo physics, not scene geometry or error algorithms.'})))
        self.timer = self.create_timer(.02, self.step)
        self.get_logger().warning('MOCK_FIXTURES_ONLY: use a separate ROS_DOMAIN_ID; no real query service.')

    def make_context(self, frame, sequence):
        c = RunContext()
        c.header.stamp.sec = self.latest_ns // 1000000000
        c.header.stamp.nanosec = self.latest_ns % 1000000000
        c.header.frame_id = frame
        c.schema_version = '0.2.0'
        c.header.experiment_id = 'MOCK_FIXTURES_ONLY'
        c.header.run_id = self.run_id
        c.header.scene_id = 'mock_bc_fixture'
        c.header.scene_version = '0.1.0'
        c.robot_id = 'shearer_01'
        c.sequence = sequence
        return c

    def step(self):
        self.tick += 1
        self.latest_ns = self.tick * 20000000
        t = self.latest_ns * 1e-9
        clock = Clock()
        clock.clock = self.make_context('map', 0).header.stamp
        self.clock_pub.publish(clock)
        # Continuous synthetic path: outside -> inside -> inside -> outside.
        # Linear phase interpolation is deliberately a fixture, not plant dynamics.
        phase = int(t / self.phase_s) % 4
        fraction = (t % self.phase_s) / self.phase_s
        ys = [5., 25., 35., 5., 5.]
        y = ys[phase] + (ys[phase+1]-ys[phase])*fraction
        vy = (ys[phase+1]-ys[phase])/self.phase_s
        truth = GroundTruth()
        truth.context = self.make_context('map', self.tick)
        truth.observation_id = f'{self.run_id}/shearer_01/ground_truth/{self.tick}'
        truth.child_frame_id = 'base_link'
        truth.pose.position = Point(x=.6, y=y, z=.6605)
        truth.pose.orientation.w = 1.
        truth.twist.linear.y = vy
        self.truth_pub.publish(truth)
        self.context_pub.publish(truth.context)
        state = RobotState()
        state.context = deepcopy(truth.context)
        state.work_mode = ['TRAVELLING', 'CUTTING', 'TRAVELLING', 'IDLE'][phase]
        state.zone_id = 'fixture_dust_zone' if 20. <= y <= 40. else 'NONE'
        state.is_moving = abs(vy) > .001
        state.commanded_linear_speed_mps = 0.
        state.commanded_velocity_map_mps.y = vy
        state.commanded_yaw_rate_radps = 0.
        self.state_pub.publish(state)
        tr = TransformStamped()
        tr.header.stamp = deepcopy(truth.context.header.stamp)
        tr.header.frame_id = 'world'
        tr.child_frame_id = 'base_link'
        tr.transform.translation = Vector3(x=.6, y=y, z=.6605)
        tr.transform.rotation.w = 1.
        self.tf.sendTransform(tr)
        if self.tick % 5:
            return
        self.publish_scan()
        for i, (anchor, case) in enumerate(CASES.items()):
            raw = UwbRangeRaw()
            raw.context = self.make_context('map', self.batch)
            raw.observation_id = f'{self.run_id}/shearer_01/tag_01/{anchor}/{self.batch}'
            raw.tag_id = 'tag_01'
            raw.anchor_id = anchor
            raw.tag_position_m = Point(x=.6, y=y, z=1.2105)
            raw.anchor_position_m = Point(x=float(i)*.2-1., y=200. if case=='OUT_OF_RANGE' else 15., z=2.)
            raw.truth_range_m = math.dist([.6, y, 1.2105], [raw.anchor_position_m.x, raw.anchor_position_m.y, 2.])
            raw.sensor_max_range_m = 120.
            raw.raw_status = case if case in ['INPUT_INVALID', 'SENSOR_UNAVAILABLE', 'OUT_OF_RANGE'] else 'OK'
            raw.raw_valid = raw.raw_status == 'OK'
            raw.raw_range_m = raw.truth_range_m if raw.raw_valid else math.nan
            self.uwb_pub.publish(raw)
            self.observations[raw.observation_id] = deepcopy(raw)
        # Bound memory while preserving the 2-second query window.
        threshold = self.latest_ns - 2000000000
        self.observations = {k: v for k, v in self.observations.items()
                             if self.ns(v.context.header.stamp) >= threshold}
        self.batch += 1

    def publish_scan(self):
        scan = LidarFrameRaw()
        scan.context = self.make_context('laser_link', self.batch)
        scan.observation_id = f'{self.run_id}/shearer_01/lidar_01/frame/{self.batch}'
        scan.sensor_id = 'lidar_01'
        scan.beam_count = 720
        scan.angle_min_rad = -math.pi
        scan.angle_max_rad = math.pi
        scan.angle_increment_rad = 2*math.pi/719
        scan.scan_time_s = .1
        scan.time_increment_s = 0.
        scan.range_min_m = .05
        scan.range_max_m = 30.
        ranges = [float(v) for v in self.fixture['lidar']['ranges_m']]
        scan.raw_ranges_m = ranges
        scan.truth_ranges_m = ranges
        scan.intensities = []
        scan.raw_valid = [False]*4 + [True]*716
        scan.raw_status = [1, 2, 3, 2] + [0]*716
        self.scan_pub.publish(scan)

    @staticmethod
    def ns(stamp):
        return stamp.sec*1000000000 + stamp.nanosec

    def query(self, req, res):
        begin = time.perf_counter()
        res.context = deepcopy(req.context)
        res.request_id = req.request_id
        res.observation_id = req.observation_id
        res.served_scene_version = '0.1.0'
        res.error_message = 'MOCK_ONLY: synthetic fixture facts, not scene geometry'
        try:
            c = req.context
            values = [req.origin_m.x, req.origin_m.y, req.origin_m.z,
                      req.target_m.x, req.target_m.y, req.target_m.z,
                      req.reflector_search_radius_m]
            if not req.request_id or not req.observation_id or c.header.frame_id != 'map' or not all(math.isfinite(v) for v in values) or not 0 <= values[-1] <= 5.:
                res.status = 'INVALID_REQUEST'
            elif c.header.scene_id != 'mock_bc_fixture':
                res.status = 'SCENE_MISMATCH'
            elif c.header.scene_version != '0.1.0':
                res.status = 'VERSION_MISMATCH'
            elif c.schema_version != '0.2.0':
                res.status = 'UNSUPPORTED'
            elif c.header.run_id != self.run_id or not 0 <= self.latest_ns-self.ns(c.header.stamp) <= 2000000000:
                res.status = 'TIME_UNAVAILABLE'
            else:
                raw = self.observations.get(req.observation_id)
                if raw is None or raw.context != c or math.dist(values[:3], values[3:6]) < 1e-6:
                    res.status = 'INVALID_REQUEST'
                elif req.origin_m != raw.tag_position_m or req.target_m != raw.anchor_position_m:
                    res.status = 'INVALID_REQUEST'
                elif not raw.raw_valid or CASES[raw.anchor_id] == 'QUERY_ERROR':
                    res.status = 'UNSUPPORTED'
                    res.error_message += '; scripted unavailable input/query'
                else:
                    res.status = 'OK'
                    self.geometry(req, res, CASES[raw.anchor_id])
            return res
        finally:
            res.processing_time_ms = (time.perf_counter()-begin)*1000.

    def geometry(self, req, res, case):
        a = [req.origin_m.x, req.origin_m.y, req.origin_m.z]
        b = [req.target_m.x, req.target_m.y, req.target_m.z]
        length = math.dist(a, b)
        unit = [(y-x)/length for x, y in zip(a,b)]
        center = [(x+y)/2 for x,y in zip(a,b)]
        if case in ['INTERSECTION', 'BOTH']:
            hit = Intersection()
            hit.geometry_id = 'MOCK_BLOCK'
            hit.geometry_type = 'BOX'
            hit.material = 'MOCK_CONCRETE'
            hit.reflection_level = 'LOW'
            width = min(.3, length*.1)
            hit.entry_point_m = Point(**dict(zip('xyz', [x-u*width/2 for x,u in zip(center,unit)])))
            hit.exit_point_m = Point(**dict(zip('xyz', [x+u*width/2 for x,u in zip(center,unit)])))
            hit.entry_normal = Vector3(**dict(zip('xyz', [-u for u in unit])))
            hit.distance_from_origin_m = length/2-width/2
            hit.thickness_m = width
            hit.path_length_m = width
            res.direct_intersections = [hit]
        if case in ['REFLECTOR', 'BOTH'] and req.reflector_search_radius_m > 0:
            normal = [-unit[1], unit[0], 0.]
            norm = math.sqrt(sum(v*v for v in normal))
            normal = [v/norm for v in normal] if norm > 1e-9 else [1.,0.,0.]
            offset = min(.5, req.reflector_search_radius_m)
            ref = ReflectorCandidate()
            ref.geometry_id = 'MOCK_REFLECTOR'
            ref.geometry_type = 'BOX'
            ref.material = 'MOCK_METAL'
            ref.reflection_level = 'HIGH'
            ref.closest_point_m = Point(**dict(zip('xyz', [x+n*offset for x,n in zip(center,normal)])))
            ref.normal = Vector3(**dict(zip('xyz', [-n for n in normal])))
            ref.distance_to_link_m = offset
            res.reflector_candidates = [ref]
        res.error_message += '; case='+case


def main():
    rclpy.init()
    node = FixtureInputs()
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
