#!/usr/bin/env python3
"""Regression: called-but-failed queries must not be labelled NOT_CALLED."""
import math
import unittest
from concurrent.futures import Future
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/coal_uwb_disturbance'))
import rclpy
from coal_platform_interfaces.msg import UwbRangeRaw
from coal_platform_interfaces.srv import QueryRay
from coal_uwb_disturbance.node import UwbDisturbance


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class QueryFailures(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = UwbDisturbance()
        self.node.decision_pub = Capture()
        self.node.observed_pub = Capture()
        self.node.query_trace_pub = Capture()
        self.raw = UwbRangeRaw(raw_valid=True, raw_status='OK',
                               tag_id='tag', anchor_id='anchor',
                               observation_id='failure-case')
        self.request = QueryRay.Request(request_id='request-case')

    def tearDown(self):
        self.node.destroy_node()

    def check_failed(self, expected):
        decision = self.node.decision_pub.messages[-1]
        observed = self.node.observed_pub.messages[-1]
        self.assertEqual(decision.query_status, expected)
        self.assertFalse(decision.valid)
        self.assertFalse(observed.valid)
        self.assertTrue(math.isnan(observed.observed_range_m))
        self.assertEqual(decision.nlos_type, 'none')
        self.assertEqual(observed.nlos_type, 'none')
        if expected == 'INTERNAL_ERROR':
            trace = self.node.query_trace_pub.messages[-1]
            self.assertEqual(trace.status, expected)
            self.assertEqual(trace.nlos_type, 'none')
            self.assertTrue(trace.request_id)
        else:
            self.assertEqual(self.node.query_trace_pub.messages, [])

    def test_future_exception(self):
        future = Future()
        future.set_exception(RuntimeError('injected transport error'))
        self.node.on_query(future, self.raw, self.request)
        self.check_failed('INTERNAL_ERROR')

    def test_empty_response(self):
        future = Future()
        future.set_result(None)
        self.node.on_query(future, self.raw, self.request)
        self.check_failed('INTERNAL_ERROR')

    def test_call_exception(self):
        class Client:
            def service_is_ready(self):
                return True

            def call_async(self, request):
                raise RuntimeError('injected call error')
        self.node.client = Client()
        self.node.on_raw(self.raw)
        self.check_failed('INTERNAL_ERROR')

    def test_service_not_ready(self):
        class Client:
            def service_is_ready(self):
                return False
        self.node.client = Client()
        self.node.on_raw(self.raw)
        self.check_failed('NOT_CALLED')

    def test_invalid_input(self):
        self.raw.raw_valid = False
        self.node.on_raw(self.raw)
        self.check_failed('NOT_CALLED')

    def test_timeout_emits_exactly_one_invalid_result(self):
        class Client:
            def remove_pending_request(self, future):
                pass
        self.node.client = Client()
        future = Future()
        self.node._pending_queries[self.request.request_id] = (
            future, self.raw, self.request, time.monotonic() - 6.)
        self.node._check_pending()
        self.check_failed('INTERNAL_ERROR')
        self.assertEqual(self.node.query_trace_pub.messages[-1].error_message, 'RPC_TIMEOUT')
        self.assertEqual(self.node._transport_stats['timed_out'], 1)
        self.assertEqual(self.node._pending_queries, {})
        self.node._on_managed_query(future, self.raw, self.request)
        self.assertEqual(len(self.node.decision_pub.messages), 1)

    def test_inflight_request_keeps_waiting_before_deadline(self):
        future = Future()
        self.node._pending_queries[self.request.request_id] = (
            future, self.raw, self.request, time.monotonic())
        self.node._check_pending()
        self.assertEqual(len(self.node._pending_queries), 1)
        self.assertEqual(self.node.decision_pub.messages, [])

    def test_timeout_removes_actual_ros_client_future(self):
        future = self.node.client.call_async(self.request)
        self.node._pending_queries[self.request.request_id] = (
            future, self.raw, self.request, time.monotonic() - 6.)
        self.node._check_pending()
        self.check_failed('INTERNAL_ERROR')
        self.assertTrue(future.cancelled())
        self.assertEqual(self.node._pending_queries, {})


if __name__ == '__main__':
    unittest.main()
