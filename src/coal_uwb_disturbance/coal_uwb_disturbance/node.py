#!/usr/bin/env python3
"""B 侧 UWB 扰动节点（初版）。

输入：
  /platform/v1/uwb_ranges_raw   coal_platform_interfaces/UwbRangeRaw
  /scene/query_ray | /mock/scene/query_ray   coal_platform_interfaces/srv/QueryRay
输出：
  /disturbance_decision/uwb     coal_platform_interfaces/UwbDecision
  /uwb_ranges                   coal_platform_interfaces/UwbRangeObserved

内部保留四级状态 LOS/SOFT/HARD/OUTAGE，上报按平台枚举映射：
  SOFT、HARD -> propagation_state="NLOS"，分别用 cause=THIN_WALL / SINGLE_REFLECTION 区分；
  LOS -> "LOS"；OUTAGE -> "OUTAGE"；输入无效 -> "INPUT_INVALID"；查询失败 -> "UNKNOWN"。
"""

import hashlib
import math
import random
import uuid
import time
import json

import rclpy
import yaml
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from coal_platform_interfaces.msg import UwbDecision, UwbQueryTrace, UwbRangeObserved, UwbRangeRaw
from coal_platform_interfaces.srv import QueryRay

from coal_uwb_disturbance.hysteresis import StateHysteresis

# 内部状态
LOS, SOFT, HARD, OUTAGE, INPUT_INVALID, QUERY_FAILED = range(6)

STATE_TEXT = {
    LOS: "LOS",
    SOFT: "NLOS",
    HARD: "NLOS",
    OUTAGE: "OUTAGE",
    INPUT_INVALID: "INPUT_INVALID",
    QUERY_FAILED: "UNKNOWN",
}
CAUSE_TEXT = {
    LOS: "DIRECT_CLEAR",
    SOFT: "THIN_WALL",
    HARD: "SINGLE_REFLECTION",
    OUTAGE: "BLOCKED_NO_REFLECTOR",
    INPUT_INVALID: "INPUT_INVALID",
    QUERY_FAILED: "QUERY_FAILED",
}
NLOS_TYPE = {LOS: "none", SOFT: "soft", HARD: "hard", OUTAGE: "none",
             INPUT_INVALID: "none", QUERY_FAILED: "none"}


class UwbDisturbance(Node):
    def __init__(self):
        super().__init__("coal_uwb_disturbance")

        self.declare_parameter("query_service", "/mock/scene/query_ray")
        self.declare_parameter("config_file", "")
        self.declare_parameter("seed", 42)
        self.declare_parameter("gaussian_std_m", 0.03)
        self.declare_parameter("bias_mean_m", 0.8)
        self.declare_parameter("bias_mode", "iid")
        self.declare_parameter("soft_bias_ratio", 0.2)
        self.declare_parameter("ar1_rho", 0.9)
        self.declare_parameter("multipath_sigma_m", 0.02)
        self.declare_parameter("thin_wall_max_m", 0.25)
        self.declare_parameter("reflector_search_radius_m", 2.0)
        self.declare_parameter("max_range_m", 120.0)
        self.declare_parameter("params_version", "v0.1")
        self.declare_parameter("log_every", 200)
        self.declare_parameter("hysteresis_enter_frames", 3)
        self.declare_parameter("hysteresis_exit_frames", 2)
        self.declare_parameter("query_timeout_s", 5.0)
        self.declare_parameter("publish_processing_diagnostics", False)

        cfg = {}
        path = self.get_parameter("config_file").value
        if path:
            with open(path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
        self.seed = int(cfg.get("seed", self.get_parameter("seed").value))
        self.sigma_g = float(cfg.get("gaussian_std_m", self.get_parameter("gaussian_std_m").value))
        self.bias_mean = float(cfg.get("bias_mean_m", self.get_parameter("bias_mean_m").value))
        self.bias_mode = str(cfg.get("bias_mode", self.get_parameter("bias_mode").value))
        self.soft_ratio = float(cfg.get("soft_bias_ratio", self.get_parameter("soft_bias_ratio").value))
        self.rho = float(cfg.get("ar1_rho", self.get_parameter("ar1_rho").value))
        self.sigma_m = float(cfg.get("multipath_sigma_m", self.get_parameter("multipath_sigma_m").value))
        self.thin_wall = float(cfg.get("thin_wall_max_m", self.get_parameter("thin_wall_max_m").value))
        self.reflector_radius = float(
            cfg.get("reflector_search_radius_m", self.get_parameter("reflector_search_radius_m").value)
        )
        self.params_version = str(cfg.get("params_version", self.get_parameter("params_version").value))
        self.log_every = int(self.get_parameter("log_every").value)
        self.hyst = StateHysteresis(
            int(cfg.get("hysteresis_enter_frames",
                        self.get_parameter("hysteresis_enter_frames").value)),
            int(cfg.get("hysteresis_exit_frames",
                        self.get_parameter("hysteresis_exit_frames").value)))

        self._rng = {}
        self._ar = {}
        self._episode_bias = {}
        self._count = 0
        self._stats = {}
        self._pending_queries = {}
        self._transport_stats = dict(received=0, sent=0, completed=0, timed_out=0, call_errors=0)
        self.query_timeout_s = float(self.get_parameter('query_timeout_s').value)
        if not math.isfinite(self.query_timeout_s) or self.query_timeout_s <= 0:
            raise ValueError('query_timeout_s must be positive and finite')
        self.processing_pub = self.create_publisher(String, '/platform/v1/uwb_processing', 10) if self.get_parameter('publish_processing_diagnostics').value else None
        self._last_processing_report = 0.
        self._pending_timer = self.create_timer(.1, self._check_pending,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        raw_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.raw_sub = self.create_subscription(
            UwbRangeRaw, "/platform/v1/uwb_ranges_raw", self.on_raw, raw_qos
        )
        self.decision_pub = self.create_publisher(
            UwbDecision, "/disturbance_decision/uwb",
            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.observed_pub = self.create_publisher(
            UwbRangeObserved, "/uwb_ranges",
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.query_trace_pub = self.create_publisher(
            UwbQueryTrace, "/disturbance_query/uwb",
            QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.client = self.create_client(
            QueryRay, self.get_parameter("query_service").value,
            qos_profile=QoSProfile(depth=512, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.get_logger().info(
            f"UWB disturbance ready: query={self.get_parameter('query_service').value}, "
            f"seed={self.seed}, params={self.params_version}"
        )

    # ---------- 随机流：每条链路独立、可复现 ----------
    def _link_key(self, raw):
        return (raw.tag_id, raw.anchor_id)

    def _rng_for(self, key):
        if key not in self._rng:
            h = hashlib.sha256(f"{self.seed}|{key[0]}|{key[1]}".encode()).hexdigest()
            self._rng[key] = random.Random(int(h[:16], 16))
        return self._rng[key]

    def _bias_for(self, state, key, rng):
        if state == HARD:
            if self.bias_mode == "episode":
                if key not in self._episode_bias:
                    self._episode_bias[key] = rng.expovariate(1.0 / self.bias_mean)
                return self._episode_bias[key]
            return rng.expovariate(1.0 / self.bias_mean)
        if state == SOFT:
            if self.bias_mode == "episode":
                if key not in self._episode_bias:
                    self._episode_bias[key] = self.soft_ratio * rng.expovariate(1.0 / self.bias_mean)
                return self._episode_bias[key]
            return self.soft_ratio * rng.expovariate(1.0 / self.bias_mean)
        return 0.0

    def _multipath(self, key, rng):
        x = self._ar.get(key, 0.0)
        x = self.rho * x + math.sqrt(max(0.0, 1.0 - self.rho ** 2)) * self.sigma_m * rng.gauss(0.0, 1.0)
        self._ar[key] = x
        return x

    # ---------- 输入 ----------
    def on_raw(self, raw: UwbRangeRaw):
        self._transport_stats['received'] += 1
        if not raw.raw_valid:
            state = OUTAGE if raw.raw_status == "OUT_OF_RANGE" else INPUT_INVALID
            self._publish(raw, state, None, valid=False)
            return
        if not self.client.service_is_ready():
            self._publish(raw, QUERY_FAILED, None, valid=False)
            return

        req = QueryRay.Request()
        req.context = raw.context
        req.request_id = uuid.uuid4().hex
        req.observation_id = raw.observation_id
        req.origin_m = raw.tag_position_m
        req.target_m = raw.anchor_position_m
        req.reflector_search_radius_m = self.reflector_radius
        try:
            future = self.client.call_async(req)
        except Exception as exc:  # noqa: BLE001
            self._transport_stats['call_errors'] += 1
            self.get_logger().warning(f"query call failed: {exc}")
            self._publish(raw, QUERY_FAILED, None, valid=False, query_status="INTERNAL_ERROR",
                          request=req, query_error=str(exc))
            return
        self._transport_stats['sent'] += 1
        self._pending_queries[req.request_id] = (future, raw, req, time.monotonic())
        future.add_done_callback(lambda fut, r=raw, q=req: self._on_managed_query(fut, r, q))

    def _on_managed_query(self, future, raw, request):
        if self._pending_queries.pop(request.request_id, None) is None:
            return  # A timed-out request has already emitted its one result.
        self._transport_stats['completed'] += 1
        self.on_query(future, raw, request)

    def _check_pending(self):
        now = time.monotonic()
        for request_id, (future, raw, request, start) in list(self._pending_queries.items()):
            if now - start < self.query_timeout_s:
                continue
            self._pending_queries.pop(request_id)
            self.client.remove_pending_request(future)
            self._transport_stats['timed_out'] += 1
            self._publish(raw, QUERY_FAILED, None, valid=False, query_status='INTERNAL_ERROR',
                          request=request, query_error='RPC_TIMEOUT')
            future.cancel()
        if self.processing_pub is not None and now - self._last_processing_report >= 1.:
            self.processing_pub.publish(String(data=json.dumps(dict(self._transport_stats,
                pending=len(self._pending_queries), published=self._count))))
            self._last_processing_report = now

    def on_query(self, future, raw: UwbRangeRaw, request=None):
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"query failed: {exc}")
            self._publish(raw, QUERY_FAILED, None, valid=False, query_status="INTERNAL_ERROR",
                          request=request, query_error=str(exc))
            return
        if res is None or res.status != "OK":
            self._publish(raw, QUERY_FAILED, res, valid=False,
                          query_status="INTERNAL_ERROR" if res is None else res.status,
                          request=request, query_error="Empty RPC response" if res is None else "")
            return
        state = self._classify(raw, res)
        state = self.hyst.update(self._link_key(raw), state)
        # Outage 表示没有可用观测，不能当作有效测距。
        valid = state in (LOS, SOFT, HARD)
        self._publish(raw, state, res, valid=valid, request=request)

    def _classify(self, raw: UwbRangeRaw, res: QueryRay.Response) -> int:
        hits = list(res.direct_intersections)
        if not hits:
            return LOS
        thickness = [h.thickness_m for h in hits if math.isfinite(h.thickness_m)]
        min_thick = min(thickness) if thickness else float("nan")
        if math.isfinite(min_thick) and min_thick < self.thin_wall:
            return SOFT
        if len(res.reflector_candidates) > 0:
            return HARD
        return OUTAGE

    # ---------- 输出 ----------
    def _publish(self, raw: UwbRangeRaw, state: int, res, valid: bool, query_status=None,
                 request=None, query_error=""):
        key = self._link_key(raw)
        rng = self._rng_for(key)
        decision = UwbDecision()
        decision.context = raw.context
        decision.observation_id = raw.observation_id
        decision.decision_id = uuid.uuid4().hex
        decision.params_version = self.params_version
        decision.seed = self.seed
        decision.anchor_id = raw.anchor_id
        decision.propagation_state = STATE_TEXT[state]
        decision.nlos_type = NLOS_TYPE[state]
        decision.cause = CAUSE_TEXT[state]
        decision.query_status = query_status or (res.status if res is not None else "NOT_CALLED")
        decision.valid = valid
        decision.nlos_active = state in (SOFT, HARD)
        decision.outage_active = state in (OUTAGE, INPUT_INVALID, QUERY_FAILED)
        decision.multipath_active = bool(valid and self.sigma_m > 0.0)
        decision.blocking_count = len(res.direct_intersections) if res is not None else 0
        decision.reflector_count = len(res.reflector_candidates) if res is not None else 0
        decision.trigger_mask = 0
        if not raw.raw_valid:
            decision.trigger_mask |= 1
        if raw.raw_status == "OUT_OF_RANGE":
            decision.trigger_mask |= 2
        if decision.blocking_count > 0:
            decision.trigger_mask |= 4
        if decision.multipath_active:
            decision.trigger_mask |= 8
        if state == QUERY_FAILED:
            decision.trigger_mask |= 16

        observed = UwbRangeObserved()
        observed.context = raw.context
        observed.observation_id = raw.observation_id
        observed.decision_id = decision.decision_id
        observed.tag_id = raw.tag_id
        observed.anchor_id = raw.anchor_id
        observed.tag_position_m = raw.tag_position_m
        observed.anchor_position_m = raw.anchor_position_m
        observed.truth_range_m = raw.truth_range_m
        observed.raw_range_m = raw.raw_range_m
        observed.state = decision.propagation_state
        observed.nlos_type = decision.nlos_type
        observed.valid = valid

        if valid:
            noise = rng.gauss(0.0, self.sigma_g)
            bias = self._bias_for(state, key, rng)
            multipath = self._multipath(key, rng)
            decision.random_noise_m = noise
            decision.nlos_error_m = bias
            decision.multipath_error_m = multipath
            observed.observed_range_m = raw.raw_range_m + noise + bias + multipath
            observed.error_m = observed.observed_range_m - raw.truth_range_m
        else:
            decision.random_noise_m = 0.0
            decision.nlos_error_m = 0.0
            decision.multipath_error_m = 0.0
            observed.observed_range_m = float("nan")
            observed.error_m = float("nan")

        self.decision_pub.publish(decision)
        self.observed_pub.publish(observed)
        if request is not None:
            trace = UwbQueryTrace()
            trace.context = raw.context
            trace.observation_id = raw.observation_id
            trace.decision_id = decision.decision_id
            trace.request_id = request.request_id
            trace.tag_id = raw.tag_id
            trace.anchor_id = raw.anchor_id
            trace.nlos_type = decision.nlos_type
            trace.valid = decision.valid
            trace.status = decision.query_status
            if res is not None:
                trace.direct_intersections = res.direct_intersections
                trace.reflector_candidates = res.reflector_candidates
                trace.processing_time_ms = res.processing_time_ms
                trace.error_message = res.error_message
            else:
                trace.processing_time_ms = float("nan")
                trace.error_message = query_error
            self.query_trace_pub.publish(trace)
        self._count += 1
        self._stats[STATE_TEXT[state]] = self._stats.get(STATE_TEXT[state], 0) + 1
        if self.log_every > 0 and self._count % self.log_every == 0:
            self.get_logger().info(f"processed={self._count} stats={self._stats}")


def main(args=None):
    rclpy.init(args=args)
    node = UwbDisturbance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
