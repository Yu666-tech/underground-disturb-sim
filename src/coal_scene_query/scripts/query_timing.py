"""Static-scene query time admission; never rewrites stamps or extrapolates poses."""
import math


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def tolerance_ns(milliseconds):
    if not math.isfinite(milliseconds) or not 0 <= milliseconds <= 2000:
        raise ValueError('future_tolerance_ms must be finite and in [0, 2000]')
    return round(milliseconds * 1000000)


def run_identity(context):
    h = context.header
    return (h.experiment_id, h.run_id, h.scene_id, h.scene_version, context.robot_id)


def rejection_reason(request_context, truth_context, run_start_ns, future_ns):
    if run_identity(request_context) != run_identity(truth_context):
        return 'IDENTITY_MISMATCH'
    requested = stamp_ns(request_context.header.stamp)
    latest = stamp_ns(truth_context.header.stamp)
    if requested < run_start_ns:
        return 'BEFORE_RUN_START'
    if requested > latest + future_ns:
        return 'FUTURE_BEYOND_TOLERANCE'
    if latest - requested > 2000000000:
        return 'OLDER_THAN_2S'
    return None
