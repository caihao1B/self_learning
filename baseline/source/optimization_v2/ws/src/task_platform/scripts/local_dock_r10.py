"""Small differential-drive docking paths anchored once in measured local odometry.

This is a motion proposal only. The caller must check the entire map-frame sweep,
the live Nav2 costmap, measured stop and the original global work admission.
"""
import math
import statistics


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ready_for_stopped_observation(samples,goal,now):
    """A local proximity proposal to cancel motion, never navigation/work credit.

    The original global stationary gate must still pass after the actual stop.
    Requiring a short fresh window avoids reacting to one noisy odometry sample.
    """
    if len(samples)<8 or samples[-1][0]-samples[0][0]<.25:return False
    if not -.03<=now-samples[-1][0]<.15:return False
    if any(not 0<b[0]-a[0]<=.15 for a,b in zip(samples,samples[1:])):return False
    return all(math.isfinite(t) and all(math.isfinite(v) for v in xy) and math.dist(xy[:2],goal[:2])<=.010 for t,xy in samples)


def offset_observable(metrics):
    """Enough stationary evidence to propose a correction, never to grant work.

    A distance failure cannot disappear by collecting more of the same stable
    offset. Keep the original evidence and scatter limits; the work gate is
    still independently evaluated after the actual correction and stop.
    """
    keys=['samples','span_s','max_gap_s','position_scatter_p95_m','median_distance_m','latest_distance_m']
    if any(k not in metrics or not math.isfinite(metrics[k]) for k in keys):return False
    return (metrics['samples']>=20 and metrics['span_s']>=1.8 and
            metrics['max_gap_s']<=.15 and metrics['position_scatter_p95_m']<=.015 and
            metrics['median_distance_m']>.035 and metrics['latest_distance_m']>.035)


def propose(history, target, global_pose, odom_pose, max_distance=.15):
    if len(history) < 20 or history[-1][0] - history[0][0] < 1.8:
        raise ValueError('INSUFFICIENT_STOPPED_OBSERVATIONS')
    if any(not all(math.isfinite(v) for v in p) for p in [target, global_pose, odom_pose]):
        raise ValueError('NONFINITE_POSE')
    median = [statistics.median(p[1][j] for p in history) for j in (0, 1)]
    if math.dist(global_pose[:2], median) > .03:
        raise ValueError('CURRENT_POSE_DISAGREES_WITH_STATIONARY_MEDIAN')
    delta = [target[j] - median[j] for j in (0, 1)]
    distance = math.hypot(*delta)
    if not .008 < distance <= max_distance:
        raise ValueError('CORRECTION_OUTSIDE_LOCAL_DOCKING_BOUND')
    bearing = math.atan2(delta[1], delta[0])
    turn = wrap(bearing - global_pose[2])
    rotation_steps = max(1, math.ceil(abs(turn) / .04))
    points = [[*global_pose[:2], global_pose[2] + turn * i / rotation_steps]
              for i in range(rotation_steps + 1)]
    steps = max(1, math.ceil(distance / .02))
    points += [[global_pose[0] + delta[0] * i / steps,
                global_pose[1] + delta[1] * i / steps, bearing]
               for i in range(1, steps + 1)]
    angle = wrap(odom_pose[2] - global_pose[2])
    c, s = math.cos(angle), math.sin(angle)
    local = []
    for x, y, h in points:
        dx, dy = x - global_pose[0], y - global_pose[1]
        local.append([odom_pose[0] + c * dx - s * dy,
                      odom_pose[1] + s * dx + c * dy, wrap(h + angle)])
    return dict(map_path=points, odom_path=local, median=median,
                correction_m=distance, frozen_map_to_odom_yaw=angle)
