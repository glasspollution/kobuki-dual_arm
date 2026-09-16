"""Analytic FK/IK for the 4-DOF arms (yaw + 3 coplanar pitch).

Pure Python + math only, so it can be unit-tested without ROS.

Frames and conventions (MUST match kobuki_bimanual_description):
  * All poses are in the base_link frame: origin on the ground under the
    base center, x forward, y left, z up.
  * Shoulder pitch center (J2) sits at
        S = (SHOULDER_X, side * SHOULDER_Y, SHOULDER_Z),  side = +1 left / -1 right
    and the J1 yaw axis passes vertically through it (the riser is coaxial
    with J1), so the yaw/planar decomposition is exact.
  * Joint q1 is yaw about +z (0 = arm along +x).
  * Joints q2..q4 rotate about the local +y axis; POSITIVE pitch moves the
    arm DOWN (right-hand rule).
  * "pitch" of a target is the elevation angle of the end-effector +x axis:
    0 = horizontal (pointing away from the shoulder), -pi/2 = straight down.
  * The target point is the GRASP POINT: LG along the end-effector +x axis
    beyond the J4 wrist (between the finger pads).
"""
import math

# geometry (metres) - single source of truth for the behavior side
SHOULDER_X = 0.095
SHOULDER_Y = 0.08875
SHOULDER_Z = 0.180
L1 = 0.160
L2 = 0.160
LG = 0.070

# joint limits (rad) - keep in sync with arm.xacro
YAW_LIM = 2.618
PITCH_LIM = 2.0

# reachable band for the wrist relative to the shoulder. Below ~0.174 m the
# elbow interior angle exceeds the +/-115 deg joint limit.
D_MIN_SOFT = 0.174


class IKError(Exception):
    pass


def shoulder(side):
    """side: +1 = left arm, -1 = right arm."""
    return (SHOULDER_X, side * SHOULDER_Y, SHOULDER_Z)


def fk(q, side):
    """Forward kinematics of the grasp point.

    Returns (x, y, z, pitch) for joint vector q = [q1, q2, q3, q4].
    """
    q1, q2, q3, q4 = q
    sx, sy, sz = shoulder(side)
    e2 = -q2
    e3 = -(q2 + q3)
    e4 = -(q2 + q3 + q4)
    r = L1 * math.cos(e2) + L2 * math.cos(e3) + LG * math.cos(e4)
    dz = L1 * math.sin(e2) + L2 * math.sin(e3) + LG * math.sin(e4)
    return (sx + r * math.cos(q1),
            sy + r * math.sin(q1),
            sz + dz,
            e4)


def solve(x, y, z, pitch, side):
    """Inverse kinematics. Returns [q1, q2, q3, q4] (elbow-up preferred).

    Raises IKError when the target is unreachable or violates joint limits.
    """
    sx, sy, sz = shoulder(side)
    dx = x - sx
    dy = y - sy
    q1 = math.atan2(dy, dx)
    if abs(q1) > YAW_LIM:
        raise IKError(f'yaw {q1:.2f} beyond limit')

    r = math.hypot(dx, dy)
    # wrist (J4) position in the yawed vertical plane
    rw = r - LG * math.cos(pitch)
    zw = (z - sz) - LG * math.sin(pitch)
    d = math.hypot(rw, zw)

    if d > L1 + L2 - 1e-9:
        raise IKError(f'target too far: wrist dist {d:.3f} > {L1 + L2:.3f}')
    if d < 1e-6:
        raise IKError('target degenerate (wrist at shoulder)')

    gamma = math.atan2(zw, rw)
    cos_a = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d)
    cos_b = (L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2)
    cos_a = max(-1.0, min(1.0, cos_a))
    cos_b = max(-1.0, min(1.0, cos_b))
    alpha = math.acos(cos_a)
    beta = math.acos(cos_b)

    solutions = []
    for elbow_sign in (+1, -1):          # +1 elbow-up first
        e2 = gamma + elbow_sign * alpha
        e3 = e2 - elbow_sign * (math.pi - beta)
        q2 = -e2
        q3 = e2 - e3
        q4 = -pitch - q2 - q3
        qs = [q1, q2, q3, q4]
        if all(abs(v) <= lim + 1e-9 for v, lim in
               zip(qs, (YAW_LIM, PITCH_LIM, PITCH_LIM, PITCH_LIM))):
            solutions.append(qs)

    if not solutions:
        raise IKError(
            f'no in-limit solution for ({x:.3f},{y:.3f},{z:.3f},p={pitch:.2f}) '
            f'side={side} (wrist dist {d:.3f})')
    return solutions[0]


def reachable(x, y, z, pitch, side):
    try:
        solve(x, y, z, pitch, side)
        return True
    except IKError:
        return False
