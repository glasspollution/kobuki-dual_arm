"""Bimanual choreography waypoint generators (pure Python, no ROS).

Both arms work on the robot's sagittal centerline (y = 0). The lying object
has its long axis along +x. The LEFT hand always takes the FAR station
(becomes the TOP hand when the object is upright); the RIGHT hand takes the
NEAR station (bottom).

A waypoint is ((x, y, z, pitch)_left, (x, y, z, pitch)_right) in base_link
frame, targeting the grasp point between the finger pads (see ik.py).

The flip is two-phased to keep the bottom wrist outside the elbow-limit
exclusion zone near the shoulder:
  phase A: lift the object center from the grasp spot to the hover point
           while pre-rotating only slightly (phi 0 -> PHI_LIFT),
  phase B: rotate phi -> 90 deg about the object center at the fixed,
           farther hover point.
"""
import math

# nominal choreography constants (overridable via mission params)
GRASP_FORWARD = 0.30   # object center distance ahead of base center at grasp
FLIP_X = 0.35          # object center x while flipping / placing
FLIP_Z = 0.20          # object center z while flipping
HOVER_DZ = 0.12        # pregrasp hover height above the lying center
PHI_LIFT = 0.35        # rad of pre-rotation during the lift phase
CLEAR_DZ = 0.08        # vertical clearance move after releasing

# safe folded travel pose (joint space), within limits; hands high at the
# sides, out of the camera's forward view
TUCK_LEFT = [0.8, -1.5, 1.85, 1.2]
TUCK_RIGHT = [-0.8, -1.5, 1.85, 1.2]

# staging pose between tuck and any IK trajectory: arm raised forward over
# the deck, so joint-space interpolation from tuck cannot sweep the elbow or
# wrist down through the base disc
STAGE_LEFT = [-0.3, -1.0, 1.5, 0.6]
STAGE_RIGHT = [0.3, -1.0, 1.5, 0.6]

DOWN = -math.pi / 2


def _stations(cx, cz, phi, s):
    """Hand grasp points for object center (cx, 0, cz) at axis angle phi."""
    ax, az = math.cos(phi), math.sin(phi)
    pitch = phi - math.pi / 2
    far = (cx + s * ax, 0.0, cz + s * az, pitch)
    near = (cx - s * ax, 0.0, cz - s * az, pitch)
    return far, near


def grasp_waypoints(s, lying_z, grasp_forward=GRASP_FORWARD, hover_dz=HOVER_DZ):
    """Approach from above: hover -> descend to grasp height."""
    far_x = grasp_forward + s
    near_x = grasp_forward - s
    return [
        ((far_x, 0.0, lying_z + hover_dz, DOWN),
         (near_x, 0.0, lying_z + hover_dz, DOWN)),
        ((far_x, 0.0, lying_z, DOWN),
         (near_x, 0.0, lying_z, DOWN)),
    ]


def vertical_waypoints(s, z_start, z_end, center_x=GRASP_FORWARD, steps=6):
    """Move a lying object vertically without changing its orientation.

    The two grasp stations stay at the same x/y coordinates and both wrists
    keep the downward pitch used for the grasp.  Dense Cartesian samples are
    intentional: the trajectory controller interpolates in joint space, so a
    start/end-only command does not produce a straight Cartesian path.
    """
    if steps < 2:
        raise ValueError('vertical motion needs at least two waypoints')

    far_x = center_x + s
    near_x = center_x - s
    result = []
    for i in range(steps):
        t = i / (steps - 1)
        z = z_start + t * (z_end - z_start)
        result.append(((far_x, 0.0, z, DOWN),
                       (near_x, 0.0, z, DOWN)))
    return result


def flip_waypoints(s, lying_z, grasp_forward=GRASP_FORWARD,
                   flip_x=FLIP_X, flip_z=FLIP_Z, n_lift=4, n_rot=8):
    """Lift-and-flip arc, returns list of (left_wp, right_wp)."""
    wps = []
    for i in range(1, n_lift + 1):
        t = i / n_lift
        cx = grasp_forward + t * (flip_x - grasp_forward)
        cz = lying_z + t * (flip_z - lying_z)
        far, near = _stations(cx, cz, t * PHI_LIFT, s)
        wps.append((far, near))
    for i in range(1, n_rot + 1):
        phi = PHI_LIFT + (i / n_rot) * (math.pi / 2 - PHI_LIFT)
        far, near = _stations(flip_x, flip_z, phi, s)
        wps.append((far, near))
    return wps


def place_waypoints(s, z_place, flip_x=FLIP_X, flip_z=FLIP_Z, y_off=0.0):
    """Move the upright object from the flip hover down to the shelf.

    y_off shifts the release point laterally along the slab so successive
    objects land side by side instead of on top of each other.
    """
    far, near = _stations(flip_x, z_place, math.pi / 2, s)
    far = (far[0], y_off, far[2], far[3])
    near = (near[0], y_off, near[2], near[3])
    return [(far, near)]


def clear_waypoints(s, z_place, flip_x=FLIP_X, clear_dz=CLEAR_DZ, y_off=0.0):
    """After release: straight vertical clearance (object stays on shelf)."""
    far, near = _stations(flip_x, z_place + clear_dz, math.pi / 2, s)
    far = (far[0], y_off, far[2], far[3])
    near = (near[0], y_off, near[2], near[3])
    return [(far, near)]


def all_arm_waypoints(s, lying_z, z_place):
    """Every cartesian waypoint of one object cycle (for validation)."""
    seq = []
    seq += grasp_waypoints(s, lying_z)
    seq += flip_waypoints(s, lying_z)
    seq += place_waypoints(s, z_place)
    seq += clear_waypoints(s, z_place)
    return seq
