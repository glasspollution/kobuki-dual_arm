"""Unit tests for ik.py.

Beyond FK/IK round-trips, this validates the ENTIRE mission choreography
offline: every grasp, flip and place waypoint used by mission_node must be
reachable within joint limits for both arms. Runnable without ROS:

    python3 -m pytest test/test_ik.py     (or)     python3 test/test_ik.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from kobuki_bimanual_behavior import ik


def check_roundtrip(x, y, z, p, side, tol=1e-9):
    q = ik.solve(x, y, z, p, side)
    fx, fy, fz, fp = ik.fk(q, side)
    assert abs(fx - x) < tol, (fx, x)
    assert abs(fy - y) < tol, (fy, y)
    assert abs(fz - z) < tol, (fz, z)
    assert abs(math.remainder(fp - p, 2 * math.pi)) < tol, (fp, p)


def test_roundtrip_grid():
    n_ok = 0
    for side in (+1, -1):
        for x in (0.18, 0.24, 0.30, 0.36):
            for y in (-0.12, -0.05, 0.0, 0.05, 0.12):
                for z in (0.05, 0.15, 0.25, 0.35):
                    for p in (-math.pi / 2, -math.pi / 4, 0.0):
                        if ik.reachable(x, y, z, p, side):
                            check_roundtrip(x, y, z, p, side)
                            n_ok += 1
    assert n_ok > 100, f'grid too sparse, only {n_ok} reachable points'


def test_unreachable_raises():
    try:
        ik.solve(1.0, 0.0, 0.2, 0.0, +1)
        raise AssertionError('expected IKError')
    except ik.IKError:
        pass


def _flip_waypoints_unused():
    pass


# object parameter sets - keep in sync with config/mission_params.yaml
OBJECTS = {
    'bottle': dict(lying_z=0.031, s=0.050, z_off=0.010, z_place=0.225,
                   y_off=0.0),
    'bowl': dict(lying_z=0.041, s=0.035, z_off=0.010, z_place=0.175,
                 y_off=0.10),
    'cracker_box': dict(lying_z=0.031, s=0.050, z_off=0.010, z_place=0.215,
                        y_off=-0.10),
}


def test_mission_choreography_reachable():
    from kobuki_bimanual_behavior import choreography as ch
    for name, o in OBJECTS.items():
        # mirror mission_node.pick_flip_place: hands grip z_off above the
        # lying center-line and release z_off above the nominal place height
        grasp_z = o['lying_z'] + o['z_off']
        place_z = o['z_place'] + o['z_off']
        seq = []
        seq += ch.grasp_waypoints(o['s'], grasp_z)
        seq += ch.flip_waypoints(o['s'], grasp_z)
        seq += ch.place_waypoints(o['s'], place_z, y_off=o['y_off'])
        seq += ch.clear_waypoints(o['s'], place_z, y_off=o['y_off'])
        assert len(seq) > 10
        for i, (wp_l, wp_r) in enumerate(seq):
            assert ik.reachable(*wp_l, +1), \
                f'{name}: left wp {i} unreachable: {wp_l}'
            assert ik.reachable(*wp_r, -1), \
                f'{name}: right wp {i} unreachable: {wp_r}'


def test_tuck_pose_within_limits():
    from kobuki_bimanual_behavior import choreography as ch
    for q, side in ((ch.TUCK_LEFT, +1), (ch.TUCK_RIGHT, -1),
                    (ch.STAGE_LEFT, +1), (ch.STAGE_RIGHT, -1)):
        for v, lim in zip(q, (ik.YAW_LIM, ik.PITCH_LIM, ik.PITCH_LIM,
                              ik.PITCH_LIM)):
            assert abs(v) <= lim
        x, y, z, p = ik.fk(q, side)
        assert z > 0.15, f'pose hand too low: {z:.3f}'
        assert x < 0.35, f'pose hand too far forward: {x:.3f}'


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f'PASS {name}')
            except Exception as exc:  # noqa: BLE001
                fails += 1
                print(f'FAIL {name}: {exc}')
    sys.exit(1 if fails else 0)
