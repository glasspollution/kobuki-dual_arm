"""Offline contract tests for the rotation-only diagnostic mission."""
import ast
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent
GAZEBO_ROOT = WORKSPACE_SRC / 'kobuki_bimanual_gazebo'
sys.path.insert(0, os.fspath(PACKAGE_ROOT))

from kobuki_bimanual_behavior import choreography as ch
from kobuki_bimanual_behavior import ik


def _config():
    with (PACKAGE_ROOT / 'config' / 'stationary_params.yaml').open(
            encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return (data['perception_node']['ros__parameters'],
            data['mission_node']['ros__parameters'])


def _include_poses():
    root = ET.parse(
        GAZEBO_ROOT / 'worlds' / 'stationary.sdf').getroot()
    result = {}
    for include in root.findall('.//include'):
        name = include.findtext('name')
        pose = [float(value) for value in include.findtext('pose').split()]
        result[name] = pose
    return result


def test_stationary_world_matches_fixed_grasp_radius():
    _, mission = _config()
    poses = _include_poses()
    expected = set(mission['objects'])
    assert expected == {'bottle', 'bowl', 'cracker_box'}
    assert expected <= poses.keys()
    radius = float(mission['grasp_forward'])
    for name in expected:
        x, y, _, _, pitch, yaw = poses[name]
        assert abs(math.hypot(x, y) - radius) < 1e-5
        assert abs(math.remainder(yaw - math.atan2(y, x),
                                  2 * math.pi)) < 1e-4
        assert abs(pitch - math.pi / 2) < 1e-4

    shelf_root = ET.parse(
        GAZEBO_ROOT / 'models' / 'stationary_shelf' / 'model.sdf').getroot()
    slab = shelf_root.find(".//collision[@name='slab_col']")
    slab_z = float(slab.findtext('pose').split()[2])
    slab_h = float(slab.findtext('geometry/box/size').split()[2])
    slab_bottom = slab_z - slab_h / 2.0
    slab_top = slab_z + slab_h / 2.0
    assert slab_bottom > 0.14  # clear the robot's deck during every yaw

    vertical_half = {'bottle': 0.03, 'bowl': 0.04,
                     'cracker_box': 0.03}
    for name in expected:
        hand_z = float(mission[name]['stationary_place_z'])
        object_bottom = (hand_z - float(mission[name]['grasp_z_off'])
                         - vertical_half[name])
        assert abs(object_bottom - slab_top) < 0.002


def test_stationary_camera_can_see_the_grasp_annulus():
    perception, mission = _config()
    fx = 640.0 / (2.0 * math.tan(1.204 / 2.0))
    pitch = float(perception['cam_pitch'])
    cam_height = float(perception['cam_height'])
    cam_x = float(perception['cam_x'])

    def floor_range(row):
        depression = pitch + math.atan2(row - 240.0, fx)
        return cam_height / math.tan(depression) + cam_x

    near = floor_range(480.0)
    far = floor_range(0.0)
    for name in mission['objects']:
        contact_range = (float(mission['grasp_forward'])
                         - float(mission[name]['half_len']))
        assert near < contact_range < far, (name, near, contact_range, far)

    launch_source = (GAZEBO_ROOT / 'launch'
                     / 'stationary.launch.py').read_text(encoding='utf-8')
    assert f"'camera_pitch': '{pitch:.6f}'" in launch_source


def test_vertical_paths_are_reachable_and_preserve_orientation():
    _, mission = _config()
    center_x = float(mission['grasp_forward'])
    lift_z = float(mission['stationary_lift_z'])
    steps = int(mission['stationary_vertical_steps'])
    for name in mission['objects']:
        obj = mission[name]
        s = float(obj['s'])
        grasp_z = float(obj['lying_z']) + float(obj['grasp_z_off'])
        place_z = float(obj['stationary_place_z'])
        paths = [
            ch.vertical_waypoints(s, grasp_z + ch.HOVER_DZ, grasp_z,
                                  center_x, steps),
            ch.vertical_waypoints(s, grasp_z, lift_z, center_x, steps),
            ch.vertical_waypoints(s, lift_z, place_z, center_x, steps),
            ch.vertical_waypoints(s, place_z, lift_z, center_x, steps),
        ]
        for path in paths:
            left_xy_pitch = {(round(left[0], 9), round(left[1], 9), left[3])
                             for left, _ in path}
            right_xy_pitch = {(round(right[0], 9), round(right[1], 9),
                               right[3]) for _, right in path}
            assert left_xy_pitch == {
                (round(center_x + s, 9), 0.0, ch.DOWN)}
            assert right_xy_pitch == {
                (round(center_x - s, 9), 0.0, ch.DOWN)}
            for left, right in path:
                assert ik.reachable(*left, +1), (name, left)
                assert ik.reachable(*right, -1), (name, right)

        # Sample every millimetre as a margin check, not just the commanded
        # waypoints.  This guards against a barely reachable endpoint passing
        # while the continuous vertical path brushes a limit.
        z_min = min(grasp_z, place_z)
        sample_count = math.ceil((lift_z - z_min) * 1000.0)
        for index in range(sample_count + 1):
            z = z_min + (lift_z - z_min) * index / sample_count
            for x, side in ((center_x + s, +1), (center_x - s, -1)):
                target = (x, 0.0, z, ch.DOWN)
                q = ik.solve(*target, side)
                limits = (ik.YAW_LIM, ik.PITCH_LIM,
                          ik.PITCH_LIM, ik.PITCH_LIM)
                assert min(limit - abs(value)
                           for value, limit in zip(q, limits)) > 0.10

                sx, sy, sz = ik.shoulder(side)
                radial = math.hypot(x - sx, -sy)
                wrist_z = (z - sz) - ik.LG * math.sin(ch.DOWN)
                wrist_distance = math.hypot(radial, wrist_z)
                assert wrist_distance > ik.D_MIN_SOFT + 0.01
                assert wrist_distance < ik.L1 + ik.L2 - 0.01


def test_shelf_slots_fit_and_loaded_sweeps_do_not_cross_prior_slots():
    _, mission = _config()
    radius = float(mission['grasp_forward'])
    yaws = [float(mission[name]['stationary_place_yaw'])
            for name in mission['objects']]
    slots = [(radius * math.cos(yaw), radius * math.sin(yaw))
             for yaw in yaws]
    for x, y in slots:
        assert -0.425 < x < -0.175
        assert -0.21 < y < 0.21
    assert min(math.dist(a, b) for i, a in enumerate(slots)
               for b in slots[i + 1:]) > 0.095

    # Pick bearings are +60, 0 and -60 degrees.  Positive carry arcs end at
    # shelf headings 199.5, 180 and 158.9 degrees respectively.  Each later
    # endpoint is reached before any already occupied slot on that CCW arc.
    poses = _include_poses()
    occupied = []
    for name, target in zip(mission['objects'], yaws):
        x, y = poses[name][:2]
        start = math.atan2(y, x)
        travel = (target - start) % (2 * math.pi)
        for prior in occupied:
            prior_travel = (prior - start) % (2 * math.pi)
            assert not (1e-3 < prior_travel < travel - 1e-3)
        occupied.append(target)


def test_stationary_code_has_no_translation_or_full_mission_calls():
    source = (PACKAGE_ROOT / 'kobuki_bimanual_behavior'
              / 'mission_node.py').read_text(encoding='utf-8')
    tree = ast.parse(source)
    methods = {}
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            methods[node.name] = node

    forbidden = {'approach', 'goto', 'drive_odom', 'dock_to_shelf',
                 'flip_waypoints', 'pick_flip_place'}
    for method_name in ('run_stationary', 'stationary_pick_place'):
        called = {node.func.attr for node in ast.walk(methods[method_name])
                  if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Attribute)}
        assert not (called & forbidden), (method_name, called & forbidden)

    # The only low-level velocity command in the stationary call graph is in
    # rotate_by, and its linear component is the literal 0.0.
    rotate_calls = [node for node in ast.walk(methods['rotate_by'])
                    if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'cmd']
    assert rotate_calls
    for call in rotate_calls:
        assert isinstance(call.args[0], ast.Constant)
        assert float(call.args[0].value) == 0.0


if __name__ == '__main__':
    failures = 0
    for test_name, fn in sorted(globals().items()):
        if test_name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f'PASS {test_name}')
            except Exception as exc:  # noqa: BLE001
                failures += 1
                print(f'FAIL {test_name}: {exc}')
    raise SystemExit(1 if failures else 0)
