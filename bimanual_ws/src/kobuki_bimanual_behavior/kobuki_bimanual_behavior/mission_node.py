"""Mission state machines for the bimanual Gazebo demonstrations.

The default ``full`` mode keeps the original mobile pick/reorient/place
mission.  The ``stationary`` mode is a deliberately smaller diagnostic:
scan by yawing in place, grasp each nearby object, lift it vertically without
changing wrist pitch, yaw to a shelf slot, and lower it vertically.  It never
commands base translation.

Phases per object (matches the design sheet):
  1 SEARCH   rotate in place until the object's color blob is stable
  2 APPROACH visual servo toward it; below the camera's near blind zone,
             dead-reckon the last metres on odometry
  3 ALIGN    keep the blob centered while approaching (folded into 2)
  4 IK GRASP both grippers descend on the lying object, fingers close,
             a DetachableJoint pins the object to the left palm
  5 CO-MANIP synced two-arm trajectories keep the rigid grasp constraint
  6 REORIENT two-phase lift + 90 deg pitch flip to upright
  7 PLACE    navigate to the shelf, lower, release, retract, back away
"""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory
from rclpy.node import Node

from . import arm_commander as ac
from . import choreography as ch
from . import ik


def norm_ang(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


DEFAULT_OBJECTS = {
    'bottle': dict(half_len=0.10, lying_z=0.031, s=0.050, close=0.037,
                   grasp_z_off=0.010, z_place=0.225, place_y_off=0.0,
                   upright_h=0.20, check_upright=True,
                   stationary_place_z=0.211,
                   stationary_place_yaw=-2.801756,
                   stationary_clear_yaw=-1.754558,
                   stationary_clear_direction=1.0),
    'bowl': dict(half_len=0.05, lying_z=0.041, s=0.035, close=0.047,
                 grasp_z_off=0.010, z_place=0.175, place_y_off=0.10,
                 upright_h=0.10, check_upright=False,
                 stationary_place_z=0.221,
                 stationary_place_yaw=3.14159,
                 stationary_clear_yaw=2.0944,
                 stationary_clear_direction=-1.0),
    'cracker_box': dict(half_len=0.09, lying_z=0.031, s=0.050, close=0.047,
                        grasp_z_off=0.010, z_place=0.215, place_y_off=-0.10,
                        upright_h=0.18, check_upright=True,
                        stationary_place_z=0.211,
                        stationary_place_yaw=2.773325,
                        stationary_clear_yaw=1.726127,
                        stationary_clear_direction=-1.0),
}

ARM_JOINTS = ['left_j1', 'left_j2', 'left_j3', 'left_j4',
              'right_j1', 'right_j2', 'right_j3', 'right_j4',
              'left_finger_l_joint', 'left_finger_r_joint',
              'right_finger_l_joint', 'right_finger_r_joint']


class MissionNode(Node):

    def __init__(self):
        super().__init__(
            'mission_node',
            automatically_declare_parameters_from_overrides=True)

        def p(name, default):
            try:
                v = self.get_parameter(name).value
                return default if v is None else v
            except Exception:  # noqa: BLE001
                return default

        self.objects = list(p('objects', list(DEFAULT_OBJECTS)))
        self.obj = {}
        for name in self.objects:
            d = dict(DEFAULT_OBJECTS.get(name, {}))
            for k in ('half_len', 'lying_z', 's', 'close', 'grasp_z_off',
                      'z_place', 'place_y_off', 'upright_h',
                      'stationary_place_z', 'stationary_place_yaw',
                      'stationary_clear_yaw',
                      'stationary_clear_direction'):
                d[k] = float(p(f'{name}.{k}', d.get(k, 0.0)))
            d['check_upright'] = bool(p(f'{name}.check_upright',
                                        d.get('check_upright', False)))
            self.obj[name] = d

        self.grasp_forward = float(p('grasp_forward', ch.GRASP_FORWARD))
        self.mission_mode = str(p('mission_mode', 'full')).strip().lower()
        self.interface_timeout = float(p('interface_timeout', 45.0))
        self.stationary_scan_angle = float(
            p('stationary_scan_angle', 2 * math.pi + 0.3))
        self.stationary_lift_z = float(p('stationary_lift_z', 0.270))
        self.stationary_range_tolerance = float(
            p('stationary_range_tolerance', 0.08))
        self.stationary_vertical_steps = int(
            p('stationary_vertical_steps', 7))
        self.stop_gate = float(p('approach_stop_range', 0.55))
        self.shelf_pose = [float(v) for v in
                           p('shelf_approach', [-0.83, 0.0, math.pi])]
        self.scan_hub = [float(v) for v in p('scan_hub', [0.0, 0.0])]
        self.backup_dist = float(p('backup_dist', 0.6))
        self.open_pos = float(p('open_pos', 0.062))
        self.v_lin = float(p('v_lin', 0.22))
        self.w_rot = float(p('w_rot', 0.5))
        self.scan_w = float(p('scan_w', 0.4))
        self.front_guard = float(p('front_guard', 0.22))
        # Real clearance wanted between the BUMPER and an obstacle. goto()
        # converts this to a raw LiDAR range by adding the 0.2775 m offset
        # between the scanner and the front of the base.
        self.bumper_clear = float(p('bumper_clear', 0.15))
        autostart = bool(p('autostart', True))

        # ---- interfaces ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.left_pub = self.create_publisher(
            JointTrajectory, '/left_arm_controller/joint_trajectory', 10)
        self.right_pub = self.create_publisher(
            JointTrajectory, '/right_arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)
        self.attach_pub = {}
        self.detach_pub = {}
        for name in self.objects:
            self.attach_pub[name] = self.create_publisher(
                Empty, f'/grasp/{name}/attach', 10)
            self.detach_pub[name] = self.create_publisher(
                Empty, f'/grasp/{name}/detach', 10)

        self.det = {}
        self.det_stamp = 0.0
        self.odom = None            # (x, y, yaw)
        self.odom_stamp = 0.0
        self.joints = {}
        self.front_min = 99.0
        self.scan = None
        self.shelf_stop_range = float(p('shelf_stop_range', 0.594))
        # Gazebo Sim 8's DetachableJoint starts detached and does not publish
        # an initial state sample.  Seed that real initial state so startup
        # does not spend several seconds waiting for a transition that cannot
        # occur; later attach/detach operations still require feedback.
        self.grasp_state = {name: 'detached' for name in self.objects}

        for name in self.objects:
            self.create_subscription(
                String, f'/grasp/{name}/state',
                lambda msg, n=name: self.grasp_state.__setitem__(
                    n, msg.data.strip().lower()), 10)

        self.create_subscription(String, '/detections', self._on_det, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.create_subscription(JointState, '/joint_states', self._on_js, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 5)

        if autostart:
            self.worker = threading.Thread(target=self._safe_run, daemon=True)
            self.worker.start()
        else:
            self.get_logger().info('autostart=false: mission idle')

    # ------------- callbacks -------------
    def _on_det(self, msg):
        try:
            self.det = json.loads(msg.data)
            self.det_stamp = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.odom_stamp = time.monotonic()

    def _on_js(self, msg):
        for n, v in zip(msg.name, msg.position):
            self.joints[n] = v

    def _on_scan(self, msg):
        self.scan = msg
        n = len(msg.ranges)
        if n == 0:
            return
        # The LiDAR spans angle_min = -pi to angle_max = +pi, so index 0 is
        # DIRECTLY BEHIND the robot and forward (angle 0) is the MIDDLE index.
        # Sampling ranges[:k] + ranges[-k:] therefore watched the rear, which
        # is why the forward guard never once fired. Sample around n//2.
        sector = n // 24                      # ~+/-15 deg around forward
        mid = n // 2
        vals = [r for r in msg.ranges[max(0, mid - sector):mid + sector + 1]
                if msg.range_min < r < msg.range_max]
        self.front_min = min(vals) if vals else 99.0

    # ------------- low-level helpers -------------
    def log(self, s):
        self.get_logger().info(s)

    def cmd(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def stop(self):
        self.cmd(0.0, 0.0)

    def fresh(self, name):
        if time.monotonic() - self.det_stamp > 0.6:
            return None
        d = self.det.get(name)
        return d if d and d.get('found') else None

    def grippers(self, left, right=None, settle=1.2):
        """Command finger positions. The controller drives all four finger
        joints from one array, so left and right MUST be sent together - this
        wrapper keeps them independently addressable. right defaults to left.
        """
        if right is None:
            right = left
        self.grip_pub.publish(Float64MultiArray(
            data=[float(left), float(left), float(right), float(right)]))
        time.sleep(settle)

    def set_grasp(self, name, attach, tries=6):
        """Command the DetachableJoint and WAIT for its state topic to
        confirm. A single dropped Empty message otherwise leaves the object
        silently welded (or loose) and poisons everything downstream."""
        pub = self.attach_pub[name] if attach else self.detach_pub[name]
        want = 'attached' if attach else 'detached'
        for _ in range(tries):
            pub.publish(Empty())
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.6:
                if self.grasp_state.get(name) == want:
                    return True
                time.sleep(0.05)
        self.log(f'warn: [{name}] grasp "{want}" not confirmed by plugin')
        return False

    def send_pair(self, traj_l, traj_r, extra_wait=6.0):
        self.left_pub.publish(traj_l)
        self.right_pub.publish(traj_r)
        t_end = max(ac.total_time(traj_l), ac.total_time(traj_r))
        target = {}
        target.update(ac.final_positions(traj_l))
        target.update(ac.final_positions(traj_r))
        if self._wait_for_joint_targets(target, t_end, extra_wait):
            return True
        # Name the offenders: which joints are stuck, and by how much. Without
        # this a non-convergence is unactionable.
        errs = sorted(((abs(self.joints.get(j, 99.0) - v), j) for j, v in
                       target.items()), reverse=True)
        detail = ', '.join(f'{j} off {e:.3f}' for e, j in errs[:4])
        self.log(f'warn: arm trajectory did not fully converge - {detail}')
        return False

    def send_single(self, publisher, traj, extra_wait=6.0):
        """Publish one arm trajectory and verify its final joint state."""
        publisher.publish(traj)
        t_end = ac.total_time(traj)
        target = ac.final_positions(traj)
        if self._wait_for_joint_targets(target, t_end, extra_wait):
            return True
        detail = ', '.join(
            f'{j} off {abs(self.joints.get(j, 99.0) - v):.3f}'
            for j, v in target.items())
        self.log(f'warn: single-arm trajectory did not converge - {detail}')
        return False

    def _wait_for_joint_targets(self, target, trajectory_time, extra_wait):
        """Wait in ROS time, with a generous wall-time deadlock guard.

        Controllers execute trajectories in simulation time.  A wall-time
        deadline falsely declares failure whenever Gazebo runs below real
        time, then a retry interrupts a trajectory that is still valid.
        """
        start_ros = self.get_clock().now().nanoseconds
        ros_budget = int((trajectory_time + extra_wait) * 1e9)
        wall_deadline = time.monotonic() + max(
            30.0, 10.0 * (trajectory_time + extra_wait))
        while time.monotonic() < wall_deadline:
            if all(abs(self.joints.get(j, 99.0) - v) < 0.08
                   for j, v in target.items()):
                return True
            if self.get_clock().now().nanoseconds - start_ros >= ros_budget:
                return False
            time.sleep(0.05)
        self.log('warn: joint wait hit wall-time guard; simulation clock '
                 'may be paused or missing')
        return False

    def move_arms(self, waypoint_pairs, dt, t0=1.5):
        tl, tr = ac.pair_trajs(waypoint_pairs, dt, t0)
        return self.send_pair(tl, tr)

    def tuck(self):
        tl = ac.joint_traj(ac.LEFT_JOINTS, [ch.TUCK_LEFT], 2.5, t0=3.0)
        tr = ac.joint_traj(ac.RIGHT_JOINTS, [ch.TUCK_RIGHT], 2.5, t0=3.0)
        return self.send_pair(tl, tr)

    def wait_odom(self):
        while self.odom is None:
            time.sleep(0.1)

    def wait_for_interfaces(self, timeout=None):
        """Bounded readiness gate used by the diagnostic mission."""
        timeout = self.interface_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.odom is not None and all(
                    j in self.joints for j in ARM_JOINTS):
                return True
            time.sleep(0.1)
        missing = [j for j in ARM_JOINTS if j not in self.joints]
        self.log('interfaces not ready before timeout; '
                 f'odom={self.odom is not None}, missing joints={missing}')
        return False

    # ------------- base motion -------------
    def rotate_by(self, delta, note_seen=None, seen=None, timeout=None):
        """Rotate delta rad (odom-integrated). Optionally record which
        objects appear while turning.  Only angular velocity is commanded.

        Returns False instead of hanging if odometry stops updating.
        """
        self.wait_odom()
        turned = 0.0
        last = self.odom[2]
        direction = 1.0 if delta >= 0 else -1.0
        if timeout is None:
            timeout = max(10.0, 3.0 * abs(delta) / max(self.scan_w, 0.05))
        start_ros = self.get_clock().now().nanoseconds
        ros_budget = int(timeout * 1e9)
        wall_deadline = time.monotonic() + max(30.0, 10.0 * timeout)
        try:
            while abs(turned) < max(0.0, abs(delta) - 0.015):
                if time.monotonic() - self.odom_stamp > 2.0:
                    self.log('rotation aborted: odometry stopped updating')
                    return False
                ros_expired = (self.get_clock().now().nanoseconds - start_ros
                               >= ros_budget)
                wall_expired = time.monotonic() >= wall_deadline
                if ros_expired or wall_expired:
                    self.log(f'rotation timed out after turning {turned:.2f} '
                             f'of {delta:.2f} rad')
                    return False
                remaining = abs(delta) - abs(turned)
                speed = min(self.scan_w, max(0.12, 1.5 * remaining))
                self.cmd(0.0, direction * speed)
                time.sleep(0.05)
                cur = self.odom[2]
                turned += norm_ang(cur - last)
                last = cur
                if note_seen is not None:
                    for name in note_seen:
                        if self.fresh(name):
                            seen.add(name)
            return True
        finally:
            self.stop()
            time.sleep(0.3)

    def rotate_to_yaw(self, target, direction=0):
        """Yaw to an absolute odometry heading without translating.

        ``direction=+1`` forces a counter-clockwise carry path.  The compact
        world uses that deterministic direction so a carried object never
        sweeps through an unprocessed object or an occupied shelf slot.
        """
        self.wait_odom()
        current = self.odom[2]
        if direction > 0:
            delta = (target - current) % (2 * math.pi)
        elif direction < 0:
            delta = -((current - target) % (2 * math.pi))
        else:
            delta = norm_ang(target - current)
        return self.rotate_by(delta)

    def rotate_until_found(self, name, timeout=40.0):
        self.wait_odom()
        t0 = time.monotonic()
        hits = 0
        while time.monotonic() - t0 < timeout:
            d = self.fresh(name)
            if d:
                hits += 1
                if hits >= 4:
                    self.stop()
                    time.sleep(0.3)
                    return True
                self.cmd(0.0, 0.0)
            else:
                hits = 0
                self.cmd(0.0, self.scan_w)
            time.sleep(0.08)
        self.stop()
        return False

    def center_on(self, name, tol=0.03, timeout=15.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            d = self.fresh(name)
            if not d:
                # Do not leave the previous steering command active while
                # waiting for vision to recover.
                self.stop()
                time.sleep(0.1)
                continue
            b = d['bearing']
            if abs(b) < tol:
                self.stop()
                return True
            self.cmd(0.0, max(-0.4, min(0.4, 1.2 * b)))
            time.sleep(0.05)
        self.stop()
        return False

    def drive_odom(self, dist, guard=True):
        """Straight drive by odometry (negative = reverse)."""
        self.wait_odom()
        x0, y0, _ = self.odom
        sgn = 1.0 if dist >= 0 else -1.0
        while True:
            x, y, _ = self.odom
            if math.hypot(x - x0, y - y0) >= abs(dist) - 0.005:
                break
            if guard and sgn > 0 and self.front_min < self.front_guard:
                self.log('front guard triggered, stopping')
                break
            self.cmd(sgn * min(self.v_lin, 0.12 + abs(dist)), 0.0)
            time.sleep(0.03)
        self.stop()
        time.sleep(0.2)

    def goto(self, gx, gy, gyaw, slow=False, guard=True):
        """Turn-drive-turn to an odometry pose.

        Returns True if the pose was reached, False if the forward collision
        guard stopped the drive. The guard is expressed as BUMPER clearance:
        front_min is measured at the LiDAR, which sits 0.1 m behind base
        centre and so 0.2775 m behind the bumper, hence the raw range limit is
        bumper_clear + 0.2775.
        """
        stop_range = self.bumper_clear + 0.2775
        v_cap = 0.14 if slow else self.v_lin
        w_cap = 0.35 if slow else 0.5
        self.wait_odom()
        x, y, yaw = self.odom
        heading = math.atan2(gy - y, gx - x)
        self.rotate_by(norm_ang(heading - yaw))
        while True:
            x, y, yaw = self.odom
            d = math.hypot(gx - x, gy - y)
            if d < 0.04:
                break
            if guard and self.front_min < stop_range:
                self.stop()
                self.log(f'goto: collision guard stopped drive - obstacle at '
                         f'{self.front_min:.2f} m (limit {stop_range:.2f} m)')
                return False
            heading = math.atan2(gy - y, gx - x)
            err = norm_ang(heading - yaw)
            if abs(err) > 0.8:            # overshot: re-aim
                self.stop()
                self.rotate_by(err)
                continue
            self.cmd(min(v_cap, 0.1 + 0.5 * d),
                     max(-w_cap, min(w_cap, 1.5 * err)))
            time.sleep(0.03)
        self.stop()
        x, y, yaw = self.odom
        self.rotate_by(norm_ang(gyaw - yaw))
        x, y, yaw = self.odom
        self.log(f'goto arrived: odom=({x:.3f}, {y:.3f}, yaw {yaw:.2f})')
        return True

    # ------------- shelf docking (closed loop) -------------
    def _board_cluster(self, max_range=1.6, half_fov=0.9):
        """Find the shelf back-board in the front LiDAR sector.

        A candidate must (a) have the physical width of the 0.40 m board,
        (b) stand out in DEPTH from its neighbors — the beams just beyond
        both cluster edges must be much farther or invalid. Wall fragments
        created by max_range clipping fail (b), so docking can never lock
        onto a wall.
        """
        s = self.scan
        if s is None:
            return None
        n = len(s.ranges)

        def rng(i):
            r = s.ranges[i % n]
            return r if s.range_min < r < s.range_max else None

        clusters = []
        cur = []
        for i in range(n):
            ang = norm_ang(s.angle_min + i * s.angle_increment)
            r = rng(i)
            good = (abs(ang) <= half_fov and r is not None and r < max_range)
            if good:
                cur.append((i, ang, r))
            elif cur:
                clusters.append(cur)
                cur = []
        if cur:
            clusters.append(cur)

        valid = []
        self._dock_reject = []
        for c in clusters:
            angs = [a for _, a, _ in c]
            rmin = min(r for _, _, r in c)
            bear = sum(angs) / len(angs) if angs else 0.0
            if len(c) < 3:
                self._dock_reject.append(
                    f'bearing {bear:+.2f} rng {rmin:.2f}: only {len(c)} beams')
                continue
            ang_width = max(angs) - min(angs)
            phys_width = 2.0 * rmin * math.tan(ang_width / 2.0)
            if not (0.15 < phys_width < 0.70):
                self._dock_reject.append(
                    f'bearing {bear:+.2f} rng {rmin:.2f}: width '
                    f'{phys_width:.2f} m outside 0.15-0.70')
                continue
            # depth contrast: neighbors beyond both edges must be far/absent
            before = rng(c[0][0] - 1)
            after = rng(c[-1][0] + 1)
            edge_r = max(c[0][2], c[-1][2])

            def stands_out(nb):
                return nb is None or nb > edge_r + 0.4
            if not (stands_out(before) and stands_out(after)):
                self._dock_reject.append(
                    f'bearing {bear:+.2f} rng {rmin:.2f} width '
                    f'{phys_width:.2f}: no depth contrast (edges '
                    f'{before} / {after} vs {edge_r + 0.4:.2f})')
                continue
            valid.append((bear, rmin))
        if not valid:
            return None
        return min(valid, key=lambda br: br[1])

    def _dock_servo(self, timeout=30.0):
        """Servo onto a validated board cluster. Returns 'docked', 'lost'
        or 'notfound'."""
        t0 = time.time()
        seen = False
        sweep_dir = 1
        swept = 0.0
        last_yaw = None
        while time.time() - t0 < timeout:
            hit = self._board_cluster()
            if hit is None:
                if seen:
                    self.stop()
                    return 'lost'
                # bounded sweep +/-40 deg around arrival heading, no free spin
                self.wait_odom()
                yaw = self.odom[2]
                if last_yaw is not None:
                    swept += norm_ang(yaw - last_yaw) * sweep_dir
                last_yaw = yaw
                if swept > 0.7:
                    sweep_dir = -sweep_dir
                    swept = -0.7
                self.cmd(0.0, 0.3 * sweep_dir)
                time.sleep(0.05)
                continue
            seen = True
            bearing, rng = hit
            err = rng - self.shelf_stop_range
            if abs(err) < 0.02 and abs(bearing) < 0.04:
                self.stop()
                self.log(f'docked at shelf: board range {rng:.3f} m, '
                         f'bearing {bearing:.3f}')
                return 'docked'
            lin = max(-0.12, min(0.15, 0.6 * err))
            ang = max(-0.4, min(0.4, 1.5 * bearing))
            self.cmd(lin, ang)
            time.sleep(0.05)
        self.stop()
        # C4.1: say WHY nothing was accepted instead of a bare "notfound".
        rej = getattr(self, '_dock_reject', [])
        if rej:
            self.log('dock: no board accepted. rejected clusters:')
            for r in rej[:6]:
                self.log('   ' + r)
        else:
            self.log('dock: no board accepted and NO clusters were formed at '
                     'all - nothing within 1.6 m in the forward 100 deg, so '
                     'either the scan is empty or the robot is not facing the '
                     'shelf')
        return 'notfound'

    def dock_to_shelf(self):
        """Closed-loop shelf docking with retries. NEVER leaves the robot at
        an arbitrary interrupted position: on any failure it returns to the
        odometry-estimated approach pose, so the worst case is a bounded
        odom-frame placement instead of a random spot."""
        for attempt in (1, 2):
            result = self._dock_servo()
            if result == 'docked':
                # sanity: docked pose must roughly agree with odometry
                x, y, _ = self.odom
                dx = math.hypot(x - self.shelf_pose[0], y - self.shelf_pose[1])
                if dx < 0.6:
                    return True
                self.log(f'warn: dock disagrees with odometry by {dx:.2f} m '
                         '- treating as phantom lock')
            self.log(f'dock attempt {attempt} failed ({result}); '
                     're-approaching by odometry')
            # clear the shelf before turning: the carried object + extended
            # arms sweep ~0.45 m and would snag the back board
            self.drive_odom(-0.35, guard=False)
            if not self.goto(*self.shelf_pose, slow=True):
                self.log('dock: re-approach stopped by the collision guard - '
                         'the shelf is closer than odometry believes')
        self.log('warn: docking failed twice; placing at odometry pose')
        return False

    def approach(self, name):
        """Visual servo until the stop gate, then odom creep to the grasp
        stand-off. Returns True when parked."""
        o = self.obj[name]
        last_center = None
        t0 = time.time()
        while time.time() - t0 < 60.0:
            d = self.fresh(name)
            if d:
                center = d['range_base'] + o['half_len']
                last_center = center
                if d['range_base'] <= self.stop_gate:
                    break
                ang = max(-0.5, min(0.5, 1.2 * d['bearing']))
                lin = self.v_lin if abs(d['bearing']) < 0.25 else 0.0
                if self.front_min < self.front_guard:
                    self.log('front guard during approach')
                    self.stop()
                    return False
                self.cmd(lin, ang)
            else:
                if last_center is not None:
                    break                  # dropped below FOV: creep blind
                self.cmd(0.0, 0.0)
            time.sleep(0.05)
        self.stop()
        if last_center is None:
            return False
        # re-center precisely while the object is still visible, then
        # refresh the range estimate for the blind creep
        if self.fresh(name):
            self.center_on(name, tol=0.015, timeout=8.0)
            d = self.fresh(name)
            if d:
                last_center = d['range_base'] + o['half_len']
        creep = last_center - self.grasp_forward
        self.log(f'creeping {creep:.3f} m by odometry')
        if creep > 0:
            self.drive_odom(creep, guard=False)
        return True

    # ------------- manipulation -------------
    def stationary_pick_place(self, name):
        """Pick and place one object with no base translation or wrist flip.

        The compact world puts every object at ``grasp_forward`` from the
        base.  After a vertical lift, a counter-clockwise base yaw carries the
        unchanged grasp to that object's shelf slot; the arms then lower on
        the same vertical line.  Any unconfirmed motion/attach/release is a
        hard failure so the baseline cannot hide the original fault.
        """
        o = self.obj[name]
        s = o['s']
        grasp_z = o['lying_z'] + o['grasp_z_off']
        lift_z = self.stationary_lift_z
        place_z = o['stationary_place_z']
        steps = self.stationary_vertical_steps

        if lift_z <= max(grasp_z, place_z):
            raise RuntimeError(
                f'[{name}] stationary_lift_z must exceed pick/place height')

        self.log(f'[{name}] stationary grasp: open, stage, descend')
        self.grippers(self.open_pos, settle=0.8)
        tl = ac.joint_traj(ac.LEFT_JOINTS, [ch.STAGE_LEFT], 2.0, t0=2.5)
        tr = ac.joint_traj(ac.RIGHT_JOINTS, [ch.STAGE_RIGHT], 2.0, t0=2.5)
        if not self.send_pair(tl, tr):
            raise RuntimeError(f'[{name}] staging did not converge')

        # Deploy one arm at a time to avoid the centreline crossing transient.
        hover = ch.grasp_waypoints(
            s, grasp_z, self.grasp_forward)[:1][0]
        hov_l = ac.joint_traj(
            ac.LEFT_JOINTS, [ik.solve(*hover[0], +1)], 2.0, t0=2.5)
        if not self.send_single(self.left_pub, hov_l):
            raise RuntimeError(f'[{name}] left hover did not converge')
        hov_r = ac.joint_traj(
            ac.RIGHT_JOINTS, [ik.solve(*hover[1], -1)], 2.0, t0=2.5)
        if not self.send_single(self.right_pub, hov_r):
            raise RuntimeError(f'[{name}] right hover did not converge')

        descend = ch.vertical_waypoints(
            s, grasp_z + ch.HOVER_DZ, grasp_z,
            center_x=self.grasp_forward, steps=steps)
        if not self.move_arms(descend, dt=0.8, t0=1.0):
            self.grippers(self.open_pos, settle=0.5)
            self.tuck()
            raise RuntimeError(
                f'[{name}] grasp descent did not converge; attach suppressed')

        # Attach before visual finger closure so contact cannot move the object
        # and cause the weld to preserve an accidental tilt.
        if not self.set_grasp(name, True):
            self.grippers(self.open_pos, settle=0.5)
            self.tuck()
            raise RuntimeError(f'[{name}] attach was not confirmed')
        self.grippers(o['close'] + 0.006, settle=0.8)

        lift = ch.vertical_waypoints(
            s, grasp_z, lift_z, center_x=self.grasp_forward, steps=steps)
        if not self.move_arms(lift, dt=0.8, t0=1.0):
            raise RuntimeError(f'[{name}] vertical lift did not converge')

        self.log(f'[{name}] lifted without reorientation; rotating to shelf')
        if not self.rotate_to_yaw(o['stationary_place_yaw'], direction=+1):
            raise RuntimeError(f'[{name}] shelf rotation did not converge')

        lower = ch.vertical_waypoints(
            s, lift_z, place_z, center_x=self.grasp_forward, steps=steps)
        if not self.move_arms(lower, dt=0.8, t0=1.0):
            raise RuntimeError(f'[{name}] vertical place did not converge')
        if not self.set_grasp(name, False):
            raise RuntimeError(f'[{name}] release was not confirmed')

        self.grippers(self.open_pos, settle=0.8)
        clear = ch.vertical_waypoints(
            s, place_z, lift_z, center_x=self.grasp_forward, steps=steps)
        if not self.move_arms(clear, dt=0.8, t0=1.0):
            raise RuntimeError(f'[{name}] post-release vertical clear failed')
        # Leave the compact shelf with the hands still high, then tuck where
        # the joint-space interpolation cannot clip the slab or placed items.
        if not self.rotate_to_yaw(
                o['stationary_clear_yaw'],
                direction=o['stationary_clear_direction']):
            raise RuntimeError(f'[{name}] post-place clearance yaw failed')
        if not self.tuck():
            raise RuntimeError(f'[{name}] tuck did not converge')
        self.log(f'[{name}] stationary pick/place complete')
        return True

    def pick_flip_place(self, name):
        o = self.obj[name]
        s, z_off = o['s'], o['grasp_z_off']
        grasp_z = o['lying_z'] + z_off      # hands grip above center-line
        place_z = o['z_place'] + z_off      # compensate at release height

        self.log(f'[{name}] grasp: descend both grippers')
        self.grippers(self.open_pos, settle=0.8)
        # staging first: keeps the tuck->grasp joint interpolation above the
        # deck instead of sweeping the wrist through the base
        tl = ac.joint_traj(ac.LEFT_JOINTS, [ch.STAGE_LEFT], 2.0, t0=2.5)
        tr = ac.joint_traj(ac.RIGHT_JOINTS, [ch.STAGE_RIGHT], 2.0, t0=2.5)
        self.send_pair(tl, tr)
        # deploy to hover STAGGERED (left, then right): both arms swing
        # toward the centerline and cross paths if moved simultaneously
        hover_pair = ch.grasp_waypoints(s, grasp_z, self.grasp_forward)[:1]
        hov_l = ac.joint_traj(ac.LEFT_JOINTS,
                              [ik.solve(*hover_pair[0][0], +1)], 2.0, t0=2.5)
        self.left_pub.publish(hov_l)
        time.sleep(3.0)
        hov_r = ac.joint_traj(ac.RIGHT_JOINTS,
                              [ik.solve(*hover_pair[0][1], -1)], 2.0, t0=2.5)
        self.right_pub.publish(hov_r)
        time.sleep(3.0)
        ok = self.move_arms(ch.grasp_waypoints(s, grasp_z,
                                               self.grasp_forward),
                            dt=2.5, t0=3.5)
        if not ok:
            # likely finger/object contact from residual misalignment:
            # reopen, lift back to hover, try the descend once more
            self.log(f'[{name}] grasp descend blocked; retrying once')
            self.grippers(self.open_pos, settle=0.6)
            hover = ch.grasp_waypoints(s, grasp_z, self.grasp_forward)[:1]
            self.move_arms(hover, dt=2.0, t0=2.5)
            ok = self.move_arms(ch.grasp_waypoints(s, grasp_z,
                                                   self.grasp_forward),
                                dt=2.5, t0=2.5)
        if not ok:
            # A DetachableJoint can attach across a gap, so continuing here
            # hides a bad IK/controller pose by welding the object remotely.
            # Abort before any attach request instead.
            self.log(f'[{name}] grasp descent did not converge twice; '
                     'attach suppressed')
            self.grippers(self.open_pos, settle=0.5)
            self.tuck()
            self.drive_odom(-0.3, guard=False)
            return False

        # ---- form the grasp -------------------------------------------
        # ORDER MATTERS. A DetachableJoint welds the object at whatever
        # relative pose exists the instant it is created. Closing the fingers
        # first pushes the object off its true lying pose, and the weld then
        # freezes that tilt for the rest of the cycle - which was the visible
        # cause of every tilted carry. So: attach FIRST, while the object is
        # still sitting undisturbed, and only then close the fingers.
        # Getting here means the descent converged, so the joint is never
        # created from an unverified arm pose.
        if not self.set_grasp(name, True):
            self.log(f'[{name}] attach not confirmed; aborting this object')
            self.grippers(self.open_pos, settle=0.5)
            self.tuck()
            self.drive_odom(-0.3, guard=False)
            return False
        # Close to a small CLEARANCE, not into contact. The object is welded
        # to the left palm, so pressing the left fingers into it is a
        # redundant constraint (weld + contact on the same pair of bodies) -
        # the solver fights itself and shoves the arm off its trajectory.
        # ~6 mm per side still reads as a closed gripper visually.
        self.grippers(o['close'] + 0.006, settle=0.8)
        self.log(f'[{name}] attached; lifting + reorienting to upright')

        # C2.2: 2.5 s per waypoint. At 1.1 s the arm could not track the
        # commanded motion and arrived at the shelf with the object at a pose
        # the IK never intended.
        # C1.4: the flip's convergence is now checked, retried and escalated -
        # previously the return value was discarded and the mission carried a
        # bad pose all the way to the shelf.
        flip_ok = self.move_arms(
            ch.flip_waypoints(s, grasp_z, self.grasp_forward), dt=2.5, t0=1.5)
        if not flip_ok:
            self.log(f'[{name}] flip did not converge; retrying once')
            flip_ok = self.move_arms(
                ch.flip_waypoints(s, grasp_z, self.grasp_forward),
                dt=3.0, t0=2.0)
        if not flip_ok:
            # NOT fatal. Aborting here costs the whole pickup, and the object
            # is usually close to upright even when the tolerance is missed.
            # Continue, but say so clearly so the placement can be judged.
            self.log(f'[{name}] WARNING: flip did not converge twice - '
                     'continuing to the shelf with an unverified pose')
        self.log(f'[{name}] upright; navigating to shelf')

        if not self.goto(*self.shelf_pose, slow=True):
            self.log(f'[{name}] shelf approach stopped early by the collision '
                     'guard; letting the LiDAR dock take over from here')
        self.dock_to_shelf()

        self.log(f'[{name}] placing on shelf (lateral offset '
                 f'{o["place_y_off"]:+.2f} m)')
        place_ok = self.move_arms(ch.place_waypoints(s, place_z,
                                                    y_off=o['place_y_off']),
                                  dt=3.0, t0=3.0)
        if not place_ok:
            self.log(f'[{name}] place motion did not converge; retrying once')
            place_ok = self.move_arms(
                ch.place_waypoints(s, place_z, y_off=o['place_y_off']),
                dt=3.5, t0=2.5)
        if not place_ok:
            self.log(f'[{name}] WARNING: releasing without confirmed place '
                     'pose - object may land short of its slot')
        released = self.set_grasp(name, False)
        if not released:
            self.log(f'[{name}] release unconfirmed, retrying once more')
            self.set_grasp(name, False)
        self.grippers(self.open_pos, settle=1.0)
        self.move_arms(ch.clear_waypoints(s, place_z,
                                          y_off=o['place_y_off']),
                       dt=2.0, t0=2.0)

        self.log(f'[{name}] released; backing away')
        self.drive_odom(-self.backup_dist, guard=False)
        self.tuck()
        return True

    # ------------- mission -------------
    def _safe_run(self):
        try:
            if self.mission_mode == 'stationary':
                self.run_stationary()
            elif self.mission_mode == 'full':
                self.run()
            else:
                raise ValueError(
                    f'unknown mission_mode {self.mission_mode!r}; '
                    'expected "full" or "stationary"')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'mission aborted: {exc!r}')
            self.stop()

    def run_stationary(self):
        """Three-object diagnostic mission with rotation-only base motion."""
        self.log('stationary mission: waiting for odom and all 12 joints')
        if not self.wait_for_interfaces():
            raise RuntimeError('stationary interface readiness timeout')
        self.log('stationary mission: controllers ready')
        time.sleep(2.0)

        for name in self.objects:
            if not self.set_grasp(name, False):
                raise RuntimeError(f'[{name}] initial detach not confirmed')
        self.grippers(self.open_pos, settle=0.5)
        if not self.tuck():
            raise RuntimeError('initial tuck did not converge')

        self.log('stationary mission: 360 degree detection scan')
        seen = set()
        if not self.rotate_by(self.stationary_scan_angle,
                              note_seen=self.objects, seen=seen):
            raise RuntimeError('survey rotation did not complete')
        missing = [name for name in self.objects if name not in seen]
        self.log(f'stationary mission: survey saw {sorted(seen)}')
        if missing:
            raise RuntimeError(
                f'survey did not detect all configured objects: {missing}')

        completed = []
        for name in self.objects:
            self.log(f'[{name}] reacquiring with rotation only')
            if not self.rotate_until_found(name, timeout=25.0):
                raise RuntimeError(f'[{name}] could not be reacquired')
            if not self.center_on(name, tol=0.02, timeout=10.0):
                raise RuntimeError(f'[{name}] could not be centered')

            det = self.fresh(name)
            if det is None:
                raise RuntimeError(f'[{name}] detection became stale')
            estimated_center = det['range_base'] + self.obj[name]['half_len']
            range_error = estimated_center - self.grasp_forward
            if abs(range_error) > self.stationary_range_tolerance:
                raise RuntimeError(
                    f'[{name}] is outside fixed grasp range: estimated center '
                    f'{estimated_center:.3f} m, expected '
                    f'{self.grasp_forward:.3f} +/- '
                    f'{self.stationary_range_tolerance:.3f} m')
            self.log(f'[{name}] fixed-range check passed '
                     f'({estimated_center:.3f} m)')
            self.stationary_pick_place(name)
            completed.append(name)

        self.stop()
        self.log('STATIONARY MISSION COMPLETE: '
                 f'{len(completed)}/{len(self.objects)} objects placed')

    def run(self):
        self.log('mission: waiting for sim interfaces...')
        self.wait_odom()
        # wait until EVERY controlled joint reports state: guards against
        # controllers that are still loading (right arm inactive would
        # otherwise silently sit out the grasp)
        while not all(j in self.joints for j in ARM_JOINTS):
            time.sleep(0.2)
        self.log('mission: all 12 joints reporting, controllers ready')
        time.sleep(2.0)

        # Normalize every grasp to detached before the mission starts.
        for name in self.objects:
            self.set_grasp(name, False)

        self.grippers(self.open_pos, settle=0.5)
        self.tuck()

        self.log('mission: survey rotation')
        seen = set()
        self.rotate_by(2 * math.pi + 0.3, note_seen=self.objects, seen=seen)
        self.log(f'mission: survey saw {sorted(seen)}')

        for name in self.objects:
            try:
                if name not in seen:
                    self.log(f'[{name}] not present, skipping')
                    continue
                # approach every object from the central hub so the arrival
                # direction matches the object's lying axis
                self.log(f'[{name}] returning to scan hub')
                hx, hy = self.scan_hub
                x, y, _ = self.odom
                if math.hypot(hx - x, hy - y) > 0.15:
                    self.goto(hx, hy, 0.0)
                self.log(f'[{name}] reacquiring')
                if not self.rotate_until_found(name):
                    self.log(f'[{name}] not visible from hub; trying a '
                             'second vantage point')
                    self.goto(0.45, 0.0, 0.0)
                    if not self.rotate_until_found(name):
                        self.log(f'[{name}] lost after survey, skipping')
                        continue
                self.center_on(name)

                d = self.fresh(name)
                o = self.obj[name]
                if (d and o['check_upright']
                        and d['est_h'] >= 0.7 * o['upright_h']):
                    self.log(f'[{name}] appears upright already '
                             f'(est_h={d["est_h"]:.3f}), skipping')
                    continue

                self.log(f'[{name}] approaching')
                if not self.approach(name):
                    self.log(f'[{name}] approach failed, skipping')
                    continue
                self.pick_flip_place(name)
            except Exception as exc:  # noqa: BLE001 - one object must not
                self.log(f'[{name}] FAILED with {exc!r}; recovering and '
                         'moving to the next object')
                self.stop()
                self.set_grasp(name, False)
                self.grippers(self.open_pos, settle=0.5)
                try:
                    self.tuck()
                except Exception:  # noqa: BLE001
                    pass
                self.drive_odom(-0.4, guard=False)

        self.stop()
        self.log('MISSION COMPLETE: all detected fallen objects processed')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
