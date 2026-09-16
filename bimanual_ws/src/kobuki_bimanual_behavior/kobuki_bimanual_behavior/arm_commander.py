"""Builders that turn cartesian waypoint pairs into JointTrajectory msgs."""
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import ik

LEFT_JOINTS = ['left_j1', 'left_j2', 'left_j3', 'left_j4']
RIGHT_JOINTS = ['right_j1', 'right_j2', 'right_j3', 'right_j4']


def _duration(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int((t - sec) * 1e9))


def _point(q, t):
    pt = JointTrajectoryPoint()
    pt.positions = [float(v) for v in q]
    pt.time_from_start = _duration(t)
    return pt


def joint_traj(names, q_list, dt, t0=1.0):
    """Joint-space trajectory through q_list, dt seconds apart."""
    msg = JointTrajectory()
    msg.joint_names = list(names)
    t = t0
    for q in q_list:
        msg.points.append(_point(q, t))
        t += dt
    return msg


def pair_trajs(waypoint_pairs, dt, t0=1.0):
    """IK-solve [(left_wp, right_wp), ...] into two synced trajectories.

    left_wp/right_wp are (x, y, z, pitch) grasp-point targets in base_link.
    Raises ik.IKError if any waypoint is unreachable.
    """
    q_left = [ik.solve(*wl, +1) for wl, _ in waypoint_pairs]
    q_right = [ik.solve(*wr, -1) for _, wr in waypoint_pairs]
    return (joint_traj(LEFT_JOINTS, q_left, dt, t0),
            joint_traj(RIGHT_JOINTS, q_right, dt, t0))


def final_positions(traj):
    return dict(zip(traj.joint_names, traj.points[-1].positions))


def total_time(traj):
    tf = traj.points[-1].time_from_start
    return tf.sec + tf.nanosec * 1e-9
