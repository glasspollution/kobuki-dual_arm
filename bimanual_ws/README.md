# Kobuki Bimanual Manipulation — ROS 2 Jazzy + Gazebo Harmonic

Dual 4-DOF arms on a Kobuki-class base. The robot scans the room, finds
fallen color-coded objects, drives to them, grasps them with both grippers,
rotates them upright mid-air, and places them on a shelf — repeating for
every fallen object it saw.

## Rotation-only diagnostic (start here)

Use this baseline before the mobile/reorientation mission:

```bash
ros2 launch kobuki_bimanual_gazebo stationary.launch.py
```

It uses a dedicated compact world and does exactly this:

1. Keep both wheels at zero linear velocity and rotate once to detect all
   three color-coded objects.
2. Rotate to and center each object; the objects are already at the fixed
   0.30 m grasp radius, so there is no approach or navigation.
3. Descend vertically, require arm convergence, attach the grasp constraint,
   and lift through a dense straight-up waypoint path.
4. Rotate in place to a free shelf slot without changing wrist pitch, lower
   vertically, require a confirmed release, clear vertically, and repeat.

The objects deliberately remain lying down on the shelf.  This removes
navigation, LiDAR docking, in-air reorientation and object-pose manipulation
from the test.  A run is only reported complete after all three confirmed
attach/release cycles succeed; failures stop at the named stage.

The compact scene also pitches the camera 45 degrees down.  The original
15-degree view cannot see the floor inside the arms' shared reach envelope.

This is a simulation diagnostic, not yet a hardware-safe grasp.  The original
robot model does not enable self-collision, and the centred bimanual hand/arm
meshes overlap during the low grasp.  Resolve that geometry (or use a single
arm) before transferring this baseline to collision-enabled planning or real
hardware.

Pipeline (matches the design sheet):
Full mission pipeline:
`Perception -> Approach -> Align -> IK Grasp -> Co-manipulate -> Reorient -> Place`

## Packages

| package | contents |
|---|---|
| `kobuki_bimanual_description` | URDF/Xacro: base, mast + sensors, 2x 4-DOF arms + grippers, ros2_control, Gazebo plugins, `controllers.yaml` |
| `kobuki_bimanual_gazebo` | worlds (`stationary`, `main`, `bottle`, `bowl`, `box`), object + shelf models, launch files |
| `kobuki_bimanual_behavior` | analytic IK, bimanual choreography, perception node, mission state machine, parameters |

## Requirements

- Pop!\_OS **24.04** (Ubuntu Noble base — required for ROS 2 Jazzy debs)
- ROS 2 Jazzy + Gazebo Harmonic (Harmonic is the default Gazebo paired
  with Jazzy's `ros_gz`)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  python3-opencv python3-numpy \
  python3-colcon-common-extensions python3-rosdep
```

## Build

```bash
cd ~/bimanual_ws            # wherever you copied this workspace
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # optional safety net
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# recommended diagnostic: rotation only, fixed-range vertical pick/place
ros2 launch kobuki_bimanual_gazebo stationary.launch.py

# full demo: 3 fallen objects, successive pick -> reorient -> place
ros2 launch kobuki_bimanual_gazebo sim.launch.py

# single-object worlds
ros2 launch kobuki_bimanual_gazebo sim.launch.py world:=bottle
ros2 launch kobuki_bimanual_gazebo sim.launch.py world:=bowl
ros2 launch kobuki_bimanual_gazebo sim.launch.py world:=box

# simulation only (drive/test manually, no autonomous mission)
ros2 launch kobuki_bimanual_gazebo sim.launch.py autostart:=false
```

What you should see, in order:

1. Gazebo opens; robot at the origin, shelf behind it, objects lying ahead.
2. Mission node releases the startup grasp joints, opens grippers, tucks
   both arms, then does one full survey rotation logging which objects it saw.
3. Per object (bottle -> bowl -> box): rotate to reacquire, center, drive in,
   final blind creep on odometry, both grippers descend, fingers close,
   grasp attach, two-phase lift + 90° flip to upright, drive to the shelf,
   lower, release, retract, back away, tuck. Then the next object.
4. `MISSION COMPLETE` in the console when done.

## Tuning (config/mission_params.yaml in the behavior package)

| symptom | knob |
|---|---|
| object not detected | `hsv_lo/hsv_hi` per object, `min_area` |
| stops too far / too close before grasping | `grasp_forward`, `approach_stop_range` |
| fingers hit or miss the object | per-object `lying_z`, `s`, `close` |
| object placed too high/low on shelf | per-object `z_place` |
| robot parks badly at the shelf | `shelf_approach` (odom frame; world-dependent) |
| drives too fast/slow | `v_lin`, `w_rot`, `scan_w` |

Arm geometry constants live in `ik.py` + `robot.urdf.xacro` and must stay in
sync (shoulder at x 0.095, y ±0.08875, z 0.18; L1 = L2 = 0.16; grasp point
0.07 beyond the wrist).

## Design assumptions (deliberate, documented for the report)

- **LiDAR at 0.46 m sees structure, not floor objects.** A horizontal 2D
  scan plane physically cannot intersect 5–10 cm tall lying objects from any
  mast height, so the camera does object detection *and* ranging
  (ground-plane back-projection); the LiDAR provides the forward safety
  guard and room structure. Mast height itself is not critical and can be
  changed in `robot.urdf.xacro` (`lidar_z`).
- Objects spawn with their long axis pointing at the robot start area, so
  arriving head-on leaves the axis aligned with the grasp choreography.
  Repositioning around an arbitrarily-oriented object is future work.
- The grasp is a `DetachableJoint` (rigid constraint, per the design sheet)
  rather than friction-only contact; fingers close to a light-touch fit.
  Gazebo Sim 8 starts these joints detached and does not publish an initial
  state sample; the mission seeds that known state, then requires feedback
  for every actual attach and release transition.
- Shelf pose is a parameter in the odom frame (odom == world at spawn);
  the tall shelf back-board makes it LiDAR-visible for future localization.
- The bowl skips the upright check (a cylinder bowl's silhouette is
  ambiguous); bottle and box classify lying-vs-upright by apparent height.
- Masses/inertias/limits are placeholders consistent with Dynamixel
  XM-class servos (±150° yaw, ±115° pitch, 4–6 N·m efforts).

## Troubleshooting

- **No camera images / `ros2 topic hz /camera` silent:** check gz-side names
  with `gz topic -l`; if the image is not on `/camera`, adjust the bridge
  arguments in `sim.launch.py` accordingly.
- **Three `DetachableJoint` errors in single-object worlds:** expected —
  plugins for absent objects log an error and stay inert.
- **Controllers fail to spawn:** confirm `gz_ros2_control` installed;
  `ros2 control list_controllers` should show 4 active controllers.
- **Black/empty Gazebo window on VMs:** try `export LIBGL_ALWAYS_SOFTWARE=1`.
- **Objects twitch when grasped:** increase finger `close` value slightly
  (less squeeze) — the detachable joint carries the object, not friction.

## Verifying the math without Gazebo

```bash
cd src/kobuki_bimanual_behavior
python3 test/test_ik.py      # FK/IK round-trip + full choreography check
python3 test/test_stationary.py  # compact-world + vertical-path contract
```
