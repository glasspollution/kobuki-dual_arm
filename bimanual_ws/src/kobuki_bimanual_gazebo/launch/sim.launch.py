"""Bring up the full dual-arm Kobuki simulation.

Usage:
    ros2 launch kobuki_bimanual_gazebo sim.launch.py                 # main world, 3 objects
    ros2 launch kobuki_bimanual_gazebo sim.launch.py world:=bottle   # single-object worlds:
    ros2 launch kobuki_bimanual_gazebo sim.launch.py world:=bowl     #   bottle | bowl | box
    ros2 launch kobuki_bimanual_gazebo sim.launch.py autostart:=false  # sim only, no mission

For the rotation-only diagnostic use ``stationary.launch.py``; it supplies a
compact world, matching camera pitch and stationary mission parameters.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('kobuki_bimanual_gazebo')
    pkg_desc = get_package_share_directory('kobuki_bimanual_description')
    pkg_behavior = get_package_share_directory('kobuki_bimanual_behavior')

    world_arg = DeclareLaunchArgument(
        'world', default_value='main',
        description='World to load: main | stationary | bottle | bowl | box')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Start the pick-reorient-place mission automatically')
    mission_mode_arg = DeclareLaunchArgument(
        'mission_mode', default_value='full',
        description='Mission implementation: full | stationary')
    camera_pitch_arg = DeclareLaunchArgument(
        'camera_pitch', default_value='0.2618',
        description='Physical camera down-pitch in radians')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_behavior, 'config',
                                   'mission_params.yaml'),
        description='Behavior parameter YAML')

    world = LaunchConfiguration('world')
    mission_mode = LaunchConfiguration('mission_mode')
    camera_pitch = LaunchConfiguration('camera_pitch')
    params_file = LaunchConfiguration('params_file')
    world_path = PythonExpression(
        ["'", os.path.join(pkg_gazebo, 'worlds'), "/' + '", world, "' + '.sdf'"])

    # Gazebo finds model:// URIs in our models dir
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_gazebo, 'models'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v3 ', world_path]}.items())

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro'),
                 ' cam_pitch:=', camera_pitch]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}])

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'kobuki_bimanual',
                   '-x', '0', '-y', '0', '-z', '0.005'])

    grasp_bridges = []
    for obj in ('bottle', 'bowl', 'cracker_box'):
        grasp_bridges += [
            f'/grasp/{obj}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/grasp/{obj}/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            f'/grasp/{obj}/state@std_msgs/msg/String[gz.msgs.StringMsg',
        ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf_gz@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ] + grasp_bridges,
        remappings=[('/tf_gz', '/tf')],
        parameters=[{'use_sim_time': True}])

    # ---- controllers: STRICTLY SERIAL chain. Parallel spawners contend for
    # the controller_manager lock (seen in testing: left arm failed 3 load
    # attempts, right arm hit lock timeouts), and the mission must not start
    # until every controller is active. spawn -> jsb -> left -> right ->
    # gripper -> behavior.
    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    output='screen', arguments=[name])

    jsb_spawner = spawner('joint_state_broadcaster')
    left_spawner = spawner('left_arm_controller')
    right_spawner = spawner('right_arm_controller')
    gripper_spawner = spawner('gripper_controller')

    chain = [
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=[jsb_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner, on_exit=[left_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=left_spawner, on_exit=[right_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=right_spawner, on_exit=[gripper_spawner])),
    ]

    # ---- behavior (perception + mission) ----
    perception = Node(
        package='kobuki_bimanual_behavior',
        executable='perception_node',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': True,
                     'cam_pitch': ParameterValue(camera_pitch,
                                                   value_type=float)}])

    mission = Node(
        package='kobuki_bimanual_behavior',
        executable='mission_node',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': True,
                     'autostart': LaunchConfiguration('autostart'),
                     'mission_mode': mission_mode}])

    behavior_after_controllers = RegisterEventHandler(
        OnProcessExit(target_action=gripper_spawner,
                      on_exit=[TimerAction(period=3.0,
                                           actions=[perception, mission])]))

    return LaunchDescription([
        world_arg,
        autostart_arg,
        mission_mode_arg,
        camera_pitch_arg,
        params_file_arg,
        set_resource_path,
        gz_sim,
        rsp,
        spawn_robot,
        bridge,
        *chain,
        behavior_after_controllers,
    ])
