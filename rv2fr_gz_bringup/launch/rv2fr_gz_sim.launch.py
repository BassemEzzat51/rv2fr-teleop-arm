import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)

    pkg_share = get_package_share_directory('rv2fr_gz_bringup')
    xacro_file = os.path.join(pkg_share, 'urdf', 'rv2fr_gz.urdf.xacro')

    # Expand xacro -> URDF string via subprocess rather than the launch
    # Command() substitution: Command()'s output is fed to robot_state_publisher
    # as a raw parameter string and, on this Jazzy install, launch_ros tries to
    # YAML-parse it and throws "Unable to parse the value of parameter
    # robot_description as yaml" before the node even starts. subprocess avoids
    # that parameter-typing path entirely. Same fix already applied in this
    # workspace's dual_arm/launch/dual_arm_control.launch.py.
    result = subprocess.run(
        ['xacro', xacro_file, f'prefix:={prefix}'],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f'xacro failed:\nSTDOUT: {result.stdout}\nSTDERR: {result.stderr}')

    robot_description = {
        'robot_description': result.stdout,
        'use_sim_time': use_sim_time == 'true',
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'rv2fr', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    rv2fr_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rv2fr_controller'],
    )

    # Bridge sim clock and the eye-to-hand RGB-D camera's gz-transport topics into ROS2.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/eye_to_hand_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/eye_to_hand_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/eye_to_hand_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/eye_to_hand_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    return [
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rv2fr_controller_spawner],
            )
        ),
    ]


def generate_launch_description():
    gz_args = LaunchConfiguration('gz_args', default='')
    world_path = PathJoinSubstitution(
        [FindPackageShare('rv2fr_gz_bringup'), 'worlds', 'empty.sdf']
    )

    # melfa_description's own env-hook (env-hooks/melfa_description.dsv.in) sets
    # IGN_GAZEBO_RESOURCE_PATH -- the pre-rename Ignition variable name -- so gz sim
    # (Harmonic) never finds it and mesh (model://melfa_description/meshes/...) loads
    # fail (robot spawns with no visuals). Rather than patching the vendored package,
    # set the Harmonic-correct GZ_SIM_RESOURCE_PATH ourselves here.
    #
    # This is set directly on os.environ (not via a launch.actions.AppendEnvironmentVariable
    # entity) because the latter did not reliably apply before the nested
    # IncludeLaunchDescription below reads os.environ in its own OpaqueFunction --
    # confirmed by testing (mesh load still failed with the action-based approach,
    # only worked once the var was exported before `ros2 launch` even started).
    # generate_launch_description() runs synchronously before any entity is executed,
    # so mutating os.environ directly here is guaranteed to run first. See docs/decisions.md.
    melfa_description_share = get_package_share_directory('melfa_description')
    melfa_description_share_parent = os.path.dirname(melfa_description_share)
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        p for p in [existing_resource_path, melfa_description_share_parent] if p
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='If true, use simulated clock'),
        DeclareLaunchArgument('prefix', default_value='', description='Joint/link name prefix, for multi-robot setups'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]
            ),
            # Note: if `gz sim`'s GUI crashes with a symbol lookup error against a
            # snap core20 libpthread.so.0, it's caused by GTK_PATH/GTK_EXE_PREFIX env
            # vars leaking from a snap-packaged app (e.g. VS Code) into the shell --
            # not a bug in this package. Fix: unset GTK_PATH/GTK_EXE_PREFIX/GTK_MODULES
            # before launching, or add `-s` here to run headless. See docs/decisions.md.
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 ', world_path])],
        ),
        OpaqueFunction(function=launch_setup),
    ])
