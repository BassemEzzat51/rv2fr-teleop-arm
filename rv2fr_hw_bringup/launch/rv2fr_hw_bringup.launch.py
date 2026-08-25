"""Single entry point for the RV-2FR that pairs melfa_rv2fr_moveit_config's MoveIt2 +
RViz2 with a choice of hardware backend:
  - fake hardware (default): melfa_bringup/launch/rv2fr_control.launch.py with
    use_fake_hardware:=true (mock_components/GenericSystem mirrors commanded positions
    back as state -- no physics, no real timing, no Gazebo).
  - real/RT-ToolBox3-simulated hardware: same melfa_bringup launch with
    use_fake_hardware:=false and robot_ip/controller_type set.
  - Gazebo: rv2fr_gz_bringup/launch/rv2fr_gz_sim.launch.py (use_gz_sim:=true) --
    reuses that package's already-verified gz_ros2_control/GazeboSimSystem overlay
    rather than melfa_bringup's own use_sim path, which targets the old Ignition-era
    ign_ros2_control plugin name and was never fixed for this (see
    rv2fr_gz_bringup/launch/rv2fr_gz_sim.launch.py's own module docstring). MoveIt
    connects to it exactly the same way as the fake-hardware/real backends: both
    expose the same 'rv2fr_controller'/'joint_state_broadcaster' controller and joint
    names, so melfa_rv2fr_moveit_config's config works unmodified against either.

All three backends are staggered against the same MoveIt/RViz include via TimerAction
so it comes up once the controllers are active. rv2fr_teleop_gui (start_gui:=true by
default) is layered on top the same way, staggered further out so servo_node's
services are ready first -- it works against any backend, though its camera/
object-tracking panels only show anything when use_gz_sim:=true (no eye_to_hand_camera
or rv2fr_perception exist on the fake/real-hardware backends).

See docs/decisions.md and docs/real_hardware_roadmap.md for the three vendored-package
version-drift bugs (robot_description YAML parsing, fake_components/GenericSystem ->
mock_components/GenericSystem rename, servo_node_main -> servo_node rename) that had to
be fixed in melfa_ros2_driver for the fake-hardware path to work on this Jazzy install,
and for the RViz2/snap-core20 GTK env-var workaround this launch file applies below.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # RViz2 crashes with a "libpthread.so.0: undefined symbol: __libc_pthread_init"
    # error when GTK_PATH/GTK_EXE_PREFIX/GTK_MODULES have leaked into the shell from a
    # snap-packaged app (e.g. VS Code) -- not a bug in this package. Mutating
    # os.environ directly here (not a launch.actions-based approach) because
    # generate_launch_description() runs synchronously before any entity is executed,
    # guaranteeing this applies before RViz2 (or anything else) spawns. Same pattern
    # already used in rv2fr_gz_bringup/launch/rv2fr_gz_sim.launch.py for a different
    # env var, for the same reason. See docs/decisions.md.
    for var in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_MODULES'):
        os.environ.pop(var, None)

    use_gz_sim = LaunchConfiguration('use_gz_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    controller_type = LaunchConfiguration('controller_type')
    packet_lost_log = LaunchConfiguration('packet_lost_log')
    start_rviz = LaunchConfiguration('start_rviz')
    gz_args = LaunchConfiguration('gz_args')
    start_gui = LaunchConfiguration('start_gui')

    # Both rv2fr_control.launch.py and rv2fr_moveit.launch.py independently declare
    # their own 'start_rviz' argument (each controls a different RViz instance -- a
    # bare one there, one with the MoveIt panel here). ROS2 launch configurations are
    # a single flat global namespace across IncludeLaunchDescriptions, not scoped per
    # include: whichever DeclareLaunchArgument claims a name FIRST wins for the rest
    # of the launch session, and later same-named declarations (even ones passed an
    # explicit override value) silently keep the already-set value instead of
    # replacing it. Confirmed live two different ways: passing 'start_rviz':'false' to
    # control_launch made RViz never start even with start_rviz:=true on the command
    # line (control_launch's declaration ran first and won); wrapping control_launch in
    # GroupAction(scoped=True) to isolate that override instead broke an unrelated
    # deferred controller spawner inside rv2fr_control.launch.py, which references a
    # DIFFERENT launch configuration ('robot_controller') from inside an
    # OnProcessExit-triggered event handler that runs after the scope's push/pop
    # already unwound. Simplest robust fix: don't fight the shared namespace -- this
    # file's own 'start_rviz' declaration below runs first and wins, so it's the one
    # controlling both RViz instances together. When true, melfa_bringup's own bare
    # RViz (no MotionPlanning panel) also opens alongside the MoveIt one below -- a
    # redundant extra window, not a functional problem. See docs/decisions.md.
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('melfa_bringup'), 'launch', 'rv2fr_control.launch.py'])
        ),
        condition=UnlessCondition(use_gz_sim),
        launch_arguments=[
            ('use_fake_hardware', use_fake_hardware),
            ('robot_ip', robot_ip),
            ('robot_port', robot_port),
            ('controller_type', controller_type),
            ('packet_lost_log', packet_lost_log),
        ],
    )

    # rv2fr_gz_bringup's own overlay, not melfa_bringup's use_sim path -- see module
    # docstring above for why. Exposes the same 'rv2fr_controller'/
    # 'joint_state_broadcaster' names as the fake/real-hardware backends, so
    # moveit_launch below needs no changes to work against whichever backend is active.
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rv2fr_gz_bringup'), 'launch', 'rv2fr_gz_sim.launch.py'])
        ),
        condition=IfCondition(use_gz_sim),
        launch_arguments=[('gz_args', gz_args)],
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('melfa_rv2fr_moveit_config'), 'launch', 'rv2fr_moveit.launch.py'])
        ),
        launch_arguments=[('start_rviz', start_rviz)],
    )

    # rv2fr_teleop_gui works against any backend (it only touches
    # /rv2fr_controller/joint_trajectory, controller_manager, and /servo_node/... --
    # all present regardless of which backend is active). Its joint-limit discovery
    # (urdf_joints.py) is hardcoded to read rv2fr_gz_bringup's own xacro overlay
    # specifically, not melfa_description's -- harmless here since both share the same
    # underlying rv2fr_macro.xacro joint limits, but it does mean rv2fr_gz_bringup must
    # be installed regardless of backend (already true: it's this package's own
    # gz_sim_launch dependency). The camera/object-tracking panels will simply show
    # nothing on the fake-hardware/real-hardware backends, since there's no
    # eye_to_hand_camera or rv2fr_perception running there -- not an error, just an
    # empty panel.
    gui_node = Node(
        package='rv2fr_teleop_gui',
        executable='teleop_gui',
        output='screen',
        condition=IfCondition(start_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gz_sim', default_value='false',
            description='true: drive the Gazebo-simulated arm via rv2fr_gz_bringup instead of '
                        'melfa_bringup\'s fake/real-hardware path. Overrides use_fake_hardware.',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='true',
            description='true: mock_components/GenericSystem loopback (no Gazebo, no real controller needed). '
                        'false: connect to a real MELFA controller or RT ToolBox3 Simulator at robot_ip. '
                        'Ignored when use_gz_sim:=true.',
        ),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.20', description='Real/simulated controller IP (only used when use_fake_hardware:=false and use_gz_sim:=false).'),
        DeclareLaunchArgument('robot_port', default_value='10000'),
        DeclareLaunchArgument('controller_type', default_value='R', description='MELFA controller type: R, Q, or D.'),
        DeclareLaunchArgument('packet_lost_log', default_value='1', description='DO NOT disable when using a real robot.'),
        DeclareLaunchArgument('start_rviz', default_value='true', description='Start RViz2 with the MoveIt Motion Planning panel.'),
        DeclareLaunchArgument('gz_args', default_value='', description='Extra gz sim args (e.g. "-s" for headless). Only used when use_gz_sim:=true.'),
        DeclareLaunchArgument('start_gui', default_value='true', description='Start rv2fr_teleop_gui.'),
        control_launch,
        gz_sim_launch,
        # Controllers take a few seconds to load/configure/activate (confirmed
        # empirically this session for the fake-hardware path; Gazebo takes a bit
        # longer to spawn the world/entity/controllers, matching the 8s delay
        # rv2fr_full_system.launch.py already uses for its own moveit_servo include --
        # 8s here covers both backends comfortably).
        TimerAction(period=8.0, actions=[moveit_launch]),
        # GUI needs servo_node's services (switch_command_type etc.) ready for its
        # Cartesian-jog/object-tracking controller-switch logic, which only exist once
        # moveit_launch's servo_node has finished initializing -- same 5s gap after
        # moveit_launch used by rv2fr_full_system.launch.py for its own GUI include.
        TimerAction(period=13.0, actions=[gui_node]),
    ])
