"""Single entry point for the whole RV-2FR research platform: sim, MoveIt2 (move_group +
moveit_servo), perception, and the teleop GUI.

Brings up the same pieces documented in each package's own README as a multi-terminal
workflow (rv2fr_gz_bringup's sim, rv2fr_gz_bringup's moveit_servo launch,
rv2fr_perception, rv2fr_teleop_gui) as one launch tree, for convenience once each piece
has already been verified independently (see docs/decisions.md's per-phase entries).
Individual launch files still exist and work standalone -- this doesn't replace them,
it composes them.

The robot has no gripper and no pick-and-place baseline (removed, see
docs/decisions.md) -- the GUI is the primary way of controlling the arm.

Staggering via TimerAction rather than chained OnProcessExit/OnProcessStart event
handlers: the sim's own controller bring-up (rv2fr_gz_sim.launch.py) already does the
latter internally for the entities *it* owns (spawn -> joint_state_broadcaster ->
rv2fr_controller), but reaching into a nested IncludeLaunchDescription's own internal
entities from the outside to hook a "controllers are active" event isn't something
launch_ros exposes cleanly. Delays below are empirically sized against this workspace's
actual startup logs (sim+controllers settle within ~5s, move_group+servo within another
~5-6s), with slack -- not exact, but the downstream nodes (perception, the GUI) all
connect to their ROS interfaces lazily/async rather than requiring them at process
start, so arriving a little early just means the first couple of seconds show "waiting
for..." rather than failing outright.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    gz_args = LaunchConfiguration('gz_args')
    start_rviz = LaunchConfiguration('start_rviz')
    start_gui = LaunchConfiguration('start_gui')
    start_perception = LaunchConfiguration('start_perception')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rv2fr_gz_bringup'), 'launch', 'rv2fr_gz_sim.launch.py'])
        ),
        launch_arguments=[('use_sim_time', use_sim_time), ('prefix', prefix), ('gz_args', gz_args)],
    )

    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rv2fr_gz_bringup'), 'launch', 'rv2fr_moveit_servo.launch.py'])
        ),
        launch_arguments=[('use_sim_time', use_sim_time), ('start_rviz', start_rviz)],
    )

    perception_node = Node(
        package='rv2fr_perception',
        executable='color_pose_estimator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_perception),
    )

    gui_node = Node(
        package='rv2fr_teleop_gui',
        executable='teleop_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument(
            'gz_args', default_value='',
            description="Extra gz sim args, e.g. '-s' for headless. See rv2fr_gz_sim.launch.py.",
        ),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('start_gui', default_value='true', description='Start rv2fr_teleop_gui'),
        DeclareLaunchArgument('start_perception', default_value='true', description='Start rv2fr_perception'),
        sim_launch,
        TimerAction(period=8.0, actions=[moveit_servo_launch]),
        TimerAction(period=8.0, actions=[perception_node]),
        TimerAction(period=13.0, actions=[gui_node]),
    ])
