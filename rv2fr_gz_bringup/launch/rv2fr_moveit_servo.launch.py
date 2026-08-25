"""move_group + moveit_servo for the RV-2FR, wired to run against rv2fr_gz_bringup's
sim robot rather than melfa_rv2fr_moveit_config's own real-hardware-oriented default
xacro. Reuses the vendor moveit_config package's SRDF/kinematics/joint_limits/planning
pipeline/controller-mapping config as-is -- those don't depend on which hardware
interface is behind ros2_control, only on link/joint names, which are identical since
both this package's overlay and the vendor's own xacro build on the same
melfa_description geometry macro.

Must be launched against a running rv2fr_gz_bringup sim (rv2fr_gz_sim.launch.py) --
this only starts move_group + servo_node, not the sim itself, so the two can be
brought up/down independently while iterating.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    start_rviz = LaunchConfiguration('start_rviz').perform(context) == 'true'

    rv2fr_gz_bringup_share = get_package_share_directory('rv2fr_gz_bringup')
    overlay_xacro_path = os.path.join(rv2fr_gz_bringup_share, 'urdf', 'rv2fr_gz.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder('rv2fr', package_name='melfa_rv2fr_moveit_config')
        .robot_description(file_path=overlay_xacro_path)
        .robot_description_semantic(file_path='config/rv2fr.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'chomp', 'pilz_industrial_motion_planner'])
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()
    # Overrides melfa_rv2fr_moveit_config's own moveit_controllers.yaml
    # (allowed_start_tolerance: 0.05) rather than editing that vendor file. Mutating
    # the dict directly, not appending a separate {'trajectory_execution.allowed_start_
    # tolerance': ...} dict to the parameters list below: moveit_config.to_dict()
    # stores this as a NESTED dict ({'trajectory_execution': {'allowed_start_tolerance':
    # ...}}), not a flat dotted-string key, so a separate dotted-key dict is a distinct,
    # unrecognized parameter that silently does nothing -- confirmed live via
    # `ros2 param get /move_group trajectory_execution.allowed_start_tolerance` still
    # reading 0.05 after first trying that approach.
    #
    # A real, still-unresolved low-amplitude oscillation of the arm at rest (found live
    # via gz's own ground-truth pose topic, confirmed independent of
    # position_proportional_gain -- see rv2fr_gz.urdf.xacro and docs/decisions.md) means
    # the reported joint state can drift more than 0.05 rad from a just-planned
    # trajectory's start point by the time execution begins, which
    # trajectory_execution_manager treats as an error and refuses to execute ("Invalid
    # Trajectory: start point deviates..."), confirmed live to abort real
    # pick-and-place missions outright. Widening this tolerance unblocks execution
    # without addressing the oscillation itself, which remains tracked separately.
    moveit_config_dict['trajectory_execution']['allowed_start_tolerance'] = 0.15

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('melfa_rv2fr_moveit_config'), 'rviz', 'rv2fr_moveit.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(str(start_rviz)),
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    servo_params = (
        ParameterBuilder('rv2fr_gz_bringup')
        .yaml(parameter_namespace='moveit_servo', file_path='config/rv2fr_gz_servo.yaml')
        .to_dict()
    )
    servo_node = Node(
        package='moveit_servo',
        # The vendor's own launch file (melfa_rv2fr_moveit_config/launch/rv2fr_moveit.launch.py,
        # written for Humble) uses 'servo_node_main', which doesn't exist on this Jazzy
        # install (only 'servo_node' does) -- a real Humble/Jazzy API rename, not a guess.
        executable='servo_node',
        output='screen',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    # moveit_servo starts paused; the vendor launch file's own pattern (start it via
    # a service call the moment the node comes up) is reused here.
    servo_trigger = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
        output='screen',
    )
    servo_trigger_event_handler = RegisterEventHandler(
        OnProcessStart(target_action=servo_node, on_start=[servo_trigger])
    )

    return [move_group_node, rviz_node, servo_node, servo_trigger_event_handler]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
