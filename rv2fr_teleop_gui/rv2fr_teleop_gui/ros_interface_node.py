"""Bridge between rclpy and the Qt event loop.

Event-loop integration choice (documented per PROJECT_SPEC.md, see docs/decisions.md):
rclpy spins on a dedicated background thread (its own SingleThreadedExecutor), fully
independent of Qt's event loop. Results cross to the GUI thread only via PyQt6 signals
(pyqtSignal.emit() from a non-Qt thread is queued and delivered safely on the Qt thread
by Qt's own cross-thread signal/slot machinery -- no manual locking needed).

Rejected alternative: a QTimer periodically calling rclpy.spin_once() on the Qt thread.
That keeps everything on one thread (simpler locking) but ties ROS callback latency to
whatever Qt is doing (redraws, dialogs, drag events) and vice versa -- a slow subscription
callback would visibly stall the UI. A dedicated thread avoids that coupling and is the
pattern used by most existing rclpy+Qt integrations.

Qt widgets must NOT import rclpy directly (PROJECT_SPEC.md ss7) -- panels only call
methods on / connect to signals from this class.
"""
import math
import os
import subprocess
import threading

from PyQt6.QtCore import QObject, pyqtSignal

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped

from rv2fr_teleop_gui.urdf_joints import load_arm_joints

JOINT_JOG_CONTROLLER = 'rv2fr_controller'

# Cartesian/TCP control: solved via MoveIt's /compute_ik service and sent through
# rv2fr_controller (JointTrajectoryController), the same controller joint jogging
# already uses -- NOT moveit_servo. moveit_servo's own Cartesian/TCP control
# (delta_twist_cmds/pose_target_cmds) was found to be permanently non-functional on
# this install: its CurrentStateMonitor never marks itself as having received a valid
# robot state (confirmed live: "Waiting to receive robot state update." repeats
# forever, even across a fresh servo_node restart with the arm already away from its
# home pose), a confirmed-open upstream bug with no fix yet
# (github.com/moveit/moveit2#3040). See docs/decisions.md. Since this no longer needs
# forward_position_controller at all, there is no more controller-switching step --
# rv2fr_controller stays active for both joint and Cartesian control.
IK_GROUP_NAME = 'rv2fr'
TCP_LINK_NAME = 'rv2fr_default_tcp'
IK_PLANNING_FRAME = 'rv2fr_base'  # matches rv2fr_gz_servo.yaml's/rv2fr_servo.yaml's planning_frame
CARTESIAN_LINEAR_SCALE = 0.4  # m/s, matches the old servo config's scale.linear
CARTESIAN_ANGULAR_SCALE = 0.8  # rad/s, matches the old servo config's scale.rotational
CARTESIAN_TICK_SEC = 0.02  # matches cartesian_jog_panel.py's JOG_BURST_INTERVAL_MS
CARTESIAN_JOG_TRAJECTORY_SEC = 0.1  # short, overlapping points -> smooth continuous jog
TRACKING_TRAJECTORY_SEC = 0.3  # object tracking updates less often than jog ticks
IK_TIMEOUT_SEC = 0.1

# Object tracking (no gripper on this robot, so "tracking" means hovering/pointing
# near the detected object, not touching it):
TRACKING_STANDOFF_Z = 0.15  # meters above the object's published pose
TRACKING_ORIENTATION = (1.0, 0.0, 0.0, 0.0)  # x,y,z,w -- flange pointing straight down,
# the same convention already verified working in the (now dormant) pick_place_baseline's
# GRIPPER_DOWN_ORIENTATION.
TRACKING_STALE_TIMEOUT_SEC = 0.75  # rv2fr_perception's color_pose_estimator does not
# republish stale poses -- absence of new messages for this long means "not currently
# visible", not "hasn't moved".


def _quat_multiply(q1, q2):
    """Hamilton product q1 * q2, both (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _axis_angle_quat(axis_index, angle):
    """Small-rotation quaternion (x, y, z, w) of `angle` radians about the given axis
    (0=x, 1=y, 2=z)."""
    half = angle / 2.0
    s = math.sin(half)
    q = [0.0, 0.0, 0.0, math.cos(half)]
    q[axis_index] = s
    return tuple(q)

# encoding -> (QImage.Format, bytes per pixel). Only what the eye-to-hand camera's
# rgb8 stream actually needs; extend if a depth/mono view is added later.
_QIMAGE_FORMATS = {
    'rgb8': ('Format_RGB888', 3),
    'bgr8': ('Format_BGR888', 3),
    'mono8': ('Format_Grayscale8', 1),
}


class RosInterfaceNode(QObject):

    joint_state_received = pyqtSignal(dict)  # {joint_name: position (rad)}
    connection_status_changed = pyqtSignal(bool)  # True once /joint_states has been seen
    # (message, is_error) -- controller-switch/servo-setup outcomes. These used to only
    # go to the node's rclpy logger, invisible unless watching the terminal teleop_gui
    # was launched from; a failed switch left the robot uncontrollable with zero
    # feedback in the GUI itself. See docs/decisions.md.
    jog_status_changed = pyqtSignal(str, bool)
    # (raw bytes, width, height, bytes_per_line, encoding) -- raw, not a QImage,
    # since QImage should be constructed on the Qt (receiving) thread.
    camera_image_received = pyqtSignal(bytes, int, int, int, str)
    # (message, is_error) -- same convention as jog_status_changed -- object tracking
    # start/stop/visibility outcomes.
    object_tracking_status_changed = pyqtSignal(str, bool)

    def __init__(self, controller_topic='/rv2fr_controller/joint_trajectory',
                 camera_topic='/eye_to_hand_camera/image', prefix=''):
        super().__init__()
        self.joints = load_arm_joints(prefix=prefix)
        self._joint_names = [j.name for j in self.joints]
        self._got_joint_state = False

        rclpy.init(args=None)
        self._node = Node('rv2fr_teleop_gui_interface')

        # Command topics are reliable (we need every jog command delivered);
        # /joint_states is high-rate sensor data, so best-effort/small-depth per
        # PROJECT_SPEC.md ss7's QoS guidance.
        command_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._trajectory_pub = self._node.create_publisher(
            JointTrajectory, controller_topic, command_qos
        )
        self._joint_state_sub = self._node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, state_qos
        )
        # Camera stream: high-rate sensor data, same best-effort/small-depth QoS as
        # /joint_states, and matches ros_gz_bridge's default QoS for bridged image
        # topics (bridge uses sensor-data QoS on the ROS side).
        self._camera_sub = self._node.create_subscription(
            Image, camera_topic, self._on_camera_image, state_qos
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)
        self._object_pose_sub = None
        self._tracking_object = None
        self._last_tracking_pose_time = None
        self._current_joint_positions = {}  # cached from _on_joint_state, used to seed IK

        self._ik_client = self._node.create_client(GetPositionIK, '/compute_ik')
        self._cartesian_ik_busy = False  # drop a jog tick rather than let IK requests pile up
        self._tracking_staleness_timer = self._node.create_timer(0.25, self._check_tracking_staleness)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)

    def start(self):
        self._spin_thread.start()

    def shutdown(self):
        # executor.shutdown() signals spin() to return, but doesn't wait for the
        # thread -- destroying the node/context while spin() is still mid-callback
        # on the other thread aborts (rclpy issues this as a fatal C-level error,
        # not a Python exception). Join first.
        self._executor.shutdown()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        rclpy.shutdown()

    def _on_joint_state(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        self._current_joint_positions = positions
        if not self._got_joint_state:
            self._got_joint_state = True
            self.connection_status_changed.emit(True)
        self.joint_state_received.emit(positions)

    def _on_camera_image(self, msg: Image):
        if msg.encoding not in _QIMAGE_FORMATS:
            return
        self.camera_image_received.emit(
            bytes(msg.data), msg.width, msg.height, msg.step, msg.encoding
        )

    def send_joint_positions(self, positions: dict, duration_sec: float = 0.5):
        """positions: {joint_name: target_rad}, must cover all controlled joints --
        our rv2fr_controller.yaml doesn't set allow_partial_joints_update, so the
        JointTrajectoryController expects every joint on every point."""
        msg = JointTrajectory()
        msg.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = [positions[name] for name in self._joint_names]
        sec = int(duration_sec)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        msg.points = [point]
        self._trajectory_pub.publish(msg)

    def send_cartesian_twist(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
        """linear/angular: unitless [-1, 1] per axis, one jog tick. Computes a small
        Cartesian delta from the TCP's *actual current* pose (via TF -- self-correcting,
        not an open-loop accumulator) and solves the corresponding joint target via
        MoveIt's /compute_ik, then sends it through rv2fr_controller as a short
        JointTrajectory point. Linear deltas are applied in IK_PLANNING_FRAME (fixed
        world/base axes); angular deltas are applied as a local (tool-frame) rotation
        about the TCP's own current orientation, so translation buttons always move the
        same physical direction regardless of tool orientation, while rotation buttons
        always roll/pitch/yaw about the tool's own axes."""
        if self._cartesian_ik_busy:
            return  # drop this tick rather than let requests queue up; next tick retries
        try:
            transform = self._tf_buffer.lookup_transform(
                IK_PLANNING_FRAME, TCP_LINK_NAME, Time(),
            )
        except Exception as exc:
            self.jog_status_changed.emit(f'TF lookup failed: {exc}', True)
            return

        dt = CARTESIAN_TICK_SEC
        dx, dy, dz = (float(v) * CARTESIAN_LINEAR_SCALE * dt for v in linear)
        target = PoseStamped()
        target.header.frame_id = IK_PLANNING_FRAME
        target.pose.position.x = transform.transform.translation.x + dx
        target.pose.position.y = transform.transform.translation.y + dy
        target.pose.position.z = transform.transform.translation.z + dz

        orientation = (
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w,
        )
        for axis_index, w in enumerate(angular):
            if w == 0.0:
                continue
            delta_q = _axis_angle_quat(axis_index, float(w) * CARTESIAN_ANGULAR_SCALE * dt)
            orientation = _quat_multiply(orientation, delta_q)
        target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = orientation

        self._solve_ik_and_publish(target, duration_sec=CARTESIAN_JOG_TRAJECTORY_SEC)

    def _solve_ik_and_publish(self, target: PoseStamped, duration_sec: float):
        """Shared by Cartesian jogging and object tracking: solve IK for `target`
        (in IK_PLANNING_FRAME, for TCP_LINK_NAME) and, on success, send the result
        through rv2fr_controller. Async, not spin_until_future_complete -- this runs on
        the Qt thread while the node's own executor spins on a separate thread."""
        if not self._ik_client.service_is_ready():
            self.jog_status_changed.emit(
                'MoveIt /compute_ik not available -- launch a MoveIt-providing launch file first '
                '(e.g. rv2fr_hw_bringup.launch.py).', True,
            )
            return

        self._cartesian_ik_busy = True
        request = GetPositionIK.Request()
        request.ik_request.group_name = IK_GROUP_NAME
        request.ik_request.ik_link_name = TCP_LINK_NAME
        request.ik_request.pose_stamped = target
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(sec=0, nanosec=int(IK_TIMEOUT_SEC * 1e9))
        if self._current_joint_positions:
            # Seed IK with the current joint state so it converges near the arm's
            # actual configuration rather than an arbitrary/default seed -- keeps
            # successive small jog steps continuous instead of jumping between
            # unrelated IK solution branches.
            request.ik_request.robot_state.joint_state.name = list(self._current_joint_positions.keys())
            request.ik_request.robot_state.joint_state.position = list(self._current_joint_positions.values())

        future = self._ik_client.call_async(request)

        def on_done(fut):
            self._cartesian_ik_busy = False
            result = fut.result()
            if not result or result.error_code.val != MoveItErrorCodes.SUCCESS:
                code = result.error_code.val if result else 'no response'
                self.jog_status_changed.emit(f'IK failed (error_code={code})', True)
                return
            positions = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
            try:
                self.send_joint_positions(positions, duration_sec=duration_sec)
            except KeyError:
                self.jog_status_changed.emit('IK solution missing expected joints', True)

        future.add_done_callback(on_done)

    def start_object_tracking(self, object_name: str):
        """Starts following /rv2fr_perception/{object_name}_pose with a fixed standoff
        offset -- there's no gripper on this robot, so "tracking" means hovering/
        pointing near the object, not touching it. Solved the same way as Cartesian
        jogging: /compute_ik + rv2fr_controller, not moveit_servo."""
        if self._tracking_object == object_name:
            return
        if self._object_pose_sub is not None:
            self.stop_object_tracking()

        self._tracking_object = object_name
        self._last_tracking_pose_time = None
        self._object_pose_sub = self._node.create_subscription(
            PoseStamped, f'/rv2fr_perception/{object_name}_pose', self._on_tracking_pose, 10,
        )
        self.object_tracking_status_changed.emit(f'Tracking {object_name}...', False)

    def stop_object_tracking(self):
        if self._object_pose_sub is not None:
            self._node.destroy_subscription(self._object_pose_sub)
            self._object_pose_sub = None
        self._tracking_object = None
        self._last_tracking_pose_time = None
        self.object_tracking_status_changed.emit('Tracking stopped', False)

    def _on_tracking_pose(self, msg: PoseStamped):
        self._last_tracking_pose_time = self._node.get_clock().now()
        try:
            transform = self._tf_buffer.lookup_transform(
                IK_PLANNING_FRAME, msg.header.frame_id, Time(),
            )
        except Exception as exc:
            self.object_tracking_status_changed.emit(f'TF lookup failed: {exc}', True)
            return
        transformed = do_transform_pose_stamped(msg, transform)

        target = PoseStamped()
        target.header.frame_id = IK_PLANNING_FRAME
        target.pose.position.x = transformed.pose.position.x
        target.pose.position.y = transformed.pose.position.y
        target.pose.position.z = transformed.pose.position.z + TRACKING_STANDOFF_Z
        ox, oy, oz, ow = TRACKING_ORIENTATION
        target.pose.orientation.x = ox
        target.pose.orientation.y = oy
        target.pose.orientation.z = oz
        target.pose.orientation.w = ow
        self._solve_ik_and_publish(target, duration_sec=TRACKING_TRAJECTORY_SEC)

    def _check_tracking_staleness(self):
        if self._tracking_object is None or self._last_tracking_pose_time is None:
            return
        elapsed_sec = (self._node.get_clock().now() - self._last_tracking_pose_time).nanoseconds / 1e9
        if elapsed_sec > TRACKING_STALE_TIMEOUT_SEC:
            self.object_tracking_status_changed.emit(f'{self._tracking_object} not currently visible', True)
            self._last_tracking_pose_time = None  # avoid re-emitting every timer tick

    def spawn_object(self, model_name: str, instance_name: str, x: float, y: float, z: float):
        """model_name: a directory under rv2fr_gz_bringup/models/ (e.g. 'rv2fr_cube').
        Fire-and-forget via `ros2 run ros_gz_sim create`, matching how
        rv2fr_gz_sim.launch.py spawns the robot itself -- there's no persistent
        ROS2 spawn *service* in ros_gz_sim on this Jazzy/Harmonic install, `create`
        is a one-shot CLI tool by design. subprocess.Popen (not .run) so this
        doesn't block the Qt thread waiting for gz sim to process the spawn."""
        sdf_path = os.path.join(
            get_package_share_directory('rv2fr_gz_bringup'), 'models', model_name, 'model.sdf'
        )
        subprocess.Popen(
            ['ros2', 'run', 'ros_gz_sim', 'create',
             '-file', sdf_path, '-name', instance_name,
             '-x', str(x), '-y', str(y), '-z', str(z),
             '-allow_renaming', 'false'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def delete_object(self, instance_name: str):
        # 'delete_entity' (CLI11-style --name flag) hangs on this Harmonic install --
        # confirmed by direct testing, not just assumed. 'remove' is the one that
        # actually works, but takes its arguments as ROS2 parameters (--ros-args -p),
        # not CLI flags -- inconsistent with 'create's gflags-style -flag syntax, but
        # confirmed empirically (ros2 run ros_gz_sim remove -name ... silently no-ops
        # with "Entity to remove name is not provided", only --ros-args -p entity_name:=
        # actually works).
        subprocess.Popen(
            ['ros2', 'run', 'ros_gz_sim', 'remove',
             '--ros-args', '-p', 'entity_name:=' + instance_name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
