"""Discover the RV-2FR's actuated joints (name + position limits) straight from the
xacro overlay, instead of hardcoding a joint count or names anywhere in the GUI.
"""
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory


@dataclass
class JointSpec:
    name: str
    lower: float
    upper: float


def load_arm_joints(xacro_relative_path=('urdf', 'rv2fr_gz.urdf.xacro'), prefix=''):
    """Expand rv2fr_gz_bringup's xacro overlay and return the arm's 6 revolute/
    prismatic joints in document order, each with the position limits declared in
    the URDF. Fixed joints (world-to-base, camera mounts, TCP frames) are skipped
    since they have no <limit> and aren't actuated.

    Filters by name (f'{prefix}rv2fr_joint_') in addition to type, not type alone --
    the overlay xacro also defines the gripper's two prismatic finger joints
    (parallel_jaw_gripper.xacro), which are a *different* controller's joints
    (gripper_controller, not rv2fr_controller). Filtering by type alone picked those
    up too, so every JointTrajectory this module's callers built included two joint
    names rv2fr_controller doesn't manage -- which JointTrajectoryController silently
    rejects outright, breaking joint jogging entirely with no error anywhere. Real
    regression, not hypothetical: introduced when the gripper was added, caught by a
    user report that jogging didn't move the robot despite the GUI clearly publishing
    well-formed, continuously-updating commands (confirmed via `ros2 topic echo`).
    """
    gz_bringup_share = get_package_share_directory('rv2fr_gz_bringup')
    xacro_path = '/'.join([gz_bringup_share, *xacro_relative_path])

    result = subprocess.run(
        ['xacro', xacro_path, f'prefix:={prefix}'],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f'xacro failed while discovering joints:\n{result.stderr}')

    root = ET.fromstring(result.stdout)
    arm_joint_prefix = f'{prefix}rv2fr_joint_'
    joints = []
    for joint_el in root.findall('joint'):
        if joint_el.get('type') not in ('revolute', 'prismatic', 'continuous'):
            continue
        name = joint_el.get('name')
        if not name.startswith(arm_joint_prefix):
            continue
        limit_el = joint_el.find('limit')
        lower = float(limit_el.get('lower', '-3.141592653589793')) if limit_el is not None else -3.141592653589793
        upper = float(limit_el.get('upper', '3.141592653589793')) if limit_el is not None else 3.141592653589793
        joints.append(JointSpec(name=name, lower=lower, upper=upper))
    return joints
