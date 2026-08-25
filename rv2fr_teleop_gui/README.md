# rv2fr_teleop_gui

PyQt6 teleoperation GUI for the Mitsubishi RV-2FR running in `gz sim`. Talks to the
robot only via ROS2 topics (no direct gz sim/simulator dependency), so it works
against real hardware too, given matching topic/controller names.

Currently implemented: joint jogging (slider + spinbox + live readback per joint,
discovered from the URDF -- not hardcoded), Cartesian jogging via moveit_servo, live
eye-to-hand camera view, and scene object spawn/reset control. The robot has no gripper
and no pick-and-place feature. Episode recording (PROJECT_SPEC.md ss4.4) is not yet built.

## Run standalone

Requires `rv2fr_gz_bringup` to be built (joint names/limits are read from its xacro
overlay at startup) and, to see live readback and have jog commands actually move a
robot, `rv2fr_gz_bringup`'s sim launch running with its controllers active:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rv2fr_teleop_gui teleop_gui
```

Without a running sim/controller, the window still opens and sliders still publish
`JointTrajectory` messages on `/rv2fr_controller/joint_trajectory` -- there's just no
subscriber to act on them and no `/joint_states` to show in the readback column.

Cartesian jogging additionally needs `rv2fr_gz_bringup`'s `rv2fr_moveit_servo.launch.py`
running (`move_group` + `moveit_servo`) -- see `rv2fr_gz_bringup/README.md` for the full
run sequence across all three launch files/processes.

## Design notes

See `docs/decisions.md` at the workspace root for the rationale behind:
- rclpy running on a dedicated background thread rather than a QTimer-driven spin
  (event-loop integration choice, PROJECT_SPEC.md ss4.2 step 5)
- reading joint names/limits from the xacro overlay instead of hardcoding them
