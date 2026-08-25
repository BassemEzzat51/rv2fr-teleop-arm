# rv2fr_gz_bringup

gz sim (Gazebo Harmonic) integration for the Mitsubishi RV-2FR: xacro overlay with
`gz_ros2_control`, fixed eye-to-hand camera, a simple parallel-jaw gripper, world files,
scene object models, and launch files to spawn and control the arm in simulation. See
`docs/decisions.md` at the workspace root for the design decisions behind all of this.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/bassem/ros2_ws
colcon build --packages-select melfa_description melfa_rv2fr_moveit_config rv2fr_gz_bringup rv2fr_teleop_gui
source install/setup.bash
```

## Run everything at once

```bash
ros2 launch rv2fr_gz_bringup rv2fr_full_system.launch.py
```

Composes the sim, `move_group` + `moveit_servo`, `rv2fr_perception`, the
`rv2fr_pick_place_baseline` service node, and the `rv2fr_teleop_gui` window into one
launch tree (staggered with delays so each piece's dependencies are up first -- see the
launch file's own docstring for the exact timing rationale). Useful launch arguments:

- `gz_args:="-s"` -- headless gz sim (no GUI window)
- `start_gui:=false` -- skip the teleop GUI (e.g. on a headless machine)
- `start_perception:=false`, `start_pick_place:=false` -- skip either of those nodes
- `start_rviz:=true` -- also open MoveIt2's RViz plugin

Each piece below also has its own standalone launch file/command if you want to bring
things up individually instead (e.g. while iterating on one package) -- this all-in-one
launch file composes them, it doesn't replace them.

## Run the simulation

```bash
ros2 launch rv2fr_gz_bringup rv2fr_gz_sim.launch.py
```

Spawns the RV-2FR into an empty world, brings up `joint_state_broadcaster`,
`rv2fr_controller` (joint trajectory control), and `gripper_left_controller`/
`gripper_right_controller` (one finger each -- not a single two-joint controller, see
docs/decisions.md for the real bug that caused), and bridges the eye-to-hand camera +
`/clock`. Add `gz_args:="-s"` to run headless (no gz sim GUI window).

**If the gz sim GUI window crashes** with a `symbol lookup error` mentioning
`/snap/core20/.../libpthread.so.0`: this is caused by `GTK_PATH`/`GTK_EXE_PREFIX`/
`GTK_MODULES` environment variables leaking in from a snap-packaged app (this affects
VS Code's own integrated terminal specifically, not a normal terminal). Fix: run
`unset GTK_PATH GTK_EXE_PREFIX GTK_MODULES GTK_IM_MODULE_FILE` before launching, or use a
regular system terminal instead of VS Code's integrated one. See `docs/decisions.md` for
the full diagnosis.

## Run the teleop GUI

In a second terminal (same sourcing as above):

```bash
ros2 run rv2fr_teleop_gui teleop_gui
```

You should see: the robot's joint positions live-updating, sliders that jog each joint,
a gripper open/close, a live camera feed, and scene object spawn/reset controls.

## Enable Cartesian jogging (moveit_servo)

Cartesian jogging needs `move_group` + `moveit_servo` running against the sim, in a third
terminal:

```bash
ros2 launch rv2fr_gz_bringup rv2fr_moveit_servo.launch.py
```

Then in the teleop GUI, check "Enable Cartesian jog" in the Cartesian Jog panel before
using its X/Y/Z/Roll/Pitch/Yaw buttons -- this switches the active controller from
`rv2fr_controller` to `forward_position_controller` (the two cannot be active
simultaneously; unchecking, or using a Joint Jog slider, switches back). Add
`start_rviz:=true` to also open MoveIt2's RViz plugin.

**Note**: moveit_servo's singularity-avoidance safety logic will refuse to move the
robot (and `/servo_node/status` will report `HALT_FOR_SINGULARITY`) if it's at or near a
singular pose -- notably the all-zero home position. Jog the joints to a bent pose first
if Cartesian jogging doesn't seem to do anything.

## Verify it's actually working

```bash
ros2 control list_controllers                     # joint_state_broadcaster, rv2fr_controller,
                                                    # gripper_left_controller, gripper_right_controller
                                                    # should all be "active"
ros2 topic hz /joint_states                        # should publish continuously
ros2 topic hz /eye_to_hand_camera/image             # should publish continuously
```
