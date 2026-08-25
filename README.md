# RV-2FR Teleop & Simulation Stack

ROS 2 (Jazzy) + Gazebo Harmonic + MoveIt 2 workspace for the Mitsubishi RV-2FR
6-axis arm: simulation bringup, a PyQt6 teleoperation GUI, and a single-launch
entry point that can target either simulation or real hardware.

## Packages

| Package | Purpose |
|---|---|
| [`rv2fr_gz_bringup`](rv2fr_gz_bringup/) | Gazebo Harmonic integration: xacro overlay with `gz_ros2_control`, fixed eye-to-hand camera, world/scene files, and launch files to spawn and control the arm in simulation. |
| [`rv2fr_teleop_gui`](rv2fr_teleop_gui/) | PyQt6 teleoperation GUI — joint jogging (sliders discovered from the URDF, not hardcoded), Cartesian jogging via `moveit_servo`, live eye-to-hand camera view, and scene object spawn/reset. Talks to the robot only via ROS 2 topics, so it also works against real hardware given matching topic/controller names. |
| [`rv2fr_hw_bringup`](rv2fr_hw_bringup/) | Single-launch entry point for MoveIt2 + RViz2 against either a real-hardware backend or `rv2fr_gz_bringup`'s simulation (`use_gz_sim:=true`), for validating motion planning and reachability without needing the vendor toolbox. |

Each package has its own README with build flags, topics, and run sequences.

## Requirements

- ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2 (`moveit_servo`)
- Python packages in [`requirements.txt`](requirements.txt)
- `melfa_description`, `melfa_rv2fr_moveit_config`, `melfa_bringup`,
  `melfa_ros2_driver` (vendor/description packages — not vendored in this repo)

## Quick start (simulation)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select melfa_description melfa_rv2fr_moveit_config \
  rv2fr_gz_bringup rv2fr_teleop_gui rv2fr_hw_bringup
source install/setup.bash
ros2 launch rv2fr_gz_bringup rv2fr_full_system.launch.py
```

See `rv2fr_gz_bringup/README.md` for the full launch sequence and design
decisions behind the simulation setup.

## License

Apache License 2.0 — see [LICENSE](LICENSE).
