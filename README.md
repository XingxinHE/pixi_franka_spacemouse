# Franka Spacemouse

This repository is a modified fork of [franka_spacemouse](https://github.com/frankarobotics/franka_spacemouse). It provides a single ROS 2 Humble package: `spacemouse_publisher`. It reads a 3Dconnexion SpaceMouse and publishes teleoperation commands.

## Scope

The repository now contains only:

- `src/spacemouse_publisher`

Removed from this repository:

- `franka_arm_controllers`
- `gripper_manager`
- devcontainer-based development setup

## Environment (Pixi)

This project is managed with Pixi using `robostack-humble` and `conda-forge`.

```bash
pixi install
```

## Build, Test, Run

Use Pixi tasks:

```bash
pixi run build
pixi run test
pixi run run-spacemouse
pixi run run-spacemouse-recording
```

Mode definitions:
- `run-spacemouse`: testing mode, publishes direct robot commands (`target_pose` and gripper width topic).
- `run-spacemouse-recording`: recording mode, publishes streamed teleop topics (`phone_pose`, `phone_gripper`) without direct robot commands.

Do not run both modes at the same time.

If you want a fresh build directory:

```bash
pixi run clean
pixi run build
```


## Troubleshooting
Before the teleoperation, make sure you do receive messages/topics from the Franka robot.

```shell
pixi run ros2 topic list
```
