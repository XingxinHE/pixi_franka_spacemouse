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

If you want a fresh build directory:

```bash
pixi run clean
pixi run build
```

## SpaceMouse setup (Linux)

If multiple SpaceMouse devices are connected, identify the correct hidraw path:

```bash
grep -H . /sys/class/hidraw/hidraw*/device/uevent | grep SpaceMouse
```

Example output:

`/sys/class/hidraw/hidraw1/device/uevent:HID_NAME=3Dconnexion SpaceMouse Wireless BT`

Set `device_path` to `/dev/hidraw1` in your config file.

## Launch usage

Default launch:

```bash
pixi run run-spacemouse
```

With explicit config:

```bash
pixi run bash -lc "source install/setup.bash && ros2 launch spacemouse_publisher spacemouse_publisher.launch.py config_file:=example_fr3_config.yaml"
```

## Configuration

Configuration files live in:

- `src/spacemouse_publisher/config/example_fr3_config.yaml`
- `src/spacemouse_publisher/config/example_fr3_duo_config.yaml`
- `src/spacemouse_publisher/config/example_crisp_controllers_demo_fr3.yaml`
- `src/spacemouse_publisher/config/example_crisp_gym_recording_fr3.yaml`

Important fields:

- `namespace`
- `device_path`
- `operator_position_front`
- `command_mode` (`pose` or `twist`)
- `current_pose_topic`
- `target_pose_topic`
- `publish_legacy_twist`
- `target_twist_topic`
- `linear_scale`, `angular_scale`, `deadband`
- `max_translation_step`, `max_rotation_step`

SpaceMouse buttons continue to publish gripper-width commands on
`gripper_client/target_gripper_width_percent` for downstream compatibility.

## Use with `crisp_controllers_demos`

This setup teleoperates FR3 launched via `docker compose up launch_franka` from
`crisp_controllers_demos`.

1. Start FR3 in Docker (CycloneDDS):

```bash
cd ../crisp_controllers_demos
RMW=cyclone ROS_NETWORK_INTERFACE=lo ROS_DOMAIN_ID=100 docker compose up launch_franka
```

2. Switch active controller for pose teleop (from host):

```bash
cd ../crisp_controllers_demos
docker compose exec launch_franka bash -lc "cd /home/ros/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export RMW=cyclone && export ROS_NETWORK_INTERFACE=lo && export ROS_DOMAIN_ID=100 && source ./src/crisp_controllers_demos/scripts/setup_middleware.sh && ros2 control switch_controllers --deactivate joint_trajectory_controller --activate cartesian_impedance_controller"
```

3. Run SpaceMouse publisher (host, same middleware):

```bash
cd ../franka_spacemouse
export RMW=cyclone
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_NETWORK_INTERFACE=lo
export ROS_DOMAIN_ID=100
export CYCLONEDDS_URI=file://$PWD/../crisp_controllers_demos/config/cyclone_config.xml
pixi run build
pixi run bash -lc "source install/setup.bash && ros2 launch spacemouse_publisher spacemouse_publisher.launch.py config_file:=example_crisp_controllers_demo_fr3.yaml"
```

Notes:

- `example_crisp_controllers_demo_fr3.yaml` enables `publish_crisp_gripper_command`,
  which bridges SpaceMouse button commands to
  `gripper/gripper_position_controller/commands` expected by
  `crisp_py_franka_hand_adapter` in `crisp_controllers_demos`.
- If your SpaceMouse is not on `/dev/hidraw0`, update `device_path` in the config.

