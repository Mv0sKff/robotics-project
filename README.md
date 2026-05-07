# robotics-project

## Build

## Start RViz

```sh
ros2 launch lbr_bringup rviz.launch.py \
    rviz_cfg_pkg:=lbr_bringup \
    rviz_cfg:=config/mock.rviz
```

# Run on Hardware

<https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html>

## Configure Network PC

1. Connect ethernet cable `X66`
2. Set interface `172.31.1.148` with Netmask 255.255.0.0

## Configure Robot LBRServer

Launch the `LBRServer` application.

1. FRI send period: 10 ms
2. IP address: your configuration
3. FRI control mode: POSITION_CONTROL
4. FRI client command mode: POSITION

## Run Test Program

### Terminal 1

```sh
ros2 run lbr_demos_py joint_sine_overlay --ros-args -r __ns:=/lbr
```

### Terminal 2

```sh
ros2 launch lbr_bringup hardware.launch.py \
    ctrl:=lbr_joint_position_command_controller \
    model:=iiwa14
```
