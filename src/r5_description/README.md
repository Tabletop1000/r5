# r5

Robot description, ros2_control hardware interface and controller config for R5, a four-wheel
skid-steer robot. Based on the ros2_control DiffBot demo (`example_2`).

The two motor nodes connect over serial as `/dev/ttyMotornodeFront` and `/dev/ttyMotornodeRear`,
named by udev rules on the robot.

## Usage

Start the robot and sensors from `r5_bringup`:

```bash
ros2 launch r5_bringup r5.launch.py
```

Start only the base, optionally with RViz or without any hardware attached:

```bash
ros2 launch r5 diffbot.launch.py gui:=true use_mock_hardware:=true
```

View the robot model:

```bash
ros2 launch r5 view_robot.launch.py
```

Read a motor node directly:

```bash
ros2 run r5 motornode_console /dev/ttyMotornodeFront
```
