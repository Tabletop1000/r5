# r5_sensors

The BNO055 IMU driver, and the config and launch files for the IMU and the u-blox GPS. The GPS driver
comes from the `ublox` package.

## BNO055 driver

`bno055` is a C++ driver for the BNO055 over I2C. It replaced the Python driver from the `bno055`
package, which couldn't keep up with 100 Hz on the robot. It has the same node name, parameters,
topics and service, so `config/bno055_params.yaml` and everything downstream are unchanged.

```bash
ros2 launch r5_sensors bno055_imu_node-launch.py
```

| Topic | Type | Contents |
| --- | --- | --- |
| `/bno055/imu` | `sensor_msgs/Imu` | Fused orientation, gyro, and acceleration including gravity |
| `/bno055/imu_raw` | `sensor_msgs/Imu` | Gyro and accelerometer, no orientation |
| `/bno055/mag` | `sensor_msgs/MagneticField` | Tesla |
| `/bno055/grav` | `geometry_msgs/Vector3` | Gravity alone, m/s² |
| `/bno055/temp` | `sensor_msgs/Temperature` | °C, 1 °C steps |
| `/bno055/calib_status` | `std_msgs/String` | JSON, `{"sys": 3, "gyro": 3, "accel": 3, "mag": 3}`, each 0 to 3 |

`/bno055/calibration_request` (`example_interfaces/Trigger`) returns the chip's calibration offsets.

### Differences from the Python driver

- `/bno055/imu` acceleration includes gravity, as REP 145 and the simulator have it. The Python driver
  sent acceleration with gravity removed. The EKFs don't use acceleration; the dashboard's IMU test
  expects gravity.
- Acceleration is in m/s². The Python driver read 1 g as 10.0 instead of 9.81.
- After about a second of failed reads it reconfigures the chip, and publishes nothing until that
  works, rather than warning for ever.
- Only I2C (`connection_type: "i2c"`). The UART parameters are gone.

### Code

- `include/r5_sensors/bno055.hpp`, `src/bno055.cpp`: the chip without ROS: configuring, reading and
  decoding.
- `include/r5_sensors/i2c_bus.hpp`, `src/i2c_bus.cpp`: Linux `/dev/i2c-N`. A whole sample is one
  45-byte transfer.
- `include/r5_sensors/bno055_node.hpp`: the ROS node, built as the component `r5_sensors::Bno055Node`
  and the executable `bno055`.
- `test/test_bno055.cpp`: unit tests with a fake bus, so no hardware is needed.
