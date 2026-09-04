# motor_control_g4dual

Upper-layer ROS 2 Humble motor-control package for the G4 dual-motor
controller. This first increment is a safe node template: it converts
`geometry_msgs/msg/Twist` commands into left/right motor RPM targets and
publishes them for inspection. It does **not** access CAN hardware yet.

## Current ROS interface

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Subscribe | `cmd_vel` | `geometry_msgs/msg/Twist` | Linear-x and angular-z drive command |
| Publish | `motor_rpm_command` | `std_msgs/msg/Float64MultiArray` | Dry-run `[left_rpm, right_rpm]` target |

The node clamps targets to `max_motor_speed_rpm` and publishes zero RPM after
`command_timeout_ms` without a valid command. See `config/motor_control.yaml`
for geometry, gearing, motor inversion, limits, and timing parameters. The
default 135 RPM limit and 50 ms control period match the current
`Test_H503RB` firmware contract.

## Build and run (ROS 2 Humble)

Place this repository in a colcon workspace's `src` directory, then run:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select motor_control_g4dual
source install/setup.bash
ros2 launch motor_control_g4dual motor_control.launch.py
```

For a dry-run command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic echo /motor_rpm_command
```

Confirm `wheel_radius_m`, `wheel_separation_m`, `gear_ratio`, and motor
inversion against the real platform before any hardware transport is enabled.

## Implementation plan

1. **ROS node template (this increment):** package/build metadata, launch and
   parameter files, `cmd_vel` input, differential-drive conversion, RPM clamp,
   and stale-command watchdog.
2. **CAN protocol module:** encode the STM32-compatible classic CAN frames
   (`0x601` commands; `0x581`, `0x481`, and `0x381` feedback/fault frames),
   including signed RPM x10 encoding.
3. **SocketCAN transport:** configurable interface (for example `can0`), RX/TX
   error handling, reconnect behavior, and hardware-disabled startup mode.
4. **Lifecycle and safety:** explicit enable/stop/reset services, emergency-stop
   behavior, heartbeat/feedback timeout, and safe shutdown that requests zero
   speed before disabling the drive.
5. **Feedback and odometry:** publish measured wheel speed, encoder deltas,
   faults, diagnostics, joint states, and odometry with verified sign and unit
   conventions.
6. **Verification:** unit tests for kinematics and byte encoding, virtual-CAN
   integration tests, then guarded bench testing with wheels off the ground.

## Repository layout

| Path | Purpose |
|---|---|
| `include/motor_control_g4dual/` | Node declarations |
| `src/` | ROS node implementation and executable entry point |
| `config/` | Runtime parameters |
| `launch/` | ROS 2 launch description |
