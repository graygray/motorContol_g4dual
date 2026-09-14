# motor_control_g4dual

Upper-layer ROS 2 Humble motor-control package for the G4 dual-motor
controller. This first increment is a safe node template: it converts
`geometry_msgs/msg/Twist` commands into left/right motor RPM targets and
publishes them for inspection. SocketCAN transmission is available but remains
disabled by default.

The repository also contains a transport-independent C++ CAN encoder matching
the command payloads in `Test_H503RB/Core/Src/motor_can.c`. The encoder creates
eight-byte standard-ID `0x601` frames. When `enable_can` is true, the node
transmits those frames through the interface selected by `can_interface`.
The same transport receives and decodes controller replies (`0x581`), absolute
encoder positions (`0x481`), and motor-fault reports (`0x381`).

## Current ROS interface

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Subscribe | `cmd_vel` | `geometry_msgs/msg/Twist` | Linear-x and angular-z drive command |
| Publish | `motor_rpm_command` | `std_msgs/msg/Float64MultiArray` | Dry-run `[left_rpm, right_rpm]` target |
| Publish | `wheel_speed_feedback` | `std_msgs/msg/Float64MultiArray` | Measured `[left_rpm, right_rpm]` |
| Publish | `encoder_delta_feedback` | `std_msgs/msg/Int32MultiArray` | Raw 50 ms `[left, right]` encoder deltas |
| Publish | `motor_fault` | `std_msgs/msg/UInt32MultiArray` | `[motor_selector, fault_mask]` |
| Publish | `joint_states` | `sensor_msgs/msg/JointState` | Wheel position and angular velocity |
| Publish | `odom` | `nav_msgs/msg/Odometry` | Differential-drive wheel odometry |
| Publish | `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | CAN and safety state |

All motor commands use topics; the former motor-control services have been removed.

| Subscribe topic | Type | Purpose |
|---|---|---|
| `enable_motors` | `std_msgs/msg/Empty` | Send the staged enable sequence and an initial zero-speed command |
| `stop_motors` | `std_msgs/msg/Empty` | Stop motion and gate further physical speed commands |
| `emergency_stop` | `std_msgs/msg/Bool` | `true` sends the original EMO stop command (`0x02`), which uses ramp-stop behavior; `false` releases the hold without re-enabling motion |
| `reset_faults` | `std_msgs/msg/Empty` | Request fault reset and clear upper-layer safety latches |

Control-event subscriptions use reliable, volatile QoS with a queue depth of 10.
Publish each event once with volatile durability; do not periodically repeat or
retain enable/reset commands. Messages published before a subscriber connects
are not replayed. Topic publication does not return a success response.
The node publishes `diagnostics` immediately after each control event and once
per second, including `last_control_command`, `last_control_success`, and
`last_control_message` after the first event. These fields describe the latest
local command result, not a controller acknowledgement or a per-client reply.
Failures are also logged even when `info` is false.

The node clamps targets to `max_motor_speed_rpm` and publishes zero RPM after
`command_timeout_ms` without a valid command. See `config/motor_control.yaml`
for geometry, gearing, motor inversion, limits, and timing parameters. The
default 135 RPM limit and 50 ms control period match the current
`Test_H503RB` firmware contract.

| Parameter | Default | Purpose |
|---|---:|---|
| `enable_can` | `false` | Enables physical SocketCAN transmission |
| `rpm_resolution` | `0.1` | Startup-only CAN speed resolution: `1.0` or `0.1` RPM per unit |
| `can_interface` | `can0` | Linux SocketCAN network-interface name |
| `can_receive_poll_ms` | `50` | Interval used to drain received CAN frames |
| `feedback_timeout_ms` | `500` | Enabled-motion timeout for speed/position feedback |
| `odom_frame_id` / `base_frame_id` | `odom` / `base_link` | Odometry frame names |
| `left_joint_name` / `right_joint_name` | wheel joint names | Joint-state names |
| `publish_odom_tf` | `true` | Broadcast the `odom` to `base_link` transform |

SocketCAN support requires Linux. If CAN is explicitly enabled and the named
interface cannot be opened, node startup fails instead of silently continuing
in dry-run mode. Physical speed commands remain gated until `enable_motors`
succeeds. A motor-fault report or feedback timeout closes that gate and sends
the original EMO stop command, which uses ramp-stop behavior. Shutdown sends
zero RPM and then stop before closing the CAN socket.

The upper layer maps Motor 1 to the left wheel and Motor 2 to the right wheel.
Direction inversion is applied consistently to commands, feedback, joint
states, and odometry. Absolute `0x481` positions are treated as wheel-side
quadrature counts at 16,384 counts/revolution; command-side `gear_ratio` is not
applied to encoder feedback.

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

Select the CAN RPM resolution at launch (default: `0.1`):

```bash
ros2 launch motor_control_g4dual motor_control.launch.py rpm_resolution:=1
ros2 launch motor_control_g4dual motor_control.launch.py rpm_resolution:=0.1
```

For direct execution, use a floating-point ROS parameter:

```bash
ros2 run motor_control_g4dual motor_control_node --ros-args -p rpm_resolution:=1.0
```

The setting must match the controller firmware. It applies to speed command
encoding and speed feedback decoding: 12 RPM is encoded as 12 at resolution
`1.0`, or 120 at resolution `0.1`. Commands round to the nearest CAN unit;
ROS topics still use physical RPM and the speed limit remains 135 RPM.
Other values are rejected. Restart the node to change the resolution.
The launch argument overrides the value in the YAML configuration.

Informational event logging defaults to `false`. Warnings and errors are always
submitted to the ROS logger, independently of `info`. Normal ROS severity
filtering still applies. Enable informational events at startup with:

```bash
ros2 launch motor_control_g4dual motor_control.launch.py info:=true
ros2 run motor_control_g4dual motor_control_node --info
ros2 run motor_control_g4dual motor_control_node --info=false
```

Change the Boolean ROS parameter while the node is running (no restart):

```bash
ros2 param set /motor_control info true
ros2 param set /motor_control info false
ros2 param get /motor_control info
```

`--info` sets the initial default; a ROS parameter override such as
`--ros-args -p info:=true` takes precedence. CLI Boolean values accept
`true`/`false`, `1`/`0`, and `on`/`off`. The old `--log` and launch `log:=`
options remain aliases for the initial `info` setting; they no longer suppress
warnings/errors or enable periodic telemetry.

Repetitive control, feedback, odometry, and diagnostics messages now use
throttled `DEBUG` logging, independently of `info`. To see them during debugging:

```bash
ros2 run motor_control_g4dual motor_control_node --ros-args --log-level motor_control:=debug
```

Changing `info` affects subsequent informational events only; it does not replay
past events or enable periodic telemetry. Topic publishing and safety checks
continue regardless of logging settings.

For a dry-run command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
ros2 topic echo /motor_rpm_command
```

Publish motor-control events using the same CLI pattern:

```bash
ros2 topic pub --once /enable_motors std_msgs/msg/Empty '{}'
ros2 topic pub --once /stop_motors std_msgs/msg/Empty '{}'
ros2 topic pub --once /reset_faults std_msgs/msg/Empty '{}'
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: false}'
ros2 topic echo /diagnostics
```

These are individual command examples, not a startup sequence. After fault
reset or emergency-stop release, publish `enable_motors` separately, check
`diagnostics` for the result, then send fresh `cmd_vel` messages. Commands on
different topics have no shared ordering; wait for each result before sending
the next dependent command. In dry-run mode, control events report failure
because SocketCAN is disabled.

Confirm `wheel_radius_m`, `wheel_separation_m`, `gear_ratio`, and motor
inversion against the real platform before any hardware transport is enabled.

## Tests

The normal test run covers differential-drive conversion, proportional RPM
limiting, direction inversion, encoder rollover, CAN command encoding, reply
decoding, malformed-frame rejection, and topic command delivery with dry-run
failure diagnostics (including both emergency-stop values):

```bash
colcon test --packages-select motor_control_g4dual
colcon test-result --verbose
```

The SocketCAN integration test is built on Linux but skips unless a test
interface is explicitly selected. To run it on `vcan0`:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
MOTOR_CONTROL_VCAN_INTERFACE=vcan0 \
  colcon test --packages-select motor_control_g4dual
colcon test-result --verbose
```

Create `vcan0` only if it does not already exist. The integration test verifies
an encoded `0x601` transmission through the kernel and injects a `0x581` reply
back through the filtered receive and decode path. It never enables motors.

## Implementation plan

1. **ROS node template (this increment):** package/build metadata, launch and
   parameter files, `cmd_vel` input, differential-drive conversion, RPM clamp,
   and stale-command watchdog.
2. **CAN protocol module (complete):** encode the STM32-compatible classic CAN frames
   (`0x601` commands; `0x581`, `0x481`, and `0x381` feedback/fault frames),
   including selectable signed RPM x1 or x10 encoding.
3. **SocketCAN transmit transport (complete):** configurable interface (for
   example `can0`), strict `0x601` command-ID validation, transmit error
   reporting, and hardware-disabled startup mode.
4. **CAN receive and decoding (complete):** filtered nonblocking reception and
   typed decoding for firmware, measured speed, encoder delta, absolute encoder
   position, and motor-fault frames on `0x581`, `0x481`, and `0x381`.
5. **Lifecycle and safety (complete):** explicit enable/stop/reset topics,
   emergency-stop behavior, feedback timeout, command gating, fault latching,
   and safe shutdown that requests zero speed before stopping the drive.
6. **Feedback and odometry (complete):** publish measured wheel speed, encoder
   deltas, faults, diagnostics, joint states, odometry, and optional odometry TF
   with consistent wheel-side sign and unit conventions.
7. **Verification suite (complete):** unit tests for kinematics and byte
   encoding plus an opt-in kernel `vcan` RX/TX integration test. Execution on a
   ROS 2 Humble Linux host and guarded bench testing remain deployment steps.

## Repository layout

| Path | Purpose |
|---|---|
| `include/motor_control_g4dual/` | Node declarations |
| `src/motor_can_protocol.cpp` | ROS-independent `0x601` frame encoder |
| `src/socket_can_transport.cpp` | Filtered Linux SocketCAN RX/TX transport |
| `src/motor_control_node.cpp` | ROS node implementation |
| `src/main.cpp` | ROS executable entry point |
| `config/` | Runtime parameters |
| `launch/` | ROS 2 launch description |
| `test/` | Kinematics, protocol, and opt-in SocketCAN integration tests |
