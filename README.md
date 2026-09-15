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

`last_control_reply_state` separately reports controller replies. It defaults to
`unavailable`, since the local G4 firmware does not acknowledge control writes.
For firmware that replies to **every** `0x6040:00` write, set the startup-only
parameter `expect_control_ack: true`. States then include `idle`, `pending`,
`acknowledged`, `rejected`, `timed_out`, `transmission_failed`, and `ambiguous`.
An enable event expects three acknowledgements; stop, reset, and emergency-stop
events each expect one. `control_replies_remaining` and `control_abort_code`
provide details. These fields describe control-word replies, not the initial
zero-speed write, actual motor state, or completion of physical stopping.

Acknowledgement tracking is diagnostic only; it does not delay commands or
change the existing motion gates. Use it only with a single CAN command client
and ordered, nonduplicated replies. Replies contain an object index and subindex
but no command value or transaction ID. Overlapping events, internal safety
commands, partial transmission failures, or a timeout make subsequent reply
association ambiguous until the node restarts. A late reply cannot clear a
timeout or confirm a later command. Stop events are still transmitted immediately.

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
| `reply_timeout_ms` | `500` | Startup-only deadline for firmware and optional control replies |
| `expect_control_ack` | `false` | Startup-only opt-in control acknowledgement diagnostics |
| `expected_firmware_identifier` | empty | Startup-only optional identity comparison; mismatch produces a warning |
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

## Built-in motion tests (version 1)

The **existing `motor_control_node`** executes tests through an internal state
machine. No additional ROS node or executable is launched. The test feature is
**enabled by default alongside normal `cmd_vel` control**. Tests start only on
request and never enable/reset motors themselves. Set `motion_test.enabled: false`
at startup if you want to disable test commands.

| `motion_test/command` (`std_msgs/msg/String`) | Sequence |
|---|---|
| `straight` | Forward by `distance_m`, stop, reverse by the same distance, stop; repeat |
| `rotate` | Turn left by `angle_deg`, stop, turn right by the same angle, stop; repeat |
| `cancel` | Abort the active test, request zero speed and the existing ramp emergency stop |

Commands use reliable, volatile QoS. Publish once after discovery; do not retain
or periodically repeat start commands. Duplicate starts are rejected while a
test is active. `motion_test/status` is a reliable, transient-local String topic
that retains the latest status for late subscribers. It reports state, segment,
reason, last stopped segment progress, total odometry displacement/yaw and CSV
path. `diagnostics` also includes `motion_test_*` state fields.

### Run on the robot

Build and source the workspace as above. In the installed YAML (or your own
copy), set `enable_can: true` (the test feature already defaults to enabled), after verifying
CAN interface, geometry, direction inversion and RPM resolution. Alternatively,
start the same executable explicitly:

```bash
ros2 run motor_control_g4dual motor_control_node --ros-args \
  --params-file /absolute/path/to/motor_control.yaml \
  -p enable_can:=true
```

1. Stop navigation/teleoperation publishers of `cmd_vel`.
2. Publish `enable_motors` once using the existing command shown above. Check
   diagnostics for `motion_commands_enabled: true`, no safety latches and fresh
   encoder-position **and** wheel-speed feedback. This is a local enable result,
   not proof of hardware acknowledgement.
3. Wait for standstill and for any previous `cmd_vel` to expire (default 500 ms).
4. Observe status, then start **one** selected test:

```bash
ros2 topic echo /motion_test/status --qos-durability transient_local
# In another terminal: forward/backward test
ros2 topic pub --once /motion_test/command std_msgs/msg/String '{data: straight}'
```

For rotation, after the previous test finishes, use:

```bash
ros2 topic pub --once /motion_test/command std_msgs/msg/String '{data: rotate}'
```

Cancel an active run with:

```bash
ros2 topic pub --once /motion_test/command std_msgs/msg/String '{data: cancel}'
```

All `motion_test.*` settings are startup-only; restart after changing them.
Defaults are 1 m, 90 degrees, three forward/reverse or left/right pairs,
0.1 m/s, 0.2 rad/s, acceleration limits 0.1 m/s² and 0.2 rad/s². The YAML lists
all settings. Angles such as 180 or 360 degrees use continuous accumulated yaw.
There is no upper-layer heading correction: these tests expose motor/platform
asymmetry. Distance termination uses displacement projected on each segment's
starting heading; reverse is measured from the actual stopped forward endpoint,
not a navigation command back to the original pose.

### Mixing tests with normal driving

There is no separate operating mode to switch into. Tests and ordinary commands
share the same node and motor enable state. One command source owns motion at a
time: a running test generates velocities until it completes or a valid new
`cmd_vel` takes over. Velocities are not added together. Stop periodic normal
publishers while you want a test to run; otherwise their next message will
interrupt it. Starting a test still requires standstill and expiry of the previous
normal command, so a test does not abruptly reverse a moving robot.

### Completion, interruption and recording

- Initial motion and every direction change require both measured wheel speeds
  to remain within `stop_rpm` for `settle_time_s` (defaults 0.5 RPM / 0.5 s).
- Commands accelerate and decelerate within the configured limits. Distance and
  angle tolerances start final deceleration; they are **not** guaranteed physical
  endpoint accuracy. Coast/overshoot is retained in the stopped-segment result.
- Each movement and each standstill wait has its own `segment_timeout_s`
  (default 60 s). No directed progress for `stall_timeout_s` (5 s) aborts motion.
  Increase the segment deadline for larger distances or slower speeds.
- Absolute encoder data and wheel-speed data have independent freshness checks
  using `feedback_timeout_ms`. A missed control tick beyond that deadline,
  implausible encoder jump, or test speed-transmission error aborts the run.
- A valid external `cmd_vel` during a run ends the test and takes control using
  that same command, with no motor re-enable required. Zero is also a valid
  takeover command. Invalid (non-finite) commands are ignored without interrupting
  the test. The interrupted test is recorded as `aborted` with reason
  `external cmd_vel took control`; it does not resume automatically.
- Completion confirms measured standstill, requests zero speed and leaves the
  motors enabled. You can send a normal command or start another test immediately.
  Previously received normal commands are not replayed. The ordinary command
  watchdog applies after takeover; feedback monitoring remains active throughout.
- Explicit `cancel`, faults, and stop/reset/emergency-stop events still terminate
  the test and close the motion gate. An enable event during a run aborts and
  requires a new explicit enable event. These interruptions request the existing
  ramp emergency stop; **aborted does not mean physical standstill has been
  confirmed**. Re-enable explicitly after these events before subsequent motion.
- Tests refuse to run with CAN disabled. Dry-run is not a simulated robot.

Every accepted run creates a timestamped CSV under `motion_test.log_directory`
(default `/tmp/motor_control_tests` on the machine running the node). Use a
persistent writable directory to retain records across reboot. A file-open
failure rejects the run; a detected write failure aborts it. Recording is
synchronous at the control rate, so use local storage with predictable latency.
A process/power failure may leave a partial log without a terminal row.

The first CSV line is a `#` configuration comment. Remaining rows contain:

- elapsed monotonic seconds, state, segment, progress and last stopped segment
  progress (metres for straight tests, radians for rotation);
- x/y displacement in the test's initial coordinate frame and continuous yaw;
- commanded body velocities, direction-normalized left/right target RPM and
  measured RPM, independent feedback ages, and reason.

Segment numbers start at 1; odd segments are forward/left, even segments are
reverse/right. Segment 0 is the initial standstill wait. When a segment settles,
`last_stopped_segment_progress` captures its achieved distance/angle before the
next segment starts. Subtract the configured target to inspect overshoot.
Compare normalized target/actual RPM with feedback delay in mind; rows capture
the latest received feedback, not time-synchronized motor measurements.

`completed` means the sequence finished; it is not an accuracy pass/fail result.
The CSV final displacement and yaw are **wheel odometry estimates**. Use ground
marks or independent positioning to measure real drift and wheel slip. Version 1
does not provide obstacle detection or an independent hardware watchdog.

The state-machine scenarios can be tested without ROS:

```bash
c++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
  test/test_motion_test.cpp -o /tmp/test_motion_test
/tmp/test_motion_test
```

They are also registered in the normal `colcon test` run, together with ROS
checks for defaults, command takeover, completion, cancellation and rejection
when CAN is disabled.

## Reply handling and diagnostics

On startup with CAN enabled, the node sends one read-only firmware query
(`601#4031200000000000`). It accepts raw ASCII identification only while this
query is pending. Write acknowledgements (`60 index-lo index-hi subindex 00 00
00 00`) and aborts (`80 index-lo index-hi subindex code[4]`, little-endian) are
decoded before text. Protocol-library callers must explicitly pass `true` as
the third `decode` argument while their own firmware query is pending.

Diagnostics include `firmware_query_state`, `firmware_identifier`,
`firmware_identity_matches`, and the configured `rpm_resolution`. The current
firmware identifier (`primax` in the local source) does not identify a build or
report RPM scaling. Therefore `rpm_resolution_verified` remains `false`, even
when the identifier matches. A mismatch or failed query does not gate motion.
Verify the flashed controller's build and speed units before bench operation;
the local firmware contract uses `0.1 RPM/unit`. Do not change a deployed
`1.0` setting based solely on an identifier or these logs.

Rejected-frame warnings include ID, length, bytes, and a readable reason, and
are throttled to once per second. Diagnostics retain `rejected_frame_count`,
`last_rejected_frame`, `last_rejected_reason`, `acknowledgement_count`,
`abort_reply_count`, `last_acknowledgement`, and `last_abort_reply`. Accepted
acknowledgements no longer generate firmware-info messages. Aborts are logged
with their object and code and do not refresh the motor-feedback watchdog.

`command_age_ms`, `command_timeout_ms`, `command_watchdog_expired`, and
`command_timeout_count` describe missing command input separately from
`feedback_age_ms`, `feedback_timeout_ms`, and `feedback_timeout_latched`.
Use `candump -tz can0 > /tmp/motor-can.log` during a controlled bench test to
correlate commands with replies and confirm which firmware protocol is deployed.

## Tests

The normal test run covers differential-drive conversion, proportional RPM
limiting, direction inversion, encoder rollover, CAN command encoding, reply
decoding, malformed-frame rejection, and topic command delivery with dry-run
failure diagnostics (including both emergency-stop values), the observed
printable/binary acknowledgements, unsigned abort codes, pending-query text
decoding, and acknowledgement timeouts/overlap handling:

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
