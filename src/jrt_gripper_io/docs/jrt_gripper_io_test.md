# JRT JEGB-4285P-A-X340 Tool I/O Test

This package controls a JRT JEGB-4285P-A-X340 gripper through Doosan A0509
Tool Digital Outputs. It does not command robot arm motion.

This implementation only maps two Doosan Tool DO channels to two gripper
commands:

- Tool DO1 / service index 1: CLOSE command
- Tool DO2 / service index 2: OPEN command
- `value: 0`: OFF
- `value: 1`: ON

The mapping is configurable with `close_do_index` and `open_do_index`. The code
does not define JRT M16 connector pin numbers or cable colors; wire those from
the JRT cable/manual pinout.

## Safety Notes

- DO1/DO2 are command signal lines, not gripper motor power lines.
- JEGB-4285 requires DC 24V power and can draw up to 2.6A input current.
- Do not supply motor current through Doosan Tool DO.
- Do not turn CLOSE and OPEN inputs ON at the same time.
- Verify 24V PNP output behavior and common ground with a multimeter before
  connecting the gripper input lines.
- Confirm JEGB input current is within the Doosan Tool DO output rating.
- Configure Tool Weight, TCP, and Tool Shape before real robot operation.
- Consider object drop risk when power, servo, or Tool I/O state changes.

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select jrt_gripper_io
source install/setup.bash
```

## Discover Doosan Tool I/O

```bash
ros2 service list | grep tool_digital
ros2 service type /dsr01/io/set_tool_digital_output
ros2 interface show dsr_msgs2/srv/SetToolDigitalOutput
```

Expected service:

```text
/dsr01/io/set_tool_digital_output
```

## Manual Pulse Tests

Use these only after the gripper area is clear.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

All off:

```bash
src/jrt_gripper_io/scripts/gripper_all_off.sh
```

Open pulse:

```bash
src/jrt_gripper_io/scripts/gripper_open_pulse.sh
```

Close pulse:

```bash
src/jrt_gripper_io/scripts/gripper_close_pulse.sh
```

Sequence:

```bash
src/jrt_gripper_io/scripts/gripper_test_sequence.sh
```

Override defaults:

```bash
SERVICE_NAME=/dsr01/io/set_tool_digital_output \
CLOSE_INDEX=1 \
OPEN_INDEX=2 \
PULSE_SEC=0.20 \
src/jrt_gripper_io/scripts/gripper_close_pulse.sh
```

Equivalent close pulse:

```bash
ros2 service call /dsr01/io/set_tool_digital_output dsr_msgs2/srv/SetToolDigitalOutput "{index: 2, value: 0}"
ros2 service call /dsr01/io/set_tool_digital_output dsr_msgs2/srv/SetToolDigitalOutput "{index: 1, value: 1}"
sleep 0.2
ros2 service call /dsr01/io/set_tool_digital_output dsr_msgs2/srv/SetToolDigitalOutput "{index: 1, value: 0}"
```

Equivalent open pulse:

```bash
ros2 service call /dsr01/io/set_tool_digital_output dsr_msgs2/srv/SetToolDigitalOutput "{index: 1, value: 0}"
ros2 service call /dsr01/io/set_tool_digital_output dsr_msgs2/srv/SetToolDigitalOutput "{index: 2, value: 1}"
sleep 0.2
ros2 service call /dsr01/io/set_tool_digital_output dsr_msgs2/srv/SetToolDigitalOutput "{index: 2, value: 0}"
```

If open/close are reversed, swap `CLOSE_INDEX` and `OPEN_INDEX` or launch with
swapped `close_do_index` and `open_do_index`.

## Joy Button Index Check

For a `sensor_msgs/msg/Joy` bridge:

```bash
ros2 topic echo /joy
```

Press the Quest A button and find the `buttons[]` index that changes from 0 to
1. Use that as `a_button_index`. Press B and use that changed index as
`b_button_index`.

Do not hardcode Meta Quest A/B indices in code; keep them launch parameters.

## Dry-Run Node Test

```bash
ros2 launch jrt_gripper_io jrt_gripper_io.launch.py \
  dry_run:=true \
  joy_topic:=/joy \
  a_button_index:=<A_INDEX> \
  b_button_index:=<B_INDEX>
```

Expected:

- A: CLOSE pulse plan
- B: OPEN pulse plan
- neither or both: STOP / all off

Dry-run mode logs planned Tool DO writes and does not call the Doosan service.

## Real Node Test

```bash
ros2 launch jrt_gripper_io jrt_gripper_io.launch.py \
  dry_run:=false \
  joy_topic:=/joy \
  a_button_index:=<A_INDEX> \
  b_button_index:=<B_INDEX> \
  set_tool_do_service:=/dsr01/io/set_tool_digital_output \
  close_do_index:=1 \
  open_do_index:=2 \
  command_mode:=pulse \
  pulse_sec:=0.20 \
  interlock_sec:=0.05 \
  debounce_sec:=0.30
```

Relevant parameters:

- `close_do_index`: Doosan Tool DO index for close, default `1`
- `open_do_index`: Doosan Tool DO index for open, default `2`
- `active_value`: ON value, default `1`
- `inactive_value`: OFF value, default `0`
- `command_mode`: `pulse` or `level`, default `pulse`
- `pulse_sec`: active pulse duration, default `0.20`
- `interlock_sec`: delay after turning the opposite DO OFF, default `0.05`
- `debounce_sec`: minimum time between close/open commands, default `0.30`
- `startup_all_off`: send both DOs OFF at node start, default `true`
- `shutdown_all_off`: send both DOs OFF at shutdown, default `true`
- `dry_run`: log only when `true`, default `false`

`pulse` mode:

- close: open OFF, wait `interlock_sec`, close ON, wait `pulse_sec`, close OFF
- open: close OFF, wait `interlock_sec`, open ON, wait `pulse_sec`, open OFF

`level` mode:

- close: open OFF, wait `interlock_sec`, close ON and hold
- open: close OFF, wait `interlock_sec`, open ON and hold
- stop: both OFF

## Robot Bringup With Quest2ROS Inputs

Terminal 1:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

Terminal 2:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jrt_gripper_io jrt_gripper_robot_bringup.launch.py \
  dry_run:=true
```

The default Quest2ROS input topic is `/q2r_right_hand_inputs`:

- `button_lower`: A button on the right controller
- `button_upper`: B button on the right controller

Real Tool I/O:

```bash
ros2 launch jrt_gripper_io jrt_gripper_robot_bringup.launch.py \
  dry_run:=false \
  close_do_index:=1 \
  open_do_index:=2
```

## Troubleshooting

If no movement:

1. Check gripper DC 24V power.
2. Check Tool DO service name and type.
3. Check DO index wiring.
4. Check JRT input common / GND wiring.
5. Check PNP compatibility.
6. Check whether the JEGB input expects pulse or level.
7. Check whether the gripper input is wired to Tool I/O or cabinet I/O.
8. Check whether `close_do_index` and `open_do_index` need to be swapped.

If the gripper moves at Doosan bringup even with `start_gripper_io:=false`, that
is not caused by this package's gripper node. Inspect Tool I/O power/reset and
the JEGB wiring state during Doosan hardware initialization.
