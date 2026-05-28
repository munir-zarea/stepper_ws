# stepper_ws

ROS 2 workspace for joystick-driven control of a soft underwater robotic manipulator. Two Arduino-based serial bridges handle independent subsystems: stepper motors (buoyancy / tendon actuation) and servos (pouch actuators, gripper, and stiffening mechanism).

---

## Repository Structure

stepper_ws/
├── src/
│   └── stepper_joy/
│       ├── stepper_joy_node.cpp                 # Stepper motor controller (Arduino #1)
│       ├── joystick_servo7_serial_node.cpp      # Servo controller (Arduino #2)
│       └── launch/
│           └── stepper_joy.launch.py            # Combined launch file

---

## Packages

### `stepper_joy`

Contains two nodes and a launch file.

---

## Nodes

### `stepper_joy_node`

Controls up to four stepper motors via serial commands to an Arduino. Left stick X-axis drives the selected active stepper (1–3) in step increments while held outside the deadband. A configurable slack compensation automatically relieves the opposing steppers to account for cable/tendon slack. RB/LB buttons drive stepper 4 independently (e.g., buoyancy syringe). Y button homes all steppers.

**Serial protocol (Arduino #1):**

| Command | Description |
|---|---|
| `P t1 t2 t3 t4\n` | Absolute step targets for all four steppers |
| `Z t4\n` | Stepper 4 position only |
| `EN 1\n` | Enable drivers (optional) |

**Key parameters:**

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyACM0` | Serial port |
| `baud` | `115200` | Baud rate |
| `joy_stepper_id` | `1` | Active stepper driven by left stick (1–3) |
| `joy_steps_per_tick` | `1` | Steps per control loop tick while joystick deflected |
| `deadband` | `0.15` | Joystick deadband |
| `slack_fraction` | `0.05` | Fractional relief applied to opposing steppers |
| `slack_extra_enable` | `true` | Extra slack relief once t2 exceeds threshold |
| `step4_steps_per_tick` | `5.0` | Steps per tick for stepper 4 (RB/LB) |
| `step4_max_travel_steps` | `2000` | Travel limit for stepper 4 in each direction |
| `btn_home` | `3` (Y) | Home all steppers |
| `update_rate_hz` | `100.0` | Control loop rate |

---

### `joystick_servo7_serial_node`

Controls 7 servos via serial commands to a second Arduino. Right stick independently inflates individual pouch actuators (S1–S4) around a configurable home position. X/B buttons trim the shared home up or down continuously. RT/LT full press activates the gripper (S6) and stiffening mechanism (S7) respectively. A button sends a propulsion macro (`PROP\n`) to the Arduino.

**Serial protocol (Arduino #2):**

| Command | Description |
|---|---|
| `SET:s1,s2,s3,s4,s5,s6,s7\n` | Absolute positions (degrees) for all 7 servos |
| `PROP\n` | Propulsion macro (executed on Arduino) |

**Right stick mapping:**

| Input | Servo | Effect |
|---|---|---|
| Stick up | S1 | Inflate pouch 1 |
| Stick down | S3 | Inflate pouch 3 |
| Stick right | S2 | Inflate pouch 2 |
| Stick left | S4 | Inflate pouch 4 |
| Hold X | All pouches | Deflate trim |
| Hold B | All pouches | Inflate trim |
| RT (full press) | S6 | Gripper actuate |
| LT (full press) | S7 | Stiffening actuate |
| A | — | Propulsion macro |

**Key parameters:**

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyACM0` | Serial port |
| `baud` | `115200` | Baud rate |
| `default_pos` | `150` | Servo home position (degrees) |
| `pouch_range_deg` | `150` | Max deflection from home per stick axis |
| `pouch_max_speed_dps` | `20.0` | Pouch slew rate limit (deg/sec) |
| `pouch_trim_speed_dps` | `25.0` | Trim speed when holding X/B (deg/sec) |
| `gripper_speed_dps` | `100.0` | Gripper slew rate limit |
| `stiff_speed_dps` | `120.0` | Stiffening slew rate limit |
| `servo_min` / `servo_max` | `0` / `270` | Servo travel limits (degrees) |
| `update_rate_hz` | `100.0` | Control loop rate |

---

## Launch

The combined launch file brings up all three nodes together with stable `/dev/serial/by-id` paths to avoid port remapping issues on reconnect.

```bash
ros2 launch stepper_joy stepper_joy.launch.py
```

Override serial ports if needed:

```bash
ros2 launch stepper_joy stepper_joy.launch.py \
  stepper_port:=/dev/serial/by-id/usb-Arduino_...-if00 \
  servo_port:=/dev/serial/by-id/usb-Arduino_...-if00
```

---

## Dependencies

- ROS 2 (Humble or later)
- `joy` package (`sudo apt install ros-<distro>-joy`)
- Logitech F710 gamepad (or compatible; axes/buttons remappable via parameters)
- Two Arduino boards running compatible firmware

---

## Building

```bash
cd stepper_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Hardware

| Arduino | Subsystem | Connected to |
|---|---|---|
| #1 (stepper) | 3–4 stepper motors | Tendon drive, buoyancy syringe |
| #2 (servo) | 6–7 servos | Pouch actuators (S1–S4), propulsion (S5), gripper (S6), stiffening (S7) |

Both communicate at **115200 baud** over USB-serial. Stable `/dev/serial/by-id` paths are used in the launch file to ensure correct device assignment regardless of enumeration order.

---

## Notes

- All servo positions are in degrees (0–270). Adjust `servo_min`/`servo_max` to match your hardware's range.
- Slack compensation (`slack_fraction`, `slack_extra_*` params) should be tuned empirically for your tendon routing geometry.
- The `PROP` macro and propulsion behavior are defined in the Arduino firmware; the ROS node sends only the trigger command.
- Both nodes auto-respawn on serial disconnection (2-second delay).
