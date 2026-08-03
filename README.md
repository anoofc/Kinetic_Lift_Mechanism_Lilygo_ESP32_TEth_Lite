# Kinetic Lift Mechanism — LilyGO T-ETH-Lite ESP32

Firmware for a two-motor kinetic lift mechanism driven by an ESP32. Both stepper
motors support synchronized homing, Bluetooth calibration/jogging, and automatic
movement between two saved endpoints.

The mechanism is treated as one shared load supported by a motor at each end.
Motor motion is implemented as non-blocking state machines. Bluetooth command
input uses `readStringUntil('\n')`, so an incomplete command can wait for the
Serial timeout before motor state-machine processing resumes.

## Hardware configuration

| Signal | ESP32 GPIO | Mode |
| --- | ---: | --- |
| Motor 1 pulse | 13 | Output |
| Motor 1 direction | 32 | Output |
| Motor 2 pulse | 14 | Output |
| Motor 2 direction | 33 | Output |
| Motor 1 home limit | 34 | Input |
| Motor 2 home limit | 35 | Input |

Ethernet uses the tested RTL8201 configuration from `src/eth_properties.h`:

| Ethernet signal | ESP32 GPIO / value |
| --- | ---: |
| PHY type | RTL8201 |
| PHY address | 0 |
| Reference clock | GPIO0 input |
| MDC | 23 |
| MDIO | 18 |
| PHY power | 12 |
| PHY reset | Not used (`-1`) |

GPIO34 and GPIO35 are input-only pins and do not have internal pull resistors.
The limit-switch circuits must therefore provide suitable external pull
resistors. The current firmware expects both switches to be active LOW.

The configured homing direction is LOW for both motor direction outputs:

```cpp
#define MOTOR_1_HOMING_DIRECTION LOW
#define MOTOR_2_HOMING_DIRECTION LOW
#define MOTOR_1_LIMIT_ACTIVE_STATE LOW
#define MOTOR_2_LIMIT_ACTIVE_STATE LOW
```

Verify both motor directions and limit-switch polarities with the mechanism
unloaded before normal operation.

### Mechanical conversion and limits

The configured mechanism uses 6400 driver steps per motor revolution and a 1204
lead screw with 4 mm travel per revolution:

```text
6400 steps/revolution ÷ 4 mm/revolution = 1600 steps/mm
1600 steps/mm × 400 mm travel = 640000 steps maximum travel
```

Automatic mode has a configurable movement ceiling of 450 motor RPM:

```text
6400 steps/revolution × 450 RPM ÷ 60 = 48000 steps/s
48000 steps/s ÷ 1600 steps/mm = 30 mm/s
```

Maximum configurable acceleration and deceleration are 160000 steps/s², equal
to 100 mm/s². These are software ceilings, not guaranteed safe operating values.
The motor, driver, supply voltage, load, and structure may require substantially
lower settings. Jog mode remains capped at 32000 steps/s (300 RPM or 20 mm/s)
because it starts without an acceleration profile.

## Startup sequence

On every power-up, the firmware:

1. Initializes all GPIO with the pulse and direction outputs LOW.
2. Loads saved parameters and calibrated positions from ESP32 NVS.
3. Starts Bluetooth Classic Serial using `KINETIC_LIFT_<DEVICE_ID>`.
4. Initializes Ethernet with the saved static network configuration and opens
   the UDP input port.
5. Starts non-blocking homing for both motors.
6. Sets both live motor-position counters to zero after both switches are reached.
7. Immediately performs a coordinated move to the saved P1 coordinates.
8. Starts the selected working mode after reaching P1.

Both motors receive synchronized step edges during homing. When one motor reaches
its own switch, that motor stops while the other continues until its switch is
reached. A 30-second homing timeout stops both motors and reports a critical
fault. If P1 has not been calibrated, the mechanism remains at home and reports
that a valid P1 position is required.

If either home-limit switch becomes newly active during Jog, GOTO, or Automatic
movement, both motors stop immediately. The firmware waits 5 seconds without
blocking and then starts synchronized homing. Switch activation is edge-detected,
so a switch that is already held at home can be released normally when beginning
an AWAY move. Jog, GOTO, and position-save commands are unavailable while this
delayed homing is pending.

## Working modes

| Value | Mode | Current behavior |
| ---: | --- | --- |
| 0 | Standby/P1 | Moves to P1 when enabled and accepts manual `GOTO` commands |
| 1 | OSC control | Moves to P1 or P2 from device-addressed UDP OSC messages |
| 2 | Automatic | Continuously moves P1 → P2 → P1 with independent endpoint delays |
| 3 | Jog | Bluetooth synchronized or individual finite-step adjustment |

The selected working mode is stored in NVS. Startup homing always runs before
the saved mode is allowed to move the motors.

## Bluetooth Serial protocol

Commands are case-insensitive and must end with a newline (`\n`). Bluetooth
replies with `OK`, `ERROR`, movement status, or the requested configuration.
Incoming commands are also printed to USB Serial when `DEBUG` is enabled.
Send Bluetooth configuration commands while the mechanism is stationary.

### Read the current configuration

```text
GET CONFIG
```

The response includes all motion and Ethernet/UDP parameters, current motor
positions, and both motors' saved P1/P2 coordinates.

### Trigger homing

```text
HOME
```

This immediately stops any Jog or Automatic movement and starts the same
non-blocking homing sequence used at power-up. After homing completes, the lift
moves to P1 before the currently selected working mode is allowed to resume.

### Restart the ESP32

```text
RESTART
```

This stops active motor pulse generation, acknowledges the command over
Bluetooth, and performs a full ESP32 software restart. The normal power-up
sequence then reloads NVS settings, initializes Ethernet, and starts homing.

### Move manually to a saved position

The following commands are available in working modes `0` and `3`:

```text
GOTO P1
GOTO P2
```

`GO P1` and `GO P2` are accepted as aliases. These commands use the same
coordinated acceleration/deceleration trajectory as Automatic mode. Both motors
move to their individually calibrated coordinates and arrive together. Entering
working mode `0` also triggers a move to P1:

```text
SET WORKING_MODE 0
```

### Persistent configuration commands

| Command | Range | Unit | Default | Notes |
| --- | ---: | --- | ---: | --- |
| `SET DEVICE_ID <value>` | 1–255 | — | 1 | Bluetooth name changes after restart |
| `SET HOMING_SPEED <value>` | 1–5000 | steps/s | 1000 | Homing speed |
| `SET WORKING_MODE <value>` | 0–3 | — | 0 | Selects the working mode |
| `SET MOVE_SPEED <value>` | 1–48000 | master steps/s | 1000 | Maximum automatic-mode speed |
| `SET JOG_SPEED <value>` | 1–32000 | steps/s | 6400 | Constant Jog-mode speed |
| `SET ACCELERATION <value>` | 1–160000 | steps/s² | 1000 | Automatic acceleration |
| `SET DECELERATION <value>` | 1–160000 | steps/s² | 1000 | Automatic deceleration |
| `SET P1_DELAY <value>` | 0–600000 | ms | 1000 | Dwell after reaching P1 |
| `SET P2_DELAY <value>` | 0–600000 | ms | 1000 | Dwell after reaching P2 |
| `SET DEVICE_IP <address>` | Valid IPv4 host | — | 192.168.1.100 | ESP32 static Ethernet IP |
| `SET GATEWAY <address>` | Valid IPv4 host | — | 192.168.1.1 | Ethernet gateway |
| `SET SUBNET <mask>` | Valid IPv4 mask | — | 255.255.255.0 | Ethernet subnet mask |
| `SET UDP_IN_PORT <value>` | 1–65535 | — | 8000 | Local OSC receive port |
| `SET UDP_OUT_IP <address>` | Valid IPv4 host | — | 192.168.1.101 | OSC destination IP |
| `SET UDP_OUT_PORT <value>` | 1–65535 | — | 8001 | OSC destination port |

For compatibility, the following command sets both endpoint delays to the same
value:

```text
SET POSITION_DELAY 3000
```

All validated configuration values are stored in the NVS namespace
`kinetic_lift` and survive power cycles.

Network changes are saved immediately but take effect after restarting the
device. For example:

```text
SET DEVICE_IP 192.168.10.50
SET GATEWAY 192.168.10.1
SET SUBNET 255.255.255.0
SET UDP_IN_PORT 9000
SET UDP_OUT_IP 192.168.10.20
SET UDP_OUT_PORT 9001
GET CONFIG
```

Ethernet startup does not wait for a physical link, so it does not delay the
non-blocking homing sequence.

## OSC control mode

Working mode `1` accepts device-addressed position triggers on the configured
`UDP_IN_PORT`. Select it through Bluetooth:

```text
SET WORKING_MODE 1
```

Use these OSC addresses, replacing `<deviceID>` with the configured numeric
`DEVICE_ID`:

```text
/trigger/<deviceID>/p1
/trigger/<deviceID>/p2
```

For example, device ID `1` responds to:

```text
/trigger/1/p1
/trigger/1/p2
```

Position triggers for other device IDs are ignored. They are also ignored
outside working mode `1`, before homing has completed, or when the requested
position has not been calibrated. OSC processing is non-blocking and motor-step
updates run before checking for a new UDP packet.

The separate device-addressed restart command is:

```text
/restart/<deviceID>
```

For example, `/restart/1` restarts only device ID `1`. Restart is an
administrative command and is accepted in every working mode. Incoming OSC
messages larger than 256 bytes or containing invalid OSC data are discarded.

## Jog mode and endpoint calibration

Jogging is available only after homing completes and while working mode `3` is
selected:

```text
SET WORKING_MODE 3
SET JOG_SPEED 6400
```

### Jog both motors together

These commands are suitable for the application's two primary jog buttons:

```text
JOG BOTH AWAY
JOG BOTH HOME
```

### Fine-adjust one motor

```text
JOG MOTOR1 AWAY
JOG MOTOR1 HOME
JOG MOTOR2 AWAY
JOG MOTOR2 HOME
```

`M1` and `M2` are accepted as shorter target names. A jog command moves 10 steps
by default. Supply a final value to select an increment from 1 to 640000 steps:

```text
JOG BOTH AWAY 100
JOG M1 HOME 2
```

Jogging runs non-blocking at the saved `JOG_SPEED`. Its default is 6400 steps/s,
equal to 4 mm/s, and it is configurable up to 32000 steps/s (20 mm/s). Jogging
uses constant speed without the Automatic mode acceleration profile, so increase
this value carefully. Each command is a bounded movement, which prevents a lost
Bluetooth button-release message from leaving a motor running continuously.
AWAY jogging is also clamped to the calculated 640000-step travel range. A
synchronized `BOTH` jog uses the same safe step count for both motors. A new jog
command replaces any unfinished jog movement.
Use the following command when an immediate stop is required:

```text
JOG STOP
```

When a jog finishes, Bluetooth reports both live positions:

```text
JOG_DONE: M1=120 M2=118
```

When jogging toward HOME, each motor stops independently at its home switch and
its live position is reset to zero.

### Save P1 and P2

P1 is the endpoint nearer the homing switches. P2 is the endpoint at the other
end of travel. After jogging and individually aligning both ends of the load,
save the current position pair:

```text
SAVE P1
```

Then jog both motors away from home, fine-adjust each side at the far endpoint,
and save:

```text
SAVE P2
```

Each save records separate coordinates for Motor 1 and Motor 2. Both coordinate
pairs and their validity flags are stored in NVS. The firmware requires each P1
coordinate to be closer to home than its corresponding P2 coordinate.

## Automatic mode

Automatic mode requires valid saved P1 and P2 coordinates:

```text
SET MOVE_SPEED 1200
SET ACCELERATION 800
SET DECELERATION 1000
SET P1_DELAY 2000
SET P2_DELAY 5000
SET WORKING_MODE 2
```

After homing or after changing from another mode to mode `2`, the sequence is:

```text
Move to P1
Wait P1_DELAY
Move to P2
Wait P2_DELAY
Repeat
```

Automatic movement uses a shared acceleration/deceleration profile. If the two
motors have slightly different calibrated travel counts, proportional coordinated
stepping distributes the correction across the move so both motors reach their
individual target coordinates together.

Typical Bluetooth status messages are:

```text
AUTO_MOVING_TO: P1
AUTO_REACHED: P1 M1=10 M2=12
AUTO_MOVING_TO: P2
AUTO_REACHED: P2 M1=5000 M2=4996
```

If P1 or P2 has not been calibrated, automatic mode does not move and reports:

```text
AUTO_WAITING: Valid P1 and P2 positions are required
```

An unexpected home-switch activation before a nonzero automatic target stops
both motors and places automatic mode in a fault state.

## Debug and critical messages

Normal USB Serial diagnostic messages are compiled only when `DEBUG` is enabled:

```cpp
#define DEBUG 1
```

Set it to `0` to disable normal diagnostic output. Critical Bluetooth, NVS,
homing, and automatic-motion errors continue to print regardless of this setting.
Bluetooth command responses are always transmitted because they are part of the
control protocol.

## Build and upload

This is a PlatformIO Arduino project configured for the ESP32-based T-ETH-Lite
environment:

```bash
pio run
pio run --target upload
pio device monitor
```

The configured Serial Monitor speed is 115200 baud.

## Safety notes

- Test motor direction, switch polarity, and step scaling without the shared load
  before powered mechanism testing.
- Use externally biased limit-switch signals on GPIO34 and GPIO35.
- Calibrate P1 and P2 at a conservative jog increment and speed.
- Individual-motor jogging intentionally changes alignment; use only small
  increments while observing the mechanism.
- Configure conservative speed, acceleration, and deceleration values before
  enabling automatic mode.
- The firmware cannot detect the far end mechanically because only home-limit
  switches are currently defined. A correct P2 calibration is therefore a
  critical software travel limit.
