# Kinetic Lift Mechanism — LilyGO T-ETH-Lite ESP32

Firmware for a two-motor kinetic lift mechanism driven by an ESP32. Both stepper
motors support synchronized homing, Bluetooth calibration/jogging, and automatic
movement between two saved endpoints.

The mechanism is treated as one shared load supported by a motor at each end.
Motion is implemented as non-blocking state machines so limit switches,
Bluetooth commands, and motor pulses continue to be serviced from `loop()`.

## Hardware configuration

| Signal | ESP32 GPIO | Mode |
| --- | ---: | --- |
| Motor 1 pulse | 13 | Output |
| Motor 1 direction | 32 | Output |
| Motor 2 pulse | 14 | Output |
| Motor 2 direction | 33 | Output |
| Motor 1 home limit | 34 | Input |
| Motor 2 home limit | 35 | Input |

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

## Startup sequence

On every power-up, the firmware:

1. Initializes all GPIO with the pulse and direction outputs LOW.
2. Loads saved parameters and calibrated positions from ESP32 NVS.
3. Starts Bluetooth Classic Serial using `KINETIC_LIFT_<DEVICE_ID>`.
4. Starts non-blocking homing for both motors.
5. Sets both live motor-position counters to zero after both switches are reached.
6. Starts the selected working mode after homing completes.

Both motors receive synchronized step edges during homing. When one motor reaches
its own switch, that motor stops while the other continues until its switch is
reached. A 30-second homing timeout stops both motors and reports a critical
fault.

## Working modes

| Value | Mode | Current behavior |
| ---: | --- | --- |
| 0 | Home | Startup homing; no subsequent movement scenario |
| 1 | Manual | Reserved for a later Bluetooth manual-control scenario |
| 2 | Automatic | Continuously moves P1 → P2 → P1 with independent endpoint delays |
| 3 | Jog | Bluetooth synchronized or individual finite-step adjustment |

The selected working mode is stored in NVS. Startup homing always runs before
the saved mode is allowed to move the motors.

## Bluetooth Serial protocol

Commands are case-insensitive and must end with a newline (`\n`). Bluetooth
replies with `OK`, `ERROR`, movement status, or the requested configuration.
Incoming commands are also printed to USB Serial when `DEBUG` is enabled.

### Read the current configuration

```text
GET CONFIG
```

The response includes all motion parameters, current motor positions, and both
motors' saved P1/P2 coordinates.

### Persistent configuration commands

| Command | Range | Unit | Default | Notes |
| --- | ---: | --- | ---: | --- |
| `SET DEVICE_ID <value>` | 1–255 | — | 1 | Bluetooth name changes after restart |
| `SET HOMING_SPEED <value>` | 1–5000 | steps/s | 1000 | Homing speed |
| `SET WORKING_MODE <value>` | 0–3 | — | 0 | Selects the working mode |
| `SET MOVE_SPEED <value>` | 1–5000 | master steps/s | 1000 | Maximum automatic-mode speed |
| `SET ACCELERATION <value>` | 1–50000 | steps/s² | 1000 | Automatic acceleration |
| `SET DECELERATION <value>` | 1–50000 | steps/s² | 1000 | Automatic deceleration |
| `SET P1_DELAY <value>` | 0–600000 | ms | 1000 | Dwell after reaching P1 |
| `SET P2_DELAY <value>` | 0–600000 | ms | 1000 | Dwell after reaching P2 |

For compatibility, the following command sets both endpoint delays to the same
value:

```text
SET POSITION_DELAY 3000
```

All validated configuration values are stored in the NVS namespace
`kinetic_lift` and survive power cycles.

## Jog mode and endpoint calibration

Jogging is available only after homing completes and while working mode `3` is
selected:

```text
SET WORKING_MODE 3
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
by default. Supply a final value to select an increment from 1 to 10000 steps:

```text
JOG BOTH AWAY 100
JOG M1 HOME 2
```

Jogging runs non-blocking at 250 steps/s. Each command is a bounded movement,
which prevents a lost Bluetooth button-release message from leaving a motor
running continuously. A new jog command replaces any unfinished jog movement.
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
