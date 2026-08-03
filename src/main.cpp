#ifndef DEBUG
#define DEBUG 1
#endif

#define MOTOR_1_PUL 13
#define MOTOR_1_DIR 32

#define MOTOR_2_PUL 14
#define MOTOR_2_DIR 33

#define MOTOR_1_LIMIT 34
#define MOTOR_2_LIMIT 35

#define MOTOR_1_HOMING_DIRECTION LOW
#define MOTOR_2_HOMING_DIRECTION LOW
#define MOTOR_1_LIMIT_ACTIVE_STATE LOW
#define MOTOR_2_LIMIT_ACTIVE_STATE LOW

#include <Arduino.h>

#include <BluetoothSerial.h>
#include <Preferences.h>

#if DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) ((void)0)
#define DEBUG_PRINTLN(...) ((void)0)
#endif

BluetoothSerial SerialBT;
Preferences pref;

constexpr char NVS_NAMESPACE[] = "kinetic_lift";
constexpr char NVS_DEVICE_ID_KEY[] = "device_id";
constexpr char NVS_HOMING_SPEED_KEY[] = "home_speed";
constexpr char NVS_WORKING_MODE_KEY[] = "work_mode";
constexpr char NVS_JOG_SPEED_KEY[] = "jog_speed";
constexpr char NVS_MOTOR_1_P1_KEY[] = "m1_p1";
constexpr char NVS_MOTOR_2_P1_KEY[] = "m2_p1";
constexpr char NVS_MOTOR_1_P2_KEY[] = "m1_p2";
constexpr char NVS_MOTOR_2_P2_KEY[] = "m2_p2";
constexpr char NVS_P1_SAVED_KEY[] = "p1_saved";
constexpr char NVS_P2_SAVED_KEY[] = "p2_saved";
constexpr char NVS_MOVE_SPEED_KEY[] = "move_speed";
constexpr char NVS_ACCELERATION_KEY[] = "acceleration";
constexpr char NVS_DECELERATION_KEY[] = "deceleration";
constexpr char NVS_LEGACY_POSITION_DELAY_KEY[] = "pos_delay";
constexpr char NVS_P1_DELAY_KEY[] = "p1_delay";
constexpr char NVS_P2_DELAY_KEY[] = "p2_delay";

constexpr uint8_t DEFAULT_DEVICE_ID = 1;
constexpr uint32_t DEFAULT_HOMING_SPEED = 1000;
constexpr uint8_t DEFAULT_WORKING_MODE = 0;
constexpr uint32_t DEFAULT_MOVE_SPEED = 1000;
constexpr uint32_t DEFAULT_JOG_SPEED = 6400;
constexpr uint32_t DEFAULT_ACCELERATION = 1000;
constexpr uint32_t DEFAULT_DECELERATION = 1000;
constexpr uint32_t DEFAULT_P1_DELAY_MS = 1000;
constexpr uint32_t DEFAULT_P2_DELAY_MS = 1000;

constexpr uint32_t STEPS_PER_REVOLUTION = 6400;
constexpr uint32_t LEAD_SCREW_LEAD_MM = 4;
constexpr uint32_t TRAVEL_LENGTH_MM = 400;
constexpr uint32_t STEPS_PER_MM =
    STEPS_PER_REVOLUTION / LEAD_SCREW_LEAD_MM;
constexpr uint32_t MAX_TRAVEL_STEPS = STEPS_PER_MM * TRAVEL_LENGTH_MM;
constexpr uint32_t MAX_MOVE_MOTOR_RPM = 450;
constexpr uint32_t MAX_JOG_MOTOR_RPM = 300;
constexpr uint32_t MAX_LINEAR_ACCELERATION_MM_S2 = 100;

constexpr uint8_t MIN_DEVICE_ID = 1;
constexpr uint8_t MAX_DEVICE_ID = 255;
constexpr uint32_t MIN_HOMING_SPEED = 1;
constexpr uint32_t MAX_HOMING_SPEED = 5000;
constexpr uint8_t MIN_WORKING_MODE = 0;
constexpr uint8_t MAX_WORKING_MODE = 3;
constexpr uint8_t AUTO_WORKING_MODE = 2;
constexpr uint8_t JOG_WORKING_MODE = 3;
constexpr uint32_t MIN_MOVE_SPEED = 1;
constexpr uint32_t MAX_MOVE_SPEED =
    STEPS_PER_REVOLUTION * MAX_MOVE_MOTOR_RPM / 60;
constexpr uint32_t MIN_JOG_SPEED = 1;
constexpr uint32_t MAX_JOG_SPEED =
    STEPS_PER_REVOLUTION * MAX_JOG_MOTOR_RPM / 60;
constexpr uint32_t MIN_ACCELERATION = 1;
constexpr uint32_t MAX_ACCELERATION =
    STEPS_PER_MM * MAX_LINEAR_ACCELERATION_MM_S2;
constexpr uint32_t MIN_DECELERATION = 1;
constexpr uint32_t MAX_DECELERATION =
    STEPS_PER_MM * MAX_LINEAR_ACCELERATION_MM_S2;
constexpr uint32_t MAX_ENDPOINT_DELAY_MS = 600000;

uint8_t device_id = DEFAULT_DEVICE_ID;
uint32_t motor_homing_speed = DEFAULT_HOMING_SPEED; // Steps per second.
uint8_t working_mode = DEFAULT_WORKING_MODE;
uint32_t move_speed = DEFAULT_MOVE_SPEED; // Master steps per second.
uint32_t jog_speed = DEFAULT_JOG_SPEED; // Steps per second.
uint32_t acceleration = DEFAULT_ACCELERATION; // Master steps per second squared.
uint32_t deceleration = DEFAULT_DECELERATION; // Master steps per second squared.
uint32_t p1_delay_ms = DEFAULT_P1_DELAY_MS;
uint32_t p2_delay_ms = DEFAULT_P2_DELAY_MS;
bool preferences_ready = false;

// Working modes: 0 Home, 1 Manual (Bluetooth), 2 Auto (looping through a sequence of positions), 3 Jog (Bluetooth)

int32_t motor_1_position = 0;
int32_t motor_2_position = 0;
int32_t motor_1_p1 = 0;
int32_t motor_2_p1 = 0;
int32_t motor_1_p2 = 0;
int32_t motor_2_p2 = 0;
bool p1_saved = false;
bool p2_saved = false;

constexpr uint32_t STEP_PULSE_WIDTH_US = 10;
constexpr uint32_t HOMING_TIMEOUT_MS = 30000;
constexpr uint32_t LIMIT_TRIGGER_REHOME_DELAY_MS = 5000;
constexpr uint32_t DEFAULT_JOG_STEPS = 10;
constexpr uint32_t MAX_JOG_STEPS = MAX_TRAVEL_STEPS;
constexpr float MIN_PROFILE_SPEED = 10.0F;

enum class HomingState : uint8_t {
  IDLE,
  RUNNING,
  COMPLETE,
  FAULT
};

HomingState homing_state = HomingState::IDLE;
bool motor_1_homing = false;
bool motor_2_homing = false;
bool homing_pulse_high = false;
uint32_t homing_started_ms = 0;
uint32_t last_homing_step_us = 0;
uint32_t homing_pulse_started_us = 0;
bool previous_motor_1_limit_triggered = false;
bool previous_motor_2_limit_triggered = false;
bool limit_rehome_pending = false;
uint32_t limit_rehome_requested_ms = 0;

enum class JogDirection : int8_t {
  HOME = -1,
  AWAY = 1
};

bool motor_1_jogging = false;
bool motor_2_jogging = false;
bool jog_move_in_progress = false;
JogDirection jog_direction = JogDirection::AWAY;
uint32_t motor_1_jog_steps_remaining = 0;
uint32_t motor_2_jog_steps_remaining = 0;
bool jog_pulse_high = false;
uint32_t last_jog_step_us = 0;
uint32_t jog_pulse_started_us = 0;

enum class AutoState : uint8_t {
  IDLE,
  WAITING_FOR_POSITIONS,
  MOVING_TO_P1,
  WAITING_AT_P1,
  MOVING_TO_P2,
  WAITING_AT_P2,
  FAULT
};

AutoState auto_state = AutoState::IDLE;
int32_t auto_motor_1_target = 0;
int32_t auto_motor_2_target = 0;
int8_t auto_motor_1_direction = 0;
int8_t auto_motor_2_direction = 0;
uint32_t auto_motor_1_steps = 0;
uint32_t auto_motor_2_steps = 0;
uint32_t auto_total_master_steps = 0;
uint32_t auto_master_steps_completed = 0;
uint32_t auto_motor_1_accumulator = 0;
uint32_t auto_motor_2_accumulator = 0;
bool auto_motor_1_pulse_high = false;
bool auto_motor_2_pulse_high = false;
bool auto_pulse_high = false;
bool auto_target_is_p1 = true;
uint32_t last_auto_step_us = 0;
uint32_t auto_pulse_started_us = 0;
uint32_t auto_position_reached_ms = 0;

void stopJog();
bool startJog(bool jog_motor_1, bool jog_motor_2,
              JogDirection direction, uint32_t steps);
void processJogCommand(String arguments);
void processSavePositionCommand(String arguments);
void processGotoCommand(String arguments);
void stopAutoMode();
void resetAutoMode();
bool autoMoveIsActive();
bool startSavedPositionMove(bool target_is_p1);
void startAutoMove(int32_t motor_1_target, int32_t motor_2_target,
                   bool target_is_p1);
void startHoming();
void monitorMovementLimitSwitches();
void updateDelayedRehoming();


// BLUETOOTH SERIAL FUNCTIONS

void sendConfiguration() {
  SerialBT.println("DEVICE_ID=" + String(device_id));
  SerialBT.println("HOMING_SPEED=" + String(motor_homing_speed) + " steps/s");
  SerialBT.println("WORKING_MODE=" + String(working_mode));
  SerialBT.println("MOVE_SPEED=" + String(move_speed) + " steps/s");
  SerialBT.println("JOG_SPEED=" + String(jog_speed) + " steps/s");
  SerialBT.println("ACCELERATION=" + String(acceleration) + " steps/s^2");
  SerialBT.println("DECELERATION=" + String(deceleration) + " steps/s^2");
  SerialBT.println("P1_DELAY=" + String(p1_delay_ms) + " ms");
  SerialBT.println("P2_DELAY=" + String(p2_delay_ms) + " ms");
  SerialBT.println("MOTOR_1_POSITION=" + String(motor_1_position));
  SerialBT.println("MOTOR_2_POSITION=" + String(motor_2_position));
  SerialBT.println("P1=" + String(p1_saved ? "SET" : "NOT_SET") +
                   ", M1=" + String(motor_1_p1) +
                   ", M2=" + String(motor_2_p1));
  SerialBT.println("P2=" + String(p2_saved ? "SET" : "NOT_SET") +
                   ", M1=" + String(motor_1_p2) +
                   ", M2=" + String(motor_2_p2));
}

bool parseUnsignedValue(const String &text, uint32_t &value) {
  if (text.isEmpty()) {
    return false;
  }

  uint32_t parsed_value = 0;
  for (size_t index = 0; index < text.length(); ++index) {
    const char character = text.charAt(index);
    if (character < '0' || character > '9') {
      return false;
    }

    const uint8_t digit = static_cast<uint8_t>(character - '0');
    if (parsed_value > (UINT32_MAX - digit) / 10) {
      return false;
    }
    parsed_value = parsed_value * 10 + digit;
  }

  value = parsed_value;
  return true;
}

bool saveDeviceId(uint8_t value) {
  return preferences_ready &&
         pref.putUChar(NVS_DEVICE_ID_KEY, value) == sizeof(value);
}

bool saveHomingSpeed(uint32_t value) {
  return preferences_ready &&
         pref.putUInt(NVS_HOMING_SPEED_KEY, value) == sizeof(value);
}

bool saveWorkingMode(uint8_t value) {
  return preferences_ready &&
         pref.putUChar(NVS_WORKING_MODE_KEY, value) == sizeof(value);
}

bool saveMoveSpeed(uint32_t value) {
  return preferences_ready &&
         pref.putUInt(NVS_MOVE_SPEED_KEY, value) == sizeof(value);
}

bool saveJogSpeed(uint32_t value) {
  return preferences_ready &&
         pref.putUInt(NVS_JOG_SPEED_KEY, value) == sizeof(value);
}

bool saveAcceleration(uint32_t value) {
  return preferences_ready &&
         pref.putUInt(NVS_ACCELERATION_KEY, value) == sizeof(value);
}

bool saveDeceleration(uint32_t value) {
  return preferences_ready &&
         pref.putUInt(NVS_DECELERATION_KEY, value) == sizeof(value);
}

bool saveP1Delay(uint32_t value) {
  return preferences_ready &&
         pref.putUInt(NVS_P1_DELAY_KEY, value) == sizeof(value);
}

bool saveP2Delay(uint32_t value) {
  return preferences_ready &&
         pref.putUInt(NVS_P2_DELAY_KEY, value) == sizeof(value);
}

bool savePositionPair(bool save_p1) {
  if (!preferences_ready) {
    return false;
  }

  if (save_p1) {
    if (pref.putBool(NVS_P1_SAVED_KEY, false) != 1) {
      return false;
    }
    p1_saved = false;
    const bool motor_1_saved =
        pref.putInt(NVS_MOTOR_1_P1_KEY, motor_1_position) ==
        sizeof(motor_1_position);
    const bool motor_2_saved =
        pref.putInt(NVS_MOTOR_2_P1_KEY, motor_2_position) ==
        sizeof(motor_2_position);
    const bool flag_saved = pref.putBool(NVS_P1_SAVED_KEY, true) == 1;
    if (!motor_1_saved || !motor_2_saved || !flag_saved) {
      return false;
    }

    motor_1_p1 = motor_1_position;
    motor_2_p1 = motor_2_position;
    p1_saved = true;
    return true;
  }

  if (pref.putBool(NVS_P2_SAVED_KEY, false) != 1) {
    return false;
  }
  p2_saved = false;
  const bool motor_1_saved =
      pref.putInt(NVS_MOTOR_1_P2_KEY, motor_1_position) ==
      sizeof(motor_1_position);
  const bool motor_2_saved =
      pref.putInt(NVS_MOTOR_2_P2_KEY, motor_2_position) ==
      sizeof(motor_2_position);
  const bool flag_saved = pref.putBool(NVS_P2_SAVED_KEY, true) == 1;
  if (!motor_1_saved || !motor_2_saved || !flag_saved) {
    return false;
  }

  motor_1_p2 = motor_1_position;
  motor_2_p2 = motor_2_position;
  p2_saved = true;
  return true;
}

void processData(String data) {
  data.trim();

  if (data.equalsIgnoreCase("GET CONFIG")) {
    sendConfiguration();
    return;
  }

  if (data.equalsIgnoreCase("HOME")) {
    startHoming();
    SerialBT.println("OK: Homing triggered");
    return;
  }

  const int first_separator = data.indexOf(' ');
  if (first_separator < 0) {
    SerialBT.println("ERROR: Use SET, JOG, GOTO, SAVE, HOME, or GET CONFIG");
    return;
  }

  String command = data.substring(0, first_separator);
  String arguments = data.substring(first_separator + 1);
  command.trim();
  arguments.trim();

  if (command.equalsIgnoreCase("JOG")) {
    processJogCommand(arguments);
    return;
  }

  if (command.equalsIgnoreCase("SAVE")) {
    processSavePositionCommand(arguments);
    return;
  }

  if (command.equalsIgnoreCase("GOTO") || command.equalsIgnoreCase("GO")) {
    processGotoCommand(arguments);
    return;
  }

  if (!command.equalsIgnoreCase("SET")) {
    SerialBT.println("ERROR: Unknown command. Use SET, JOG, GOTO, SAVE, HOME, or GET CONFIG");
    return;
  }

  if (!preferences_ready) {
    SerialBT.println("ERROR: NVS parameter storage is unavailable");
    return;
  }

  const int second_separator = arguments.indexOf(' ');
  if (second_separator < 0) {
    SerialBT.println("ERROR: Use SET <PARAMETER> <VALUE>");
    return;
  }

  String parameter = arguments.substring(0, second_separator);
  String value_text = arguments.substring(second_separator + 1);
  parameter.trim();
  value_text.trim();

  uint32_t value = 0;
  if (!parseUnsignedValue(value_text, value)) {
    SerialBT.println("ERROR: Value must be an unsigned integer");
    return;
  }

  if (parameter.equalsIgnoreCase("DEVICE_ID")) {
    if (value < MIN_DEVICE_ID || value > MAX_DEVICE_ID) {
      SerialBT.println("ERROR: DEVICE_ID range is 1-255");
      return;
    }

    const uint8_t new_device_id = static_cast<uint8_t>(value);
    if (new_device_id != device_id && !saveDeviceId(new_device_id)) {
      SerialBT.println("ERROR: Failed to save DEVICE_ID to NVS");
      return;
    }
    device_id = new_device_id;
    SerialBT.println("OK: DEVICE_ID=" + String(device_id) +
                     " (Bluetooth name changes after restart)");
    return;
  }

  if (parameter.equalsIgnoreCase("HOMING_SPEED")) {
    if (value < MIN_HOMING_SPEED || value > MAX_HOMING_SPEED) {
      SerialBT.println("ERROR: HOMING_SPEED range is 1-5000 steps/s");
      return;
    }

    if (value != motor_homing_speed && !saveHomingSpeed(value)) {
      SerialBT.println("ERROR: Failed to save HOMING_SPEED to NVS");
      return;
    }
    motor_homing_speed = value;
    SerialBT.println("OK: HOMING_SPEED=" + String(motor_homing_speed) +
                     " steps/s");
    return;
  }

  if (parameter.equalsIgnoreCase("MOVE_SPEED")) {
    if (value < MIN_MOVE_SPEED || value > MAX_MOVE_SPEED) {
      SerialBT.println("ERROR: MOVE_SPEED range is 1-" +
                       String(MAX_MOVE_SPEED) + " steps/s");
      return;
    }
    if (value != move_speed && !saveMoveSpeed(value)) {
      SerialBT.println("ERROR: Failed to save MOVE_SPEED to NVS");
      return;
    }
    move_speed = value;
    SerialBT.println("OK: MOVE_SPEED=" + String(move_speed) + " steps/s");
    return;
  }

  if (parameter.equalsIgnoreCase("JOG_SPEED")) {
    if (value < MIN_JOG_SPEED || value > MAX_JOG_SPEED) {
      SerialBT.println("ERROR: JOG_SPEED range is 1-" +
                       String(MAX_JOG_SPEED) + " steps/s");
      return;
    }
    if (value != jog_speed && !saveJogSpeed(value)) {
      SerialBT.println("ERROR: Failed to save JOG_SPEED to NVS");
      return;
    }
    jog_speed = value;
    SerialBT.println("OK: JOG_SPEED=" + String(jog_speed) + " steps/s");
    return;
  }

  if (parameter.equalsIgnoreCase("ACCELERATION")) {
    if (value < MIN_ACCELERATION || value > MAX_ACCELERATION) {
      SerialBT.println("ERROR: ACCELERATION range is 1-" +
                       String(MAX_ACCELERATION) + " steps/s^2");
      return;
    }
    if (value != acceleration && !saveAcceleration(value)) {
      SerialBT.println("ERROR: Failed to save ACCELERATION to NVS");
      return;
    }
    acceleration = value;
    SerialBT.println("OK: ACCELERATION=" + String(acceleration) +
                     " steps/s^2");
    return;
  }

  if (parameter.equalsIgnoreCase("DECELERATION")) {
    if (value < MIN_DECELERATION || value > MAX_DECELERATION) {
      SerialBT.println("ERROR: DECELERATION range is 1-" +
                       String(MAX_DECELERATION) + " steps/s^2");
      return;
    }
    if (value != deceleration && !saveDeceleration(value)) {
      SerialBT.println("ERROR: Failed to save DECELERATION to NVS");
      return;
    }
    deceleration = value;
    SerialBT.println("OK: DECELERATION=" + String(deceleration) +
                     " steps/s^2");
    return;
  }

  if (parameter.equalsIgnoreCase("P1_DELAY")) {
    if (value > MAX_ENDPOINT_DELAY_MS) {
      SerialBT.println("ERROR: P1_DELAY range is 0-600000 ms");
      return;
    }
    if (value != p1_delay_ms && !saveP1Delay(value)) {
      SerialBT.println("ERROR: Failed to save P1_DELAY to NVS");
      return;
    }
    p1_delay_ms = value;
    SerialBT.println("OK: P1_DELAY=" + String(p1_delay_ms) + " ms");
    return;
  }

  if (parameter.equalsIgnoreCase("P2_DELAY")) {
    if (value > MAX_ENDPOINT_DELAY_MS) {
      SerialBT.println("ERROR: P2_DELAY range is 0-600000 ms");
      return;
    }
    if (value != p2_delay_ms && !saveP2Delay(value)) {
      SerialBT.println("ERROR: Failed to save P2_DELAY to NVS");
      return;
    }
    p2_delay_ms = value;
    SerialBT.println("OK: P2_DELAY=" + String(p2_delay_ms) + " ms");
    return;
  }

  if (parameter.equalsIgnoreCase("POSITION_DELAY")) {
    if (value > MAX_ENDPOINT_DELAY_MS) {
      SerialBT.println("ERROR: POSITION_DELAY range is 0-600000 ms");
      return;
    }
    const bool p1_saved_ok = value == p1_delay_ms || saveP1Delay(value);
    const bool p2_saved_ok = value == p2_delay_ms || saveP2Delay(value);
    if (!p1_saved_ok || !p2_saved_ok) {
      SerialBT.println("ERROR: Failed to save both endpoint delays to NVS");
      return;
    }
    p1_delay_ms = value;
    p2_delay_ms = value;
    SerialBT.println("OK: P1_DELAY and P2_DELAY=" + String(value) + " ms");
    return;
  }

  if (parameter.equalsIgnoreCase("WORKING_MODE")) {
    if (value < MIN_WORKING_MODE || value > MAX_WORKING_MODE) {
      SerialBT.println("ERROR: WORKING_MODE range is 0-3 (3=Jog)");
      return;
    }

    const uint8_t new_working_mode = static_cast<uint8_t>(value);
    if (new_working_mode != working_mode && !saveWorkingMode(new_working_mode)) {
      SerialBT.println("ERROR: Failed to save WORKING_MODE to NVS");
      return;
    }
    if (new_working_mode != working_mode) {
      stopJog();
      stopAutoMode();
    }
    working_mode = new_working_mode;
    if (working_mode == AUTO_WORKING_MODE) {
      resetAutoMode();
    } else if (working_mode == 0 && homing_state == HomingState::COMPLETE) {
      if (limit_rehome_pending) {
        SerialBT.println("MODE_0_WAITING: Homing is pending");
      } else if (!startSavedPositionMove(true)) {
        SerialBT.println("MODE_0_WAITING: A valid P1 position is required");
      }
    }
    SerialBT.println("OK: WORKING_MODE=" + String(working_mode));
    return;
  }

  SerialBT.println("ERROR: Unknown configuration parameter");
}

void readBTSerial() {
  if (SerialBT.available() > 0) {
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim();

    if (!incoming.isEmpty()) {
      DEBUG_PRINT("BT Serial received: ");
      DEBUG_PRINTLN(incoming);
      processData(incoming);
    }
  }
}

// NVS PARAMETER FUNCTIONS

void initializeParameters() {
  if (!pref.begin(NVS_NAMESPACE, false)) {
    // NVS failure is critical and prints regardless of the DEBUG setting.
    Serial.println("CRITICAL: Failed to initialize NVS parameters.");
    return;
  }

  preferences_ready = true;
  device_id = pref.getUChar(NVS_DEVICE_ID_KEY, DEFAULT_DEVICE_ID);
  motor_homing_speed =
      pref.getUInt(NVS_HOMING_SPEED_KEY, DEFAULT_HOMING_SPEED);
  working_mode = pref.getUChar(NVS_WORKING_MODE_KEY, DEFAULT_WORKING_MODE);
  move_speed = pref.getUInt(NVS_MOVE_SPEED_KEY, DEFAULT_MOVE_SPEED);
  jog_speed = pref.getUInt(NVS_JOG_SPEED_KEY, DEFAULT_JOG_SPEED);
  acceleration = pref.getUInt(NVS_ACCELERATION_KEY, DEFAULT_ACCELERATION);
  deceleration = pref.getUInt(NVS_DECELERATION_KEY, DEFAULT_DECELERATION);
  const uint32_t legacy_position_delay = pref.getUInt(
      NVS_LEGACY_POSITION_DELAY_KEY, DEFAULT_P1_DELAY_MS);
  p1_delay_ms = pref.getUInt(NVS_P1_DELAY_KEY, legacy_position_delay);
  p2_delay_ms = pref.getUInt(NVS_P2_DELAY_KEY, legacy_position_delay);
  motor_1_p1 = pref.getInt(NVS_MOTOR_1_P1_KEY, 0);
  motor_2_p1 = pref.getInt(NVS_MOTOR_2_P1_KEY, 0);
  motor_1_p2 = pref.getInt(NVS_MOTOR_1_P2_KEY, 0);
  motor_2_p2 = pref.getInt(NVS_MOTOR_2_P2_KEY, 0);
  p1_saved = pref.getBool(NVS_P1_SAVED_KEY, false);
  p2_saved = pref.getBool(NVS_P2_SAVED_KEY, false);

  if (device_id < MIN_DEVICE_ID || device_id > MAX_DEVICE_ID) {
    device_id = DEFAULT_DEVICE_ID;
    if (!saveDeviceId(device_id)) {
      Serial.println("CRITICAL: Failed to repair DEVICE_ID in NVS.");
    }
  }

  if (motor_homing_speed < MIN_HOMING_SPEED ||
      motor_homing_speed > MAX_HOMING_SPEED) {
    motor_homing_speed = DEFAULT_HOMING_SPEED;
    if (!saveHomingSpeed(motor_homing_speed)) {
      Serial.println("CRITICAL: Failed to repair HOMING_SPEED in NVS.");
    }
  }

  if (working_mode < MIN_WORKING_MODE || working_mode > MAX_WORKING_MODE) {
    working_mode = DEFAULT_WORKING_MODE;
    if (!saveWorkingMode(working_mode)) {
      Serial.println("CRITICAL: Failed to repair WORKING_MODE in NVS.");
    }
  }

  if (move_speed < MIN_MOVE_SPEED || move_speed > MAX_MOVE_SPEED) {
    move_speed = DEFAULT_MOVE_SPEED;
    if (!saveMoveSpeed(move_speed)) {
      Serial.println("CRITICAL: Failed to repair MOVE_SPEED in NVS.");
    }
  }

  if (jog_speed < MIN_JOG_SPEED || jog_speed > MAX_JOG_SPEED) {
    jog_speed = DEFAULT_JOG_SPEED;
    if (!saveJogSpeed(jog_speed)) {
      Serial.println("CRITICAL: Failed to repair JOG_SPEED in NVS.");
    }
  }

  if (acceleration < MIN_ACCELERATION || acceleration > MAX_ACCELERATION) {
    acceleration = DEFAULT_ACCELERATION;
    if (!saveAcceleration(acceleration)) {
      Serial.println("CRITICAL: Failed to repair ACCELERATION in NVS.");
    }
  }

  if (deceleration < MIN_DECELERATION || deceleration > MAX_DECELERATION) {
    deceleration = DEFAULT_DECELERATION;
    if (!saveDeceleration(deceleration)) {
      Serial.println("CRITICAL: Failed to repair DECELERATION in NVS.");
    }
  }

  if (p1_delay_ms > MAX_ENDPOINT_DELAY_MS) {
    p1_delay_ms = DEFAULT_P1_DELAY_MS;
    if (!saveP1Delay(p1_delay_ms)) {
      Serial.println("CRITICAL: Failed to repair P1_DELAY in NVS.");
    }
  }

  if (p2_delay_ms > MAX_ENDPOINT_DELAY_MS) {
    p2_delay_ms = DEFAULT_P2_DELAY_MS;
    if (!saveP2Delay(p2_delay_ms)) {
      Serial.println("CRITICAL: Failed to repair P2_DELAY in NVS.");
    }
  }

  DEBUG_PRINTLN("Parameters loaded from NVS.");
}

// HOMING FUNCTIONS

bool isMotor1LimitTriggered() {
  return digitalRead(MOTOR_1_LIMIT) == MOTOR_1_LIMIT_ACTIVE_STATE;
}

bool isMotor2LimitTriggered() {
  return digitalRead(MOTOR_2_LIMIT) == MOTOR_2_LIMIT_ACTIVE_STATE;
}

void scheduleHomingAfterLimitTrigger() {
  stopJog();
  stopAutoMode();
  limit_rehome_pending = true;
  limit_rehome_requested_ms = millis();

  DEBUG_PRINTLN("Limit triggered during movement. Delayed homing scheduled.");
  SerialBT.println("LIMIT_TRIGGERED: Motors stopped; homing in " +
                   String(LIMIT_TRIGGER_REHOME_DELAY_MS) + " ms");
}

void monitorMovementLimitSwitches() {
  const bool motor_1_limit_triggered = isMotor1LimitTriggered();
  const bool motor_2_limit_triggered = isMotor2LimitTriggered();
  const bool movement_active = autoMoveIsActive() || motor_1_jogging ||
                               motor_2_jogging || jog_pulse_high;

  if (!limit_rehome_pending && homing_state != HomingState::RUNNING &&
      movement_active &&
      ((motor_1_limit_triggered && !previous_motor_1_limit_triggered) ||
       (motor_2_limit_triggered && !previous_motor_2_limit_triggered))) {
    scheduleHomingAfterLimitTrigger();
  }

  previous_motor_1_limit_triggered = motor_1_limit_triggered;
  previous_motor_2_limit_triggered = motor_2_limit_triggered;
}

void updateDelayedRehoming() {
  if (limit_rehome_pending &&
      millis() - limit_rehome_requested_ms >= LIMIT_TRIGGER_REHOME_DELAY_MS) {
    startHoming();
  }
}

// JOGGING FUNCTIONS

String takeFirstToken(String &text) {
  text.trim();
  const int separator = text.indexOf(' ');
  if (separator < 0) {
    const String token = text;
    text = "";
    return token;
  }

  const String token = text.substring(0, separator);
  text = text.substring(separator + 1);
  text.trim();
  return token;
}

void stopJog() {
  if (motor_1_jogging || motor_2_jogging || jog_pulse_high) {
    digitalWrite(MOTOR_1_PUL, LOW);
    digitalWrite(MOTOR_2_PUL, LOW);
  }
  motor_1_jogging = false;
  motor_2_jogging = false;
  motor_1_jog_steps_remaining = 0;
  motor_2_jog_steps_remaining = 0;
  jog_pulse_high = false;
  jog_move_in_progress = false;
}

bool startJog(bool jog_motor_1, bool jog_motor_2,
              JogDirection direction, uint32_t steps) {
  if (working_mode != JOG_WORKING_MODE ||
      homing_state != HomingState::COMPLETE || steps == 0) {
    return false;
  }

  stopJog();
  jog_direction = direction;

  uint32_t motor_1_permitted_steps = steps;
  uint32_t motor_2_permitted_steps = steps;
  if (direction == JogDirection::AWAY) {
    const uint32_t motor_1_available_steps =
        motor_1_position <= 0
            ? MAX_TRAVEL_STEPS
            : (motor_1_position >= static_cast<int32_t>(MAX_TRAVEL_STEPS)
                   ? 0
                   : MAX_TRAVEL_STEPS -
                         static_cast<uint32_t>(motor_1_position));
    const uint32_t motor_2_available_steps =
        motor_2_position <= 0
            ? MAX_TRAVEL_STEPS
            : (motor_2_position >= static_cast<int32_t>(MAX_TRAVEL_STEPS)
                   ? 0
                   : MAX_TRAVEL_STEPS -
                         static_cast<uint32_t>(motor_2_position));
    motor_1_permitted_steps = min(steps, motor_1_available_steps);
    motor_2_permitted_steps = min(steps, motor_2_available_steps);

    // A BOTH jog uses the same bounded step count to preserve alignment.
    if (jog_motor_1 && jog_motor_2) {
      const uint32_t common_steps =
          min(motor_1_permitted_steps, motor_2_permitted_steps);
      motor_1_permitted_steps = common_steps;
      motor_2_permitted_steps = common_steps;
    }
  }

  if (jog_motor_1) {
    digitalWrite(MOTOR_1_DIR,
                 direction == JogDirection::HOME
                     ? MOTOR_1_HOMING_DIRECTION
                     : !MOTOR_1_HOMING_DIRECTION);
    motor_1_jogging = motor_1_permitted_steps > 0 &&
                      (direction != JogDirection::HOME ||
                       !isMotor1LimitTriggered());
    motor_1_jog_steps_remaining =
        motor_1_jogging ? motor_1_permitted_steps : 0;
    if (!motor_1_jogging && direction == JogDirection::HOME &&
        isMotor1LimitTriggered()) {
      motor_1_position = 0;
    }
  }

  if (jog_motor_2) {
    digitalWrite(MOTOR_2_DIR,
                 direction == JogDirection::HOME
                     ? MOTOR_2_HOMING_DIRECTION
                     : !MOTOR_2_HOMING_DIRECTION);
    motor_2_jogging = motor_2_permitted_steps > 0 &&
                      (direction != JogDirection::HOME ||
                       !isMotor2LimitTriggered());
    motor_2_jog_steps_remaining =
        motor_2_jogging ? motor_2_permitted_steps : 0;
    if (!motor_2_jogging && direction == JogDirection::HOME &&
        isMotor2LimitTriggered()) {
      motor_2_position = 0;
    }
  }

  last_jog_step_us = micros();
  jog_move_in_progress = motor_1_jogging || motor_2_jogging;
  return jog_move_in_progress;
}

void processJogCommand(String arguments) {
  if (arguments.equalsIgnoreCase("STOP")) {
    stopJog();
    SerialBT.println("OK: Jog stopped");
    return;
  }

  if (working_mode != JOG_WORKING_MODE) {
    SerialBT.println("ERROR: Set WORKING_MODE to 3 before jogging");
    return;
  }

  if (homing_state != HomingState::COMPLETE) {
    SerialBT.println("ERROR: Jogging is unavailable until homing completes");
    return;
  }

  if (limit_rehome_pending) {
    SerialBT.println("ERROR: Jogging is unavailable while homing is pending");
    return;
  }

  if (autoMoveIsActive()) {
    SerialBT.println("ERROR: Wait for the P1/P2 positioning move to finish");
    return;
  }

  const String target = takeFirstToken(arguments);
  const String direction_text = takeFirstToken(arguments);
  bool jog_motor_1 = false;
  bool jog_motor_2 = false;

  if (target.equalsIgnoreCase("BOTH")) {
    jog_motor_1 = true;
    jog_motor_2 = true;
  } else if (target.equalsIgnoreCase("MOTOR1") ||
             target.equalsIgnoreCase("M1")) {
    jog_motor_1 = true;
  } else if (target.equalsIgnoreCase("MOTOR2") ||
             target.equalsIgnoreCase("M2")) {
    jog_motor_2 = true;
  } else {
    SerialBT.println("ERROR: Jog target must be BOTH, MOTOR1, or MOTOR2");
    return;
  }

  JogDirection direction;
  if (direction_text.equalsIgnoreCase("HOME")) {
    direction = JogDirection::HOME;
  } else if (direction_text.equalsIgnoreCase("AWAY")) {
    direction = JogDirection::AWAY;
  } else {
    SerialBT.println("ERROR: Jog direction must be HOME or AWAY");
    return;
  }

  uint32_t steps = DEFAULT_JOG_STEPS;
  if (!arguments.isEmpty() && !parseUnsignedValue(arguments, steps)) {
    SerialBT.println("ERROR: Jog steps must be an unsigned integer");
    return;
  }
  if (steps == 0 || steps > MAX_JOG_STEPS) {
    SerialBT.println("ERROR: Jog steps range is 1-" +
                     String(MAX_JOG_STEPS));
    return;
  }

  if (!startJog(jog_motor_1, jog_motor_2, direction, steps)) {
    SerialBT.println("ERROR: Selected motor cannot move farther in that direction");
    return;
  }

  SerialBT.println("OK: Jog started for " + target + " " + direction_text +
                   " " + String(steps) + " steps");
}

void processSavePositionCommand(String arguments) {
  arguments.trim();

  if (working_mode != JOG_WORKING_MODE) {
    SerialBT.println("ERROR: Set WORKING_MODE to 3 before saving P1 or P2");
    return;
  }
  if (homing_state != HomingState::COMPLETE) {
    SerialBT.println("ERROR: Positions cannot be saved until homing completes");
    return;
  }
  if (limit_rehome_pending) {
    SerialBT.println("ERROR: Positions cannot be saved while homing is pending");
    return;
  }
  if (motor_1_jogging || motor_2_jogging || jog_pulse_high) {
    SerialBT.println("ERROR: Wait for jogging to stop before saving a position");
    return;
  }

  const bool save_p1 = arguments.equalsIgnoreCase("P1");
  const bool save_p2 = arguments.equalsIgnoreCase("P2");
  if (!save_p1 && !save_p2) {
    SerialBT.println("ERROR: Use SAVE P1 or SAVE P2");
    return;
  }

  if (save_p1 && p2_saved &&
      (motor_1_position >= motor_1_p2 || motor_2_position >= motor_2_p2)) {
    SerialBT.println("ERROR: P1 must be nearer home than P2 for both motors");
    return;
  }
  if (save_p2 && p1_saved &&
      (motor_1_position <= motor_1_p1 || motor_2_position <= motor_2_p1)) {
    SerialBT.println("ERROR: P2 must be farther from home than P1 for both motors");
    return;
  }

  if (!savePositionPair(save_p1)) {
    SerialBT.println("ERROR: Failed to save position to NVS");
    return;
  }

  SerialBT.println("OK: " + String(save_p1 ? "P1" : "P2") +
                   " saved (M1=" + String(motor_1_position) +
                   ", M2=" + String(motor_2_position) + ")");
}

void updateJog() {
  if (working_mode != JOG_WORKING_MODE ||
      homing_state != HomingState::COMPLETE) {
    if (motor_1_jogging || motor_2_jogging || jog_pulse_high) {
      stopJog();
    }
    return;
  }

  if (jog_direction == JogDirection::HOME) {
    if (motor_1_jogging && isMotor1LimitTriggered()) {
      motor_1_jogging = false;
      motor_1_jog_steps_remaining = 0;
      motor_1_position = 0;
      digitalWrite(MOTOR_1_PUL, LOW);
    }
    if (motor_2_jogging && isMotor2LimitTriggered()) {
      motor_2_jogging = false;
      motor_2_jog_steps_remaining = 0;
      motor_2_position = 0;
      digitalWrite(MOTOR_2_PUL, LOW);
    }
  }

  const uint32_t now_us = micros();
  if (jog_pulse_high) {
    if (now_us - jog_pulse_started_us >= STEP_PULSE_WIDTH_US) {
      digitalWrite(MOTOR_1_PUL, LOW);
      digitalWrite(MOTOR_2_PUL, LOW);
      jog_pulse_high = false;
    }
    return;
  }

  if (motor_1_jogging && motor_1_jog_steps_remaining == 0) {
    motor_1_jogging = false;
  }
  if (motor_2_jogging && motor_2_jog_steps_remaining == 0) {
    motor_2_jogging = false;
  }
  if (!motor_1_jogging && !motor_2_jogging) {
    if (jog_move_in_progress) {
      jog_move_in_progress = false;
      SerialBT.println("JOG_DONE: M1=" + String(motor_1_position) +
                       ", M2=" + String(motor_2_position));
    }
    return;
  }

  const uint32_t jog_step_interval_us = 1000000UL / jog_speed;
  if (now_us - last_jog_step_us < jog_step_interval_us) {
    return;
  }

  if (motor_1_jogging) {
    digitalWrite(MOTOR_1_PUL, HIGH);
    --motor_1_jog_steps_remaining;
    if (jog_direction == JogDirection::AWAY) {
      if (motor_1_position < INT32_MAX) {
        ++motor_1_position;
      }
    } else if (motor_1_position > 0) {
      --motor_1_position;
    }
  }

  if (motor_2_jogging) {
    digitalWrite(MOTOR_2_PUL, HIGH);
    --motor_2_jog_steps_remaining;
    if (jog_direction == JogDirection::AWAY) {
      if (motor_2_position < INT32_MAX) {
        ++motor_2_position;
      }
    } else if (motor_2_position > 0) {
      --motor_2_position;
    }
  }

  last_jog_step_us = now_us;
  jog_pulse_started_us = now_us;
  jog_pulse_high = true;
}

// AUTOMATIC P1/P2 LOOP FUNCTIONS

bool p1PositionIsValid() {
  return p1_saved && motor_1_p1 >= 0 && motor_2_p1 >= 0 &&
         motor_1_p1 <= static_cast<int32_t>(MAX_TRAVEL_STEPS) &&
         motor_2_p1 <= static_cast<int32_t>(MAX_TRAVEL_STEPS);
}

bool p2PositionIsValid() {
  return p2_saved && motor_1_p2 >= 0 && motor_2_p2 >= 0 &&
         motor_1_p2 <= static_cast<int32_t>(MAX_TRAVEL_STEPS) &&
         motor_2_p2 <= static_cast<int32_t>(MAX_TRAVEL_STEPS);
}

bool autoPositionsAreValid() {
  return p1PositionIsValid() && p2PositionIsValid() &&
         motor_1_p2 > motor_1_p1 && motor_2_p2 > motor_2_p1;
}

bool autoMoveIsActive() {
  return auto_state == AutoState::MOVING_TO_P1 ||
         auto_state == AutoState::MOVING_TO_P2 || auto_pulse_high;
}

void stopAutoMode() {
  if (autoMoveIsActive()) {
    digitalWrite(MOTOR_1_PUL, LOW);
    digitalWrite(MOTOR_2_PUL, LOW);
  }

  auto_pulse_high = false;
  auto_motor_1_pulse_high = false;
  auto_motor_2_pulse_high = false;
  auto_total_master_steps = 0;
  auto_master_steps_completed = 0;
  auto_state = AutoState::IDLE;
}

void resetAutoMode() {
  stopAutoMode();
  auto_state = AutoState::IDLE;
}

bool startSavedPositionMove(bool target_is_p1) {
  if (homing_state != HomingState::COMPLETE || limit_rehome_pending ||
      (target_is_p1 ? !p1PositionIsValid() : !p2PositionIsValid())) {
    return false;
  }

  stopJog();
  stopAutoMode();
  startAutoMove(target_is_p1 ? motor_1_p1 : motor_1_p2,
                target_is_p1 ? motor_2_p1 : motor_2_p2,
                target_is_p1);
  return true;
}

void processGotoCommand(String arguments) {
  arguments.trim();

  if (working_mode != 0 && working_mode != JOG_WORKING_MODE) {
    SerialBT.println("ERROR: GOTO is available only in working modes 0 and 3");
    return;
  }
  if (homing_state != HomingState::COMPLETE) {
    SerialBT.println("ERROR: GOTO is unavailable until homing completes");
    return;
  }

  const bool target_is_p1 = arguments.equalsIgnoreCase("P1");
  const bool target_is_p2 = arguments.equalsIgnoreCase("P2");
  if (!target_is_p1 && !target_is_p2) {
    SerialBT.println("ERROR: Use GOTO P1 or GOTO P2");
    return;
  }

  if (!startSavedPositionMove(target_is_p1)) {
    SerialBT.println("ERROR: Requested position has not been calibrated");
    return;
  }

  SerialBT.println("OK: Moving to " + String(target_is_p1 ? "P1" : "P2"));
}

void setAutoFault(const char *message) {
  if (autoMoveIsActive()) {
    digitalWrite(MOTOR_1_PUL, LOW);
    digitalWrite(MOTOR_2_PUL, LOW);
  }
  auto_pulse_high = false;
  auto_motor_1_pulse_high = false;
  auto_motor_2_pulse_high = false;
  auto_state = AutoState::FAULT;

  // Unexpected limit activation is critical and prints regardless of DEBUG.
  Serial.println(message);
  SerialBT.println(message);
}

void completeAutoMove() {
  digitalWrite(MOTOR_1_PUL, LOW);
  digitalWrite(MOTOR_2_PUL, LOW);
  motor_1_position = auto_motor_1_target;
  motor_2_position = auto_motor_2_target;
  auto_pulse_high = false;
  auto_motor_1_pulse_high = false;
  auto_motor_2_pulse_high = false;
  auto_position_reached_ms = millis();
  auto_state = auto_target_is_p1 ? AutoState::WAITING_AT_P1
                                 : AutoState::WAITING_AT_P2;

  SerialBT.println("AUTO_REACHED: " + String(auto_target_is_p1 ? "P1" : "P2") +
                   " M1=" + String(motor_1_position) +
                   " M2=" + String(motor_2_position));
  DEBUG_PRINTLN("Automatic move reached target position.");
}

void startAutoMove(int32_t motor_1_target, int32_t motor_2_target,
                   bool target_is_p1) {
  auto_motor_1_target = motor_1_target;
  auto_motor_2_target = motor_2_target;
  auto_target_is_p1 = target_is_p1;

  const int64_t motor_1_delta =
      static_cast<int64_t>(motor_1_target) - motor_1_position;
  const int64_t motor_2_delta =
      static_cast<int64_t>(motor_2_target) - motor_2_position;
  auto_motor_1_direction = (motor_1_delta > 0) - (motor_1_delta < 0);
  auto_motor_2_direction = (motor_2_delta > 0) - (motor_2_delta < 0);
  auto_motor_1_steps = static_cast<uint32_t>(
      motor_1_delta < 0 ? -motor_1_delta : motor_1_delta);
  auto_motor_2_steps = static_cast<uint32_t>(
      motor_2_delta < 0 ? -motor_2_delta : motor_2_delta);
  auto_total_master_steps =
      max(auto_motor_1_steps, auto_motor_2_steps);
  auto_master_steps_completed = 0;
  auto_motor_1_accumulator = 0;
  auto_motor_2_accumulator = 0;
  auto_pulse_high = false;
  auto_motor_1_pulse_high = false;
  auto_motor_2_pulse_high = false;

  if (auto_motor_1_direction != 0) {
    digitalWrite(MOTOR_1_DIR,
                 auto_motor_1_direction < 0
                     ? MOTOR_1_HOMING_DIRECTION
                     : !MOTOR_1_HOMING_DIRECTION);
  }
  if (auto_motor_2_direction != 0) {
    digitalWrite(MOTOR_2_DIR,
                 auto_motor_2_direction < 0
                     ? MOTOR_2_HOMING_DIRECTION
                     : !MOTOR_2_HOMING_DIRECTION);
  }

  auto_state = target_is_p1 ? AutoState::MOVING_TO_P1
                            : AutoState::MOVING_TO_P2;
  last_auto_step_us = micros();

  if (auto_total_master_steps == 0) {
    completeAutoMove();
  } else {
    SerialBT.println("AUTO_MOVING_TO: " + String(target_is_p1 ? "P1" : "P2"));
  }
}

uint32_t calculateAutoStepIntervalUs() {
  const uint32_t steps_remaining =
      auto_total_master_steps - auto_master_steps_completed;
  const float acceleration_speed =
      sqrtf(2.0F * static_cast<float>(acceleration) *
            static_cast<float>(auto_master_steps_completed + 1));
  const float deceleration_speed =
      sqrtf(2.0F * static_cast<float>(deceleration) *
            static_cast<float>(steps_remaining));
  float profile_speed = min(static_cast<float>(move_speed),
                            min(acceleration_speed, deceleration_speed));
  const float minimum_speed =
      min(static_cast<float>(move_speed), MIN_PROFILE_SPEED);
  profile_speed = max(profile_speed, minimum_speed);
  return static_cast<uint32_t>(1000000.0F / profile_speed);
}

void updateAutoMove() {
  if (auto_motor_1_direction < 0 && isMotor1LimitTriggered()) {
    if (auto_motor_1_target != 0) {
      setAutoFault("CRITICAL: Motor 1 limit triggered before automatic target.");
      return;
    }
    motor_1_position = 0;
    auto_motor_1_direction = 0;
    auto_motor_1_steps = 0;
    auto_motor_1_accumulator = 0;
    digitalWrite(MOTOR_1_PUL, LOW);
    auto_motor_1_pulse_high = false;
  }

  if (auto_motor_2_direction < 0 && isMotor2LimitTriggered()) {
    if (auto_motor_2_target != 0) {
      setAutoFault("CRITICAL: Motor 2 limit triggered before automatic target.");
      return;
    }
    motor_2_position = 0;
    auto_motor_2_direction = 0;
    auto_motor_2_steps = 0;
    auto_motor_2_accumulator = 0;
    digitalWrite(MOTOR_2_PUL, LOW);
    auto_motor_2_pulse_high = false;
  }

  const uint32_t now_us = micros();
  if (auto_pulse_high) {
    if (now_us - auto_pulse_started_us >= STEP_PULSE_WIDTH_US) {
      if (auto_motor_1_pulse_high) {
        digitalWrite(MOTOR_1_PUL, LOW);
      }
      if (auto_motor_2_pulse_high) {
        digitalWrite(MOTOR_2_PUL, LOW);
      }
      auto_motor_1_pulse_high = false;
      auto_motor_2_pulse_high = false;
      auto_pulse_high = false;
    }
    return;
  }

  if (auto_master_steps_completed >= auto_total_master_steps) {
    completeAutoMove();
    return;
  }

  const uint32_t step_interval_us = calculateAutoStepIntervalUs();
  if (now_us - last_auto_step_us < step_interval_us) {
    return;
  }

  if (auto_motor_1_direction != 0) {
    auto_motor_1_accumulator += auto_motor_1_steps;
    if (auto_motor_1_accumulator >= auto_total_master_steps) {
      auto_motor_1_accumulator -= auto_total_master_steps;
      digitalWrite(MOTOR_1_PUL, HIGH);
      auto_motor_1_pulse_high = true;
      motor_1_position += auto_motor_1_direction;
    }
  }

  if (auto_motor_2_direction != 0) {
    auto_motor_2_accumulator += auto_motor_2_steps;
    if (auto_motor_2_accumulator >= auto_total_master_steps) {
      auto_motor_2_accumulator -= auto_total_master_steps;
      digitalWrite(MOTOR_2_PUL, HIGH);
      auto_motor_2_pulse_high = true;
      motor_2_position += auto_motor_2_direction;
    }
  }

  ++auto_master_steps_completed;
  last_auto_step_us = now_us;
  auto_pulse_started_us = now_us;
  auto_pulse_high = auto_motor_1_pulse_high || auto_motor_2_pulse_high;
}

void updateAutoMode() {
  if (homing_state != HomingState::COMPLETE || auto_state == AutoState::FAULT) {
    return;
  }

  // A coordinated one-shot GOTO/P1-after-home move runs in modes 0, 1, or 3.
  if (auto_state == AutoState::MOVING_TO_P1 ||
      auto_state == AutoState::MOVING_TO_P2) {
    updateAutoMove();
    return;
  }

  if (working_mode != AUTO_WORKING_MODE) {
    return;
  }

  if (auto_state == AutoState::IDLE) {
    if (!autoPositionsAreValid()) {
      auto_state = AutoState::WAITING_FOR_POSITIONS;
      SerialBT.println("AUTO_WAITING: Valid P1 and P2 positions are required");
      return;
    }
    startAutoMove(motor_1_p1, motor_2_p1, true);
    return;
  }

  if (auto_state == AutoState::WAITING_FOR_POSITIONS) {
    if (autoPositionsAreValid()) {
      startAutoMove(motor_1_p1, motor_2_p1, true);
    }
    return;
  }

  if (auto_state == AutoState::WAITING_AT_P1 &&
      millis() - auto_position_reached_ms >= p1_delay_ms) {
    if (!autoPositionsAreValid()) {
      auto_state = AutoState::WAITING_FOR_POSITIONS;
      SerialBT.println("AUTO_WAITING: Valid P1 and P2 positions are required");
      return;
    }
    startAutoMove(motor_1_p2, motor_2_p2, false);
    return;
  }

  if (auto_state == AutoState::WAITING_AT_P2 &&
      millis() - auto_position_reached_ms >= p2_delay_ms) {
    if (!autoPositionsAreValid()) {
      auto_state = AutoState::WAITING_FOR_POSITIONS;
      SerialBT.println("AUTO_WAITING: Valid P1 and P2 positions are required");
      return;
    }
    startAutoMove(motor_1_p1, motor_2_p1, true);
  }
}

void stopHomingWithFault(const char *message) {
  digitalWrite(MOTOR_1_PUL, LOW);
  digitalWrite(MOTOR_2_PUL, LOW);
  motor_1_homing = false;
  motor_2_homing = false;
  homing_pulse_high = false;
  homing_state = HomingState::FAULT;

  // Homing faults are critical and print regardless of the DEBUG setting.
  Serial.println(message);
}

void finishHoming() {
  digitalWrite(MOTOR_1_PUL, LOW);
  digitalWrite(MOTOR_2_PUL, LOW);
  motor_1_position = 0;
  motor_2_position = 0;
  homing_pulse_high = false;
  homing_state = HomingState::COMPLETE;
  DEBUG_PRINTLN("Homing complete. Both motor positions set to zero.");

  if (!startSavedPositionMove(true)) {
    SerialBT.println("P1_WAITING: A valid P1 position is required after homing");
  }
}

void startHoming() {
  limit_rehome_pending = false;
  stopJog();
  stopAutoMode();
  digitalWrite(MOTOR_1_PUL, LOW);
  digitalWrite(MOTOR_2_PUL, LOW);
  digitalWrite(MOTOR_1_DIR, MOTOR_1_HOMING_DIRECTION);
  digitalWrite(MOTOR_2_DIR, MOTOR_2_HOMING_DIRECTION);

  motor_1_homing = !isMotor1LimitTriggered();
  motor_2_homing = !isMotor2LimitTriggered();
  previous_motor_1_limit_triggered = !motor_1_homing;
  previous_motor_2_limit_triggered = !motor_2_homing;
  homing_pulse_high = false;
  homing_started_ms = millis();
  last_homing_step_us = micros();
  homing_state = HomingState::RUNNING;

  DEBUG_PRINTLN("Homing started.");

  if (!motor_1_homing && !motor_2_homing) {
    finishHoming();
  }
}

void updateHoming() {
  if (homing_state != HomingState::RUNNING) {
    return;
  }

  if (motor_1_homing && isMotor1LimitTriggered()) {
    motor_1_homing = false;
    digitalWrite(MOTOR_1_PUL, LOW);
    DEBUG_PRINTLN("Motor 1 homing limit reached.");
  }

  if (motor_2_homing && isMotor2LimitTriggered()) {
    motor_2_homing = false;
    digitalWrite(MOTOR_2_PUL, LOW);
    DEBUG_PRINTLN("Motor 2 homing limit reached.");
  }

  if (!motor_1_homing && !motor_2_homing) {
    finishHoming();
    return;
  }

  if (millis() - homing_started_ms >= HOMING_TIMEOUT_MS) {
    stopHomingWithFault("CRITICAL: Homing timed out. Both motors stopped.");
    return;
  }

  const uint32_t now_us = micros();

  if (homing_pulse_high) {
    if (now_us - homing_pulse_started_us >= STEP_PULSE_WIDTH_US) {
      digitalWrite(MOTOR_1_PUL, LOW);
      digitalWrite(MOTOR_2_PUL, LOW);
      homing_pulse_high = false;
    }
    return;
  }

  const uint32_t homing_step_interval_us = 1000000UL / motor_homing_speed;
  if (now_us - last_homing_step_us >= homing_step_interval_us) {
    // Both active motors receive the same step edge to keep the load aligned.
    if (motor_1_homing) {
      digitalWrite(MOTOR_1_PUL, HIGH);
    }
    if (motor_2_homing) {
      digitalWrite(MOTOR_2_PUL, HIGH);
    }

    last_homing_step_us = now_us;
    homing_pulse_started_us = now_us;
    homing_pulse_high = true;
  }
}


void initializeGPIO() {
  // Set safe output levels before enabling the motor-control pins as outputs.
  digitalWrite(MOTOR_1_PUL, LOW);
  digitalWrite(MOTOR_1_DIR, LOW);
  digitalWrite(MOTOR_2_PUL, LOW);
  digitalWrite(MOTOR_2_DIR, LOW);

  pinMode(MOTOR_1_PUL, OUTPUT);
  pinMode(MOTOR_1_DIR, OUTPUT);
  pinMode(MOTOR_2_PUL, OUTPUT);
  pinMode(MOTOR_2_DIR, OUTPUT);

  // GPIO34 and GPIO35 are input-only and have no internal pull resistors.
  pinMode(MOTOR_1_LIMIT, INPUT);
  pinMode(MOTOR_2_LIMIT, INPUT);
}

void setup() {
  Serial.begin(115200);
  initializeGPIO();
  initializeParameters();

  const String bluetoothDeviceName = "KINETIC_LIFT_" + String(device_id);
  if (SerialBT.begin(bluetoothDeviceName)) {
    DEBUG_PRINTLN("Bluetooth Serial initialized as " + bluetoothDeviceName);
  } else {
    // Critical errors are printed regardless of the DEBUG setting.
    Serial.println("Bluetooth Serial initialization failed");
  }

  startHoming();
}


void loop() {
  monitorMovementLimitSwitches();
  updateDelayedRehoming();
  updateHoming();
  if (!limit_rehome_pending) {
    updateAutoMode();
    updateJog();
  }
  readBTSerial();
}
