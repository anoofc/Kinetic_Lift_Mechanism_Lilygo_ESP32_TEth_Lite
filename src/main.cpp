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
constexpr char NVS_MOTOR_1_P1_KEY[] = "m1_p1";
constexpr char NVS_MOTOR_2_P1_KEY[] = "m2_p1";
constexpr char NVS_MOTOR_1_P2_KEY[] = "m1_p2";
constexpr char NVS_MOTOR_2_P2_KEY[] = "m2_p2";
constexpr char NVS_P1_SAVED_KEY[] = "p1_saved";
constexpr char NVS_P2_SAVED_KEY[] = "p2_saved";

constexpr uint8_t DEFAULT_DEVICE_ID = 1;
constexpr uint32_t DEFAULT_HOMING_SPEED = 1000;
constexpr uint8_t DEFAULT_WORKING_MODE = 0;
constexpr uint8_t MIN_DEVICE_ID = 1;
constexpr uint8_t MAX_DEVICE_ID = 255;
constexpr uint32_t MIN_HOMING_SPEED = 1;
constexpr uint32_t MAX_HOMING_SPEED = 5000;
constexpr uint8_t MIN_WORKING_MODE = 0;
constexpr uint8_t MAX_WORKING_MODE = 3;
constexpr uint8_t JOG_WORKING_MODE = 3;

uint8_t device_id = DEFAULT_DEVICE_ID;
uint32_t motor_homing_speed = DEFAULT_HOMING_SPEED; // Steps per second.
uint8_t working_mode = DEFAULT_WORKING_MODE;
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
constexpr uint32_t JOG_SPEED = 250; // Steps per second.
constexpr uint32_t DEFAULT_JOG_STEPS = 10;
constexpr uint32_t MAX_JOG_STEPS = 10000;
constexpr uint8_t BT_MAX_BYTES_PER_LOOP = 64;
constexpr size_t BT_MAX_COMMAND_LENGTH = 128;

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

void stopJog();
bool startJog(bool jog_motor_1, bool jog_motor_2,
              JogDirection direction, uint32_t steps);
void processJogCommand(String arguments);
void processSavePositionCommand(String arguments);


// BLUETOOTH SERIAL FUNCTIONS

void sendConfiguration() {
  SerialBT.println("DEVICE_ID=" + String(device_id));
  SerialBT.println("HOMING_SPEED=" + String(motor_homing_speed) + " steps/s");
  SerialBT.println("WORKING_MODE=" + String(working_mode));
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

  const int first_separator = data.indexOf(' ');
  if (first_separator < 0) {
    SerialBT.println("ERROR: Use SET <PARAMETER> <VALUE> or GET CONFIG");
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

  if (!command.equalsIgnoreCase("SET")) {
    SerialBT.println("ERROR: Unknown command. Use SET, JOG, SAVE, or GET CONFIG");
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
    working_mode = new_working_mode;
    if (working_mode != JOG_WORKING_MODE) {
      stopJog();
    }
    SerialBT.println("OK: WORKING_MODE=" + String(working_mode));
    return;
  }

  SerialBT.println("ERROR: Parameter must be DEVICE_ID, HOMING_SPEED, or WORKING_MODE");
}

void readBTSerial() {
  static String incoming;
  static bool discard_until_newline = false;
  uint8_t bytes_read = 0;

  while (SerialBT.available() > 0 && bytes_read < BT_MAX_BYTES_PER_LOOP) {
    const char received = static_cast<char>(SerialBT.read());
    ++bytes_read;

    if (received == '\n') {
      if (!discard_until_newline) {
        incoming.trim();
      }
      if (!discard_until_newline && !incoming.isEmpty()) {
        DEBUG_PRINT("BT Serial received: ");
        DEBUG_PRINTLN(incoming);
        processData(incoming);
      }
      incoming = "";
      discard_until_newline = false;
    } else if (!discard_until_newline && received != '\r' &&
               incoming.length() < BT_MAX_COMMAND_LENGTH) {
      incoming += received;
    } else if (!discard_until_newline && received != '\r') {
      incoming = "";
      discard_until_newline = true;
      DEBUG_PRINTLN("BT Serial command discarded: command is too long.");
      SerialBT.println("ERROR: Command is too long");
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

  DEBUG_PRINTLN("Parameters loaded from NVS.");
}

// HOMING FUNCTIONS

bool isMotor1LimitTriggered() {
  return digitalRead(MOTOR_1_LIMIT) == MOTOR_1_LIMIT_ACTIVE_STATE;
}

bool isMotor2LimitTriggered() {
  return digitalRead(MOTOR_2_LIMIT) == MOTOR_2_LIMIT_ACTIVE_STATE;
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
  digitalWrite(MOTOR_1_PUL, LOW);
  digitalWrite(MOTOR_2_PUL, LOW);
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

  if (jog_motor_1) {
    digitalWrite(MOTOR_1_DIR,
                 direction == JogDirection::HOME
                     ? MOTOR_1_HOMING_DIRECTION
                     : !MOTOR_1_HOMING_DIRECTION);
    motor_1_jogging =
        direction != JogDirection::HOME || !isMotor1LimitTriggered();
    motor_1_jog_steps_remaining = motor_1_jogging ? steps : 0;
    if (!motor_1_jogging) {
      motor_1_position = 0;
    }
  }

  if (jog_motor_2) {
    digitalWrite(MOTOR_2_DIR,
                 direction == JogDirection::HOME
                     ? MOTOR_2_HOMING_DIRECTION
                     : !MOTOR_2_HOMING_DIRECTION);
    motor_2_jogging =
        direction != JogDirection::HOME || !isMotor2LimitTriggered();
    motor_2_jog_steps_remaining = motor_2_jogging ? steps : 0;
    if (!motor_2_jogging) {
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
    SerialBT.println("ERROR: Jog steps range is 1-10000");
    return;
  }

  if (!startJog(jog_motor_1, jog_motor_2, direction, steps)) {
    SerialBT.println("ERROR: Selected motor is already at its home limit");
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

  const uint32_t jog_step_interval_us = 1000000UL / JOG_SPEED;
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
}

void startHoming() {
  stopJog();
  digitalWrite(MOTOR_1_PUL, LOW);
  digitalWrite(MOTOR_2_PUL, LOW);
  digitalWrite(MOTOR_1_DIR, MOTOR_1_HOMING_DIRECTION);
  digitalWrite(MOTOR_2_DIR, MOTOR_2_HOMING_DIRECTION);

  motor_1_homing = !isMotor1LimitTriggered();
  motor_2_homing = !isMotor2LimitTriggered();
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
  updateHoming();
  updateJog();
  readBTSerial();
}
