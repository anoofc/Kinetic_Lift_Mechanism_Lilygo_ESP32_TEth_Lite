#define DEBUG 1

#define MOTOR_1_PUL 13
#define MOTOR_1_DIR 32

#define MOTOR_2_PUL 14
#define MOTOR_2_DIR 33

#define MOTOR_1_LIMIT 34
#define MOTOR_2_LIMIT 35

#include <Arduino.h>

#include <BluetoothSerial.h>
#include <Preferences.h>

BluetoothSerial SerialBT;
Preferences preferences;

uint8_t device_id = 1;




void setup() {
  Serial.begin(115200);
  SerialBT.begin("KINETIC_LIFT_" + String(device_id));
}


void loop() {
  // TODO
}