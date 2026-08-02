#ifndef DEBUG
#define DEBUG 1
#endif

#define MOTOR_1_PUL 13
#define MOTOR_1_DIR 32

#define MOTOR_2_PUL 14
#define MOTOR_2_DIR 33

#define MOTOR_1_LIMIT 34
#define MOTOR_2_LIMIT 35

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

uint8_t device_id = 1;


// BLUETOOTH SERIAL FUNCTIONS

void processData(String data) {

}

void readBTSerial(){
  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim(); // Remove any leading/trailing whitespace, including newline characters
    processData(incoming); 
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

  const String bluetoothDeviceName = "KINETIC_LIFT_" + String(device_id);
  if (SerialBT.begin(bluetoothDeviceName)) {
    DEBUG_PRINTLN("Bluetooth Serial initialized as " + bluetoothDeviceName);
  } else {
    // Critical errors are printed regardless of the DEBUG setting.
    Serial.println("Bluetooth Serial initialization failed");
  }
}


void loop() {
  
}
