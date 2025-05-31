#include "motor_v3.h"

Motor_control wheelL, wheelR;

// Pin definitions
#define STOP_LED_PIN 42
#define STOP_BUTTON_PIN 36
#define LINE_SENSOR_0 37
#define LINE_SENSOR_1 38
#define LINE_SENSOR_2 39
#define LINE_SENSOR_3 40
#define LINE_SENSOR_4 41
#define EDGE_SENSOR_LEFT 43
#define EDGE_SENSOR_RIGHT 44

// Control flags
bool b_reachEdge = false;
bool b_serialPrint = true;

// Motor speeds
uint8_t wheelLSpeed = 0;
uint8_t wheelRSpeed = 0;

void setup() {
  Serial.begin(115200);

  wheelL.begin(17, 18, 0, 1);
  wheelR.begin(34, 33, 2, 3);

  // Setup line sensor inputs
  for (int pin = 37; pin <= 41; pin++) {
    pinMode(pin, INPUT);
  }

  // Edge and stop buttons
  pinMode(EDGE_SENSOR_LEFT, INPUT_PULLUP);
  pinMode(EDGE_SENSOR_RIGHT, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

  // Stop LED
  pinMode(STOP_LED_PIN, OUTPUT);
  digitalWrite(STOP_LED_PIN, HIGH);

  Serial.println("Robot Line Follower Initialized");
  delay(500);
}

void loop() {
  // Stop button pressed
  if (digitalRead(STOP_BUTTON_PIN) == LOW) {
    wheelL.stop();
    wheelR.stop();
    Serial.println("STOP button pressed — halting!");
    while (true);
  }

  // Read line sensor values (1 = black, 0 = white)
  uint8_t sensor = 0;
  for (int i = 0; i < 5; i++) {
    sensor |= (digitalRead(37 + i) << (4 - i));
  }

  // Read edge sensors (active LOW)
  bool edgeLeft = (digitalRead(EDGE_SENSOR_LEFT) == LOW);
  bool edgeRight = (digitalRead(EDGE_SENSOR_RIGHT) == LOW);

  if (b_serialPrint) {
    Serial.print("Line Sensor: ");
    for (int i = 4; i >= 0; i--) {
      Serial.print((sensor >> i) & 1);
    }

    Serial.print("  Raw: ");
    for (int i = 0; i < 5; i++) {
      Serial.print("D");
      Serial.print(37 + i);
      Serial.print("=");
      Serial.print(digitalRead(37 + i));
      Serial.print(" ");
    }

    Serial.print(" Edge: L=");
    Serial.print(edgeLeft);
    Serial.print(" R=");
    Serial.println(edgeRight);
  }

  if (!b_reachEdge) {
    followLine(sensor);
    if ((sensor & 0b00111) == 0) {
      b_reachEdge = true;
      Serial.println("Approaching platform edge...");
    }
  } else {
    alignToEdge(sensor, edgeLeft, edgeRight);
  }

  // Apply speeds
  wheelL.speed(wheelLSpeed, true);
  wheelR.speed(wheelRSpeed, true);

  delay(10);
}

void followLine(uint8_t sensor) {
  wheelLSpeed = 0;
  wheelRSpeed = 0;

  // Left bias
  if (sensor & 0b10000) wheelLSpeed += 150; // S1
  if (sensor & 0b01000) wheelLSpeed += 90;  // S2

  // Right bias
  if (sensor & 0b00010) wheelRSpeed += 90;  // S4
  if (sensor & 0b00001) wheelRSpeed += 150; // S5

  // Cap speeds at 160
  wheelLSpeed = min(wheelLSpeed, (uint8_t)160);
  wheelRSpeed = min(wheelRSpeed, (uint8_t)160);
}

void alignToEdge(uint8_t sensor, bool edgeLeft, bool edgeRight) {
  if (!edgeLeft && !edgeRight) {
    wheelLSpeed = 0;
    wheelRSpeed = 0;
    digitalWrite(STOP_LED_PIN, LOW);
    Serial.println("Aligned and STOPPED at platform edge.");
    return;x
  }

  // Nudge into alignment
  wheelLSpeed = edgeLeft ? 128 : 0;
  wheelRSpeed = edgeRight ? 128 : 0;
  digitalWrite(STOP_LED_PIN, HIGH);
}
