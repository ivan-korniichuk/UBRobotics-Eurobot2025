#include <FastAccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TMCStepper.h>
#include <ESP32Servo.h>
#include <Wire.h>

#define r 58
#define d 120
#define PI 3.14159265358979323

#define UART2_TX 18
#define UART2_RX 19
#define UART0_TX 12
#define UART0_RX 13
#define R_SENSE 0.11f

#define I2C_DEV_ADDR 0x41
#define I2C_SDA 26
#define I2C_SCL 25
#define I2C_FREQ 100000

int EN_PINS [] = {15, 16, 4, 2, 22, 23}; 
int DIAG_PINS [] = {36, 39, 34, 35, 32, 33}; 
bool HOMED [] = {0, 0, 0, 0, 0, 0};

float MOT_1_speed = 0;
float MOT_2_speed = 0;
float MOT_3_speed = 0;

volatile float omega = 0;
volatile float v_x = 0;
volatile float v_y = 0;
volatile int scalar = 0;
volatile unsigned long last_command_time = 0;

TwoWire I2C = TwoWire(0);
uint8_t buffer[32];
uint8_t length = 0;

HardwareSerial TMCSerial_1(2);
HardwareSerial TMCSerial_2(1);

TMC2209Stepper TMCdriver_1(&TMCSerial_1, R_SENSE, 0b00);
TMC2209Stepper TMCdriver_2(&TMCSerial_1, R_SENSE, 0b10);
TMC2209Stepper TMCdriver_3(&TMCSerial_1, R_SENSE, 0b11);

void setup() {
  Serial.begin(115200);
  TMCSerial_1.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
  TMCSerial_2.begin(115200, SERIAL_8N1, UART0_RX, UART0_TX);

  for (byte i = 0; i < sizeof(EN_PINS)/sizeof(EN_PINS[0]); i++) {
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], LOW);
  }

  for (byte i = 0; i < sizeof(DIAG_PINS)/sizeof(DIAG_PINS[0]); i++) {
    pinMode(DIAG_PINS[i], INPUT_PULLUP);
  }

  TMCdriver_1.begin();
  TMCdriver_1.toff(5);
  TMCdriver_1.rms_current(850);
  TMCdriver_1.microsteps(8);

  TMCdriver_2.begin();
  TMCdriver_2.toff(5);
  TMCdriver_2.rms_current(850);
  TMCdriver_2.microsteps(8);

  TMCdriver_3.begin();
  TMCdriver_3.toff(5);
  TMCdriver_3.rms_current(850);
  TMCdriver_3.microsteps(8);

  WiFi.mode(WIFI_STA);
  delay(100);
  if (esp_now_init() != ESP_OK) return;

  I2C.onReceive(onReceive);
  I2C.onRequest(onRequest);
  I2C.begin((uint8_t)I2C_DEV_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
  Serial.println("Locomotion I2C Ready (0x41)");
}

void loop() {
  if (millis() - last_command_time > 500) {
    TMCdriver_1.VACTUAL(0);
    TMCdriver_2.VACTUAL(0);
    TMCdriver_3.VACTUAL(0);
    return;
  }

  MOT_1_speed = ((1.0/r)*((-d*omega)+v_x))*scalar;
  MOT_2_speed = ((1.0/r)*((-d*omega)+(v_x*0.5)-(sqrt(3)/2.0)*v_y))*scalar;
  MOT_3_speed = ((1.0/r)*((-d*omega)+(v_x*0.5)+(sqrt(3)/2.0)*v_y))*scalar;

  TMCdriver_1.VACTUAL(MOT_1_speed);
  TMCdriver_2.VACTUAL(MOT_2_speed);
  TMCdriver_3.VACTUAL(MOT_3_speed);

  delay(20);

  // Serial.printf("\u03c9: %.2f || v_X: %.2f || v_y: %.2f || M1: %.2f || M2: %.2f || M3: %.2f || Scalar: %d\n",
  //               omega, v_x, v_y, MOT_1_speed, MOT_2_speed, MOT_3_speed, scalar);
}

void onReceive(int len) {
  if (len < 10) return;
  I2C.readBytes(buffer, len);
  uint8_t cmd = buffer[1];
  // Serial.printf("[I2C] CMD_ID: %d | Bytes: %d\n", cmd, len);

  if (cmd == 1) {
    int16_t recvOmega  = (buffer[2] << 8) | buffer[3];
    int16_t recvVX     = (buffer[4] << 8) | buffer[5];
    int16_t recvVY     = (buffer[6] << 8) | buffer[7];
    int16_t recvScalar = (buffer[8] << 8) | buffer[9];

    omega = recvOmega / 100.0f;
    v_x = recvVX / 100.0f;
    v_y = recvVY / 100.0f;
    scalar = recvScalar;
    last_command_time = millis();

    Serial.printf("I2C LOCOMOTION | \u03c9: %.2f, vX: %.2f, vY: %.2f, S: %d\n", omega, v_x, v_y, scalar);
  }
}

void onRequest() {
  I2C.printf("%.*s", length, buffer); 
  I2C.print('\0');
  Serial.printf("onRequestCB\n");
}

void performHoming(TMC2209Stepper &driver, int threshold) {
  driver.VACTUAL(7500);
  delay(100);
  while (driver.SG_RESULT() > threshold) {
    driver.VACTUAL(7500);
  }
  driver.VACTUAL(0);
  delay(500);
}
