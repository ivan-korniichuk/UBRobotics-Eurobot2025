#include <FastAccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TMCStepper.h>  // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <ESP32Servo.h>
#include "Wire.h"

#define I2C_SDA 26
#define I2C_SCL 25
#define I2C_DEV_ADDR 0x40
#define I2C_FREQ 100000

// #include <Adafruit_PWMServoDriver.h>
// Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);       // called this way, it uses the default address 0x40   

#define SERVOMIN  125                                                 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625                                                 // this is the 'maximum' pulse length count (out of 4096)

// Vehicle parameters used for motor speed calculations 
#define r 58   // Wheel radius in mm
#define d 120  // Distance from center to center of wheel in mm 
#define PI 3.14159265358979323


#define EN_PIN          36      // Enable - PURPLE
#define DIR_PIN         14      // Direction - WHITE
#define STEP_PIN        27      // Step - ORANGE
#define UART2_TX        18      // UART2 TX Pin (ESP32 Default TX2)
#define UART2_RX        19      // UART2 RX Pin (ESP32 Default RX2)
#define UART0_TX        12      // UART2 TX Pin (ESP32 Default TX2)
#define UART0_RX        13      // UART2 RX Pin (ESP32 Default RX2)
#define DRIVER_ADDRESS  0b01    // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE         0.11f

int EN_PINS [] = {4, 16, 17, 5, 22, 23}; 
int THRS [] = {180, 180, 120, 180, 250, 250, 250, 250};
bool HOMED [] = {0, 0, 0, 0, 0, 0, 0, 0};
bool DIR [] = {false, false, false, false, false, false};

uint32_t i = 0;
uint8_t buffer[32]; //please dont exceed this
uint8_t comma_ptr = 0;
uint8_t length = 0;

int accel;
long maxSpeed;
int speedChangeDelay;
bool dir = false;

// TMC2209 Driver Initiliasiationn with two Serial buses
HardwareSerial TMCSerial_1(2); 

// TMC2209Stepper V_UPPER_LEVEL(&TMCSerial_1, R_SENSE, 0b11);

HardwareSerial TMCSerial_2(1); 

TMC2209Stepper V_RIGHT_TWIST(&TMCSerial_1, R_SENSE, 0b00);
TMC2209Stepper V_LEFT_GRIPPER(&TMCSerial_1, R_SENSE, 0b10);
TMC2209Stepper V_LOWER_LEVEL(&TMCSerial_1, R_SENSE, 0b11);

TMC2209Stepper V_UPPER_LEVEL(&TMCSerial_2, R_SENSE, 0b00);
TMC2209Stepper V_RIGHT_GRIPPER(&TMCSerial_2, R_SENSE, 0b10);
TMC2209Stepper V_LEFT_TWIST(&TMCSerial_2, R_SENSE, 0b11);


struct MotorState {
  TMC2209Stepper* driver;
  long current_position = 0;
  long target_position = 0;
  int max_velocity = 0;
  bool is_active = false;
};

MotorState motor_states[] = {
  { &V_LOWER_LEVEL },
  { &V_UPPER_LEVEL },
  { &V_LEFT_TWIST },
  { &V_RIGHT_TWIST },
  { &V_LEFT_GRIPPER },
  { &V_RIGHT_GRIPPER }
};

const int NUM_MOTORS = sizeof(motor_states) / sizeof(MotorState);

// Locomotion Parameters
float MOT_1_speed = 0;
float MOT_2_speed = 0;
float MOT_3_speed = 0;
float max_omega = 2 * PI;
float max_v_x = 600;
float max_v_y = 600;

// Initial Locomotion Parameters 
float omega = 0;
float v_x = 0;
float v_y = 0;
int scalar = 1;

// // Initial motor speed parameters Configuration
// int maxSpeed = 6000; // steps per second
// int speed = 1700;     // steps per second
// int maxAcceleration = 5000;  
// int acceleration = 2 * speed; // steps per second^2

// Remote Controller Data Structure 
struct Recive_Data {
  int16_t X_JOYSTICK_state; 
  int16_t Y_JOYSTICK_state;
  int16_t Z_JOYSTICK_state;
  int16_t POT_R_state;
  int16_t POT_C_state;  
  int16_t POT_L_state; 
  int16_t BTN_JOYSTICK_state; 
  int16_t BTN_R_state;
  int16_t BTN_C_state;
  int16_t BTN_L_state;
  int16_t SW_R_state;
  int16_t SW_C_state;
  int16_t SW_L_state;
};
Recive_Data data;

bool BTN_LATCH = 0; 


long current_position = 0; // You can manage this per motor if needed

TwoWire I2C = TwoWire(0);

void setup() {
  Serial.begin(115200);           // Initialize hardware serial for debugging
  delay(100);
  // Initialize UART2 for TMC2209
  TMCSerial_1.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);  
  TMCSerial_2.begin(115200, SERIAL_8N1, UART0_RX, UART0_TX);  
  // Note: USB debuging will not be avaliable UART0 is used to drive the Steppers motors
  
  delay(100);
  
  // board1.begin();
  // board1.setPWMFreq(60); 

  for (byte i = 0; i < (sizeof(EN_PINS) / sizeof(EN_PINS[0])); i++){
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], LOW); 
  }
  
  // Initilize TMC2209 board  
  initDrivers();
  delay(100);

  // WiFi initialization for ESP-NOW
  WiFi.mode(WIFI_STA);
  delay(100);
  if (esp_now_init() != ESP_OK) {
      // Serial.println("ESP-NOW Init Failed");
      return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println(WiFi.macAddress()); // Print MAC address
  
  delay(400);
  I2C.onReceive(onReceive);
  I2C.onRequest(onRequest);
  I2C.begin((uint8_t)I2C_DEV_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
}

void loop() {
  updateAllMotors();
  delay(2);  // prevent overload, tune if needed
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&data, incomingData, sizeof(data));
}


void print_received_data() {
  Serial.printf("X: %d Y: %d Z: %d BTN_JOYSTICK: %d POT_R: %d POT_C: %d POT_L: %d\n",
                data.X_JOYSTICK_state, data.Y_JOYSTICK_state, data.Z_JOYSTICK_state,
                data.BTN_JOYSTICK_state, data.POT_R_state, data.POT_C_state, data.POT_L_state);
  Serial.printf("BTN_R: %d BTN_C: %d BTN_L: %d SW_R: %d SW_C: %d SW_L: %d\n",
                data.BTN_R_state, data.BTN_C_state, data.BTN_L_state,
                data.SW_R_state, data.SW_C_state, data.SW_L_state);
}

void print_speeds() {
  Serial.printf("ω: %.2f || v_X: %.2f || v_y: %.2f || MOT_1: %.2f || MOT_2: %.2f || MOT_3: %.2f || Scalar: %d\n",
                omega, v_x, v_y, MOT_1_speed, MOT_2_speed, MOT_3_speed, scalar);
}
float roundToNearest100(float num) {
    return round(num / 100.0) * 100;
}
float roundToNearest10(float num) {
    return round(num / 10.0) * 10;
}

void onRequest() {
  I2C.printf("%.*s", length, buffer); 
  I2C.print('\0');

  Serial.printf("onRequestCB\n");
}

void onReceive(int len) {
  if (len <= 1) return;
  if (len > sizeof(buffer)) len = sizeof(buffer);

  I2C.readBytes(buffer, len);

  uint8_t cmd = buffer[1];
  Serial.printf("[I2C] Raw bytes (%d): ", len);
  for (int i = 0; i < len; ++i) Serial.printf("0x%02X ", buffer[i]);
  Serial.println();
  Serial.printf("[I2C] CMD_ID: %d\n", cmd);

  switch (cmd) {
    case 1:
      Serial.println("I2C CMD: Homing");
      homeAll();
      break;

    case 2:
      Serial.println("I2C CMD: Move Full");
      setMotorTarget(motor_states[0], -4900, 10000);  // LOWER
      setMotorTarget(motor_states[1], -1300, 2600);   // UPPER
      setMotorTarget(motor_states[2], -1300, 5000);   // LEFT_TWIST
      setMotorTarget(motor_states[3], -2000, 4300);   // RIGHT_TWIST
      break;

    case 3:
      Serial.println("I2C CMD: Move Partial");
      setMotorTarget(motor_states[0], -4600 * 0.75, 10000);
      setMotorTarget(motor_states[1], 1300 * 0.75, 2600);
      setMotorTarget(motor_states[3], -2000 * 0.75, 4300);  // Only RIGHT_TWIST
      break;

    case 4:
      if (len >= 18) {
        int16_t pos1 = (buffer[2] << 8) | buffer[3];
        int16_t pos2 = (buffer[4] << 8) | buffer[5];
        int16_t pos3 = (buffer[6] << 8) | buffer[7];
        int16_t pos4 = (buffer[8] << 8) | buffer[9];
        int16_t spd1 = (buffer[10] << 8) | buffer[11];
        int16_t spd2 = (buffer[12] << 8) | buffer[13];
        int16_t spd3 = (buffer[14] << 8) | buffer[15];
        int16_t spd4 = (buffer[16] << 8) | buffer[17];

        Serial.printf("I2C CMD: Custom Move\n");
        Serial.printf("   L: %d @%d\n   U: %d @%d\n  LT: %d @%d\n  RT: %d @%d\n",
                      pos1, spd1, pos2, spd2, pos3, spd3, pos4, spd4);

        setMotorTarget(motor_states[0], pos1, spd1);  // LOWER
        setMotorTarget(motor_states[1], pos2, spd2);  // UPPER
        setMotorTarget(motor_states[2], pos3, spd3);  // LEFT_TWIST
        setMotorTarget(motor_states[3], pos4, spd4);  // RIGHT_TWIST
      } else {
        Serial.println("Invalid length for CMD 4");
      }
      break;

    default:
      Serial.printf("[I2C] Unknown command: %d\n", cmd);
  }
} 

void performHoming(TMC2209Stepper &driver, int threshold) {
  Serial.println("Starting sensorless homing...");

  const int retreatSpeed = -5000;
  const int homingSpeed = 7500;
  const unsigned long timeout = 5000; // 5 seconds max homing time
  unsigned long startTime;

  // Step 1: Move away from homing direction
  driver.VACTUAL(retreatSpeed);
  delay(350);
   Serial.println(driver.SG_RESULT());
  driver.VACTUAL(0);
  delay(200);
   Serial.println(driver.SG_RESULT());

  // Step 2: Begin homing movement
  Serial.println("Moving towards stall...");
  driver.VACTUAL(homingSpeed);
  delay(300); // Let motor reach speed
 Serial.println(driver.SG_RESULT());
  startTime = millis();
  while (driver.SG_RESULT() > threshold) {
    Serial.println(driver.SG_RESULT());
    // Add timeout for safety
    // if (millis() - startTime > timeout) {
    //   Serial.println("Homing failed: timeout reached.");
    //   driver.VACTUAL(0);
    //   return;
    // }
    delay(10); // Prevent spamming SG_RESULT too fast
  }

  // Stop motor
  driver.VACTUAL(0);
  delay(500); // Allow motion to fully stop

  HOMED[0] = true;
  Serial.println("Homing complete.");
}

void restDrivers(){
  for (byte i = 0; i < (sizeof(EN_PINS) / sizeof(EN_PINS[0])); i++){
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], LOW); 
  }
  delay(100);
  for (byte i = 0; i < (sizeof(EN_PINS) / sizeof(EN_PINS[0])); i++){
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], HIGH); 
  }
  delay(100);
  for (byte i = 0; i < (sizeof(EN_PINS) / sizeof(EN_PINS[0])); i++){
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], LOW); 
  }
}

void homeAll(){
  restDrivers();
  delay(2000);
  performHoming(V_LEFT_TWIST, 180);
  restDrivers();
  delay(2000);
  performHoming(V_RIGHT_TWIST, 250);
  restDrivers();
  delay(2000);
  performHoming(V_UPPER_LEVEL, 250);
  restDrivers();
  delay(2000);
  restDrivers();
  performHoming(V_LOWER_LEVEL, 250);
  delay(2000);
  // performHoming(V_LEFT_TWIST, 230);
  // delay(2000);
  // performHoming(V_LEFT_TWIST, 230);
  // delay(2000);
  

}
void initDrivers(){
  V_LEFT_TWIST.begin();
  V_LEFT_TWIST.toff(5);
  V_LEFT_TWIST.rms_current(800);
  V_LEFT_TWIST.microsteps(32);
  V_LEFT_TWIST.en_spreadCycle(true);
  V_LEFT_TWIST.pwm_autoscale(true);
  V_LEFT_TWIST.TCOOLTHRS(0xFFFFF);
  V_LEFT_TWIST.SGTHRS(50);
  V_LEFT_TWIST.semin(5);
  V_LEFT_TWIST.semax(2);

  V_LEFT_GRIPPER.begin();
  V_LEFT_GRIPPER.toff(5);
  V_LEFT_GRIPPER.rms_current(800);
  V_LEFT_GRIPPER.microsteps(32);
  V_LEFT_GRIPPER.en_spreadCycle(true);
  V_LEFT_GRIPPER.pwm_autoscale(true);
  V_LEFT_GRIPPER.TCOOLTHRS(0xFFFFF); 
  V_LEFT_GRIPPER.SGTHRS(10);
  V_LEFT_GRIPPER.semin(5);
  V_LEFT_GRIPPER.semax(2);

  V_LOWER_LEVEL.begin();
  V_LOWER_LEVEL.toff(5);
  V_LOWER_LEVEL.rms_current(800);
  V_LOWER_LEVEL.microsteps(32);
  V_LOWER_LEVEL.en_spreadCycle(true);
  V_LOWER_LEVEL.pwm_autoscale(true);
  V_LOWER_LEVEL.TCOOLTHRS(0xFFFFF);
  V_LOWER_LEVEL.SGTHRS(10);
  V_LOWER_LEVEL.semin(5);
  V_LOWER_LEVEL.semax(2);

  
  V_UPPER_LEVEL.begin();
  V_UPPER_LEVEL.toff(5);
  V_UPPER_LEVEL.rms_current(800);
  V_UPPER_LEVEL.microsteps(32);
  V_UPPER_LEVEL.en_spreadCycle(true);
  V_UPPER_LEVEL.pwm_autoscale(true);
  V_UPPER_LEVEL.TCOOLTHRS(0xFFFFF);
  V_UPPER_LEVEL.SGTHRS(10);
  V_UPPER_LEVEL.semin(5);
  V_UPPER_LEVEL.semax(2);

  V_RIGHT_GRIPPER.begin();
  V_RIGHT_GRIPPER.toff(5);
  V_RIGHT_GRIPPER.rms_current(800);
  V_RIGHT_GRIPPER.microsteps(32);
  V_RIGHT_GRIPPER.en_spreadCycle(true);
  V_RIGHT_GRIPPER.pwm_autoscale(true);
  V_RIGHT_GRIPPER.TCOOLTHRS(0xFFFFF); 
  V_RIGHT_GRIPPER.SGTHRS(10);
  V_RIGHT_GRIPPER.semin(5);
  V_RIGHT_GRIPPER.semax(2);

  V_RIGHT_TWIST.begin();
  V_RIGHT_TWIST.toff(5);
  V_RIGHT_TWIST.rms_current(800);
  V_RIGHT_TWIST.microsteps(32);
  V_RIGHT_TWIST.en_spreadCycle(true);
  V_RIGHT_TWIST.pwm_autoscale(true);
  V_RIGHT_TWIST.TCOOLTHRS(0xFFFFF); 
  V_RIGHT_TWIST.SGTHRS(50);
  V_RIGHT_TWIST.semin(5);
  V_RIGHT_TWIST.semax(2);

}

void moveToPosition(TMC2209Stepper &driver, long target_position, int v) {
  current_position = 0;
  long error = target_position - current_position;
  int velocity = 0;

  while (abs(error) > 0) {
    if (error > 0) {
      velocity = v; // Move forward
      current_position= current_position+1;
    } else {
      velocity = -v; // Move backward
      current_position=current_position-1;
    }

    driver.VACTUAL(velocity);
    Serial.print("VACTUAL: ");
    Serial.print(driver.VACTUAL());
    Serial.print(" Position: ");
    Serial.print(current_position);
    Serial.print(" Error: ");
    Serial.println(error);
    delayMicroseconds(200);  // Slow down loop to let motor react
    error = target_position - current_position;
  }

  // Ensure the motor stops
  driver.VACTUAL(0);
 
  // Small delay to let the command take effect
  delay(10);
}

void setMotorTarget(MotorState &motor, long target, int velocity) {
  motor.target_position = target;
  motor.max_velocity = velocity;
  motor.is_active = true;
}

void updateAllMotors() {
  static unsigned long last_update = millis();
  unsigned long now = millis();
  float dt = (now - last_update) / 1000.0;  // delta time in seconds
  last_update = now;

  for (int i = 0; i < NUM_MOTORS; ++i) {
    MotorState &m = motor_states[i];
    if (!m.is_active) continue;

    long error = m.target_position - m.current_position;

    if (abs(error) > 0) {
      int direction = (error > 0) ? 1 : -1;
      int v = direction > 0 ? m.max_velocity : -m.max_velocity;

      m.driver->VACTUAL(v);

      // Simulate position change based on velocity
      long delta_steps = v * dt;
      if (abs(delta_steps) > abs(error)) {
        delta_steps = error;  // do not overshoot
      }

      m.current_position += delta_steps;

    } else {
      m.driver->VACTUAL(0);
      m.is_active = false;
      Serial.printf("Motor %d done. Final position: %ld\n", i, m.current_position);
    }
  }
}