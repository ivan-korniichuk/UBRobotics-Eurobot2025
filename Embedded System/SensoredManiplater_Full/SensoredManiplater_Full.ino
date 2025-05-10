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
  // delay(2000);
  // performHoming(V_LEFT_TWIST, 160);
  // delay(2000);
  // // performHoming(V_LOWER_LEVEL, 260);
  // // delay(2000);
  // // performHoming(V_UPPER_LEVEL, 260);
  // delay(2000);
  // performHoming(V_RIGHT_TWIST, 260);
  // // V_RIGHT_TWIST.VACTUAL(5000);
  // delay(2000);

  // // V_LEFT_TWIST.XACTUAL(-5000);
  // moveToPosition(V_LEFT_TWIST, -500);  // Move to 800 steps
  // // moveToPosition(V_UPPER_LEVEL, -500);  // Move to 800 steps
  // moveToPosition(V_RIGHT_TWIST, -1000);  // Move to 800 steps
  // delay(1000);

  // homeAll();
  

}

void loop() {
  if (data.BTN_R_state == LOW){
      delay(2000);
    performHoming(V_LEFT_TWIST, 180);
    delay(2000);
    performHoming(V_RIGHT_TWIST, 250);
    delay(2000);
    performHoming(V_UPPER_LEVEL, 250);
    delay(2000);
    performHoming(V_LOWER_LEVEL, 25
    0);
    delay(2000);
    
    // V_RIGHT_TWIST.VACTUAL(5000);
    delay(2000);
  }
  if (data.BTN_C_state == LOW){
  // V_LEFT_TWIST.XACTUAL(-5000);
    moveToPosition(V_LOWER_LEVEL, -4900, 10000);  // Move to 800 steps
    moveToPosition(V_UPPER_LEVEL, -1300, 2600);  // Move to 800 steps
    moveToPosition(V_LEFT_TWIST, -1300, 5000);  // Move to 800 steps
    
    moveToPosition(V_RIGHT_TWIST, -2000,4300);  // Move to 800 steps
    delay(1000);
  }
  delay(10);
  if (data.BTN_L_state == LOW){
  // V_LEFT_TWIST.XACTUAL(-5000);
    moveToPosition(V_LOWER_LEVEL, -4600*0.75, 10000);  // Move to 800 steps
    moveToPosition(V_UPPER_LEVEL, 1300*0.75, 2600);  // Move to 800 steps
    moveToPosition(V_RIGHT_TWIST, -2000*0.75,4300);  // Move to 800 steps
    delay(1000);
  }
   

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


// void performHoming(TMC2209Stepper &driver, int threshold) {
//   // Serial.println("Starting sensorless homing...");
//   // driver.shaft(true);
//   driver.VACTUAL(7500);

//   delay(100);
//   Serial.print("   |SG_RESULT: ");
//     Serial.println(driver.SG_RESULT());
//   while (driver.SG_RESULT()>threshold) {
    
//     driver.VACTUAL(7500);
//   }

//   driver.VACTUAL(0);
//   delay(500);
//   HOMED[0] = true;

//   // V_LEFT_TWIST.shaft(false);
//   // while (digitalRead(DIAG_PINS[0]) == LOW) {
//   //   V_LEFT_TWIST.VACTUAL(5000);
//   // }

//   // V_LEFT_TWIST.VACTUAL(0);
//   Serial.println("Homing complete.");
// // }
// void performHoming(TMC2209Stepper &driver, int threshold) {
//   Serial.println("Starting sensorless homing...");

//   // Step 1: Move away from the homing direction briefly
//   driver.VACTUAL(-5000);  // Move backward a little
//   delay(350);
//   driver.VACTUAL(0);
//   delay(200);

//   // Step 2: Move forward until stall is detected
//   driver.VACTUAL(7500);

//   delay(450); // let it spin up

//   while (driver.SG_RESULT() > threshold) {
//     driver.VACTUAL(7500); // keep pushing forward
//     // Optional: add timeout here for safety
//   }

//   driver.VACTUAL(0);
//   delay(500);

//   HOMED[0] = true;

//   Serial.println("Homing complete.");
// }
void onRequest() {
  I2C.printf("%.*s", length, buffer); 
  I2C.print('\0');

  Serial.printf("onRequestCB\n");
}

void onReceive(int len) {
  length = len;  // cat is cat, okay?
  Serial.printf("onReceiveCB[%d]: \n", len);
  while (I2C.available()) {
    I2C.readBytes(buffer, len);
    log_print_buf(buffer, len);
    Serial.printf("%s\n", buffer);
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
  performHoming(V_RIGHT_TWIST, 225);
  restDrivers();
  delay(2000);
  performHoming(V_UPPER_LEVEL, 320);
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
