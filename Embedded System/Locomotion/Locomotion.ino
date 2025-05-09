#include <FastAccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TMCStepper.h>  // TMCstepper - https://github.com/teemuatlut/TMCStepper
#include <ESP32Servo.h>

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

int EN_PINS [] = {15, 16, 4, 2, 22, 23}; 
int DIAG_PINS [] = {36, 39, 34, 35, 32, 33}; // DIAG - Diagnositc Output used for stall detection 
int THRS [] = {180, 180, 120, 180, 250, 250, 250, 250};
bool HOMED [] = {0, 0, 0, 0, 0, 0, 0, 0};


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
float v_x = 100;
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

// TMC2209 Driver Initiliasiationn with two Serial buses
HardwareSerial TMCSerial_1(2); 

// TMC2209Stepper TMCdriver_4(&TMCSerial_1, R_SENSE, 0b11);

HardwareSerial TMCSerial_2(1); 
TMC2209Stepper TMCdriver_1(&TMCSerial_1, R_SENSE, 0b00);
TMC2209Stepper TMCdriver_2(&TMCSerial_1, R_SENSE, 0b10);
TMC2209Stepper TMCdriver_3(&TMCSerial_1, R_SENSE, 0b11);
// TMC2209Stepper TMCdriver_4(&TMCSerial_2, R_SENSE, 0b00);
// TMC2209Stepper TMCdriver_5(&TMCSerial_2, R_SENSE, 0b10);
// TMC2209Stepper TMCdriver_6(&TMCSerial_2, R_SENSE, 0b11);
// Structure to handle moving averages for different values
const int N = 5;  // Moving average window size

struct MovingAverage {
    float history[N] = {0};  // Buffer for past values
    int index = 0;
    
    float compute(float new_value) {
        history[index] = new_value;
        index = (index + 1) % N;

        float sum = 0;
        for (int i = 0; i < N; i++) {
            sum += history[i];
        }
        return sum / N;
    }
};
MovingAverage omega_filter;
MovingAverage v_x_filter;
MovingAverage v_y_filter;

void setup() {
  Serial.begin(115200);           // Initialize hardware serial for debugging

  // Initialize UART2 for TMC2209
  TMCSerial_1.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);  
  TMCSerial_2.begin(115200, SERIAL_8N1, UART0_RX, UART0_TX);  
  // Note: USB debuging will not be avaliable UART0 is used to drive the Steppers motors

  
  // board1.begin();
  // board1.setPWMFreq(60); 
  for (byte i = 0; i < (sizeof(EN_PINS) / sizeof(EN_PINS[0])); i++){
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], HIGH); 
  }

  
  delay(500);
  for (byte i = 0; i < (sizeof(EN_PINS) / sizeof(EN_PINS[0])); i++){
    pinMode(EN_PINS[i], OUTPUT);
    digitalWrite(EN_PINS[i], LOW); 
  }

  
  for (byte i = 0; i < (sizeof(DIAG_PINS) / sizeof(DIAG_PINS[0])); i++){
    pinMode(DIAG_PINS[i], INPUT_PULLUP);
  }


  
       // Enable TMC2209 board  

  TMCdriver_1.begin();
  TMCdriver_1.toff(5);
  TMCdriver_1.rms_current(850);
  TMCdriver_1.microsteps(8);
  // TMCdriver_1.en_spreadCycle(true);
  // TMCdriver_1.pwm_autoscale(true);
  // TMCdriver_1.TCOOLTHRS(0xFFFFF); // 20bit max
  // TMCdriver_1.SGTHRS(256);
  // TMCdriver_1.semin(5);
  // TMCdriver_1.semax(2);


  TMCdriver_2.begin();
  TMCdriver_2.toff(5);
  TMCdriver_2.rms_current(850);
  TMCdriver_2.microsteps(8);
  // TMCdriver_2.en_spreadCycle(true);
  // TMCdriver_2.pwm_autoscale(true);
  // TMCdriver_2.TCOOLTHRS(0xFFFFF); // 20bit max
  // TMCdriver_2.SGTHRS(256);
  // TMCdriver_2.semin(5);
  // TMCdriver_2.semax(2);

  TMCdriver_3.begin();
  TMCdriver_3.toff(5);
  TMCdriver_3.rms_current(850);
  TMCdriver_3.microsteps(8);
  // TMCdriver_3.en_spreadCycle(true);
  // TMCdriver_3.pwm_autoscale(true);
  // TMCdriver_3.TCOOLTHRS(0xFFFFF); // 20bit max
  // TMCdriver_3.SGTHRS(256);
  // TMCdriver_3.semin(5);
  // TMCdriver_3.semax(2);


  delay(100);

  // WiFi initialization for ESP-NOW
  WiFi.mode(WIFI_STA);
  delay(100);
  if (esp_now_init() != ESP_OK) {
      // Serial.println("ESP-NOW Init Failed");
      return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  //performHoming();
  // delay(2000);



}

void loop() {
  float raw_omega = (map(roundToNearest100(data.POT_C_state)-1600, 0-1600, 4100-1600, -10, 6)+2);
  float raw_v_x = map(roundToNearest100(data.POT_L_state)-2100, 0-2100, 4100-2100, -max_v_x, max_v_x)+15;
  float raw_v_y = map(roundToNearest100(data.X_JOYSTICK_state)-2100, 0-2100, 4100-2100, -max_v_y, max_v_y)-43-30;
  
  v_x = v_x_filter.compute(raw_v_x);
  v_y = v_y_filter.compute(raw_v_y);
  omega = omega_filter.compute(raw_omega)+v_y_filter.compute(raw_v_y)*0.0005;
  scalar = roundToNearest10(int(map(data.POT_R_state,0 ,4096, 1, 1000)));
  // Conversion from motor speed into sutible step intervals 
  MOT_1_speed = ((1.0/r)*((-d*omega)+v_x))*scalar;
  MOT_2_speed = ((1.0/r)*((-d*omega)-(v_x*0.5)-(sin(PI*0.333)*v_y)))*scalar;
  MOT_3_speed = ((1.0/r)*((-d*omega)-(v_x*0.5)+(sin(PI*0.333)*v_y)))*scalar;
  print_speeds();



// TMCdriver_4.VACTUAL(5000);
// TMCdriver_5.VACTUAL(5000);
// TMCdriver_6.VACTUAL(5000);

// TMCdriver_1.shaft(dir);
// TMCdriver_2.shaft(dir);
// TMCdriver_3.shaft(dir);
// TMCdriver_4.shaft(dir);
// TMCdriver_5.shaft(dir);
// TMCdriver_6.shaft(dir);

// dir != dir;
// float omega = int(map(data.POT_C_state,0 ,4096, -10, 10));
// float v_x = map(roundToNearest100(data.X_JOYSTICK_state)-2100, 0-2100, 4100-2100, -max_v_x, max_v_x)+15-88;
// float v_y = map(roundToNearest100(data.Y_JOYSTICK_state)-2100, 0-2100, 4100-2100, -max_v_y, max_v_y)-43+29;
// scalar = roundToNearest100(int(map(data.POT_R_state,0 ,4096, 1, 1000)));

// MOT_1_speed = ((1.0/r) * (-d * omega + v_x)) *scalar;
// MOT_2_speed = ((1.0/r) * (-d * omega + v_x * cos(2*PI/3) + v_y * sin(2*PI/3))) * scalar;
// MOT_3_speed = ((1.0/r) * (-d * omega + v_x * cos(4*PI/3) + v_y * sin(4*PI/3))) * scalar;

// Serial.printf( "%f    %f    %f   %d    %f    %f   %f  \n",omega, v_x, v_y,scalar,  MOT_1_speed, MOT_2_speed, MOT_3_speed);
// if (Serial.available() > 0) {
//     String input = Serial.readStringUntil('\n');
//     sscanf(input.c_str(), "%f %f %f %d", &omega, &v_x, &v_y, &scalar);

//   // Serial.printf("M1= %d, M2= %d, M3= %d,\n", 
//   // TMCdriver_1.SG_RESULT(), TMCdriver_2.SG_RESULT(), TMCdriver_3.SG_RESULT());
//   // Conversion from motor speed into sutible step intervals 
//   // MOT_1_speed = ((1.0/r)*((-d*omega)+v_x));
//   // MOT_2_speed = ((1.0/r)*((-d*omega)-(v_x*0.5)-(sin(PI*0.333)*v_y)));
//   // MOT_3_speed = ((1.0/r)*((-d*omega)-(v_x*0.5)+(sin(PI*0.333)*v_y)));


//   MOT_1_speed = ((1.0/r) * (-d * omega + v_x)) *scalar;
//   MOT_2_speed = ((1.0/r) * (-d * omega + v_x * cos(2*PI/3) + v_y * sin(2*PI/3))) * scalar;
//   MOT_3_speed = ((1.0/r) * (-d * omega + v_x * cos(4*PI/3) + v_y * sin(4*PI/3))) * scalar;

//   Serial.printf( "%f    %f    %f   %d    %f    %f   %f  \n",omega, v_x, v_y,scalar,  MOT_1_speed, MOT_2_speed, MOT_3_speed);

// }

  TMCdriver_1.VACTUAL(MOT_1_speed);
  TMCdriver_2.VACTUAL(MOT_2_speed);
  TMCdriver_3.VACTUAL(MOT_3_speed);
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

void performHoming(TMC2209Stepper &driver, int threshold) {
  // Serial.println("Starting sensorless homing...");
  // driver.shaft(true);
  driver.VACTUAL(7500);

  delay(100);
  Serial.print("   |SG_RESULT: ");
    Serial.println(driver.SG_RESULT());
  while (driver.SG_RESULT()>threshold) {
    
    driver.VACTUAL(7500);
  }

  driver.VACTUAL(0);
  delay(500);
  HOMED[0] = true;

  // TMCdriver_1.shaft(false);
  // while (digitalRead(DIAG_PINS[0]) == LOW) {
  //   TMCdriver_1.VACTUAL(5000);
  // }

  // TMCdriver_1.VACTUAL(0);
  Serial.println("Homing complete.");
}
