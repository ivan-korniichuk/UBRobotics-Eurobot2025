#include <FastAccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <TMCStepper.h>  // TMCstepper - https://github.com/teemuatlut/TMCStepper

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);       // called this way, it uses the default address 0x40   

#define SERVOMIN  125                                                 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625                                                 // this is the 'maximum' pulse length count (out of 4096)

// Vehicle parameters used for motor speed calculations 
#define r 58   // Wheel radius in mm
#define d 120  // Distance from center to center of wheel in mm 
#define PI 3.14159265358979323

#define EN_PIN          36      // Enable - PURPLE
#define DIR_PIN         14      // Direction - WHITE
#define STEP_PIN        27      // Step - ORANGE
#define UART2_TX        17      // UART2 TX Pin (ESP32 Default TX2)
#define UART2_RX        18      // UART2 RX Pin (ESP32 Default RX2)
#define DRIVER_ADDRESS  0b01    // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE         0.11f

int accel;
long maxSpeed;
int speedChangeDelay;
bool dir = false;

// Use ESP32 Hardware Serial2
HardwareSerial TMCSerial(1);
TMC2209Stepper TMCdriver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);

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

void setup() {
  Serial.begin(115200);           // Initialize hardware serial for debugging
  TMCSerial.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);  // Initialize UART2 for TMC2209
  // TMCdriver.beginSerial(115200);
  
  board1.begin();
  board1.setPWMFreq(60); 

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable TMC2209 board  

  TMCdriver_1.begin();
  TMCdriver_1.toff(5);
  TMCdriver_1.rms_current(500);
  TMCdriver_1.microsteps(256);
  TMCdriver_1.en_spreadCycle(false);
  TMCdriver_1.pwm_autoscale(true);

  delay(100);

  // WiFi initialization for ESP-NOW
  WiFi.mode(WIFI_STA);
  delay(100);
  if (esp_now_init() != ESP_OK) {
      Serial.println("ESP-NOW Init Failed");
      return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  accel = 10000;
  maxSpeed = 50000;
  speedChangeDelay = 100;
  
   for(int i=0; i<16; i++)
        { board1.setPWM(i, 0, angleToPulse(0) );}
      delay(1000);
   
      for( int angle =0; angle<181; angle +=10)
        { for(int i=0; i<16; i++)
            { board1.setPWM(i, 0, angleToPulse(angle) );}
        }
      delay(100);

  TMCdriver.VACTUAL(5000);

  //performHoming(TMCdriver_1, DIAG_PINS[0]);
  // performHoming(TMCdriver_2, DIAG_PINS[1]);
  // performHoming(TMCdriver_3, DIAG_PINS[2]);
  // performHoming(TMCdriver_4, DIAG_PINS[3]);
  // for (long i = 0; i <= maxSpeed; i += accel) {
  //   TMCdriver.VACTUAL(i);
  //   Serial.println(TMCdriver.VACTUAL());
  //   delay(100);
  // }
  
  // for (long i = maxSpeed; i >= 0; i -= accel) {
  //   TMCdriver.VACTUAL(i);
  //   Serial.println(TMCdriver.VACTUAL());
  //   delay(100);
  // }  
  
  // dir = !dir; // Reverse direction
  // TMCdriver.shaft(dir);
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&data, incomingData, sizeof(data));
}
int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     Serial.print("Angle: ");Serial.print(ang);
     Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
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


void performHoming(TMC2209Stepper &driver, int diagPin) {
  Serial.println("Starting sensorless homing...");
  driver.shaft(true);

  while (digitalRead(diagPin) == HIGH) {
    driver.VACTUAL(5000);
  }

  driver.VACTUAL(0);
  delay(500);

  driver.shaft(false);
  while (digitalRead(diagPin) == HIGH) {
    driver.VACTUAL(5000);
  }

  driver.VACTUAL(0);
  Serial.println("Homing complete.");
}

