#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

#define I2C_SDA 26
#define I2C_SCL 25
#define I2C_DEV_ADDR 0x42
#define I2C_FREQ 100000


const int servoCount = 14;  // Define the total number of servos
int option; 
int position;
int command;
// Define servo parameters: {Pin, Min Pulse Width, Max Pulse Width, Open, Closed}
const int SERVO[servoCount][5] = {
    {15, 500, 2400, 55, 134},  // L_Twist_Servo
    {2, 500, 2400, 150, 105},    // L_Servo
    {4, 500, 2400, 110, 75},    // L_L_Servo
    {16, 500, 2400, 105, 80},    // L_CL_Servo
    {17, 500, 2400, 60, 87},   // L_CR_Servo
    {5, 500, 2400, 45, 87},   // L_R_Servo
    {18, 500, 2400, 50, 87},   // R_Servo
    {19, 500, 2400, 141, 65},  // R_Twist_Servo
    {21, 500, 2400, 120, 100},   // U_L_Servo
    {22, 500, 2400, 70, 90},   // U_R_Servo
    {23, 500, 2400, 50, 87},   // R_Servo
    {13, 500, 2400, 141, 65},  // R_Twist_Servo
    {12, 500, 2400, 120, 100},   // U_L_Servo
    {14, 500, 2400, 70, 90}   // U_R_Servo

};

// Declare an array of servo objects
Servo servos[servoCount];
TwoWire I2C = TwoWire(0);
uint8_t buffer[32];
uint8_t length = 0;


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


void setServos(int degrees) {
    for (int i = 0; i < servoCount; ++i) {
        servos[i].write(degrees);
    }
}

void setup() {
    Serial.begin(115200);  // Initialize serial communication
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    WiFi.mode(WIFI_STA);
    delay(100);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println(WiFi.macAddress()); // Print MAC address
    delay(1000);

    for (int i = 0; i < servoCount; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(SERVO[i][0], SERVO[i][1], SERVO[i][2]);  // Pin, Min Pulse, Max Pulse
    }

    I2C.onReceive(onReceive);
    I2C.onRequest(onRequest);
    I2C.begin(I2C_DEV_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
    Serial.println("Servo ESP32 ready (0x42)");
}

void loop() {
  delay(10);
  // print_received_data();
  // if (Serial.available() > 0) {
    
  //   String input = Serial.readStringUntil('\n');
  //   sscanf(input.c_str(), "%d %d %d",&command, &option, &position);
     
  //   switch (command) {
  //       case 0:
  //           set_LOWER_C_GRIPPERS(option);
  //           break;
  //       case 1:
  //           set_OUTER_C_GRIPPERS(option);
  //           break;
  //       case 2:
  //           set_OUTER_C_GRIPPERS(option);
  //           break;
  //       case 3:
  //           setTWIST(option, position);
      
  //       default:
  //           break;
  //   }
  //   }

  
      // set_LOWER_C_GRIPPERS(data.SW_R_state);
      // set_OUTER_C_GRIPPERS(data.SW_C_state);
      // set_UPPER_C_GRIPPERS(data.SW_C_state);
      // setTWIST(2, data.SW_L_state);
}


// void set_P_GRIPPER(int g){};

void set_LOWER_C_GRIPPERS(int position) {
  int index = (position==1) ? 3 : 4;
  for (int i = 2; i < 6; ++i) {
    servos[i].write(SERVO[i][index]);
  }
}
void set_OUTER_C_GRIPPERS(int position) {
  int index = (position==1) ? 3 : 4;
  servos[1].write(SERVO[1][index]);
  servos[6].write(SERVO[6][index]);
}
void set_UPPER_C_GRIPPERS(int position) {
    int index = (position==1) ? 3 : 4;

    // OPEN/CLOSE UPPER with Outer servos
    for (int i = 8; i < 10; ++i) {
        servos[i].write(SERVO[i][index]);
    }
}
void setTWIST(int option, int position) {
    /* This functions open or closed either or both 
    twist servos based on parameters*/
    /* option 0 left, 1 right, 2 both servos*/
    /* position 0 open, 1 closed*/ 

   
    // Mapping postion to array index
    int index = (position == 1) ? 3 : 4;

    // Acutaute Twist servos
    switch (option) {
        case 0:
            servos[0].write(SERVO[0][index]);
            break;
        case 1:
            servos[7].write(SERVO[7][index]);
            break;
        case 2:
            servos[0].write(SERVO[0][index]);
            servos[7].write(SERVO[7][index]);
            break;
    }}

void onReceive(int len) {
  if (len <= 2) return;  // Must have at least register + cmd + arg
  if (len > sizeof(buffer)) len = sizeof(buffer);
  
  I2C.readBytes(buffer, len);

  uint8_t cmd = buffer[1];
  int arg = static_cast<int>(buffer[2]);  // Ensure it's interpreted correctly

  Serial.printf("[I2C] CMD: %d | ARG: %d\n", cmd, arg);

  switch (cmd) {
    case 1:
      Serial.println("I2C CMD 1: LOWER_C_GRIPPERS");
      set_LOWER_C_GRIPPERS(arg);
      break;
    case 2:
      Serial.println("I2C CMD 2: UPPER + OUTER GRIPPERS");
      set_UPPER_C_GRIPPERS(arg);
      set_OUTER_C_GRIPPERS(arg);
      break;
    case 3:
      Serial.println("I2C CMD 3: TWIST");
      setTWIST(2, arg);
      break;
    default:
      Serial.printf("Unknown I2C command: %d\n", cmd);
  }
}

void onRequest() {
  I2C.printf("%.*s", length, buffer); 
  I2C.print('\0');

  Serial.printf("onRequestCB\n");
}

void test(){
  for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
          setServos(posDegrees);
        // Serial.println(posDegrees);
          delay(20);
      }

      for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {
          setServos(posDegrees);
        // Serial.println(posDegrees);
          delay(20);
      }}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&data, incomingData, sizeof(data));}
void print_received_data() {
  // Serial.printf("X: %d Y: %d Z: %d BTN_JOYSTICK: %d POT_R: %d POT_C: %d POT_L: %d\n",
  //               data.X_JOYSTICK_state, data.Y_JOYSTICK_state, data.Z_JOYSTICK_state,
  //               data.BTN_JOYSTICK_state, data.POT_R_state, data.POT_C_state, data.POT_L_state);
  // Serial.printf("BTN_R: %d BTN_C: %d BTN_L: %d SW_R: %d SW_C: %d SW_L: %d\n",
  //               data.BTN_R_state, data.BTN_C_state, data.BTN_L_state,
  //               data.SW_R_state, data.SW_C_state, data.SW_L_state);
                }