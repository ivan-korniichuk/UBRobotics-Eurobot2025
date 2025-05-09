#include <esp_now.h>
#include <WiFi.h>

// Replace with the receiver MAC address
uint8_t receiver_1_MAC[] = {0x14, 0x2b, 0x2f, 0xc1, 0xf5, 0x6c}; // Maniplater Stepper and DC 
uint8_t Manipulator_MAC[] = {0xEC, 0x64, 0xC9, 0x7B, 0x8D, 0xEC}; // Servos 
uint8_t Gripper_MAC[] = {0xec, 0x64, 0xc9, 0x7b, 0xb3, 0x5c}; // Locomotion 

// Define pins
#define X_JOYSTICK     36
#define Y_JOYSTICK     39
#define Z_JOYSTICK     34
#define POT_R          35
#define POT_C          32
#define POT_L          33
#define BTN_JOYSTICK   23



#define BTN_R          14
#define BTN_C          12
#define BTN_L          13

#define SW_R           21
#define SW_C           19
#define SW_L           18



struct Send_Data {
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
Send_Data data;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.print("Send Status: ");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);

    // Pin modes
    pinMode(X_JOYSTICK, INPUT);
    pinMode(Y_JOYSTICK, INPUT);
    pinMode(Z_JOYSTICK, INPUT);

    pinMode(POT_R, INPUT);
    pinMode(POT_C, INPUT);
    pinMode(POT_L, INPUT);

    pinMode(BTN_JOYSTICK, INPUT_PULLUP);


    pinMode(BTN_R, INPUT_PULLUP); 
    pinMode(BTN_C, INPUT_PULLUP);
    pinMode(BTN_L, INPUT_PULLUP);

    pinMode(SW_R, INPUT_PULLUP);
    pinMode(SW_C, INPUT_PULLUP);
    pinMode(SW_L, INPUT_PULLUP);

    WiFi.mode(WIFI_STA);
    delay(100);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    memset(&peerInfo, 0, sizeof(peerInfo)); // Initialize peerInfo
    memcpy(peerInfo.peer_addr, receiver_1_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer 1");
        return;
    } else {
        Serial.println("Peer 1 added successfully");
    }

    memcpy(peerInfo.peer_addr, Manipulator_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer 2");
        return;
    } else {
        Serial.println("Peer 2 added successfully");
    } 

    memcpy(peerInfo.peer_addr, Gripper_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer 3");
        return;
    } else {
        Serial.println("Peer 3 added successfully");
    } 
}

void loop() {
    data.X_JOYSTICK_state = analogRead(X_JOYSTICK);
    data.Y_JOYSTICK_state = analogRead(Y_JOYSTICK);
    data.Z_JOYSTICK_state = analogRead(Z_JOYSTICK);

    data.POT_R_state = analogRead(POT_R);
    data.POT_C_state = analogRead(POT_C);
    data.POT_L_state = analogRead(POT_L);

    data.BTN_JOYSTICK_state = digitalRead(BTN_JOYSTICK);
    data.BTN_R_state = digitalRead(BTN_R);
    data.BTN_C_state = digitalRead(BTN_C);
    data.BTN_L_state = digitalRead(BTN_L);
    data.SW_R_state = digitalRead(SW_R);
    data.SW_C_state = digitalRead(SW_C);
    data.SW_L_state = digitalRead(SW_L);

    

    esp_err_t result = esp_now_send(receiver_1_MAC, (uint8_t *)&data, sizeof(data));
    esp_err_t result_2 = esp_now_send(Manipulator_MAC, (uint8_t *)&data, sizeof(data));
    esp_err_t result_3 = esp_now_send(Gripper_MAC, (uint8_t *)&data, sizeof(data));
    if (result == ESP_OK) {
        // Serial.println("Sent successfully");
        print_recived_data();}
    // } else {
    //     Serial.println("Send failed");
        
    // }
    // print_recived_data();
    delay(50);  // Increased delay for stability
}



void print_recived_data(){
  Serial.print("X:  ");
  Serial.print(data.X_JOYSTICK_state);
  Serial.print("   Y: ");
  Serial.print(data.Y_JOYSTICK_state);
  Serial.print("   Z: ");
  Serial.print(data.Z_JOYSTICK_state);

  Serial.print("   BTN_JOYSTICK: ");
  Serial.print(data.BTN_JOYSTICK_state);


  Serial.print("   POT_R:  ");
  Serial.print(data.POT_R_state);
  Serial.print("   POT_C: ");
  Serial.print(data.POT_C_state);
  Serial.print("   POT_L: ");
  Serial.print(data.POT_L_state);

  Serial.print("   BTN_R:  ");
  Serial.print(data.BTN_R_state);
  Serial.print("   BTN_C: ");
  Serial.print(data.BTN_C_state);
  Serial.print("   BTN_L: ");
  Serial.print(data.BTN_L_state);
    
  Serial.print("   SW_R:  ");
  Serial.print(data.SW_R_state);
  Serial.print("   SW_C: ");
  Serial.print(data.SW_C_state);
  Serial.print("   SW_L: ");
  Serial.println(data.SW_L_state);

}