#include <esp_now.h>
#include <WiFi.h>

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

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&data, incomingData, sizeof(data));
    print_recived_data();
    // Serial.print("Received from MAC: ");
    // for (int i = 0; i < 6; i++) {
    //     Serial.printf("%02X", info->src_addr[i]);
    //     if (i < 5) Serial.print(":");
    // }

    
  



}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    delay(100);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    Serial.println(WiFi.macAddress()); // Print MAC address
    
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {}
