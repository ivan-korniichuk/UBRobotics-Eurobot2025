#include <esp_now.h>
#include <WiFi.h>
#include "vlookup.h"

// Set the wifi channel (0-13)
#define _channel 0
byte localMAC[6];
bool MAC_found, MAC_isnz = false;
uint8_t controller_Num = 255;
esp_now_peer_info_t peerInfo;

uint32_t  t_lastPacket      = 0,
          t_lastSent        = 0,
          t_lastFunElement  = 0;

typedef struct TargetCommands{
  uint8_t cmd;
  uint8_t data[8];
} targetCommands;
targetCommands dataIn, dataOut;

void Wifi_begin()
{ 
  bool MAC_found, MAC_isnz = false;
  uint8_t i, j;

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
  //WiFi.channel(channel_);
  while (!MAC_isnz){
    delay(50);
    WiFi.macAddress(localMAC);
    Serial.print("My MAC is ");
    MAC_isnz = true;
    for (i = 0; i <= 5; i++){
      Serial.print(localMAC[5 - i], HEX);
      Serial.print(":");
      if (localMAC[5 - i] == 0x00) {
        MAC_isnz = false;
      }
    }
    Serial.println();
  }

  for (i = 0; i <= 6; i++){
    MAC_found = true;
    for (j = 0; j <= 5; j++){ 
      if (localMAC[j] != MAC_address[i][j]){
        MAC_found = false;
        break;
      }
    }
    if (MAC_found){
      controller_Num = i; //use this to identify itself
      break;
    }
  }
  #ifdef MOTOR_DEFAULT
    MAC_found = false;
  #endif

  esp_now_init();
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
}

void Wifi_addPeer(uint8_t conNum){
  uint8_t count = 0;
  memcpy(peerInfo.peer_addr, MAC_address[conNum], 6);
  peerInfo.channel = _channel;
  peerInfo.encrypt = false;
  while((count <= 3)&&(esp_now_add_peer(&peerInfo) != ESP_OK)){
    Serial.println("Connection failed, retrying....");
    delay(20);
    count++;
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  if (status != ESP_NOW_SEND_SUCCESS){
    ohNo();
  }
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&dataIn, incomingData, sizeof(dataIn));
  t_lastPacket = millis() + 300;
  Wifi_Process();
  //Serial.print(dataIn.data);
  //Serial.println();
}

void ohNo(){
  //fix it pls.
  //fix what?
  Serial.println("Failed to send msg, check if peer is online");
  //flah LED or something
}

void Wifi_Process()
{
  switch(dataIn.cmd){
    case 0x00: // sync
      break;
      
    /*case 0x08:  //stop
      wheelL.stop();
      wheelR.stop();
      break;

    case 0x09:  //move
      wheelL.speed(dataIn.data[0], dataIn.data[1] && 0x01);
      wheelR.speed(dataIn.data[2], dataIn.data[3] && 0x01);
      break;

    case 0x0A:  //dance
      funElementToggle = dataIn.data[0] && 0x01;
      break;*/
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wifi_begin();
  Wifi_addPeer(0);
  Wifi_addPeer(1);
  Wifi_addPeer(2);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("forward");
  dataOut.cmd = 0x09;
  dataOut.data[0] = 255;
  dataOut.data[1] = 1;
  dataOut.data[2] = 255;
  dataOut.data[3] = 1;
  if(millis() > t_lastSent){
    t_lastSent = millis() + 50;
    esp_err_t sentSuccessfully0 = esp_now_send(MAC_address[0], (uint8_t *) &dataOut, sizeof(dataOut));
    esp_err_t sentSuccessfully1 = esp_now_send(MAC_address[1], (uint8_t *) &dataOut, sizeof(dataOut));
    esp_err_t sentSuccessfully2 = esp_now_send(MAC_address[2], (uint8_t *) &dataOut, sizeof(dataOut));
  }
  delay(750);

  Serial.println("backward");
  dataOut.cmd = 0x09;
  dataOut.data[0] = 255;
  dataOut.data[1] = 0;
  dataOut.data[2] = 255;
  dataOut.data[3] = 0;
  if(millis() > t_lastSent){
    t_lastSent = millis() + 50;
    esp_err_t sentSuccessfully0 = esp_now_send(MAC_address[0], (uint8_t *) &dataOut, sizeof(dataOut));
    esp_err_t sentSuccessfully1 = esp_now_send(MAC_address[1], (uint8_t *) &dataOut, sizeof(dataOut));
    esp_err_t sentSuccessfully2 = esp_now_send(MAC_address[2], (uint8_t *) &dataOut, sizeof(dataOut));
  }
  delay(750);

  Serial.println("stop");
  dataOut.cmd = 0x08;
  dataOut.data[0] = 0;
  dataOut.data[1] = 0;
  dataOut.data[2] = 0;
  dataOut.data[3] = 0;
  if(millis() > t_lastSent){
    t_lastSent = millis() + 50;
    esp_err_t sentSuccessfully0 = esp_now_send(MAC_address[0], (uint8_t *) &dataOut, sizeof(dataOut));
    esp_err_t sentSuccessfully1 = esp_now_send(MAC_address[1], (uint8_t *) &dataOut, sizeof(dataOut));
    esp_err_t sentSuccessfully2 = esp_now_send(MAC_address[2], (uint8_t *) &dataOut, sizeof(dataOut));
  }
  delay(750);

}
