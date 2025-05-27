#include <esp_now.h>
#include <WiFi.h>
#include "vlookup.h"
#include "motor_v3.h"

Motor_control wheelL, wheelR, funElement;

// Set the wifi channel (0-13)
#define _channel 0
byte localMAC[6];
bool MAC_found, MAC_isnz = false;
uint8_t controller_Num = 255;
esp_now_peer_info_t peerInfo;

uint32_t t_lastPacket = 0,
         t_lastFunElement = 0;

typedef struct TargetCommands {
  uint8_t cmd;
  uint8_t targetID;
  uint8_t data[6];
} targetCommands;
targetCommands dataIn, dataOut;

bool funElementToggle = false;
bool funElementUp     = true;

void Wifi_begin()
{ 
  bool MAC_found, MAC_isnz = false;
  int8_t i, j;

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);
  //WiFi.channel(channel_);
  while (!MAC_isnz){
    delay(50);
    WiFi.macAddress(localMAC);
    Serial.print("My MAC is ");
    MAC_isnz = true;
    for (i = 0; i <= 5; i++){
      Serial.print(localMAC[i], HEX);
      Serial.print(":");
      if (localMAC[i] == 0x00) {
        MAC_isnz = false;
      }
    }
    Serial.println();
  }

  for (i = 0; i <= MAC_ADDRESS_NUM -1; i++){
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
  //Serial.println("Packet Recieved");
  memcpy(&dataIn, incomingData, sizeof(dataIn));
  t_lastPacket = millis() + 3000;
  Serial.printf("[%2d] | CMD: 0x%2X 0x%2.2X%2.2X%2.2X", controller_Num, dataIn.cmd, dataIn.data[0], dataIn.data[1], dataIn.data[2]);
  Serial.printf("%2.2X 0x%2.2X%2.2X%2.2X%2.2X  %X\n", dataIn.data[3], dataIn.data[4], dataIn.data[5], dataIn.data[6], dataIn.data[7], millis());
  Wifi_Process();
  //Serial.print(dataIn.data);
  //Serial.println();
}

void ohNo(){
  //fix it pls.
  //fix what?
  Serial.print("Failed to send msg, check if peer is online");
  //flah LED or something
}

void Wifi_Process()
{
  switch(dataIn.cmd){
    case 0x08:  //stop
      wheelL.stop();
      wheelR.stop();
      break;

    case 0x09:  //move
      wheelL.speed(dataIn.data[0], dataIn.data[1] && 0x01);
      wheelR.speed(dataIn.data[2], dataIn.data[3] && 0x01);
      break;

    case 0x0A:  //dance
      funElementToggle = dataIn.data[0] && 0x01;
      break;
  }
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  wheelL.begin(17, 18);
  wheelR.begin(34, 33);
  funElement.begin(4, 5); //litte arm go bruuuu
  Wifi_begin();
  //WS2812B on pin48
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() > t_lastPacket){
    wheelL.stop();
    wheelR.stop();
  }

  if (millis() > t_lastFunElement){
    if (funElementUp){
      funElement.speed(100, 1);
    } else {
      if (funElementToggle){
        funElement.speed(100, 0);
      } else {
        funElement.stop();
      }
    }
    funElementUp = !funElementUp;
    t_lastFunElement = millis() + 500;
  }

  //Serial.println(controller_Num);
  delay(20);
  //Serial.printf("Hello World! \n");
}
