#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "vlookup.h"

#define _channel 0
#define I2C_ADDRESS 0x12

byte localMAC[6];
esp_now_peer_info_t peerInfo;

typedef struct TargetCommands {
  uint8_t cmd;         // Command ID (e.g., 0x09)
  uint8_t targetID;    // Destination robot ID (0, 1, 2, 3, or 0xFF for broadcast)
  uint8_t data[6];     // Payload data
} targetCommands;


targetCommands dataIn;

void Wifi_begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);

  while (true) {
    WiFi.macAddress(localMAC);
    bool allZero = true;
    for (int i = 0; i < 6; i++) {
      if (localMAC[i] != 0x00) {
        allZero = false;
        break;
      }
    }
    if (!allZero) break;
    delay(50);
  }

  esp_now_init();
  esp_now_register_send_cb(onDataSent);
}

void Wifi_addPeer(uint8_t index) {
  memcpy(peerInfo.peer_addr, MAC_address[index], 6);
  peerInfo.channel = _channel;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.printf("Failed to add peer %d\n", index);
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Failed to send msg");
  }
}

void receiveI2C(int numBytes) {
  if (Wire.available() >= 8) {
    Wire.read();  // discard dummy register byte from Raspberry Pi

    uint8_t buffer[8];
    Wire.readBytes(buffer, 8);  // [cmd, targetID, d0, d1, ..., d5]

    dataIn.cmd = buffer[0];
    dataIn.targetID = buffer[1];
    memcpy(dataIn.data, &buffer[2], 6);

    forwardCommandViaESPNOW();
  }
}

void forwardCommandViaESPNOW() {
  uint8_t targetID = dataIn.targetID;
  Serial.printf("Forwarding cmd 0x%02X to target ID: %d\n", dataIn.cmd, targetID);

  if (targetID == 0xFF) {
    for (int i = 0; i < 4; i++) {
      esp_now_send(MAC_address[i], (uint8_t*)&dataIn, sizeof(targetCommands));
    }
  } else if (targetID < 4) {
    esp_now_send(MAC_address[targetID], (uint8_t*)&dataIn, sizeof(targetCommands));
  } else {
    Serial.println("Invalid target ID");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS, 1, 2, 400000);
  Wire.onReceive(receiveI2C);

  Wifi_begin();

  // Add all 4 Simas as peers
  for (int i = 0; i < 4; i++) {
    Wifi_addPeer(i);
  }
}

void loop() {
  delay(100);
}
