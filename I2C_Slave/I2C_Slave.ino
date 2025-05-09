#include "Wire.h"

#define I2C_SDA 26
#define I2C_SCL 25
#define I2C_DEV_ADDR 0x40
#define I2C_FREQ 100000

TwoWire I2C = TwoWire(0);

uint32_t i = 0;
uint8_t buffer[32]; //please dont exceed this
uint8_t comma_ptr = 0;
uint8_t length = 0;

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
 
  // for (uint8_t i = 0; i < len; i++){
  //   if (buffer[i] == ','){
  //     comma_ptr = i;
  //     break;
  //   }
  // }
  //Serial.printf("%s\n", buffer + comma_ptr);
  //Serial.printf("%.*s\n", comma_ptr, buffer);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(400);
  I2C.onReceive(onReceive);
  I2C.onRequest(onRequest);
  I2C.begin((uint8_t)I2C_DEV_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
}

void loop() {
  // put your main code here, to run repeatedly:
  // meow
  delay(10);
}
