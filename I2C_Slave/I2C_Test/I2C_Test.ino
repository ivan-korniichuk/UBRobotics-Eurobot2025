#include <Wire.h>

#define I2C_SDA 26
#define I2C_SCL 25
#define I2C_DEV_ADDR 0x40
#define I2C_FREQ 100000

TwoWire I2C = TwoWire(0);

const int MAX_BUF_SIZE = 32;
uint8_t buffer[MAX_BUF_SIZE];
uint8_t receivedLen = 0;

// Optional: for debugging command ID
const char* getCommandName(uint8_t cmd) {
  switch (cmd) {
    case 1: return "MOVE";
    // case 2: return "ROTATE";
    // Extend this list as needed
    default: return "UNKNOWN";
  }
}

void onReceive(int len) {
  if (len > MAX_BUF_SIZE) len = MAX_BUF_SIZE;
  I2C.readBytes(buffer, len);

  Serial.printf("[I2C] Received %d bytes:\n", len);
  for (int i = 0; i < len; i++) Serial.printf(" 0x%02X", buffer[i]);
  Serial.println();

  if (len >= 4) {
    uint8_t cmd = buffer[1];
    uint8_t speed = buffer[2];
    int8_t direction = (int8_t)buffer[3];

    Serial.printf("[I2C] CMD_ID: %d (%s)\n", cmd, getCommandName(cmd));
    Serial.printf("[I2C] MOVE | Speed: %u | Direction: %d\n", speed, direction);
  }
}

void onRequest() {
  // If needed, respond with a status or sensor data
  I2C.print("ACK\n");
  Serial.println("[I2C] onRequest callback triggered.");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  I2C.begin((uint8_t)I2C_DEV_ADDR, I2C_SDA, I2C_SCL, I2C_FREQ);
  I2C.onReceive(onReceive);
  I2C.onRequest(onRequest);

  Serial.printf("[I2C] ESP32 slave ready at address 0x%02X\n", I2C_DEV_ADDR);
}

void loop() {
  delay(10);  // Can be increased to reduce CPU use
}
