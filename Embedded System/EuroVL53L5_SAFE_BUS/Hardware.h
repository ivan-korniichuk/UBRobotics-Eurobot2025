#pragma once
/*
 *  Hardware.h  —  drivers + board objects
 *  --------------------------------------
 *  • AS5600         – magnetic encoder
 *  • MotorController– nested position→speed PID
 *  • VL53L0X        – classic API (ensurePins)
 *
 *  Public helpers:
 *      bool hw::begin();         // call once in setup()
 *      void hw::updateMotors();  // call each loop()
 */

#include <Arduino.h>
#include <Wire.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

namespace hw {

// ───────────── global SDA tracker + safe-switch helper ─────────────
/* 0xFF = “no SDA configured yet” */
inline uint8_t currentSDA = 0xFF;

/* 21 is hard-wired SCL; safely switch the bus to `newSDA`
 * – puts the previous pad in genuine high-Z
 * – skips all work if the bus is already on that SDA line
 */
inline void ensureI2CPins(uint8_t newSDA) {
  if (newSDA == currentSDA) return;  // already correct

  /* 1. High-Z the old SDA (if one was ever set) */
  if (currentSDA != 0xFF) {
    gpio_reset_pin((gpio_num_t)currentSDA);                         // disconnect from matrix & clear pulls
    gpio_set_direction((gpio_num_t)currentSDA, GPIO_MODE_DISABLE);  // disable buffers
  }

  /* 2. Route I²C0 to the new pins, keep both pull-ups enabled */
  i2c_set_pin((i2c_port_t)0,
              (gpio_num_t)newSDA, GPIO_NUM_21,
              GPIO_PULLUP_ENABLE, GPIO_PULLUP_ENABLE,
              I2C_MODE_MASTER);

  currentSDA = newSDA;
}

// ────────────────────── AS5600 ──────────────────────
class AS5600 {
public:
  static constexpr uint8_t I2C_ADDR = 0x36;

  AS5600(TwoWire& w = Wire, uint8_t sda = 10, uint8_t scl = 21)
    : _wire(w), _sda(sda), _scl(scl) {}

  bool begin(uint32_t f = 400000) {
    ensurePins();  // guarantee bus correct
    _wire.begin(_sda, _scl, f);
    _wire.beginTransmission(I2C_ADDR);
    _wire.write(0x0C);
    return _wire.endTransmission() == 0;
  }

  uint16_t raw() const {
    ensurePins();
    _wire.beginTransmission(I2C_ADDR);
    _wire.write(0x0E);
    if (_wire.endTransmission(false) != 0) return 0;
    if (_wire.requestFrom(I2C_ADDR, (uint8_t)2) != 2) return 0;
    return (_wire.read() << 8 | _wire.read()) & 0x0FFF;
  }

  float deg() const {
    ensurePins();
    return raw() * 360.0f / 4096.0f;
  }

  /* Public helper to force-sync the bus (rarely needed externally) */
  void ensurePins() const {
    ensureI2CPins(_sda);
  }

private:
  TwoWire& _wire;
  uint8_t _sda, _scl;
};

// ────────────────── MotorController ──────────────────
class MotorController {
public:
  float monitor;
  enum Mode { SPEED,
              POSITION };

  MotorController(uint8_t sdaPin, uint8_t fwdCh, uint8_t revCh,
                  float kp = 1.5f, float ki = 25.0f, float kd = 0.0f)
    : enc(Wire, sdaPin), _sda(sdaPin), _fwd(fwdCh), _rev(revCh),
      kP(kp), kI(ki), kD(kd) {}

  bool begin(uint32_t i2cF = 400000,
             uint32_t pwmF = 20000,
             uint8_t res = 10) {
    if (!enc.begin(i2cF)) {
      Serial.printf("AS5600 fail on SDA %u\n", _sda);
      return false;
    }
    enc.ensurePins();  // bus is now correct
    lastRaw = enc.deg();
    lastT = micros();

    ledcAttach(_fwd, pwmF, res);
    ledcAttach(_rev, pwmF, res);
    maxDuty = (1 << res) - 1;
    ledcWrite(_fwd, maxDuty);
    ledcWrite(_rev, maxDuty);

    totRaw = relDeg();
    return true;
  }

  /* ---------- speed interface ---------- */
  void setTargetSpeed(float dps) {
    enc.ensurePins();
    tSpeed = dps;
    mode = SPEED;
  }

  /* ---------- position interface ---------- */
  void enablePosition(float p = 5, float i = 0.01, float d = 0.0) {
    enc.ensurePins();
    mode = POSITION;
    p_kP = p;
    p_kI = i;
    p_kD = d;
    p_int = 0; 
    intg = 0;
    p_lerr = 0;
  }
  void zeroHere() {
    enc.ensurePins();
    zero = totRaw;
  }
  void setPosTarget(float deg) {
    enc.ensurePins();
    tPos = deg;
  }

  float getPosTarget() {
    return tPos;
  }

  bool reached(float tol = 2) const {
    enc.ensurePins();
    return wrap(fabsf(relDeg() - tPos)) < tol;
  }
  float relDeg() const {
    enc.ensurePins();
    return wrap(enc.deg() - zero);
  }

  float relDegC() const {
    enc.ensurePins();
    return rawRel();
  }

  Mode currentMode() const {
    enc.ensurePins();
    return mode;
  }

  float getSpeed() {
    enc.ensurePins();
    return fSpeed;
  }

  void update() {
    enc.ensurePins();
    unsigned long now = micros();
    float dt = (now - lastT) * 1e-6f;
    if (dt <= 0) return;
    // 1) read & unwrap the raw encoder angle
    float raw = enc.deg();
    float dRaw = wrap(raw - lastRaw);
    totRaw += dRaw;
    //Serial.println(dRaw);
    float rel = wrap(raw - zero);
    fSpeed = alpha * fSpeed + (1 - alpha) * (dRaw / dt);

    // 2) outer position loop runs only every POS_LOOP_INTERVAL updates
    float demand;
    if (mode == POSITION) {
      // increment & check counter
      if (++posLoopCounter >= POS_LOOP_INTERVAL) {
        posLoopCounter = 0;

        // wrap-aware shortest-path error
        monitor = rel;
        float posErr = wrap(tPos - rel);
        //Serial.print(zero);
        //Serial.print(" ");
        //Serial.print(rel);
        //Serial.print(" ");

        // PI(D) on position
        float posDer = (posErr - p_lerr) / dt;

        lastDemand = p_kP * posErr + p_kD * posDer;

        if (fabs(lastDemand) < 800) {
          p_int += p_kI * posErr*dt*300; //Todo- unbodge
          p_int = constrain(p_int, -imax, imax);
        }

        lastDemand += p_int;

        p_lerr = posErr;
      }
      demand = lastDemand;

    } else {
      demand = tSpeed;
    }

    // 3) inner speed loop (unchanged)
    float spdErr = demand - fSpeed;
    intg += kI * spdErr * dt;  //TODO- unbodge
    intg = constrain(intg, -imax, imax);
    float spdDer = (spdErr - lerr) / dt;
    float u = -(kP * spdErr + intg + kD * spdDer);

    Serial.printf("%f,%f,%f,%f,", kP * spdErr, intg, demand, fSpeed);
    

    float duty = fabsf(u);
    duty = constrain(duty, 0.f, (float)maxDuty);
    if (fabsf(fSpeed) < 2 && fabs(demand) > 0.1)
      duty = constrain(duty + 50, 0.f, (float)maxDuty);

    if (u < -1) {  // forward
      ledcWrite(_fwd, maxDuty - duty);
      ledcWrite(_rev, maxDuty);
    } else if (u > 1) {  // reverse
      ledcWrite(_fwd, maxDuty);
      ledcWrite(_rev, maxDuty - duty);
    } else {  // stop
      ledcWrite(_fwd, maxDuty);
      ledcWrite(_rev, maxDuty);
    }

    // 4) housekeeping
    lastRaw = raw;
    lerr = spdErr;
    lastT = now;
  }

private:
  float rawRel() const {
    return totRaw - zero;
  }

  static float wrap(float d) {
    while (d > 180) d -= 360;
    while (d < -180) d += 360;
    return d;
  }

  /* hardware */
  AS5600 enc;
  uint8_t _sda, _fwd, _rev;
  uint32_t maxDuty = 0;

  static constexpr uint8_t POS_LOOP_INTERVAL = 5;
  uint8_t posLoopCounter = 0;
  float lastDemand = 0;

  /* speed loop tunables */
  float kP, kI, kD;
  static constexpr float alpha = 0.3f;
  static constexpr float imax = 650.f;

  /* speed loop state */
  float fSpeed = 0, intg = 0, lerr = 0;

  /* position loop */
  Mode mode = SPEED;
  float totRaw = 0;
  float zero = 0, tPos = 0;
  float p_kP = 4, p_kI = 0, p_kD = 0, p_int = 0, p_lerr = 0;

  /* misc state */
  float tSpeed = 0, lastRaw = 0;
  unsigned long lastT = 0;
};

// ───────────────────── VL53L0X ─────────────────────
class VL53L0 {
public:
  VL53L0(TwoWire& bus = Wire, uint8_t addr = 0x29)
    : _wire(bus), _addr(addr) {}

  bool begin(uint8_t sda = SDA, uint8_t scl = SCL) {
    _wire.end();
    sdaPin = sda;
    sclPin = scl;
    ensurePins();  // guarantee bus correct
    _wire.begin(sdaPin, sclPin, 400000);

    if (read8(IDENTIFICATION_MODEL_ID) != 0xEE) return false;

    /* ST reference initialisation (abridged) */
    write8(0x88, 0x00);
    write8(0x80, 0x01);
    write8(0xFF, 0x01);
    write8(0x00, 0x00);
    (void)read8(0x91);
    write8(0x00, 0x01);
    write8(0xFF, 0x00);
    write8(0x80, 0x00);

    write8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write8(GPIO_HV_MUX_ACTIVE_HIGH,
           read8(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    clearIRQ();

    singleRefCal(0x40);
    singleRefCal(0x00);

    startContinuous();  // back-to-back ranging
    return true;
  }

  /* Change bus pins (replaces old setPins) */
  void ensurePins(uint8_t sda, uint8_t scl) {
    sdaPin = sda;
    sclPin = scl;
    ensureI2CPins(sdaPin);
  }

  /* Quick helper (no args) */
  void ensurePins() const {
    ensureI2CPins(sdaPin);
  }

  /* user API */
  bool ready() {
    ensurePins();
    return read8(RESULT_INTERRUPT_STATUS) & 0x07;
  }

  uint16_t readRangeNoWait() {
    ensurePins();
    uint16_t mm = read16(RESULT_RANGE_STATUS + 10);
    clearIRQ();
    return mm;
  }

  void startContinuous() {
    ensurePins();
    write8(SYSRANGE_START, 0x02);
  }
  void stop() {
    ensurePins();
    write8(SYSRANGE_START, 0x01);
  }

private:
  /* low-level helpers (internal, assume bus already correct) */
  void write8(uint8_t reg, uint8_t val) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(val);
    _wire.endTransmission();
  }
  uint8_t read8(uint8_t reg) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.endTransmission(false);
    _wire.requestFrom(_addr, (uint8_t)1);
    return _wire.read();
  }
  uint16_t read16(uint8_t reg) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.endTransmission(false);
    _wire.requestFrom(_addr, (uint8_t)2);
    return (uint16_t)_wire.read() << 8 | _wire.read();
  }
  void clearIRQ() {
    write8(SYSTEM_INTERRUPT_CLEAR, 0x01);
  }

  void singleRefCal(uint8_t vhv_init_byte) {
    write8(SYSRANGE_START, 0x01);
    write8(SYSTEM_SEQUENCE_CONFIG, vhv_init_byte);
    while ((read8(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
      ;
    clearIRQ();
    write8(SYSTEM_SEQUENCE_CONFIG, 0xE8);
  }

  TwoWire& _wire;
  const uint8_t _addr;
  uint8_t sdaPin = SDA, sclPin = SCL;

  /* register aliases */
  static constexpr uint8_t SYSRANGE_START = 0x00;
  static constexpr uint8_t SYSTEM_SEQUENCE_CONFIG = 0x01;
  static constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
  static constexpr uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0B;
  static constexpr uint8_t RESULT_INTERRUPT_STATUS = 0x13;
  static constexpr uint8_t RESULT_RANGE_STATUS = 0x14;
  static constexpr uint8_t GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
  static constexpr uint8_t IDENTIFICATION_MODEL_ID = 0xC0;
};

// ─────────────── pin map & global objects ───────────────
constexpr uint8_t M1_SDA = 40, M1_F = 13, M1_R = 12;
constexpr uint8_t M2_SDA = 39, M2_F = 15, M2_R = 14;
constexpr uint8_t M3_SDA = 38, M3_F = 17, M3_R = 16;

constexpr uint8_t LIDAR_FRONT_SDA = 40;
constexpr uint8_t LIDAR_RIGHT_SDA = 39;
constexpr uint8_t LIDAR_LEFT_SDA = 38;

inline MotorController motor1(M1_SDA, M1_F, M1_R);
inline MotorController motor2(M2_SDA, M2_F, M2_R);
inline MotorController motor3(M3_SDA, M3_F, M3_R);

inline VL53L0 lidarFront;
inline VL53L0 lidarRight;
inline VL53L0 lidarLeft;

inline bool begin() {
  if (!motor1.begin() || !motor2.begin() || !motor3.begin()) return false;
  if (!lidarLeft.begin(LIDAR_LEFT_SDA) || !lidarRight.begin(LIDAR_RIGHT_SDA)) {
    Serial.println(F("VL53L0X init failed"));
    return false;
  }
  return true;
}

inline void updateMotors() {
  motor1.update();
  motor2.update();
  motor3.update();
}

}  // namespace hw
