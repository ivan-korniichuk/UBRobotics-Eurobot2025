#pragma once
#include "Hardware.h"

namespace robot {
struct LidarResult {
  float range;  // mean distance in mm
  float angle;  // degrees
  bool valid;   // true if range ≤ 300 mm
};

void crawlToRelative(float dx, float dy, float dTh, unsigned long durationMs);

/* ───── robot-level state machine (unchanged) ───── */
enum State : uint8_t { DIRECT_CONTROL = 1,
                       SEEKING,
                       CLEARING,
                       ALIGNING,
                       CLOSING,
                       COARSE_STRAFE,
                       PRE_STRAFE,
                       APPROACH,
                       ABORTING };
extern State state;

/* ───── crawl-mode flags ───── */
extern bool crawlActive;
extern bool crawlFinished;
extern bool overtimeWatchdog;

/* ───── mechanical constants ───── */
namespace cfg {
constexpr float wheelRadius_mm = 29.0f;
constexpr float robotRadius_mm = 120.0f;
constexpr float posTol_deg = 3.0f;
constexpr float pickupRange = 100.0f;
}

/* ───── RC packet ───── */
struct TargetCommands {
  float roll = 0, pitch = 0, yaw = 0, throttle = 0;
};

/* ───── velocity helpers ───── */
void drive(float x, float y, float turn);
void drive(const TargetCommands&);

/* ───── crawl helpers ───── */
bool crawlUpdate();
void crawl(float vx, float vy, float vTurn);

/* ───── diagnostic helper ───── */
State evaluateStateGoals(float l, float r, float e, int8_t leftRight);
LidarResult measureObject(uint8_t kSamples = 10);
}  // namespace robot
