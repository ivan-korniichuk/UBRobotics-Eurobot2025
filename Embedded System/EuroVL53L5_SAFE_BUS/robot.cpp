#include "robot.h"
#include <cmath>

robot::State robot::state = robot::SEEKING;
bool robot::crawlActive = false;
bool robot::crawlFinished = true;
bool robot::overtimeWatchdog = false;


/* internal helpers */
namespace {

unsigned long crawlStartMs = 0;
unsigned long crawlDurMs = 0;

float priorX, priorY, warpedX, warpedY;

float goalDX = 0, goalDY = 0, goalDTh = 0;  // world-frame goal

inline void wheelsSpeed(float x, float y, float r) {
  hw::motor2.setTargetSpeed(r + -1.f * x);
  hw::motor1.setTargetSpeed(r + 0.5f * x - 0.866f * y);
  hw::motor3.setTargetSpeed(r + 0.5f * x + 0.866f * y);
}
inline float mm2deg(float mm) {
  return mm * 360.f / (2 * M_PI * robot::cfg::wheelRadius_mm);
}

inline float wheelPerDeg() {  // wheel-deg per robot-deg
  const float k = mm2deg(1);
  return k * (M_PI * robot::cfg::robotRadius_mm) / 180.f;
}
}  // namespace

/* ---- speed wrappers ---- */
void robot::drive(float x, float y, float r) {
  wheelsSpeed(x, y, r);
}
void robot::drive(const TargetCommands& c) {
  constexpr float S = 750;
  wheelsSpeed(S * c.yaw, S * c.throttle, S * c.roll);
}

/* ------------------------------------------------------------------
 *  crawlToRelative  —  time-parametrised move
 *  dx,dy in mm  •  dTh in deg  •  duration in ms (≥1)
 * ------------------------------------------------------------------ */
void robot::crawlToRelative(float dx, float dy, float dTh,
                            unsigned long durationMs) {
  if (durationMs == 0) durationMs = 1;

  for (auto* m : { &hw::motor1, &hw::motor2, &hw::motor3 }) {
    m->enablePosition();
    m->zeroHere();  // wheel-angle = 0 at start
  }

  warpedX = 0;
  warpedY = 0;

  goalDX = dx;
  goalDY = dy;
  goalDTh = dTh;

  crawlStartMs = millis();
  crawlDurMs = durationMs;

  crawlActive = true;
  crawlFinished = false;
  overtimeWatchdog = false;
}


/* ------------------------------------------------------------------
 *  crawlUpdate  —  interpolate targets each loop
 * ------------------------------------------------------------------ */
bool robot::crawlUpdate() {
  if (!crawlActive) return true;

  /* ---------- 1. fraction of move elapsed ---------- */
  unsigned long now = millis();
  float f = float(now - crawlStartMs) / float(crawlDurMs);
  if (f > 1.0f) f = 1.0f;

  if (millis() - crawlStartMs > 3 * crawlDurMs) {
    overtimeWatchdog = true;
  }

  /* desired world-frame translation so far */
  float dxW = f * goalDX;
  float dyW = f * goalDY;

  float xStepW = dxW - priorX;
  float yStepW = dyW - priorY;
  priorX = dxW;
  priorY = dyW;
  //printf("dW  X: %.1f  Y: %.1f \n", dxW, dyW);

  /* ---------- 2. scheduled heading (no longer measured) ---------- */
  float headingDeg_sched = f * goalDTh;  // planned θ
  float headingRad_sched = headingDeg_sched * DEG_TO_RAD;

  float c = cosf(headingRad_sched);  // minus θ
  float s = sinf(headingRad_sched);
  float xStepB = c * xStepW - s * yStepW;  // body-frame pos
  float yStepB = s * xStepW + c * yStepW;
  warpedX += xStepB;
  warpedY += yStepB;

  /* ---------- 2a.  body-frame velocity printout ---------- */
  {
    static float prevX = 0.0f, prevY = 0.0f;
    static unsigned long prevT = crawlStartMs;

    float dt = (now - prevT) * 1e-3f;  // ms → s
    if (dt > 0) {
      float vxB = (warpedX - prevX) / dt;  // mm/s
      float vyB = (warpedY - prevY) / dt;  // mm/s
                                           //printf("vBody  X: %.1f mm/s   Y: %.1f mm/s\n", vxB, vyB);
    }

    prevX = warpedX;
    prevY = warpedY;
    prevT = now;
  }

  /* ---------- 3. scheduled rotation component ---------- */
  float wTurn = wheelPerDeg() * headingDeg_sched;

  /* ---------- 4. convert body (x,y,θ) → wheel targets ---------- */
  const float k = mm2deg(1);
  float wX = k * warpedX;
  float wY = k * warpedY;

  float w2 = wTurn + (-wX);
  float w1 = wTurn + (0.5f * wX - 0.866f * wY);
  float w3 = wTurn + (0.5f * wX + 0.866f * wY);

  hw::motor1.setPosTarget(w1);
  hw::motor2.setPosTarget(w2);
  hw::motor3.setPosTarget(w3);

  /* ---------- 5. finish check (unchanged) ---------- */
  float avgWheelDeg =
    (hw::motor1.relDegC() + hw::motor2.relDegC() + hw::motor3.relDegC()) / 3.0f;
  float headingDeg_meas = avgWheelDeg / wheelPerDeg();

  bool stopped = (fabs(hw::motor1.getSpeed())<0.01&&fabs(hw::motor2.getSpeed())<0.01&&fabs(hw::motor3.getSpeed())<0.01);
  bool transDone = (f >= 1.0f);
  bool rotDone = fabsf(headingDeg_sched - headingDeg_meas) <= cfg::posTol_deg;
  bool wheelsOn =
    hw::motor1.reached(cfg::posTol_deg) && hw::motor2.reached(cfg::posTol_deg) && hw::motor3.reached(cfg::posTol_deg);

  if (transDone && rotDone && wheelsOn && stopped) {
    crawlActive = false;
    crawlFinished = true;

    for (auto* m : { &hw::motor1, &hw::motor2, &hw::motor3 }) {
      float hold = m->relDeg();
      m->enablePosition();
      m->setPosTarget(hold);
    }
    return true;
  }
  return crawlFinished;
}





/* ---- crawl (continuous position drive) ---- */
void robot::crawl(float vx, float vy, float vTurn) {
  static unsigned long last = millis();
  unsigned long now = millis();
  float dt = (now - last) * 1e-3f;
  last = now;

  if (!crawlActive) {
    for (auto* m : { &hw::motor1, &hw::motor2, &hw::motor3 }) {
      m->enablePosition();
      m->zeroHere();
    }
    crawlActive = true;
    crawlFinished = false;
  }

  float k = mm2deg(1), R = cfg::robotRadius_mm;
  float turn_mm = (M_PI * R / 180.f) * vTurn;
  float wT = k * turn_mm * dt, xd = k * vx * dt, yd = k * vy * dt;

  hw::motor2.setPosTarget(hw::motor2.relDeg() + wT + (-xd));
  hw::motor1.setPosTarget(hw::motor1.relDeg() + wT + (0.5f * xd - 0.866f * yd));
  hw::motor3.setPosTarget(hw::motor3.relDeg() + wT + (0.5f * xd + 0.866f * yd));
}

/* ---- evaluateStateGoals (simple demo logic) ---- */
robot::State robot::evaluateStateGoals(float l, float r, float e, int8_t lr) {
  /* placeholder thresholds – adjust for your robot */
  float diff = l - r, avg = 0.5f * (l + r);
  bool aligned = fabsf(diff) < 3, centred = avg < 200;
  if (aligned && centred) return robot::ALIGNING;  // etc.
  return robot::SEEKING;
}

robot::LidarResult robot::measureObject(uint8_t kSamples) {
  float spacing_mm = 166;
  float sumL = 0.0f, sumR = 0.0f;

  for (uint8_t i = 0; i < kSamples; ++i) {
    // wait for both sensors to be ready
    while (!(hw::lidarLeft.ready() && hw::lidarRight.ready())) {
      delay(1);
    }

    // LEFT
    sumL += static_cast<float>(hw::lidarLeft.readRangeNoWait());

    // RIGHT
    sumR += static_cast<float>(hw::lidarRight.readRangeNoWait());

    delay(20);  // optional bus pacing
  }

  float avgL = sumL / kSamples;
  float avgR = sumR / kSamples;
  float meanRange = (avgL + avgR) * 0.5f;
  float angleDeg = (RAD_TO_DEG * atan2(spacing_mm, avgL - avgR))-90;
  bool isValid = (avgL <= 300.0f && avgR <= 300.0f);

  return { meanRange, angleDeg, isValid };
}
