#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include "HX711.h"

// ===================== Pin map (ESP32) =====================
// HX711
#define DOUT 34
#define CLK  25

// DRV8833
#define AIN1 26
#define AIN2 27

// Encoder
#define ENCODER_A 32
#define ENCODER_B 33

// ===================== HX711 =====================
HX711 scale;
float calibration_factor = 7180;  // g
const float HX_RATE_HZ = 80.0f;

// ===================== Encoder → v_line =====================
const float COUNTS_PER_METER = 11357.0f;
volatile long encoderCount = 0;    // ISR writes
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

long lastCount = 0;
uint32_t lastEncMs = 0;
float v_line = 0.0f;
float leash_len_m = 0.0f;

// ===================== Motor / PWM =====================
const int remotorSpeed = 65;   // retract PWM (upper bound)
int pwm_state = 0;

const int PWM_STEP_OUT = 80;
const int PWM_STEP_IN  = 30;

// retracting PWM
const int   BASE_RETRACT_PWM     = 35;
const int   MAX_RETRACT_PWM      = 60;
const int   MIN_RETRACT_PWM      = 40;
const float KP_RETRACT_PWM_PER_G = 3.0f;

// loosening PWM
const int   BASE_PAYOUT_PWM   = 50;
const int   MAX_PAYOUT_PWM    = 110;
const float KP_PWM_PER_G      = 4.0f;

// Burst（instant payout for strong drag）
const int BURST_PWM = 100;
const unsigned long BURST_MS = 50;
unsigned long burst_until_ms = 0;

// Landing impact give-way
const float IMPACT_DTDT_GPS = 90.0f;
const float IMPACT_NEAR_UPPER_G = 0.8f;
const unsigned long IMPACT_COAST_MS = 120;
const int IMPACT_PAYOUT_PWM = 70;
unsigned long impact_until_ms = 0;

// No instanst retracting after burst
const unsigned long POST_BURST_NO_RETRACT_MS = 600;
unsigned long no_retract_until_ms = 0;

// Smooth tension (EMA)
const float T_ALPHA_FAST = 0.5f;
const float T_ALPHA_SLOW = 0.3f;

// threshold
float lower_limit = 16.3f;
float upper_limit = 37.5f;

float V_OUT_MIN = 0.004f;
float T_MARGIN_G = 0.7f;
unsigned long ALLOW_PAYOUT_CONFIRM_MS = 5;
bool allow_payout_latch = false;
unsigned long allow_payout_since_ms = 0;

// locked fast path
const float V_EPS_LOCKED   = 0.003f;
const float T_LOCK_MARGING = 3.0f;
const float DTDT_LOCK_GPS  = 55.0f;
const unsigned long LOCK_CONFIRM_MS = 12;
bool locked_latch = false;
unsigned long locked_since_ms = 0;

// dT/dt smoothing
const float DTDTA = 0.25f;

// Active-Coast near upper limit
const int ASSIST_PWM = 80;

// Anti-chatter / Grace
bool inRetract = false;
unsigned long retract_start_ms = 0;
long          retract_start_cnt = 0;

const unsigned long MIN_RETRACT_MS  = 50;
const long          MIN_RETRACT_CNT = 10;

const unsigned long LOOSEN_GRACE_MS = 900;
unsigned long loosen_grace_until_ms = 0;

const float LOWER_HYST_G = 0.8f;

// ===================== ESP32 LEDC PWM =====================
static const int PWM_FREQ = 20000;   // 20 kHz
static const int PWM_RES  = 8;       // 8-bit 0..255
static const int CH_AIN1  = 0;
static const int CH_AIN2  = 1;

// ===================== FreeRTOS shared (HX711 -> Control/Log) =====================
struct TensionMsg {
  uint32_t ms;
  float raw;
  float T_fast;
  float T_slow;
  float dTdt_gps;
};

static QueueHandle_t qTension = nullptr;
static TaskHandle_t hxTaskHandle = nullptr;
static TaskHandle_t ctrlTaskHandle = nullptr;
static TaskHandle_t logTaskHandle = nullptr;

// ===================== helpers =====================
inline void motorCoast() {
  ledcWrite(CH_AIN1, 0);
  ledcWrite(CH_AIN2, 0);
}

void driveMotor(int pwmSigned) {
  if (pwmSigned == 0) { motorCoast(); return; }

  if (pwmSigned > 0) { // retract
    int pwm = constrain(pwmSigned, 0, 255);
    ledcWrite(CH_AIN1, 0);
    ledcWrite(CH_AIN2, pwm);
  } else { // payout
    int pwm = constrain(-pwmSigned, 0, 255);
    ledcWrite(CH_AIN1, pwm);
    ledcWrite(CH_AIN2, 0);
  }
}

int slewPWM(int target) {
  if (target == pwm_state) return pwm_state;

  if ((target > 0 && pwm_state >= 0) || (target < 0 && pwm_state <= 0)) {
    int step = (target > 0) ? PWM_STEP_IN : PWM_STEP_OUT;
    if (target > pwm_state) pwm_state = min(pwm_state + step, target);
    else                    pwm_state = max(pwm_state - step, target);
  } else {
    int step = (pwm_state > 0) ? PWM_STEP_IN : PWM_STEP_OUT;
    if (pwm_state > 0) pwm_state = max(0, pwm_state - step);
    else               pwm_state = min(0, pwm_state + step);
  }
  return pwm_state;
}

int map_constrain_float(float x, float in_min, float in_max, int out_min, int out_max) {
  if (in_max - in_min == 0) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  float y = out_min + t * (out_max - out_min);
  return (int)y;
}

int applyDeadzone(int pwmSigned) {
  const int DEADZONE = 70;
  if (pwmSigned == 0) return 0;
  if (pwmSigned > 0) return max(pwmSigned, DEADZONE);
  return min(pwmSigned, -DEADZONE);
}

// ===================== ISR =====================
void IRAM_ATTR encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  portENTER_CRITICAL_ISR(&encMux);
  if (a == b) encoderCount++;
  else        encoderCount--;
  portEXIT_CRITICAL_ISR(&encMux);
}

// ===================== Task: HX711 80Hz =====================
void taskHX711(void *pv) {
  const TickType_t period = pdMS_TO_TICKS(1000 / 80); // ~12 ms
  TickType_t lastWake = xTaskGetTickCount();

  float T_fast_g = 0.0f;
  float T_slow_g = 0.0f;

  float dTdt_gps = 0.0f;
  float T_prev_for_dt = 0.0f;
  bool have_prev_T = false;
  uint32_t last_hx_ms = millis();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    if (!scale.is_ready()) continue;

    float raw = scale.get_units(1);

    // EMA
    T_fast_g = T_fast_g + T_ALPHA_FAST * (raw - T_fast_g);
    T_slow_g = T_slow_g + T_ALPHA_SLOW * (raw - T_slow_g);

    uint32_t now = millis();
    float dtT_s = max(0.001f, (now - last_hx_ms) * 1e-3f);

    float dTdt_raw = 0.0f;
    if (!have_prev_T) { dTdt_raw = 0.0f; have_prev_T = true; }
    else dTdt_raw = (T_fast_g - T_prev_for_dt) / dtT_s;

    T_prev_for_dt = T_fast_g;
    last_hx_ms = now;

    dTdt_gps = DTDTA * dTdt_raw + (1.0f - DTDTA) * dTdt_gps;

    TensionMsg msg;
    msg.ms = now;
    msg.raw = raw;
    msg.T_fast = T_fast_g;
    msg.T_slow = T_slow_g;
    msg.dTdt_gps = dTdt_gps;

    xQueueOverwrite(qTension, &msg);
  }
}

// ===================== Task: Control loop =====================
void taskControl(void *pv) {
  const TickType_t period = pdMS_TO_TICKS(1); // 1 ms
  TickType_t lastWake = xTaskGetTickCount();

  // cached tension (if queue has no update, keep last)
  TensionMsg t = {0, 0, 0, 0, 0};

  lastEncMs = millis();

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    // 1) latest tension
    TensionMsg newT;
    if (xQueueReceive(qTension, &newT, 0) == pdTRUE) t = newT;

    uint32_t now = millis();

    // 2) encoder -> v_line
    uint32_t dt_ms = now - lastEncMs;
    if (dt_ms == 0) dt_ms = 1;
    lastEncMs = now;

    long cnt;
    portENTER_CRITICAL(&encMux);
    cnt = encoderCount;
    portEXIT_CRITICAL(&encMux);

    long dcount = cnt - lastCount;
    lastCount = cnt;

    float dt = dt_ms * 1e-3f;
    v_line = (COUNTS_PER_METER > 1e-6f) ? ((float)dcount / COUNTS_PER_METER) / dt : 0.0f;
    leash_len_m += v_line * dt;

    // 3) your control logic (ported)
    float raw    = t.raw;
    float T_fast = t.T_fast;
    float T_slow = t.T_slow;
    float dTdt_gps = t.dTdt_gps;

    // ---- instant payout decision ----
    bool sudden_pull = ((raw > upper_limit + 15.0f) || (T_fast > upper_limit + 15.0f)) && (dTdt_gps > 70.0f);
    bool not_retracting = (v_line <= 0.0f);
    bool instant_payout = sudden_pull && not_retracting;

    if (instant_payout) {
      unsigned long new_until = now + BURST_MS;
      if (new_until > burst_until_ms) burst_until_ms = new_until;

      loosen_grace_until_ms = max(loosen_grace_until_ms, now + LOOSEN_GRACE_MS);
      no_retract_until_ms   = max(no_retract_until_ms,   now + POST_BURST_NO_RETRACT_MS);

      inRetract = false;

      allow_payout_latch = true;
      allow_payout_since_ms = now;
    }

    // ---- burst window ----
    if (now < burst_until_ms) {
      int steppedPWM = slewPWM(-BURST_PWM);
      steppedPWM = applyDeadzone(steppedPWM);
      driveMotor(steppedPWM);
      continue;
    }

    // ---- impact detect ----
    bool impact_detect = inRetract && ((dTdt_gps > IMPACT_DTDT_GPS) || (T_fast > upper_limit - IMPACT_NEAR_UPPER_G));
    if (impact_detect) {
      unsigned long new_until = now + IMPACT_COAST_MS;
      if (new_until > impact_until_ms) impact_until_ms = new_until;
    }

    if (now < impact_until_ms) {
      int targetPWM = 0;
      if (T_fast > upper_limit - 0.3f) {
        targetPWM = -IMPACT_PAYOUT_PWM;
      }
      int steppedPWM = slewPWM(targetPWM);
      steppedPWM = applyDeadzone(steppedPWM);
      driveMotor(steppedPWM);
      inRetract = false;
      continue;
    }

    // ---- allow payout latches ----
    bool line_out = (v_line < -V_OUT_MIN);
    (void)line_out; // kept for future use

    bool high_T = (T_fast > (upper_limit + T_MARGIN_G));
    if (high_T) {
      if (!allow_payout_latch) { allow_payout_latch = true; allow_payout_since_ms = now; }
    } else {
      allow_payout_latch = false;
    }
    bool allow_payout_dir = allow_payout_latch && (now - allow_payout_since_ms >= ALLOW_PAYOUT_CONFIRM_MS);

    bool spool_static = (fabs(v_line) < 0.003f);
    bool locked_trig  = spool_static && (T_fast > upper_limit + T_LOCK_MARGING) && (dTdt_gps > DTDT_LOCK_GPS);
    if (locked_trig) {
      if (!locked_latch) { locked_latch = true; locked_since_ms = now; }
    } else {
      locked_latch = false;
    }
    bool allow_payout_locked = locked_latch && (now - locked_since_ms >= LOCK_CONFIRM_MS);

    bool allow_payout = allow_payout_dir || allow_payout_locked;

    // ---- main targetPWM decision ----
    int targetPWM = 0;

    if (T_fast > upper_limit) {
      if (allow_payout) {
        float over_g = T_fast - upper_limit;
        int pwm = (int)(BASE_PAYOUT_PWM + KP_PWM_PER_G * over_g);
        pwm = constrain(pwm, BASE_PAYOUT_PWM, MAX_PAYOUT_PWM);
        targetPWM = -pwm;

        loosen_grace_until_ms = now + LOOSEN_GRACE_MS;
        no_retract_until_ms   = max(no_retract_until_ms, now + POST_BURST_NO_RETRACT_MS);

        inRetract = false;
      } else {
        targetPWM = 0;
      }
    } else if (T_slow < (lower_limit - LOWER_HYST_G)
               && now >= loosen_grace_until_ms
               && now >= no_retract_until_ms) {

      if (!inRetract) {
        inRetract = true;
        retract_start_ms = now;
        retract_start_cnt = cnt;
      }

      float under_g = (lower_limit - T_slow);
      int pwm = (int)(BASE_RETRACT_PWM + KP_RETRACT_PWM_PER_G * under_g);
      pwm = constrain(pwm, MIN_RETRACT_PWM, MAX_RETRACT_PWM);
      targetPWM = +pwm;

    } else if (T_fast < lower_limit) {
      targetPWM = 0;
    } else {
      if (inRetract) {
        if (T_fast > upper_limit - 0.8f) {
          targetPWM = 0;
          inRetract = false;
        } else {
          bool time_ok = (now - retract_start_ms) >= MIN_RETRACT_MS;
          bool cnt_ok  = (labs(cnt - retract_start_cnt) >= MIN_RETRACT_CNT);
          bool window_ok = (now >= loosen_grace_until_ms) && (now >= no_retract_until_ms);

          if (time_ok && cnt_ok && window_ok) {
            targetPWM = 0;
            inRetract = false;
          } else {
            int cap = map_constrain_float(T_fast,
                                          lower_limit - 1.0f,
                                          upper_limit - 0.5f,
                                          remotorSpeed,
                                          10);
            cap = constrain(cap, 10, remotorSpeed);
            targetPWM = +cap;
          }

          if (!window_ok) {
            targetPWM = 0;
            inRetract = false;
          }
        }
      } else {
        targetPWM = 0;
      }
    }

    // ---- assist payout near upper, spool static ----
    bool nearly_upper = (T_fast >= (upper_limit - 0.3f));
    if (targetPWM == 0 && nearly_upper && spool_static) {
      targetPWM = -ASSIST_PWM;
    }

    // ---- no retract windows ----
    if (now < loosen_grace_until_ms || now < no_retract_until_ms) {
      if (targetPWM > 0) targetPWM = 0;
    }

    // ---- apply slew + deadzone ----
    int steppedPWM = slewPWM(targetPWM);
    steppedPWM = applyDeadzone(steppedPWM);
    driveMotor(steppedPWM);
  }
}

// ===================== Task: Log 10Hz =====================
void taskLog(void *pv) {
  const TickType_t period = pdMS_TO_TICKS(100); // 10 Hz
  TickType_t lastWake = xTaskGetTickCount();

  TensionMsg t = {0, 0, 0, 0, 0};

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    TensionMsg newT;
    if (xQueueReceive(qTension, &newT, 0) == pdTRUE) t = newT;

    Serial.print("T_fast(g)="); Serial.print(t.T_fast, 2);
    Serial.print("  T_slow(g)="); Serial.print(t.T_slow, 2);
    Serial.print("  dTdt="); Serial.print(t.dTdt_gps, 2);
    Serial.print("  v_line(m/s)="); Serial.print(v_line, 3);
    Serial.print("  PWM="); Serial.print(pwm_state);
    Serial.print("  inRetract="); Serial.print(inRetract ? 1 : 0);
    Serial.println();
  }
}

// ===================== setup / loop =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  // HX711 init
  scale.begin(DOUT, CLK);
  delay(1500);
  if (!scale.is_ready()) {
    Serial.println("HX711 not found.");
    while (1) delay(200);
  }
  scale.set_scale(calibration_factor);
  scale.tare();

  // Encoder init
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // LEDC init (motor PWM)
  ledcSetup(CH_AIN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_AIN2, PWM_FREQ, PWM_RES);
  ledcAttachPin(AIN1, CH_AIN1);
  ledcAttachPin(AIN2, CH_AIN2);
  motorCoast();

  // Queue: length 1 (keep latest)
  qTension = xQueueCreate(1, sizeof(TensionMsg));
  if (!qTension) {
    Serial.println("Queue create failed.");
    while (1) delay(200);
  }

  // Start tasks
  // 建議：Control pin core 1，HX711/Log core 0
  xTaskCreatePinnedToCore(taskHX711,   "hx711",  4096, nullptr, 2, &hxTaskHandle,   0);
  xTaskCreatePinnedToCore(taskControl, "ctrl",   6144, nullptr, 3, &ctrlTaskHandle, 1);
  xTaskCreatePinnedToCore(taskLog,     "log",    4096, nullptr, 1, &logTaskHandle,  0);

  Serial.println("ESP32 FreeRTOS version started.");
}

void loop() {
  // All work is in tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
