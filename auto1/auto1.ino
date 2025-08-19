#include "HX711.h"

// ---------------- Pins ----------------
#define DOUT 4
#define CLK  5

#define AIN1 6     // DRV8833 IN1 (PWM)
#define AIN2 9     // DRV8833 IN2 (PWM)

// (可選) nSLEEP：
// #define HAVE_NSLEEP 1
// #define PIN_NSLEEP 7

// Encoder pins
#define ENCODER_A 2
#define ENCODER_B 3

// ---------------- HX711 ----------------
HX711 scale;
float calibration_factor = 4180;  // g

// ---------------- Encoder → v_line ----------------
const float COUNTS_PER_METER = 11357.0f;   // TODO: 
volatile long encoderCount = 0;
long lastCount = 0;
unsigned long lastVtMs = 0;
float v_line = 0.0f;         // m/s（放線 < 0，收線 > 0）
float leash_len_m = 0.0f;

// ---------------- Motor / PWM ----------------
const int remotorSpeed = 150;  // retracting speed(PWM)
int pwm_state = 0;
const int PWM_STEP_OUT = 25;   // loosen ramp（prevent over loosening）
const int PWM_STEP_IN  = 4;    // retract ramp

// 放線 PWM（比例律，越拉越快：基礎+比例）
const int   BASE_PAYOUT_PWM   = 180;   // ★ 原 90 -> 70
const int   MAX_PAYOUT_PWM    = 255;
const float KP_PWM_PER_G      = 20.0f; // ★ 原 10 -> 8

// ---------------- EMG（if sudden strong drag bursts） ----------------
const int threshold_emg = 50;         // g
bool emg_boost = false;
unsigned long emg_until_ms = 0;
const unsigned long EMG_BOOST_MS = 70;

const int BURST_PWM = 240;
const unsigned long BURST_MS = 100;
unsigned long burst_until_ms = 0;

// ---------------- tension smoothening  ----------------
float T_fast_g = 0.0f;                // 快速 EMA（進入放線用）
float T_slow_g = 0.0f;                // 慢速 EMA（退出判斷用）
const float T_ALPHA_FAST = 0.40f;     // ★ 快
const float T_ALPHA_SLOW = 0.19f;     // ★ 慢

// ---------------- 門檻/許可 ----------------
float lower_limit = 17.8;
float upper_limit = 22.8f;

float V_OUT_MIN = 0.002f;
float T_MARGIN_G = 0.7f;
unsigned long ALLOW_PAYOUT_CONFIRM_MS = 2;
bool allow_payout_latch = false;
unsigned long allow_payout_since_ms = 0;

// 鎖軸快速通道
const float V_EPS_LOCKED   = 0.003f;
const float T_LOCK_MARGING = 3.0f;
const float DTDT_LOCK_GPS  = 55.0f;
const unsigned long LOCK_CONFIRM_MS = 12;
bool locked_latch = false;
unsigned long locked_since_ms = 0;
float dTdt_gps = 0.0f;
float T_prev_for_dt = 0.0f;
unsigned long last_T_ms = 0;

// Active-Coast：停止時小外放助力（只在靠近上限才給）
const int ASSIST_PWM = 90;

// ---------------- Prevent tremble ----------------
// 1) 放線後寬鬆期：禁止立刻收線
// --- Retract 判斷用變數 ---
bool inRetract = false;         // if it's retracting
unsigned long retract_start_ms; // retract start
long retract_start_cnt;         // retract encoder count

// --- Retract 進入的最小條件 ---
const unsigned long MIN_RETRACT_MS = 50;  // 至少 100 ms
const long MIN_RETRACT_CNT = 10;           // 至少轉過 20 counts

const unsigned long LOOSEN_GRACE_MS = 120;
unsigned long loosen_grace_until_ms = 0;

// 2) retracting lag：a bit lower than lower_limit 
const float LOWER_HYST_G = 0.8f;  // ★ lower_limit - 0.8g 才允許收線

// ---------------- rh ----------------
const unsigned long LOOP_MS = 2;
unsigned long last_loop_ms = 0;

// ---------------- helpers ----------------
inline void motorCoast() {
#ifdef HAVE_NSLEEP
  digitalWrite(PIN_NSLEEP, LOW);
#endif
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void driveMotor(int pwmSigned) {
#ifdef HAVE_NSLEEP
  digitalWrite(PIN_NSLEEP, HIGH);
#endif
  if (pwmSigned == 0) { motorCoast(); return; }
  if (pwmSigned > 0) {
    int pwm = constrain(pwmSigned, 0, 255);
    digitalWrite(AIN1, LOW);
    analogWrite(AIN2, pwm);
  } else {
    int pwm = constrain(-pwmSigned, 0, 255);
    analogWrite(AIN1, pwm);
    digitalWrite(AIN2, LOW);
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

// ---------------- setup ----------------
void setup() {
  Serial.begin(9600);
  delay(300);

  scale.begin(DOUT, CLK);
  delay(500);
  if (!scale.is_ready()) { Serial.println("HX711 not found."); while (1); }
  scale.set_scale(calibration_factor);
  scale.tare();

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
#ifdef HAVE_NSLEEP
  pinMode(PIN_NSLEEP, OUTPUT);
#endif
  motorCoast();

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  lastVtMs = millis();
  last_loop_ms = millis();
  last_T_ms = millis();
  T_prev_for_dt = 0.0f;

  Serial.println("Ready (anti-chatter + grace).");
}

// ---------------- loop ----------------
void loop() {
  if (millis() - last_loop_ms < LOOP_MS) return;
  last_loop_ms = millis();

  // ---- 張力：雙 EMA ----
  if (scale.is_ready()) {
    float raw = scale.get_units(1);
    T_fast_g = T_fast_g + T_ALPHA_FAST * (raw - T_fast_g);
    T_slow_g = T_slow_g + T_ALPHA_SLOW * (raw - T_slow_g);
  }
  float T_fast = T_fast_g;
  float T_slow = T_slow_g;

  // ---- v_line ----
  unsigned long now = millis();
  unsigned long dt_ms = now - lastVtMs; if (dt_ms == 0) dt_ms = 1;
  lastVtMs = now;

  long cnt; noInterrupts(); cnt = encoderCount; interrupts();
  long dcount = cnt - lastCount; lastCount = cnt;

  float dt = dt_ms * 1e-3f;
  v_line = (COUNTS_PER_METER > 1e-6f) ? ((float)dcount / COUNTS_PER_METER) / dt : 0.0f;
  // 如方向顛倒：取消註解
  //v_line = -v_line;
  leash_len_m += v_line * dt;

  // ---- dT/dt ----
  unsigned long nowT = millis();
  float dtT_s = max(1.0f, (float)(nowT - last_T_ms)) * 1e-3f;
  dTdt_gps = (T_fast - T_prev_for_dt) / dtT_s;
  T_prev_for_dt = T_fast; last_T_ms = nowT;

  // ---- EMG ----
  if (T_fast > threshold_emg) { emg_boost = true; emg_until_ms = millis() + EMG_BOOST_MS; }
  if (emg_boost && millis() > emg_until_ms) emg_boost = false;

  // ---- 立即觸發（強拉/快速上升） → 立刻放線 + burst ----
  bool instant_payout = (T_fast > upper_limit + 1.0f) || (dTdt_gps > 80.0f);
  if (instant_payout) {
    allow_payout_latch = true;
    allow_payout_since_ms = millis();
    burst_until_ms = millis() + BURST_MS;
    loosen_grace_until_ms = millis() + LOOSEN_GRACE_MS;   // ★ 放線後寬鬆期
  }

  // ---- 放線許可：方向 + 張力 + 連續 ----
  bool line_out = (v_line < -V_OUT_MIN);
  bool high_T   = (T_fast > (upper_limit + T_MARGIN_G));

  if (line_out && high_T) {
    if (!allow_payout_latch) { allow_payout_latch = true; allow_payout_since_ms = millis(); }
  } else {
    allow_payout_latch = false;
  }
  bool allow_payout_dir = allow_payout_latch && (millis() - allow_payout_since_ms >= ALLOW_PAYOUT_CONFIRM_MS);

  // 鎖軸快速通道
  bool spool_static = (fabs(v_line) < V_EPS_LOCKED);
  bool locked_trig  = spool_static && (T_fast > upper_limit + T_LOCK_MARGING) && (dTdt_gps > DTDT_LOCK_GPS);
  if (locked_trig) {
    if (!locked_latch) { locked_latch = true; locked_since_ms = millis(); }
  } else {
    locked_latch = false;
  }
  bool allow_payout_locked = locked_latch && (millis() - locked_since_ms >= LOCK_CONFIRM_MS);
  bool allow_payout = allow_payout_dir || allow_payout_locked;

  // ---- 主邏輯：目標 PWM（含 anti-chatter） ----
  int targetPWM = 0; // >0 收線；<0 放線；=0 停

  if (T_fast > upper_limit) {
    // 放線
    if (emg_boost) {
      targetPWM = -255;
      loosen_grace_until_ms = millis() + LOOSEN_GRACE_MS;
    } else if (allow_payout) {
      float over_g = T_fast - upper_limit;
      int pwm = (int)(BASE_PAYOUT_PWM + KP_PWM_PER_G * over_g);
      pwm = constrain(pwm, BASE_PAYOUT_PWM, MAX_PAYOUT_PWM);
      targetPWM = -pwm;
      loosen_grace_until_ms = millis() + LOOSEN_GRACE_MS; // ★ 每次有效放線都延長寬鬆期
    } else {
      // 許可尚未成立 → 看是否在 burst 期間
      if (millis() < burst_until_ms) {
        targetPWM = -BURST_PWM;
        loosen_grace_until_ms = millis() + LOOSEN_GRACE_MS;
      } else {
        targetPWM = 0; // 等待，交由 Active-Coast（下方）輕推
      }
    }

  } else if (T_slow < (lower_limit - LOWER_HYST_G) && millis() >= loosen_grace_until_ms) {
    // ★ 只有在「超過遲滯」且「放線寬鬆期結束」後，才允許收線
    if (!inRetract) { inRetract = true; retract_start_ms = millis(); retract_start_cnt = cnt; }
    targetPWM = +remotorSpeed;

  } else if (T_fast < lower_limit) {
    // ★ 介於 lower_limit 與 lower_limit-LOWER_HYST_G：先停止，不立刻收
    targetPWM = 0;

  } else {
    // 區間內
    if (inRetract) {
      bool time_ok = (millis() - retract_start_ms) >= MIN_RETRACT_MS;
      bool cnt_ok  = (labs(cnt - retract_start_cnt) >= MIN_RETRACT_CNT);
      targetPWM = (time_ok && cnt_ok) ? 0 : +remotorSpeed;
      if (time_ok && cnt_ok) inRetract = false;
    } else {
      targetPWM = 0;
    }
  }

  // ---- Active-Coast：只在接近上限 0.5 g 內才給輕助力（避免滑出）----
  bool nearly_upper = (T_fast >= (upper_limit - 0.5f));
  if (targetPWM == 0 && nearly_upper && spool_static) {
    targetPWM = -ASSIST_PWM;
  }

  // ---- 脈衝期間直通，其餘走 ramp ----
  bool burst_active = (millis() < burst_until_ms);
  int steppedPWM = burst_active ? targetPWM : slewPWM(targetPWM);

  // ---- 輸出 ----
  driveMotor(steppedPWM);

  // ---- Debug ----
  if ((millis() % 100) < LOOP_MS) {
    Serial.print("T_fast(g)="); Serial.print(T_fast, 2);
    Serial.print("  T_slow(g)="); Serial.print(T_slow, 2);
    Serial.print("  v_line(m/s)="); Serial.print(v_line, 3);
    Serial.print("  mode=");
    if (steppedPWM > 0) Serial.print("RETRACT");
    else if (steppedPWM < 0) Serial.print("LOOSEN");
    else Serial.print("STOP");
    Serial.print("  allow="); Serial.print(allow_payout ? "Y" : "N");
    Serial.print("  dTdt(g/s)="); Serial.print(dTdt_gps, 0);
    Serial.print("  grace(ms)="); Serial.print(max(0, (int)(loosen_grace_until_ms - millis())));
    Serial.print("  burst="); Serial.print(burst_active ? "Y":"N");
    Serial.println();
  }
}

// ---------------- ISR ----------------
void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  if (a == b) encoderCount++;
  else        encoderCount--;
}
