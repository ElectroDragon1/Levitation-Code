/*
 * STM32 Maglev Coil Driver — Current-Mode Control
 *
 * Protocol (3 bytes per frame, value little-endian):
 *   'P' lo hi   → direct PWM mode (calibration), value = 0..4095
 *   'I' lo hi   → current mode (target in milliamps), value = 0..4095
 *
 * TX: "I:<amps>\n" ASCII at ~500 Hz.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA260.h>

// ===== Pins =====
#define PWM_PIN  PA3
#define IN3_PIN  PG2
#define IN4_PIN  PG3
#define LED_PIN  LED_GREEN

// ===== PWM hardware =====
#define PWM_RES  4096
#define PWM_MAX  (PWM_RES - 1)
HardwareTimer *PwmTim;
const uint32_t PWM_CHANNEL = 4;

// ===== INA260 =====
Adafruit_INA260 ina260;
bool ina260_ok = false;

// ===== Mode + setpoints =====
enum Mode { MODE_PWM, MODE_CURRENT };
Mode mode = MODE_PWM;
uint16_t target_pwm  = 0;     // raw-PWM setpoint
uint16_t target_ma   = 0;     // current-mode setpoint (mA)
uint16_t actual_pwm  = 0;     // last value written to the timer
float    measured_ma = 0.0f;  // last INA260 reading

// ===== PI controller =====
float integrator = 0.0f;
unsigned long last_loop_us = 0;

// === TUNING — start here and adjust via step-response tests ===
// Values below scaled for 24V supply (plant gain doubles vs 12V)
const float FF_OFFSET = 950.0f;      // PWM offset to cross H-bridge dead zone at 24V
const float FF_SLOPE  = 0.74f;       // PWM per mA, slope from coil-current LUT (24V)
const float KP_PI     = 2.0f;        // PWM per mA error — halved for 24V
const float KI_PI     = 40.0f;       // PWM per (mA · s) — halved for 24V
const float INT_MAX   = 2500.0f;     // anti-windup limits
const float INT_MIN   = -2500.0f;
const uint16_t HARD_PWM_MAX  = 3900; // slew headroom (no longer the safety limit)
const uint16_t MAX_TARGET_MA = 5000; // current ceiling — safety limit at 24V
const uint16_t BRAKE_CURRENT_FLOOR_MA = 200;  // stop active brake below this (prevents reversal)

// ===== Telemetry =====
#define REPORT_MS 2
unsigned long last_report = 0;
unsigned long last_blink  = 0;
bool led_state = false;

// ===== Serial framer =====
uint8_t rx_buf[3];
uint8_t rx_idx = 0;
bool    rx_awaiting_sync = true;

// =============================================================

// Set H-bridge direction. true = forward (current builds), false = reverse (active brake).
static inline void set_drive_direction(bool forward) {
  if (forward) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  }
}

// Unsigned PWM (always forward direction). Used for raw 'P' frames.
static inline void apply_pwm(int pwm) {
  set_drive_direction(true);
  if (pwm < 0) pwm = 0;
  if (pwm > HARD_PWM_MAX) pwm = HARD_PWM_MAX;
  actual_pwm = (uint16_t)pwm;
  PwmTim->setCaptureCompare(PWM_CHANNEL, actual_pwm, TICK_COMPARE_FORMAT);
}

// Signed PWM. Positive = drive current up. Negative = active brake (current down).
// Magnitude controls PWM duty; sign controls H-bridge direction.
static inline void apply_pwm_signed(int pwm_signed) {
  bool forward = (pwm_signed >= 0);
  set_drive_direction(forward);
  int pwm_abs = forward ? pwm_signed : -pwm_signed;
  if (pwm_abs > HARD_PWM_MAX) pwm_abs = HARD_PWM_MAX;
  actual_pwm = (uint16_t)pwm_abs;
  PwmTim->setCaptureCompare(PWM_CHANNEL, actual_pwm, TICK_COMPARE_FORMAT);
}

void run_current_loop() {
  if (!ina260_ok) return;
  if (mode != MODE_CURRENT) return;

// SIGNED current: positive = forward direction (the way we normally drive).
  // Your wiring reads negative for forward, so we flip the sign.
  measured_ma = -ina260.readCurrent();

  // Special case: target=0 means actively brake to zero, then idle.
  // Only brake if we KNOW we have forward current to bring down.
  // (If measured is 0 or negative, braking would just push current reverse.)
  if (target_ma == 0) {
    if (measured_ma > (float)BRAKE_CURRENT_FLOOR_MA) {
      apply_pwm_signed(-HARD_PWM_MAX);   // forward current above floor → brake
    } else {
      apply_pwm_signed(0);               // already low/zero/reverse → coast
      integrator = 0;
    }
    last_loop_us = micros();
    return;
  }

  unsigned long now = micros();
  float dt = (now - last_loop_us) * 1.0e-6f;
  last_loop_us = now;
  if (dt > 0.05f) dt = 0.001f;

  float err = (float)target_ma - measured_ma;

  float ff = FF_OFFSET + FF_SLOPE * (float)target_ma;
  float pi = KP_PI * err + KI_PI * integrator;
  float command = ff + pi;     // SIGNED — can be negative for active brake

  // Anti-windup: don't accumulate integrator in a direction that worsens saturation
  bool sat_high = (command >=  (float)HARD_PWM_MAX && err > 0);
  bool sat_low  = (command <= -(float)HARD_PWM_MAX && err < 0);
  if (!sat_high && !sat_low) {
    integrator += err * dt;
    if (integrator > INT_MAX) integrator = INT_MAX;
    if (integrator < INT_MIN) integrator = INT_MIN;
  }

  // Brake floor: stop reversing the bridge once current is already small.
  // Prevents the loop from driving current through zero into reverse
  // (which fabsf() of the INA260 reading would misinterpret as "still high").
  if (command < 0.0f && measured_ma < (float)BRAKE_CURRENT_FLOOR_MA) {
    command = 0.0f;
  }

  apply_pwm_signed((int)command);
}


void handle_serial() {
  while (Serial3.available()) {
    uint8_t b = Serial3.read();

    if (rx_awaiting_sync) {
      if (b == 'P' || b == 'I') {
        rx_buf[0] = b;
        rx_idx = 1;
        rx_awaiting_sync = false;
      }
      continue;
    }

    rx_buf[rx_idx++] = b;
    if (rx_idx >= 3) {
      uint8_t cmd = rx_buf[0];
      uint16_t val = (uint16_t)rx_buf[1] | ((uint16_t)rx_buf[2] << 8);

      if (cmd == 'P') {
        mode = MODE_PWM;
        target_pwm = (val > PWM_MAX) ? PWM_MAX : val;
        integrator = 0;
        apply_pwm(target_pwm);
      } else { // 'I'
        if (mode != MODE_CURRENT) {
          integrator = 0;
          last_loop_us = micros();
        }
        mode = MODE_CURRENT;
        if (val > MAX_TARGET_MA) val = MAX_TARGET_MA;
        target_ma = val;
      }

      rx_idx = 0;
      rx_awaiting_sync = true;
    }
  }
}

void setup() {
  Serial3.begin(115200);
  while (Serial3.available() > 0) Serial3.read();

  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);

  PwmTim = new HardwareTimer(TIM2);
  PwmTim->pause();
  PwmTim->setMode(PWM_CHANNEL, TIMER_OUTPUT_COMPARE_PWM1, PWM_PIN);
  PwmTim->setPrescaleFactor(1);
  PwmTim->setOverflow(PWM_MAX, TICK_FORMAT);
  PwmTim->setCaptureCompare(PWM_CHANNEL, 0, TICK_COMPARE_FORMAT);
  PwmTim->resume();

  Wire.begin();
  Wire.setClock(400000);
  ina260_ok = ina260.begin();
  if (ina260_ok) {
    ina260.setAveragingCount(INA260_COUNT_4);
    ina260.setCurrentConversionTime(INA260_TIME_140_us);
    ina260.setVoltageConversionTime(INA260_TIME_140_us);
    Serial3.println("Ready: P-frame=PWM, I-frame=mA (current-mode PI loop)");
  } else {
    Serial3.println("ERR: INA260 not detected");
  }

  last_loop_us = micros();
}

void loop() {
  unsigned long now = millis();

  handle_serial();
  run_current_loop();

  if (now - last_blink >= 500) {
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state);
    last_blink = now;
  }

  if (ina260_ok && (now - last_report >= REPORT_MS)) {
    last_report = now;
    if (mode != MODE_CURRENT) {
      // Refresh when PI loop isn't running. Same signed convention as run_current_loop.
      measured_ma = -ina260.readCurrent();
    }
    // Signed amps as float — negative means reverse direction.
    // If you see a negative value during normal operation, something is wrong.
    Serial3.print("I:");
    Serial3.println(measured_ma * 0.001f, 4);
  }
}