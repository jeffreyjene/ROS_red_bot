#include <Arduino.h>

// ====================================================
// ROBOT PARAMETERS
// ====================================================
const float WHEEL_RADIUS = 0.0325;     // meters
const float WHEEL_BASE   = 0.15;       // meters
const float TICKS_PER_REV = 1940.0;
const float CONTROL_DT = 0.01;          // 100 Hz

// ====================================================
// SAFETY LIMITS (NON-NEGOTIABLE)
// ====================================================
const float MAX_FWD_V = 0.30;           // m/s forward
const float MAX_REV_V = 0.15;           // m/s reverse (slower)
const float MAX_WHEEL_W = 12.0;          // rad/s
const float MAX_MEASURED_W = 20.0;       // sanity clamp
const int   MAX_PWM = 150;               // NEVER 255
const int   MIN_PWM = 20;                // overcome gearbox friction
const float I_LIMIT = 2.0;               // integral clamp

// ====================================================
// TB6612 PINS
// ====================================================
#define AIN1 7
#define AIN2 8
#define BIN2 9
#define BIN1 10
#define PWMA 5
#define PWMB 6
#define STBY 4

// ====================================================
// ENCODER PINS (QUADRATURE)
// ====================================================
#define ENCODER_L_A 2    // interrupt
#define ENCODER_L_B 11

#define ENCODER_R_A 3    // interrupt
#define ENCODER_R_B 12

volatile long left_ticks  = 0;
volatile long right_ticks = 0;

long prev_left_ticks  = 0;
long prev_right_ticks = 0;

// ====================================================
// ROBOT STATE (ODOM)
// ====================================================
float x = 0.0;
float y = 0.0;
float theta = 0.0;

// ====================================================
// COMMANDS
// ====================================================
float target_v = 0.0;
float target_w = 0.0;

// ====================================================
// WHEEL TARGETS
// ====================================================
float target_w_l = 0.0;
float target_w_r = 0.0;
float measured_w_l = 0.0;
float measured_w_r = 0.0;

// ====================================================
// PID CONTROLLER
// ====================================================
float Kp = 10.0;
float Ki = 3.0;
float Kd = 0.05;

float err_l = 0.0, prev_err_l = 0.0, int_l = 0.0;
float err_r = 0.0, prev_err_r = 0.0, int_r = 0.0;

// ====================================================
// TIMING
// ====================================================
unsigned long last_control = 0;
unsigned long last_cmd_time = 0;

// ====================================================
// QUADRATURE ISRs  (THIS IS THE KEY FIX)
// ====================================================
void leftEncoderISR() {
  if (digitalRead(ENCODER_L_B) == LOW) {
    left_ticks++;    // forward
  } else {
    left_ticks--;    // reverse
  }
}

void rightEncoderISR() {
  if (digitalRead(ENCODER_R_B) == LOW) {
    right_ticks--;   // forward
  } else {
    right_ticks++;   // reverse
  }
}

// ====================================================
// MOTOR CONTROL (COAST MODE)
// ====================================================
void setMotor(int pwm, int in1, int in2, int pwmPin) {

  if (abs(pwm) < MIN_PWM) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);   // COAST
    analogWrite(pwmPin, 0);
    return;
  }

  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwm = -pwm;
  }

  analogWrite(pwmPin, constrain(pwm, 0, MAX_PWM));
}

// ====================================================
// SERIAL INPUT ($CMD,v,w)
// ====================================================
void readSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  if (!line.startsWith("$CMD")) return;

  int first = line.indexOf(',');
  int second = line.indexOf(',', first + 1);
  if (first < 0 || second < 0) return;

  float v = line.substring(first + 1, second).toFloat();
  float w = line.substring(second + 1).toFloat();

  if (abs(v) > 1.0 || abs(w) > 5.0) return;

  target_v = v;
  target_w = w;
  last_cmd_time = millis();
}

// ====================================================
// SETUP
// ====================================================
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, RISING);
}

// ====================================================
// MAIN LOOP
// ====================================================
void loop() {
  readSerial();

  if (millis() - last_control >= 10) {
    controlLoop();
    last_control = millis();
  }
}

// ====================================================
// CONTROL LOOP
// ====================================================
void controlLoop() {

  // ---------- TIMEOUT SAFETY ----------
  if (millis() - last_cmd_time > 500) {
    target_v = 0;
    target_w = 0;
    int_l = int_r = 0;
    prev_err_l = prev_err_r = 0;
  }

  // ---------- LINEAR VELOCITY LIMITS ----------
  if (target_v > MAX_FWD_V) {
    target_v = MAX_FWD_V;
  } else if (target_v < -MAX_REV_V) {
    target_v = -MAX_REV_V;
  }

  // ---------- CMD → WHEEL TARGETS ----------
  target_w_l = (target_v - (WHEEL_BASE / 2.0) * target_w) / WHEEL_RADIUS;
  target_w_r = (target_v + (WHEEL_BASE / 2.0) * target_w) / WHEEL_RADIUS;

  target_w_l = constrain(target_w_l, -MAX_WHEEL_W, MAX_WHEEL_W);
  target_w_r = constrain(target_w_r, -MAX_WHEEL_W, MAX_WHEEL_W);

  // ---------- ENCODERS ----------
  long dL = left_ticks  - prev_left_ticks;
  long dR = right_ticks - prev_right_ticks;

  prev_left_ticks  = left_ticks;
  prev_right_ticks = right_ticks;

  measured_w_l = (dL / TICKS_PER_REV) * 2.0 * PI / CONTROL_DT;
  measured_w_r = (dR / TICKS_PER_REV) * 2.0 * PI / CONTROL_DT;

  measured_w_l = constrain(measured_w_l, -MAX_MEASURED_W, MAX_MEASURED_W);
  measured_w_r = constrain(measured_w_r, -MAX_MEASURED_W, MAX_MEASURED_W);

  // ---------- PID LEFT ----------
  err_l = target_w_l - measured_w_l;
  int_l += err_l * CONTROL_DT;
  int_l = constrain(int_l, -I_LIMIT, I_LIMIT);
  float pwm_l = Kp * err_l + Ki * int_l + Kd * ((err_l - prev_err_l) / CONTROL_DT);
  prev_err_l = err_l;

  // ---------- PID RIGHT ----------
  err_r = target_w_r - measured_w_r;
  int_r += err_r * CONTROL_DT;
  int_r = constrain(int_r, -I_LIMIT, I_LIMIT);
  float pwm_r = Kp * err_r + Ki * int_r + Kd * ((err_r - prev_err_r) / CONTROL_DT);
  prev_err_r = err_r;

  pwm_l = constrain(pwm_l, -MAX_PWM, MAX_PWM);
  pwm_r = constrain(pwm_r, -MAX_PWM, MAX_PWM);

  // ---------- APPLY MOTORS ----------
  setMotor((int)pwm_l, AIN1, AIN2, PWMA);
  setMotor((int)pwm_r, BIN1, BIN2, PWMB);

  // ---------- ODOMETRY ----------
  float dtheta_l = (dL / TICKS_PER_REV) * 2.0 * PI;
  float dtheta_r = (dR / TICKS_PER_REV) * 2.0 * PI;

  float ds = WHEEL_RADIUS * (dtheta_r + dtheta_l) / 2.0;
  float dtheta = WHEEL_RADIUS * (dtheta_r - dtheta_l) / WHEEL_BASE;

  x     += ds * cos(theta + dtheta / 2.0);
  y     += ds * sin(theta + dtheta / 2.0);
  theta += dtheta;

  // ---------- SEND ODOM ----------
  Serial.print("$ODOM,");
  Serial.print(x, 4); Serial.print(",");
  Serial.print(y, 4); Serial.print(",");
  Serial.print(theta, 4); Serial.print(",");
  Serial.print(measured_w_l * WHEEL_RADIUS, 4); Serial.print(",");
  Serial.println(measured_w_r * WHEEL_RADIUS, 4);
}
