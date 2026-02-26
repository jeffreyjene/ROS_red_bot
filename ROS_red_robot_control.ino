#include <Arduino.h>

// -------- Robot Parameters --------
const float WHEEL_RADIUS = 0.0325;
const float WHEEL_BASE   = 0.15;
const float TICKS_PER_REV = 1940.0;
const float CONTROL_DT = 0.01; // 100 Hz

// -------- TB6612 Pins --------
#define AIN1 7
#define AIN2 8
#define BIN2 9
#define BIN1 10
#define PWMA 5
#define PWMB 6
#define STBY 4

// -------- Encoder Pins --------
#define ENCODER_L 2
#define ENCODER_R 3

volatile long left_ticks = 0;
volatile long right_ticks = 0;

long prev_left_ticks = 0;
long prev_right_ticks = 0;

// -------- Robot State --------
float x = 0, y = 0, theta = 0;

float target_v = 0;
float target_w = 0;

float target_w_l = 0;
float target_w_r = 0;

float measured_w_l = 0;
float measured_w_r = 0;

// -------- PID --------
float Kp = 10;
float Ki = 5;
float Kd = 0.1;

float err_l = 0, prev_err_l = 0, int_l = 0;
float err_r = 0, prev_err_r = 0, int_r = 0;

// -------- Timing --------
unsigned long last_control = 0;
unsigned long last_cmd_time = 0;

// -------- Encoder ISRs --------
void leftEncoderISR() { left_ticks++; }
void rightEncoderISR() { right_ticks++; }

// -------- Motor Control --------
void setMotor(int pwm, int in1, int in2, int pwmPin) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwm = -pwm;
  }
  analogWrite(pwmPin, constrain(pwm, 0, 255));
}

// -------- Serial Parsing --------
void readSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  if (!line.startsWith("$CMD")) return;

  int first = line.indexOf(',');
  int second = line.indexOf(',', first + 1);

  if (first < 0 || second < 0) return;

  target_v = line.substring(first + 1, second).toFloat();
  target_w = line.substring(second + 1).toFloat();

  last_cmd_time = millis();
}

// -------- Setup --------
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

  pinMode(ENCODER_L, INPUT_PULLUP);
  pinMode(ENCODER_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), rightEncoderISR, RISING);
}

// -------- Main Loop --------
void loop() {
  readSerial();

  if (millis() - last_control >= 10) {
    controlLoop();
    last_control = millis();
  }
}

// -------- Control Loop --------
void controlLoop() {

  // Timeout safety
  if (millis() - last_cmd_time > 500) {
    target_v = 0;
    target_w = 0;
  }

  // Convert cmd_vel → wheel angular velocity
  target_w_l = (target_v - (WHEEL_BASE / 2.0) * target_w) / WHEEL_RADIUS;
  target_w_r = (target_v + (WHEEL_BASE / 2.0) * target_w) / WHEEL_RADIUS;

  // MAX wheel angular velocity (rad/s)
  const float MAX_WHEEL_W = 12.0;  // safe for 160 RPM motors

  target_w_l = constrain(target_w_l, -MAX_WHEEL_W, MAX_WHEEL_W);
  target_w_r = constrain(target_w_r, -MAX_WHEEL_W, MAX_WHEEL_W);

  // Tick difference
  long dL = left_ticks - prev_left_ticks;
  long dR = right_ticks - prev_right_ticks;

  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  // Measured angular velocity
  measured_w_l = (dL / TICKS_PER_REV) * 2.0 * PI / CONTROL_DT;
  measured_w_r = (dR / TICKS_PER_REV) * 2.0 * PI / CONTROL_DT;

  // PID LEFT
  err_l = target_w_l - measured_w_l;
  int_l += err_l * CONTROL_DT;
  float d_l = (err_l - prev_err_l) / CONTROL_DT;
  float pwm_l = Kp * err_l + Ki * int_l + Kd * d_l;
  prev_err_l = err_l;

  // PID RIGHT
  err_r = target_w_r - measured_w_r;
  int_r += err_r * CONTROL_DT;
  float d_r = (err_r - prev_err_r) / CONTROL_DT;
  float pwm_r = Kp * err_r + Ki * int_r + Kd * d_r;
  prev_err_r = err_r;

  setMotor((int)pwm_l, AIN1, AIN2, PWMA);
  setMotor((int)pwm_r, BIN1, BIN2, PWMB);

  // -------- Odometry --------
  float dtheta_l = (dL / TICKS_PER_REV) * 2.0 * PI;
  float dtheta_r = (dR / TICKS_PER_REV) * 2.0 * PI;

  float ds = WHEEL_RADIUS * (dtheta_r + dtheta_l) / 2.0;
  float dtheta = WHEEL_RADIUS * (dtheta_r - dtheta_l) / WHEEL_BASE;

  x += ds * cos(theta + dtheta / 2.0);
  y += ds * sin(theta + dtheta / 2.0);
  theta += dtheta;

  // -------- Send ODOM --------
  Serial.print("$ODOM,");
  Serial.print(x, 4); Serial.print(",");
  Serial.print(y, 4); Serial.print(",");
  Serial.print(theta, 4); Serial.print(",");
  Serial.print(measured_w_l * WHEEL_RADIUS, 4); Serial.print(",");
  Serial.println(measured_w_r * WHEEL_RADIUS, 4);
}

//$CMD,0.2,0.0