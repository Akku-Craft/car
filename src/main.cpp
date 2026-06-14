#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <IRremote.h>
#include "MPU6050_getdata.h"

// ===== Motor Driver Pins (Elegoo V4.0 / TB6612) =====
#define PIN_MOTOR_PWMA  5
#define PIN_MOTOR_PWMB  6
#define PIN_MOTOR_AIN1  7
#define PIN_MOTOR_BIN1  8
#define PIN_MOTOR_STBY  3

#define PIN_IR_RECV     9

#define MOTOR_SPEED     200

// ===== IR Remote Codes (Elegoo V4.0 remote) =====
// Type A remote
#define IR_A_UP    16736925UL
#define IR_A_DOWN  16754775UL
// Type B remote
#define IR_B_UP    5316027UL
#define IR_B_DOWN  2747854299UL
// Left / Right (from Elegoo driver headers)
#define IR_A_LEFT  16720605UL
#define IR_A_RIGHT 16761405UL
#define IR_B_LEFT  1386468383UL
#define IR_B_RIGHT 553536955UL

// ===== Safety =====
#define IR_SAFETY_TIMEOUT  2000

// ===== Globals =====
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
IRrecv irrecv(PIN_IR_RECV);
decode_results irResults;

enum State { STOPPED, FORWARD, BACKWARD };
State state = STOPPED;
unsigned long lastIRTime = 0;
bool isTurning = false;

// Turn configuration
#define TURN_DEGREES_DEFAULT 45
#define TURN_MS_PER_DEGREE 14 // fallback calibration (ms per degree)
#define TURN_PWM 200
#define TURN_TIMEOUT_MS 3000
#define TURN_TOLERANCE_DEG 3

// ===== Motor =====
void motorInit() {
  pinMode(PIN_MOTOR_PWMA, OUTPUT);
  pinMode(PIN_MOTOR_PWMB, OUTPUT);
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_BIN1, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, LOW);
}

void motorApply(State s, uint8_t speed) {
  if (s == STOPPED) {
    analogWrite(PIN_MOTOR_PWMA, 0);
    analogWrite(PIN_MOTOR_PWMB, 0);
    digitalWrite(PIN_MOTOR_STBY, LOW);
    return;
  }
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  if (s == FORWARD) {
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMA, speed);
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMB, speed);
  } else {
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    analogWrite(PIN_MOTOR_PWMA, speed);
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    analogWrite(PIN_MOTOR_PWMB, speed);
  }
}

// Turn in-place: direction true = LEFT, false = RIGHT
void motorTurn(bool turnLeft, uint8_t speed) {
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  if (turnLeft) {
    // Left turn: right motor forward, left motor backward
    digitalWrite(PIN_MOTOR_AIN1, HIGH); // right forward
    analogWrite(PIN_MOTOR_PWMA, speed);
    digitalWrite(PIN_MOTOR_BIN1, LOW);  // left backward
    analogWrite(PIN_MOTOR_PWMB, speed);
  } else {
    // Right turn: right motor backward, left motor forward
    digitalWrite(PIN_MOTOR_AIN1, LOW);  // right backward
    analogWrite(PIN_MOTOR_PWMA, speed);
    digitalWrite(PIN_MOTOR_BIN1, HIGH); // left forward
    analogWrite(PIN_MOTOR_PWMB, speed);
  }
}

void motorStop() {
  analogWrite(PIN_MOTOR_PWMA, 0);
  analogWrite(PIN_MOTOR_PWMB, 0);
  digitalWrite(PIN_MOTOR_STBY, LOW);
}

// Display helper for turning
void displayTurn(const char *dir, int degrees) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(8, 22, "TURNING");
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(8, 40, dir);
  u8g2.setCursor(80, 50);
  u8g2.print(degrees);
  u8g2.print(" deg");
  u8g2.sendBuffer();
}

// Blocking turn by degrees using a time-based fallback implementation.
// Uses MPU6050 if available and returning sensible yaw values could be
// implemented later; for now we use a calibrated ms/degree.
void turnByAngle(int degrees, bool left) {
  isTurning = true;
  displayTurn(left ? "LEFT" : "RIGHT", degrees);

  // Try IMU-based closed-loop turn first
  float startYaw = 0.0f;
  MPU6050Getdata.MPU6050_dveGetEulerAngles(&startYaw); // initialize integrator
  delay(20);
  MPU6050Getdata.MPU6050_dveGetEulerAngles(&startYaw);
  float targetYaw = startYaw + (left ? degrees : -degrees);
  unsigned long startTime = millis();
  motorTurn(left, TURN_PWM);
  // loop until target reached or timeout
  while (millis() - startTime < TURN_TIMEOUT_MS) {
    float currentYaw = 0.0f;
    MPU6050Getdata.MPU6050_dveGetEulerAngles(&currentYaw);
    if (left) {
      if (currentYaw >= targetYaw - TURN_TOLERANCE_DEG) break;
    } else {
      if (currentYaw <= targetYaw + TURN_TOLERANCE_DEG) break;
    }
    delay(10);
  }
  motorStop();
  // If IMU didn't reach the goal, do a short time-based correction as fallback
  float finalYaw = 0.0f;
  MPU6050Getdata.MPU6050_dveGetEulerAngles(&finalYaw);
  float delta = fabs(finalYaw - startYaw);
  if (fabs(delta - degrees) > TURN_TOLERANCE_DEG) {
    // fallback: time-based correction
    unsigned long extraMs = (unsigned long)((degrees - delta) * TURN_MS_PER_DEGREE);
    if ((int)extraMs > 0) {
      motorTurn(left, TURN_PWM);
      unsigned long s = millis();
      while (millis() - s < extraMs && millis() - startTime < TURN_TIMEOUT_MS) delay(10);
      motorStop();
    }
  }
  isTurning = false;
  displayUpdate(STOPPED);
}

void turnRequest(bool left, int degrees) {
  if (isTurning) return;
  turnByAngle(degrees, left);
}

// ===== OLED =====
void displayUpdate(State s) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  switch (s) {
    case FORWARD:  u8g2.drawStr(12, 30, "FORWARD");  break;
    case BACKWARD: u8g2.drawStr(12, 30, "REVERSE");  break;
    default:       u8g2.drawStr(16, 30, "STOPPED");  break;
  }
  u8g2.drawFrame(0, 40, 128, 24);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(4, 56);
  u8g2.print("Speed: ");
  u8g2.print(MOTOR_SPEED);
  uint8_t bw = map(MOTOR_SPEED, 0, 255, 0, 120);
  u8g2.drawBox(4, 42, bw, 6);
  u8g2.sendBuffer();
}

// ===== Setup =====
void setup() {
  Wire.begin();
  u8g2.begin();
  motorInit();
  irrecv.enableIRIn();
  // Initialize IMU (MPU6050) if present
  MPU6050Getdata.MPU6050_dveInit();
  MPU6050Getdata.MPU6050_calibration();
  displayUpdate(STOPPED);
}

// ===== Main Loop =====
void loop() {
  if (irrecv.decode(&irResults)) {
    lastIRTime = millis();
    State newState = state;
    if (irResults.value == IR_A_UP || irResults.value == IR_B_UP)
      newState = FORWARD;
    else if (irResults.value == IR_A_DOWN || irResults.value == IR_B_DOWN)
      newState = BACKWARD;
    else if (irResults.value == IR_A_LEFT || irResults.value == IR_B_LEFT) {
      // trigger left turn (blocking)
      state = STOPPED;
      motorApply(state, 0);
      displayUpdate(state);
      turnRequest(true, TURN_DEGREES_DEFAULT);
      irrecv.resume();
      continue;
    } else if (irResults.value == IR_A_RIGHT || irResults.value == IR_B_RIGHT) {
      // trigger right turn (blocking)
      state = STOPPED;
      motorApply(state, 0);
      displayUpdate(state);
      turnRequest(false, TURN_DEGREES_DEFAULT);
      irrecv.resume();
      continue;
    }
    else if (irResults.value != REPEAT)
      newState = STOPPED;
    if (newState != state) {
      state = newState;
      motorApply(state, MOTOR_SPEED);
      displayUpdate(state);
    }
    irrecv.resume();
  }

  if (state != STOPPED && millis() - lastIRTime > IR_SAFETY_TIMEOUT) {
    state = STOPPED;
    motorApply(state, 0);
    displayUpdate(state);
  }
}
