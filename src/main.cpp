#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <IRremote.h>

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

// ===== Safety =====
#define IR_SAFETY_TIMEOUT  2000

// ===== Globals =====
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
IRrecv irrecv(PIN_IR_RECV);
decode_results irResults;

enum State { STOPPED, FORWARD, BACKWARD };
State state = STOPPED;
unsigned long lastIRTime = 0;

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
