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
// scale motor output by 75% (use integer math to avoid floats)
#define MOTOR_SCALE_NUM  3
#define MOTOR_SCALE_DEN  4

// ===== IR Remote Codes (Elegoo V4.0 remote) =====
// Type A remote
#define IR_A_2     16736925UL   // button 2 instead of UP
#define IR_A_8     16754775UL   // button 8 instead of DOWN
//#define IR_A_LEFT  16720605UL
//#define IR_A_RIGHT 16761405UL
// Type B remote
//#define IR_B_UP    5316027UL
//#define IR_B_DOWN  2747854299UL
//#define IR_B_LEFT  1386468383UL
//#define IR_B_RIGHT 553536955UL

#define IR_A_5     0x12345678UL // button 5 instead of OK
//#define IR_B_OK    0x87654321UL

#define IR_SAFETY_TIMEOUT  300
#define IR_TURN_DURATION   300    // ~90 degrees

// ===== Globals =====
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
IRrecv irrecv(PIN_IR_RECV);
decode_results irResults;

bool BoostorNot = false; // shows whether the boost mode is activated

enum State { STOPPED, FORWARD, BACKWARD, LEFT, RIGHT };
State state = STOPPED;
unsigned long lastIRTime = 0;
unsigned long turnStartTime = 0;
unsigned long lastCode = 0; // remember last non-REPEAT IR code

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
  // apply 75% scaling to requested speed
  uint8_t scaledSpeed;
  if (BoostorNot) {
    // Boost: volle Leistung
    scaledSpeed = speed;              // 100 % von speed
    // oder wenn du wirklich "doppelt" willst:
    // scaledSpeed = min((uint16_t)speed * 2, 255);
  } else {
    // Normal: 75 % Leistung
    scaledSpeed = (uint8_t)((uint16_t)speed * MOTOR_SCALE_NUM / MOTOR_SCALE_DEN);
  }
  if (s == FORWARD) {
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMA, scaledSpeed);
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMB, scaledSpeed);
  } else if (s == BACKWARD) {
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    analogWrite(PIN_MOTOR_PWMA, scaledSpeed);
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    analogWrite(PIN_MOTOR_PWMB, scaledSpeed);
  } else if (s == LEFT) {
    // Left turn: right wheels forward, left wheels backward
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMA, scaledSpeed);
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    analogWrite(PIN_MOTOR_PWMB, scaledSpeed);
  } else if (s == RIGHT) {
    // Right turn: right wheels backward, left wheels forward
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    analogWrite(PIN_MOTOR_PWMA, scaledSpeed);
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMB, scaledSpeed);
  }
}

// ===== OLED =====
void displayUpdate(State s) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  switch (s) {
    case FORWARD:  u8g2.drawStr(12, 30, "FORWARD");  break;
    case BACKWARD: u8g2.drawStr(12, 30, "REVERSE");  break;
    case LEFT:     u8g2.drawStr(20, 30, "LEFT");     break;
    case RIGHT:    u8g2.drawStr(16, 30, "RIGHT");    break;
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
    unsigned long code = irResults.value;
    State newState = state;
    if (code == REPEAT) {
      // keep moving forward/backward only if the last non-repeat was up/down
      if (lastCode == IR_A_2)
        newState = FORWARD;
      else if (lastCode == IR_A_8)
        newState = BACKWARD;
      //else
        //newState = (state == LEFT || state == RIGHT) ? state : STOPPED;
    } else {
      // store last non-repeat code and act on it
      lastCode = code;
      if (code == IR_A_2)
        newState = FORWARD;
      else if (code == IR_A_8)
        newState = BACKWARD;
      //else if (code == IR_A_LEFT || code == IR_B_LEFT)
        //newState = LEFT;
      //else if (code == IR_A_RIGHT || code == IR_B_RIGHT)
       // newState = RIGHT;
      else if (code == IR_A_5 || BoostorNot == true)
        // button 5 was pressed
        BoostorNot = false;
      else if (code == IR_A_5 || BoostorNot == false) {
        // button 5 was pressed
        // hier deine Aktion einfügen
        BoostorNot = true;
      } else {
        newState = STOPPED;
      }

      
    }
    if (newState != state) {
      state = newState;
      if (state == LEFT || state == RIGHT) {
        turnStartTime = millis();
      }
      motorApply(state, MOTOR_SPEED);
      displayUpdate(state);
    }
    irrecv.resume();
  }

  // Auto-stop turn after duration
  if ((state == LEFT || state == RIGHT) && millis() - turnStartTime > IR_TURN_DURATION) {
    state = STOPPED;
    motorApply(state, 0);
    displayUpdate(state);
  }

  // Safety timeout for forward/backward
  if ((state == FORWARD || state == BACKWARD) && millis() - lastIRTime > IR_SAFETY_TIMEOUT) {
    state = STOPPED;
    motorApply(state, 0);
    displayUpdate(state);
  }
}
