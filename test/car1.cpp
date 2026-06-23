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

#define IR_A_5     16712445UL   // button 5 (OK) - typical Elegoo NEC value
//#define IR_B_OK    0x87654321UL

#define IR_SAFETY_TIMEOUT  300
#define IR_TURN_DURATION   300    // ~90 degrees

// ===== Globals =====
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
IRrecv irrecv(PIN_IR_RECV);
decode_results irResults;

bool boostActive = false; // shows whether the boost mode is activated
// Turn speed (independent of boost)
#define TURN_SPEED 150

// return the effective drive speed (honors boost and 75% scaling)
uint8_t driveSpeed() {
  if (boostActive) return MOTOR_SPEED; // full speed in boost
  return (uint8_t)((uint16_t)MOTOR_SPEED * MOTOR_SCALE_NUM / MOTOR_SCALE_DEN);
}

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
  uint8_t out = speed; // speed is already the effective PWM value
  if (s == FORWARD) {
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMA, out);
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMB, out);
  } else if (s == BACKWARD) {
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    analogWrite(PIN_MOTOR_PWMA, out);
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    analogWrite(PIN_MOTOR_PWMB, out);
  } else if (s == LEFT) {
    // Left turn: right wheels forward, left wheels backward
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMA, out);
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    analogWrite(PIN_MOTOR_PWMB, out);
  } else if (s == RIGHT) {
    // Right turn: right wheels backward, left wheels forward
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    analogWrite(PIN_MOTOR_PWMA, out);
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    analogWrite(PIN_MOTOR_PWMB, out);
  }
}

enum DriveMode { MODE_NORMAL, MODE_BOOST };
DriveMode driveMode = MODE_NORMAL;

// ===== OLED =====
void displayUpdate() {

  if (boostActive) {
    driveMode = MODE_BOOST;
  } else {
    driveMode = MODE_NORMAL;
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  switch (state) {
    case FORWARD:  u8g2.drawStr(12, 28, "FORWARD"); break;
    case BACKWARD: u8g2.drawStr(12, 28, "REVERSE"); break;
    default:       u8g2.drawStr(16, 28, "STOPPED"); break;
  }

  u8g2.drawFrame(0, 40, 128, 24);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.setCursor(4, 52);
  u8g2.print("Speed: ");
  u8g2.print(driveMode);
  if (boostActive) u8g2.drawStr(78, 52, "BOOST");

  u8g2.sendBuffer();
}

// ===== Setup =====
void setup() {
  Wire.begin();
  u8g2.begin();
  motorInit();
  irrecv.enableIRIn();
  displayUpdate();
}

// ===== Main Loop =====
void loop() {
  if (irrecv.decode(&irResults)) {
    lastIRTime = millis();
    unsigned long code = irResults.value;
    State newState = state;
    bool boostToggled = false;
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
      else if (code == IR_A_5) {
        // toggle boost
        boostActive = !boostActive;
        boostToggled = true;
      } else {
        newState = STOPPED;
      }

      
    }
    // always update motors/display if state changed or boost toggled
    if (newState != state || boostToggled) {
      state = newState;
      if (state == LEFT || state == RIGHT) {
        turnStartTime = millis();
      }
      uint8_t eff = (state == LEFT || state == RIGHT) ? TURN_SPEED : driveSpeed();
      motorApply(state, eff);
      displayUpdate();
    }
    irrecv.resume();
  }

  // Auto-stop turn after duration
  if ((state == LEFT || state == RIGHT) && millis() - turnStartTime > IR_TURN_DURATION) {
    state = STOPPED;
    motorApply(state, 0);
    displayUpdate();
  }

  // Safety timeout for forward/backward
  if ((state == FORWARD || state == BACKWARD) && millis() - lastIRTime > IR_SAFETY_TIMEOUT) {
    state = STOPPED;
    motorApply(state, 0);
    displayUpdate();
  }
}
