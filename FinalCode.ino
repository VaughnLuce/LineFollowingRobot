/*******************************************************************************************************
FAST LINE FOLLOWER - FULL CONSISTENCY FIXED VERSION
Fixes: direction inversion, recovery mismatch, constant-speed racing
*******************************************************************************************************/

#include <QTRSensors.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// --- Pins ---
#define SERVO_PIN 9
#define ENA 5
#define IN1 10
#define IN2 8
#define PIXEL_PIN A0
#define BUTTON_START 7
#define BUTTON_MODE 6

// --- Constants ---
const uint8_t SensorCount = 5;
const uint16_t STangle = 1350;

// --- Sensors ---
uint16_t sensorValues[SensorCount];

// --- PD tuning ---
int32_t Kp_scaled = 70;
int32_t Kd_scaled = 1500;
int lastError = 0;
int DEADBAND = 35;

// --- Speed (constant, no slowdown) ---
int target_speed = 255;
int min_speed = 230;

// --- Objects ---
Servo myservo;
QTRSensors qtr;
Adafruit_NeoPixel strip(1, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

enum State { STOPPED, CALIBRATE, RACE, HILL_CLIMB, TUG_OF_WAR, STANDBY };
State currentState = STOPPED;
State selectedMode = RACE;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  pinMode(BUTTON_START, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A5, A4, A3, A2, A1}, SensorCount);

  myservo.attach(SERVO_PIN);
  myservo.writeMicroseconds(STangle);

  strip.begin();
  strip.setBrightness(50);
  setLED(255, 0, 0);
}

void loop() {
  switch (currentState) {

    case STOPPED:
      setLED(255, 0, 0);
      analogWrite(ENA, 0);
      myservo.writeMicroseconds(STangle);

      if (digitalRead(BUTTON_MODE) == LOW) { delay(250); cycleSelection(); }
      if (digitalRead(BUTTON_START) == LOW) { delay(250); currentState = CALIBRATE; }
      break;

    case CALIBRATE:
      setLED(50, 50, 50);
      for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
      currentState = STANDBY;
      break;

    case STANDBY:
      setLED(100, 255, 100);
      if (digitalRead(BUTTON_START) == LOW) { delay(250); currentState = selectedMode; }
      break;

    case RACE:
      setLED(0, 255, 0);
      fastLineFollow(target_speed);
      if (digitalRead(BUTTON_START) == LOW) { delay(250); currentState = STOPPED; }
      break;

    case HILL_CLIMB:
      setLED(255, 0, 255);
      fastLineFollow(255);
      if (digitalRead(BUTTON_START) == LOW) { delay(250); currentState = STOPPED; }
      break;

    case TUG_OF_WAR:
      setLED(0, 0, 255);
      analogWrite(ENA, 255);
      myservo.writeMicroseconds(STangle);
      if (digitalRead(BUTTON_START) == LOW) { delay(250); currentState = STOPPED; }
      break;
  }
}

void fastLineFollow(int speed) {

  uint16_t position = qtr.readLineBlack(sensorValues);

  long sensorSum = 0;
  for (uint8_t i = 0; i < SensorCount; i++) sensorSum += sensorValues[i];

  // =====================================================
  // RECOVERY MODE (CONSISTENT WITH PD DIRECTION)
  // =====================================================
  if (position <= 50) {
    // line is LEFT → turn LEFT
    myservo.writeMicroseconds(STangle - 350);
    analogWrite(ENA, 215);
    lastError = -2000;
    return;
  }
  else if (position >= 3950) {
    // line is RIGHT → turn RIGHT
    myservo.writeMicroseconds(STangle + 350);
    analogWrite(ENA, 215);
    lastError = 2000;
    return;
  }

  // =====================================================
  // PD CONTROL (FIXED SIGN CONVENTION)
  // =====================================================
  if (sensorSum > 250) {

    int error = (int)position - 2000;   // ✔ FIXED SIGN SYSTEM

    if (abs(error) < DEADBAND) error = 0;

    int derivative = error - lastError;

    int32_t adjustment =
      ((Kp_scaled * (int32_t)error) +
      (Kd_scaled * (int32_t)derivative)) / 500;

    int32_t turn_signal = STangle + adjustment;

    myservo.writeMicroseconds(constrain(turn_signal, 950, 1850));

    // CONSTANT SPEED (NO CORNER SLOWING)
    analogWrite(ENA, constrain(speed, min_speed, 255));

    lastError = error;
  }
  else {
    analogWrite(ENA, min_speed);
  }
}

void cycleSelection() {
  if (selectedMode == RACE) selectedMode = HILL_CLIMB;
  else if (selectedMode == HILL_CLIMB) selectedMode = TUG_OF_WAR;
  else selectedMode = RACE;

  strip.setPixelColor(0, 0, 0, 0);
  strip.show();
  delay(100);
}

void setLED(int r, int g, int b) {
  static int lastR, lastG, lastB;

  if (r != lastR || g != lastG || b != lastB) {
    strip.setPixelColor(0, r, g, b);
    strip.show();
    lastR = r; lastG = g; lastB = b;
  }
}