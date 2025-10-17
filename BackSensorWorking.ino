#include <DueTimer.h>

// ---------------- Sensor Setup ----------------
int rsensor = A2;   // Front right
int msensor = A1;   // Front middle
int lsensor = A0;   // Front left
int bmsensor = A3;  // Rear middle sensor

// Adjust after calibration
const int blacklevl = 900;   // threshold for detecting black line
const int backblacklevl = 1000;

// ---------------- Motor Setup ----------------
const int leftDirPins[]  = {2, 4};
const int leftEnPins[]   = {3, 5};
const int rightDirPins[] = {6, 8};
const int rightEnPins[]  = {7, 9};

const bool leftInvert[]  = {true, true};
const bool rightInvert[] = {false, false};

const int numLeft  = 2;
const int numRight = 2;

// ---------------- Control State ----------------
volatile bool lineDetected = false;
volatile int lineSide = 0; // -1=left, 0=front, +1=right, 99=rear

unsigned long lastActionTime = 0;
int currentAction = 0; // 0=forward, 1=backward, 2=right, 3=left, 4=rear detected

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);

  // Setup motors
  for (int i = 0; i < numLeft; i++) {
    pinMode(leftDirPins[i], OUTPUT);
    pinMode(leftEnPins[i], OUTPUT);
  }
  for (int i = 0; i < numRight; i++) {
    pinMode(rightDirPins[i], OUTPUT);
    pinMode(rightEnPins[i], OUTPUT);
  }

  // Set PWM resolution for Due (valid)
  analogWriteResolution(8);

  // Start timer interrupt — check sensors every 1 ms (1000 µs)
  Timer3.attachInterrupt(sensorCheck).start(1000);
}

// ---------------- Main Loop ----------------
void loop() {
  if (lineDetected) {
    if (lineSide == 0) {         // Black in front
      backward();
      currentAction = 1;
    } 
    else if (lineSide < 0) {     // Black left
      turnright();
      currentAction = 2;
    } 
    else if (lineSide > 0 && lineSide != 99) {  // Black right
      turnleft();
      currentAction = 3;
    } 
    else if (lineSide == 99) {   // Black detected behind
      goforward();
      currentAction = 4;
    }

    lastActionTime = millis();
    lineDetected = false;
  }

  // Resume forward after 400 ms of correction
  if (millis() - lastActionTime > 400 && currentAction != 0) {
    goforward();
    currentAction = 0;
  }
}

// ---------------- Sensor ISR ----------------
void sensorCheck() {
  // ---- FRONT SENSORS ----
  int leftVal  = analogRead(lsensor);
  int midVal   = analogRead(msensor);
  int rightVal = analogRead(rsensor);

  // ---- REAR SENSOR ----
  int backVal  = analogRead(bmsensor);

  bool frontBlack = (leftVal > blacklevl || midVal > blacklevl || rightVal > blacklevl);
  bool rearBlack  = (backVal > backblacklevl);

  if (frontBlack) {
    lineDetected = true;

    // Determine where the black is strongest
    if (midVal > blacklevl)
      lineSide = 0;   // center
    else if (leftVal > blacklevl && rightVal > blacklevl)
      lineSide = 0;   // both sides = ahead
    else if (leftVal > blacklevl)
      lineSide = -1;  // left
    else if (rightVal > blacklevl)
      lineSide = 1;   // right
    else
      lineSide = 0;
  } 
  else if (rearBlack) {
    lineDetected = true;
    lineSide = 99; // special code for rear detection
  }
}

// ---------------- Motor Control ----------------
void setDrive(int leftSpeed, int rightSpeed) {
  setMotorGroup(leftDirPins, leftEnPins, leftInvert, numLeft, leftSpeed);
  setMotorGroup(rightDirPins, rightEnPins, rightInvert, numRight, rightSpeed);
}

void setMotorGroup(const int dirPins[], const int enPins[], const bool invert[], int numMotors, int speed) {
  for (int i = 0; i < numMotors; i++) {
    int actualSpeed = invert[i] ? -speed : speed;
    if (actualSpeed >= 0) {
      digitalWrite(dirPins[i], HIGH);
      analogWrite(enPins[i], actualSpeed);
    } else {
      digitalWrite(dirPins[i], LOW);
      analogWrite(enPins[i], -actualSpeed);
    }
  }
}

// ---------------- Movement Functions ----------------
void goforward() { setDrive(120, 120); }
void backward()  { setDrive(-150, -150); }
void turnright() { setDrive(220, 0); }
void turnleft()  { setDrive(0, 220); }
void stopMotors() { setDrive(0, 0); }
