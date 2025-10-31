#include <DueTimer.h>

// ---------------- Sensor Setup ----------------
int rsensor = A2;   // Front right
int msensor = A1;   // Front middle
int lsensor = A0;   // Front left
int bmsensor = A3;  // Rear middle sensor

int obstacleSensor = 49;

// Adjust after calibration
const int blacklevl = 800;   // threshold for detecting black line
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

volatile int obstacleDetected = false;

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);
  
  // Enable watchdog timer to detect if program hangs
  WDT_Enable(WDT, 0x2000 | 0x20); // Enable with roughly 2-second timeout
  
  // Setup motors
  for (int i = 0; i < numLeft; i++) {
    pinMode(leftDirPins[i], OUTPUT);
    pinMode(leftEnPins[i], OUTPUT);
  }
  for (int i = 0; i < numRight; i++) {
    pinMode(rightDirPins[i], OUTPUT);
    pinMode(rightEnPins[i], OUTPUT);
  }

  // Setup sensors
  pinMode(obstacleSensor, INPUT);

  // Set PWM resolution for Due (valid)
  analogWriteResolution(8);

  // Start timer interrupt
  Timer3.attachInterrupt(sensorCheck).start(1000);
}

// ---------------- Main Loop ----------------
void loop() {
  // Reset watchdog timer
  WDT_Restart(WDT);
  
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
      spinleft();
      currentAction = 4;
    }

    lastActionTime = millis();
    lineDetected = false;
  }

  // Resume forward after 400 ms of correction
  if (millis() - lastActionTime > 400 && currentAction != 0) {
    if(obstacleDetected) {
      goforward();
      currentAction = 0;
    }
    else{
      spinright();
    }
  }
}

// ---------------- Sensor ISR ----------------
// Combined into sensorCheck()
void obstacleCheck() {}

// Combined sensor check function
void sensorCheck() {
  // Check line sensors
  int leftVal  = analogRead(lsensor);
  int midVal   = analogRead(msensor);
  int rightVal = analogRead(rsensor);
  int backVal  = analogRead(bmsensor);

  bool frontBlack = (leftVal >= blacklevl || midVal >= blacklevl || rightVal >= blacklevl);
  bool rearBlack  = (backVal > backblacklevl);

  if (frontBlack) {
    lineDetected = true;

    // Determine where the black is strongest
    if (midVal >= blacklevl)
      lineSide = 0;   // center
    else if (leftVal >= blacklevl && rightVal >= blacklevl)
      lineSide = 0;   // both sides = ahead
    else if (leftVal >= blacklevl)
      lineSide = -1;  // left
    else if (rightVal >= blacklevl)
      lineSide = 1;   // right
    else
      lineSide = 0;
  } 
  // else if (rearBlack) {
  //   lineDetected = true;
  //   lineSide = 99; // special code for rear detection
  // }

  // Check obstacle sensor
  static unsigned long lastPrint = 0;
  obstacleDetected = !digitalRead(obstacleSensor);
  
  // // Only print every 100ms to prevent serial buffer overflow
  // if (millis() - lastPrint >= 1000) {
  //   Serial.println("Front Left: ");
  //   Serial.println(leftVal);
  //   Serial.println("Front Right: ");
  //   Serial.println(rightVal);
  //   lastPrint = millis();
  // }
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
void spinright() { setDrive(150,-150); }
void spinleft() { setDrive(-150,150); }
void stopMotors() { setDrive(0, 0); }
