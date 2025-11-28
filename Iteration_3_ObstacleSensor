#include <DueTimer.h>

// ---------------- Sensor Setup ----------------
int rsensor = A2;   // Front right
int msensor = A1;   // Front middle
int lsensor = A0;   // Front left
int bmsensor = A3;  // Rear middle sensor

int obstacleSensor = 49;

int trig = 50;
int echo = A7;

const int blacklevl = 850;   // threshold for detecting black line
const int backblacklevl = 1100;

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

unsigned long lastLineDetected = 0;
int currentAction = 0; // 0=forward, 1=backward, 2=right, 3=left, 4=rear detected

volatile int obstacleDetected = false;

volatile float USSdistance = 0.0;
float distancethresh = 9.0;
volatile int USSDetected = false;
float duration;

volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool waitingForEcho = false;
volatile bool measuringEcho = false;

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
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

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

    lastLineDetected = millis();
    lineDetected = false;
  }

  // Resume forward after ms of correction
  if (millis() - lastLineDetected > 600 && currentAction != 0) {
    if(!obstacleDetected) {
      spinleft();
    }
    else if(obstacleDetected && !USSDetected){ // Sees cube and not obstacle
      goforward();
      currentAction = 0;
    }
    else {spinright();}
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
  obstacleDetected = !digitalRead(obstacleSensor);

  // ---------------- Ultrasonic Non-blocking ----------------
  unsigned long nowMicros = micros();

  // Step 1: If we are not waiting or measuring, start a new ping every ~50ms
  static unsigned long lastPing = 0;
  if (!waitingForEcho && !measuringEcho && (millis() - lastPing > 50)) {
    digitalWrite(trig, HIGH);
    // we can't use delayMicroseconds(10), so schedule turn-off after 10us
    waitingForEcho = true;
    echoStart = nowMicros;  // reuse variable for timing pulse
    lastPing = millis();
  }

  // Step 2: End trigger pulse after 10us
  if (waitingForEcho && (nowMicros - echoStart >= 10)) {
    digitalWrite(trig, LOW);
    waitingForEcho = false;
    measuringEcho = true;
    echoStart = 0;
    echoEnd = 0;
  }

  // Step 3: While measuring, watch for echo pin transitions
  if (measuringEcho) {
    int echoState = digitalRead(echo);
    if (echoState == HIGH && echoStart == 0) {
      echoStart = nowMicros; // rising edge
    } else if (echoState == LOW && echoStart != 0 && echoEnd == 0) {
      echoEnd = nowMicros;   // falling edge

      unsigned long duration = echoEnd - echoStart;
      USSdistance = (duration * 0.0343) / 2.0;  // cm
      USSDetected = (USSdistance > 0 && USSdistance < distancethresh);

      measuringEcho = false; // done
    }

    // timeout safety (in case echo never returns)
    if (measuringEcho && (nowMicros - echoStart > 30000)) { // 30ms timeout
      measuringEcho = false;
      USSDetected = false;
    }
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
void spinright() { setDrive(150,-150); }
void spinleft() { setDrive(-150,150); }
void stopMotors() { setDrive(0, 0); }
