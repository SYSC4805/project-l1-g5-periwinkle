#include <DueTimer.h>

int rsensor = A2;            
int msensor = A1;            
int lsensor = A0;            

const int whitelevl = 760;  
const int blacklevl = 900;  

const int leftDirPins[]  = {2, 4};
const int leftEnPins[]   = {3, 5};
const int rightDirPins[] = {6, 8};
const int rightEnPins[]  = {7, 9};

const bool leftInvert[]  = {true, true};
const bool rightInvert[] = {false, false};

const int numLeft  = 2;
const int numRight = 2;

volatile bool lineDetected = false;
volatile int lineSide = 0; // -1 = left, 0 = middle, +1 = right

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Line-Avoiding Rover with DueTimer...");

  for (int i = 0; i < numLeft; i++) {
    pinMode(leftDirPins[i], OUTPUT);
    pinMode(leftEnPins[i], OUTPUT);
  }
  for (int i = 0; i < numRight; i++) {
    pinMode(rightDirPins[i], OUTPUT);
    pinMode(rightEnPins[i], OUTPUT);
  }

  Timer3.attachInterrupt(sensorCheck).start(10000); // Trigger every 10 ms
}

void loop() {
  if (lineDetected) {
    if (lineSide == 0) {
      Serial.println("Black ahead! Backward");
      backward();
      delay(750);
    } else if (lineSide < 0) {
      Serial.println("Black LEFT! Turn right");
      turnright();
      delay(350);
    } else if (lineSide > 0) {
      Serial.println("Black RIGHT! Turn left");
      turnleft();
      delay(350);
    }
    lineDetected = false; // Reset flag
  } else {
    goforward();
  }
}

void sensorCheck() {
  int leftVal = analogRead(lsensor);
  int midVal  = analogRead(msensor);
  int rightVal = analogRead(rsensor);

  if (leftVal > blacklevl || midVal > blacklevl || rightVal > blacklevl) {
    lineDetected = true;
    if (midVal > blacklevl && leftVal < blacklevl && rightVal < blacklevl) {
      lineSide = 0;   // black only in middle
    } else if (leftVal > blacklevl && rightVal < blacklevl) {
      lineSide = -1;  // black more on left
    } else if (rightVal > blacklevl && leftVal < blacklevl) {
      lineSide = 1;   // black more on right
    } else if (leftVal > blacklevl && rightVal > blacklevl) {
      lineSide = 0;   // black on both sides — treat as middle
    } else {
      lineSide = 0;   // fallback
    }
  }
}

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

void goforward() { setDrive(120, 120); }
void backward()  { setDrive(-150, -150); }
void turnright() { setDrive(200, 50); }
void turnleft()  { setDrive(50, 200); }
