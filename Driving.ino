// Motor pin definitions
// Left wheels: M1 (front), M2 (back)
const int leftDirPins[]  = {2, 4};  // DIR pins
const int leftEnPins[]   = {3, 5};  // EN pins

// Right wheels: M3 (front), M4 (back)
const int rightDirPins[] = {6, 8};
const int rightEnPins[]  = {7, 9};

// Invert flags if any motor is mounted backwards
const bool leftInvert[]  = {true, true};  // set true if a motor spins backward
const bool rightInvert[] = {false, false};

const int numLeft  = 2;
const int numRight = 2;

void setup() {
  // Initialize pins
  for(int i = 0; i < numLeft; i++){
    pinMode(leftDirPins[i], OUTPUT);
    pinMode(leftEnPins[i], OUTPUT);
  }
  for(int i = 0; i < numRight; i++){
    pinMode(rightDirPins[i], OUTPUT);
    pinMode(rightEnPins[i], OUTPUT);
  }
}

// Set a group of motors (handles inversion and speed)
void setMotorGroup(const int dirPins[], const int enPins[], const bool invert[], int numMotors, int speed){
  for(int i = 0; i < numMotors; i++){
    int actualSpeed = speed;
    if(invert[i]) actualSpeed = -speed;

    if(actualSpeed >= 0){
      digitalWrite(dirPins[i], HIGH);
      analogWrite(enPins[i], actualSpeed);
    } else {
      digitalWrite(dirPins[i], LOW);
      analogWrite(enPins[i], -actualSpeed);
    }
  }
}

// Set the left and right wheel speeds
// leftSpeed, rightSpeed: -255 to 255
void setDrive(int leftSpeed, int rightSpeed){
  setMotorGroup(leftDirPins, leftEnPins, leftInvert, numLeft, leftSpeed);
  setMotorGroup(rightDirPins, rightEnPins, rightInvert, numRight, rightSpeed);
}

void stop() {
  setMotorGroup(leftDirPins, leftEnPins, leftInvert, numLeft, 0);
  setMotorGroup(rightDirPins, rightEnPins, rightInvert, numRight, 0);
}

void spinRight(int speed) {
  setMotorGroup(leftDirPins, leftEnPins, leftInvert, numLeft, speed);
  setMotorGroup(rightDirPins, rightEnPins, rightInvert, numRight, 0);

}

void spinLeft(int speed) {
  setMotorGroup(leftDirPins, leftEnPins, leftInvert, numLeft, 0);
  setMotorGroup(rightDirPins, rightEnPins, rightInvert, numRight, speed);

}

void loop() {
  // Example: ramp forward
  for(int i = 0; i <= 255; i+=5){ // 2.2 Seconds to reach max speed
    setDrive(i, i);   // move straight forward
    delay(100);
  }
  delay(3000);
  setDrive(255, 0);
  delay(2000);

  delay(3000);
  stop();
  delay(2000);
  
}
