#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>
#include <queue.h>


// ---------------- Sensor Setup ----------------
int rsensor = A2;  // Front right
int msensor = A1;  // Front middle
int lsensor = A0;  // Front left
int bmsensor = A3; // Rear middle sensor

int obstacleSensor = 49;

int trig = 50;
int echo = A7;

// Adjust after calibration
const int blacklevl = 830; // threshold for detecting front black line
const int backblacklevl = 1100;

// Ultrasonic thresholds
float distancethresh = 15.0;

// ---------------- Motor Setup ----------------
const int leftDirPins[] = {2, 4};
const int leftEnPins[] = {3, 5};
const int rightDirPins[] = {6, 8};
const int rightEnPins[] = {7, 9};

const bool leftInvert[] = {true, true};
const bool rightInvert[] = {false, false};

const int numLeft = 2;
const int numRight = 2;

// ---------------- Control State ----------------
volatile bool lineDetected = false;
volatile int lineSide = 0; // -1=left, 0=front, +1=right, 99=rear

unsigned long lastLineDetected = 0;
int currentAction = 0; // 0=forward, 1=backward, 2=right, 3=left, 4=rear detected

volatile int obstacleDetected = false;
volatile float USSdistance = 0.0;
volatile int USSDetected = false;

// ----------------FreeRTOS objects----------------
enum DriveCmd
{
    CMD_FWD,
    CMD_BACK,
    CMD_TURN_L,
    CMD_TURN_R,
    CMD_STOP,
    BACK_RIGHT,
    BACK_LEFT
};

QueueHandle_t motorQueue;
EventGroupHandle_t zoneEvents;

// EventGroup bits
#define EVENT_INSIDE (1 << 0)
#define EVENT_OUTSIDE (1 << 1)

// ----------------Function Prototypes ----------------
void lineTask(void *pv);
void obstacleTask(void *pv);
void motorTask(void *pv);

void setDrive(int leftSpeed, int rightSpeed);
void setMotorGroup(const int dirPins[], const int enPins[], const bool invert[], int numMotors, int speed);

void goforward();
void backward();
void turnright();
void turnleft();
void spinright();
void spinleft();
void backRight();
void backLeft();
void stopMotors();

// ----------------Setup ----------------
void setup()
{
    Serial.begin(9600);

    // Enable watchdog timer to detect if program hangs
    WDT_Enable(WDT, 0x2000 | 0x20); // Enable with roughly 2-second timeout

    // Setup motors
    for (int i = 0; i < numLeft; i++)
    {
        pinMode(leftDirPins[i], OUTPUT);
        pinMode(leftEnPins[i], OUTPUT);
    }
    for (int i = 0; i < numRight; i++)
    {
        pinMode(rightDirPins[i], OUTPUT);
        pinMode(rightEnPins[i], OUTPUT);
    }

    // Setup sensors
    pinMode(obstacleSensor, INPUT);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    // Set PWM resolution for Due (valid)
    analogWriteResolution(8);

    // Create queue and event group
    motorQueue = xQueueCreate(10, sizeof(DriveCmd));
    zoneEvents = xEventGroupCreate();

    // Create FreeRTOS tasks 
    xTaskCreate(lineTask, "LINE", 256, NULL, 3, NULL);
    xTaskCreate(obstacleTask, "OBS", 384, NULL, 2, NULL);
    xTaskCreate(motorTask, "MOTOR", 384, NULL, 1, NULL);

    // Start scheduler
    vTaskStartScheduler();
}

// ---------------- Main Loop ----------------
void loop()
{
    // No code to run in a loop, FreeRTOS handles it
}

// Reads front and back line sensors, sets EventGroup (inside/outside)
void lineTask(void *pv) {
  (void) pv;

  for (;;) {
    int leftVal  = analogRead(lsensor);
    int midVal   = analogRead(msensor);
    int rightVal = analogRead(rsensor);
    int backVal  = analogRead(bmsensor);

    DriveCmd cmd = CMD_FWD;
    EventBits_t bits = EVENT_INSIDE;

    bool frontBlack = (leftVal >= blacklevl || midVal >= blacklevl || rightVal >= blacklevl);
    bool rearBlack  = (backVal >= backblacklevl);

    if (frontBlack) {
      // boundary ahead/front
      if (midVal >= blacklevl || (leftVal >= blacklevl && rightVal >= blacklevl)) {
        cmd = CMD_BACK;
        bits = EVENT_OUTSIDE;
        lineSide = 0;
      } 
      else if (midVal >= blacklevl && (leftVal >= blacklevl && rightVal < blacklevl)) {
        cmd = BACK_RIGHT;
        bits = EVENT_OUTSIDE;
        lineSide = -2;
      } 
      else if (midVal >= blacklevl && (rightVal >= blacklevl && leftVal < blacklevl)) {
        cmd = BACK_LEFT;
        bits = EVENT_OUTSIDE;
        lineSide = 2;
      } 
      else if (rightVal >= blacklevl) {
        cmd = CMD_TURN_R;
        bits = EVENT_OUTSIDE;
        lineSide = -1;
      } else if (leftVal >= blacklevl) {
        cmd = CMD_TURN_L;
        bits = EVENT_OUTSIDE;
        lineSide = 1;
      } else {
        cmd = CMD_BACK;
        bits = EVENT_OUTSIDE;
        lineSide = 0;
      }
      lineDetected = true;
      lastLineDetected = millis();
    // }  else if (rearBlack) {
    //   // rear boundary detected
    //   cmd = CMD_STOP; // we'll use STOP as placeholder; motor task will do spinleft for rear
    //   bits = EVENT_OUTSIDE;
    //   lineDetected = true;
    //   lineSide = 99;
    //   lastLineDetected = millis();
    } else {
      // inside safe area
      cmd = CMD_FWD;
      bits = EVENT_INSIDE;
      lineDetected = false;
      lineSide = 0;
    }
    
    // clear 
    xEventGroupClearBits(zoneEvents, EVENT_INSIDE | EVENT_OUTSIDE);

    // publish event & command
    xEventGroupSetBits(zoneEvents, bits);
    xQueueSend(motorQueue, &cmd, 0);

    vTaskDelay(pdMS_TO_TICKS(10)); // about 10ms loop 
  }
}

// ---------------- obstacleTask ----------------
// Reads IR obstacle and HC-SR04 ultrasonic sensors
// updates global flags and may post commands
void obstacleTask(void *pv) {
  (void) pv;

  for (;;) {
    // Check IR obstacle sensor
    obstacleDetected = !digitalRead(obstacleSensor);

    // Trigger ultrasonic
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // pulseIn with timeout 30000 us (30 ms)
    unsigned long duration = pulseIn(echo, HIGH, 30000UL);
    if (duration == 0) {
      USSdistance = 0.0;
      USSDetected = false;
    } else {
      float distance_cm = (duration * 0.0343f) / 2.0f;
      USSdistance = distance_cm;
      USSDetected = (distance_cm > 0.0f && distance_cm < distancethresh);
    }

    // If ultrasonic detects something very near, post a BACK command to the motor queue
    if (USSDetected) {
      DriveCmd cmd = CMD_BACK;
      xQueueSend(motorQueue, &cmd, 0);
    } else if (obstacleDetected) {
      // If digital obstacle sensor detects cube, want to push it forward
      DriveCmd cmd = CMD_FWD;
      xQueueSend(motorQueue, &cmd, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(15)); // poll ultrasonic at 15 ms
  }
}

// ---------------- motorTask ----------------
// Receives DriveCmd from queue and applies motor outputs,
// using EventGroup and USS/obstacle flags
// Also restarts watchdog to avoid reset
void motorTask(void *pv) {
  (void) pv;
  DriveCmd cmd;
  EventBits_t zone;

  for (;;) {
    // Wait for a command 
    if (xQueueReceive(motorQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      // Get zone state
      zone = xEventGroupGetBits(zoneEvents);

      // 1) If outside: override to back
     if (zone & EVENT_OUTSIDE) {
        if (lineSide == -1) { 
            turnright(); 
        } 
        else if (lineSide == 1) { 
            turnleft(); 
        } 
        else if (lineSide == 99) { 
            spinleft();
        } 
        else if (lineSide == -2){ 
            backRight(); 
        }
        else if (lineSide == 2){ 
            backLeft(); 
        }
        else {
          backward();
        }
      } else {
        // 2) If ultrasonic too close: back up
        if (USSDetected) {
          backward();
          currentAction = 1;
          USSDetected = false;
        } else {
          // 3) If digital obstacleDetected (cube present) and no USS: push forward
          if (obstacleDetected && !USSDetected) {
            goforward();
            currentAction = 0;
            obstacleDetected = false;
          } else {
            // 4) Otherwise obey command from lineTask
            switch (cmd) {
              case CMD_FWD:
                goforward();
                currentAction = 0;
                break;
              case CMD_BACK:
                backward();
                currentAction = 1;
                break;
              case CMD_TURN_L:
                turnleft();
                currentAction = 3;
                break;
              case CMD_TURN_R:
                turnright();
                currentAction = 2;
                break;
              case CMD_STOP:
                stopMotors();
                currentAction = 0;
                break;
              case BACK_RIGHT:
                backRight();
                currentAction = 1;
                break;
              case BACK_LEFT:
                backLeft();
                currentAction = 1;
                break;
              default:
                goforward();
                currentAction = 0;
                break;
            }
          }
        }
      }

      // Restart watchdog
      WDT_Restart(WDT);
    } // if queue receive

    // small yield to allow other tasks
    vTaskDelay(pdMS_TO_TICKS(5));
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
void goforward() { setDrive(255, 210); }
void backward()  { setDrive(-255, -255); }
void turnright() { setDrive(255, 0); }
void turnleft()  { setDrive(0, 255); }
void spinright() { setDrive(150,-150); }
void spinleft() { setDrive(-150,150); }
void backRight() {setDrive(-150, -255);}
void backLeft() {setDrive(-255, -150);}
void stopMotors() { setDrive(0, 0); }

