#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>
#include <queue.h>

// ---------------- Sensor Setup ----------------
int rsensor = A2;   // Front right
int msensor = A1;   // Front middle
int lsensor = A0;   // Front left
int bmsensor = A3;  // Rear middle sensor

int obstacleSensor = 49;


// Middle Ultrasonic Sensor
int trig_m = 50;
int echo_m = A7;

// Right Ultrasonic Sensor
int trig_r = 51;
int echo_r = A8;

// Left Ultrasonic Sensor
int trig_l = 48;
int echo_l = A9;


// Adjust after calibration
const int blacklevl = 800;  // threshold for detecting front black line
const int backblacklevl = 1100;

// Ultrasonic thresholds
float distancethresh = 16.0;

// ---------------- Motor Setup ----------------
const int leftDirPins[] = { 2, 4 };
const int leftEnPins[] = { 3, 5 };
const int rightDirPins[] = { 6, 8 };
const int rightEnPins[] = { 7, 9 };

const bool leftInvert[] = { true, true };
const bool rightInvert[] = { false, false };

const int numLeft = 2;
const int numRight = 2;

// ---------------- Control State ----------------
volatile bool lineDetected = false;
volatile int lineSide = 0;  // -1=left, 0=front, +1=right, 99=rear

unsigned long lastLineDetected = 0;
int currentAction = 0;  // 0=forward, 1=backward, 2=right, 3=left, 4=rear detected

volatile int obstacleDetected = false;

volatile float USSdistance_m = 0.0;
volatile float USSdistance_r = 0.0;
volatile float USSdistance_l = 0.0;

volatile int USSDetected_m = false;
volatile int USSDetected_r = false;
volatile int USSDetected_l = false;


// ----------------FreeRTOS objects----------------
enum DriveCmd {
  CMD_FWD,
  CMD_BACK,
  CMD_TURN_L,
  CMD_TURN_R,
  CMD_STOP,
  BACK_RIGHT,
  BACK_LEFT,
};

QueueHandle_t motorQueue;
EventGroupHandle_t zoneEvents;

// EventGroup bits
#define EVENT_INSIDE (1 << 0)
#define EVENT_OUTSIDE (1 << 1)

// ---------------- Debounce / Filter ----------------
const int LINE_CONFIRM_COUNT = 3;
int frontBlackCount = 0;
int rearBlackCount = 0;

const int USS_CONFIRM_COUNT = 2;
int ussNearCount_m = 0;
int ussNearCount_r = 0;
int ussNearCount_l = 0;


const int OBST_CONFIRM_COUNT = 2;
int obsCount = 0;

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
void setup() {
  Serial.begin(9600);

  // Enable watchdog timer to detect if program hangs
  WDT_Enable(WDT, 0x2000 | 0x20);  // Enable with roughly 2-second timeout

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
  pinMode(trig_m, OUTPUT);
  pinMode(echo_m, INPUT);
  pinMode(trig_r, OUTPUT);
  pinMode(echo_r, INPUT);
  pinMode(trig_l, OUTPUT);
  pinMode(echo_l, INPUT);

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
void loop() {
  // No code to run in a loop, FreeRTOS handles it
}

// ---------------- lineTask ----------------
void lineTask(void *pv) {
  (void)pv;

  for (;;) {
    int leftVal = analogRead(lsensor);
    int midVal = analogRead(msensor);
    int rightVal = analogRead(rsensor);
    int backVal = analogRead(bmsensor);

    bool frontBlack = (leftVal >= blacklevl || midVal >= blacklevl || rightVal >= blacklevl);
    bool rearBlack = (backVal >= backblacklevl);

    // Serial.println("Left Val ");
    // Serial.print( leftVal);
    //     Serial.println("Right Val " );
    //     Serial.print(rightVal);
      
    // Serial.println("Middle Val " );
    // Serial.print(midVal);


    // Debounce front line
    if (frontBlack) frontBlackCount++;
    else frontBlackCount = 0;

    if (rearBlack) rearBlackCount++;
    else rearBlackCount = 0;

    DriveCmd cmd = CMD_FWD;
    EventBits_t bits = EVENT_INSIDE;

    if (frontBlackCount >= LINE_CONFIRM_COUNT) {
      // Use original logic
      if (midVal >= blacklevl || (leftVal >= blacklevl && rightVal >= blacklevl)) {
        cmd = CMD_BACK;
        lineSide = 0;
      } else if (midVal >= blacklevl && (leftVal >= blacklevl && rightVal < blacklevl)) {
        cmd = BACK_RIGHT;
        lineSide = -2;
      } else if (midVal >= blacklevl && (rightVal >= blacklevl && leftVal < blacklevl)) {
        cmd = BACK_LEFT;
        lineSide = 2;
      } else if (rightVal >= blacklevl) {
        cmd = CMD_TURN_R;
        lineSide = -1;
      } else if (leftVal >= blacklevl) {
        cmd = CMD_TURN_L;
        lineSide = 1;
      } else {
        cmd = CMD_BACK;
        lineSide = 0;
      }
      bits = EVENT_OUTSIDE;
      lineDetected = true;
      lastLineDetected = millis();
    } else {
      cmd = CMD_FWD;
      bits = EVENT_INSIDE;
      lineDetected = false;
      lineSide = 0;
    }

    xEventGroupClearBits(zoneEvents, EVENT_INSIDE | EVENT_OUTSIDE);
    xEventGroupSetBits(zoneEvents, bits);
    xQueueSend(motorQueue, &cmd, 0);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------- obstacleTask ----------------
void obstacleTask(void *pv) {
  (void)pv;

  for (;;) {
    // Read IR obstacle sensor
    bool obs = !digitalRead(obstacleSensor);
    if (obs) obsCount++;
    else obsCount = 0;
    obstacleDetected = (obsCount >= OBST_CONFIRM_COUNT);

    // Trigger middle ultrasonic
    digitalWrite(trig_m, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_m, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_m, LOW);
    unsigned long duration_m = pulseIn(echo_m, HIGH, 30000UL);

    delayMicroseconds(100);

    // Trigger right ultrasonic
    digitalWrite(trig_r, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_r, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_r, LOW);
    unsigned long duration_r = pulseIn(echo_r, HIGH, 30000UL);

    // Trigger left ultrasonic
    digitalWrite(trig_l, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_l, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_l, LOW);
    unsigned long duration_l = pulseIn(echo_l, HIGH, 30000UL);


    // Read middle ultrasonic
    float distance_cm_m = 0.0;
    if (duration_m > 0) distance_cm_m = (duration_m * 0.0343f) / 2.0f;

    if (distance_cm_m > 0 && distance_cm_m < distancethresh) ussNearCount_m++;
    else ussNearCount_m = 0;
    USSDetected_m = (ussNearCount_m >= USS_CONFIRM_COUNT);
    USSdistance_m = distance_cm_m;

    // Read right ultrasonic
    float distance_cm_r = 0.0;
    if (duration_r > 0) distance_cm_r = (duration_r * 0.0343f) / 2.0f;

    if (distance_cm_r > 0 && distance_cm_r < distancethresh) ussNearCount_r++;
    else ussNearCount_r = 0;
    USSDetected_r = (ussNearCount_r >= USS_CONFIRM_COUNT);
    USSdistance_r = distance_cm_r;

    // Read left ultrasonic
    float distance_cm_l = 0.0;
    if (duration_l > 0) distance_cm_l = (duration_l * 0.0343f) / 2.0f;

    if (distance_cm_l > 0 && distance_cm_l < distancethresh) ussNearCount_l++;
    else ussNearCount_l = 0;
    USSDetected_l = (ussNearCount_l >= USS_CONFIRM_COUNT);
    USSdistance_l = distance_cm_l;

    //----------- Print debug info for ultrasonics only -----------
    // Serial.print("Middle USS: ");
    // Serial.print(distance_cm_m);
    // Serial.print(" cm | Detected: ");
    // Serial.print(USSDetected_m);
    // Serial.print(" || Right USS: ");
    // Serial.print(distance_cm_r);
    // Serial.print(" cm | Detected: ");
    // Serial.println(USSDetected_r);
    // Serial.print(" || Left USS: ");
    // Serial.print(distance_cm_l);
    // Serial.print(" cm | Detected: ");
    // Serial.println(USSDetected_l);

    vTaskDelay(pdMS_TO_TICKS(10));  // slower print rate (100 ms)
  }
}

// ---------------- motorTask ----------------
void motorTask(void *pv) {
  (void)pv;
  DriveCmd cmd;
  EventBits_t zone;

  for (;;) {
    if (xQueueReceive(motorQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      zone = xEventGroupGetBits(zoneEvents);

      if (zone & EVENT_OUTSIDE) {
        // Outside -> override
        if (lineSide == -1) turnright();
        else if (lineSide == 1) turnleft();
        else if (lineSide == 99) spinleft();
        else if (lineSide == -2) backRight();
        else if (lineSide == 2) backLeft();
        else { 
          backward();
          delay(500);
          turnright();
          delay(1000);

        }
        
      } else {
        // Ultrasonic too close
        if (USSDetected_m && USSDetected_r) {
          if (USSdistance_r < USSdistance_m) {
            turnleft();
            // Serial.print("Middle & Right USS: Turn left ");
            delay(500);

            currentAction = 1;
          } else {
            // Serial.print("Middle & Right USS: Backwards ");

            backward();
            delay(500);

            currentAction = 0;
          }
          USSDetected_r = false;
          USSDetected_m = false;
        } else if (USSDetected_r) {
          // Serial.print("Right USS: Turn left ");

          turnleft();
          delay(500);

          currentAction = 1;
          USSDetected_r = false;
        } else if (USSDetected_m) {
          // Serial.print("Middle USS: Backwards ");

          backward();
          delay(500);

          currentAction = 0;
          USSDetected_m = false;
        }

        else if (obstacleDetected && !USSDetected_m) {
          goforward();
          currentAction = 0;
          obstacleDetected = false;

        } else {
          // Obey command from lineTask
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

      WDT_Restart(WDT);
    }
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
void goforward() {
  setDrive(255, 210);
}
void backward() {
  setDrive(-255, -255);
}
void turnright() {
  setDrive(255, -255);
}
void turnleft() {
  setDrive(-255, 255);
}
void spinright() {
  setDrive(150, -150);
}
void spinleft() {
  setDrive(-150, 150);
}
void backRight() {
  setDrive(-150, -255);
}
void backLeft() {
  setDrive(-255, -150);
}
void stopMotors() {
  setDrive(0, 0);
}
