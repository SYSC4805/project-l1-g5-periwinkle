#include <Arduino.h>

// --------------- Sensor Pins (from your original code) ---------------
int rsensor = A2;   // Front right
int msensor = A1;   // Front middle
int lsensor = A0;   // Front left
int bmsensor = A3;  // Rear middle sensor

int obstacleSensor = 49;

int trig = 50;
int echo = A7;

// Thresholds from your original code
const int blacklevl = 850;
const int backblacklevl = 1100;
float distancethresh = 9.0;

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);
  delay(2000);
  Serial.println("=== SENSOR DIAGNOSTIC TEST START ===");

  pinMode(obstacleSensor, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(lsensor, INPUT);
  pinMode(msensor, INPUT);
  pinMode(rsensor, INPUT);
  pinMode(bmsensor, INPUT);
}

// ---------------- Helper Functions ----------------
float readUltrasonic() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long duration = pulseIn(echo, HIGH, 30000UL); // timeout at 30ms
  float distance = duration * 0.034 / 2.0; // cm
  return (duration == 0) ? -1.0 : distance;
}

// ---------------- Loop (Runs Tests) ----------------
void loop() {
  // ---- Line Sensors ----
  int leftVal  = analogRead(lsensor);
  int midVal   = analogRead(msensor);
  int rightVal = analogRead(rsensor);
  int backVal  = analogRead(bmsensor);

  Serial.println("\n--- Line Sensors ---");
  Serial.print("Left: "); Serial.print(leftVal);
  Serial.print(" | Mid: "); Serial.print(midVal);
  Serial.print(" | Right: "); Serial.print(rightVal);
  Serial.print(" | Rear: "); Serial.println(backVal);

  // Evaluate detection
  bool lineFront = (midVal < blacklevl || leftVal < blacklevl || rightVal < blacklevl);
  bool lineBack  = (backVal < backblacklevl);
  Serial.print("Front Line Detected: "); Serial.println(lineFront ? "YES" : "NO");
  Serial.print("Rear Line Detected: "); Serial.println(lineBack ? "YES" : "NO");

  // ---- Ultrasonic Sensor ----
  Serial.println("\n--- Ultrasonic Sensor ---");
  float distance = readUltrasonic();
  if (distance < 0)
    Serial.println("No echo detected (timeout)");
  else {
    Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
    Serial.print("Obstacle: ");
    Serial.println(distance < distancethresh ? "TOO CLOSE" : "CLEAR");
  }

  // ---- Obstacle IR Sensor ----
  Serial.println("\n--- IR Obstacle Sensor ---");
  int obstacleVal = digitalRead(obstacleSensor);
  Serial.print("Digital Value: "); Serial.println(obstacleVal);
  Serial.println(obstacleVal == HIGH ? "Obstacle Detected!" : "No Obstacle");

  Serial.println("---------------------------");
  delay(1000);
}
