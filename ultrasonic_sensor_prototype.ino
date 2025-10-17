// Pin definitions
#define TRIG_PIN 2  // Trigger pin connected to Arduino digital pin 2
#define ECHO_PIN 3  // Echo pin connected to Arduino digital pin 3

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Send 10us trigger pulse
  digitalWrite(TRIG_PIN, LOW); //Clear pin to remove noise
  delayMicroseconds(2); // Ensure a clean LOW
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); // 10us HIGH pulse
  digitalWrite(TRIG_PIN, LOW); //Pulse over

  // Measure the length of the echo pulse in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 50000); // 50ms timeout

  // Calculate distance (speed of sound = 343 m/s)
  float distance = (duration * 0.0343) / 2.0; // in centimeters

  // Print distance to serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(60); // 60ms between measurements (lab requirement)
}
