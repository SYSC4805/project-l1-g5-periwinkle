#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin) {
    this->trigPin = trigPin;
    this->echoPin = echoPin;
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float UltrasonicSensor::getDistance() {
    // Send 10us trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure echo pulse duration (microseconds) with 50ms timeout
    long duration = pulseIn(echoPin, HIGH, 50000);

    // Calculate distance in centimeters (speed of sound = 343 m/s)
    float distance = (duration * 0.0343) / 2.0;

    return distance;
}
