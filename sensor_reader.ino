#include <SoftwareSerial.h>

// GPS Module
SoftwareSerial gpsSerial(2, 3); // RX, TX

// Ultrasonic Sensor
const int trigPin = 9;
const int echoPin = 10;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  
  // Ultrasonic setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Read and forward GPS data
  if (gpsSerial.available()) {
    Serial.write(gpsSerial.read());
  }
  
  // Read ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;
  
  // Send distance to Python
  Serial.println(distance);
  
  delay(500);
}