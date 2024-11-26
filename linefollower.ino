#include <Servo.h>

// Motor pins
#define MOTOR1_ENA 3
#define MOTOR1_IN1 5
#define MOTOR1_IN2 12
#define MOTOR2_ENB 11
#define MOTOR2_IN3 4
#define MOTOR2_IN4 6

// Infrared sensor pins
#define LEFT_IR A0
#define RIGHT_IR A1

// Servo pin
#define SERVO_PIN 8

// Ultrasonic sensor pins
#define TRIGGER_PIN 10
#define ECHO_PIN 9

// Servo object
Servo servo;

void setup() {
  // Motor pins as outputs
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);

  // Infrared sensor pins as inputs
  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);

  // Servo pin as output
  servo.attach(SERVO_PIN);

  // Ultrasonic sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Line following
  if (digitalRead(LEFT_IR == 0) && digitalRead(RIGHT_IR == 0)) {
    // Move forward
    moveForward();
  } else if (digitalRead(LEFT_IR == 0) && digitalRead(RIGHT_IR == 1)) {
    // Turn slightly right
    turnRight();
  } else if (digitalRead(LEFT_IR == 1) && !digitalRead(RIGHT_IR == 0)) {
    // Turn slightly left
    turnLeft();
  } else {
    // Stop
    stopMoving();
  }

  // Obstacle detection
  long duration, distance;
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1;

  if (distance < 15) { // Adjust this distance according to your requirement
    // Obstacle detected, move servo to scan
    servo.write(90); // Move servo to center position
    delay(500);
    turnLeft();
    turnLeft();  
    
    // Add code here to handle obstacle avoidance
  }
}

// Functions to control the motors
void moveForward() {
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3,  LOW);
  digitalWrite(MOTOR2_IN4, HIGH);
  analogWrite(MOTOR1_ENA, 150);
  analogWrite(MOTOR2_ENB, 150);
}

void turnLeft() {
  digitalWrite(MOTOR1_IN1, HIGH);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR1_ENA, 200);
  analogWrite(MOTOR2_ENB, 200);
}

void turnRight() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, HIGH);
  analogWrite(MOTOR1_ENA, 200);
  analogWrite(MOTOR2_ENB, 200);
}

void stopMoving() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN3, LOW);
  digitalWrite(MOTOR2_IN4, LOW);
  analogWrite(MOTOR1_ENA, 0);
  analogWrite(MOTOR2_ENB, 0);
}
