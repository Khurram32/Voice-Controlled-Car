#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>

// === Pin Definitions ===
#define ECHO_PIN A0
#define TRIG_PIN A1
#define SERVO_PIN 10

// === Constants ===
#define DEFAULT_SPEED 170
#define CENTER_SERVO 103

// === Objects ===
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);
Servo servo;

// === Globals ===
char command;
int mode = 0; // 0 = Bluetooth, 1 = Obstacle, 2 = Voice

// === Setup ===
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);
  servo.write(CENTER_SERVO);

  M1.setSpeed(DEFAULT_SPEED);
  M2.setSpeed(DEFAULT_SPEED);
  M3.setSpeed(DEFAULT_SPEED);
  M4.setSpeed(DEFAULT_SPEED);

  Serial.println("System Ready. Mode: Manual (Bluetooth)");
}

// === Loop ===
void loop() {
  if (Serial.available()) {
    command = Serial.read();
    Serial.print("Command: ");
    Serial.println(command);
    handleCommand(command);
  }

  switch (mode) {
    case 0: /* Manual mode – all handled by handleCommand */ break;
    case 1: obstacleAvoidance(); break;
    case 2: voiceControl(); break;
  }
}

// === Movement ===
void forward() {
  M1.run(FORWARD); M2.run(FORWARD);
  M3.run(FORWARD); M4.run(FORWARD);
}

void backward() {
  M1.run(BACKWARD); M2.run(BACKWARD);
  M3.run(BACKWARD); M4.run(BACKWARD);
}

void left() {
  M1.run(FORWARD); M2.run(FORWARD);
  M3.run(BACKWARD); M4.run(BACKWARD);
}

void right() {
  M1.run(BACKWARD); M2.run(BACKWARD);
  M3.run(FORWARD); M4.run(FORWARD);
}

void stopAll() {
  M1.run(RELEASE); M2.run(RELEASE);
  M3.run(RELEASE); M4.run(RELEASE);
}

void setSpeed(int speed) {
  M1.setSpeed(speed); M2.setSpeed(speed);
  M3.setSpeed(speed); M4.setSpeed(speed);
}

// === Ultrasonic Distance ===
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration / 29 / 2;
}

int scanLeft() {
  servo.write(180);
  delay(300);
  return getDistance();
}

int scanRight() {
  servo.write(20);
  delay(300);
  return getDistance();
}

void centerServo() {
  servo.write(CENTER_SERVO);
  delay(200);
}

// === Obstacle Avoidance Mode ===
void obstacleAvoidance() {
  int distance = getDistance();
  if (distance < 15) {
    stopAll();
    Serial.println("Obstacle detected!");

    backward();
    delay(300);
    stopAll();

    int leftDist = scanLeft();
    centerServo();
    int rightDist = scanRight();
    centerServo();

    if (leftDist > rightDist) {
      Serial.println("Turning Left");
      left(); delay(400);
    } else {
      Serial.println("Turning Right");
      right(); delay(400);
    }
    stopAll();
  } else {
    forward();
  }
}

// === Voice Control Mode ===
void voiceControl() {
  switch (command) {
    case '^': forward(); break;
    case '-': backward(); break;
    case '<':
      if (scanLeft() > 10) {
        left(); delay(400);
        stopAll();
      }
      break;
    case '>':
      if (scanRight() > 10) {
        right(); delay(400);
        stopAll();
      }
      break;
    case '*': stopAll(); break;
  }
}

// === Command Handler ===
void handleCommand(char cmd) {
  switch (cmd) {
    case 'M':
      mode = (mode + 1) % 3;
      Serial.print("Switched to Mode: ");
      if (mode == 0) Serial.println("Bluetooth");
      else if (mode == 1) Serial.println("Obstacle Avoidance");
      else Serial.println("Voice Control");
      break;

    case 'F': if (mode == 0) forward(); break;
    case 'B': if (mode == 0) backward(); break;
    case 'L': if (mode == 0) left(); break;
    case 'R': if (mode == 0) right(); break;
    case 'S': stopAll(); break;

    case '^': case '-': case '<': case '>': case '*':
      if (mode == 2) voiceControl(); break;
  }
}
