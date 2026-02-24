#include <QTRSensors.h>

// DRV8833 inputs 
int ain1 = 6;   
int ain2 = 9;   
int bin1 = 10;  
int bin2 = 11;  

int P;
int I = 0;
int D;

float Kp = 0.05;
float Ki = 0.00001;
float Kd = 0.8;

int lastError = 0;

int button_calibration = 12;

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

void driveMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // Forward: IN1 = PWM, IN2 = 0
    analogWrite(in1, speed);
    digitalWrite(in2, LOW);
  } 
  else if (speed < 0) {
    // Reverse: IN1 = 0, IN2 = PWM
    analogWrite(in2, -speed);
    digitalWrite(in1, LOW);
  } 
  else {
    // Stop (coast)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){2, 3, 4, 5, 7}, SensorCount);
  qtr.setEmitterPin(8);

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  pinMode(button_calibration, INPUT);

  while (digitalRead(button_calibration) == LOW) {}

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
}

void loop() {
  PID_control();
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = positionLine - 2000;

  Serial.print("pos="); Serial.print(positionLine);
  Serial.print(" err="); Serial.print(error);
  Serial.print(" | ");

  P = error;
  I = I + error;                 // accumulate
  D = error - lastError;
  lastError = error;

  int motorSpeedChange = (int)(P * Kp + I * Ki + D * Kd);

  int motorSpeedA = 100 + motorSpeedChange;
  int motorSpeedB = 100 - motorSpeedChange;

  motorSpeedA = constrain(motorSpeedA, -125, 125);
  motorSpeedB = constrain(motorSpeedB, -125, 125);

  driveMotor(ain1, ain2, motorSpeedA);
  driveMotor(bin1, bin2, motorSpeedB);

  Serial.print("speedA="); Serial.print(motorSpeedA);
  Serial.print(" speedB="); Serial.println(motorSpeedB);
}
