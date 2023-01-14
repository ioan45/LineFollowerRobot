#include <QTRSensors.h>


const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

float kp = 1;
float ki = 0;
float kd = 1;

int p = 0;
int i = 0;
int d = 0;

const float multiplierD = 0.1;  // good alternatives: 2.3, 1.3, 0.75
int counterD = 0;  // counts the no. of consecutive iterations in which the error scaling stays the same (it keeps increasing/decreasing).
int lastSignD = 1;  // remembers the sign of D in the previous loop iteration.

int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -255;
const int maxSensorValue = 500;
const int minSensorValue = -500;

const int baseSpeed = 255;

QTRSensors qtr;

const int sensorCount = 6;
unsigned int sensorValues[sensorCount];
int sensors[sensorCount] = {0, 0, 0, 0, 0, 0};

void setup() {
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  
  performCalibration();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  int pidValue = calculatePIDValue();
  int factor = getSpeedDecreasingFactor(pidValue);
  int m1Speed, m2Speed;
  calculateNewSpeeds(m1Speed, m2Speed, factor);
  setMotorSpeed(m1Speed, m2Speed);
}

void performCalibration() {
  const int motorSpeed = 225;
  const int lineOnSensor1Threshold = 250;
  const int lineOnSensor6Threshold = 4750;

  setMotorSpeed(motorSpeed, -motorSpeed);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    
    int readingValue = qtr.readLineBlack(sensorValues);
    if (readingValue <= lineOnSensor1Threshold)
      setMotorSpeed(-motorSpeed, motorSpeed);
    else if (readingValue >= lineOnSensor6Threshold)
      setMotorSpeed(motorSpeed, -motorSpeed);
  }
  setMotorSpeed(0, 0);
}

int calculatePIDValue() {
  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, minSensorValue, maxSensorValue);

  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;

  return kp * p + ki * i + kd * d;
}

int getSpeedDecreasingFactor(int fromPIDValue) {
  if (lastSignD * d > 0)  // if error keeps getting higher or keeps getting lower, increase the counter.
    counterD++;
  else                    // if error scaling changes orientation (error begins getting lower/higher), reset counter.
    counterD = 0;
  lastSignD = d < 0 ? -1 : 1;  // remember the orientation or error scaling (remember if error decreased or increased this iteration).

  int factor = fromPIDValue - multiplierD * counterD;  // decelerate more and more as error scaling stays the same way (error keeps increasing or decreasing)
  return factor;
}

void calculateNewSpeeds(int& newM1Speed, int& newM2Speed, int decreasingFactor) {
  newM1Speed = baseSpeed;
  newM2Speed = baseSpeed;

  // a bit counter intuitive because of the signs
  // basically in the first if, you substract the error from m1Speed (you add the negative)
  // in the 2nd if you add the error to m2Speed (you substract the negative)
  // it's just the way the values of the sensors and/or motors lined up
  if (error < 0) {
    newM1Speed += decreasingFactor;
  }
  else if (error > 0) {
    newM2Speed -= decreasingFactor;
  }
  newM1Speed = constrain(newM1Speed, minSpeed, maxSpeed);
  newM2Speed = constrain(newM2Speed, minSpeed, maxSpeed);
}

// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  motor2Speed = -motor2Speed;
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  }
  else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  }
  else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}
