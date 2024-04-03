/*
 by: Maximus Theis and Sawyer Theis
 date: April 1, 2024
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

*/


#include <Servo.h>

#include <Wire.h>

#include <Math.h>

#include <IBusBM.h>

//IMU imports
#include "quaternionFilters.h"
#include "MPU9250.h"

// Controller variable
IBusBM ibus;

int SerialBaudRate = 115200;

// Arduino ports x,y motors are connected to (-1 right now to represent temp values)
// Forward and backwards are digital pins and speeds are anolog pins
const int mflForwardID = -1, mflBackwardID = -1, mflSpeedID = -1,
  mfrForwardID = -1, mfrBackwardID = -1, mfrSpeedID = -1,
  mbrForwardID = -1, mbrBackwardID = -1, mbrSpeedID = -1,
  mblForwardID = -1, mblBackwardID = -1, mblSpeedID = -1;

// Arduino ports for vertical motors. Connect to anolog ports(-1 right now to represent temp values)
const int VM_1 = -1, VM_2 = -1;

const int xyMotorAmount = 4;

// Various constants used to calculate the Rotation Constant
float const robotLength = 62.4, robotWidth = 43.6, 
// Angle of the x,y motors from the center of the robot
  motorLocAngle[xyMotorAmount] = {atan2(robotWidth/2, robotLength/2), atan2(-robotWidth/2, robotLength/2),
  atan2(-robotWidth/2, -robotLength/2), atan2(robotWidth/2, -robotLength/2)}, 
// Angle of the x,y motors. Assumes forward is 0 and counterclockwise is positive
// Direction of Forward thrust defines the motor angle
  motorAngle[xyMotorAmount] = {-M_PI / 4, M_PI / 4, M_PI * (3.0 / 4), -M_PI * (3.0 / 4)};

// A constant of ethier -1 or 1 that is used to detemine wether an 
// x,y motor should go forwards or backwards while turning 
// (value can be 0 if motor doesn't contribute to turning)
int motorRotCont[xyMotorAmount];

Servo arm;
// Arduino port for arm servo. Connects to a digital pin
int ArmServoID = 11;

// RC controller variablies
// 0 is joystick 1 x, 2 is joystick 1 y
// 1 is joystick 2 y, 3 is joystick 2 x
// 4 is switch a, 5 is switch b
// 6 is dial a, 7 is dial b (not currently working (switch 5 affects this value?))
// 8 is the 3-step switch c, 9 is switch d
int const channelSize = 10;
int rc_values[channelSize];

// Reads a joystick, dial, or 3 step switch off controller
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Reads a switch from the controller
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

// Runs x,y motors, power should be a value from -255 to 255
void setMotor(int motorNum, int power) {
  int motorForward, motorBackward, motorSpeed;

  switch (motorNum) {
    case 1:
    motorForward = mflForwardID;
    motorBackward = mflBackwardID;
    motorSpeed = mflSpeedID;
    break;

    case 2:
    motorForward = mfrForwardID;
    motorBackward = mfrBackwardID;
    motorSpeed = mfrSpeedID;
    break;

    case 3:
    motorForward = mbrForwardID;
    motorBackward = mbrBackwardID;
    motorSpeed = mbrSpeedID;
    break;

    case 4:
    motorForward = mblForwardID;
    motorBackward = mblBackwardID;
    motorSpeed = mblSpeedID;
    break;

    default:
    return;
  }
  
  if (power < 0) {
    digitalWrite(motorForward, LOW);
    digitalWrite(motorBackward, HIGH);
  } else {
    digitalWrite(motorForward, HIGH);
    digitalWrite(motorBackward, LOW);
  }
  
  //int speed = map(abs(power), 0, 100, 0, 255);
  analogWrite(motorSpeed, abs(power));
}

// finds if any value is greater than the limit, then balances out all speeds to be lower than the limit 
void balanceSpeeds(int limit, int motorSpeeds[], int size) {
  double maxValue = 0;
  for (int i = 0; i < size; i++) {
    if (maxValue < abs(motorSpeeds[i]) && abs(motorSpeeds[i]) > limit)
      maxValue = abs(motorSpeeds[i]);
  }

  // returns if no value is greater than the limit
  if (maxValue == 0) return;

  for (int i = 0; i < size; i++) {
    motorSpeeds[i] = (int)((motorSpeeds[i]/maxValue) * limit);
  }
}

// Finds the number to multiply the turning speed for each motor 
void findMotorRotCont(int indexValue) {
  double motorRotationEffectiveness = sin(motorLocAngle[indexValue] - motorAngle[indexValue]);

  if (motorRotationEffectiveness != 0)
    // Balances the value to be either -1 or 1
    motorRotCont[indexValue] = (int)(motorRotationEffectiveness / abs(motorRotationEffectiveness));
  else motorRotCont[indexValue] = 0;
}

// Sets the speed for one of the vertical motors. 
// MotorNum should be 1 or 2 Power should be from -127 to 127
void setVerticalMotor(int motorNum, int power) {
  int pin;
  switch (motorNum) {
    case 1: 
      pin = VM_1;
      break;
    case 2:
      pin = VM_2;
      break;
    default:
      return;
  }

  analogWrite(pin, power + 127);
}

void setup() {
  // Calculate motor Rotation Contstant
  for (int i = 0; i < xyMotorAmount; i++)
    findMotorRotCont(i);

  // Attaches the vertical motors to an anolog pins 
  pinMode(VM_1, OUTPUT);
  pinMode(VM_2, OUTPUT);

  // Sets up the arm servo
  arm.attach(ArmServoID);

  // Start Serial Communcation
  Serial.begin(SerialBaudRate);
  // Start Controller Communcation
  ibus.begin(Serial1);
}

void loop() {


  int Joy1_X, Joy1_Y, Joy2_X, Joy2_Y;

  int yPwr, xPwr, vertPwr, spinPwr;

  int xyMotorSpeeds[xyMotorAmount], mV;

  // RC controller variablies
  // 0 is joystick 1 x, 2 is joystick 1 y
  // 1 is joystick 2 y, 3 is joystick 2 x
  // 4 is switch a, 5 is switch b
  // 6 is dial a, 7 is dial b (not currently working (switch 5 affects this value?))
  // 8 is the 3-step switch c, 9 is switch d
  for (byte i = 0; i < channelSize; i++) {
    if (i < 4 | i == 6 | i == 7 | i == 8) {
      rc_values[i] = readChannel(i, -255, 255, 0);
    } else {
      rc_values[i] = readSwitch(i, false);
    }

    //Debug serial output for controller
    /*Serial.print("Ch");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(rc_values[i]);
    Serial.print(" | ");*/
  }

  yPwr = rc_values[1];
  xPwr = rc_values[3];
  spinPwr = rc_values[0];
  vertPwr = rc_values[2] / 2;

  // converts the joystick 1 to polar coordinates
  int mag;
  double angle;
  angle = atan2(xPwr, yPwr);
  
  // Convert the joystick from a square to a circle
  xPwr = (int) (xPwr * cos(angle));
  yPwr = (int) (yPwr * sin(angle));

  mag = (int) sqrt(((long)yPwr * yPwr) + ((long)xPwr * xPwr));


  // only does the larger of spin or x,y movement
  if (mag >= abs(spinPwr)) {
    for (int i = 0; i < xyMotorAmount; i++)
      xyMotorSpeeds[i] = (int)(mag * cos(angle + motorAngle[i]));
  } else {
    for (int i = 0; i < xyMotorAmount; i++)
      xyMotorSpeeds[i] = spinPwr * motorRotCont[i];
  }

  // Limits the speeds so they can't exceed the max speed of the motors
  balanceSpeeds(255, xyMotorSpeeds, xyMotorAmount);

  // Set the power in vertical motors
  setVerticalMotor(1, vertPwr);
  setVerticalMotor(2, vertPwr);

  // Set the power in x,y motors
  for (int i = 0; i < xyMotorAmount; i++) {
    setMotor(i+1, xyMotorSpeeds[i]);
  }
}
