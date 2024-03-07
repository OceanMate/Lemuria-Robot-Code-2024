// Serial SaberTooth Motor Test

// authors: Scott Fraser & LeRoy van der Vegt

// created: July 2020

// Creative license use freely

//

// This tests 2 Sabertooth controllers and 4 motors.  Each motor will be tested forward and reverse.

// Test status displayed on a 4x20 LCD screen.  Pins for each Sabertooth controller have to be set

// in the following order.

//

//    Simplified Serial Mode 3 (MicroController controlled-Arduino)

//    pin 1 - ON     (Lithum battery - ON, other battery type - OFF)

//    pin 2 - OFF    (R/C radio control signal control - loss of signal stop motors; active-ON, not active-OFF)

//    pin 3 - ON     (Lithum battery voltage protection - protection-OFF, no protection-ON)

//    pin 4 - OFF    (set Baud rate of serial connection @9600 see below)

//    pin 5 - ON     (set Baud rate of serial connection @9600 see below)

//    pin 6 - OFF    (Serial Slave - controllers are independent-OFF controller are synchronized-ON)

//

//    Different baud rate for pin 4 and 5

//    baud  pin  pin

//    rate   4    5

//    ----- ---  ---

//    2400  ON   ON

//    9600  OFF  ON   <------  9600 baud is used in this sketch

//    19200 ON   OFF

//    38400 OFF  OFF

//


// Arduino Serial communications

#include <SoftwareSerial.h>  // used for Sabretooth & BlueTooth

#include <Servo.h>

#include <Wire.h>

#include <Math.h>

#include <IBusBM.h>

IBusBM ibus;



int SerialBaudRate = 115200;

int mflForwardID = -1, mflBackwardID = -1, mflSpeedID = -1,
  mfrForwardID = -1, mfrBackwardID = -1, mfrSpeedID = -1,
  mbrForwardID = -1, mbrBackwardID = -1, mbrSpeedID = -1,
  mblForwardID = -1, mblBackwardID = -1, mblSpeedID = -1;


int VM_1 = -1;
int VM_2 = -1;

int ArmServo = 11;

float mFLangle = -M_PI / 4,
 mFRangle = M_PI / 4,
 mBRangle = M_PI * (3.0 / 4),
 mBLangle = -M_PI * (3.0 / 4);

float robotLength = 62.4, robotWidth = 43.6, motorLocAngle[4];
int motorRotCont[4];
Servo arm;

// RC controller variablies
// 0 is joystick 1 x, 2 is joystick 1 y
// 1 is joystick 2 y, 3 is joystick 2 x
// 4 is switch a, 5 is switch b
// 6 is dial a, 7 is dial b (not currently working (switch 5 affects this value?))
// 8 is the 3-step switch c, 9 is switch d
int const channelSize = 10;
int rc_values[channelSize];

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

// Run moters, -100 to 100
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

    case 3;
    motorForward = mbrForwardID;
    motorBackward = mbrBackwardID;
    motorSpeed = mbrSpeedID;
    break;

    case 4;
    motorForward = mblForwardID;
    motorBackward = mblBackwardID;
    motorSpeed = mblSpeedID;
    break;

    default:
    return;
  }
  
  if (speed < 0) {
    digitalWrite(motorForward, LOW);
    digitalWrite(motorBackward, HIGH);
  } else {
    digitalWrite(motorForward, HIGH);
    digitalWrite(motorBackward, LOW);
  }
  
  int speed = map(abs(power), -100, 100, 0, 255);
  analogWrite(motorSpeed, abs(speed));
}

// Limits all the motor speed to be between -1 to 1
void balanceSpeeds(int limit, int& value1, int& value2, int& value3, int& value4) {
  double maxValue = 0;
  if (maxValue < abs(value1) && abs(value1) > limit) maxValue = abs(value1);
  if (maxValue < abs(value2) && abs(value2) > limit) maxValue = abs(value2);
  if (maxValue < abs(value3) && abs(value3) > limit) maxValue = abs(value3);
  if (maxValue < abs(value4) && abs(value4) > limit) maxValue = abs(value4);

  if (maxValue == 0) return;

  value1 = (value1/maxValue) * limit;
  value2 = (value2/maxValue) * limit;
  value3 = (value3/maxValue) * limit;
  value4 = (value4/maxValue) * limit;
}

// Finds the number to mulitple the turning for each motor 
void findMotorRotCont(int indexValue, double motorAngle) {

  if (sin(motorLocAngle[indexValue] - motorAngle) != 0)
    motorRotCont[indexValue] = (int)(sin(motorLocAngle[indexValue] - motorAngle) / abs(sin(motorLocAngle[indexValue] - motorAngle)));
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
  motorLocAngle[0] = atan2(robotWidth/2, robotLength/2);
  motorLocAngle[1] = atan2(-robotWidth/2, robotLength/2);
  motorLocAngle[2] = atan2(-robotWidth/2, -robotLength/2);
  motorLocAngle[3] = atan2(robotWidth/2, -robotLength/2);

  findMotorRotCont(0, mFLangle);
  findMotorRotCont(1, mFRangle);
  findMotorRotCont(2, mBRangle);
  findMotorRotCont(3, mBLangle);


  pinMode(VM_1, OUTPUT);

  pinMode(VM_2, OUTPUT);

  arm.attach(ArmServo);

  // Start Serial Communcation
  Serial.begin(SerialBaudRate);
  ibus.begin(Serial1);


}

void loop() {



  int Joy1_X, Joy1_Y, Joy2_X, Joy2_Y;

  int yPwr, xPwr, vertPwr, spinPwr;

  int mFL, mFR, mBR, mBL, mV;

  // RC controller variablies
// 0 is joystick 1 x, 2 is joystick 1 y
// 1 is joystick 2 y, 3 is joystick 2 x
// 4 is switch a, 5 is switch b
// 6 is dial a, 7 is dial b (not currently working (switch 5 affects this value?))
// 8 is the 3-step switch c, 9 is switch d
  for (byte i = 0; i < channelSize; i++) {
    if (i < 4 | i == 6 | i == 7 | i == 8) {
      rc_values[i] = readChannel(i, -100, 100, 0);
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


  // maps the joysitck outputs to something the motor can use.
  // the motors take a value from -127 - 127
  // Map to 511 to improve accuracy when doing math
  yPwr = rc_values[1];
  xPwr = rc_values[3];
  spinPwr = rc_values[0];
  vertPwr = rc_values[2];

  // converts the joystick 1 to polar coordinates
  int mag;
  double angle;
  angle = atan2(xPwr, yPwr);
  
  // Convert the joystick from a square to a circle
  xPwr = (int) (xPwr * cos(angle));
  yPwr = (int) (yPwr * sin(angle));

  mag = (int) sqrt(((long)yPwr * yPwr) + ((long)xPwr * xPwr));


  // only does the larger spin or x,y movement
  if (mag >= abs(spinPwr)) {
    mFL = (int)(mag * cos(angle + mFLangle));
    mFR = (int)(mag * cos(angle + mFRangle));
    mBR = (int)(mag * cos(angle + mBRangle));
    mBL = (int)(mag * cos(angle + mBLangle));
  } else {
    mFL = spinPwr * motorRotCont[0];
    mFR = spinPwr * motorRotCont[1];
    mBR = spinPwr * motorRotCont[2];
    mBL = spinPwr * motorRotCont[3];
  }

  // Limits the speeds so they can't exceed the max speed of the motors
  balanceSpeeds(100, mFL, mFR, mBL, mBR);

  // Set the power in each motor
  //setVerticalMotor(1,127);

  setMotor(1, mFL);

  delay(1);

  setMotor(2, mFR);

  delay(1);

  setMotor(3, mBR);

  delay(1);

  setMotor(4, mBL);

  delay(1);

  for (int pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    arm.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (int pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    arm.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}
