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

#include <AFMotor.h>

#include <Wire.h>

#include <Math.h>


int SerialBaudRate = 9600;

//set Hozantal motors
/*AF_DCMotor HM_1(1);
AF_DCMotor HM_2(2);
AF_DCMotor HM_3(3);
AF_DCMotor HM_4(4);*/


int VM_1 = -1;
int VM_2 = -1;

int AM = 11;

float mFLangle = -M_PI / 4,
 mFRangle = M_PI / 4,
 mBRangle = M_PI * (3.0 / 4),
 mBLangle = -M_PI * (3.0 / 4);

float robotLength = 62.4, robotWidth = 43.6, motorLocAngle[4];
int motorRotCont[4];
Servo arm;

// Initialize motor test parameters
void setMotor(int motorNum, int power) {
  

  AF_DCMotor temp(motorNum);



  if(power >= 0) temp.run(FORWARD);
  else temp.run(BACKWARD);
  temp.setSpeed(power);
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

  arm.attach(AM);

  // Start Serial Communcation
  Serial.begin(SerialBaudRate);



}

void loop() {



  int Joy1_X, Joy1_Y, Joy2_X, Joy2_Y;

  int yPwr, xPwr, vertPwr, spinPwr;

  int mFL, mFR, mBR, mBL, mV;


  Joy1_Y = analogRead(A0);  // get the left vertical (Y) joystick input

  //Bad
  Joy1_X = analogRead(A1);  // get the left horizontal (X) joystick input

  //bad
  Joy2_Y = analogRead(A2);  // get the right vertical (Y) joystick input

  Joy2_X = analogRead(A3);  // get the right horizontal (X) joystick input

  //Serial.println((String)"Joy1y: " + Joy1_Y + " Joy2x: " + Joy1_X + "Joy2y: " + Joy2_Y + " Joy2x: " + Joy2_X);

  // maps the joysitck outputs to something the motor can use.
  // the motors take a value from -127 - 127
  // Map to 511 to improve accuracy when doing math
  yPwr = map(Joy1_Y, 0, 865, -511, 511);
  xPwr = map(Joy1_X, 0, 865, -511, 511);
  spinPwr = map(Joy2_X, 0, 865, -511, 511);
  //vertPwr = map(Joy2_Y, 0, 1023, -127, 127);

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
  balanceSpeeds(511, mFL, mFR, mBL, mBR);

  // scale back down to +-127
  mFL /= 4;
  mFR /= 4;
  mBL /= 4;
  mBR /= 4;

  // Set the power in each motor

  //setVerticalMotor(1,127);

  setMotor(1,127);

  delay(1);

  setMotor(2, 127);

  delay(1);

  setMotor(3, 127);

  delay(1);

  setMotor(4, 127);

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
