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


// Sabertooth device driver

#include <SabertoothSimplified.h>


// include the DISPLAY library

#include <Wire.h>

#include <LiquidCrystal_I2C.h>
#include <Math.h>


int SerialBaudRate = 9600;


// Initialize the LCD screen

int LCD_I2C_Addr = 0x27;  //I2C address for the LCD Screen (Default=0x27)

LiquidCrystal_I2C lcd(LCD_I2C_Addr, 20, 4);  // Set the LCD I2C address. Use 20 Space 4-line LCD.


// Initialize Arduino serial communications

SoftwareSerial SWSerial(NOT_A_PIN, 10);  // RX on no pin (unused), TX on pin 10 (to S1).


SoftwareSerial BTSerial(2, 3);  // For communication to with the Bluetooth Device

int BT_Ena = 4;


// Initialize Sabertooth driver passing it the Arduino serial communications object

SabertoothSimplified ST(SWSerial);  // Use SWSerial as the serial port.

int ST1_S2 = 8;  // Arduino pin attached to Sabertooth controller

int ST2_S2 = 9;  // Arduino pin attached to Sabertooth controller

int VM_1 = -1;
int VM_2 = -1;

float mFLangle = -M_PI / 4,
 mFRangle = M_PI / 4,
 mBRangle = M_PI * (3.0 / 4),
 mBLangle = -M_PI * (3.0 / 4);

float robotLength = 62.4, robotWidth = 43.6, motorLocAngle[4];
int motorRotCont[4];

// Sends motor testing information to LCD screen
void motorMessage(int motorNum, int power) {

  int lineNum = -1;

  int colNum = -1;



  switch (motorNum) {

    case 1:  // Motor number

      lineNum = 0;

      colNum = 3;

      break;

    case 2:  // Motor number

      lineNum = 0;

      colNum = 11;

      break;

    case 3:  // Motor number

      lineNum = 1;

      colNum = 3;

      break;

    case 4:  // Motor number

      lineNum = 1;

      colNum = 11;

      break;
  }

  lcd.setCursor(colNum, lineNum);

  lcd.print(power);

  lcd.print(" ");
}

// Initialize motor test parameters
void setMotor(int motorNum, int power) {



  int controllerNum = -1;  // Controller to be tested

  int motorNumber = -1;  // Motor on that the Controller to be tested



  switch (motorNum) {

    case 1:  // Motor to be updated

      controllerNum = ST2_S2;

      motorNumber = 1;

      break;

    case 2:  // Motor to be updated

      controllerNum = ST2_S2;

      motorNumber = 2;

      break;

    case 3:  // Motor to be updated

      controllerNum = ST1_S2;

      motorNumber = 1;

      break;

    case 4:  // Motor to be updated

      controllerNum = ST1_S2;

      motorNumber = 2;

      break;
  }


  // Set Motor power

  digitalWrite(controllerNum, HIGH);

  motorMessage(motorNum, power);

  ST.motor(motorNumber, power);

  delayMicroseconds(50);

  digitalWrite(controllerNum, LOW);
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

 
  // Set Arduino pin for each sabertooth as OUTPUT

  pinMode(ST1_S2, OUTPUT);  // Arduino pin control to sabertooth

  pinMode(ST2_S2, OUTPUT);  // Arduino pin control to sabertooth

  pinMode(BT_Ena, OUTPUT);  // Bluetooth ena pin

  digitalWrite(BT_Ena, LOW);  // Set the Bluetooth ena low

  // Start Serial Communications

  SWSerial.begin(SerialBaudRate);  // Start the Sabretooth channel

  BTSerial.begin(SerialBaudRate);  // Start the Bluetooth channel

  Serial.begin(SerialBaudRate);


  // Set up LCD

  lcd.begin();

  lcd.backlight();

  lcd.write(12);

  lcd.setCursor(0, 0);

  lcd.print("Connection at: ");

  lcd.print(SerialBaudRate);

  lcd.setCursor(0, 1);

  lcd.print("Test...");




  lcd.setCursor(8, 1);

  lcd.print("COMPLETE!");

  delay(1000);

  lcd.clear();

  lcd.setCursor(0, 0);

  lcd.print("M1:     M2: ");  // 'Ms' - Motor Speed for motor 1 and 2

  lcd.setCursor(0, 1);

  lcd.print("M3:     M4: ");  // 'Ms' - Motor Speed for motor 3 and 4
}

void loop() {



  int Joy1_X, Joy1_Y, Joy2_X, Joy2_Y;

  int yPwr, xPwr, vertPwr, spinPwr;

  int mFL, mFR, mBR, mBL, mV;


  Joy1_Y = analogRead(A0);  // get the left vertical (Y) joystick input

  Joy1_X = analogRead(A1);  // get the left horizontal (X) joystick input

  Joy2_Y = analogRead(A2);  // get the right vertical (Y) joystick input

  Joy2_X = analogRead(A3);  // get the right horizontal (X) joystick input

  // maps the joysitck outputs to something the motor can use.
  // the motors take a value from -127 - 127
  // Map to 511 to improve accuracy when doing math
  yPwr = map(Joy1_Y, 0, 865, -511, 511);
  xPwr = map(Joy1_X, 0, 865, -511, 511);
  spinPwr = 0;
  spinPwr = map(Joy1_X, 0, 865, -511, 511);
  //vertPwr = map(Joy2_Y, 0, 1023, -127, 127);  //Currently not simulated. Need to add 2 more motors

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

  setMotor(1, mFL);

  delay(1);

  setMotor(2, mFR);

  delay(1);

  setMotor(3, mBL);

  delay(1);

  setMotor(4, mBR);

  delay(1);
}
