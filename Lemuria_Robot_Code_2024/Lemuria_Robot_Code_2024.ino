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

// Imports for temp
#include <OneWire.h>
#include <DallasTemperature.h>

//IMU imports
#include "quaternionFilters.h"
#include "MPU9250.h"

//LCD import
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);

//Set IMU
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0  // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
float imuYawOffset = 0, imuPitchOffset = 0, imuRollOffset = 0;


// Data wire is plugged into port 11 on the Arduino
#define ONE_WIRE_BUS 12
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


// Controller variable
IBusBM ibus;

int SerialBaudRate = 9600;
// Set to true to get Serial output for debugging
bool SerialDebug = true;

// Arduino ports x,y motors are connected to (-1 right now to represent temp values)
// Forward and backwards are digital pins and speeds are anolog pins
const int mflForwardID = 28, mflBackwardID = 30, mflSpeedID = 3,
          mfrForwardID = 32, mfrBackwardID = 34, mfrSpeedID = 4,
          mbrForwardID = 44, mbrBackwardID = 46, mbrSpeedID = 6,
          mblForwardID = 40, mblBackwardID = 42, mblSpeedID = 5;

// Arduino ports for vertical motors. Connect to anolog ports(-1 right now to represent temp values)
const int VerticalForwardID = 7, VerticalBackwardID = 8;
Servo fowardVM, backwardVM;
double pitchLockAngle = 0;
bool pitchLocked = false;
double pitchKP = 0;

const int xyMotorAmount = 4;

float bodyToXYMotorMatrix[xyMotorAmount][3] = { { 1, 1, 1 },
                                                { 1, -1, -1 },
                                                { -1, -1, 1 },
                                                { -1, 1, -1 } };

int const MAX = 100;




Servo arm, claw;
// Arduino port for arm servo. Connects to a digital pin
int ArmServoID = 11;
int ClawServoID = 9;

// RC controller variablies
// 0 is joystick 1 x, 2 is joystick 1 y
// 1 is joystick 2 y, 3 is joystick 2 x
// 4 is switch a, 5 is switch b
// 6 is dial a, 7 is dial b (not currently working (switch 5 affects this value?))
// 8 is the 3-step switch c, 9 is switch d
int const channelSize = 10;
float rc_values[channelSize];

//initilize IMU
void imuInit() {
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71)  // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1);
    Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1);
    Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48) {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug) {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
    //    delay(2000); // Add delay to see results before serial spew of data

    if (SerialDebug) {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

  }  // if (c == 0x71)
  else {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  //need to update twice before zeroing because
  //MAX HAS NO FUCKING CLUE WHY THE FIRST YAW,PTICH,ROLL IMU READS
  //IS FUCKING WILDLY DIFFENT
  imuUpdate();
  imuUpdate();
}


/*
* Update imu values and print values if debug = true
*/

void imuUpdate() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;  // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;  // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;  // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
                 * myIMU.factoryMagCalibration[0]
               - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
                 * myIMU.factoryMagCalibration[1]
               - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
                 * myIMU.factoryMagCalibration[2]
               - myIMU.magBias[2];

    // Must be called before updating quaternions!
    myIMU.updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                           myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                           myIMU.mx, myIMU.mz, myIMU.deltat);


    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
    myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
    myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw *= RAD_TO_DEG;

    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw -= 2.65;
    myIMU.roll *= RAD_TO_DEG;

    //apply offset for zeroing imu
    myIMU.yaw = myIMU.yaw - atan2(sin(imuYawOffset / RAD_TO_DEG), cos(imuYawOffset / RAD_TO_DEG)) * RAD_TO_DEG;
    myIMU.pitch = myIMU.pitch - atan2(sin(imuPitchOffset / RAD_TO_DEG), cos(imuPitchOffset / RAD_TO_DEG)) * RAD_TO_DEG;
    myIMU.roll = myIMU.roll - atan2(sin(imuRollOffset / RAD_TO_DEG), cos(imuRollOffset / RAD_TO_DEG)) * RAD_TO_DEG;
  }  // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)



  // update debug once per half-second independent of read rate
  if (myIMU.delt_t > 500) {
    if (SerialDebug) {
      Serial.print("ax = ");
      Serial.print((int)1000 * myIMU.ax);
      Serial.print(" ay = ");
      Serial.print((int)1000 * myIMU.ay);
      Serial.print(" az = ");
      Serial.print((int)1000 * myIMU.az);
      Serial.println(" mg");
    }

    if (SerialDebug) {
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(myIMU.yaw, 2);
      Serial.print(", ");
      Serial.print(myIMU.pitch, 2);
      Serial.print(", ");
      Serial.println(myIMU.roll, 2);
    }

    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;
  }  // if (myIMU.delt_t > 500)
}

void imuZero() {
  imuYawOffset = myIMU.yaw;
  imuPitchOffset = myIMU.pitch;
  imuRollOffset = myIMU.roll;
}

// Reads a joystick, dial, or 3 step switch off controller
int readChannel(byte channelInput, float minLimit, float maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return mapFloat(ch, 1000.0, 2000.0, minLimit, maxLimit);
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

void motorMessage(int motorNum, int power) {
  switch (motorNum) {
    case 1:
      lcd.setCursor(0, 4);
      break;

    case 2:
      lcd.setCursor(0, 9);
      break;

    case 3:
      lcd.setCursor(1, 4);
      break;

    case 4:
      lcd.setCursor(1, 9);
      break;

    case 5:
      lcd.setCursor(2, 4);
      break;

    case 6:
      lcd.setCursor(2, 9);
      break;

    default:
      return;
  }

  lcd.print((int)power);
}

// finds if any value is  greater than the limit, then balances out all speeds to be lower than the limit
void balanceSpeeds(int limit, int motorSpeeds[], int size) {
  double maxValue = 0;
  for (int i = 0; i < size; i++) {
    if (maxValue < abs(motorSpeeds[i]) && abs(motorSpeeds[i]) > limit)
      maxValue = abs(motorSpeeds[i]);
  }

  // returns if no value is greater than the limit
  if (maxValue == 0) return;

  for (int i = 0; i < size; i++) {
    motorSpeeds[i] = (int)((motorSpeeds[i] / maxValue) * limit);
  }
}

void balanceSpeeds(float limit, float motorSpeeds[], int size) {
  float maxValue = 0;
  for (int i = 0; i < size; i++) {
    if (maxValue < abs(motorSpeeds[i]) && abs(motorSpeeds[i]) > limit)
      maxValue = abs(motorSpeeds[i]);
  }

  // returns if no value is greater than the limit
  if (maxValue == 0) return;

  for (int i = 0; i < size; i++) {
    motorSpeeds[i] = (int)((motorSpeeds[i] / maxValue) * limit);
  }
}


// Sets the speed for one of the vertical motors.
// MotorNum should be 1 or 2 Power should be from -127 to 127
void setVerticalMotor(int motorNum, int power) {
  int pin;
  power = map(power, -127, 127, 1000, 2000);
  switch (motorNum) {
    case 1:
      fowardVM.writeMicroseconds(power);

      break;
    case 2:
      backwardVM.writeMicroseconds(power);
      break;
    default:
      return;
  }




  //analogWrite(pin, power);
}

void updateTemp() {
  float tempC = sensors.getTempCByIndex(0);
  // Check if reading temp was successful
  if (tempC != DEVICE_DISCONNECTED_C) {
    lcd.setCursor(0, 4);
    lcd.print("Temperature is: ");
    lcd.setCursor(15, 4);
    lcd.print(tempC);
  } else {
    lcd.setCursor(0, 4);
    Serial.println("Could not read temp");
  }
}

//TODO fine tune these values before use
void lockVerticalMotors(int vertPwr) {
  double disFromTarget = pitchLockAngle - myIMU.pitch;
  //kp pid control for vertical motors
  int speed = constrain(disFromTarget * pitchKP, -50, 50);


  int vertPwrs[2] = {vertPwr+speed,vertPwr-speed};
  balanceSpeeds(127,vertPwrs,2);

  setVerticalMotor(1, vertPwrs[0]);
  motorMessage(5,vertPwrs[0]);
  setVerticalMotor(2, vertPwrs[1]);
  motorMessage(6,vertPwrs[1]);
}

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}


void linearTransform(float matrix1[][3], float matrix2[], float result[], int rows1, int cols1) {
  // Perform a linear transform
  for (int i = 0; i < rows1; ++i) {
    result[i] = 0;
    for (int k = 0; k < cols1; ++k) {
      result[i] += matrix1[i][k] * matrix2[k];
    }
  }
}

void setup() {

  // Attaches the vertical motors to an anolog pins
  pinMode(VerticalForwardID, OUTPUT);
  pinMode(VerticalBackwardID, OUTPUT);

  // Sets up the arm servo
  arm.attach(ArmServoID);
  claw.attach(ClawServoID);

  fowardVM.attach(VerticalForwardID);
  backwardVM.attach(VerticalBackwardID);

  //Initizal veritcal motors at stop position and wait to properly work
  fowardVM.writeMicroseconds(1500);
  backwardVM.writeMicroseconds(1500);
  delay(1000);


  // Start Serial Communcation
  Serial.begin(SerialBaudRate);

  // Start Controller Communcation
  ibus.begin(Serial1);

  // Start up the library for temp sensor
  sensors.begin();

  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("They turned the frogs");
  lcd.setCursor(0, 1);
  lcd.print("Gay");
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("FL:     FR:");
  lcd.setCursor(0, 1);
  lcd.print("BL:     BR:");
  lcd.setCursor(0, 2);
  lcd.print("VF:     VB:");
  //init the IMU
  //imuInit();
  //imuZero();
}

void loop() {
  //imuUpdate();

  sensors.requestTemperatures();  // Send the command to get temperatures
  // get temp value in celceious
  float tempC = sensors.getTempCByIndex(0);

  float horizontalVector[3];
  int vertPwr;

  float xyMotorSpeeds[xyMotorAmount];
  int mV;

  // RC controller variablies
  // 0 is joystick 1 x, 2 is joystick 1 y
  // 1 is joystick 2 y, 3 is joystick 2 x
  // 4 is switch a, 5 is switch b
  // 6 is dial a, 7 is dial b (7 not currently working (switch 5 affects this value?))
  // 8 is the 3-step switch c, 9 is switch d
  for (byte i = 0; i < channelSize; i++) {
    if (i < 4 | i == 6 | i == 7 | i == 8) {
      rc_values[i] = readChannel(i, -1, 1, 0);
    } else {
      rc_values[i] = readSwitch(i, false);
    }

    //Debug serial output for controller
    Serial.print("Ch");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(rc_values[i]);
    Serial.print(" | ");
  }
  Serial.println();

  horizontalVector[0] = rc_values[3];
  horizontalVector[1] = rc_values[1];
  horizontalVector[2] = rc_values[0];
  vertPwr = (int)mapFloat(rc_values[2],-1,1,-127,127);

  linearTransform(bodyToXYMotorMatrix, horizontalVector, xyMotorSpeeds, xyMotorAmount, 3);

  // Limits the speeds so they can't exceed the max speed of the motors
  balanceSpeeds(1.0, xyMotorSpeeds, xyMotorAmount);


  // Set the power in vertical motors
  if (rc_values[5] == 0) {
    setVerticalMotor(1, vertPwr);
    motorMessage(5, vertPwr);
    setVerticalMotor(2, vertPwr);
    motorMessage(6, vertPwr);
    pitchLocked = false;
  } else {
    if (!pitchLocked) {
      pitchLockAngle = myIMU.pitch;
      pitchLocked = true;
    }
    lockVerticalMotors(vertPwr);
  }

  // mapping dial to servo values
  int clawPwr;
  if (rc_values[4] == 1)
    clawPwr = 0;
  else
    clawPwr = 180;
  //int clawPwr = map(rc_values[5], -255, 255, 0, 180);
  int armPwr = (int)mapFloat(rc_values[6], -1, 1, 0, 180);

  //writing power to servos
  claw.write(clawPwr);
  arm.write(armPwr);

  // Set the power in x,y motors
  for (int i = 0; i < xyMotorAmount; i++) {
    int motorSpeed = (int)mapFloat(xyMotorSpeeds[i], -1, 1, -255, 255);
    setMotor(i + 1, xyMotorSpeeds[i]);
    motorMessage(i, xyMotorSpeeds[i]);
  }
}


