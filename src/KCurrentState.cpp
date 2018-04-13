/*
 * KCurrentState.cpp
 *
 *  Created on: 15 maj 2014
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "KCurrentState.h"
#include "FarmbotSensor.h"

static KCurrentState *instance;

FarmbotSensor FarmbotSensor;

long x = 0;
long y = 0;
long z = 0;
unsigned int speed = 0;
bool endStopState[3][2];
long Q = 0;
int lastError = 0;

KCurrentState *KCurrentState::getInstance()
{
  if (!instance)
  {
    instance = new KCurrentState();
  };
  return instance;
};

KCurrentState::KCurrentState()
{
  x = 0;
  y = 0;
  z = 0;
  speed = 0;
  Q = 0;
  lastError = 0;
}

float KCurrentState::getAX()
{
  return FarmbotSensor.ax;
}

float KCurrentState::getAY()
{
  return FarmbotSensor.ay;
}

float KCurrentState::getAZ()
{
  return FarmbotSensor.az;
}
float KCurrentState::getGX()
{
  return FarmbotSensor.gx;
}

float KCurrentState::getGY()
{
  return FarmbotSensor.gy;
}

float KCurrentState::getGZ()
{
  return FarmbotSensor.gz;
}
float KCurrentState::getMX()
{
  return FarmbotSensor.mx;
}

float KCurrentState::getMY()
{
  return FarmbotSensor.my;
}

float KCurrentState::getMZ()
{
  return FarmbotSensor.mz;
}

double KCurrentState::getLat()
{
  return FarmbotSensor.latitude;
}

double KCurrentState::getLog()
{
  return FarmbotSensor.longitude;
}

double KCurrentState::getSpeed()
{
  return FarmbotSensor.speed;
}

double KCurrentState::getCovQ()
{
  return FarmbotSensor.covQ;
}
long KCurrentState::getRow()
{
  return FarmbotSensor.row;
}
long KCurrentState::getCol()
{
  return FarmbotSensor.col;
}
double KCurrentState::getCovR()
{
  return FarmbotSensor.covR;
}
float KCurrentState::getRoll()
{
  return FarmbotSensor.roll;
}

float KCurrentState::getPitch()
{
  return FarmbotSensor.pitch;
}

float KCurrentState::getHeading()
{
  return FarmbotSensor.heading;
}
bool KCurrentState::getisIMUReady()
{
  return FarmbotSensor.isIMUReady;
}
bool KCurrentState::getisMotorReady()
{
  return FarmbotSensor.isMotorReady;
}
bool KCurrentState::getisGPSReady()
{
  return FarmbotSensor.isGPSReady;
}
bool KCurrentState::getisKalmanReady()
{
  return FarmbotSensor.isKalmanReady;
}
bool KCurrentState::getisKalmanFiltering()
{
  return FarmbotSensor.isKalmanFiltering;
}
bool KCurrentState::getisMovingForward()
{
  return FarmbotSensor.isMovingForward;
}
bool KCurrentState::getisMovingBackward()
{
  return FarmbotSensor.isMovingBackward;
}
bool KCurrentState::getisArrvied()
{
  return FarmbotSensor.isArrived;
}
bool KCurrentState::getisStopped()
{
  return FarmbotSensor.isStopped;
}
bool KCurrentState::getisGPSAveraging()
{
  return FarmbotSensor.isGPSAveraging;
}
bool KCurrentState::getisUpdatingInitialOrientation()
{
  return FarmbotSensor.isUpdatingInitialOrientation;
}
bool KCurrentState::getisUpdatingReferenceLocation()
{
  return FarmbotSensor.isUpdatingReferenceLocation;
}
bool KCurrentState::getisInitKalman()
{
  return FarmbotSensor.isInitKalman;
}
bool KCurrentState::getisUpdatingQ()
{
  return FarmbotSensor.isUpdatingQ;
}
bool KCurrentState::getisUpdatingR()
{
  return FarmbotSensor.isUpdatingR;
}
long *KCurrentState::getPoint()
{
  //static long currentPoint[3] = {x, y, z};
  //return currentPoint;
}




void KCurrentState::setAX(float newAX)
{
  FarmbotSensor.ax = newAX;
}

void KCurrentState::setAY(float newAY)
{
  FarmbotSensor.ay = newAY;
}

void KCurrentState::setAZ(float newAZ)
{
  FarmbotSensor.az = newAZ;
}

void KCurrentState::setGX(float newGX)
{
  FarmbotSensor.gx = newGX;
}

void KCurrentState::setGY(float newGY)
{
  FarmbotSensor.gy = newGY;
}

void KCurrentState::setGZ(float newGZ)
{
  FarmbotSensor.gz = newGZ;
}


void KCurrentState::setMX(float newMX)
{
  FarmbotSensor.mx = newMX;
}

void KCurrentState::setMY(float newMY)
{
  FarmbotSensor.my = newMY;
}

void KCurrentState::setMZ(float newMZ)
{
  FarmbotSensor.mz = newMZ;
}

void KCurrentState::setLat(double newLat)
{
  FarmbotSensor.latitude = newLat;
}

void KCurrentState::setLog(double newLog)
{
  FarmbotSensor.longitude = newLog;
}

void KCurrentState::setSpeed(double newSpeed)
{
  FarmbotSensor.speed = newSpeed;
}

void KCurrentState::setRoll(float newRoll)
{
  FarmbotSensor.roll_deg = newRoll;
}

void KCurrentState::setPitch(float newPitch)
{
  FarmbotSensor.pitch_deg = newPitch;
}

void KCurrentState::setHeading(float newHeading)
{
  FarmbotSensor.heading_deg = newHeading;
}

void KCurrentState::setInitRoll(float newRoll)
{
  FarmbotSensor.initRoll = newRoll;
}

void KCurrentState::setInitPitch(float newPitch)
{
  FarmbotSensor.initPitch = newPitch;
}

void KCurrentState::setInitHeading(float newHeading)
{
  FarmbotSensor.initHeading = newHeading;
}
void KCurrentState::setInitLat(double newinitLat)
{
  FarmbotSensor.initLat = newinitLat;
}
void KCurrentState::setInitLon(double newinitLon)
{
  FarmbotSensor.initLon = newinitLon;
}

void KCurrentState::setisIMUReady(bool newis)
{
  FarmbotSensor.isIMUReady = newis;
}

void KCurrentState::setisMotorReady(bool newis)
{
  FarmbotSensor.isMotorReady = newis;
}
void KCurrentState::setisGPSReady(bool newis)
{
  FarmbotSensor.isGPSReady = newis;
}
void KCurrentState::setisKalmanReady(bool newis)
{
  FarmbotSensor.isKalmanReady = newis;
}
void KCurrentState::setisKalmanFiltering(bool newis)
{
  FarmbotSensor.isKalmanFiltering = newis;
}
void KCurrentState::setisMovingForward(bool newis)
{
  FarmbotSensor.isMovingForward = newis;
  FarmbotSensor.isMovingBackward = !(newis);
}
void KCurrentState::setisMovingBackward(bool newis)
{
  FarmbotSensor.isMovingBackward = newis;
  FarmbotSensor.isMovingForward = !(newis);
}
void KCurrentState::setisArrived(bool newis)
{
  FarmbotSensor.isArrived = newis;
}
void KCurrentState::setisStopped(bool newis)
{
  FarmbotSensor.isStopped = newis;
  FarmbotSensor.isMovingBackward = !(newis);
  FarmbotSensor.isMovingForward = !(newis);
}
void KCurrentState::setisGPSAveraging(bool newis)
{
  FarmbotSensor.isGPSAveraging = newis;
}
void KCurrentState::setisUpdatingInitialOrientation(bool newis)
{
  FarmbotSensor.isUpdatingInitialOrientation = newis;
}
void KCurrentState::setisUpdatingReferenceLocation(bool newis)
{
  FarmbotSensor.isUpdatingReferenceLocation = newis;
}
void KCurrentState::setisInitKalman(bool newis)
{
  FarmbotSensor.isInitKalman = newis;
}
void KCurrentState::setMovingSpeed(unsigned int newspeed)
{
  FarmbotSensor.movingSpeed = newspeed;
}
void KCurrentState::setisUpdatingQ(bool newis)
{
  FarmbotSensor.isUpdatingQ = newis;
}
void KCurrentState::setisUpdatingR(bool newis)
{
  FarmbotSensor.isUpdatingR = newis;
}
void KCurrentState::setCovQ(double newQ, long row, long col)
{
  FarmbotSensor.covQ = newQ;
  FarmbotSensor.row = row;
  FarmbotSensor.col = col;
}
void KCurrentState::setCovR(double newR, long row, long col){
  FarmbotSensor.covR = newR;
  FarmbotSensor.row = row;
  FarmbotSensor.col = col;
}

void KCurrentState::setEndStopState(unsigned int axis, unsigned int position, bool state)
{
  endStopState[axis][position] = state;
}

void KCurrentState::setStepsPerMm(long stepsX, long stepsY, long stepsZ)
{
  stepsPerMmX = stepsX;
  stepsPerMmY = stepsY;
  stepsPerMmZ = stepsZ;
}





int KCurrentState::getLastError()
{
  return lastError;
}

void KCurrentState::setLastError(int error)
{
  lastError = error;
}



void KCurrentState::storeEndStops()
{
/* From Frambot
  KCurrentState::getInstance()->setEndStopState(0, 0, digitalRead(X_MIN_PIN));
  KCurrentState::getInstance()->setEndStopState(0, 1, digitalRead(X_MAX_PIN));
  KCurrentState::getInstance()->setEndStopState(1, 0, digitalRead(Y_MIN_PIN));
  KCurrentState::getInstance()->setEndStopState(1, 1, digitalRead(Y_MAX_PIN));
  KCurrentState::getInstance()->setEndStopState(2, 0, digitalRead(Z_MIN_PIN));
  KCurrentState::getInstance()->setEndStopState(2, 1, digitalRead(Z_MAX_PIN));
*/
}

void KCurrentState::printPosition()
{
  Serial.print("R82");
  Serial.print(" Latitude");
  Serial.print(FarmbotSensor.latitude);
  Serial.print(" Longtidue");
  Serial.print(FarmbotSensor.longitude);
  Serial.print(" Speed(mph)");
  Serial.print(FarmbotSensor.speed);
  printQAndNewLine();
}
void KCurrentState::printAcceleration(){
  Serial.print("R82");
  Serial.print(" AX");
  Serial.print(FarmbotSensor.ax);
  Serial.print(" AY");
  Serial.print(FarmbotSensor.ay);
  Serial.print(" AZ");
  Serial.print(FarmbotSensor.az);
  printQAndNewLine();
}

void KCurrentState::printOrientation(){
  Serial.print("R82");
  Serial.print(" R");
  Serial.print(FarmbotSensor.roll_deg);
  Serial.print(" P");
  Serial.print(FarmbotSensor.pitch_deg);
  Serial.print(" H");
  Serial.print(FarmbotSensor.heading_deg);
  printQAndNewLine();
}

String KCurrentState::getPosition()
{
/* From Frambot
  if (stepsPerMmX <= 0) { stepsPerMmX = 1; }
  if (stepsPerMmY <= 0) { stepsPerMmY = 1; }
  if (stepsPerMmZ <= 0) { stepsPerMmZ = 1; }

  String output = "";

  output += "R82";
  output += " X";
  output += (float)x / (float)stepsPerMmX * 1.0;
  output += " Y";
  output += (float)y / (float)stepsPerMmY * 1.0;
  output += " Z";
  output += (float)z / (float)stepsPerMmZ * 1.0;
  //output += getQAndNewLine();
*/
  return "0";

}

void KCurrentState::printBool(bool value)
{
  if (value)
  {
    Serial.print("1");
  }
  else
  {
    Serial.print("0");
  }
}

void KCurrentState::printEndStops()
{
/* From Frambot  
  Serial.print("R81");
  Serial.print(" XA");
  printBool(endStopState[0][0]);
  Serial.print(" XB");
  printBool(endStopState[0][1]);
  Serial.print(" YA");
  printBool(endStopState[1][0]);
  Serial.print(" YB");
  printBool(endStopState[1][1]);
  Serial.print(" ZA");
  printBool(endStopState[2][0]);
  Serial.print(" ZB");
  printBool(endStopState[2][1]);
  //Serial.print("\r\n");
  printQAndNewLine();
*/
}

void KCurrentState::print()
{
  printPosition();
  printEndStops();
}

void KCurrentState::setQ(long q)
{
  Q = q;
}

void KCurrentState::resetQ()
{
  Q = 0;
}

void KCurrentState::printQAndNewLine()
{
  Serial.print(" Q");
  Serial.print(Q);
  Serial.print("\r\n");
}

String KCurrentState::getQAndNewLine()
{
  String output = "";

  output += " Q";
  output += Q;
  output += "\r\n";

  return output;
}

void KCurrentState::setEmergencyStop()
{
  emergencyStop = true;
}

void KCurrentState::resetEmergencyStop()
{
  emergencyStop = false;
}

bool KCurrentState::isEmergencyStop()
{
  return emergencyStop;
}
