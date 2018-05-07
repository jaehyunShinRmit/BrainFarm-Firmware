/*
 * KCurrentState.cpp
 *
 *  Created on: 15 maj 2014
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "KCurrentState.h"
#include "BrainBotSensor.h"

static KCurrentState *instance;

BrainBotSensor BrainBotSensor;

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
  return BrainBotSensor.ax;
}

float KCurrentState::getAY()
{
  return BrainBotSensor.ay;
}

float KCurrentState::getAZ()
{
  return BrainBotSensor.az;
}
float KCurrentState::getGX()
{
  return BrainBotSensor.gx;
}

float KCurrentState::getGY()
{
  return BrainBotSensor.gy;
}

float KCurrentState::getGZ()
{
  return BrainBotSensor.gz;
}
float KCurrentState::getMX()
{
  return BrainBotSensor.mx;
}

float KCurrentState::getMY()
{
  return BrainBotSensor.my;
}

float KCurrentState::getMZ()
{
  return BrainBotSensor.mz;
}

double KCurrentState::getLat()
{
  return BrainBotSensor.latitude;
}

double KCurrentState::getLog()
{
  return BrainBotSensor.longitude;
}

double KCurrentState::getSpeed()
{
  return BrainBotSensor.speed;
}

double KCurrentState::getCovQ()
{
  return BrainBotSensor.covQ;
}
long KCurrentState::getRow()
{
  return BrainBotSensor.row;
}
long KCurrentState::getCol()
{
  return BrainBotSensor.col;
}
double KCurrentState::getCovR()
{
  return BrainBotSensor.covR;
}
float KCurrentState::getRoll()
{
  return BrainBotSensor.roll;
}

float KCurrentState::getPitch()
{
  return BrainBotSensor.pitch;
}

float KCurrentState::getHeading()
{
  return BrainBotSensor.heading;
}
bool KCurrentState::getisIMUReady()
{
  return BrainBotSensor.isIMUReady;
}
bool KCurrentState::getisMotorReady()
{
  return BrainBotSensor.isMotorReady;
}
bool KCurrentState::getisGPSReady()
{
  return BrainBotSensor.isGPSReady;
}
bool KCurrentState::getisKalmanReady()
{
  return BrainBotSensor.isKalmanReady;
}
bool KCurrentState::getisKalmanFiltering()
{
  return BrainBotSensor.isKalmanFiltering;
}
bool KCurrentState::getisMovingForward()
{
  return BrainBotSensor.isMovingForward;
}
bool KCurrentState::getisMovingBackward()
{
  return BrainBotSensor.isMovingBackward;
}
bool KCurrentState::getisArrvied()
{
  return BrainBotSensor.isArrived;
}
bool KCurrentState::getisStopped()
{
  return BrainBotSensor.isStopped;
}
bool KCurrentState::getisGPSAveraging()
{
  return BrainBotSensor.isGPSAveraging;
}
bool KCurrentState::getisUpdatingInitialOrientation()
{
  return BrainBotSensor.isUpdatingInitialOrientation;
}
bool KCurrentState::getisUpdatingReferenceLocation()
{
  return BrainBotSensor.isUpdatingReferenceLocation;
}
bool KCurrentState::getisInitKalman()
{
  return BrainBotSensor.isInitKalman;
}
bool KCurrentState::getisUpdatingQ()
{
  return BrainBotSensor.isUpdatingQ;
}
bool KCurrentState::getisUpdatingR()
{
  return BrainBotSensor.isUpdatingR;
}
bool KCurrentState::getisRawdataLogging()
{
  return BrainBotSensor.isRawdataLogging;
}
bool KCurrentState::getisReinforceddataLogging()
{
  return BrainBotSensor.isReinforceddataLogging;
}
bool KCurrentState::getisAdvencing()
{
  return BrainBotSensor.isAdvencing;
}
long *KCurrentState::getPoint()
{
  //static long currentPoint[3] = {x, y, z};
  //return currentPoint;
  return 0;
}
long KCurrentState::getMovingDistance()
{
  return BrainBotSensor.movingDistance;
}



void KCurrentState::setAX(float newAX)
{
  BrainBotSensor.ax = newAX;
}

void KCurrentState::setAY(float newAY)
{
  BrainBotSensor.ay = newAY;
}

void KCurrentState::setAZ(float newAZ)
{
  BrainBotSensor.az = newAZ;
}

void KCurrentState::setGX(float newGX)
{
  BrainBotSensor.gx = newGX;
}

void KCurrentState::setGY(float newGY)
{
  BrainBotSensor.gy = newGY;
}

void KCurrentState::setGZ(float newGZ)
{
  BrainBotSensor.gz = newGZ;
}


void KCurrentState::setMX(float newMX)
{
  BrainBotSensor.mx = newMX;
}

void KCurrentState::setMY(float newMY)
{
  BrainBotSensor.my = newMY;
}

void KCurrentState::setMZ(float newMZ)
{
  BrainBotSensor.mz = newMZ;
}

void KCurrentState::setLat(double newLat)
{
  BrainBotSensor.latitude = newLat;
}

void KCurrentState::setLog(double newLog)
{
  BrainBotSensor.longitude = newLog;
}

void KCurrentState::setSpeed(double newSpeed)
{
  BrainBotSensor.speed = newSpeed;
}

void KCurrentState::setRoll(float newRoll)
{
  BrainBotSensor.roll_deg = newRoll;
}

void KCurrentState::setPitch(float newPitch)
{
  BrainBotSensor.pitch_deg = newPitch;
}

void KCurrentState::setHeading(float newHeading)
{
  BrainBotSensor.heading_deg = newHeading;
}

void KCurrentState::setInitRoll(float newRoll)
{
  BrainBotSensor.initRoll = newRoll;
}

void KCurrentState::setInitPitch(float newPitch)
{
  BrainBotSensor.initPitch = newPitch;
}

void KCurrentState::setInitHeading(float newHeading)
{
  BrainBotSensor.initHeading = newHeading;
}
void KCurrentState::setInitLat(double newinitLat)
{
  BrainBotSensor.initLat = newinitLat;
}
void KCurrentState::setInitLon(double newinitLon)
{
  BrainBotSensor.initLon = newinitLon;
}

void KCurrentState::setisIMUReady(bool newis)
{
  BrainBotSensor.isIMUReady = newis;
}

void KCurrentState::setisMotorReady(bool newis)
{
  BrainBotSensor.isMotorReady = newis;
}
void KCurrentState::setisGPSReady(bool newis)
{
  BrainBotSensor.isGPSReady = newis;
}
void KCurrentState::setisKalmanReady(bool newis)
{
  BrainBotSensor.isKalmanReady = newis;
}
void KCurrentState::setisKalmanFiltering(bool newis)
{
  BrainBotSensor.isKalmanFiltering = newis;
}
void KCurrentState::setisMovingForward(bool newis)
{
  BrainBotSensor.isMovingForward = newis;
  BrainBotSensor.isMovingBackward = !(newis);
}
void KCurrentState::setisMovingBackward(bool newis)
{
  BrainBotSensor.isMovingBackward = newis;
  BrainBotSensor.isMovingForward = !(newis);
}
void KCurrentState::setisArrived(bool newis)
{
  BrainBotSensor.isArrived = newis;
}
void KCurrentState::setisStopped(bool newis)
{
  BrainBotSensor.isStopped = newis;
  BrainBotSensor.isMovingBackward = !(newis);
  BrainBotSensor.isMovingForward = !(newis);
}
void KCurrentState::setisGPSAveraging(bool newis)
{
  BrainBotSensor.isGPSAveraging = newis;
}
void KCurrentState::setisUpdatingInitialOrientation(bool newis)
{
  BrainBotSensor.isUpdatingInitialOrientation = newis;
}
void KCurrentState::setisUpdatingReferenceLocation(bool newis)
{
  BrainBotSensor.isUpdatingReferenceLocation = newis;
}
void KCurrentState::setisInitKalman(bool newis)
{
  BrainBotSensor.isInitKalman = newis;
}
void KCurrentState::setisRawdataLogging(bool newis)
{
  BrainBotSensor.isRawdataLogging = newis;
}
void KCurrentState::setisReinforceddataLogging(bool newis)
{
  BrainBotSensor.isReinforceddataLogging = newis;
}
void KCurrentState::setisAdvencing(bool newis)
{
  BrainBotSensor.isAdvencing = newis;
}
void KCurrentState::setMovingSpeed(unsigned int newspeed)
{
  BrainBotSensor.movingSpeed = newspeed;
}
void KCurrentState::setisUpdatingQ(bool newis)
{
  BrainBotSensor.isUpdatingQ = newis;
}
void KCurrentState::setisUpdatingR(bool newis)
{
  BrainBotSensor.isUpdatingR = newis;
}
void KCurrentState::setCovQ(double newQ, long row, long col)
{
  BrainBotSensor.covQ = newQ;
  BrainBotSensor.row = row;
  BrainBotSensor.col = col;
}
void KCurrentState::setCovR(double newR, long row, long col){
  BrainBotSensor.covR = newR;
  BrainBotSensor.row = row;
  BrainBotSensor.col = col;
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

void KCurrentState::setMovingDistance(long movingDistance)
{
  BrainBotSensor.movingDistance = movingDistance;
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
  Serial.print(BrainBotSensor.latitude);
  Serial.print(" Longtidue");
  Serial.print(BrainBotSensor.longitude);
  Serial.print(" Speed(mph)");
  Serial.print(BrainBotSensor.speed);
  printQAndNewLine();
}
void KCurrentState::printAcceleration(){
  Serial.print("R82");
  Serial.print(" AX");
  Serial.print(BrainBotSensor.ax);
  Serial.print(" AY");
  Serial.print(BrainBotSensor.ay);
  Serial.print(" AZ");
  Serial.print(BrainBotSensor.az);
  printQAndNewLine();
}

void KCurrentState::printOrientation(){
  Serial.print("R82");
  Serial.print(" R");
  Serial.print(BrainBotSensor.roll_deg);
  Serial.print(" P");
  Serial.print(BrainBotSensor.pitch_deg);
  Serial.print(" H");
  Serial.print(BrainBotSensor.heading_deg);
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

// printHeader() - prints our eight column names to the top of our log file
void KCurrentState::printHeader(){
  File logFile = SD.open(KCurrentState::getInstance()->logFileName, FILE_WRITE); // Open the log file

  if (logFile) // If the log file opened, print our column names to the file
  {
    int i = 0;
    for (; i < LOG_COLUMN_COUNT; i++)
    {
      logFile.print(log_col_names[i]);
      if (i < LOG_COLUMN_COUNT - 1) // If it's anything but the last column
        logFile.print(','); // print a comma
      else // If it's the last column
        logFile.println(); // print a new line
    }
    logFile.close(); // close the file
  }
}

// updateFileName() - Looks through the log files already present on a card,
// and creates a new file with an incremented file index.
void KCurrentState::updateFileName(){
  int i = 0;
  for (; i < MAX_LOG_FILES; i++)
  {
    memset(KCurrentState::getInstance()->logFileName, 0, strlen(KCurrentState::getInstance()->logFileName)); // Clear logFileName string
    // Set logFileName
    sprintf(KCurrentState::getInstance()->logFileName, "%s%d.%s", LOG_FILE_PREFIX, i, LOG_FILE_SUFFIX);
    if (!SD.exists(KCurrentState::getInstance()->logFileName)) // If a file doesn't exist
    {
      break; // Break out of this loop. We found our index
    }
  }
  Serial.print("File name: ");
  Serial.println(KCurrentState::getInstance()->logFileName); // Debug print the file name
}