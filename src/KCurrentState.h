/*
 * KCurrentState.h
 *  Created on: 15 maj 2014
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#ifndef KCURRENTSTATE_H_
#define KCURRENTSTATE_H_
#include "Arduino.h"
#include <SD.h>
#include "pins.h"
#define ADVANCE 0 
#define RETREAT 1
#define LEFT    2
#define RIGHT   3

class KCurrentState
{
public:
  static KCurrentState *getInstance();

  //SD card
  void printHeader();
  void printRawHeader();
  void updateFileName();
  // Keep in mind, the SD library has max file name lengths of 8.3 - 8 char prefix,
  // and a 3 char suffix.
  // Our log files are called "logXX.csv, so "gpslog99.csv" is our max file.
  


  char *getlogFileName();
  float getAX();
  float getAY();
  float getAZ();
  float getGX();
  float getGY();
  float getGZ();
  float getMX();
  float getMY();
  float getMZ();
  double getLat();
  double getLog();
  double getSpeed();
  double getCovQ();
  double getCovR();
  long getRow();
  long getCol();
  long getMovingDistance();
  long getMovingDirection();
  float getRoll();
  float getPitch();
  float getHeading();
  bool getisIMUReady();
  bool getisMotorReady();
  bool getisGPSReady();
  bool getisKalmanReady();
  bool getisKalmanFiltering();
  bool getisInitKalman();
  bool getisMovingForward();
  bool getisMovingBackward();
  bool getisArrvied();
  bool getisStopped();
  bool getisGPSAveraging();
  bool getisUpdatingInitialOrientation();
  bool getisUpdatingReferenceLocation();
  bool getisUpdatingQ();
  bool getisUpdatingR();
  bool getisRawdataLogging();
  bool getisReinforceddataLogging();
  bool getisMoving();

  void setAX(float);
  void setAY(float);
  void setAZ(float);
  void setGX(float);
  void setGY(float);
  void setGZ(float);
  void setMX(float);
  void setMY(float);
  void setMZ(float);
  void setLat(double);
  void setLog(double);
  void setSpeed(double);
  void setRoll(float);
  void setPitch(float);
  void setHeading(float);
  void setInitRoll(float);
  void setInitPitch(float);
  void setInitHeading(float);
  void setInitLat(double);
  void setInitLon(double);
  void setisIMUReady(bool);
  void setisMotorReady(bool);
  void setisGPSReady(bool);
  void setisKalmanReady(bool);
  void setisKalmanFiltering(bool);
  void setisMovingForward(bool);
  void setisMovingBackward(bool);
  void setisArrived(bool);
  void setisStopped(bool);
  void setisGPSAveraging(bool);
  void setisUpdatingInitialOrientation(bool);
  void setisUpdatingReferenceLocation(bool);
  void setisInitKalman(bool);
  void setisUpdatingQ(bool);
  void setisUpdatingR(bool);
  void setMovingSpeed(unsigned int);
  void setisRawdataLogging(bool);
  void setisReinforceddataLogging(bool);
  void setisMoving(bool);
  void setCovQ(double,long,long);
  void setCovR(double,long,long);
  void setMovingDistance(long);
  void setMovingDirection(long);
  long *getPoint();
  int getLastError();
  void setLastError(int error);

  void setEndStopState(unsigned int, unsigned int, bool);
  void printPosition();
  void printAcceleration();
  void printOrientation();
  
  String getPosition();
  void storeEndStops();
  void printEndStops();
  void print();
  void printBool(bool);

  void setQ(long);
  void resetQ();
  void printQAndNewLine();
  String getQAndNewLine();

  void setEmergencyStop();
  void resetEmergencyStop();
  bool isEmergencyStop();

  void setStepsPerMm(long stepsX, long stepsY, long stepsZ);

private:
  KCurrentState();
  KCurrentState(KCurrentState const &);
  void operator=(KCurrentState const &);

  long stepsPerMmX;
  long stepsPerMmY;
  long stepsPerMmZ;

  bool emergencyStop = false;
};

#endif /* KCurrentState_H_ */
