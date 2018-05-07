/*
 * KCurrentState.h
 *  Created on: 15 maj 2014
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#ifndef KCURRENTSTATE_H_
#define KCURRENTSTATE_H_
#include "Arduino.h"
#include "pins.h"

class KCurrentState
{
public:
  static KCurrentState *getInstance();

  //SD card
  void printHeader();
  void updateFileName();
  // Keep in mind, the SD library has max file name lengths of 8.3 - 8 char prefix,
  // and a 3 char suffix.
  // Our log files are called "logXX.csv, so "gpslog99.csv" is our max file.
  
  #define LOG_FILE_PREFIX "log" // Name of the log file.
  #define MAX_LOG_FILES 100000 // Number of log files that can be made
  #define LOG_FILE_SUFFIX "csv" // Suffix of the log file
  #define LOG_COLUMN_COUNT 14

  char logFileName[13]; // Char string to store the log file name
  // Data to be logged:
  const char * log_col_names[LOG_COLUMN_COUNT] = {
  "ms", "E(mm)", "N(mm)", "V(mm/s)","P(rad)","DP(rad/s)", "A(mm/s^2)","Heading(IMU)","roll(deg)", "pitch(deg)", "latitude" ,"longitude","Speed(GPS)","Heading(GPS)"
  }; // log_col_names is printed at the top of the file.
  const char * log_col_names_raw[LOG_COLUMN_COUNT] = {
  "ms", "ax(m/s^2)", "ay(m/s^2)", "az(mm/s^2)","gx(rad/s)","gy(rad/s)", "gy(rad/s)","Roll(deg)","pitch(deg)", "heading(deg)", "latitude" ,"longitude","Speed(GPS)","Heading(GPS)"
  }; // log_col_names is printed at the top of the file.

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
  bool getisAdvencing();

  void setAX(float);
  void setAY(float);
  void setAZ(float);
  void setGX(float);
  void setGY(float);
  void setGZ(float);
  void setMX(float);
  void setMY(float);
  void setMZ(float);
  void setMovingDistance(long);
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
  void setisAdvencing(bool);
  void setCovQ(double,long,long);
  void setCovR(double,long,long);
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
