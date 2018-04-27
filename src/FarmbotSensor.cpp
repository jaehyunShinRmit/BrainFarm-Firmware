#include "FarmbotSensor.h"

FarmbotSensor::FarmbotSensor()
{
  //Initialization
  initRoll = 0;
  initPitch = 0;
  initHeading = 0;
  roll_deg = 0;
  pitch_deg = 0;
  heading_deg = 0;
  roll = 0;
  pitch = 0;
  heading = 0;
  ax = 0;
  ay = 0;
  az = 0;
  gx = 0;
  gy = 0;
  gz = 0;
  mx = 0;
  my = 0;
  mz = 0;
  latitude  = 0;
  longitude = 0;
  speed     = 0;
  CorrectedAcc =0;
  vx =0;
  px =0;
  E=0; //mm
  N=0; //mm
  P=0; //deg or rad
  V=0; //mm/s
  dP=0; // deg/s or rad/s
  A=0; // mm/s^2
  isInitKalman = false;
  isIMUReady = false;
  isMotorReady= false;
  isGPSReady= false;
  isKalmanReady= false;
  isKalmanFiltering= false;
  isMovingForward= false;
  isMovingBackward= false;
  isStopped= false;
  isArrived= false;
  isGPSAveraging= false;
  isUpdatingInitialOrientation= false;
  isUpdatingReferenceLocation= false;
  isInitKalman= false;  
  isUpdatingQ= false;
  isUpdatingR= false;
  isRawdataLogging =false;
  isAdvencing = false;
  covR = 0.001;
  covQ = 0.01;
  row = 0;
  col = 0;
}
