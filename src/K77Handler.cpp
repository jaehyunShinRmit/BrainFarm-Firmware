/*
 * K77Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K77Handler.h"

static K77Handler *instance;

K77Handler *K77Handler::getInstance()
{
  if (!instance)
  {
    instance = new K77Handler();
  };
  return instance;
};

K77Handler::K77Handler()
{
}

int K77Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Start Kalman filtering  \r\n");
  }

  KCurrentState::getInstance()->setisKalmanFiltering(true);

  return 0;
}
