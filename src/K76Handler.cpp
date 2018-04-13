/*
 * K76Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K76Handler.h"

static K76Handler *instance;

K76Handler *K76Handler::getInstance()
{
  if (!instance)
  {
    instance = new K76Handler();
  };
  return instance;
};

K76Handler::K76Handler()
{
}

int K76Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Updating robot initialise Kalman filter \r\n");
  }

  KCurrentState::getInstance()->setisStopped(true);

  return 0;
}
