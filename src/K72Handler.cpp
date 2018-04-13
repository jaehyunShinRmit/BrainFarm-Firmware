/*
 * K72Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K72Handler.h"

static K72Handler *instance;

K72Handler *K72Handler::getInstance()
{
  if (!instance)
  {
    instance = new K72Handler();
  };
  return instance;
};

K72Handler::K72Handler()
{
}

int K72Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Initialise Kalman filter \r\n");
  }

  KCurrentState::getInstance()->setisInitKalman(true);

  return 0;
}
