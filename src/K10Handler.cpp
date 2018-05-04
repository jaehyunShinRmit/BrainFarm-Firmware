/*
 * K10Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "K10Handler.h"

static K10Handler *instance;

K10Handler *K10Handler::getInstance()
{
  if (!instance)
  {
    instance = new K10Handler();
  };
  return instance;
};

K10Handler::K10Handler()
{
}

int K10Handler::execute(Command *command)
{
  if (LOGGING)
  {
    Serial.print("Stop data Logging \r\n");
  }

  KCurrentState::getInstance()->setisRawdataLogging(false);
  KCurrentState::getInstance()->setisReinforceddataLogging(false);
  return 0;
}
