/*
 * K11Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "K11Handler.h"

static K11Handler *instance;

K11Handler *K11Handler::getInstance()
{
  if (!instance)
  {
    instance = new K11Handler();
  };
  return instance;
};

K11Handler::K11Handler()
{
}

int K11Handler::execute(Command *command)
{
  if (LOGGING)
  {
    Serial.print("Raw data Logging start \r\n");
  }

  KCurrentState::getInstance()->setisRawdataLogging(true);

  return 0;
}
