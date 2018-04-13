/*
 * K61Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "K61Handler.h"

static K61Handler *instance;

K61Handler *K61Handler::getInstance()
{
  if (!instance)
  {
    instance = new K61Handler();
  };
  return instance;
};

K61Handler::K61Handler()
{
}

int K61Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Heading \r\n");
  }

  KCurrentState::getInstance()->printPosition();

  return 0;
}
