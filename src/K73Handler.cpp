/*
 * K73Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K73Handler.h"

static K73Handler *instance;

K73Handler *K73Handler::getInstance()
{
  if (!instance)
  {
    instance = new K73Handler();
  };
  return instance;
};

K73Handler::K73Handler()
{
}

int K73Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Setting moving speed \r\n");
  }

  KCurrentState::getInstance()->setMovingSpeed(command->getMS());

  return 0;
}
