/*
 * K12Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#include "K12Handler.h"

static K12Handler *instance;

K12Handler *K12Handler::getInstance()
{
  if (!instance)
  {
    instance = new K12Handler();
  };
  return instance;
};

K12Handler::K12Handler()
{
}

int K12Handler::execute(Command *command)
{
  if (LOGGING)
  {
    Serial.print("Advance the robot \r\n");
  }

  KCurrentState::getInstance()->setisAdvencing(true);

  return 0;
}
