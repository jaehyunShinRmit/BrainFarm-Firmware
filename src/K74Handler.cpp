/*
 * K74Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K74Handler.h"

static K74Handler *instance;

K74Handler *K74Handler::getInstance()
{
  if (!instance)
  {
    instance = new K74Handler();
  };
  return instance;
};

K74Handler::K74Handler()
{
}

int K74Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Advance robot\r\n");
  }

  KCurrentState::getInstance()->setisMovingForward(true);

  return 0;
}
