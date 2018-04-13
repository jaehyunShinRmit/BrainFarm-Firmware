/*
 * K75Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K75Handler.h"

static K75Handler *instance;

K75Handler *K75Handler::getInstance()
{
  if (!instance)
  {
    instance = new K75Handler();
  };
  return instance;
};

K75Handler::K75Handler()
{
}

int K75Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Move Backward \r\n");
  }

  KCurrentState::getInstance()->setisMovingBackward(true);

  return 0;
}
