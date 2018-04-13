/*
 * K62Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "K62Handler.h"

static K62Handler *instance;

K62Handler *K62Handler::getInstance()
{
  if (!instance)
  {
    instance = new K62Handler();
  };
  return instance;
};

K62Handler::K62Handler()
{
}

int K62Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Setting system noise covariance \r\n");
  }
  KCurrentState::getInstance()->setisUpdatingQ(true);
  KCurrentState::getInstance()->setisInitKalman(true);
  KCurrentState::getInstance()->setCovQ(command->getsysCovariance(),command->getRow(),command->getCol());

  return 0;
}
