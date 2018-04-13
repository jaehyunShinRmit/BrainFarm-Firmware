/*
 * K78Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K78Handler.h"

static K78Handler *instance;

K78Handler *K78Handler::getInstance()
{
  if (!instance)
  {
    instance = new K78Handler();
  };
  return instance;
};

K78Handler::K78Handler()
{
}

int K78Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Stop Kalman filter \r\n");
  }

  KCurrentState::getInstance()->setisKalmanFiltering(false);

  return 0;
}
