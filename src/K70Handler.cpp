/*
 * K70Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K70Handler.h"

static K70Handler *instance;

K70Handler *K70Handler::getInstance()
{
  if (!instance)
  {
    instance = new K70Handler();
  };
  return instance;
};

K70Handler::K70Handler()
{
}

int K70Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Updating robot initial orientation \r\n");
  }

  KCurrentState::getInstance()->setisUpdatingInitialOrientation(true);

  return 0;
}
