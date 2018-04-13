/*
 * K79Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#include "K79Handler.h"

static K79Handler *instance;

K79Handler *K79Handler::getInstance()
{
  if (!instance)
  {
    instance = new K79Handler();
  };
  return instance;
};

K79Handler::K79Handler()
{
}

int K79Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Report Averaging GPS... \r\n");
  }

  KCurrentState::getInstance()->setisGPSAveraging(true);

  return 0;
}
