/*
 * K15Handler.cpp
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#include "K15Handler.h"

static K15Handler *instance;

K15Handler *K15Handler::getInstance()
{
  if (!instance)
  {
    instance = new K15Handler();
  };
  return instance;
};

K15Handler::K15Handler()
{
}

int K15Handler::execute(Command *command)
{
  Serial.print("K15 Turn Right the robot\r\n");


  KCurrentState::getInstance()->setMovingDistance(command->getMD());
  KCurrentState::getInstance()->setMovingDirection(RIGHT);
  KCurrentState::getInstance()->setisMoving(true);
  return 0;
}
