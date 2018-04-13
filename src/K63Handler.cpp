/*
 * K63Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#include "K63Handler.h"

static K63Handler *instance;

K63Handler *K63Handler::getInstance()
{
  if (!instance)
  {
    instance = new K63Handler();
  };
  return instance;
};

K63Handler::K63Handler()
{
}

int K63Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 Setting measurement noise covriance \r\n");
    // Serial.print("Q :");
    // Serial.print(command->getsysCovariance(),5);
    // Serial.print("Col : ");
    // Serial.print(command->getCol());
    // Serial.print("Row : ");
    // Serial.print(command->getRow());
  }
  KCurrentState::getInstance()->setisUpdatingR(true);
  KCurrentState::getInstance()->setisInitKalman(true);
  KCurrentState::getInstance()->setCovR(command->getmeasureCovariance(),command->getRow(),command->getCol());

  return 0;
}
