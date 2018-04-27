/*
 * K12Handler.h
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#ifndef K12Handler_H_
#define K12Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K12Handler : public KCodeHandler
{
public:
  static K12Handler *getInstance();
  int execute(Command *);

private:
  K12Handler();
  K12Handler(K12Handler const &);
  void operator=(K12Handler const &);
};

#endif /* K12Handler_H_ */
