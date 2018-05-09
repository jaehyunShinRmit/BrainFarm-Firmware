/*
 * K13Handler.h
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 27/04/2018
 */

#ifndef K13Handler_H_
#define K13Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K13Handler : public KCodeHandler
{
public:
  static K13Handler *getInstance();
  int execute(Command *);

private:
  K13Handler();
  K13Handler(K13Handler const &);
  void operator=(K13Handler const &);
};

#endif /* K12Handler_H_ */
