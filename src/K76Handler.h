/*
 * K76Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K76Handler_H_
#define K76Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K76Handler : public KCodeHandler
{
public:
  static K76Handler *getInstance();
  int execute(Command *);

private:
  K76Handler();
  K76Handler(K76Handler const &);
  void operator=(K76Handler const &);
};

#endif /* K76Handler_H_ */
