/*
 * K16Handler.h
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 4/05/2018
 */

#ifndef K16Handler_H_
#define K16Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K16Handler : public KCodeHandler
{
public:
  static K16Handler *getInstance();
  int execute(Command *);

private:
  K16Handler();
  K16Handler(K16Handler const &);
  void operator=(K16Handler const &);
};

#endif /* K16Handler_H_ */
