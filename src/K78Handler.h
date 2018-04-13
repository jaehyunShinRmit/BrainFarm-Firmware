/*
 * K78Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K78Handler_H_
#define K78Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K78Handler : public KCodeHandler
{
public:
  static K78Handler *getInstance();
  int execute(Command *);

private:
  K78Handler();
  K78Handler(K78Handler const &);
  void operator=(K78Handler const &);
};

#endif /* K78Handler_H_ */
