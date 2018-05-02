/*
 * K10Handler.h
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#ifndef K10Handler_H_
#define K10Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K10Handler : public KCodeHandler
{
public:
  static K10Handler *getInstance();
  int execute(Command *);

private:
  K10Handler();
  K10Handler(K10Handler const &);
  void operator=(K10Handler const &);
};

#endif /* K10Handler_H_ */
