/*
 * K70Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K70Handler_H_
#define K70Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K70Handler : public KCodeHandler
{
public:
  static K70Handler *getInstance();
  int execute(Command *);

private:
  K70Handler();
  K70Handler(K70Handler const &);
  void operator=(K70Handler const &);
};

#endif /* K70Handler_H_ */
