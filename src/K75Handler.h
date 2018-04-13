/*
 * K75Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K75Handler_H_
#define K75Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K75Handler : public KCodeHandler
{
public:
  static K75Handler *getInstance();
  int execute(Command *);

private:
  K75Handler();
  K75Handler(K75Handler const &);
  void operator=(K75Handler const &);
};

#endif /* K75Handler_H_ */
