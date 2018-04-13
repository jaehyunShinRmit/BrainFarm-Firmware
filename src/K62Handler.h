/*
 * K62Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 20/02/2018
 */

#ifndef K62Handler_H_
#define K62Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K62Handler : public KCodeHandler
{
public:
  static K62Handler *getInstance();
  int execute(Command *);

private:
  K62Handler();
  K62Handler(K62Handler const &);
  void operator=(K62Handler const &);
};

#endif /* K62Handler_H_ */
