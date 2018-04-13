/*
 * K77Handler.h
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 *      Modified by Jaehyun Shin 02/03/2018
 */

#ifndef K77Handler_H_
#define K77Handler_H_
#include "KCodeHandler.h"
#include "Config.h"
#include "KCurrentState.h"
#include "pins.h"
#include "Config.h"


class K77Handler : public KCodeHandler
{
public:
  static K77Handler *getInstance();
  int execute(Command *);

private:
  K77Handler();
  K77Handler(K77Handler const &);
  void operator=(K77Handler const &);
};

#endif /* K77Handler_H_ */
