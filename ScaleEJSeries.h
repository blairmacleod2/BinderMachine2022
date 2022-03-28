/*
  ScaleEJSeries.cpp - simple library to interface with AND EJ series of weigh scales using their RS-232 port
  Created by Matthew M. Sylvester, June 25, 2019.
  3DEO, Inc
*/

#ifndef __SCALE_EJ_SERIES_H
#define __SCALE_EJ_SERIES_H

#include <Arduino.h>

class Scale{
public:
  Scale();
  void receiveData();
  
  char commandStrings[3][4] = {
    "Q\r\n",
    "Z\r\n",
    "U\r\n"
  };

  enum Command {
    REQUEST,
    TARE,
    MODE
  };

  char headerStrings[4][3] = {
    "ST",
    "QT",
    "US",
    "OL"
  };

  enum Header {
    STABLE_WEIGH,
    STABLE_COUNT,
    UNSTABLE_WEIGH,
    OUT_OF_RANGE,
    HEADER_ERROR
  };

  char unitStrings[14][4] = {
    "  g",
    " PC",
    "  %",
    " oz",
    " lb",
    "ozt",
    " ct",
    "mom",
    "dwt",
    " GN",
    "  N",
    " tl",
    "  t",
    " DS"
  };

  enum Units {
    GRAMS,
    PCS,
    PERCENT,
    OUNCE,
    POUND,
    TROY_OUNCE,
    CARAT,
    MOMME,
    PENNYWEIGHT,
    GRAIN,
    NEWTON,
    TAEL,
    TOLA,
    SPECIFIC_GRAVITY,
    UNIT_ERROR
  };

  Header receivedHeader;
  Units receivedUnits;
  bool newData = 0;
  float measureData;
  void sendCommand(Command); 

private:
  void parseData(char* frame);
};

#endif
