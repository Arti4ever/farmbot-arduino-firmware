/*
 * MovementEncoder.h
 *
 *  Created on: 29 okt 2015
 *      Author: Tim Evers
 */

#ifndef STEPPERCONTROLENCODER_H_
#define STEPPERCONTROLENCODER_H_

#include "Arduino.h"
//#include "CurrentState.h"
//#include "ParameterList.h"
#include "pins.h"
#include "Config.h"
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>

class MovementEncoder
{

public:
  MovementEncoder();

  void loadPinNumbers(int channelA, int channelB, int channelAQ, int channelBQ);
  void loadSettings(int encType, long scaling, int invert);

#if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
  // Load the id for the motor dynamics lab encoder
  void loadMdlEncoderId(MdlSpiEncoders encoder);
#endif

  void setPosition(long newPosition);
  long currentPosition();
  long currentPositionRaw();

  void checkEncoder(bool channelA, bool channelB, bool channelAQ, bool channelBQ);
  void processEncoder();
  void readChannels();
  void setChannels(bool channelA, bool channelB, bool channelAQ, bool channelBQ);
  void shiftChannels();

private:
  // pin settings
  int pinChannelA;
  int pinChannelAQ;
  int pinChannelB;
  int pinChannelBQ;

  // channels
  bool prvValChannelA;
  bool prvValChannelB;
  bool curValChannelA;
  bool curValChannelB;

  bool readChannelA;
  bool readChannelAQ;
  bool readChannelB;
  bool readChannelBQ;

  // encoder
  long position;
  long scalingFactor;
  float floatScalingFactor;
  int encoderType;
  int encoderInvert;

#if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
  MdlSpiEncoders mdlEncoder = _MDL_X1;
#endif

};

#endif /* STEPPERCONTROLENCODER_H_ */
