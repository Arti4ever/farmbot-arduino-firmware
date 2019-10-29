
/*
 * F12Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 */

#include "F12Handler.h"

static F12Handler *instance;

F12Handler *F12Handler::getInstance()
{
  if (!instance)
  {
    instance = new F12Handler();
  };
  return instance;
};

F12Handler::F12Handler()
{
}

int F12Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 HOME Y\r\n");
  }

  int homeIsUp = ParameterList::getInstance()->getValue(MOVEMENT_HOME_UP_X);
  int moveAwayCoord = 10;
  int execution;
  bool emergencyStop;

  if (homeIsUp == 1)
  {
    moveAwayCoord = -moveAwayCoord;
  }

  // save current position
  double xPos = (double)CurrentState::getInstance()->getX() / ParameterList::getInstance()->getValue(MOVEMENT_STEP_PER_MM_X);
  double zPos = (double)CurrentState::getInstance()->getZ() / ParameterList::getInstance()->getValue(MOVEMENT_STEP_PER_MM_Z);

  // Move to home position. Then 3 times move away and move to home again.
  for (int stepNr = 0; stepNr < 7; stepNr++)
  {
    switch (stepNr)
    {
    case 0: StepperControl::getInstance()->moveToCoords(xPos, 0, zPos, 0, 0, 0, false, true, false); break;
    case 1: StepperControl::getInstance()->moveToCoords(xPos, moveAwayCoord, zPos, 0, 0, 0, false, false, false); break;
    case 2: StepperControl::getInstance()->moveToCoords(xPos, 0, zPos, 0, 0, 0, false, true, false); break;
    case 3: StepperControl::getInstance()->moveToCoords(xPos, moveAwayCoord, zPos, 0, 0, 0, false, false, false); break;
    case 4: StepperControl::getInstance()->moveToCoords(xPos, 0, zPos, 0, 0, 0, false, true, false); break;
    case 5: StepperControl::getInstance()->moveToCoords(xPos, moveAwayCoord, zPos, 0, 0, 0, false, false, false); break;
    case 6: StepperControl::getInstance()->moveToCoords(xPos, 0, zPos, 0, 0, 0, false, true, false); break;
    }

    execution = CurrentState::getInstance()->getLastError();
    emergencyStop = CurrentState::getInstance()->isEmergencyStop();

    if (emergencyStop || execution != 0)
    {
      break;
    }
  }

  if (LOGGING)
  {
    CurrentState::getInstance()->print();
  }
  return 0;
}
