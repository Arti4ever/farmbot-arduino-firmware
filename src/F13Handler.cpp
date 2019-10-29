
/*
 * F13Handler.cpp
 *
 *  Created on: 2014/07/21
 *      Author: MattLech
 */

#include "F13Handler.h"

static F13Handler *instance;

F13Handler *F13Handler::getInstance()
{
  if (!instance)
  {
    instance = new F13Handler();
  };
  return instance;
};

F13Handler::F13Handler()
{
}

int F13Handler::execute(Command *command)
{

  if (LOGGING)
  {
    Serial.print("R99 HOME Z\r\n");
  }

  StepperControl::getInstance()->moveToCoords(0, 0, 0, 0, 0, 0, false, false, true);

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
  double yPos = (double)CurrentState::getInstance()->getY() / ParameterList::getInstance()->getValue(MOVEMENT_STEP_PER_MM_Y);

  // Move to home position. Then 3 times move away and move to home again.
  for (int stepNr = 0; stepNr < 7; stepNr++)
  {
    switch (stepNr)
    {
    case 0: StepperControl::getInstance()->moveToCoords(xPos, yPos, 0, 0, 0, 0, false, false, true); break;
    case 1: StepperControl::getInstance()->moveToCoords(xPos, yPos, moveAwayCoord, 0, 0, 0, false, false, false); break;
    case 2: StepperControl::getInstance()->moveToCoords(xPos, yPos, 0, 0, 0, 0, false, false, true); break;
    case 3: StepperControl::getInstance()->moveToCoords(xPos, yPos, moveAwayCoord, 0, 0, 0, false, false, false); break;
    case 4: StepperControl::getInstance()->moveToCoords(xPos, yPos, 0, 0, 0, 0, false, false, true); break;
    case 5: StepperControl::getInstance()->moveToCoords(xPos, yPos, moveAwayCoord, 0, 0, 0, false, false, false); break;
    case 6: StepperControl::getInstance()->moveToCoords(xPos, yPos, 0, 0, 0, 0, false, false, true); break;
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
