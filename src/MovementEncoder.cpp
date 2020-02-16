#include "MovementEncoder.h"

MovementEncoder::MovementEncoder()
{
  //lastCalcLog	= 0;

  pinChannelA = 0;
  pinChannelB = 0;

  position = 0;
  encoderType = 0; // default type
  scalingFactor = 10000;

  curValChannelA = false;
  curValChannelA = false;
  prvValChannelA = false;
  prvValChannelA = false;

  readChannelA = false;
  readChannelAQ = false;
  readChannelB = false;
  readChannelBQ = false;

#if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
  mdlEncoder = _MDL_X1;
#endif
}

void MovementEncoder::loadPinNumbers(int channelA, int channelB, int channelAQ, int channelBQ)
{
  pinChannelA = channelA;
  pinChannelB = channelB;
  pinChannelAQ = channelAQ;
  pinChannelBQ = channelBQ;

  readChannels();
  shiftChannels();
}

void MovementEncoder::loadSettings(int encType, long scaling, int invert)
{
  encoderType = encType;
  scalingFactor = scaling;
  if (invert == 1)
  {
    encoderInvert = -1;
  }
  else
  {
    encoderInvert = 1;
  }

//  encoderType = 0; // TEVE 2017-04-20 Disabling the differential channels. They take too much time to read.
}

#if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
void MovementEncoder::loadMdlEncoderId(MdlSpiEncoders encoder)
{
  mdlEncoder = encoder;
}
#endif

void MovementEncoder::setPosition(long newPosition)
{
  #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
    if (newPosition == 0)
    {
      position = newPosition;

      const byte reset_cmd = 0x00;

      SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
      digitalWrite(NSS_PIN, LOW);
      delayMicroseconds(2);
      SPI.transfer(reset_cmd | (mdlEncoder << mdl_spi_encoder_offset));
      delayMicroseconds(5);
      digitalWrite(NSS_PIN, HIGH);
      SPI.endTransaction();
    }
  #elif defined(BOARD_HAS_ENCODER)
    position = newPosition;
  #endif
}

long MovementEncoder::currentPosition()
{


  // Apply scaling to the output of the encoder, except when scaling is zero or lower
  if (scalingFactor == 10000 || scalingFactor <= 0)
  {
    return position * encoderInvert;
  }
  else
  {
    #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
      floatScalingFactor = scalingFactor / 40000.0;
      return position * floatScalingFactor * encoderInvert;
    #endif
    floatScalingFactor = scalingFactor / 10000.0;
    return position * floatScalingFactor * encoderInvert;
  }
}

long MovementEncoder::currentPositionRaw()
{
    return position * encoderInvert;
}

void MovementEncoder::checkEncoder(bool channelA, bool channelB, bool channelAQ, bool channelBQ)
{
  #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
    processEncoder();
  #elif defined(BOARD_HAS_ENCODER)
    shiftChannels();
    setChannels(channelA, channelB, channelAQ, channelBQ);
    processEncoder();
  #endif
}


/* Check the encoder channels for movement according to this specification
                    ________            ________
Channel A          /        \          /        \
             _____/          \________/          \________
                         ________            ________
Channel B               /        \          /        \
             __________/          \________/          \____
                                   __
Channel I                         /  \
             ____________________/    \___________________

rotation ----------------------------------------------------->

*/

void MovementEncoder::processEncoder()
{
  // If using farmduino, revision 1.4, use the SPI interface to read from the Motor Dynamics Lab chip
  #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
    const byte read_cmd = 0x0F;
    int readSize = 4;
    long encoderVal = 0;

    SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
    digitalWrite(NSS_PIN, LOW);
    delayMicroseconds(2);
    SPI.transfer(read_cmd | (mdlEncoder << mdl_spi_encoder_offset));
    delayMicroseconds(5);

    for (int i = 0; i < readSize; ++i)
    {
      encoderVal <<= 8;
      encoderVal |= SPI.transfer(0x01);
    }

    digitalWrite(NSS_PIN, HIGH);
    SPI.endTransaction();
    position = encoderVal;

  #elif defined(BOARD_HAS_ENCODER)

    // Detect edges on the A channel when the B channel is high
    if (curValChannelB == true && prvValChannelA == false && curValChannelA == true)
    {
      //delta--;
      position--;
    }
    if (curValChannelB == true && prvValChannelA == true && curValChannelA == false)
    {
      //delta++;
      position++;
    }
  #endif

}

void MovementEncoder::readChannels()
{
#if !defined(BOARD_HAS_DYNAMICS_LAB_CHIP) && defined(BOARD_HAS_ENCODER)
  // read the new values from the coder

  readChannelA = digitalRead(pinChannelA);
  readChannelB = digitalRead(pinChannelB);

  if (encoderType == 1)
  {
    readChannelAQ = digitalRead(pinChannelAQ);
    readChannelBQ = digitalRead(pinChannelBQ);

    // differential encoder
    if ((readChannelA ^ readChannelAQ) && (readChannelB ^ readChannelBQ))
    {
      curValChannelA = readChannelA;
      curValChannelB = readChannelB;
    }
  }
  else
  {

    // encoderType <= 0
    // non-differential incremental encoder
    curValChannelA = readChannelA;
    curValChannelB = readChannelB;
  }
#endif

}

void MovementEncoder::setChannels(bool channelA, bool channelB, bool channelAQ, bool channelBQ)
{
  // read the new values from the coder

  if (encoderType == 1)
  {
    // differential encoder
    if ((channelA ^ channelAQ) && (channelB ^ channelBQ))
    {
      curValChannelA = channelA;
      curValChannelB = channelB;
    }
  }
  else
  {
    // encoderType <= 0
    // non-differential incremental encoder
    curValChannelA = channelA;
    curValChannelB = channelB;
  }
}

void MovementEncoder::shiftChannels()
{

  // Save the current enoder status to later on compare with new values

  prvValChannelA = curValChannelA;
  prvValChannelB = curValChannelB;
}


void MovementEncoder::setEnable(bool enable)
{
  encoderEnabled = enable;
}

void MovementEncoder::setStepDecay(float stepDecay)
{
  encoderStepDecay = stepDecay;
}

void MovementEncoder::setMovementDirection(bool up)
{
  encoderMovementUp = up;
}

float MovementEncoder::getMissedSteps()
{
  return missedSteps;
}

void MovementEncoder::checkMissedSteps()
{
  #if defined(BOARD_HAS_ENCODER)
    if (encoderEnabled)
    {
      bool stepMissed = false;

      // Decrease amount of missed steps if there are no missed step
      if (missedSteps > 0)
      {
        (missedSteps) -= (encoderStepDecay);
      }

      // Check if the encoder goes in the wrong direction or nothing moved
      if ((encoderMovementUp && encoderLastPosition > currentPositionRaw()) ||
        (!encoderMovementUp && encoderLastPosition < currentPositionRaw()))
      {
        stepMissed = true;
      }

      if (stepMissed && missedSteps < 32000)
      {
        (missedSteps)++;
      }

      encoderLastPosition = currentPositionRaw();
      //axis->setLastPosition(axis->currentPosition());
   }

  #endif

/*
  #if defined(FARMDUINO_EXP_V20) || defined(FARMDUINO_V30)

    if (encoderEnabled) {
      if (axis->stallDetected()) {
        // In case of stall detection, count this as a missed step
        (*missedSteps)++;
        //axis->setCurrentPosition(*lastPosition);
      }
      else {
        // Decrease amount of missed steps if there are no missed step
        if (missedSteps > 0)
        {
          (missedSteps) -= (encoderStepDecay);
        }
        setPosition(axis->currentPosition());
        //axis->setLastPosition(axis->currentPosition());
      }
    }
  #endif
*/
}
