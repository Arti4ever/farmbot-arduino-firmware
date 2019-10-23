#include "StepperControlEncoder.h"

StepperControlEncoder::StepperControlEncoder()
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

void StepperControlEncoder::test()
{
  /*
                Serial.print("R88 ");
                Serial.print("position ");
                Serial.print(position);
                Serial.print(" channel A ");
                Serial.print(prvValChannelA);
                Serial.print(" -> ");
                Serial.print(curValChannelA);
                Serial.print(" channel B ");
                Serial.print(prvValChannelB);
                Serial.print(" -> ");
                Serial.print(curValChannelB);
                Serial.print("\r\n");
*/
}

void StepperControlEncoder::loadPinNumbers(int channelA, int channelB, int channelAQ, int channelBQ)
{
  pinChannelA = channelA;
  pinChannelB = channelB;
  pinChannelAQ = channelAQ;
  pinChannelBQ = channelBQ;

  readChannels();
  shiftChannels();
}

void StepperControlEncoder::loadSettings(int encType, long scaling, int invert)
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
void StepperControlEncoder::loadMdlEncoderId(MdlSpiEncoders encoder)
{
  mdlEncoder = encoder;
}
#endif

void StepperControlEncoder::setPosition(long newPosition)
{
  #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
    if (newPosition == 0)
    {
      position = newPosition;

      const byte reset_cmd = 0x00;

      digitalWrite(NSS_PIN, LOW);
      SPI.transfer(reset_cmd | (mdlEncoder << mdl_spi_encoder_offset));
      digitalWrite(NSS_PIN, HIGH);
    }
  #elif defined(STEPPER_HAS_ENCODER)
    position = newPosition;
  #endif
}

long StepperControlEncoder::currentPosition()
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

long StepperControlEncoder::currentPositionRaw()
{
    return position * encoderInvert;
}

void StepperControlEncoder::checkEncoder(bool channelA, bool channelB, bool channelAQ, bool channelBQ)
{
  #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
    processEncoder();
  #elif defined(STEPPER_HAS_ENCODER)
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

void StepperControlEncoder::processEncoder()
{
  // If using farmduino, revision 1.4, use the SPI interface to read from the Motor Dynamics Lab chip
  #if defined(BOARD_HAS_DYNAMICS_LAB_CHIP)
    const byte read_cmd = 0x0F;
    int readSize = 4;
    long encoderVal = 0;

    digitalWrite(NSS_PIN, LOW);
    SPI.transfer(read_cmd | (mdlEncoder << mdl_spi_encoder_offset));
    delayMicroseconds(10);

    for (int i = 0; i < readSize; ++i)
    {
      encoderVal <<= 8;
      encoderVal |= SPI.transfer(0x01);
    }

    digitalWrite(NSS_PIN, HIGH);
    position = encoderVal;

  #elif defined(STEPPER_HAS_ENCODER)

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

void StepperControlEncoder::readChannels()
{
#if !defined(BOARD_HAS_DYNAMICS_LAB_CHIP) && defined(STEPPER_HAS_ENCODER)
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

void StepperControlEncoder::setChannels(bool channelA, bool channelB, bool channelAQ, bool channelBQ)
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

void StepperControlEncoder::shiftChannels()
{

  // Save the current enoder status to later on compare with new values

  prvValChannelA = curValChannelA;
  prvValChannelB = curValChannelB;
}
