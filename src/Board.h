/**
 * BOARD selection 
 */
#ifndef FARMBOT_BOARD_ID

  //#define RAMPS_V14
  //#define FARMDUINO_V10
  //#define FARMDUINO_V14
  //#define FARMDUINO_EXP_V20
  #define FYSETC_F6

#else

  #undef RAMPS_V14
  #undef FARMDUINO_V10
  #undef FARMDUINO_V14
  #undef FARMDUINO_EXP_V20

  #if FARMBOT_BOARD_ID == 0
    #define RAMPS_V14
  #elif FARMBOT_BOARD_ID == 1
    #define FARMDUINO_V10
  #elif FARMBOT_BOARD_ID == 2
    #define FARMDUINO_V14
  #elif FARMBOT_BOARD_ID == 3
    #define FARMDUINO_EXP_V20
  #endif

#endif


#if defined(RAMPS_V14)
  #undef STEPPER_HAS_ENCODER
  #undef BOARD_HAS_TMC2130_DRIVER 
#endif

#if defined(FARMDUINO_V10)
  #define STEPPER_HAS_ENCODER
  #undef BOARD_HAS_TMC2130_DRIVER 
#endif

#if defined(FARMDUINO_V14)
  #define STEPPER_HAS_ENCODER
  #undef BOARD_HAS_TMC2130_DRIVER 
#endif

#if defined(FARMDUINO_EXP_V20)
  #undef STEPPER_HAS_ENCODER
  #define BOARD_HAS_TMC2130_DRIVER 
#endif

#if defined(FYSETC_F6)
  #undef STEPPER_HAS_ENCODER
  #define BOARD_HAS_TMC2130_DRIVER 
#endif

