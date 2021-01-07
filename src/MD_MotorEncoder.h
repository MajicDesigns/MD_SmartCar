#pragma once
/**
 * \file
 * \brief Header file for the MD_MotorEncoder class.
 */

#include <Arduino.h>

#ifndef NO_PIN
#define NO_PIN 255    ///< Pin number when pin is not defined
#endif

#define MAX_ISR 8      ///< Allow up to 8 encoders

/**
 * Core object for the MD_MotorEncoder class
 * This class is a simple abstraction for DC motor encoder feedback.
 */
class MD_MotorEncoder
{
public:
  //--------------------------------------------------------------
  /** \name Class constructor and destructor.
   * @{
   */
  /**
   * Class Constructor.
   *
   * Instantiate a new instance of the class.
   * 
   * The main function for the core object is to initialize the internal
   * shared variables and timers to default values.
   *
   * \param pinInt The interrupt pin for the encoder. This is digital pin that support interrupts.
   */
  MD_MotorEncoder(uint8_t pinInt);

  /**
   * Class Destructor.
   *
   * Release any allocated memory and clean up anything else.
   */
  ~MD_MotorEncoder(void);
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for core object control.
   * @{
   */
  /**
   * Initialize the object.
   *
   * Initialize the object data. This needs to be called during setup() to initialize new
   * data for the class that cannot be done during the object creation.
   *
   * \return false if the pin specified is not an interrupt pin.
   */
  bool begin(void);

  /**
   * Reset the encoder.
   *
   * This sets the encoder counter back to 0 and the the time marker to 'now'.
   */
  void reset(void);

  /**
   * Read encoder values.
   *
   * Returns the encoder accumulated count and the time period over which it was
   * accumulated, and resets the two registers.
   * 
   * \param interval variable for the time interval.
   * \param count    variable for the accumulator count.
   * \param bReset   if true (f=default) rset the counters, otherwise leave them as they are.
   */
  void read(uint32_t& interval, uint16_t& count, bool bReset = true);
  /** @} */

private:
  // Define the class variables
  uint8_t _pinInt;            ///< The interrupt pin in use
  uint32_t _timeLast;         ///< Keep track of when the counter was reset
  uint8_t _myISRId;           ///< This is my instance ISR Id for myInstance[x] and encoderISRx
  volatile uint16_t _counter; ///< Encoder interrupt counter

  static uint8_t _ISRAlloc;              ///< Keep track of which ISRs are used (global bit field)
  static MD_MotorEncoder* _myInstance[]; ///< callback instance for the ISR to reach handleISR()

  void handleISR(void);       ///< Instance ISR handler called from static ISR encoderISRx

  // declare all the [MAX_ISR] encoder ISRs
  static void encoderISR0(void);
  static void encoderISR1(void);
  static void encoderISR2(void);
  static void encoderISR3(void);
  static void encoderISR4(void);
  static void encoderISR5(void);
  static void encoderISR6(void);
  static void encoderISR7(void);
};
