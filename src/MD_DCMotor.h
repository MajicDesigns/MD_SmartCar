#pragma once
/**
 * \file
 * \brief Header file for the MD_DCMotor class.
 */

#include <Arduino.h>

#ifndef NO_PIN
#define NO_PIN 255    ///< Pin number when pin is not defined
#endif

/**
 * Core object for the MD_DCMotor class
 * 
 * This class is a simple abstraction for PWM motor controllers that use
 * 2 digital pins for direction control and PWM for speed control. This includes
 * commonly available based on the L298 and L293 speed controllers.
 */
class MD_DCMotor
{
public:
  //--------------------------------------------------------------
  /** \name Enumerated values and Typedefs.
  * @{
  */
  /**
   * Define the different motor commands
   * 
   * These commands are used to direct the different modes of running the DC motors.
   * The commands encode the 2 bits required for pinInA (LSB) and pinInB. For a deive
   * that is different, just recode these commands and the software will adapt.
   */
  enum runCmd_t
  {
    DIR_REL = 0,    ///< Stop the motor. May coast to a stop.
    DIR_FWD = 1,    ///< Rotate in forward direction.
    DIR_REV = 2,    ///< Rotate in reverse direction (opposite of DIR_FWD).
    DIR_BRK = 3,    ///< Brake the motor. This should stop faster than DIR_REL.
  };
  /** @} */

  //--------------------------------------------------------------
  /** \name Class constructor and destructor.
   * @{
   */
  /**
   * Class Constructor (with PWM control).
   *
   * Instantiate a new instance of the class.
   * This variant is for motor controllers that have a PWM input for speed control.
   * 
   * The main function for the core object is to initialize the internal
   * shared variables and timers to default values.
   *
   * \param pinInA The InA pin number for command output to the controller. This is a simple digital pin.
   * \param pinInB The InB pin number for command output to the controller. This is a simple digital pin.
   * \param pinEn  The En pin number for PWM output to the controller. This is a PWM enabled pin.
   */
  MD_DCMotor(uint8_t pinInA, uint8_t pinInB, uint8_t pinEn);

  /**
   * Class Constructor (no PWM control).
   *
   * Instantiate a new instance of the class.
   * This variant is for motor controllers without PWM control (ie, just on/off).
   * 
   * The main function for the core object is to initialize the internal
   * shared variables and timers to default values.
   *
   * \param pinInA The InA pin number for command output to the controller. This is a simple digital pin.
   * \param pinInB The InB pin number for command output to the controller. This is a simple digital pin.
   */
  MD_DCMotor(uint8_t pinInA, uint8_t pinInB);

  /**
   * Class Destructor.
   *
   * Release any allocated memory and clean up anything else.
   */
  ~MD_DCMotor(void);
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
   */
  void begin(void);

  /**
   * Run/Stop the motor.
   *
   * This controls how the motor runs/stops. The command specified is one of the 
   * runCMD_t values. 
   *
   * \param cmd the run/stop mode.
   */
  inline void run(runCmd_t cmd) { setMode(cmd); }

  /**
   * Run/Stop the motor with speed.
   *
   * This controls how the motor runs/stops. This is a convenience wrapper for 
   * the run() and setSpeed() methods. The command specified is one of the
   * runCmd_t values and the speed must be suitable for the setSpeed() method.
   * 
   * \sa setSpeed(), run()
   *
   * \param cmd   the run/stop mode.
   * \param speed the speed to run at.
   */
  inline void run(runCmd_t cmd, uint8_t speed) { setSpeed(speed); setMode(cmd); }

  /**
   * Set the speed for the motor
   *
   * This controls the PWM setting for the motor. Note that the actual speed is dependent on 
   * the motor and it is unlikely that it will be a linear response across the range of 
   * valid values.
   *
   * \param s the speed setting [0..255].
   */
  inline void setSpeed(uint8_t s) { if (_pinEn != NO_PIN) analogWrite(_pinEn, s); }
  /** @} */

private:
  // Define the hardware interface pins
  uint8_t _pinInA;  ///< One of the mode control control pins
  uint8_t _pinInB;  ///< The other mode control control pins
  uint8_t _pinEn;   ///< Must be a PWM enabled pin for speed control

  inline void setMode(runCmd_t cmd)
  // Set the mode bits for the controller.
  // The enumerated values are coded as the bit pattern for each command.
  {
    digitalWrite(_pinInA, (cmd & 1) ? HIGH : LOW);
    digitalWrite(_pinInB, (cmd & 2) ? HIGH : LOW);
  }
};
