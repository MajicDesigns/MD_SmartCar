#pragma once
/**
 * \file
 * \brief Header file for the SC_DCMotor class.
 */

/**
\page pageMotorController Motor Controllers
The library supports a number of inexpensive DC motor controllers. All use PWM
as the mechanism for speed control.

Common to all these motor controllers is that there is no 'set' way to wire
them to the motors. The direction of motor spin relative to the body of the 
vehicle it is attached to depends on the orientation of the motor and how the 
gearbox works as well as the way the motor is wired.

The best way to work out the wiring is to temporarily make the connections between 
motor and controller and use the test software and drive the motor 'forward'. If 
it spins in the wrong direction, reverse the wiring at the motor.

## The L298N Motor Controller

The L298N Motor Driver Module (see below), commonly available from low-cost online 
stores, is a high power motor driver module for driving DC and Stepper Motors. 
This module consists of an L298 motor driver IC and a 78M05 5V regulator which can
be used to power the Arduino Board. L298N Module can control up to 4 DC motors, 
or 2 stepper motors, with directional and speed control.

![L298N Controller] (SmartCar_Controller_L298N.jpg "L298N Controller Module")

ENA/ENB pins are PWM speed control pins for Motor A and Motor B while IN1/IN2 and 
IN3/IN4 are direction control pins for Motor A and Motor B. Motors are connected to 
OUT1/OUT2 and OUT3/OUT4.

An external power supply up to 12V can be connected to the screw connectors to supply 
power for the motors. This voltage can also feed the 5V regulator to power the Arduino 
processor board. If the processor has its own 5V supply, then the '5V  Enable' jumper 
should be removed to disable this function.

## The L293D Motor Controller

The L293D is another commonly available 16-Pin Motor Driver Module to drive small 
motors. A single L293D IC is capable of running two DC motors at the same time with 
independent direction and speed control.

![L293 Controller] (SmartCar_Controller_L293.jpg "L293 Controller Module")

This controller is physically smaller than the L298N and can control less motor current, 
but it has essentially the same control pins, output terminals and control mechanism as 
the L298N, so can be considered a drop in replacement.

## The MX1508 Motor Controller

The MX1508 Motor Driver Module is an highly inexpensive driver module available from 
low-cost online suppliers. It has specifications similar to the L293D and looks like it 
is a low cost Chinese version of the DRV8833 Controller IC, with a few less functions. 
The MX1805 module can control up to 2 DC motors, with directional and speed control.

![MX1508 Controller] (SmartCar_Controller_MX1508.jpg "MX1508 Controller Module")

IN1/IN2 and IN3/IN4 are direction control pins for Motor A and Motor B. These double as 
speed control pins using PWM. Motors are connected to OUT1/OUT2 and OUT3/OUT4 (MOTOR-A 
and MOTOR-B solder pads).

An external power supply up to 10V can be connected to the screw connectors to supply 
power for the motors. This module does not have a 5V regulator so the Arduino processor 
must be powered separately.

## The DRV8833 Motor Controller

The DRV8833 is a low voltage, 2A stepper or single/dual brushed DC motor driver. 

![DRV8833 Controller] (SmartCar_Controller_DRV8833.jpg "DRV8833 Controller Module")

Similar to the M1805, IN1/IN2 and IN3/IN4 are direction control pins for Motor A and 
Motor B. These double as speed PWM control pins. Motors are connected to OUT1/OUT2 
and OUT3/OUT4. Additionally the DRV8833 has an output status pin to signal fault mode 
and an input to put the device into 'sleep' mode for very low power consumption.
 */
#include <Arduino.h>

/**
 * Core object for the SC_DCMotor class
 * 
 * This class is a simple abstraction for PWM motor controllers that use
 * 2 digital pins for direction control and PWM for speed control.
 */
class SC_DCMotor
{
public:
  //--------------------------------------------------------------
  /** \name Enumerated values and Typedefs.
  * @{
  */
  /**
   * Define the different motor commands/states
   * 
   * These commands are used to direct the different modes of 
   * running the DC motors allowed by the controller.
   */
  enum runCmd_t
  {
    DIR_FWD,   ///< Rotate in forward direction.
    DIR_REV    ///< Rotate in reverse direction (opposite of DIR_FWD).
  };
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for core object control.
   * @{
   */
  /**
   * Initialize the object.
   *
   * Initialize the object data. This needs to be called during setup() to reset new
   * data for the class that cannot be done during the object creation.
   */
  virtual void begin(void) = 0;

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
  virtual void run(runCmd_t cmd, uint8_t speed) = 0;

  /**
   * Set the speed for the motor.
   *
   * This controls the speed setting for the motor. This is replaced with a method to
   * implement the specifics for the derived class.
   *
   * \param s the speed setting appropriate for the controller type.
   */
  virtual void setSpeed(uint16_t s) = 0;


  /**
   * Get the currently set speed for the motor.
   *
   * Returns the current speed setting
   *
   * \return the speed setting.
   */
  inline uint16_t getSpeed() { return(_speed); }
  /** @} */

protected:
  runCmd_t _mode;     ///< The current mode for the motor
  uint16_t _speed;    ///< The current speed setting for the motor
};


/**
 * Core object for the derived SC_DCMotor_L298 class
 * 
 * This motor controller uses 2 standard digital pins for direction
 * control and one PWM capable pin for speed control.
 */
class SC_DCMotor_L29x : public SC_DCMotor
{
public:
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
    * The main function for the core object is to reset the internal
    * shared variables and timers to default values.
    *
    * \param pinIn1 The In1 pin number for command output to the controller. This is a simple digital pin.
    * \param pinIn2 The In2 pin number for command output to the controller. This is a simple digital pin.
    * \param pinEn  The En pin number for PWM output to the controller. This is a PWM enabled pin.
    */
  SC_DCMotor_L29x(uint8_t pinIn1, uint8_t pinIn2, uint8_t pinEn) :
    _pinIn1(pinIn1), _pinIn2(pinIn2), _pinEn(pinEn)
  {
    _mode = DIR_FWD;
    _speed = 0;
  }

  /**
   * Class Destructor.
   *
   * Release any allocated memory and clean up anything else.
   */
  ~SC_DCMotor_L29x(void) { setSpeed(0); }
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for core object control.
   * @{
   */
   /**
   * Initialize the object.
   *
   * Initialize the object data. Sets up the pins for output and puts the 
   * controlled in a stopped position.
   */
  void begin(void);

  /**
   * Run/Stop the motor with speed.
   *
   * This controls how the motor runs/stops. This sets the motor controller
   * into the correct operating mode and then sets the speed to what is specified.
   * The command specified is one of the runCmd_t values and the speed must be 
   * suitable for the setSpeed() method.
   *
   * \sa setSpeed(), run()
   *
   * \param cmd   the run/stop mode.
   * \param speed the speed to run at.
   */
  void run(runCmd_t cmd, uint8_t speed) { setSpeed(speed); setMode(cmd); }

  /**
   * Set the speed for the motor.
   *
   * This controls the PWM setting for the motor. Note that the actual speed is dependent on
   * the motor and it is unlikely that it will be a linear response across the range of
   * valid values.
   *
   * \param s the speed setting [0..255].
   */
  void setSpeed(uint16_t s);
  /** @} */

private:
  // Define the hardware interface pins
  uint8_t _pinIn1;  ///< One of the mode control control pins
  uint8_t _pinIn2;  ///< The other mode control control pins
  uint8_t _pinEn;   ///< Must be a PWM enabled pin for speed control

  void setMode(runCmd_t cmd);
};

/**
 * Core object for the derived SC_DCMotor_M1508 class
 * 
 * This motor controller uses 2 PWM capable pins for direction and 
 * PWM speed control.
 */
class SC_DCMotor_MX1508 : public SC_DCMotor
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
    * This variant is for motor controllers that have a PWM input for speed control.
    *
    * The main function for the core object is to reset the internal
    * shared variables and timers to default values.
    *
    * \param pinIn1 The In1 pin number for command output to the controller. This is a PWM enabled pin.
    * \param pinIn2 The In2 pin number for command output to the controller. This is a PWM enabled pin.
    */
  SC_DCMotor_MX1508(uint8_t pinIn1, uint8_t pinIn2)
  { 
    _pinIn[0] = pinIn1;
    _pinIn[1] = pinIn2;
    _mode = DIR_FWD; 
    _speed = 0;
  }

  /**
   * Class Destructor.
   *
   * Release any allocated memory and clean up anything else.
   */
  ~SC_DCMotor_MX1508(void) { setSpeed(0); }
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for core object control.
   * @{
   */
   /**
   * Initialize the object.
   *
   * Initialize the object data. Sets up the pins for output and puts the
   * controlled in a stopped position.
   */
  void begin(void);

  /**
   * Run/Stop the motor with speed.
   *
   * This controls how the motor runs/stops. This sets the motor controller
   * into the correct operating mode and then sets the speed to what is specified.
   * The command specified is one of the runCmd_t values and the speed must be
   * suitable for the setSpeed() method.
   *
   * \sa setSpeed(), run()
   *
   * \param cmd   the run/stop mode.
   * \param speed the speed to run at.
   */
  void run(runCmd_t cmd, uint8_t speed) { setSpeed(0);  setMode(cmd); setSpeed(speed); }

  /**
   * Set the speed for the motor
   *
   * This controls the PWM setting for the motor. Note that the actual speed is dependent on
   * the motor and it is unlikely that it will be a linear response across the range of
   * valid values.
   *
   * \param s the speed setting [0..255].
   */
  void setSpeed(uint16_t s);
  /** @} */

private:
  // Define the hardware interface pins
  uint8_t _pinIn[2];  ///< The mode control control pins
  uint8_t _pinPWM;    ///< the pin to use for PWM control

  void setMode(runCmd_t cmd);
};