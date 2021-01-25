#pragma once
/**
\mainpage Smart Car Robot Library

This library is for the implementation of a Smart Car Autonomous Robot.

See Also
- \subpage pageMotorController
- \subpage pageMotorEncoder
- \subpage pagePID
- \subpage pageRevisionHistory
- \subpage pageDonation
- \subpage pageCopyright

\page pageDonation Support the Library
If you like and use this library please consider making a small donation 
using [PayPal](https://paypal.me/MajicDesigns/4USD)

\page pageCopyright Copyright
Copyright (C) 2021 Marco Colli. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

\page pageRevisionHistory Revision History
Jan 2020 version 1.0.0
- Initial release
 */

#include <Arduino.h>
#include <SC_HardwareDefs.h>
#include <SC_DCMotor.h>
#include <SC_MotorEncoder.h>
#include <SC_PID.h>

 /**
 * \file
 * \brief Main header file and class definition for the MD_SmartCar library.
 */

#define SCDEBUG     1

#if SCDEBUG
#define PRINT(s,v)   do { Serial.print(F(s)); Serial.print(v); } while (false)
#define PRINTX(s,v)  do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false)
#define PRINTS(s)    do { Serial.print(F(s)); } while (false)
#else
#define PRINT(s,v)
#define PRINTX(s,v)
#define PRINTS(s)
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#endif

/**
 * Core object for the MD_SmartCar library
 */
class MD_SmartCar
{
public:
  //--------------------------------------------------------------
  /** \name Enumerated Types and Constants.
   * @{
   */
  /**
    * Maximum number of motors
    *
    * Define the maximum number of motors that this library can control
    */
    static const uint8_t MAX_MOTOR = 2;

   /** @} */

  //--------------------------------------------------------------
  /** \name Class constructor and destructor.
   * @{
   */
  /**
   * Class Constructor
   *
   * Instantiate a new instance of the class.
   * This variant is for motor controllers that have a PWM input for speed control.
   * 
   * The main function for the core object is to reset the internal
   * shared variables and timers to default values.
   * 
   * \param ml The object for controlling the left side motor.
   * \param el The object to use as the left side encoder input.
   * \param mr The object for controlling the right side motor.
   * \param er The object to use as the right side encoder input.
   */
  MD_SmartCar(SC_DCMotor *ml, SC_MotorEncoder *el, SC_DCMotor *mr, SC_MotorEncoder *er);

  /**
   * Class Destructor.
   *
   * Release any allocated memory and clean up anything else.
   */
  ~MD_SmartCar(void);
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for core object control.
   * @{
   */
  /**
   * Initialize the object.
   *
   * Initialize the object data. This needs to be called during setup() to reset new
   * items that cannot be done during object creation.
   * 
   * \return false if either encoder did not reset, true otherwise.
   */
  bool begin(void);

  /**
   * Run the Robot Management Services.
   *
   * This is called every iteration through loop() to run all the required
   * Smart Car Management functions.
   */
  void run(void);

 /**
  * Check if motors are running
  * 
  * Check if motors are commanded to run. This method is useful to check when
  * drive() or move() have completed their motions.
  *
  * \return true if any of the motors are not idle
  */ 
  bool isRunning(void);

  /**
   * Stop the smart car.
   *
   * This method will set the speed to 0 and disable all the motor functions to
   * bring the smart car to a complete stop.
   *
   * \sa setSpeed()
   */
  void stop(void);
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for free running the vehicle.
   * @{
   */
  /**
   * Drive the vehicle along specified path (degrees).
   *
   * Run the vehicle along a path with the specified velocity and angular orientation.
   * Moves the motors under PID control.
   *
   * The velocity is specified as a percentage of the maximum vehicle velocity [0..100].
   * Positive velocity move the vehicle forward, negative moves it in reverse.
   *
   * Angular velocity is specified in degrees per second [-180..180]. Positive angle
   * is clockwise rotation.
   *
   * \sa setVelocity(), setPIDTuning(), move()
   *
   * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
   * \param vAngularD the angular velocity in degrees per second [-180..180].
   */
  void drive(int8_t vLinear, int8_t vAngularD) { drive(vLinear, deg2rad(vAngularD)); }

  /**
   * Drive the vehicle along specified path (radians).
   *
   * Run the vehicle along a path with the specified velocity and angular orientation.
   * Moves the motors under PID control.
   *
   * The velocity is specified as a percentage of the maximum vehicle velocity [0..100].
   * Positive velocity move the vehicle forward, negative moves it in reverse.
   *
   * Angular velocity direction is specified in radians per second [-pi..pi]. Positive
   * angle is clockwise rotation.
   *
   * \sa setVelocity(), setPIDTuning(), move()
   *
   * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
   * \param vAngularR the angular velocity in radians per second [-pi..pi].
   */
  void drive(int8_t vLinear, float vAngularR);

  /**
   * Set the current master linear velocity.
   *
   * Linear velocity is expressed as a percentage of the maximum velocity [0..100].
   * 
   * When this method is called all the motors will be set to the appropriate velocity 
   * differentials, taking into account the currently set angular velocity.
   *
   * \sa getVelocity(), drive()
   * 
   * \param vel the new linear velocity setting.
   */
  void setVelocity(int8_t vel);

  /**
   * Get the current master linear velocity.
   *
   * Linear velocity is expressed as a percentage of the maximum velocity [0..100].
   * The Master velocity is used to regulate all the speed functions for the motors.
   *
   * \sa setVelocity()
   *
   * \return the current master speed setting.
   */
  inline int8_t getVelocity(void) { return(_vLinear); }

  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for precision movements of the vehicle.
   * @{
   */
   /**
   * Precisely move the vehicle (radians).
   *
   * Move controls the movement by counting the encoder pulses rather that PID,
   * which should make it more precise and controlled. This is useful for specific 
   * movements run at slow speed.
   * 
   * The call to move() specifies the precise motion of the motors, independently.
   * This method is design to allow close movements such as spin-in-place or 
   * other short precise motions.
   * 
   * The motion for each wheel is specified as speed as percentage of maximum speed 
   * [-100..100] and the total angle subtended by the turned by the wheel in radians.
   * Negative speed is a wheel rotation in reverse
   * 
   * \sa drive()
   * 
   * \param vL   left motor speed as a percentage of full speed [-100..100].
   * \param angL left wheel angle subtended by the motion.
   * \param vR   right motor speed as a percentage of full speed [-100..100].
   * \param angR right wheel angle subtended by the motion.
   */
  void move(int8_t vL, float angL, int8_t vR, float angR);

  /**
  * Precisely move the vehicle (degrees).
  *
  * Move controls the movement by counting the encoder pulses rather that PID,
  * which should make it more precise and controlled. This is useful for specific
  * movements run at slow speed.
  *
  * The call to move() specifies the precise motion of the motors, independently.
  * This method is design to allow close movements such as spin-in-place or
  * other short precise motions.
  *
  * The motion for each wheel is specified as speed as percentage of maximum speed
  * [-100..100] and the total angle subtended by the turned by the wheel in degrees.
  * Negative speed is a wheel rotation in reverse
  *
  * \sa drive()
  *
  * \param vL   left motor speed as a percentage of full speed [-100..100].
  * \param angL left wheel angle subtended by the motion.
  * \param vR   right motor speed as a percentage of full speed [-100..100].
  * \param angR right wheel angle subtended by the motion.
  */
  void move(int8_t vL, uint16_t angL, int8_t vR, uint16_t angR) { move(vL, deg2rad(angL), vR, deg2rad(angR)); }

  /** @} */
  //--------------------------------------------------------------
  /** \name Methods for EEPROM and Configuration Management.
   * @{
   */
  /**
   * Load settings from EEPROM.
   *
   * Load the config settings from EEPROM. These will have been saved 
   * to EEPROM by saveConfig(). If there is no currently saved config,
   * defaults are loaded.
   *
   * \sa saveConfig()
   */
  void loadConfig(void);

  /**
   * Save settings to EEPROM.
   *
   * Save the current settings to EEPROM. These will overwrite any previously 
   * saved settings.
   *
   * \sa loadConfig()
   */
  void saveConfig(void);

  /**
   * Set the creep speed.
   *
   * Set the creep speed units for the motors. These units will be passed
   * on directly to DCMotorControl and are meaningful to that class.
   *
   * \sa getCreepSP(), saveConfig()
   * 
   * \param units speed units to be used with the motor controller.
   * \return true if the value was set, false if it fails sanity checks.
   */
  bool setCreepSP(uint8_t units);

  /**
   * Get the creep speed.
   *
   * Get the creep speed units for the motors. These units will be passed
   * on directly to DCMotorControl and are meaningful to that class.
   *
   * \sa getCreepSP(), saveConfig()
   *
   * \return the previously configured speed.
   */
  uint8_t getCreepSP(void) { return(_config.creepPWM); }

  /**
   * Set the minimum motor setpoint.
   *
   * Set the minimum set point at which the motor will turn. This is used 
   * as a lower bound by the PID controller. The units are meaningful to the 
   * motor controller (eg, PWM setting). This is applicable to all motors.
   *
   * \sa setMaxMotorSP(), saveConfig()
   *
   * \param units speed units to be used with the motor controller.
   */
  void setMinMotorSP(uint8_t units) { if (units < _config.maxPWM) _config.minPWM = units; setPIDOutputLimits(); }

  /**
   * Set the maximum motor setpoint.
   *
   * Set the maximum set point for the motor will turn. This is used
   * as the upper bound by the PID controller. The units are meaningful 
   * to the motor controller (eg, PWM setting).  This is applicable to 
   * all motors.
   *
   * \sa setMinMotorSP(), saveConfig()
   * 
   * \param units speed units to be used with the motor controller.
   */
  void setMaxMotorSP(uint8_t units) { if (units > _config.minPWM) _config.maxPWM = units; setPIDOutputLimits(); }

  /**
   * Get the minimum motor setpoint.
   *
   * Get the currently configured minimum set point. The units are 
   * meaningful to the motor controller (eg, PWM setting).
   * 
   * \sa setMinMotorSP(), saveConfig()
   *
   * \return speed units currently configured.
   */
  uint8_t getMinMotorSP(void) { return(_config.minPWM); }

  /**
   * Get the maximum motor setpoint.
   *
   * Get the currently configured maximum set point. The units are
   * meaningful to the motor controller (eg, PWM setting).
   *
   * \sa setMaxMotorSP(), saveConfig()
   *
   * \return speed units currently configured.
   */
  uint8_t getMaxMotorSP(void) { return(_config.maxPWM); }

  /**
   * Change PID tuning parameters.
   *
   * Change the current set of PID tuning parameters.
   *
   * \sa saveConfig()
   *
   * \param Kp the proportional PID parameter.
   * \param Ki the integral PID parameter.
   * \param Kd the derivative PID parameter.
   */
  void setPIDTuning(float Kp, float Ki, float Kd);
  /** @} */

private:
  // Motor array indices
  const uint8_t MLEFT = 0;      ///< Array index for the Left motor
  const uint8_t MRIGHT = 1;     ///< Array index for the right motor

  enum runState_t { S_IDLE, S_DRIVE_INIT, S_DRIVE, S_MOVE_INIT, S_MOVE };

  float _vMaxLinear;      ///< Maximum linear speed in mm/s 
  int16_t _vLinear;       ///< Master velocity setting as percentage full scale (_vMaxLinear)
  float _vAngular;        ///< angular velocity in in radians per second

  // Define the control objects
  SC_DCMotor* _M[MAX_MOTOR];      ///< Motor controllers
  SC_MotorEncoder* _E[MAX_MOTOR]; ///< Motor encoders for feedback

  // Configuration data that is saved to EEPROM
  struct
  {
    uint8_t sig[2];   ///< config signature bytes
    uint8_t minPWM;   ///< the min PWM setting for DC motors
    uint8_t maxPWM;   ///< the max PWM setting for DC motors
    uint8_t creepPWM; ///< the creep PWM setting for DC motors
    float Kp;         ///< PID parameter
    float Ki;         ///< PID parameter
    float Kd;         ///< PID parameter
  } _config;

  // Motor state data used to manage each motor
  struct motorData_t
  {
    SC_DCMotor::runCmd_t direction;   ///< turning direction

    // PID variables
    int16_t sp;     ///< drive() PID set point value / move() PWM setting
    int16_t cv;     ///< drive() PID current value / move() target number of encoder pulses
    int16_t co;     ///< PID control output
    SC_PID* pid;    ///< PID object for control

    // Run state variables
    runState_t state;      ///< control state for this motor
    uint32_t   lastPIDRun; ///< time PID was last run (ms)
  };
  
  motorData_t _mData[MAX_MOTOR];  ///< keeping track of each motor's parameters

  // Methods
  void printConfig(void);             ///< debug only
  void setPIDOutputLimits(void);      ///< set the PID limits for all motors

  void calcVMax(void);         ///< calculate _vMaxLinear from the physical constants
  float deg2rad(int16_t deg);  ///< convert degrees to radians
};
