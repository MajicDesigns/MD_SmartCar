#pragma once
/**
\mainpage Smart Car Robot Library

This library is for the implementation of a Smart Car Autonomous Robot.

See Also
- \subpage pageRevisionHistory
- \subpage pageDonation
- \subpage pageCopyright

\page pageDonation Support the Library
If you like and use this library please consider making a small donation using [PayPal](https://paypal.me/MajicDesigns/4USD)

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
#include <EEPROM.h>
#include "MD_SmartCarHW.h"
#include "MD_DCMotor.h"
#include "MD_MotorEncoder.h"
#include <PID_v1.h>

 /**
 * \file
 * \brief Main header file and class definition for the MD_SmartCar library.
 */

#define SCDEBUG     1

#if SCDEBUG
#define PRINT(s,v)   do { Serial.print(F(s)); Serial.print(v); } while (false);
#define PRINTX(s,v)  do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false);
#define PRINTS(s)    do { Serial.print(F(s)); } while (false);
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

    /**
     * Movement path type
     *
     * Define possible 'canned' paths for the smart car.
     */
    enum pathType_t
    {
      LINEAR,  ///< Straight path
      VEER,    ///< Veer left 90 degrees (arc)
      TURN,    ///< On-the-spot turn (90 degrees)
      ATURN,   ///< On-the-spot 'about' turn (180 degrees)
    };

  /**
   * Movement direction type
   * 
   * Define the possible directions SmartCar can travel
   */
    enum dirType_t
    {
      STOP,  ///< No motion
      FWD,   ///< Forward direction
      REV,   ///< Reverse direction
    };

  /**
   * Movement side direction type
   *
   * Define the possible sides SmartCar can travel
   */
    enum sideType_t
    {
      CENTER, ///< Not applicable
      LEFT,   ///< Left direction
      RIGHT,  ///< Right direction
    };

  /** 
   * Running mode type
   *
   * Define how the motors will be controlled.
   * All motors will be controlled in the same during a specific path.
   */
  enum runType_t 
  { 
    CREEP,    ///< Creep - use the encoder to count pulses for motion. Should be more precise.
    RUN,      ///< Run - run motor with PID to control speed.
  };

  /**
   * Path specifcation record
   *
   * Define the possible directions SmartCar can travel
   */
  struct pathDef_t
  {
    pathType_t path;    ///< the path to follow
    sideType_t side;    ///< the side to move
    dirType_t dir;      ///< the direction of travel
    runType_t run;      ///< how the motors are controlled for this path
    struct 
    {
      MD_DCMotor::runCmd_t dir;   ///< the direction to drive this motor
      uint16_t count;             ///< count of pulses for CREEP
    } mspec[MAX_MOTOR];           ///< Motor settings specification  
  };

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
   * The main function for the core object is to initialize the internal
   * shared variables and timers to default values.
   * 
   * \param ml The object for controlling the left side motor.
   * \param el The object to use as the left side encoder input.
   * \param mr The object for controlling the right side motor.
   * \param er The object to use as the right side encoder input.
   */
  MD_SmartCar(MD_DCMotor *ml, MD_MotorEncoder *el, MD_DCMotor *mr, MD_MotorEncoder *er);

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
   * Initialize the object data. This needs to be called during setup() to initialize new
   * items that cannot be done during object creation.
   * 
   * \return false if either encoder did not initialize, true otherwise.
   */
  bool begin(void);

  /**
   * Run the Robot Management Services.
   *
   * This is called every iteration through loop() to run all the required
   * Smart Car Management functions.
   */
  void move(void);

  /**
   * Set the current master speed setting.
   *
   * Speed is expressed as frequency of the encoder input (ie, clicks per second).
   * The Master speed is used to regulate all the speed functions for the motors.
   * When this method is called all the motors will be set to this speed as their
   * set point, overriding any path defined speed differentials.
   *
   * \sa getSpeed()
   * 
   * \param speed the new speed setting.
   */
  void setSpeed(uint16_t speed);

  /**
   * Get the current master speed setting.
   *
   * Speed is expressed as frequency of the encoder input (ie, clicks per second).
   * The Master speed is used to regulate all the speed functions for the motors.
   *
   * \sa setSpeed()
   *
   * \return the current master speed setting.
   */
  inline uint16_t getSpeed(void) { return(_speed); }

  /**
   * Free run the smart car at current speed.
   *
   * Free running moves the motors under PID control. 
   * 
   * Free running moves are defined in a path_t type definition stored in the 
   * library, keyed by the tuple made up of the (path, side, dir) parameters passed 
   * to this method (a unique combination). This form of the method maintains the 
   * current speed setting.
   *
   * \sa setSpeed(), setPIDTuning(), 
   *
   * \param path the pathType_t path name for this movement.
   * \param side the sideType_t definition for the side the move will be executed.
   * \param dir  the dirType_t direction of to travel when executing this move.
   */
  void run(pathType_t path, sideType_t side, dirType_t dir) { run(path, side, dir, getSpeed()); }

  /**
   * Free run the smart car at specified speed.
   *
   * Free running moves the motors under PID control.
   *
   * Free running moves are defined in a path_t type definition stored in the
   * library, keyed by the tuple made up of the (path, side, dir) parameters passed
   * to this method (a unique combination). The speed specified is passed to setSpeed()
   * and must be suitable for that method.
   *
   * \sa setSpeed(), setPIDTuning(),
   *
   * \param path  the pathType_t path name for this movement.
   * \param side  the sideType_t definition for the side the move will be executed.
   * \param dir   the dirType_t direction of to travel when executing this move.
   * \param speed the new speed setting for this movement.
   */
  void run(pathType_t path, sideType_t side, dirType_t dir, uint16_t speed);
  
  /**
   * Creep move the smart car.
   *
   * Creeping controls the movement by counting the encoder pulses.
   *
   * Creeping moves should be more precise and controlled, useful for specific 
   * movements run at slower speed.
   * 
   * All creeping moves are defined in a path_t type definition stored in the 
   * library, keyed by the tuple made up of the (path, side, dir) parameters passed 
   * to this method (a unique combination).
   * 
   * \sa run()
   * 
   * \param path the pathType_t path name for this movement.
   * \param side the sideType_t definition for the side the move will be executed.
   * \param dir  the dirType_t direction of to travel when executing this move.
   */
  void creep(pathType_t path, sideType_t side, dirType_t dir);

  /**
   * Stop the smart car.
   *
   * This method will set the speed to 0 and disable all the morot functions to 
   * bring the smart car to a complete stop.
   *
   * \sa setSpeed()
   */
  void stop(void);
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
   * Save the current settings to EEPROM. These will overwrite any previouly 
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
   * saved settings.
   *
   * \sa getCreepSP(), saveConfig()
   * 
   * \param units speed units to be used with the motor controller.
   * \return true if the value was set, false if it fails sanity checks.
   */
  bool setCreepSP(uint16_t units);

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
  void setMinMotorSP(uint16_t units) { _config.minPWM = units; setPIDOutputLimits(); }

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
  void setMaxMotorSP(uint16_t units) { _config.maxPWM = units; setPIDOutputLimits(); }

  /**
   * Get the creep speed.
   *
   * Get the currently configured creep speed. The units are
   * meaningful to the motor controller (eg, PWM setting).
   *
   * \sa setCreepSP(), saveConfig()
   *
   * \return speed units currently configured.
   */
  uint16_t getCreepSP(void) { return(_config.creepSP); }

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
  uint16_t getMinMotorSP(void) { return(_config.minPWM); }

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
  uint16_t getMaxMotorSP(void) { return(_config.maxPWM); }

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
  void setPIDTuning(double Kp, double Ki, double Kd);
  /** @} */

private:
  // Motor array indices
  const uint8_t MLEFT = 0;      ///< Array index for the Left motor
  const uint8_t MRIGHT = 1;     ///< Array index for the right motor

  enum runState_t { S_IDLE, S_RUN_INIT, S_RUN, S_CREEP_INIT, S_CREEP, S_CREEP_END };

  uint16_t _speed;         ///< master speed setting

  // Define the control objects
  MD_DCMotor* _M[MAX_MOTOR];      ///< Motor controllers
  MD_MotorEncoder* _E[MAX_MOTOR]; ///< Motor encoders for feedback

  // Configuration data that is saved to EEPROM
  struct
  {
    uint8_t sig[2];       ///< config signature bytes
    uint16_t creepSP;     ///< the creep set point
    uint16_t minPWM;      ///< the min PWM setting for DC motors
    uint16_t maxPWM;      ///< the max PWM setting for DC motors
    double Kp;            ///< PID parameter
    double Ki;            ///< PID parameter
    double Kd;            ///< PID parameter
  } _config;

  // Motor state data used to manage each motor
  struct motorData_t
  {
    // PID variables
    double sp;          ///< PID required outcome (set point)
    double cv;          ///< PID calcs input (current value)
    double co;          ///< PID calcs output (control output)
    PID* pid;           ///< PID object

    // Run state variables
    runState_t state;   ///< control state for this motor
  };
  
  motorData_t _mData[MAX_MOTOR];  ///< keeping track of each motor's parameters

  pathDef_t _path;                      ///< the current path being followed
  static const pathDef_t PROGMEM pathTable[7];  ///< table of built-in path definitions

  // methods
  void setPIDOutputLimits(void);    ///< set the PID limits for all motors
  void updateSetpoint(void);        ///< update the current set point, propagating as necessary
  bool findPath(pathDef_t *rpath, pathType_t fpath, sideType_t fside, dirType_t fdir, runType_t frun, const pathDef_t * table, uint16_t count);
};
