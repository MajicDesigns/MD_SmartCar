#pragma once
/**
\mainpage Smart Car Robot Library

A enabler for an automonously roving robot is infrastructure that allows 
it to travel about in a controlled manner. This is carried out under the 
control of an application that responds to real-world inputs and adapts the
movement of the vehicle accordingly.

This library is designed to include core mobility functions for
an autonomous Smart Car Robot.

Whilst library is designed around the limited but highly affordable 2 wheel 
drive (plus idler castor wheel) vehicles platforms found on online marketplaces,
it is also suitable for more capable similar platforms with little or no 
modification.

![SmartCar Platform] ( SmartCar_Platform.jpg "SmartCar Platform")

The vehicle is made up of a number of subcomponents that come together to
allow the software library to function:
- Robot vehicle chassis
- \subpage pageMotorController
- \subpage pageMotorEncoder

There are broadly two types of autonomous movements:
- _Precisely controlled movements_ (eg, spin in place), where the ability to
  maneuver the orientation of the vehicle at low speed is important. 
  Independent control of motor directions and how far it spins are used as 
  control parameters for this mode type of movement.
- _General movements_ (eg, traveling at a set speed in a set direction),
  where the ability to move quickly in an accurate path is important. This type
  of movement is managed using the \ref pageControlModel "unicycle model" for
  control coupled to \ref pagePID "PID control" of the DC motors. 

See Also
- \subpage pageHardwareMap
- \subpage pageControlModel
- \subpage pagePID
- \subpage pageMotorController
- \subpage pageMotorEncoder
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

\page pageControlModel Unicycle Control Model
Two wheeled robots are very maneuverable, but steering them requires 
each of the independent wheels to rotated at different speeds (call 
them vL and vR for the left and right side). Managing vL and vR
independently to achieve a specific path is cumbersome and complex.

The _unicycle model_ of an autonomous robot is a kinematic model that 
allows us to model the movement of the vehicle using a linear velocity
vector (v) and a rotational speed (w) about a point within the vehicle.
Specifying a movement path in this model becomes much easier ("How fast 
do we want to move forward and how fast do we want to turn").

![SmartCar Movement Transform] ( SmartCar_Transform.png "SmartCar Movement Transform")

v and w can be transformed into independent motor speeds (vL and vR)
using the following formulas:
- vL = (2v+wL)/2r
- vR = (2v-wL)/2r

____
References


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

#define PID_TUNE 0    ///< set to 1 for specific PID tuning output
#define SCDEBUG 0     ///< set to 1 for general debug output

#if SCDEBUG
#define SCPRINT(s,v)   do { Serial.print(F(s)); Serial.print(v); } while (false)
#define SCPRINTX(s,v)  do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false)
#define SCPRINTS(s)    do { Serial.print(F(s)); } while (false)
#else
#define SCPRINT(s,v)
#define SCPRINTX(s,v)
#define SCPRINTS(s)
#endif

#if PID_TUNE
#define P_PID_HDR   do { Serial.print(F("/*")); } while (false)
#define P_PID_BODY(SP, CV, CO, last) do \
{ \
  Serial.print(SP); Serial.print(F(",")); \
  Serial.print(CV); Serial.print(F(",")); \
  Serial.print(CO); if (!last) Serial.print(F(",")); \
} while (false)
#define P_PID_TAIL do { Serial.print(F(",")); Serial.print(millis()); Serial.print(F("*/\n")); } while (false)
#else
#define P_PID_HDR
#define P_PID_BODY(SP, CV, CO, last)
#define P_PID_TAIL
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
   * Angular velocity is specified in degrees per second [-90..90]. Positive angle
   * is clockwise rotation.
   *
   * \sa setVelocity(), setPIDTuning(), move()
   *
   * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
   * \param vAngularD the angular velocity in degrees per second [-90..90].
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
   * Angular velocity direction is specified in radians per second [-pi/2..pi/2]. Positive
   * angle is clockwise rotation.
   *
   * \sa setVelocity(), setPIDTuning(), move()
   *
   * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
   * \param vAngularR the angular velocity in radians per second [-pi/2..pi/2].
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
   * The call to move() specifies the angle each wheels will turn, independently.
   * This method is design to allow close movements such as spin-in-place or 
   * other short precise motions.
   * 
   * The motion for each wheel is specified as speed as the total angle subtended 
   * by the turned by the wheel in radians. Negative angle is a reverse wheel 
   * rotation.
   * 
   * \sa drive(), setCreepSP()
   * 
   * \param angL left wheel angle subtended by the motion in radians.
   * \param angR right wheel angle subtended by the motion in radians.
   */
  void move(float angL, float angR);

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
  * The motion for each wheel is specified as speed as the total angle subtended 
  * by the turned by the wheel in degrees. Negative angle is a reverse wheel rotation.
  *
  * \sa drive(), setCreepSP()
  *
  * \param angL left wheel angle subtended by the motion in degrees.
  * \param angR right wheel angle subtended by the motion in degrees.
  */
  void move(int16_t angL, int16_t angR) { move(deg2rad(angL), deg2rad(angR)); }

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
   * Set the move speed.
   *
   * Set the move() speed units for the motors. These units will be passed
   * on directly to DCMotorControl and are meaningful to that class.
   *
   * \sa move(), getMoveSP(), saveConfig()
   * 
   * \param units speed units to be used with the motor controller.
   * \return true if the value was set, false if it fails sanity checks.
   */
  bool setMoveSP(uint8_t units);

  /**
   * Get the move speed.
   *
   * Get the move() speed units for the motors. These units will be passed
   * on directly to DCMotorControl and are meaningful to that class.
   *
   * \sa move(), setMoveSP(), saveConfig()
   *
   * \return the previously configured speed.
   */
  uint8_t getMoveSP(void) { return(_config.movePWM); }

  /**
   * Set the drive kicker speed.
   *
   * Set the drive() kicker speed for the motors. These units will be passed
   * on directly to DCMotorControl and are meaningful to that class.
   * 
   * The Kicker is needed to overcome the static friction when the motor is 
   * stopped and shuold be determined as the minimum that reliably gets a 
   * motor rotation started.
   *
   * \sa drive(), getKickerSP(), saveConfig()
   *
   * \param units speed units to be used with the motor controller.
   * \return true if the value was set, false if it fails sanity checks.
   */
  bool setKickerSP(uint8_t units);

  /**
   * Get the drive kicker speed.
   *
   * Get the drive() kicker speed units for the motors. These units will be passed
   * on directly to DCMotorControl and are meaningful to that class.
   *
   * The Kicker is needed to overcome the static friction when the motor is
   * stopped and shuold be determined as the minimum that reliably gets a
   * motor rotation started.
   *
   * \sa drive(), setKickerSP(), saveConfig()
   *
   * \return the previously configured speed.
   */
  uint8_t getKickerSP(void) { return(_config.kickerPWM); }

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
  void setMinMotorSP(uint8_t units);

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
  void setMaxMotorSP(uint8_t units);

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
   * Set PID tuning parameters.
   *
   * Change the current set of PID tuning parameters for the specified motor.
   *
   * \sa saveConfig()
   *
   * \param mtr the motor number [0..MAX_MOTOR].
   * \param Kp the proportional PID parameter.
   * \param Ki the integral PID parameter.
   * \param Kd the derivative PID parameter.
   */
  void setPIDTuning(uint8_t mtr, float Kp, float Ki, float Kd);

  /**
   * Get PID tuning parameters.
   *
   * Return the current set of PID tuning parameters for the specified motor.
   *
   * \sa saveConfig()
   *
   * \param mtr the motor number [0..MAX_MOTOR].
   * \param Kp the proportional PID parameter.
   * \param Ki the integral PID parameter.
   * \param Kd the derivative PID parameter.
   */
  void getPIDTuning(uint8_t mtr, float& Kp, float& Ki, float& Kd);
  /** @} */

private:
  // Motor array indices
  const uint8_t MLEFT = 0;      ///< Array index for the Left motor
  const uint8_t MRIGHT = 1;     ///< Array index for the right motor

  enum runState_t { S_IDLE, S_DRIVE_INIT, S_DRIVE_KICKER, S_DRIVE_PIDRST, S_DRIVE_RUN, S_MOVE_INIT, S_MOVE_RUN };

  float _vMaxLinear;      ///< Maximum linear speed in mm/s 
  int16_t _vLinear;       ///< Master velocity setting as percentage [0..100] = [0.._vMaxLinear]
  float _vAngular;        ///< angular velocity in in radians per second [-PI..PI]

  // Define the control objects
  SC_DCMotor* _M[MAX_MOTOR];      ///< Motor controllers
  SC_MotorEncoder* _E[MAX_MOTOR]; ///< Motor encoders for feedback

  // Configuration data that is saved to EEPROM
  struct
  {
    uint8_t sig[2];       ///< config signature bytes
    uint8_t minPWM;       ///< the min PWM setting for DC motors
    uint8_t maxPWM;       ///< the max PWM setting for DC motors
    uint8_t movePWM;      ///< the creep PWM setting for DC motors
    uint8_t kickerPWM;    ///< kicker to overcome static froction from stop position
    float Kp[MAX_MOTOR];  ///< PID parameter per motor
    float Ki[MAX_MOTOR];  ///< PID parameter per motor
    float Kd[MAX_MOTOR];  ///< PID parameter per motor
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
    uint32_t   timeLast;  ///< time last event (eg, PID) was last run (ms)
  };
  
  motorData_t _mData[MAX_MOTOR];  ///< keeping track of each motor's parameters

  // Methods
  void printConfig(void);             ///< debug only
  void setPIDOutputLimits(void);      ///< set the PID limits for all motors

  // Calculate max linear velocity (mm/s) given physical constraints.
  // Use the left motor in calc assuming that the encoders will have identical characteristics!
  inline float calcVMax(void) { return(DIST_PER_REV * (MAX_PULSE_PER_SEC / _E[MLEFT]->getPulsePerRev())); }

  inline float deg2rad(int16_t deg) { return((PI * (float)deg) / 180.0); };   // convert degress to radians
};
