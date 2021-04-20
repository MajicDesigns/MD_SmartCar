#pragma once
/**
\mainpage Smart Car Robot Library

This library is designed to provide core mobility functions for
an autonomous two wheeled Smart Car Robot. The library provides
the code infrastructure that allows the car to travel in a 
controlled manner, on top of which specific applications can 
confidently be built.

This library is designed around a commonly obtainable two wheel drive (+ 
idler castor wheel) vehicle chassis found on online marketplaces that 
look something like the one below. It is also suitable, with little or no
modifications, for more capable platforms with similar mechanisms. 

![SmartCar Platform] (SmartCar_Platform.jpg "SmartCar Platform")

The vehicle hardware and control system are made up of a number of
subcomponents that are functionally brought together by the software
library to function:
- Robot vehicle chassis (as shown above)
- \subpage pageMotorController
- \subpage pageMotorEncoder

The control hierarchy implemented in the library is shown in the figure
below. The library implements the control elements from "Motion Control" 
to the right of the figure. The components to the left of 'Motion Control' 
are defined into the application that defines the vehicle's behavior.

![Control Hierarchy] (SmartCar_Control_Hierarchy.png "Control Hierarchy")

The library is designed to control 2 types of autonomous movements:
- _Precisely controlled movements_ (eg, spin in place), where the ability to
  manoeuvre the orientation of the vehicle at low speed is important. 
  Independent control of motor directions and how far it spins are used as 
  control parameters for this mode type of movement.
- _General movements_ (eg, traveling at a set speed in a set direction),
  where the ability to move more quickly in an specifed path is important. 
  This type of movement is managed using the \ref pageControlModel "unicycle 
  model" for control coupled to \ref pagePID "PID control" of the DC motors. 

### Library Topics
- \subpage pageUsingLibrary
- \subpage pageHardwareMap
- \subpage pageControlModel
- \subpage pageActionSequence
- \subpage pagePID
- \subpage pageMotorController
- \subpage pageMotorEncoder

### Additional Topics
- \subpage pageRevisionHistory
- \subpage pageDonation
- \subpage pageCopyright

### Library dependencies
- MD_PWM library located at https://github.com/MajicDesigns/MD_PWM or the Arduino library manager.
- MD_cmdProcessor library located at https://github.com/MajicDesigns/MD_cmdProcessor or the Arduino library manager

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
Apr 2021 version 1.0.0
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

#ifndef PID_TUNE
#define PID_TUNE 1    ///< set to 1 for specific PID tuning output
#endif
#ifndef SCDEBUG
#define SCDEBUG  0    ///< set to 1 for general debug output
#endif

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
#define P_PID_HDR   do { Serial.print(F("{")); } while (false)
#define P_PID_BODY(SP, CV, CO, last) do \
{ \
  Serial.print(SP); Serial.print(F(",")); \
  Serial.print(CV); Serial.print(F(",")); \
  Serial.print(CO); if (!last) Serial.print(F(",")); \
} while (false)
#define P_PID_TAIL do { Serial.print(F(",")); Serial.print(millis()); Serial.print(F("}\n")); } while (false)
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
  /** \name Structures, Enumerated Types and Constants.
   * @{
   */
  /**
    * Maximum number of motors
    *
    * Define the maximum number of motors that this library can control
    */
  static const uint8_t MAX_MOTOR = 2;

  /**
   * Enumerated type for Action Items operation
   * 
   * Specifies which operation is being defined in the actionItem_t
   */
  enum actionId_t
  {
    DRIVE,    ///< executes drive(); param 0 lin vel, param 1 angular velocity
    MOVE,     ///< executes a move(); param 0 left rotate, param 1 right rotate
    SPIN,     ///< executes a spin(); param 0 spin percentage
    PAUSE,    ///< executes a pause; param 0 milliseconds pause
    STOP,     ///< executes a stop()
    END       ///< marks the end of the action list; should always be last item.
  };
  
  /**
    * Move sequence item definition
    * 
    * Define one of the action elements for a move() sequence.
    */
  typedef struct 
  {
    actionId_t opId;          ///< id for the action specified by this item
    float parm[MAX_MOTOR];    ///< function paremeter
  } actionItem_t;

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
  MD_SmartCar(SC_DCMotor* ml, SC_MotorEncoder* el, SC_DCMotor* mr, SC_MotorEncoder* er);

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
    * Vehicle constants are passed through to the setVehicleParameters() method. See
    * comments for that method for more details.
    *
    * \sa setVehicleParameters();
    *
    * \param ppr    Number of encoder pulses per wheel revolution.
    * \param ppsMax Maximum number of encoder pulses per second at top speed (100% velocity).
    * \param dWheel Wheel diameter in mm.
    * \param lBase  Base length (distance between wheel centers) in mm
    * \return false if either encoder did not reset, true otherwise.
    */
  bool begin(uint16_t ppr, uint16_t ppsMax, uint16_t dWheel, uint16_t lBase);

  /**
   * Set the vehicle constants
   *
   * Sets the number of pulses per encoder revolution, maximum speed reading and
   * important dimensions for the vehicle. This depends on the hardware and could
   * vary between different vehicle configurations.
   *
   * For encoder ppr, there is only one value for all whole vehicle, so all
   * encoders need to operate the same way.
   * 
   * \sa begin(), \ref pageUsingLibrary
   *
   * \param ppr    Number of encoder pulses per wheel revolution.
   * \param ppsMax Maximum number of encoder pulses per second at top speed (100% velocity).
   * \param dWheel Wheel diameter in mm.
   * \param lBase  Base length (distance between wheel centers) in mm
   */
  void setVehicleParameters(uint16_t ppr, uint16_t ppsMax, uint16_t dWheel, uint16_t lBase);

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
   * Check if specific motor is running
   *
   * Check if motors are commanded to run. This method is useful to check when
   * drive() or move() have completed their motions.
   *
   * \param mtr  The motor number being queried [0..MAX_MOTOR-1]
   * \return true if any of the motors are not idle
   */
  bool isRunning(uint8_t mtr) { return(mtr < MAX_MOTOR ? _mData[mtr].state != S_IDLE : false); }
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
    * \sa getLinearVelocity(), getAngularVelocity(), setPIDTuning()
    *
    * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
    * \param vAngularD the angular velocity in degrees per second [-90..90].
    */
  void drive(int8_t vLinear, int8_t vAngularD) { drive(vLinear, deg2rad(vAngularD)); }

  /**
   * Drive the vehicle along a straight path.
   *
   * Run the vehicle along a straight path with the specified velocity.
   * Moves the motors under PID control.
   *
   * The velocity is specified as a percentage of the maximum vehicle velocity [0..100].
   * Positive velocity move the vehicle forward, negative moves it in reverse.
   *
   * \sa getLinearVelocity(), setPIDTuning()
   *
   * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
   */
  void drive(int8_t vLinear) { drive(vLinear, (float)0.0); }

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
   * \sa getLinearVelocity(), getAngularVelocity(), setPIDTuning()
   *
   * \param vLinear   the linear velocity as a percentage of full scale [-100..100].
   * \param vAngularR the angular velocity in radians per second [-pi/2..pi/2].
   */
  void drive(int8_t vLinear, float vAngularR);

  /**
   * Stop the smart car.
   *
   * This method will sets all velocities to 0 and disables all the motor functions to
   * bring the smart car to a complete stop.
   *
   * \sa setSpeed()
   */
  void stop(void);

  /**
   * Set the linear velocity
   *
   * Sets the linear velocity without changing any other parameters. Useful for
   * adjusting the speed when already in motion.
   *
   * The velocity is specified as a percentage of the maximum vehicle velocity [0..100].
   * Positive velocity move the vehicle forward, negative moves it in reverse.
   *
   * /sa getLinearVelocity(), drive()
   *
   * \param vel the new value for the linear velocity [-100..100].
   */
  void setLinearVelocity(int8_t vel);

  /**
   * Get the current linear velocity.
   *
   * Linear velocity is expressed as a percentage of the maximum velocity [0..100].
   * The Master velocity is used to regulate all the speed functions for the motors.
   *
   * \sa drive()
   *
   * \return the current linear speed setting.
   */
  inline int8_t getLinearVelocity(void) { return(_vLinear); }

  /**
   * Set the angular velocity (radians).
   *
   * Sets the angular velocity without changing any other parameters. Useful for
   * adjusting turning when already in motion.
   *
   * Angular velocity is expressed in radians relative to the forward direction
   * [-PI/2..PI/2]. Positive angle is turn to the right, negative left.
   *
   * \sa getAngularVelocity(), drive()
   *
   * \param angR the new turning rate in radians.
   */
  inline void setAngularVelocity(float angR) { drive(_vLinear, angR); }

  /**
   * Set the angular velocity (degrees).
   *
   * Sets the angular velocity without changing any other parameters. Useful for
   * adjusting turning when already in motion.
   *
   * Angular velocity is expressed in degrees relative to the forward direction
   * [-90..90]. Positive angle is turn to the right, negative left.
   *
   * \sa getAngularVelocity(), drive()
   *
   * \param angD the new turning rate in degrees.
   */
  inline void setAngularVelocity(int8_t angD) { drive(_vLinear, deg2rad(angD)); }

  /**
   * Get the current angular velocity.
   *
   * Angular velocity is expressed in radians relative to the forward direction
   * [-PI/2..PI/2]. Positive angle is turn to the right, negative left.
   *
   * \sa drive()
   *
   * \return the current angular speed setting.
   */
  inline float getAngularVelocity(void) { return(_vAngular); }

  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for precision movements of the vehicle.
   * @{
   */
   /**
   * Precisely move the vehicle (radians).
   *
   * Controls the movement by counting the encoder pulses rather that PID,
   * which should make it more precise and controlled. This is useful for specific
   * movements run at slow speed.
   *
   * The call to move() specifies the angle each wheels will turn, independently.
   * This method is designed to allow close movements such as spin-in-place or
   * other short precise motions.
   *
   * The motion for each wheel is specified as speed as the total angle subtended
   * by the turned by the wheel in radians. Negative angle is a reverse wheel
   * rotation.
   *
   * \sa drive(), spin(), setMoveSP(), len2rad()
   *
   * \param angL left wheel angle subtended by the motion in radians.
   * \param angR right wheel angle subtended by the motion in radians.
   */
  void move(float angL, float angR);

  /**
  * Precisely move the vehicle (degrees).
  *
  * Controls the movement by counting the encoder pulses rather that PID,
  * which should make it more precise and controlled. This is useful for specific
  * movements run at slow speed.
  *
  * The call to move() specifies the precise motion of the motors, independently.
  * This method is designed to allow close movements such as spin-in-place or
  * other short precise motions.
  *
  * The motion for each wheel is specified as speed as the total angle subtended
  * by the turned by the wheel in degrees. Negative angle is a reverse wheel rotation.
  *
  * \sa drive(), spin(), setMoveSP()
  *
  * \param angL left wheel angle subtended by the motion in degrees.
  * \param angR right wheel angle subtended by the motion in degrees.
  */
  void move(int16_t angL, int16_t angR) { move(deg2rad(angL), deg2rad(angR)); }

  /**
  * Precisely move the vehicle (millimeter).
  *
  * Controls the movement by counting the encoder pulses rather that PID,
  * which should make it more precise and controlled. This is useful for specific
  * movements run at slow speed.
  *
  * The call to move() specifies the precise motion of the motors, independently.
  * This method is designed to allow close movements such as spin-in-place or
  * other short precise motions.
  *
  * The motion for each wheel will be identical to move the vehicle the required
  * distance. Negative length is a reverse wheel rotation.
  *
  * \sa drive(), spin(), setMoveSP()
  *
  * \param len distance to move in mm.
  */
  void move(int16_t len) { move(len2rad(len), len2rad(len)); }

  /**
  * Precisely spin the vehicle.
  *
  * Controls the movement by spinning the vehicle about its central vertical
  * axis. It works similar to move() to spin the wheels in an directions to
  * effect the turning motion.
  *
  * The call to spin() specifies the percentage (-100 to 100) of the full circle
  * rotation about the central axis passing through the vehicle base length.
  * Positive angle is a turn to the right, negative to the left.
  *
  * \sa drive(), move(), setMoveSP()
  *
  * \param fraction Percentage fraction of full revolution [-100..100]. Positive spins right; negative pins left.
  */
  void spin(int16_t fraction);

  /**
   * Start an action sequence stored in PROGMEM.
   * 
   * This method passed the reference to an action sequence array stored in PROGMEM
   * to the library for background execution.
   * 
   * Details on actions sequences can be found at \ref pageActionSequence
   * 
   * \sa isSequenceComplete()
   * 
   * \param actionList pointer to the array of actionItem_t ending with and END record.
   */
  void startSequence(const actionItem_t* actionList);

  /** 
   * Start an action sequence stored in RAM.
   * 
   * This method passed the reference to an action sequence array stored in RAM
   * to the library for background execution. The array must remain in scope
   * (ie, global or static declaration) for the duration of the sequence 
   * running.
   *
   * Details on actions sequences can be found at \ref pageActionSequence
   *
   * \sa isSequenceComplete()
   *
   * \param actionList pointer to the array of actionItem_t ending with and END record.
   */
  void startSequence(actionItem_t* actionList);

  /**
   * Check if the current action sequence has completed.
   * 
   * Once an action sequqnce is started it will automatically execute to 
   * completion unless interrupted. This method checks to see if the
   * action sequqnce has completed.
   * 
   * \sa startSequence()
   * 
   * \return true if the sequence has finished executing
   */
  bool isSequenceComplete(void) { return(!_inSequence); }

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
   * stopped and should be determined as the minimum that reliably gets a
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
   * stopped and should be determined as the minimum that reliably gets a
   * motor rotation started.
   *
   * \sa drive(), setKickerSP(), saveConfig()
   *
   * \return the previously configured speed.
   */
  uint8_t getKickerSP(void) { return(_config.kickerPWM); }

  /**
   * Set the spin adjustment factor.
   *
   * Set the spin() inertia adjustment. When the vehicle is spinning
   * and the motors turn off, it may continue spinning due to its inertia.
   * This factor is applied to proportionatley reduce the pulses used 
   * to execute the spin to account for the additional free spin movement 
   * after the powered spin stops.
   *
   * \sa spin(), getSpinSP(), saveConfig()
   *
   * \param adjust factor applied to the spin pulses (pulses * adjust)
   */
  void setSpinSP(float adjust) { _config.spinAdjust = adjust; }

  /**
   * Get the spin adjustment factor.
   *
   * Get the spin() inertia adjustment. When the vehicle is spinning
   * and the motors turn off, it may continue spinning due to its inertia.
   * This factor is applied to proportionatley reduce the pulses used
   * to execute the spin to account for the additional free spin movement
   * after the powered spin stops.
   *
   * \sa spin(), getSpinSP(), saveConfig()
   *
   * \return the previously configured spin adjustment.
   */
  float getSpinSP(void) { return(_config.spinAdjust); }

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

  /**
   * Read pulses per encoder revolution
   *
   * Returns the number of pulses per encoder revolution. This may be needed to 
   * change from number of pulses to revolutions and then distance.
   *
   * \sa setVehicleParameters()
   *
   * \return The number of pulses per revolution.
   */
  inline uint16_t getPulsePerRev() { return(_ppr); }

  /** @} */
  //--------------------------------------------------------------
  /** \name Utility methods.
   * @{
   */
  /**
   * Convert degrees to radians.
   *
   * Convert the degrees measure specified into radians.
   *
   * \param deg the value in degrees to be converted.
   * \return the converted value
   */
  inline float deg2rad(int16_t deg) { return((PI * (float)deg) / 180.0); }

  /**
   * Convert a length to angle of wheel rotation.
   * 
   * Convert a length in mm to travel into the radan of wheel rotation 
   * required for that travel.
   * 
   * \param len length in mm to convert.
   * \return the angle in radians of wheel rotation to achieve that distance.
   */
  inline float len2rad(int16_t len) { return(((float)len * 2 * PI) / (_lenPerPulse * _ppr)); }

  /** @} */

private:
  // Motor array indices
  const uint8_t MLEFT = 0;      ///< Array index for the Left motor
  const uint8_t MRIGHT = 1;     ///< Array index for the right motor

  enum runState_t { S_IDLE, S_DRIVE_INIT, S_DRIVE_KICKER, S_DRIVE_PIDRST, S_DRIVE_RUN, S_MOVE_INIT, S_MOVE_RUN };

  float _vMaxLinear;      ///< Maximum linear speed in pulses/second 
  int16_t _vLinear;       ///< Master velocity setting as percentage [0..100] = [0.._vMaxLinear]
  float _vAngular;        ///< angular velocity in in radians per second [-PI..PI]

  // Vehicle constants
  uint16_t _ppr;          ///< Encoder pulses per wheel revolution
  uint16_t _diaWheel;     ///< Wheel diameter in mm
  uint16_t _lenBase;      ///< Base length in mm (distance between wheel centers)
  uint16_t _ppsMax;       ///< Encoder maximum pulse per second (full speed reading)

  float _lenPerPulse;     ///< Length travelled per pulse of wheel revolution
  float _diaWheelP;       ///< Wheel diameter in pulses (calculated)
  float _lenBaseP;        ///< Base Length in pulses (calculated)

  // Data for tracking action sequences
  bool _inSequence;       ///< true if currently executing a sequence
  bool _inAction;         ///< waiting for current item to complete
  bool _seqIsConstant;    ///< true if sequence is stored is declared in PROGMEM
  union 
  {                 ///< current list of actions being sequenced
    const actionItem_t* cp; 
    actionItem_t* p;
  } _uAction;
  uint8_t _curActionItem; ///< index for the current action item
  actionItem_t _ai;       ///< current action item
  uint32_t _timeStartSeq; ///< generic time variable for sequences

  // Define the control objects
  SC_DCMotor* _M[MAX_MOTOR];      ///< Motor controllers
  SC_MotorEncoder* _E[MAX_MOTOR]; ///< Motor encoders for feedback

  // Configuration data that is saved to EEPROM
  struct
  {
    uint8_t sig[2];       ///< config signature bytes

    // PWM values
    uint8_t minPWM;       ///< the min PWM setting for DC motors
    uint8_t maxPWM;       ///< the max PWM setting for DC motors
    uint8_t movePWM;      ///< the creep PWM setting for DC motors
    uint8_t kickerPWM;    ///< kicker to overcome static friction from stop position
    float spinAdjust;     ///< spin intertia adjustment

    // PID values
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

  // Private Methods
  void printConfig(void);               ///< debug only
  void setPIDOutputLimits(void);        ///< set the PID limits for all motors

  void startSeqCommon(void);            ///< common part of sequence start
  void runSequence(void);               ///< keep running current sequence
  bool runActionItem(actionItem_t& ai); ///< run the logic for this action item

};
