#pragma once
/**
 * \file
 * \brief Header file for the SC_PID class.
 */

 /**
 \page pagePID PID Controller

 ## SmartCar PID Controller

 The PID controller is a mashup taken from a number of sources and tailored
 to suit this application.
 Elements are taken from:
 - QuickPID by dlloydev https://github.com/Dlloydev/QuickPID
 - MiniPID by tekdemo https://github.com/tekdemo/MiniPID
 - Arduino PID library (PID_v1) by br3ttb (Brett Beauregard) https://github.com/br3ttb/Arduino-PID-Library/

 Ideas also taken from a detailed explanation PID coding at
 http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
 */

#include <Arduino.h>

/**
 * Core object for the SC_PID class
 * Implements PID control algorithm DC motor control, as a hybrid fixed-point and 
 * floating-point PID controller
 */
class SC_PID
{
public:
  //--------------------------------------------------------------
  /** \name Enumerated Types and Constants.
   * @{
   */
   /**
    * Different possible modes for the controller execution
    */
  typedef enum
  {
    AUTO,    ///< The controller will keep track of time and execute one PID iteration when time expires.
    USER,    ///< The user is responsible for calling the PID computation when the calculation time interval expires.
    OFF,     ///< The controlled variable is controlled manually (ie, no PID control).
  } mode_t;

  /**
   * Different ways the controller can operate
   */
  typedef enum
  {
    DIRECT, ///< An increase in the control output increases the controlled variable.
    REVERSE ///< An increase in the control output decreases the controlled variable. 
  } control_t;
  /** @} */

 //--------------------------------------------------------------
 /** \name Class constructor and destructor.
  * @{
  */
  /**
   * Class Constructor.
   *
   * Instantiate a new instance of the class.
   *
   * The main function for the core object is to reset the internal
   * shared variables and timers to default values.
   * 
   * \sa setTuning(), setMode()
   *
   * \param cv Pointer to the current value for the controlled variable.
   * \param co Pointer to the control output for the controlled variable.
   * \param sp Pointer to the setpoint for the controlled variable.
   * \param Kp Proportional coefficient for the PID algorithm.
   * \param Ki Integral coefficient for the PID algorithm.
   * \param Kd Derivative coefficient for the PID algorithm.
   * \param pOn Factor for combining errors (proportional on Error or Measurement).
   * \param controller The type of controller (one of the controller_t values).
   */
  SC_PID(int16_t* cv, int16_t* co, int16_t* sp, float Kp, float Ki, float Kd,
    float pOn = 1.0, control_t controller = DIRECT);
  /**
   * Class Destructor.
   *
   * Release any allocated memory and clean up anything else.
   */
  ~SC_PID(void) {}
  /** @} */

  //--------------------------------------------------------------
  /** \name Methods for core object control.
   * @{
   */

   /**
    * Perform PID calculation.
    *
    * Perform the next step in the PID calculation. How this occurs will depend on
    * the current controller mode (AUTO, USER, OFF) and the time period set.
    *
    * In AUTO mode the method should be called frequently (ideally every time through loop())
    * and the return value will inform the application when the calculation actually happened.
    *
    * In USER mode, the method will run when it is invoked under the assumption that the
    * time elapsed between steps is the currently set PID calculation period.
    *
    * In OFF mode (default) the calculation is never executed.
    *
    * The current value, setpoint and new control output used are located by the pointers
    * passed to the constructor.
    *
    * \sa setMode(), setPIDPeriod(), setOutputLimits()
    *
    * \return true if the calculation was completed, false otherwise.
    */
  bool compute(void);

  /**
   * Set the calculation period.
   *
   * Set the calculation time in milliseconds between iterations of the PID calculation.
   * This value must be set for both USER and AUTO modes. Default is 100 ms.
   *
   * \sa setMode()
   *
   * \param newPeriod the new PID calculation period in milliseconds.
   */
  void setPIDPeriod(uint32_t newPeriod);

  /**
   * Set the controller mode.
   *
   * Sets the controller mode to one of the mode_t values
   *
   * In AUTO mode the library tracks time and executes the PID calculation
   * when the correct period has expired.
   *
   * In USER mode, the library does not track time and the PID calculation
   * will be run whenever it is invoked.
   *
   * In OFF mode (default) the PID calculation is never executed.
   *
   * \sa compute()
   *
   * \param newMode The new mode for the controller (default OFF).
   */
  void setMode(mode_t newMode);

  /**
   * Set the PID control limits
   *
   * Sets the upper and lower values for the control output. The value will always be
   * clamped to this range. The default is [0, 255], which suits an Arduino PWM output.
   *
   * \param min The low value of the permissible range (default 0).
   * \param max The high value for the permissible range (=default 255).
   */
  void setOutputLimits(int16_t min, int16_t max);

  /**
   * Set the PID coefficients and Proportional on Error.
   *
   * Allows the PID coefficients and proportional on error factor to be
   * changed during run time. These will replace the current coefficients,
   * including those with the constructor.
   *
   * This method can be used to implement adaptive control or during
   * tuning of the PID control loop.
   *
   * The is the Proportional on Error weighting value is expressed as 
   * a number between 0.0 and 1.0. This controls the mix of Proportional 
   * on Error (PonE) and Proportional on Measurement (PonM) that's used in 
   * the compute algorithm. Note that POn controls the PonE amount, whereas 
   * the remainder (1-PonE) is the PonM amount.
   *
   * \param Kp The new Proportional coefficient.
   * \param Ki The new Integral coefficient.
   * \param Kd The new Derivative coefficient.
   * \param pOn The new proportional on Error coefficient (default 1.0).
   */
  void setTuning(float Kp, float Ki, float Kd, float pOn);

  /**
   * Set the PID coefficients only.
   *
   * This overload of the method allows the PID coefficients to be changed during
   * run time. These will replace the current coefficients, including those supplied
   * with the constructor.
   *
   * This method can be used to implement adaptive control or during tuning of the
   * PID control loop.
   *
   * \param Kp The new Proportional coefficient.
   * \param Ki The new Integral coefficient.
   * \param Kd The new Derivative coefficient.
   */
  void setTuning(float Kp, float Ki, float Kd) { setTuning(Kp, Ki, Kd, _pOn); };
  /** @} */

 //--------------------------------------------------------------
 /** \name Utility Functions.
  * @{
  */
  /**
   * Set the controller type.
   *
   * Sets the controller type. This is already set in the constructor and is unlikely to change
   * for a given control application.
   *
   * For DIRECT controllers an increase in the output causes an increase in the input.
   * The opposite is true for REVERSE controllers.
   *
   * \param cType The new control type (one of control_t).
   */
  void setControlType(control_t cType);

  /**
   * Return the current PID error value.
   *
   * \return The error value resulting from the last PID calculation step.
   */
  inline int16_t getError(void) { return (_error); }

  /**
   * Return the current Kp value.
   *
   * \return The Kp coefficient for PID calculation.
   */
  inline float getKp(void) { return(_userKp); }

  /**
   * Return the current Ki value.
   *
   * \return The Ki coefficient for PID calculation.
   */
  inline float getKi(void) { return(_userKi); }

  /**
   * Return the current Kd value.
   *
   * \return The Kd coefficient for PID calculation.
   */
  inline float getKd(void) { return(_userKd); }

  /**
   * Return the current PID calculation period.
   *
   * \return The calculation period in milliseconds.
   */
  inline uint32_t getPIDPeriod(void) { return(_pidPeriod); }

  /**
   * Return the current controller mode.
   *
   * \return The controller mode (one of the mode_t values).
   */
  inline mode_t getMode(void) { return(_mode); }

  /**
   * Return the current controller type.
   *
   * \return The controller type (one of the controller_t values).
   */
  inline control_t getControlType(void) { return(_controller); }
  /** @} */

private:
  float _userKp, _userKi, _userKd;      ///< Original user supplied Kp, Ki, Kd
  float _kp, _ki, _kd;    ///< P, I and D tuning parameters for calculations
  float _pOn;             ///< Proportional on Error/Measurement combination factor (0.0 .. 1.0). Default = 1.0, 100% PoE/0% PoM
  float _kpi;             ///< Proportional on error amount
  float _kpd;             ///< Proportional on measurement amount

  control_t _controller;  ///< type of controller being (DIRECT, REVERSE)
  mode_t  _mode;          ///< current controller mode (OFF, AUTO, USER)

  int16_t* _cv;   ///< Pointer to the location for PID current value (input)
  int16_t* _co;   ///< Pointer to place PID control output (output)
  int16_t* _sp;   ///< Pointer to the PID setpoint

  uint32_t _pidPeriod;      ///< PID calculation period in milliseconds
  uint32_t _lastTime;       ///< Last time PID was calculated (millis() value)
  int16_t _outMin, _outMax; ///< Control output min and max values 
  int16_t _error;           ///< PID error accumulator
  int16_t _prevCv;          ///< PID previous current value (for PoM calcs)
  int16_t _prevCo;          ///< Control output calculated at the last iteration

  void reset(void);                ///< reset when changing modes
  int16_t clampOutput(int16_t value);   ///< clamp the output to be in the range [_outMin, _outMax]

  inline int32_t FL_FX(float a) { return(a * 256.0);}                   ///< float to fixed point
  inline int32_t FX_MUL(int32_t a, int32_t b) { return((a * b) >> 8); } ///< fixed point multiply
};