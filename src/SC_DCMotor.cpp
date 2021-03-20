/**
* \file
* \brief Class definition for the SC_DCMotor class.
*/

#include "SC_DCMotor.h"

bool SC_DCMotor_L29x::begin(void)
{
  bool b = true;

  pinMode(_pinIn1, OUTPUT);
  pinMode(_pinIn2, OUTPUT);
  pinMode(_pinEn, OUTPUT);

#if USE_PWM_LIBRARY
  pwm->begin(PWM_FREQ);
#endif

  return(b);
}

void SC_DCMotor_L29x::setSpeed(uint16_t s)
{
  if (s > 255) s = 255;
  _speed = s;
#if USE_PWM_LIBRARY
  pwm->write(_speed);
#else
  analogWrite(_pinEn, _speed);
#endif
}

void SC_DCMotor_L29x::setMode(runCmd_t cmd)
// Set the mode bits for the controller.
{
  _mode = cmd;
  switch (_mode)
  {
  case DIR_FWD:  digitalWrite(_pinIn1, LOW);  digitalWrite(_pinIn2, HIGH); break;
  case DIR_REV:  digitalWrite(_pinIn1, HIGH); digitalWrite(_pinIn2, LOW);  break;
  }
}

bool SC_DCMotor_MX1508::begin(void)
{
  bool b = true;

  pinMode(_pinIn[0], OUTPUT);
  pinMode(_pinIn[1], OUTPUT);
#if USE_PWM_LIBRARY
  pwm[0]->begin(PWM_FREQ);
  pwm[1]->begin(PWM_FREQ);
#endif

  setMode(_mode);   // set the current PWM pin number

  return(b);
}

void SC_DCMotor_MX1508::setSpeed(uint16_t s)
{
  if (s > 255) s = 255;
  _speed = s;

  // direction with slow decay (coasting)
  digitalWrite(_pinIn[_pinPWM ? 0 : 1], LOW);   // use the alternative pin to _pinPWM
#if USE_PWM_LIBRARY
  pwm[_pinPWM]->write(_speed);
#else
  analogWrite(_pinIn[_pinPWM], _speed);
#endif
}

void SC_DCMotor_MX1508::setMode(runCmd_t cmd)
// Set the right mode in the controller.
{
  _mode = cmd;
  _pinPWM = (_mode == DIR_FWD ? 0 : 1);  // arbitrary assignment
  setSpeed(getSpeed());
}
