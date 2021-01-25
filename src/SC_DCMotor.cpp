/**
* \file
* \brief Class definition for the SC_DCMotor class.
*/

#include "SC_DCMotor.h"

void SC_DCMotor_L298::begin(void)
{
  pinMode(_pinIn1, OUTPUT);
  pinMode(_pinIn2, OUTPUT);
  pinMode(_pinEn, OUTPUT);

  setMode(DIR_STOP);
}

void SC_DCMotor_L298::setSpeed(uint16_t s)
{
  if (s > 255) s = 255;
  _speed = s;
  analogWrite(_pinEn, _speed);
}

void SC_DCMotor_L298::setMode(runCmd_t cmd)
// Set the mode bits for the controller.
{
  _mode = cmd;
  switch (_mode)
  {
  case DIR_FWD:  digitalWrite(_pinIn1, LOW);  digitalWrite(_pinIn2, HIGH); break;
  case DIR_REV:  digitalWrite(_pinIn1, HIGH); digitalWrite(_pinIn2, LOW);  break;
  case DIR_STOP: digitalWrite(_pinIn1, HIGH); digitalWrite(_pinIn2, HIGH); break;
  }
}

void SC_DCMotor_M1508::begin(void)
{
  pinMode(_pinIn[0], OUTPUT);
  pinMode(_pinIn[1], OUTPUT);

  setMode(DIR_STOP);
}

void SC_DCMotor_M1508::setSpeed(uint16_t s)
{
  if (s > 255) s = 255;
  _speed = s;

  // direction with slow decay (coast)
  digitalWrite(_pinIn[_pinPWM == 0 ? 1 : 0], LOW);
  analogWrite(_pinIn[_pinPWM], _speed);
}

void SC_DCMotor_M1508::setMode(runCmd_t cmd)
// Set the right mode in the controller.
{
  _mode = cmd;
  _pinPWM = (_mode == DIR_FWD ? 0 : 1);  // arbitrary assignment
  if (_mode != DIR_STOP)
    setSpeed(getSpeed());
  else
  {
    // set as stop with fast decay (braking)
    digitalWrite(_pinIn[0], HIGH);
    digitalWrite(_pinIn[1], HIGH);
  }
}
