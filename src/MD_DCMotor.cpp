/**
* \file
* \brief Class definition for the MD_DCMotor class.
*/

#include "MD_DCMotor.h"

MD_DCMotor::MD_DCMotor(uint8_t pinInA, uint8_t pinInB, uint8_t pinEn) : 
_pinInA(pinInA), _pinInB(pinInB), _pinEn(pinEn) 
{
}

MD_DCMotor::MD_DCMotor(uint8_t pinInA, uint8_t pinInB) : 
_pinInA(pinInA), _pinInB(pinInB), _pinEn(NO_PIN) 
{
}

MD_DCMotor::~MD_DCMotor(void)
{
  setMode(DIR_REL); 
}

void MD_DCMotor::begin(void)
{
  pinMode(_pinInA, OUTPUT);
  pinMode(_pinInB, OUTPUT);
  if (_pinEn != NO_PIN)
    pinMode(_pinEn, OUTPUT);

  setMode(DIR_REL);
}
