#include <MD_SmartCar.h>

/**
 * \file
 * \brief Code file for SmartCar miscellaneous methods.
 */

void MD_SmartCar::setLinearVelocity(int8_t vel)
{
  if (vel == 0)
    stop();
  else
    drive(vel, _vAngular);
}

bool MD_SmartCar::isRunning(void)
// check if any of the motors are running
{
  bool b = true;

  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    b &= _mData[i].state != S_IDLE;

  return(b);
}

void MD_SmartCar::setPIDOutputLimits(void)
{
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    _mData[i].pid->setOutputLimits(getMinMotorSP(), getMaxMotorSP());
}

void MD_SmartCar::setPIDTuning(uint8_t mtr, float Kp, float Ki, float Kd)
{
  if (mtr < MAX_MOTOR)
  {
    _config.Kp[mtr] = Kp;
    _config.Ki[mtr] = Ki;
    _config.Kd[mtr] = Kd;
    _mData[mtr].pid->setTuning(Kp, Ki, Kd);
  }
}

void MD_SmartCar::getPIDTuning(uint8_t mtr, float& Kp, float& Ki, float& Kd) 
{ 
  if (mtr < MAX_MOTOR)
  {
    Kp = _config.Kp[mtr];
    Ki = _config.Ki[mtr];
    Kd = _config.Kd[mtr];
  }
}

bool MD_SmartCar::setMoveSP(uint8_t units)
{
  if (units >= _config.minPWM && units <= _config.maxPWM)
    _config.movePWM = units;

  return(units = _config.movePWM);
}

bool MD_SmartCar::setKickerSP(uint8_t units)
{
  _config.kickerPWM = units;

  return(units = _config.kickerPWM);
}

void MD_SmartCar::setMinMotorSP(uint8_t units) 
{ 
  if (units < _config.maxPWM) 
    _config.minPWM = units; 

  if (_config.movePWM < _config.minPWM)
    _config.movePWM = _config.minPWM;

  setPIDOutputLimits(); 
}

void MD_SmartCar::setMaxMotorSP(uint8_t units) 
{ 
  if (units > _config.minPWM) 
    _config.maxPWM = units; 

  if (_config.movePWM > _config.minPWM)
    _config.movePWM = _config.maxPWM;

  setPIDOutputLimits(); 
}

void MD_SmartCar::setVehicleParameters(uint16_t ppr, uint16_t ppsMax, uint16_t dWheel, uint16_t lBase)
{
  // save valid values or sensible defaults
  _ppr = (ppr != 0) ? ppr : PPR_DEF;
  _ppsMax = (ppsMax != 0) ? ppsMax : PPS_MAX_DEF;
  _diaWheel = (dWheel != 0) ? dWheel : DIA_WHEEL_DEF;
  _lenBase = (lBase != 0) ? lBase : LEN_BASE_DEF;

  // now calculate derived constants
  float lenPerPulse = (PI * (float)_diaWheel) / (float)_ppr;   // distance traveled per encoder pulse

  _diaWheelP = _diaWheel / lenPerPulse;   // wheel diameter converted to pulses
  _lenBaseP = _lenBase / lenPerPulse;     // base length converted to pulses
  SCPRINT("\nWheel dia (P): ", _diaWheelP);
  SCPRINT("\nBase Len (P): ", _lenBaseP);
}
