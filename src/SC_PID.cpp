/**
 * \file
 * \brief Class definition file for the SC_PID class.
 */

#include <SC_PID.h>

SC_PID::SC_PID(int16_t* cv, int16_t* co, int16_t* sp,
               float Kp, float Ki, float Kd, float pOn, control_t control):
    _pOn(pOn), _mode(OFF), _cv(cv), _co(co), _sp(sp), _pidPeriod(100)
{
  setOutputLimits(0, 255);
  setTuning(Kp, Ki, Kd, pOn);
  setControlType(control);

  _lastTime = millis() - _pidPeriod;
}

bool SC_PID::compute(void)
{
  if ((_mode == OFF) || 
     ((_mode == AUTO) && (millis() - _lastTime < _pidPeriod)))
    return(false);

  // Compute all the working error variables
  int16_t dCv = *_cv - _prevCv;

  _error = *_sp - *_cv;

  // Working error, proportional distribution and PID output
  if (_kpi < 31 && _kpd < 31) 
    _prevCo += FX_MUL(FL_FX(_kpi), _error) - FX_MUL(FL_FX(_kpd), dCv);
  else 
    _prevCo += (_kpi * _error) - (_kpd * dCv);

  *_co = clampOutput(_prevCo);

  // Remember some variables for next time
  _prevCv = *_cv;
  _lastTime = millis();

  return(true);
}

void SC_PID::setTuning(float Kp, float Ki, float Kd, float pOn)
{
  float pidFrequency = (float)_pidPeriod / 1000.0;

  if (Kp < 0 || Ki < 0 || Kd < 0 || pOn < 0.0 || pOn > 1.0)
    return;

  // copy values over
  _pOn = pOn;
  _userKp = Kp; 
  _userKi = Ki; 
  _userKd = Kd;

  // recalculate some dependent results
  _kp = Kp;
  _ki = Ki * pidFrequency;
  _kd = Kd / pidFrequency;
  _kpi = (_kp * _pOn) + _ki;
  _kpd = _kp * (1 - _pOn) + _kd;

  if (_controller == REVERSE)
  {
    _kp = -_kp;
    _ki = -_ki;
    _kd = -_kd;
  }
}

void SC_PID::setPIDPeriod(uint32_t newPeriod)
{
  if (newPeriod == 0) return;

  float ratio  = (float)newPeriod / (float)_pidPeriod;
  _ki *= ratio;
  _kd /= ratio;

  _pidPeriod = newPeriod;
}

void SC_PID::setOutputLimits(int16_t min, int16_t max)
{
  if (min >= max) 
    return;

  // copy values over
  _outMin = min;
  _outMax = max;

  // sanity check current output
  if (_mode != OFF)
  {
    *_co = clampOutput(*_co);
    _prevCo = clampOutput(_prevCo);
  }
}

void SC_PID::setMode(mode_t newMode)
{
  // reset the controller if we have just turn it on
  if (_mode == OFF && newMode != OFF) 
    reset();  
  _mode = newMode;
}

void SC_PID::reset(void)
{
  _prevCv = *_cv;
  _prevCo = clampOutput(*_co);
  _lastTime = millis();
  _error = 0;
}

inline int16_t SC_PID::clampOutput(int16_t value)
{
  if (value > _outMax)
    return(_outMax);
  else if (value < _outMin)
    return(_outMin);

  return(value);
}

void SC_PID::setControlType(control_t cType)
{
  if (cType != _controller)
  {
    _kp = -_kp;
    _ki = -_ki;
    _kd = -_kd;
  }
  _controller = cType;
}
