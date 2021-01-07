#include <MD_SmartCar.h>

/**
 * \file
 * \brief Code file for SmartCar library class.
 */

MD_SmartCar::MD_SmartCar(MD_DCMotor *ml, MD_MotorEncoder *el, MD_DCMotor *mr, MD_MotorEncoder *er)
{
  // Allocate the pointer to the right array reference
  _M[MLEFT] = ml;
  _M[MRIGHT] = mr;
  _E[MLEFT] = el;
  _E[MRIGHT] = er;
}

MD_SmartCar::~MD_SmartCar(void) 
{
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    delete _mData[i].pid;
}

bool MD_SmartCar::begin(void) 
// Return false if any of the encoders fail to begin
{ 
  bool b = true;

  loadConfig();

  // do PID initilization
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].pid = new PID(&_mData[i].cv, &_mData[i].co, &_mData[i].sp,
      _config.Kp, _config.Ki, _config.Kd, P_ON_E, DIRECT);

    _mData[i].pid->SetSampleTime(PID_PERIOD);
    _mData[i].pid->SetMode(AUTOMATIC);
    _mData[i].state = S_IDLE;
  }
  setPIDOutputLimits();

  // Initialise the motors and encoders
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _M[i]->begin();
    b &= _E[i]->begin();
  }

  return(b);
}

void MD_SmartCar::setPIDOutputLimits(void)
{
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    _mData[i].pid->SetOutputLimits(getMinMotorSP(), getMaxMotorSP());
}

void MD_SmartCar::move(void)
{
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    switch (_mData[i].state)
    {
    case S_IDLE: break;     // do nothing

    // --- FREE RUNNING
    case S_RUN_INIT:
      PRINTS("\n>>RUN_INIT");
      _E[i]->reset();
      _mData[i].state = S_RUN;
      // deliberately fall through to CREEP

    case S_RUN:
      //if (millis() - _mData[i].timeLast >= PID_PERIOD)
      {
        uint32_t time;
        uint16_t count;

        // run the PID loop to keep things on even keel
        _E[i]->read(time, count, false);   // read and reset the encoder counter
        if (time != 0)    // skip calcs to avoid divide-by-zero
        {
          _mData[i].cv = count;
          if (_mData[i].pid->Compute())
          {
            _E[i]->reset();
            if (i == 0)   // only print the SP once each iteration
            {
              PRINT("\nPID SP ", _mData[i].sp);
              PRINTS(" CV");
            }
            PRINT(" [", i);
            PRINT("]", _mData[i].cv);
            _M[i]->run(_path.mspec[i].dir, _mData[i].co);
          }
        }
      }
      break;

    // --- CREEPING
    case S_CREEP_INIT:
      PRINTS("\n>>CREEP_INIT");
      _E[i]->reset();
      _M[i]->run(_path.mspec[i].dir, _config.creepSP);
      _mData[i].state = S_CREEP;
      // deliberately fall through to CREEP

    case S_CREEP:
      {
        uint32_t time;
        uint16_t count;

        _E[i]->read(time, count, false);
        PRINT("\n>>CREEP ", count);
        if (count >= _path.mspec[i].count)
          _mData[i].state = S_CREEP_END;
      }
      break;

    case S_CREEP_END:
      PRINTS("\n>>CREEP_END");
      _M[i]->run(MD_DCMotor::DIR_BRK, 0);
      _mData[i].state = S_IDLE;
      break;

    default: _mData[i].state = S_IDLE; break;
    }
  }
}

void MD_SmartCar::setSpeed(uint16_t speed)
{
  _speed = speed;
  if (_speed == 0) 
    stop();
  else
    updateSetpoint();
}

void MD_SmartCar::updateSetpoint(void)
{
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    _mData[i].sp = (double)_speed / ((double)MS_PER_SEC / (double)PID_PERIOD);
}

void MD_SmartCar::run(pathType_t p, sideType_t s, dirType_t d, uint16_t speed)
{
  if (findPath(&_path, p, s, d, RUN, pathTable, ARRAY_SIZE(pathTable)))
  {
    setSpeed(speed);
    _mData[MLEFT].state = _mData[MRIGHT].state = S_RUN_INIT;
  }
}

void MD_SmartCar::creep(pathType_t p, sideType_t s, dirType_t d)
{
  if (findPath(&_path, p, s, d, CREEP, pathTable, ARRAY_SIZE(pathTable)))
    _mData[MLEFT].state = _mData[MRIGHT].state = S_CREEP_INIT;
}

bool MD_SmartCar::setCreepSP(uint16_t units)
{
  if (units >= _config.minPWM && units <= _config.maxPWM)
    _config.creepSP = units;

  return(units = _config.creepSP);
}

void MD_SmartCar::stop(void)
{
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].sp = 0;
    _mData[i].state = S_IDLE;
    _M[i]->run(MD_DCMotor::DIR_BRK, 0);
  }
}

void MD_SmartCar::setPIDTuning(double Kp, double Ki, double Kd)
{
  _config.Kp = Kp;
  _config.Ki = Ki;
  _config.Kd = Kd;
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    _mData[i].pid->SetTunings(Kp, Ki, Kd);
}
