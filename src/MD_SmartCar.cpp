#include <MD_SmartCar.h>

/**
 * \file
 * \brief Code file for SmartCar library class.
 */

MD_SmartCar::MD_SmartCar(SC_DCMotor *ml, SC_MotorEncoder *el, SC_DCMotor *mr, SC_MotorEncoder *er)
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

  // do PID initialization
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].pid = new SC_PID(&_mData[i].cv, &_mData[i].co, &_mData[i].sp, _config.Kp, _config.Ki, _config.Kd);
    _mData[i].pid->setPIDPeriod(PID_PERIOD);
    _mData[i].state = S_IDLE;
  }
  setPIDOutputLimits();

  // Initialize the motors and encoders
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _M[i]->begin();
    b &= _E[i]->begin();
  }

  // Set up default environment
  calcVMax();
  drive((int8_t)0, (float)0.0);    // initialize to all stop

  return(b);
}

void MD_SmartCar::run(void)
// run the FSM to manage motor functions
{
  bool firstPass = true;
  uint32_t now = millis();    // keep time in sync for all motors
  
  // loop through all the motors doing whatever in each state
  for (uint8_t motor = 0; motor < MAX_MOTOR; motor++)
  {
    switch (_mData[motor].state)
    {
    case S_IDLE: break;     // do nothing

    // --- FREE RUNNING
    case S_DRIVE_INIT:
      PRINT("\n>>DRIVE_INIT #", motor);
      _E[motor]->reset();
      _mData[motor].state = S_DRIVE;
      _mData[motor].pid->setMode(SC_PID::USER);
      _E[motor]->reset();   // reset the counters
      _M[motor]->run(_mData[motor].direction, MC_PWM_CREEP); // start higher to overcome startup friction
      break;

    case S_DRIVE:
      if (now - _mData[motor].lastPIDRun >= _mData[motor].pid->getPIDPeriod())
      {
        uint32_t time;      // time for encoder accumulator
        uint16_t cv;        // current encoder value

        // run the PID loop to keep things on even keel
        _E[motor]->read(time, cv, true);   // read and reset the encoder counter
        _mData[motor].cv = cv;             // save the current value for PID
        _mData[motor].pid->compute();      // run PID next step
        _M[motor]->run(_mData[motor].direction, _mData[motor].co); // set motor speed
        _mData[motor].lastPIDRun = now;

        // debug print to see what happening
        if (firstPass)   // only print the header info once each loop iteration
        {
          firstPass = false;
          PRINT("\nPID", time);
        }
        else
          PRINTS(",");
        PRINT(" [", motor);
        PRINT("] SP:", _mData[motor].sp);
        PRINT(" CV:", _mData[motor].cv);
        PRINT(" CO:", _mData[motor].co);
      }
      break;

    // --- Precision moves
    case S_MOVE_INIT:
      PRINT("\n>>MOVE_INIT #", motor);
      _E[motor]->reset();
      _M[motor]->run(_mData[motor].direction, _mData[motor].sp);
      _mData[motor].state = S_MOVE;
      // deliberately fall through to S_MOVE

    case S_MOVE:
      {
        uint32_t time;
        uint16_t count;

        _E[motor]->read(time, count, false);
        if (firstPass)
        {
          firstPass = false;
          PRINTS("\nMOVE");
        }
        else
          PRINTS(",");
        PRINT(" [", motor); 
        PRINT("] ", count);
        PRINT("/", _mData[motor].cv);
        if ((int16_t)count >= _mData[motor].cv)    // done all the pulses required
        {
          _M[motor]->run(SC_DCMotor::DIR_STOP, 0);
          _mData[motor].state = S_IDLE;
        }
      }
      break;

    default: _mData[motor].state = S_IDLE; break;
    }
  }
}

void MD_SmartCar::drive(int8_t vLinear, float vAngularR)
{
  float spL, spR;

  if (vLinear < -100 || vLinear > 100 || vAngularR < -PI || vAngularR > PI)
    return;

  if (vLinear == 0)
    stop();
  else
  {
    // decompose and save the current settings global settings
    _mData[MLEFT].direction = (vLinear < 0 ? SC_DCMotor::DIR_REV : SC_DCMotor::DIR_FWD);
    _mData[MRIGHT].direction = _mData[MLEFT].direction;
    _vLinear = abs(vLinear);
    _vAngular = vAngularR;

    // Unicycle control kinematics differential wheel velocity
    // vL = (2v - wL)/(2R); vR = (2v - wL)/(2R)
    // where 
    // vL, vR are left and right velocity of wheel in radians/sec
    // v = linear velocity of vehicle (_vLinear)
    // w = angular velocity of vehicle (_vAngular)
    // L = vehicle wheel Base (WHEEL_BASE)
    // R = radius of vehicle wheel (WHEEL_DIAM/2)
    // All length measurments in the same units cancel out
    //
    spL = spR = (_vLinear * _vMaxLinear) / 100.0; // convert velocities from percentage to mm/sec
    PRINT("\nSPLR: ", spL);

    spL = ((2.0 * spL) - (_vAngular * WHEEL_BASE)) / WHEEL_DIAM; // Diameter = 2*radius
    spR = ((2.0 * spR) + (_vAngular * WHEEL_BASE)) / WHEEL_DIAM;

    PRINT(" -> rad/s SPL:", spL);
    PRINT(" SPR:", spR);

    // Convert the radians/s velocity into encoder pulses per PID period
    // PulsePerSec = ((rad/s) * ((pulse/rev)/(rad/rev)))
    // PulsePerPeriod = PulsePerSecond/PID_PERIOD
    spL = (spL * _E[MLEFT]->getPulsePerRev()) / (2.0 * PI * PID_FREQ);
    spR = (spR * _E[MRIGHT]->getPulsePerRev()) / (2.0 * PI * PID_FREQ);
    PRINT(" -> PID SPL:", spL);
    PRINT(" SPR:", spR);

    // put values into the motor setpoint parameters for running the FSM
    _mData[MLEFT].sp = trunc(spL);
    _mData[MRIGHT].sp = trunc(spR);
    if (!isRunning())
      _mData[MLEFT].state = _mData[MRIGHT].state = S_DRIVE_INIT;
  }
}

void MD_SmartCar::move(int8_t vL, float angL, int8_t vR, float angR)
{
  uint32_t temp;    // large integer for results

  // make sure inputs make sense
  if (vL < -100 || vL > 100 || vR < -100 || vR > 100)
    return;
  if (vL == 0) angL = 0.0;
  if (vR == 0) angR = 0.0;

  // set the motor direction
  _mData[MLEFT].direction = (vL < 0 ? SC_DCMotor::DIR_REV : SC_DCMotor::DIR_FWD);
  _mData[MRIGHT].direction = (vR < 0 ? SC_DCMotor::DIR_REV : SC_DCMotor::DIR_FWD);

  // set the motor PWM setpoint
  temp = getMinMotorSP() + (((getMaxMotorSP() - getMinMotorSP()) * abs(vL)) / 100);
  _mData[MLEFT].sp = temp;
  temp = getMinMotorSP() + (((getMaxMotorSP() - getMinMotorSP()) * abs(vR)) / 100);
  _mData[MRIGHT].sp = temp;
  PRINT("\nMove PWM L ", _mData[MLEFT].sp);
  PRINT(" R ", _mData[MRIGHT].sp);

  // convert subtended angle into number of encoder pulses
  _mData[MLEFT].cv = (angL * _E[MLEFT]->getPulsePerRev()) / (2.0 * PI);
  _mData[MRIGHT].cv = (angR * _E[MRIGHT]->getPulsePerRev()) / (2.0 * PI);
  PRINT("; Pulses L ", _mData[MLEFT].cv);
  PRINT(" R ", _mData[MRIGHT].cv);

  // finally, set it up for the FSM to execute
  _mData[MLEFT].state = _mData[MRIGHT].state = S_MOVE_INIT;
}

void MD_SmartCar::stop(void)
// totally halt the vehicle
{
  _vLinear = 0;

  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].direction = SC_DCMotor::DIR_STOP;
    _mData[i].sp = 0;
    _mData[i].state = S_IDLE;
    _M[i]->run(_mData[i].direction, 0);
  }
}

void MD_SmartCar::setVelocity(int8_t vel)
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

void MD_SmartCar::calcVMax(void)
// calculate max speed based on vehicle physical constants
{
  _vMaxLinear = DIST_PER_REV * (MAX_PULSE_PER_SEC / _E[MLEFT]->getPulsePerRev());
  PRINT("\n** mm/rev ", DIST_PER_REV);
  PRINT(" -> Vmax ", _vMaxLinear);
  PRINTS(" mm/s");
}

float MD_SmartCar::deg2rad(int16_t deg)
// convert degrees to radians
{ 
  return((2 * PI * (float)deg) / 360.0); 
}

void MD_SmartCar::setPIDOutputLimits(void)
{
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    _mData[i].pid->setOutputLimits(getMinMotorSP(), getMaxMotorSP());
}

void MD_SmartCar::setPIDTuning(float Kp, float Ki, float Kd)
{
  _config.Kp = Kp;
  _config.Ki = Ki;
  _config.Kd = Kd;
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
    _mData[i].pid->setTuning(Kp, Ki, Kd);
}

bool MD_SmartCar::setCreepSP(uint8_t units)
{
  if (units >= _config.minPWM && units <= _config.maxPWM)
    _config.creepPWM = units;

  return(units = _config.creepPWM);
}
