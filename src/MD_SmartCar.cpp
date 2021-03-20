#include <MD_SmartCar.h>

/**
 * \file
 * \brief Code file for SmartCar library class.
 */

/**
\page pageControlModel Unicycle Control Model

Working out the displacement and velocities of each wheel on a
differential drive robot can be messy.

The _unicycle model_ of an autonomous robot is a kinematic
model that allows modelling the movement of the vehicle
as if it was a unicycle, using a linear velocity vector (V)
and a rotational velocity (&omega;) about a point within the
vehicle. Taking this point to be midway between axis joining 
the 2 wheels simplifies the calculation.

![SmartCar Movement Transform](SmartCar_Transform.png "SmartCar Movement Transform")

We can derive equations that translate between the unicycle model
and our wheel velocities. Steering requires each of the independent
wheels to rotated at different speeds (VL and VR for the
left and right side) to travel the equivalent unicycle path.

So specifying a movement path using this abstraction becomes much easier
as we need to just specify "How fast do we want to move forward and
how fast do we want to turn", letting the mathematics work out the
wheel rotations.

![SmartCar Unicycle](SmartCar_Unicycle.png "SmartCar Unicycle Model")

V and &omega; are transformed into independent motor speeds for the left
and right motor(VL, VR) using the following formulas:
- VL = (2V + &omega; B) / 2r
- VR = (2V - &omega; B) / 2r

where B is the vehicle base length (ie, the distance between the wheels)
and r is the radius of the wheel.

The convention used in this library is:
- Linear velocity V is postive for forward motion, negative backwards.
- Angular velocity &omega; is positive for right rotation, negative for left.

![SmartCar Convention](SmartCar_Convention.png "SmartCar Library Convention")
____
More at
http:://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html
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

bool MD_SmartCar::begin(uint16_t ppr, uint16_t ppsMax, uint16_t dWheel, uint16_t lBase)
// Return false if any of the encoders fail to begin
{ 
  bool b = true;

  loadConfig();

  // do PID initialization
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].pid = new SC_PID(&_mData[i].cv, &_mData[i].co, &_mData[i].sp, _config.Kp[i], _config.Ki[i], _config.Kd[i]);
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
  setVehicleParameters(ppr, ppsMax, dWheel, lBase);
  stop();    // initialize to all stop

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
      SCPRINT("\n>>DRIVE_INIT #", motor);
      _M[motor]->run(_mData[motor].direction, getKickerSP()); // start at kicker PWM
      _mData[motor].timeLast = now; // use this temporarily
      _mData[motor].state = S_DRIVE_KICKER;
      break;

    case S_DRIVE_KICKER:
      if (now - _mData[motor].timeLast >= MC_KICKER_ACTIVE)
      {
        _mData[motor].timeLast = now - _mData[motor].pid->getPIDPeriod();
        _mData[motor].state = S_DRIVE_PIDRST;
      }
      break;

    case S_DRIVE_PIDRST:
      SCPRINT("\n>>DRIVE_PIDRST #", motor);
      _mData[motor].pid->setMode(SC_PID::USER);
      _mData[motor].pid->reset();
      _E[motor]->reset();   // reset the counters
      _mData[motor].timeLast = now;
      _mData[motor].state = S_DRIVE_RUN;
      break;
      
    case S_DRIVE_RUN:
      if (now - _mData[motor].timeLast >= _mData[motor].pid->getPIDPeriod())
      {
        uint32_t time;      // time for encoder accumulator
        uint16_t cv;        // current encoder value

        // Print Tuning parameters if this is the first pass.
        // Only enabled with PID_TUNE set to 1.
        // This actually prints the results of the last pass but should be good 
        // enough to see what is happening during tuning.
        if (firstPass)
        {
          P_PID_HDR;
          for (uint8_t i = 0; i < MAX_MOTOR; i++)
            P_PID_BODY(_mData[i].sp, _mData[i].cv, _mData[i].co, (i == MAX_MOTOR-1));
          P_PID_TAIL;
        }

        // run the PID loop to keep things on even keel
        _E[motor]->read(time, cv, true);   // read and reset the encoder counter
        _mData[motor].cv = cv;             // save the current value for PID
        _mData[motor].pid->compute();      // run PID next step
        _M[motor]->run(_mData[motor].direction, _mData[motor].co); // set motor speed
        _mData[motor].timeLast = now;    // set the processed time marker identical for all motors

        // debug print to see what happening
        if (firstPass)   // only print the header info once each loop iteration
        {
          firstPass = false;
          SCPRINT("\nPID ", time);
          SCPRINTS("ms ");
        }
        else
          SCPRINTS(",");
        SCPRINT(" [", motor);
        SCPRINT("] SP:", _mData[motor].sp);
        SCPRINT(" CV:", _mData[motor].cv);
        SCPRINT(" CO:", _mData[motor].co);
      }
      break;

    // --- Precision moves
    case S_MOVE_INIT:
      SCPRINT("\n>>MOVE_INIT #", motor);
      _E[motor]->reset();
      _M[motor]->run(_mData[motor].direction, _mData[motor].sp);
      _mData[motor].state = S_MOVE_RUN;
      // deliberately fall through to S_MOVE

    case S_MOVE_RUN:
      {
        uint32_t time;
        uint16_t count;

        _E[motor]->read(time, count, false);
        if (firstPass)
        {
          firstPass = false;
          SCPRINTS("\nMOVE");
        }
        else
          SCPRINTS(",");
        SCPRINT(" [", motor); 
        SCPRINT("] ", count);
        SCPRINT("/", _mData[motor].cv);
        if ((int16_t)count >= _mData[motor].cv)    // done all the pulses required
        {
          _M[motor]->setSpeed(0);
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

  if (vLinear == 0)
    stop();
  else if ((abs(vLinear) == _vLinear) && (vAngularR == _vAngular))
    return;    // no change
  else
  {
    SCPRINT("\n** DRIVE v:", vLinear);
    SCPRINT(" a:", vAngularR);

    // sanitize input
    if (vLinear < -100) vLinear = -100;
    if (vLinear > 100) vLinear = 100;
    if (vAngularR < -PI/2) vAngularR = -PI/2;
    if (vAngularR > PI/2)  vAngularR = PI/2;

    // decompose and save as current global settings
    _mData[MLEFT].direction = (vLinear < 0 ? SC_DCMotor::DIR_REV : SC_DCMotor::DIR_FWD);
    _mData[MRIGHT].direction = _mData[MLEFT].direction;
    _vLinear = abs(vLinear);
    _vAngular = vAngularR;

    // Unicycle control kinematics differential wheel velocity
    // vL = (2v + wL)/(D); vR = (2v - wL)/(D)
    // where 
    // vL, vR are left and right velocity of wheel in encoder pulse/sec
    // v = linear velocity of vehicle (_vLinear)
    // w = angular velocity of vehicle (_vAngular)
    // L = vehicle wheel Base (_lenBase converted to _lenBaseP)
    // D = diameter of vehicle wheel (_diaWheel converted to _diaWheelP)
    // All length measurements in the same units cancel out
    //
    // http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html
    // for the modified equation not including the diameter, used below.
    //
    spL = spR = ((float)_ppsMax * _vLinear) / 100.0; // convert velocity from % to pps
    SCPRINT("\nSPLR: ", spL);

    spL = spL + ((_vAngular * _lenBaseP) / 2);
    spR = spR - ((_vAngular * _lenBaseP) / 2);

    SCPRINT(" -> pps L:", spL);
    SCPRINT(" R:", spR);

    // Convert the pps velocity into encoder pulses per PID period
    // PulsePerPIDPeriod = PulsePerSecond/PID_FREQUENCY
    spL /= PID_FREQ;
    spR /= PID_FREQ;
    SCPRINT(" -> PID SPL:", spL);
    SCPRINT(" SPR:", spR);

    // put values into the motor setpoint parameters (integers) for running the FSM
    _mData[MLEFT].sp = trunc(spL + 0.5);
    _mData[MRIGHT].sp = trunc(spR + 0.5);
    _mData[MLEFT].state = _mData[MRIGHT].state = (isRunning() ? S_DRIVE_PIDRST : S_DRIVE_INIT);
  }
}

void MD_SmartCar::move(float angL, float angR)
{
  SCPRINT("\n** MOVE L:", angL);
  SCPRINT(" R:", angR);

  // set the motor direction
  _mData[MLEFT].direction = (angL < 0.0 ? SC_DCMotor::DIR_REV : SC_DCMotor::DIR_FWD);
  _mData[MRIGHT].direction = (angR < 0.0 ? SC_DCMotor::DIR_REV : SC_DCMotor::DIR_FWD);
  if (angL < 0.0) angL = -angL;   // absolute value
  if (angR < 0.0) angR = -angR;   // absolute value

  // set the motor PWM setpoint
  _mData[MLEFT].sp = _mData[MRIGHT].sp = getMoveSP();
  SCPRINT("\nMove PWM L ", _mData[MLEFT].sp);
  SCPRINT(" R ", _mData[MRIGHT].sp);

  // convert subtended angle into number of encoder pulses
  _mData[MLEFT].cv = trunc((angL * _ppr) / (2.0 * PI));
  _mData[MRIGHT].cv = trunc((angR * _ppr) / (2.0 * PI));
  SCPRINT("; Pulses L ", _mData[MLEFT].cv);
  SCPRINT(" R ", _mData[MRIGHT].cv);

  // finally, set it up for the FSM to execute
  _mData[MLEFT].state = _mData[MRIGHT].state = S_MOVE_INIT;
}

void MD_SmartCar::spin(int16_t fraction)
{
  SCPRINT("\n** SPIN f:", fraction);

  // Work out the wheel directions
  _mData[MLEFT].direction = (fraction > 0.0 ? SC_DCMotor::DIR_FWD : SC_DCMotor::DIR_REV);
  _mData[MRIGHT].direction = (fraction > 0.0 ? SC_DCMotor::DIR_REV : SC_DCMotor::DIR_FWD);
  if (fraction < 0.0) fraction = -fraction; // absolute value

  // set the motor PWM setpoint
  _mData[MLEFT].sp = _mData[MRIGHT].sp = getMoveSP();

  // Convert fraction into number of encoder pulses. Both wheels will 
  // turn the same number of pulses.
  // Circle distance in pulses = PI * base length (diameter) in pulses
  _mData[MLEFT].cv = _mData[MRIGHT].cv = ((PI * _lenBaseP * fraction) / 100.0) * _config.spinAdjust;
  SCPRINT(" pulses ", _mData[MLEFT].cv);

  // finally, set it up for the FSM to execute
  _mData[MLEFT].state = _mData[MRIGHT].state = S_MOVE_INIT;
}

void MD_SmartCar::stop(void)
// totally halt the vehicle
{
  _vLinear = 0;
  _vAngular = 0.0;

  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    _mData[i].direction = SC_DCMotor::DIR_FWD;
    _mData[i].sp = 0;
    _mData[i].state = S_IDLE;
    _M[i]->run(_mData[i].direction, _mData[i].sp);
  }
}
