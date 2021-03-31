// Control an autonomous MD_SmartCar vehicle through an Bluetooth interface
// using a HC-05 BT module that has been pre-initialized and paired to the 
// controller.
//
// All vehicle behaviour types can be exercised and monitored 
// from the AI2 'SmartCar_Rover_Control' interface application.
//
// NewPing library available from https://bitbucket.org/teckel12/arduino-new-ping/src/master/
//

#include <SoftwareSerial.h>
#include <MD_SmartCar.h>
#include <MD_cmdProcessor.h>
#include "SmartCar_HW.h"
#include "SmartCar_Sensors.h"

#ifndef ENABLE_DEBUG       // Serial debugging info
#define ENABLE_DEBUG 0 
#endif

#ifndef ENABLE_TELEMETRY   // BT monitoring telemetry
#define ENABLE_TELEMETRY 1
#endif

// Serial debugging
#if ENABLE_DEBUG
#define CMDStream Serial
#define DEBUG(s,v)   do { Serial.print(F(s)); Serial.print(v); } while (false)
#define DEBUGX(s,v)  do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false)
#define DEBUGS(s)    do { Serial.print(F(s)); } while (false)
#else
#define CMDStream BTSerial
#define DEBUG(s,v)
#define DEBUGX(s,v)
#define DEBUGS(s)
#endif

// BT monitoring telemetry
#if ENABLE_TELEMETRY
#if ENABLE_DEBUG
#define TELStream Serial
#else
#define TELStream BTSerial
#endif
#define TEL_VALUE(s,v) do { TELStream.print(F(s)); TELStream.print(v); } while (false)
#define TEL_MESG(s)    do { TELStream.print(F(s)); } while (false)
#define TEL_PACKET(s)  do { TELStream.print('$'); TELStream.print(s); TELStream.print('~'); } while (false)
#else
#define TEL_MESSAGE(s,v)
#define TEL_VALUE(s)
#endif

// ------------------------------------
// Global states
bool runEnabled = false;
bool seekLight = false;
static enum { ESCAPE, AVOID, SEEK, WALLFOLLOW, CRUISE } defaultBehaviour = CRUISE, runBehaviour = CRUISE;

// ------------------------------------
// Global Variables
#if CONTROLLER_L29x
SC_DCMotor_L29x ML(MC_INB1_PIN, MC_INB2_PIN, MC_ENB_PIN);  // Left motor
SC_DCMotor_L29x MR(MC_INA1_PIN, MC_INA2_PIN, MC_ENA_PIN);  // Right motor
#endif
#if CONTROLLER_MX1508
SC_DCMotor_MX1508 ML(MC_INB1_PIN, MC_INB2_PIN);  // Left motor
SC_DCMotor_MX1508 MR(MC_INA1_PIN, MC_INA2_PIN);  // Right motor
#endif

SC_MotorEncoder EL(EN_L_PIN);                         // Left motor encoder
SC_MotorEncoder ER(EN_R_PIN);                         // Right motor encoder
MD_SmartCar Car(&ML, &EL, &MR, &ER);                  // SmartCar object
SoftwareSerial BTSerial(PIN_BT_RX, PIN_BT_TX);

// ------------------------------------
// cmdProcessor Handler functions

void handlerM(char* param)
// Change running mode
{
  DEBUGS("Mode ");
  switch (*param)
  {
  case '1': runBehaviour = WALLFOLLOW;              DEBUGS("WALL\n");   break;
  case '2': runBehaviour = SEEK; seekLight = true;  DEBUGS("LIGHT\n");  break;
  case '3': runBehaviour = SEEK; seekLight = false; DEBUGS("DARK\n");   break;
  default:  runBehaviour = CRUISE;                  DEBUGS("CRUISE\n"); break;
  }

  defaultBehaviour = runBehaviour;    // set the default so we remember
}

void handlerR(char* param) 
// turn on or off
{ 
  runEnabled = (*param == '1'); 
  DEBUG("Run ", runEnabled);  DEBUGS("\n");
  if (runEnabled)
    TEL_MESG("\nRUN start");
  else
  {
    Car.stop();
    runBehaviour = defaultBehaviour;
    TEL_MESG("\nRUN end");
  }
}

void handlerH(char* param);   // prototype for cmdTable

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?", handlerH,   "",  "Help text", 1 },
  { "r",  handlerR,  "n", "Run 0=stop, 1=start", 1 },
  { "m",  handlerM,  "m", "Mode 0=Cruise, 1=Wall, 2=Light, 3=Dark", 1 },
};

MD_cmdProcessor CP(CMDStream, cmdTable, ARRAY_SIZE(cmdTable), true);

void handlerH(char* param) {  CP.help(); }

// ------------------------------------
// Telemetry functions

char *num2ASCII(char *p, uint16_t num, uint8_t size)
// format num with leading zeroes in size field starting at *p
{
  p += size;
  *p-- = '\0';

  for (uint8_t i = size; i > 0; i--)
  {
    *p-- = (num % 10) + '0';
    num = num / 10;
  }

  return(p + size + 1);
}

void sendTelemetry(void)
// Collate current info and send it as a telemetry packet.
// 
// Data Packet format is fixed length ASCII '0' to '9' stream with 
// a start indicator '$' and end indicator '~'.
// Data contents (in order and size)
// 00 R       run (1) or stopped (0)
// 01 D       Default Behaviour (0-3) for CRUISE, WALLFOLLOW, LIGHT, DARK
// 02 C       Current Behaviour (0-5) for CRUISE, WALLFOLLOW, LIGHT, DARK, AVOID, ESCAPE
// 03 B       Bumper L status (0/1)
// 04 B       Bumper R status (0/1)
// 05 LLL     Sonar L distance (cm)
// 08 MMM     Sonar M distance (cm)
// 11 RRR     Sonar R distance (cm)
// 14 XXX     Light L value
// 17 YYY     Light R value
//
{
#if ENABLE_TELEMETRY
  const uint32_t TELEMETRY_PERIOD = 500;    // maximum sending rate in ms
  static uint32_t timeLast = 0;

  if (millis() - timeLast >= TELEMETRY_PERIOD)
  {
    char mesg[25] = { '\0' };
    char* p = mesg;

    // create the packet
    *p++ = runEnabled ? '1' : '0';

    switch (defaultBehaviour)
    {
    case WALLFOLLOW: *p = '1'; break;
    case SEEK: *p = (seekLight ? '2' : '3'); break;
    default: *p = '0'; break;
    }
    p++;

    switch (runBehaviour)
    {
    case WALLFOLLOW: *p = '1'; break;
    case SEEK: *p = (seekLight ? '2' : '3'); break;
    case AVOID:  *p = '4'; break;
    case ESCAPE: *p = '5'; break;
    default: *p = '0'; break;
    }
    p++;

    *p++ = Sensors.bumperL ? '1' : '0';
    *p++ = Sensors.bumperR ? '1' : '0';
    p = num2ASCII(p, Sensors.sonarL, 3);
    p = num2ASCII(p, Sensors.sonarM, 3);
    p = num2ASCII(p, Sensors.sonarR, 3);
    p = num2ASCII(p, Sensors.lightL, 3);
    p = num2ASCII(p, Sensors.lightR, 3);

    *p = '\0';          // make sure it is terminated
    TEL_PACKET(mesg);   // send it off - macro will add top and tail to the packet

    timeLast = millis();  // set up for next time
  }
#endif
}

// ------------------------------------
// Implemented High Level Behaviours
// 
// Each behaviour has 2 related functions:
// 
// - activateBehaviour() to test whether the behaviour should become 
//   dominant. Behaviours take effect if they were previously dominant 
//   and not finished processing their action (ie, FSM is not complete) 
//   or conditions are detected that mean the behaviour should become
//   dominant.
// 
// - doBehaviour() to excute the FSM associated with the behaviour.
//   On startup the FSM should set its name as the current behaviour and
//   on completion reset the current behaviour to the value in 
//   'defaultBehaviour'.
//
// 

bool activateEscape(void)
// Check if the escaper conditions are satisfied
{
  bool b = (runBehaviour == ESCAPE);   // currently dominant

  b = b || Sensors.bumperL || Sensors.bumperR;
  b = b || Sensors.sonarM < DIST_IMPACT;

  return(b);
}

void doEscape(bool restart)
{
  const uint32_t PAUSE_TIME = 300;  // ms
  const float REVERSE_ANGLE = -PI;  // radians
  const int16_t SPIN_PCT = 30;      // fraction %

  static const PROGMEM MD_SmartCar::actionItem_t seqEscapeL[] =
  {
    { MD_SmartCar::STOP },
    { MD_SmartCar::PAUSE, PAUSE_TIME },
    { MD_SmartCar::MOVE,  REVERSE_ANGLE },
    { MD_SmartCar::PAUSE, PAUSE_TIME },
    { MD_SmartCar::SPIN,  -SPIN_PCT },
    { MD_SmartCar::END }
  };

  static const PROGMEM MD_SmartCar::actionItem_t seqEscapeR[] =
  {
    { MD_SmartCar::STOP },
    { MD_SmartCar::PAUSE, PAUSE_TIME },
    { MD_SmartCar::MOVE,  REVERSE_ANGLE, REVERSE_ANGLE },
    { MD_SmartCar::PAUSE, PAUSE_TIME },
    { MD_SmartCar::SPIN,  SPIN_PCT},
    { MD_SmartCar::END }
  };

  if (restart)
  {
    TEL_MESG("\nESCAPE start");
    runBehaviour = ESCAPE;
    if (Sensors.sonarL > Sensors.sonarR)
    {
      TEL_MESG(" L");
      Car.startSequence(seqEscapeL);
    }
    else
    {
      TEL_MESG(" R");
      Car.startSequence(seqEscapeR);
    }
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(" end");
    runBehaviour = defaultBehaviour;
  }
}

bool activateAvoid(void)
// Check if avoider conditions are satisfied
{
  bool b = (runBehaviour == AVOID);   // currently dominant

  b = b || Sensors.sonarM < DIST_OBSTACLE;

  return(b);
}

void doAvoid(bool restart)
// Avoids collision
// This veers in the direction of the most free space, 
// more veer with closer distance to obstacle.
{
  const uint32_t DRIVE_TIME = 500;  // ms
  const float DEADBAND = 0.05;   // radians

  static MD_SmartCar::actionItem_t seqAvoid[] =
  {
    { MD_SmartCar::DRIVE, SPEED_MAX, 0 },     // angular filled in at run time
    { MD_SmartCar::PAUSE, DRIVE_TIME },       // drive curved for a short time
    { MD_SmartCar::END }
  };

  if (restart)
  {
    float turn = 0.0;

    runBehaviour = AVOID;
    TEL_MESG("\nAVOIDER start");

    // how much to turn? work out angle inverse to distance from obstacle
    // and keep it max 90 degrees/sec rotation (PI/2 radians)
    // ie, less turn further out, more turn closer to impact
    turn = (1.0 - ((float)Sensors.sonarM/(float)DIST_OBSTACLE)) * (PI / 2.0);
    
    // now only turn if the difference is worthwhile
    if (abs(turn - abs(seqAvoid[0].parm[1])) > DEADBAND)
    {
      // which way to turn? Default is R (+) but change
      // to L (-) if there is more space that side
      if (Sensors.sonarL > Sensors.sonarR) turn = -turn;

      if (turn < 0) TEL_MESG(" L:"); else TEL_MESG(" R:"); 
      TEL_VALUE("", turn);

      // modify the Avoid sequence with new value
      seqAvoid[0].parm[1] = turn;
    }

    // now run the Avoid sequence
    Car.startSequence(seqAvoid);
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(" end");
    runBehaviour = defaultBehaviour;
  }
}

bool activateSeek(void)
{
  bool b = (runBehaviour == SEEK);   // currently dominant

  if (b)    // default behaviour is to SEEK
  {
    // work out something here

  }

  return(b);
}

void doSeek(bool restart, bool toLight)
// Seeks light (true) or dark (false)
{
  const uint32_t SETTLE_TIME = 500;  // ms

  static MD_SmartCar::actionItem_t seqSeek[] =
  {
    { MD_SmartCar::DRIVE, SPEED_MAX, 0 },     // angular filled in at run time
    { MD_SmartCar::PAUSE, SETTLE_TIME },       // drive curved for a short time
    { MD_SmartCar::END }
  };

  if (restart)
  {
    TEL_MESG("\nSEEK start");
    Car.startSequence(seqSeek);
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(" end");
    runBehaviour = defaultBehaviour;
  }
}

bool activateWallFollow(void)
{
  bool b = (defaultBehaviour == WALLFOLLOW);   // currently dominant

  if (b)    // default behaviour is to WALLFOLLOW
  {
    // work out something here
  }

  return(b);
}

void doWallFollow(bool restart)
// Follows wall at set distance
{
  const uint32_t SETTLE_TIME = 500;  // ms

  static MD_SmartCar::actionItem_t seqFollow[] =
  {
    { MD_SmartCar::DRIVE, SPEED_MAX, 0 },     // angular filled in at run time
    { MD_SmartCar::PAUSE, SETTLE_TIME },      // drive curved for a short time
    { MD_SmartCar::END }
  };

  if (restart)
  {
    TEL_MESG("\nFOLLOWER start");
    Car.startSequence(seqFollow);
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(" end");
    runBehaviour = defaultBehaviour;
  }
}

void doCruise(void)
// Default is to just drive in a straight line
// We only get here when all other behaviours are not applicable!
{
  Car.drive(SPEED_MAX);
}

void setup(void)
{
#if SCDEBUG || ENABLE_DEBUG
  Serial.begin(57600);
#endif
  BTSerial.begin(BT_BAUDRATE);
  Sensors.begin();
  if (!Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE))   // take all the defaults
    TEL_MESG("\nUnable to start car!!\n");

  CP.begin();
}

void loop(void)
{
  // Always run these background tasks
  CP.run();         // Command processor
  Car.run();        // Car functions
  Sensors.read();   // read sensors
  sendTelemetry();  // send any telemetry if enabled

  if (!runEnabled)    // global running flag is off, skip the rest
    return;

  // Arbitrate the behaviours in priority order
  if      (activateEscape())     doEscape(runBehaviour != ESCAPE);
  else if (activateAvoid())      doAvoid(runBehaviour != AVOID);
  else if (activateSeek())       doSeek(runBehaviour != SEEK, seekLight);
  else if (activateWallFollow()) doWallFollow(runBehaviour != WALLFOLLOW);
  else                           doCruise();    // default choice
}
