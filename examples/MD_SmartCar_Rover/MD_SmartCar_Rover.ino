// Application to control an autonomous MD_SmartCar vehicle using a 
// Bluetooth interface implemented with a HC-05 module that has been 
// pre-initialized and paired to the controller.
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

#ifndef ENABLE_DEBUG       
#define ENABLE_DEBUG 0     // Serial debugging info
#endif

#ifndef REMOTE_START
#define REMOTE_START 1     // set to 1 to start from BT interface
#endif

#ifndef DUMP_SENSORS
#define DUMP_SENSORS 0     // set to 1 to echo sensor data
#endif

#ifndef ENABLE_TELEMETRY   
#define ENABLE_TELEMETRY 1 // set to 1 for BT telemetry monitoring
#endif

// Serial debugging macros
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
#else // not debug
#define TELStream BTSerial
#endif
#define TEL_VALUE(s,v) do { TELStream.print(F(s)); TELStream.print(v); } while (false)
#define TEL_MESG(s)    do { TELStream.print(F(s)); } while (false)
#if ENABLE_DEBUG
#define TEL_PACKET(s)
#else // not debug
#define TEL_PACKET(s)  do { TELStream.print('$'); TELStream.print(s); TELStream.print('~'); } while (false)
#endif
#else // not telemetry
#define TEL_VALUE(s,v)
#define TEL_MESG(s)
#define TEL_PACKET(s)
#endif

// ------------------------------------
// Global states
bool runEnabled = (REMOTE_START == 0);
bool seekLight = false;
static enum 
{ 
  ESCAPE,     // emergency bumping into things or way too close. Always gets priority.
  AVOID,      // CUISE seleccted, moves to best open space.
  SEEK,       // SEEK to light or dark.
  WALLFOLLOW, // follow a wall.
  CRUISE      // default operaing when nothing else is running - move straight.
} defaultBehaviour = CRUISE, runBehaviour = CRUISE;

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
    TEL_MESG("\n>> RUN <<");
  else
  {
    Car.stop();
    runBehaviour = defaultBehaviour;
    TEL_MESG("\n>> STOP <<");
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

inline uint8_t bool2ASCII(char* p, bool state, uint8_t size)
// Place a bool starting at *p, one character long.
// Return the number of inserted characters.
{
  for (uint8_t i = 0; i < size; i++)
    *p++ = (state ? '1' : '0');

  return(size);
}

uint8_t num2ASCII(char *p, int16_t num, uint8_t size)
// Place a num with leading zeroes in size field starting at *p.
// Return the number of inserted characters.
{
  bool negative = (num < 0);

  num = abs(num);
  for (int16_t i = size - 1; i >= 0; i--)
  {
    p[i] = (num % 10) + '0';
    num = num / 10;
  }

  if (negative) *p = '-';

  return(size);
}

uint8_t float2ASCII(char* p, float num, uint8_t size, uint8_t dec)
{
  uint16_t frac = abs(num - trunc(num)) * pow(10, dec);

  num2ASCII(&p[size - dec], frac, dec);
  p[size - dec - 1] = '.';
  num2ASCII(p, (int16_t)trunc(num), size - dec - 1);

  return(size);
}

void sendTelemetryData(void)
// Collate current info and send it as a telemetry packet.
// 
// Data Packet format is fixed length ASCII '-','0'..'9' stream with 
// a start indicator '$' and end indicator '~'.
// 
// Data contents (with start position and field width)
// 00 R       run (1) or stopped (0)
// 01 D       Default Behaviour (0-3) for CRUISE, WALLFOLLOW, LIGHT, DARK
// 02 C       Current Behaviour (0-5) for CRUISE, WALLFOLLOW, LIGHT, DARK, AVOID, ESCAPE
// 03 VVVV    Linear velocity % FS (signed)
// 07 AAAAA   Angular velocity (float signed)
// 12 B       Bumper L status (0/1)
// 13 B       Bumper R status (0/1)
// 14 LLL     Sonar L distance (cm)
// 15 MMM     Sonar M distance (cm)
// 20 RRR     Sonar R distance (cm)
// 23 XXX     Light L value
// 26 YYY     Light R value
// 29         end of packet
//
{
  static uint32_t timeLast = 0;

  if (millis() - timeLast >= TELEMETRY_PERIOD)
  {
    static char mesg[35] = { "RDCVVVVAAAAABBLLLMMMRRRXXXYYY" };
    char *p = mesg;

    // clear the packet (string)
    //memset(mesg, 0, ARRAY_SIZE(mesg));

    // Running modes
    p += bool2ASCII(p, runEnabled, 1);

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

    // Speed & Direction
    p += num2ASCII(p, Car.getLinearVelocity(), 4);
    p += float2ASCII(p, Car.getAngularVelocity(), 5, FLOAT_DECIMALS);

    // Sensor Data
    p += bool2ASCII(p, Sensors.bumperL, 1);
    p += bool2ASCII(p, Sensors.bumperR, 1);
    p += num2ASCII(p, Sensors.sonarL, 3);
    p += num2ASCII(p, Sensors.sonarM, 3);
    p += num2ASCII(p, Sensors.sonarR, 3);
    p += num2ASCII(p, Sensors.lightL, 3);
    p += num2ASCII(p, Sensors.lightR, 3);

    *p = '\0';          // make sure it is terminated
    TEL_PACKET(mesg);   // send it off - macro will add top and tail to the packet

    timeLast = millis();  // set up for next time
  }
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
// Check if the ESCAPE conditions are satisfied.
// Escape will always take priority over any other behaviour selected.
{
  bool b = false;
  
  b = b || (runBehaviour == ESCAPE);              // currently dominant
  b = b || Sensors.bumperL || Sensors.bumperR;    // bumpers triggered
  b = b || Sensors.sonarM < DIST_IMPACT;          // too close on sonar

  return(b);
}

void doEscape(bool restart)
// Escapes danger
// Backs away, turns to  where there is most space, resumes default behaviour.
{
  const float REVERSE_ANGLE = PI/2; // radians
  const int16_t SPIN_PCT = 30;      // fraction %

  static const PROGMEM MD_SmartCar::actionItem_t seqEscapeL[] =
  {
    { MD_SmartCar::STOP },
    { MD_SmartCar::PAUSE, ESCAPE_PAUSE_TIME },
    { MD_SmartCar::MOVE,  -REVERSE_ANGLE, -REVERSE_ANGLE },
    { MD_SmartCar::PAUSE, ESCAPE_PAUSE_TIME },
    { MD_SmartCar::SPIN,  -SPIN_PCT },
    { MD_SmartCar::END }
  };

  static const PROGMEM MD_SmartCar::actionItem_t seqEscapeR[] =
  {
    { MD_SmartCar::STOP },
    { MD_SmartCar::PAUSE, ESCAPE_PAUSE_TIME },
    { MD_SmartCar::MOVE,  -REVERSE_ANGLE, -REVERSE_ANGLE },
    { MD_SmartCar::PAUSE, ESCAPE_PAUSE_TIME },
    { MD_SmartCar::SPIN,  SPIN_PCT},
    { MD_SmartCar::END }
  };

  if (restart)
  {
    TEL_MESG("\nESCAPE start");
    if (Sensors.bumperL) TEL_MESG(" BL");
    if (Sensors.bumperR) TEL_MESG(" BR");
    if (Sensors.sonarM < DIST_IMPACT) TEL_MESG(" S");

    runBehaviour = ESCAPE;
    if (Sensors.sonarL > Sensors.sonarR)
    {
      TEL_MESG(": L");
      Car.startSequence(seqEscapeL);
    }
    else
    {
      TEL_MESG(": R");
      Car.startSequence(seqEscapeR);
    }
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(": end");
    runBehaviour = CRUISE;
  }
}

bool activateAvoid(void)
// Check if AVOID conditions are satisfied.
{
  bool b = false;

  b = b || (runBehaviour == AVOID);         // currently dominant
  b = b || Sensors.sonarM < DIST_OBSTACLE;  // within range of obstruction
  // b = b && (defaultBehaviour == CRUISE);    // but only if this is the selected overall behaviour

  return(b);
}

void doAvoid(bool restart)
// Avoids collision when in cruise mode.
// This veers in the direction of the most free space, 
// more veer with closer distance to obstacle.
{
  const float DEADBAND = 0.05;   // radians

  static MD_SmartCar::actionItem_t seqAvoid[] =
  {
    { MD_SmartCar::DRIVE, SPEED_CRUISE, 0 },       // angular speed filled in at run time
    { MD_SmartCar::PAUSE, AVOID_ACTIVE_TIME },  // drive curved for a short time
    { MD_SmartCar::END }
  };

  if (restart)
  {
    float turn = 0.0;

    // how much to turn? work out angle inverse to distance from obstacle
    // and keep it max 90 degrees/sec rotation (PI/2 radians)
    // ie, less turn further out, more turn closer to impact
    turn = (1.0 - ((float)Sensors.sonarM/(float)DIST_OBSTACLE)) * (PI / 2.0);
    
    // now only AVOID if the difference is worthwhile
    if (abs(turn - abs(seqAvoid[0].parm[1])) > DEADBAND)
    {
      runBehaviour = AVOID;
      TEL_MESG("\nAVOIDER start");

      // which way to turn? Default is R (+) but change
      // to L (-) if there is more space that side
      if (Sensors.sonarL > Sensors.sonarR) turn = -turn;

      if (turn < 0) TEL_MESG(": L"); else TEL_MESG(": R");
      TEL_VALUE(" ", turn);

      // modify the Avoid sequence with new value
      seqAvoid[0].parm[1] = turn;

      // now run the new Avoid sequence
      Car.startSequence(seqAvoid);
    }
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(": end");
    runBehaviour = CRUISE;
  }
}

bool activateSeek(void)
// Check if the SEEK conditions are satisfied.
{
  bool b = false;

  b = b || (runBehaviour == SEEK);                    // currently dominant
  b = b || abs(Sensors.lightL - Sensors.lightR) > 5;  // difference in light on 2 sides
  b = b && (defaultBehaviour == SEEK);                // but only if this is the selected behaviour

  return(b);
}

void doSeek(bool restart, bool toLight)
// Seeks light (true) or dark (false)
// Veers in the direction with the most/least light detected
{
  static MD_SmartCar::actionItem_t seqSeek[] =
  {
    { MD_SmartCar::DRIVE, SPEED_CRUISE, 0 },     // angular filled in at run time
    { MD_SmartCar::PAUSE, SEEK_ACTIVE_TIME }, // drive curved for a short time
    { MD_SmartCar::END }
  };

  if (restart)
  {
    runBehaviour = SEEK;
    TEL_MESG("\nSEEK start");

    float turn = 0.0;

    // how much to turn? work out angle proportional to difference in light
    // between the two sides and keep it max 90 degrees/sec rotation (PI/2 radians)
    // ie, higher difference = more turn.
    // Assume moving to light and reverse later if not the case.
    turn = ((float)abs(Sensors.lightL - Sensors.lightR) / 255.0) * (PI / 2.0);

    if (Sensors.lightL > Sensors.lightR) turn = -turn;    // toLight left turn is negative angle
    if (!toLight) turn = -turn;                           // not toLight just does the opposite

    if (turn < 0) TEL_MESG(": L"); else TEL_MESG(": R");
    TEL_VALUE(" ", turn);

    // modify the sequence with new value and run it
    seqSeek[0].parm[1] = turn;
    Car.startSequence(seqSeek);
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(": end"); 
    runBehaviour = CRUISE;
  }
}

bool activateWallFollow(void)
// Check if the WALLFOLLOW conditions are satisfied.
{
  bool b = false;

  b = b || (runBehaviour == WALLFOLLOW);      // currently dominant
  b = b && (defaultBehaviour == WALLFOLLOW);  // but only if this is the selected behaviour 

  return(b);
}

void doWallFollow(bool restart)
// Follows wall at set distance
{
  static MD_SmartCar::actionItem_t seqFollow[] =
  {
    { MD_SmartCar::DRIVE, SPEED_CRUISE, 0 },       // angular filled in at run time
    { MD_SmartCar::PAUSE, FOLLOW_ACTIVE_TIME }, // drive curved for a short time
    { MD_SmartCar::END }
  };

  if (restart)
  {
    runBehaviour = WALLFOLLOW;

    TEL_MESG("\nFOLLOWER start");
    Car.startSequence(seqFollow);
  }
  else if (Car.isSequenceComplete())
  {
    TEL_MESG(": end");
    runBehaviour = CRUISE;
  }
}

void doCruise(void)
// Default is to just drive in a straight line
// We only get here when all other behaviours are not applicable!
{
  Car.drive(Sensors.sonarM == DIST_ALLCLEAR ? SPEED_MAX : SPEED_CRUISE);
}

void setup(void)
{
#if SCDEBUG || ENABLE_DEBUG || DUMP_SENSORS
  Serial.begin(57600);
#endif
  BTSerial.begin(BT_BAUDRATE);

  CP.begin();
  Sensors.begin();
  if (!Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE))   // take all the defaults
    TEL_MESG("\nUnable to start car!!\n");
}

void loop(void)
{
  // Always run these background tasks
  CP.run();         // Command processor
  Car.run();        // Car functions
  Sensors.read();   // read sensors
  if (Sensors.isUpdated())
  {
#if DUMP_SENSORS
    Sensors.dump(Serial);
#endif
#if ENABLE_TELEMETRY
    sendTelemetryData();  // send any telemetry if enabled
#endif
  }

  if (!runEnabled)    // global running flag is off, skip the rest
    return;

  // Arbitrate the behaviours in priority order
  if      (activateEscape())     doEscape(runBehaviour != ESCAPE);
  else if (activateAvoid())      doAvoid(runBehaviour != AVOID);
  else if (activateSeek())       doSeek(runBehaviour != SEEK, seekLight);
  else if (activateWallFollow()) doWallFollow(runBehaviour != WALLFOLLOW);
  else                           doCruise();    // default choice
}
