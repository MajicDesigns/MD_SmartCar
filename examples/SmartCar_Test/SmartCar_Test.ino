// Test MD_DCMotor with DC Motors
//
#include <MD_SmartCar.h>
#include <MD_cmdProcessor.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#endif

// Global Variables
MD_DCMotor ML(MC_INB1_PIN, MC_INB2_PIN, MC_ENB_PIN);  // Left motor
MD_MotorEncoder EL(EN_L_PIN);                         // Left motor encoder
MD_DCMotor MR(MC_INA1_PIN, MC_INA2_PIN, MC_ENA_PIN);  // Right motor
MD_MotorEncoder ER(EN_R_PIN);                         // Right motor encoder
MD_SmartCar Car(&ML, &EL, &MR, &ER);                  // SmartCar object

// handler function prototypes
void handlerHelp(char* param);

bool char2dir(MD_SmartCar::dirType_t &d, char c)
{
  bool b = true;

  switch (toupper(c))
  {
  case 'S': d = MD_SmartCar::STOP; Serial.print(F("STP ")); break;
  case 'F': d = MD_SmartCar::FWD;  Serial.print(F("FWD ")); break;
  case 'R': d = MD_SmartCar::REV;  Serial.print(F("REV ")); break;
  default: b = false; break;
  }

  return(b);
}

bool char2side(MD_SmartCar::sideType_t& s, char c)
{
  bool b = true;

  switch (toupper(c))
  {
  case 'C': s = MD_SmartCar::CENTER; Serial.print(F("CNTR "));  break;
  case 'L': s = MD_SmartCar::LEFT;   Serial.print(F("LEFT ")); break;
  case 'R': s = MD_SmartCar::RIGHT;  Serial.print(F("RIGHT ")); break;
  default: b = false; break;
  }

  return(b);
}

bool char2path(MD_SmartCar::pathType_t& p, char c)
{
  bool b = true;

  switch (toupper(c))
  {
  case 'L': p = MD_SmartCar::LINEAR; Serial.print(F("LIN ")); break;
  case 'V': p = MD_SmartCar::VEER;   Serial.print(F("VEER "));   break;
  case 'T': p = MD_SmartCar::TURN;   Serial.print(F("TURN "));   break;
  case 'A': p = MD_SmartCar::ATURN;  Serial.print(F("ATRN "));  break;
  default: b = false; break;
  }

  return(b);
}

void handlerS(char* param)
{
  uint16_t s = 0;

  Serial.print(F("\n> Speed "));
  s = strtoul(param, nullptr, 10);
  if (s > 255) s = 255;
  Serial.print(s);

  Car.setSpeed(s);
}

void handlerR(char* param)
{
  MD_SmartCar::dirType_t d;
  MD_SmartCar::sideType_t s;
  MD_SmartCar::pathType_t p;
  uint16_t spd = 0;

  Serial.print(F("\n> Run "));
  if (!char2dir(d, param[0]))
  {
    Serial.print(F("unknown '"));
    Serial.print((char)toupper(param[0]));
    Serial.print(F("'"));
    return;
  }

  if (!char2side(s, param[1]))
  {
    Serial.print(F(" unknown '"));
    Serial.print((char)toupper(param[1]));
    Serial.print(F("'"));
    return;
  }

  if (!char2path(p, param[2]))
  {
    Serial.print(F(" unknown '"));
    Serial.print((char)toupper(param[2]));
    Serial.print(F("'"));
    return;
  }

  spd = strtoul(&param[3], nullptr, 10);
  if (spd > 255) spd = 255;
  Serial.print(spd);

  Car.run(p, s, d, spd);
}

void handlerC(char* param)
{
  MD_SmartCar::dirType_t d;
  MD_SmartCar::sideType_t s;
  MD_SmartCar::pathType_t p;

  Serial.print(F("\n> Creep "));
  if (!char2dir(d, param[0]))
  {
    Serial.print(F("unknown '"));
    Serial.print((char)toupper(param[0]));
    Serial.print(F("'"));
    return;
  }

  if (!char2side(s, param[1]))
  {
    Serial.print(F(" unknown '"));
    Serial.print((char)toupper(param[1]));
    Serial.print(F("'"));
    return;
  }

  if (!char2path(p, param[2]))
  {
    Serial.print(F(" unknown '"));
    Serial.print((char)toupper(param[2]));
    Serial.print(F("'"));
    return;
  }

  Car.creep(p, s, d);
}

void handlerX(char* param)
{
  Serial.print(F("\n> Stop "));
  Car.stop();
}

void handlerP(char* param)
{
  uint16_t ip, ii, id;
  float fp, fi, fd;

  sscanf(param, "%d %d %d", &ip, &ii, &id);
  fp = (float)ip / 100.0;
  fi = (float)ii / 100.0;
  fd = (float)id / 100.0;
  Serial.print(F("\n> PID "));
  Serial.print(fp); Serial.print(", ");
  Serial.print(fi); Serial.print(", ");
  Serial.print(fd);

  Car.setPIDTuning(fp, fi, fd);
}

void handlerCS(char* param) { Car.saveConfig(); }
void handlerCL(char* param) { Car.loadConfig(); }

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?",  handlerHelp, "",    "Help", 0 },
  { "h",  handlerHelp, "",    "Help", 0 },
  { "s",  handlerS,  "n",     "Left speed setting to n [0..255]", 1 },
  { "r",  handlerR,  "dsp v", "Run d [S,F,R] s [C,L,R] p [Ln,Vr,Tn,Atn] v", 1 },
  { "c",  handlerC,  "dsp",   "Creep d [S,F,R] s [C,L,R] p [Ln,Vr,Tn,Atn]", 1 },
  { "x",  handlerX,  "",      "Stop", 1 },
  { "p",  handlerP,  "p i d", "PID Tunings (value * 100)", 2 },
  { "cs", handlerCS, "",      "Configuration Save", 3 },
  { "cl", handlerCL, "",      "Configuration Load", 3 },
};

MD_cmdProcessor CP(Serial, cmdTable, ARRAY_SIZE(cmdTable));

void handlerHelp(char* param)
{
  Serial.print(F("\nHelp\n----"));
  CP.help();
}

void setup(void)
{
  Serial.begin(57600);

  if (!Car.begin())
    Serial.print(F("\n\n!! Unable to start car"));

  // start command processor
  Serial.print(F("\nMD_SmartCar Tester\n------------------"));
  Serial.print(F("\nEnter command. Ensure line ending set to newline.\n"));
  CP.begin();
  CP.help();
}

void loop(void)
{
  Car.move();
  CP.run();
}
