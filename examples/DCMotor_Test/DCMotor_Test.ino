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

const uint32_t REPORT_TIME = 1000;        // reporting period in ms
bool bEncoderEnabled = false;             // global flag for reporting
uint32_t timeLastReport = 0;              // report every REPORT_TIME period

// handler function prototypes
void handlerHelp(char* param);

void motorSpeed(MD_DCMotor &M, char m, char* param)
{
  uint16_t s = 0;

  Serial.print(F("\n> Speed "));
  Serial.print(m);
  Serial.print(F(" "));
  s = strtoul(param, nullptr, 10);
  if (s > 255) s = 255;
  Serial.print(s);
  M.setSpeed(s);
}

void motorMode(MD_DCMotor& M, char m, char* param)
{
  MD_DCMotor::runCmd_t cmd;

  Serial.print(F("\n> Run "));
  Serial.print(m);
  Serial.print(F(" "));
  switch (toupper(*param))
  {
  case 'S': cmd = MD_DCMotor::DIR_REL; Serial.print(F("REL")); break;
  case 'F': cmd = MD_DCMotor::DIR_FWD; Serial.print(F("FWD")); break;
  case 'R': cmd = MD_DCMotor::DIR_REV; Serial.print(F("REV")); break;
  case 'B': cmd = MD_DCMotor::DIR_BRK; Serial.print(F("BRK")); break;
  default:
    Serial.print(F("unknown '"));
    Serial.print(toupper(*param));
    Serial.print(F("'"));
    return;
  }

  M.run(cmd);
}

void handlerSL(char* param) { motorSpeed(ML, 'L', param); }
void handlerSR(char* param) { motorSpeed(MR, 'R', param); }
void handlerRL(char* param) { motorMode(ML, 'L', param); }
void handlerRR(char* param) { motorMode(MR, 'R', param); }

void handlerE(char* param)
{
  bEncoderEnabled = !bEncoderEnabled;
  Serial.print(F("\n> Encoder "));
  Serial.print(bEncoderEnabled ? F("ON") : F("OFF"));

  if (bEncoderEnabled)
    timeLastReport = millis();
}

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?",  handlerHelp, "",  "Help", 0 },
  { "h",  handlerHelp, "",  "Help", 0 },
  { "sl", handlerSL,  "n", "Left speed setting to n [0..255]", 1 },
  { "sr", handlerSR,  "n", "Right speed setting to n [0..255]", 1 },
  { "rl", handlerRL,  "m", "Run Left in mode m [f=fwd, r=rev, s=rel, b=brk]", 1 },
  { "rr", handlerRR,  "m", "Run Right in mode m [f=fwd, r=rev, s=rel, b=brk]", 1 },
  { "e", handlerE,    "",  "Toggle encoder reporting on/off", 2 },
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
  Serial.print(F("\nMD_SmartCar Motor/Encoder Tester\n------------------"));
  Serial.print(F("\nEnter command. Ensure line ending set to newline.\n"));

  ML.begin();
  MR.begin();
  if (!EL.begin())
    Serial.print(F("\n\n!! Unable to start EL"));
  if (!ER.begin())
    Serial.print(F("\n\n!! Unable to start EL"));

  // start command processor
  CP.begin();
  CP.help();
}

void loop(void)
{
  CP.run();

  // Encoder reporting
  if (bEncoderEnabled && millis() - timeLastReport >= REPORT_TIME)
  {
    uint32_t timeCount;
    uint16_t countL, countR;

    EL.read(timeCount, countL);   // read and reset
    ER.read(timeCount, countR);   // read and reset
    timeLastReport = millis();

    Serial.print(F("\n"));
    Serial.print(timeCount);
    Serial.print(F("\t"));
    Serial.print(countL);
    Serial.print(F("\t"));
    Serial.print(countR);
  }
}
