// Test MD_SmartCar class PID/DC Motors/Encoders and Unicycle model
//
#include <MD_SmartCar.h>
#include <MD_cmdProcessor.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#endif

#ifndef ECHO_COMMAND
#define ECHO_COMMAND  0   // set to 0 to not echo command on terminal
#endif

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

const uint8_t FP_SIG = 2;   // floating point number significant figures

// handler function prototypes
void handlerHelp(char* param);

void handlerV(char* param)
{
  int16_t v = 0;

  v = strtoul(param, nullptr, 10);
  if (v < -100) v = -100;
  if (v > 100) v = 100;
#if ECHO_COMMAND
  Serial.print(F("\n> Velocity "));
  Serial.print(v);
#endif
  Car.setVelocity(v);
}

void handlerD(char* param)
{
  int v, a;

  sscanf(param, "%d %d", &v, &a);
  if (v < -100) v = -100;
  if (v > 100) v = 100;
  if (a < -90) a = -90;
  if (a > 90) a = 90;
#if ECHO_COMMAND
  Serial.print(F("\n> Drive "));
  Serial.print(v);
  Serial.print(' ');
  Serial.print(a);
#endif
  Car.drive((int8_t)v, (int8_t)a);
}

void handlerM(char* param)
{
  int al, ar;

  sscanf(param, "%d %d", &al, &ar);
#if ECHO_COMMAND
  Serial.print(F("\n> Move "));
  Serial.print(al); Serial.print(' ');
  Serial.print(ar); Serial.print(' ');
#endif

  Car.move((int16_t)al, (int16_t)ar);
}

void handlerX(char* param)
{
#if ECHO_COMMAND
  Serial.print(F("\n> Stop "));
#endif
  Car.stop();
}

void handlerRP(char* param)
{
  float kp, ki, kd;

  Serial.print(F("\n\n----"));

  Serial.print(F("\nPWM Speeds: "));
  Serial.print(Car.getMinMotorSP());
  Serial.print(F(", "));
  Serial.print(Car.getMaxMotorSP());

  Serial.print(F("\nMove: "));
  Serial.print(Car.getMoveSP());

  Serial.print(F("\nKicker: "));
  Serial.print(Car.getKickerSP());

  for (uint8_t i = 0; i < MD_SmartCar::MAX_MOTOR; i++)
  {
    Car.getPIDTuning(i, kp, ki, kd);
    Serial.print(F("\nPID"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(kp, FP_SIG);
    Serial.print(F(", "));
    Serial.print(ki, FP_SIG);
    Serial.print(F(", "));
    Serial.print(kd, FP_SIG);
  }

  Serial.print(F("\n----\n"));
}

void handlerTP(char* param)
{
  uint16_t m, ip, ii, id;
  char c;
  float fp, fi, fd;
  bool bAll = (*param == '*');

  if (bAll)
    sscanf(param, "%c %d %d %d", &c, &ip, &ii, &id);
  else
    sscanf(param, "%d %d %d %d", &m, &ip, &ii, &id);
  fp = (float)ip / 100.0;
  fi = (float)ii / 100.0;
  fd = (float)id / 100.0;
#if ECHO_COMMAND
  Serial.print(F("\n> PID"));
  if (!bAll) Serial.print(m);
  Serial.print(F(" "));
  Serial.print(fp, FP_SIG); Serial.print(", ");
  Serial.print(fi, FP_SIG); Serial.print(", ");
  Serial.print(fd, FP_SIG);
#endif

  // set the tuning based on range
  uint8_t start = bAll ? 0 : m;
  uint8_t end = bAll ? MD_SmartCar::MAX_MOTOR : m + 1;
  //Serial.print("\n["); Serial.print(start); Serial.print(","); Serial.print(end); Serial.print("]");
  for (uint8_t i = start; i < end; i++)
    Car.setPIDTuning(i, fp, fi, fd);
}

void handlerTW(char* param)
{
  uint16_t l, h;

  sscanf(param, "%d %d", &l, &h);
#if ECHO_COMMAND
  Serial.print(F("\n> PWM "));
  Serial.print(l); Serial.print(", ");
  Serial.print(h);
#endif

  Car.setMinMotorSP(l);
  Car.setMaxMotorSP(h);
}

void handlerTK(char* param)
{
  uint16_t v;

  v = atoi(param);
#if ECHO_COMMAND
  Serial.print(F("\n> Kicker "));
  Serial.print(v);
#endif

  Car.setKickerSP(v);
}

void handlerTM(char* param)
{
  uint16_t v;

  v = atoi(param);
#if ECHO_COMMAND
  Serial.print(F("\n> Move "));
  Serial.print(v);
#endif

  Car.setMoveSP(v);
}

void handlerCS(char* param) { Serial.print(F("\n> Save ")); Car.saveConfig(); }
void handlerCL(char* param) { Serial.print(F("\n> Load ")); Car.loadConfig(); }

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?",  handlerHelp, "",      "Help", 0 },
  { "h",  handlerHelp, "",      "Help", 0 },
  { "v",  handlerV,  "n",       "Velocity master setting n [0..999]", 1 },
  { "rp", handlerRP,  "",       "Report Parameters", 1 },
  { "d",  handlerD,  "v a",     "Drive vel v [-100,100] angle a [-90,90]", 2 },
  { "m",  handlerM,  "l r",     "Move wheels subtended angle l, r", 2 },
  { "x",  handlerX,  "",        "Stop", 2 },
  { "tp", handlerTP, "n p i d", "Tuning PID motor n or * [p,i,d=(float * 100)]", 3 },
  { "tw", handlerTW, "l h  ",   "Tuning PWM low/high [l, h=0..255]", 3},
  { "tk", handlerTK, "p",       "Tuning drive() Kicker PWM [p=0..255]", 3 },
  { "tm", handlerTM, "p",       "Tuning Move() PWM [p=0..255]", 3 },
  { "cs", handlerCS, "",        "Configuration Save", 4 },
  { "cl", handlerCL, "",        "Configuration Load", 4 },
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
  Serial.print(F("\n\nMD_SmartCar Tester\n------------------"));
  Serial.print(F("\nEnter command. Ensure line ending set to newline.\n"));
  CP.begin();
  CP.help();
}

void loop(void)
{
  Car.run();
  CP.run();
}
