// Test MD_SmartCar with DC Motors/Encoders
//
#include <MD_SmartCar.h>
#include <MD_cmdProcessor.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#endif

// Global Variables
#if CONTROLLER_L298 || CONTROLLER_L293
SC_DCMotor_L298 ML(MC_INB1_PIN, MC_INB2_PIN, MC_ENB_PIN);  // Left motor
SC_DCMotor_L298 MR(MC_INA1_PIN, MC_INA2_PIN, MC_ENA_PIN);  // Right motor
#endif
#if CONTROLLER_M1508 || CONTROLLER_DRV8833
SC_DCMotor_M1508 ML(MC_INB1_PIN, MC_INB2_PIN);  // Left motor
SC_DCMotor_M1508 MR(MC_INA1_PIN, MC_INA2_PIN);  // Right motor
#endif

SC_MotorEncoder EL(EN_L_PIN);                         // Left motor encoder
SC_MotorEncoder ER(EN_R_PIN);                         // Right motor encoder

MD_SmartCar Car(&ML, &EL, &MR, &ER);                  // SmartCar object

// handler function prototypes
void handlerHelp(char* param);

void handlerV(char* param)
{
  int16_t v = 0;

  Serial.print(F("\n> Velocity "));
  v = strtoul(param, nullptr, 10);
  if (v < -100) v = -100;
  if (v > 100) v = 100;
  Serial.print(v);

  Car.setVelocity(v);
}

void handlerD(char* param)
{
  int v;
  int a;

  Serial.print(F("\n> Drive "));
  sscanf(param, "%d %d", &v, &a);
  if (v < -100) v = -100;
  if (v > 100) v = 100;
  if (a < -180) a = -180;
  if (a > 180) a = 180;
  Serial.print(v);
  Serial.print(' ');
  Serial.print(a);

  Car.drive((int8_t)v, (int8_t)a);
}

void handlerM(char* param)
{
  int vl, vr;
  int al, ar;

  Serial.print(F("\n> Move "));
  sscanf(param, "%d %d %d %d", &vl, &al, &vr, &ar);
  if (vl < -100) vl = -100;
  if (vl > 100) vl = 100;
  if (vr < -100) vr = -100;
  if (vr > 100) vr = 100;
  Serial.print(vl); Serial.print(' ');
  Serial.print(al); Serial.print(' ');
  Serial.print(vr); Serial.print(' ');
  Serial.print(ar); Serial.print(' ');

  Car.move((int8_t)vl, (uint16_t)al, (int8_t)vr, (uint16_t)ar);
}

void handlerX(char* param)
{
  Serial.print(F("\n> Stop "));
  Car.stop();
}

void handlerRS(char* param)
{
  Serial.print(F("\n\n----"));
  Serial.print(F("\nPWM Speeds: "));
  Serial.print(Car.getMinMotorSP());
  Serial.print(F(", "));
  Serial.print(Car.getMaxMotorSP());
  Serial.print(F("\nCreep: "));
  Serial.print(Car.getCreepSP());
  Serial.print(F("\n----\n"));
}

void handlerTK(char* param)
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

void handlerTP(char* param)
{
  uint16_t l, h;

  sscanf(param, "%d %d", &l, &h);
  Serial.print(F("\n> PWM "));
  Serial.print(l); Serial.print(", ");
  Serial.print(h);

  Car.setMinMotorSP(l);
  Car.setMaxMotorSP(h);
}

void handlerCS(char* param) { Car.saveConfig(); }
void handlerCL(char* param) { Car.loadConfig(); }

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?",  handlerHelp, "",      "Help", 0 },
  { "h",  handlerHelp, "",      "Help", 0 },
  { "v",  handlerV,  "n",       "Master velocity setting n [0..999]", 1 },
  { "rs", handlerRS,  "",       "Report configured speeds", 1 },
  { "d",  handlerD,  "v a",     "Drive vel v [-100,100] angle [-180,180]", 2 },
  { "m",  handlerM,  "l a r b", "Move speed l,r [-100,100] subtended angle a,b", 2 },
  { "x",  handlerX,  "",        "Stop", 2 },
  { "tk", handlerTK, "p i d",   "PID Tunings [p,i,d = (float value * 100)]", 3 },
  { "tp", handlerTP, "l h",     "PWM tunings [l, h = 0..255]", 3},
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
