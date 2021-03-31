// Test MD_SmartCar class through a Bluetooth interface to 
// an AI2 control application using a HC-05 BT module that has 
// been pre-initialized and paired to the controller.
// 
// This is to test 'real-world' motion after the vehicle 
// parameters have been calibrated.
// 
// All Vehicle motion types can be exercised and parameters fine
// tuned from the AI2 'SmartCar_Setup_Control' interface application.

#include <SoftwareSerial.h>
#include <MD_SmartCar.h>
#include <MD_cmdProcessor.h>
#include "SmartCar_HW.h"

#define ECHO_COMMAND 0    // Echo commands to the Serial stream for debugging

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

const uint8_t FP_SIG = 2;   // floating point significant decimal points

// Global Objects and Variables
SoftwareSerial BTSerial(PIN_BT_RX, PIN_BT_TX);

// handler function prototypes
void handlerD(char* param);
void handlerM(char* param);
void handlerR(char* param);
void handlerPI(char* param);
void handlerPW(char* param);
void handlerPS(char* param);
void handlerS(char* param);

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "r",  handlerR,  "",        "Report configured speeds", 1 },
  { "d",  handlerD,  "v a",     "Drive lin vel v [-100..100] ang vel a [-90..90]", 2 },
  { "m",  handlerM,  "l r",     "Move wheels subtended angle l, r", 2 },
  { "z",  handlerZ,  "f",       "Spin full circle fraction f% [-100, 100]", 2 },
  { "pi", handlerPI, "n p i d", "PID Tunings motor n [p,i,d = (float value * 100)]", 3 },
  { "pw", handlerPW, "l h k m", "PWM tunings (low, high, kicker, move) [0..255]", 3},
  { "ps", handlerPS, "f",       "Interial Adjust f for spin() [float value * 100]", 3},
  { "s",  handlerS,  "",        "Configuration Save", 4 },
};

MD_cmdProcessor CP(BTSerial, cmdTable, ARRAY_SIZE(cmdTable));

// handler functions
void handlerD(char* param)
{
  int v, a;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d %d", &v, &a);
  //v = trunc(fv);
  //a = trunc(fa);
  if (v < -100) v = -100;
  if (v > 100) v = 100;
  if (a < -90) a = -90;
  if (a > 90) a = 90;

  Car.drive((int8_t)v, (int8_t)a);
}

void handlerM(char* param)
{
  int al, ar;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d %d", &al, &ar);

  Car.move((int16_t)al, (int16_t)ar);
}

void handlerZ(char* param)
{
  int f;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d", &f);

  Car.spin((int16_t)f);
}
 
void handlerR(char* param)
{
  float Kp, Ki, Kd;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  Car.getPIDTuning(0, Kp, Ki, Kd);
  BTSerial.print(F("PID_L: "));   BTSerial.print(Kp, FP_SIG);
  BTSerial.print(F(", "));        BTSerial.print(Ki, FP_SIG);
  BTSerial.print(F(", "));        BTSerial.print(Kd, FP_SIG);
  Car.getPIDTuning(1, Kp, Ki, Kd);
  BTSerial.print(F("\nPID_R: ")); BTSerial.print(Kp, FP_SIG);
  BTSerial.print(F(", "));        BTSerial.print(Ki, FP_SIG);
  BTSerial.print(F(", "));        BTSerial.print(Kd, FP_SIG);
  BTSerial.print(F("\nPWM: "));   BTSerial.print(Car.getMinMotorSP());
  BTSerial.print(F(", "));        BTSerial.print(Car.getMaxMotorSP());
  BTSerial.print(F("\nKick: "));  BTSerial.print(Car.getKickerSP());
  BTSerial.print(F("\nMove: "));  BTSerial.print(Car.getMoveSP());
  BTSerial.print(F("\nSpin: "));  BTSerial.print(Car.getSpinSP());
  BTSerial.print(F("\n"));
}

void handlerPI(char* param)
{
  uint16_t m, ip, ii, id;
  float fp, fi, fd;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d %d %d %d", &m, &ip, &ii, &id);
  fp = (float)ip / 100.0;
  fi = (float)ii / 100.0;
  fd = (float)id / 100.0;

  Car.setPIDTuning(m, fp, fi, fd);
}

void handlerPW(char* param)
{
  uint16_t l, h, k, m;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d %d %d %d", &l, &h, &k, &m);

  Car.setMinMotorSP(l);
  Car.setMaxMotorSP(h);
  Car.setKickerSP(k);
  Car.setMoveSP(m);
}

void handlerPS(char* param)
{
  int ai;
  float af;

#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  sscanf(param, "%d", &ai);
  af = ai / 100.0;

  Car.setSpinSP(af);
}

void handlerS(char* param)
{
#if ECHO_COMMAND
  Serial.println(CP.getLastCmdLine());
#endif
  Car.saveConfig();
}

void setup(void)
{
#if ECHO_COMMAND || SCDEBUG || PID_TUNE
  Serial.begin(57600);
#endif
  BTSerial.begin(BT_BAUDRATE);
  if (!Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE))   // take all the defaults
    BTSerial.print(F("\n\n!! Unable to start car"));

  CP.begin();
}

void loop(void)
{
  Car.run();
  CP.run();
}
