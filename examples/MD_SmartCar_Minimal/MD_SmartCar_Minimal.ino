// Minimal MD_SmartCar implementation
// 
// Stop for TIME_PERIOD, drive in a circle for TIME_PERIOD, repeat.

#include <MD_SmartCar.h>

// ------------------------------------
// SmartCar Physical Constants
const uint16_t PPR = 40;        ///< Encoder pulses per revolution
const uint16_t PPS_MAX = 175;   ///< Maximum encoder pulses per second (@ PWM=255)
const uint16_t DIA_WHEEL = 65;  ///< Wheel diameter in mm
const uint16_t LEN_BASE = 110;  ///< Wheel base in mm (= distance between wheel centers)

// Global Variables
// L29x type motor controller
//SC_DCMotor_L29x ML(MC_INB1_PIN, MC_INB2_PIN, MC_ENB_PIN);  // Left motor
//SC_DCMotor_L29x MR(MC_INA1_PIN, MC_INA2_PIN, MC_ENA_PIN);  // Right motor

// MX1508 type motor controller
SC_DCMotor_MX1508 ML(MC_INB1_PIN, MC_INB2_PIN);  // Left motor
SC_DCMotor_MX1508 MR(MC_INA1_PIN, MC_INA2_PIN);  // Right motor

SC_MotorEncoder EL(EN_L_PIN);                         // Left motor encoder
SC_MotorEncoder ER(EN_R_PIN);                         // Right motor encoder

MD_SmartCar Car(&ML, &EL, &MR, &ER);                  // SmartCar object

void setup(void)
{
  if (!Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE))
  {
    // handle the error in come way!
  }
}

void loop(void)
{
  static const uint32_t TIME_PERIOD = 5000;    // in ms

  static bool inPause = true;
  static int8_t angularV = 90;    // degrees per second
  static int8_t velocity = 40;    // % full speed
  static uint32_t timeLast;

  if (millis() - timeLast >= TIME_PERIOD)
  {
    if (inPause)
      Car.drive(velocity, angularV);    // move the car
    else
      Car.stop();                       // stop the car
    timeLast = millis();
    inPause = !inPause;
  }

  Car.run();
}
