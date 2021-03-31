// Test the SmartCar control using a PS2 Joystick module
// and a 1602 LCD module with I2C backpack.

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include <MD_SmartCar.h>
#include "SmartCar_HW.h"

#define DEBUG 1

#if DEBUG
#define PRINTS(s)   do { Serial.print(F(s)); } while (false)
#define PRINT(s,v)  do { Serial.print(F(s)); Serial.print(v); } while (false)
#define PRINTF(s,v) do { Serial.print(F(s)); Serial.print(v, 2); } while (false)
#else
#define PRINTS(s)
#define PRINT(s,v)
#define PRINTF(s,v)
#endif

//----------------------------------------------------------
// SmartCar Variables
#if CONTROLLER_L29x
SC_DCMotor_L29x ML(MC_INB1_PIN, MC_INB2_PIN, MC_ENB_PIN);  // Left motor
SC_DCMotor_L29x MR(MC_INA1_PIN, MC_INA2_PIN, MC_ENA_PIN);  // Right motor
#endif
#if CONTROLLER_MX1508
SC_DCMotor_MX1508 ML(MC_INB1_PIN, MC_INB2_PIN);  // Left motor
SC_DCMotor_MX1508 MR(MC_INA1_PIN, MC_INA2_PIN);  // Right motor
#endif
SC_MotorEncoder EL(EN_L_PIN);                   // Left motor encoder
SC_MotorEncoder ER(EN_R_PIN);                   // Right motor encoder

MD_SmartCar Car(&ML, &EL, &MR, &ER);            // SmartCar object

//----------------------------------------------------------
// LCD Display definitions (2 wire display interface)
#define LCD_PRINT(c, r, s) do { lcd.setCursor(c,r); lcd.print(s); } while (false);
#define LCD_WRITE(c, r, s) do { lcd.setCursor(c,r); lcd.write(s); } while (false);

hd44780_I2Cexp lcd;

//----------------------------------------------------------
// Joystick definitions
// Hardware connections
const uint8_t PIN_X = A0;
const uint8_t PIN_Y = A1;
const uint8_t PIN_SW = 11;

class cJoyStick
{
public:
  cJoyStick(uint8_t pinX, uint8_t pinY, uint8_t pinSW) :
    _pinX(pinX), _pinY(pinY), _pinSW(pinSW)
  { }

  void begin(void)
  {
    uint32_t sum;

    // initialize hardware
    pinMode(_pinX, INPUT);
    pinMode(_pinY, INPUT);
    pinMode(_pinSW, INPUT);

    // work out an average for center value
    sum = 0;
    for (uint8_t i = 0; i < AVERAGE_SAMPLE; i++)
    {
      sum += analogRead(_pinX);
      delay(20);
    }
    _centerX = sum / AVERAGE_SAMPLE;

    sum = 0;
    for (uint8_t i = 0; i < AVERAGE_SAMPLE; i++)
    {
      sum += analogRead(_pinY);
      delay(20);
    }
    _centerY = sum / AVERAGE_SAMPLE;
  }

  void read(int16_t& x, int16_t& y, bool &sw)
  {
    x = analogRead(_pinX) - _centerX;
    y = analogRead(_pinY) - _centerY;
    sw = (digitalRead(_pinSW) == LOW);
  }

private:
  static const uint8_t AVERAGE_SAMPLE = 10;

  uint8_t _pinX;    // pin for X analog input
  uint8_t _pinY;    // pin for Y analog input
  uint8_t _pinSW;   // pin for center switch

  int16_t _centerX; // center value for X axis
  int16_t _centerY; // center value for Y axis
};

cJoyStick J(PIN_X, PIN_Y, PIN_SW);

void setup(void)
{
#if DEBUG
  Serial.begin(57600);
#endif
  PRINTS("\nMD_SmartCar_Joystick Debug\n--------------------------");

  // SmartCar
  if (!Car.begin(PPR, PPS_MAX, DIA_WHEEL, LEN_BASE))
  {
    pinMode(LED_BUILTIN, OUTPUT);
    do 
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(750);
    } while (true);
  }

  // LCD display
  int status = lcd.begin(SC_LCD_COLS, SC_LCD_ROWS);
  if (status) // non zero status means it was unsuccessful
  {
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the on board LED if possible
    hd44780::fatalError(status); // does not return
  }

  // Joystick
  J.begin();
}

void loop(void)
{
  static uint32_t lastCheck;

  static int8_t pvelLin; // previous iteration
  static float pvelAng;   // previous iteration
  static bool pSW;        // previous iteration

  int8_t velLin;  // current iteration
  float velAng;   // current iteration

  int16_t X, Y;   // joystick read values
  bool SW;        // joystick read value
  
  if (millis() - lastCheck >= 500)
  {
    lastCheck = millis();

    J.read(X, Y, SW);
    PRINT("\nX:", X);
    PRINT(" Y:", Y);
    PRINT(" SW:", SW);

    velLin = (uint8_t)(100.0 * (float)(-X) / 512.0);     // axes depends on how joystick is mounted
    velAng = (PI * Y) / 512.0;     // axes depends on how joystick is mounted 
    PRINT(" -> V:", velLin);
    PRINT(" W:", velAng);

    if (SW != pSW && SW)
    {
      pSW = SW;
      Car.stop();

      lcd.clear();
      LCD_PRINT(0, 0, F("Stop"));
    }
    else if (velLin != pvelLin || velAng != pvelAng)
    {
      pvelLin = velLin;
      pvelAng = velAng;
      Car.drive(velLin, velAng);

      lcd.clear();
      LCD_PRINT(0, 0, F("Drive"));
      LCD_PRINT(0, 1, F("V:"));
      LCD_PRINT(2, 1, velLin);
      LCD_PRINT(6, 1, F("W:"));
      LCD_PRINT(8, 1, velAng);
    }
  }

  Car.run();    // always run the car
}
