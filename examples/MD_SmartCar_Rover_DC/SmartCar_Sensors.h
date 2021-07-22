#pragma once
// Encapsulates all vehicle sensors into one class
//
// NewPing library available from https://bitbucket.org/teckel12/arduino-new-ping/src/master/
//

#include <NewPing.h>
#include "SmartCar_HW.h"

class cSensors
{
public:
  cSensors(void) :
    bumperL(false), bumperR(false),
    sonarL(0), sonarM(0), sonarR(0),
    lightL(0), lightR(0)
  {}

  void begin(void) 
  {
    pinMode(PIN_R_BUMPER, INPUT_PULLUP);
    pinMode(PIN_L_BUMPER, INPUT_PULLUP);
    pinMode(PIN_L_LIGHT, INPUT);
    pinMode(PIN_R_LIGHT, INPUT);
  }

  void read(void) 
  {
    _newData = false;     // will be set if any updates happen
    readBumper();
    readSonar();
    readLight();
  }

  inline bool isUpdated(void) { return(_newData); }

  void dump(Stream& S)
  {
    if (isUpdated())
    {
      S.print("\n");
      S.print("B("); S.print(bumperL); S.print(','); S.print(bumperR);
      S.print(") S("); S.print(sonarL); S.print(','); S.print(sonarM); S.print(','); S.print(sonarR);
      S.print(") L("); S.print(lightL); S.print(','); S.print(lightR);
      S.print(")");
    }
  }

  // Available Sensor Data
  bool bumperL, bumperR;              // Bump switch
  uint16_t sonarL, sonarM, sonarR;    // Sonar data (0 -> distance > MAX_DISTANCE)
  uint16_t lightL, lightR;            // Light sensors

private:
  // Bumper definitions
  uint32_t _lastBumperPoll;
  bool _newData;

  void readBumper(void)
  {
    if (millis() - _lastBumperPoll >= BUMPER_POLL_PERIOD)
    {
      bool bl = (digitalRead(PIN_L_BUMPER) == LOW);
      bool br = (digitalRead(PIN_R_BUMPER) == LOW);

      _newData = _newData || (bumperL != bl) || (bumperR != br);
      bumperL = bl;
      bumperR = br;
      _lastBumperPoll = millis();
    }
  }

  // Sonar (Ping) definitions
  static const uint8_t MAX_SONAR = 3;             // Number of ping sensors
  uint8_t _curSonar;        // next device to poll
  uint32_t _lastSonarPoll;

  NewPing _sonar[MAX_SONAR] =
  {
    NewPing(PIN_L_SONAR, PIN_L_SONAR, DIST_MAX),
    NewPing(PIN_M_SONAR, PIN_M_SONAR, DIST_MAX),
    NewPing(PIN_R_SONAR, PIN_R_SONAR, DIST_MAX)
  };

  void readSonar(void)
  {
    if (millis() - _lastSonarPoll > SONAR_POLL_PERIOD)
    {
      uint16_t ping = _sonar[_curSonar].ping_cm();

      if (ping == 0) ping = DIST_ALLCLEAR;    // distance comparisons work better than with 0

      switch (_curSonar)
      {
      case 0: _newData = _newData || (sonarL != ping); sonarL = ping; break;
      case 1: _newData = _newData || (sonarM != ping); sonarM = ping; break;
      case 2: _newData = _newData || (sonarR != ping); sonarR = ping; break;
      }
      _curSonar++;
      if (_curSonar >= MAX_SONAR) _curSonar = 0;    // roll over
      _lastSonarPoll = millis();
    }
  }

  // LightSensor definitions
  uint32_t _lastLightPoll;

  void readLight(void)
  {
    if (millis() - _lastLightPoll >= LIGHT_POLL_PERIOD)
    {
      // divide the values by 4 to eliminate jitter in the bottom bits
      uint8_t ll = (analogRead(PIN_L_LIGHT) >> 2);
      uint8_t lr = (analogRead(PIN_R_LIGHT) >> 2);

      _newData = _newData || (lightL != ll) || (lightR != lr);
      lightL = ll;
      lightR = lr;
      _lastLightPoll = millis();
    }
  }
};

cSensors Sensors;     // declare one instance!