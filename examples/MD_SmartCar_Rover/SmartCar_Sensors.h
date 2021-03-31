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
    pinMode(PIN_L_BUMPER, INPUT_PULLUP);
    pinMode(PIN_R_BUMPER, INPUT_PULLUP);
  }

  void read(void) 
  {
    readBumper();
    readSonar();
    readLight();
  }

  // Available Sensor Data
  bool bumperL, bumperR;              // Bump switch
  uint16_t sonarL, sonarM, sonarR;    // Sonar data (0 -> distance > MAX_DISTANCE)
  uint16_t lightL, lightR;            // Light sensors

private:
  // Bumper definitions
  static const uint16_t BUMPER_POLL_PERIOD = 10;   // in ms
  uint32_t _lastBumperPoll;

  void readBumper(void)
  {
    if (millis() - _lastBumperPoll >= BUMPER_POLL_PERIOD)
    {
      bumperL = (digitalRead(PIN_L_BUMPER) == LOW);
      bumperR = (digitalRead(PIN_R_BUMPER) == LOW);
      _lastBumperPoll = millis();
    }
  }

  // Sonar (Ping) definitions
  static const uint8_t MAX_SONAR = 3;             // Number of ping sensors
  static const uint8_t MAX_DISTANCE = 200;        // Maximum distance (in cm) to ping
  static const uint16_t SONAR_POLL_PERIOD = 50;   // in ms
  uint8_t _curSonar;        // next device to poll
  uint32_t _lastSonarPoll;

  NewPing _sonar[MAX_SONAR] =
  {
    NewPing(PIN_L_SONAR, PIN_L_SONAR, MAX_DISTANCE),
    NewPing(PIN_M_SONAR, PIN_M_SONAR, MAX_DISTANCE),
    NewPing(PIN_R_SONAR, PIN_R_SONAR, MAX_DISTANCE)
  };

  void readSonar(void)
  {
    if (millis() - _lastSonarPoll > SONAR_POLL_PERIOD)
    {
      uint16_t ping = _sonar[_curSonar].ping_cm();

      if (ping == 0) ping = 999;    // distance comparisons work better than with 0

      switch (_curSonar)
      {
      case 0: sonarL = ping; break;
      case 1: sonarM = ping; break;
      case 2: sonarR = ping; break;
      }
      _lastSonarPoll = millis();
      _curSonar++;
      if (_curSonar >= MAX_SONAR) _curSonar = 0;    // roll over
    }
  }

  // LightSensor definitions
  static const uint16_t LIGHT_POLL_PERIOD = 200;   // in ms
  uint32_t _lastLightPoll;

  void readLight(void)
  {
    if (millis() - _lastLightPoll >= LIGHT_POLL_PERIOD)
    {
      // do something?
      _lastLightPoll = millis();
    }
  }
};

cSensors Sensors;     // declare one instance!