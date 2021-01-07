#include <MD_SmartCar.h>

/**
 * \file
 * \brief Code file for SmartCar library class - configuration data methods.
 */

void MD_SmartCar::loadConfig(void)
{
  PRINTS("\nLoaded Config")
    EEPROM.get(EEPROM_ADDR - sizeof(_config), _config);

  // if not valid signature, load defaults
  if (_config.sig[0] != SIG[0] || _config.sig[1] != SIG[1])
  {
    PRINTS(" - defaults");
    _config.sig[0] = SIG[0];
    _config.sig[1] = SIG[1];
    _config.creepSP = PWM_CREEP;
    _config.minPWM = PWM_MIN;
    _config.maxPWM = PWM_MAX;
    _config.Kp = DefKp;
    _config.Ki = DefKi;
    _config.Kd = DefKd;

    saveConfig();
  }
}

void MD_SmartCar::saveConfig(void)
{
  PRINTS("\nSaved Config");
  EEPROM.put(EEPROM_ADDR - sizeof(_config), _config);
}
