#include <MD_SmartCar.h>
#include <EEPROM.h>

/**
 * \file
 * \brief Code file for MD_SmartCar library class - configuration data methods.
 */

void MD_SmartCar::loadConfig(void)
{
  PRINTS("\nLoaded Config");
  EEPROM.get(EEPROM_ADDR - sizeof(_config), _config);

  // if not valid signature, load defaults
  if (_config.sig[0] != SIG[0] || _config.sig[1] != SIG[1])
  {
    PRINTS(" - defaults");
    _config.sig[0] = SIG[0];
    _config.sig[1] = SIG[1];
    _config.creepPWM = MC_PWM_CREEP;
    _config.minPWM = MC_PWM_MIN;
    _config.maxPWM = MC_PWM_MAX;
    _config.Kp = DefKp;
    _config.Ki = DefKi;
    _config.Kd = DefKd;

    saveConfig();
  }

  printConfig();    // debug only
}

void MD_SmartCar::saveConfig(void)
{
  PRINTS("\nSaved Config");
  EEPROM.put(EEPROM_ADDR - sizeof(_config), _config);
}

void MD_SmartCar::printConfig(void)
// Only enabled when debugging is turned on
{
  PRINTS("\nCONFIG\n------");
  PRINTX("\nSig:\t", _config.sig[0]); PRINTX(", ", _config.sig[1]);
  PRINT("\nCreepSP:", _config.creepPWM);
  PRINT("\nPWM:\t", _config.minPWM); PRINT(", ", _config.maxPWM);
  PRINT("\nPID:\t", _config.Kp); PRINT(", ", _config.Ki); PRINT(", ", _config.Kd);
  PRINTS("\n------");
}
