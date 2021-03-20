#include <MD_SmartCar.h>
#include <EEPROM.h>

/**
 * \file
 * \brief Code file for MD_SmartCar library class - configuration data methods.
 */

void MD_SmartCar::loadConfig(void)
{
  SCPRINTS("\nLoaded Config");
  EEPROM.get(EEPROM_ADDR - sizeof(_config), _config);

  // if not valid signature, load defaults
  if (_config.sig[0] != SIG[0] || _config.sig[1] != SIG[1])
  {
    SCPRINTS(" - defaults");
    _config.sig[0] = SIG[0];
    _config.sig[1] = SIG[1];

    _config.movePWM = MC_PWM_MOVE;
    _config.kickerPWM = MC_PWM_KICKER;
    _config.minPWM = MC_PWM_MIN;
    _config.maxPWM = MC_PWM_MAX;
    _config.spinAdjust = MC_SPIN_ADJUST;

    for (uint8_t i = 0; i < MAX_MOTOR; i++)
    {
      _config.Kp[i] = DefKp;
      _config.Ki[i] = DefKi;
      _config.Kd[i] = DefKd;
    }

    saveConfig();
  }

#if SCDEBUG
  printConfig();    // debug only
#endif
}

void MD_SmartCar::saveConfig(void)
{
  SCPRINTS("\nSaved Config");
  EEPROM.put(EEPROM_ADDR - sizeof(_config), _config);
}

#if SCDEBUG
void MD_SmartCar::printConfig(void)
// Only enabled when debugging is turned on
{
  SCPRINTS("\nCONFIG\n------");
  SCPRINTX("\nSig: ", _config.sig[0]); SCPRINTX(", ", _config.sig[1]);
  SCPRINT("\nMove PWM: ", _config.movePWM);
  SCPRINT("\nKicker PWM: ", _config.kickerPWM);
  SCPRINT("\nSpin Inertial: ", _config.spinAdjust);
  SCPRINT("\nPWM: ", _config.minPWM); SCPRINT(", ", _config.maxPWM);
  for (uint8_t i = 0; i < MAX_MOTOR; i++)
  {
    SCPRINT("\nPID", i);
    SCPRINT(": ", _config.Kp[i]);
    SCPRINT(", ", _config.Ki[i]);
    SCPRINT(", ", _config.Kd[i]);
  }

  SCPRINTS("\n------");
}
#endif