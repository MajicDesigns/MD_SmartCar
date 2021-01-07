#include <MD_SmartCar.h>

/**
 * \file
 * \brief Code file for SmartCar library class - path management data and methods.
 */

const MD_SmartCar::pathDef_t PROGMEM MD_SmartCar::pathTable[] =
// REMEMBER TO CHANGE NUMBER OF ELEMENTS IN CLASS DEFINITION
{
  { LINEAR, CENTER, FWD,  CREEP, {{ MD_DCMotor::DIR_FWD, PULSE_PER_REV }, { MD_DCMotor::DIR_FWD, PULSE_PER_REV }} },
  { ATURN,  CENTER, STOP, CREEP, {{ MD_DCMotor::DIR_FWD, PULSE_PER_REV }, { MD_DCMotor::DIR_REV, PULSE_PER_REV }} },
  { TURN,   LEFT,   STOP, CREEP, {{ MD_DCMotor::DIR_BRK, 0 }, { MD_DCMotor::DIR_REV, PULSE_PER_REV }} },
  { TURN,   RIGHT,  STOP, CREEP, {{ MD_DCMotor::DIR_REV, PULSE_PER_REV }, { MD_DCMotor::DIR_BRK, 0 }} },
  { LINEAR, CENTER, FWD,  RUN,   {{ MD_DCMotor::DIR_FWD, 0 }, { MD_DCMotor::DIR_FWD, 0 }} },
  { LINEAR, CENTER, REV,  RUN,   {{ MD_DCMotor::DIR_REV, 0 }, { MD_DCMotor::DIR_REV, 0 }} },
  { LINEAR, CENTER, STOP, RUN,   {{ MD_DCMotor::DIR_BRK, 0 }, { MD_DCMotor::DIR_BRK, 0 }} },
};


bool MD_SmartCar::findPath(pathDef_t *rpath, pathType_t fpath, sideType_t fside, dirType_t fdir, runType_t frun, const pathDef_t *table, uint16_t count)
{
  bool found = false;
  pathDef_t tpath;

  for (uint8_t i = 0; i < count && !found; i++)
  {
    memcpy_P(&tpath, &table[i], sizeof(pathDef_t));   // make a local copy we can use
    found = (tpath.path == fpath && tpath.side == fside && tpath.dir == fdir && tpath.run == frun);
    if (found) memcpy(rpath, &tpath, sizeof(pathDef_t));
  }

  return(found);
}