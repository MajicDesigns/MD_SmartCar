/**
 * \file
 * \brief Class definition file for the MD_MotorEncoder class.
 */

#include "MD_MotorEncoder.h"

MD_MotorEncoder::MD_MotorEncoder(uint8_t pinInt) :
  _pinInt(pinInt) 
{
}

MD_MotorEncoder::~MD_MotorEncoder(void)
{
  if (_pinInt != NO_PIN)
    detachInterrupt(digitalPinToInterrupt(_pinInt));
  _ISRAlloc &= ~_BV(_myISRId);   // free up the ISR slot for someone else
}

bool MD_MotorEncoder::begin(void)
{
  int8_t irq = digitalPinToInterrupt(_pinInt);

  if (irq == NOT_AN_INTERRUPT)
    _pinInt = NO_PIN;
  else
  {
    pinMode(_pinInt, INPUT_PULLUP);

    // assign ourselves a ISR ID ...
    _myISRId = UINT8_MAX;
    for (uint8_t i = 0; i < MAX_ISR; i++)
    {
      if (!(_ISRAlloc & _BV(i)))    // found a free ISR Id?
      {
        _myISRId = i;                 // remember who this instance is
        _myInstance[_myISRId] = this; // record this instance
        _ISRAlloc |= _BV(_myISRId);   // lock this in the allocations table
        break;
      }
    }

    //Serial.print("\nPin ");             Serial.print(_pinInt);
    //Serial.print(" (irq ");             Serial.print(irq);
    //Serial.print(") assigned ISR Id "); Serial.print(_myISRId);
    //Serial.print(", ISRAlloc 0b");      Serial.print(_ISRAlloc, BIN);

    // ... and attach corresponding ISR callback from the lookup table
    {
      static void((*ISRfunc[MAX_ISR])(void)) =
      {
        encoderISR0, encoderISR1, encoderISR2, encoderISR3,
        encoderISR4, encoderISR5, encoderISR6, encoderISR7,
      };

      if (_myISRId != UINT8_MAX)
        attachInterrupt(irq, ISRfunc[_myISRId], CHANGE);
      else
        _pinInt = NO_PIN;
    }

    reset();
  }

  return(_pinInt != NO_PIN);
}

void MD_MotorEncoder::reset(void)
{
  if (_pinInt != NO_PIN)
  {
    _timeLast = millis();
    _counter = 0;
  }
}

void MD_MotorEncoder::read(uint32_t& interval, uint16_t& count, bool bReset)
{
  if (_pinInt != NO_PIN)
  {
    count = _counter;
    interval = millis() - _timeLast;

    if (bReset) reset();
  }
}

void MD_MotorEncoder::handleISR(void) { _counter++; }  ///< Instance ISR handler called from static ISR encoderISRx

// Interrupt handling declarations required outside the class
uint8_t MD_MotorEncoder::_ISRAlloc = 0;     ///< allocation table for the encoderISRx()
MD_MotorEncoder* MD_MotorEncoder::_myInstance[MAX_ISR]; ///< callback instance for the ISR

// ISR for each myISRId
void MD_MotorEncoder::encoderISR0(void) { MD_MotorEncoder::_myInstance[0]->handleISR(); }
void MD_MotorEncoder::encoderISR1(void) { MD_MotorEncoder::_myInstance[1]->handleISR(); }
void MD_MotorEncoder::encoderISR2(void) { MD_MotorEncoder::_myInstance[2]->handleISR(); }
void MD_MotorEncoder::encoderISR3(void) { MD_MotorEncoder::_myInstance[3]->handleISR(); }
void MD_MotorEncoder::encoderISR4(void) { MD_MotorEncoder::_myInstance[4]->handleISR(); }
void MD_MotorEncoder::encoderISR5(void) { MD_MotorEncoder::_myInstance[5]->handleISR(); }
void MD_MotorEncoder::encoderISR6(void) { MD_MotorEncoder::_myInstance[6]->handleISR(); }
void MD_MotorEncoder::encoderISR7(void) { MD_MotorEncoder::_myInstance[7]->handleISR(); }
