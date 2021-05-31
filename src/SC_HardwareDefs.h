#pragma once
/**
 * \file
 * \brief Header file for MD_SmartCar library hardware related parameters.
 */

/**
\page pageHardwareMap Hardware Allocation Map 

## Hardware Functional Allocation for Arduino Nano

The library code is mostly independent of processor used, however the original 
target platform is an Arduino Nano. The pins listed here and defined in the 
SC_HardwareDefs.h header file are reserved (R) by the library. Spare pins are 
for use by any application Application (A) built on top ogf this library.

| Pin  |Use| Description
|------|:--|:---------------
| D0*  | R | Hardware Serial Tx
| D1*  | R | Hardware Serial Rx
| D2!  | R | R Motor Encoder Pulse Interrupt
| D3!~ | R | L Motor Encoder Pulse Interrupt
| D4   | R | R Motor Controller InB1 
| D5~  | R | R Motor Controller InB2
| D6~  | R | L Motor Controller InA1
| D7   | R | L Motor Controller InA2
| D8   | A | Spare
| D9~  |A/R| Spare / R Motor Controller PWM (L29x type controller)
| D10~ |A/R| Spare / L Motor Controller PWM (L29x type controller)
| D11~*| A | Spare / Hardware SPI MOSI
| D12* | A | Spare / Hardware SPI MISO
| D13* | A | Spare / Hardware SPI SCK
| A0   | A | Spare
| A1   | A | Spare
| A2   | A | Spare
| A3   | A | Spare 
| A4*  | A | Spare / Hardware I2C SDA
| A5*  | A | Spare / Hardware I2C SCL
| A6   | A | Spare / Nano Analog I/O only
| A7   | A | Spare / Nano Analog I/O only
 
(*) shared or comms bus pins, (~) hardware PWM pin, (!) external iRQ pin
*/

// ------------------------------------
// Motor Controller - Select hardware being used
// 

// Left Motor
const uint8_t MC_INA1_PIN = 6;    ///< Motor A Mode pin 1 - simple digital pin
const uint8_t MC_INA2_PIN = 7;    ///< Motor A Mode pin 2 - simple digital pin
const uint8_t MC_ENA_PIN = 10;    ///< Motor A output enable - PWM capable pin (L29x type only)
// Right Motor
const uint8_t MC_INB1_PIN = 4;    ///< Motor B Mode pin 1 - simple digital pin
const uint8_t MC_INB2_PIN = 5;    ///< Motor B Mode pin 2 - simple digital pin
const uint8_t MC_ENB_PIN = 9;     ///< Motor B output enable - PWM capable pin (L29x type only)

// Default PWM values for speeds
const uint8_t MC_PWM_MIN = 40;    ///< Minimum PWM that will turn the motor
const uint8_t MC_PWM_MAX = 255;   ///< Maximum PWM
const uint8_t MC_PWM_MOVE = 40;   ///< Slow move() speed PWM default value
const uint8_t MC_PWM_KICKER = 60; ///< Kicker for drive() to overcome static friction from standing start
const uint8_t MC_KICKER_ACTIVE = 100; ///< Kicker active time in milliseconds
const float MC_SPIN_ADJUST = 0.75;    ///< Inertial adjustment for spin() operation

// -----------------------------------
// Motor Encoder
//
const uint8_t EN_L_PIN = 3;    ///< Left Motor encoder interrupt pin
const uint8_t EN_R_PIN = 2;    ///< Right Motor encoder interrupt pin

// -----------------------------------
// PID Control
//
const float DefKp = 1.50;    ///< PID proportional weighting default
const float DefKi = 0.00;    ///< PID integral weighting default
const float DefKd = 0.15;    ///< PID derivative weighting default

const uint16_t PID_PERIOD = 250;  ///< PID calculation period in ms
const uint16_t MS_PER_SEC = 1000; ///< number of ms in 1 second
const float PID_FREQ = ((float)MS_PER_SEC / (float)PID_PERIOD);  ///< whole divisor is better

// -----------------------------------
// Configuration EEPROM settings
const uint16_t EEPROM_ADDR = 1023;     ///< EEPROM config data ENDS at this address (ie saved below addr)
const uint8_t SIG[2] = { 0xaa, 0x33 }; ///< EEPROM config signature bytes
