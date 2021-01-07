#pragma once
/**
 * \file
 * \brief Header file for SmartCar library hardware related definitions.
 */

#ifndef NO_PIN
#define NO_PIN 255          ///< Pin number when pin is not defined
#endif

// ------------------------------------
// Motor Controller
// 
// Left Motor
const uint8_t MC_INA1_PIN = 8;    ///< Motor A Mode pin 1 - simple digital pin
const uint8_t MC_INA2_PIN = 7;    ///< Motor A Mode pin 2 - simple digital pin
const uint8_t MC_ENA_PIN = 9;     ///< Motor A output enable - PWM capable pin
//Right Motor
const uint8_t MC_INB1_PIN = 5;    ///< Motor B Mode pin 1 - simple digital pin
const uint8_t MC_INB2_PIN = 4;    ///< Motor B Mode pin 2 - simple digital pin
const uint8_t MC_ENB_PIN = 6;     ///< Motor B output enable - PWM capable pin

// -----------------------------------
// Motor Encoder
//
const uint8_t EN_L_PIN = 3;    ///< Left Motor encoder interrupt pin
const uint8_t EN_R_PIN = 2;    ///< Right Motor encoder interrupt pin

// Physical Constants
const uint8_t PULSE_PER_REV = 40;   ///< Encoder pulses per wheel revolution
const uint8_t WHEEL_CIRCUM = 200;   ///< Wheel circumference ( = distance travelled per rev) in mm

// -----------------------------------
// PID Control
//
const double DefKp = 0.9;    ///< PID proportional weighting default
const double DefKi = 0.3;    ///< PID integral weighting default
const double DefKd = 0.1;    ///< PID derivative weighting default

const uint32_t PID_PERIOD = 250;  ///< PID calculation period in ms
const uint32_t MS_PER_SEC = 1000; ///< number of ms in 1 second

// -----------------------------------
// Defined PWN values for speeds
//
const uint8_t PWM_MIN = 90;      ///< Minimum PWM that will turn the motor
const uint8_t PWM_MAX = 255;     ///< Maximum PWM
const uint8_t PWM_CREEP = 120;   ///< Creep speed PWM default value

// -----------------------------------
// Configuration EEPROM settings
const uint16_t EEPROM_ADDR = 1023;     ///< EEPROM config data ENDS at this address (ie saved below addr)
const uint8_t SIG[2] = { 0xee, 0x55 }; ///< EEPROM config signature bytes
