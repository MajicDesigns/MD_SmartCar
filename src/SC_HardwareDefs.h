#pragma once
/**
 * \file
 * \brief Header file for SmartCar library hardware related definitions.
 */

#ifndef NO_PIN
#define NO_PIN 255          ///< Pin number when pin is not defined
#endif

// ------------------------------------
// SmartCar Physical Constants
const uint8_t WHEEL_DIAM = 65;      ///< Wheel diameter in mm
const uint8_t WHEEL_BASE = 110;     ///< Wheel base in mm (= distance between wheel centers)
const float DIST_PER_REV = (PI * WHEEL_DIAM);  ///< Distance traveled per rev in mm (= wheel circumference)
const uint8_t MAX_PULSE_PER_SEC = 130;   ///< Maximum encoder pulses per second

// ------------------------------------
// Motor Controller - Select hardware being used
// 
#define CONTROLLER_L298    1    ///< Configure L298N as the motor controller
#define CONTROLLER_L293    0    ///< Configure L293 as the motor controller
#define CONTROLLER_M1508   0    ///< Configure M1508 as the motor controller
#define CONTROLLER_DRV8833 0    ///< Configure DRV8833 as the motor controller

#if CONTROLLER_L298 || CONTROLLER_L293
// Left Motor
const uint8_t MC_INA1_PIN = 8;    ///< Motor A Mode pin 1 - simple digital pin
const uint8_t MC_INA2_PIN = 7;    ///< Motor A Mode pin 2 - simple digital pin
const uint8_t MC_ENA_PIN = 9;     ///< Motor A output enable - PWM capable pin
// Right Motor
const uint8_t MC_INB1_PIN = 5;    ///< Motor B Mode pin 1 - simple digital pin
const uint8_t MC_INB2_PIN = 4;    ///< Motor B Mode pin 2 - simple digital pin
const uint8_t MC_ENB_PIN = 6;     ///< Motor B output enable - PWM capable pin
#endif

#if CONTROLLER_M1508 || CONTROLLER_DRV8833
// Left Motor
const uint8_t MC_INA1_PIN = 9;    ///< Motor A Mode pin 1 - PWM capable pin
const uint8_t MC_INA2_PIN = 10;   ///< Motor A Mode pin 2 - PWM capable pin
// Right Motor
const uint8_t MC_INB1_PIN = 5;    ///< Motor B Mode pin 1 - PWM capable pin
const uint8_t MC_INB2_PIN = 6;    ///< Motor B Mode pin 2 - PWM capable pin
#endif

// Default PWM values for speeds
const uint8_t MC_PWM_MIN = 40;    ///< Minimum PWM that will turn the motor
const uint8_t MC_PWM_MAX = 255;   ///< Maximum PWM
const uint8_t MC_PWM_CREEP = 80;  ///< slow movement speed PWM default value

// -----------------------------------
// Motor Encoder
//
const uint8_t EN_L_PIN = 3;    ///< Left Motor encoder interrupt pin
const uint8_t EN_R_PIN = 2;    ///< Right Motor encoder interrupt pin
const uint8_t EN_PPR_DEF = 40; ///< Encoder pulses per revolution default value

// -----------------------------------
// PID Control
//
const float DefKp = 0.9;    ///< PID proportional weighting default
const float DefKi = 0.9;    ///< PID integral weighting default
const float DefKd = 0.1;    ///< PID derivative weighting default

const uint32_t PID_PERIOD = 250;  ///< PID calculation period in ms
const uint32_t MS_PER_SEC = 1000; ///< number of ms in 1 second
const uint16_t PID_FREQ = (MS_PER_SEC / PID_PERIOD);  ///< whole divisor is better

// -----------------------------------
// Configuration EEPROM settings
const uint16_t EEPROM_ADDR = 1023;     ///< EEPROM config data ENDS at this address (ie saved below addr)
const uint8_t SIG[2] = { 0x33, 0xaa }; ///< EEPROM config signature bytes
