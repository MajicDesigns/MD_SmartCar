#pragma once
/**
 * \file
 * \brief Header file for SmartCar library hardware related definitions.
 */

#ifndef NO_PIN
#define NO_PIN 255          ///< Pin number when pin is not defined
#endif

/**
\page pageHardwareMap Hardware Allocation Map 
Hardware Functional Allocation for Arduino Nano

The library code is independent of processor used, however ther target
platform is an Arduino Nano.

| Pin | Description
|-----|---------------
| D0* | Hardware Serial Tx
| D1* | Hardware Serial Rx
| D2  | R Motor Encoder Pulse Interrupt
| D3  | L Motor Encoder Pulse Interrupt
| D4  | R Motor Controller PWM (L29x type controller)
| D5  | R Motor Controller InB1
| D6  | R Motor Controller InB2
| D7  | L Motor Controller PWM (L29x type controller)/R Sonar Sensor
| D8  | L Motor Controller InA1
| D9  | L Motor Controller InA2
| D10 | -
| D11*| Hardware SPI MOSI
| D12*| Hardware SPI MISO
| D13*| Hardware SPI SCK
| A0  | Software Serial Rx
| A1  | Software Serial Tx
| A2  | L Sonar Sensor
| A3  | Mid Sonar Sensor
| A4* | Hardware I2C SDA
| A5* | Hardware I2C SCL
| A6  | -
| A7  | -
 
 (*) denotes shared or communications bus pins 
*/

 // ------------------------------------
 // SmartCar Physical Constants
const uint16_t PPR_DEF = 40;        ///< Encoder pulses per revolution default value
const uint16_t DIA_WHEEL_DEF = 65;  ///< Wheel diameter in mm
const uint16_t LEN_BASE_DEF = 110;  ///< Wheel base in mm (= distance between wheel centers)
const uint16_t PPS_MAX_DEF = 120;   ///< Maximum encoder pulses per second

// ------------------------------------
// Bluetooth connections using Softwareserial
const uint8_t SC_BT_RX = A0;  ///< Arduino RX, connect to to BT TX pin
const uint8_t SC_BT_TX = A1;  ///< Arduino TX, connect to to BT RX pin

const uint16_t SC_BT_BAUDRATE = 9600; ///< BT serial connection speed (bps)

// ------------------------------------
// LCD module connections using I2C hardware connection
const uint8_t SC_LCD_ROWS = 2;  ///< LCD module number of rows (lines down)
const uint8_t SC_LCD_COLS = 16; ///< LCD module number of columns (characters across)

// ------------------------------------
// Motor Controller - Select hardware being used
// 
#ifndef CONTROLLER_L29x
#define CONTROLLER_L29x    0    ///< Configure L298, L293 as the motor controller
#endif
#ifndef CONTROLLER_MX1508
#define CONTROLLER_MX1508  1    ///< Configure MX1508, DRV8833 as the motor controller
#endif

#if CONTROLLER_L29x
// Left Motor
const uint8_t MC_INA1_PIN = 8;    ///< Motor A Mode pin 1 - simple digital pin
const uint8_t MC_INA2_PIN = 7;    ///< Motor A Mode pin 2 - simple digital pin
const uint8_t MC_ENA_PIN = 9;     ///< Motor A output enable - PWM capable pin
// Right Motor
const uint8_t MC_INB1_PIN = 5;    ///< Motor B Mode pin 1 - simple digital pin
const uint8_t MC_INB2_PIN = 4;    ///< Motor B Mode pin 2 - simple digital pin
const uint8_t MC_ENB_PIN = 6;     ///< Motor B output enable - PWM capable pin
#endif

#if CONTROLLER_MX1508
// Left Motor
const uint8_t MC_INA1_PIN = 9;    ///< Motor A Mode pin 1 - PWM capable pin
const uint8_t MC_INA2_PIN = 10;   ///< Motor A Mode pin 2 - PWM capable pin
// Right Motor
const uint8_t MC_INB1_PIN = 5;    ///< Motor B Mode pin 1 - PWM capable pin
const uint8_t MC_INB2_PIN = 6;    ///< Motor B Mode pin 2 - PWM capable pin
#endif

// Default PWM values for speeds
const uint8_t MC_PWM_MIN = 30;    ///< Minimum PWM that will turn the motor
const uint8_t MC_PWM_MAX = 255;   ///< Maximum PWM
const uint8_t MC_PWM_MOVE = 80;   ///< Slow move() speed PWM default value
const uint8_t MC_PWM_KICKER = 90; ///< Kicker for drive() to overcome static friction from standing start
const uint8_t MC_KICKER_ACTIVE = 150; ///< Kicker active time in milliseconds

// -----------------------------------
// Motor Encoder
//
const uint8_t EN_L_PIN = 3;    ///< Left Motor encoder interrupt pin
const uint8_t EN_R_PIN = 2;    ///< Right Motor encoder interrupt pin

// -----------------------------------
// PID Control
//
const float DefKp = 1.5;    ///< PID proportional weighting default
const float DefKi = 0.0;    ///< PID integral weighting default
const float DefKd = 0.1;    ///< PID derivative weighting default

const uint16_t PID_PERIOD = 250;  ///< PID calculation period in ms
const uint16_t MS_PER_SEC = 1000; ///< number of ms in 1 second
const float PID_FREQ = ((float)MS_PER_SEC / (float)PID_PERIOD);  ///< whole divisor is better

// -----------------------------------
// Configuration EEPROM settings
const uint16_t EEPROM_ADDR = 1023;     ///< EEPROM config data ENDS at this address (ie saved below addr)
const uint8_t SIG[2] = { 0x33, 0xaa }; ///< EEPROM config signature bytes
