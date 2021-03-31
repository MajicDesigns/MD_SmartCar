#pragma once

// SmartCar Application hardware definitions

// ------------------------------------
// SmartCar Physical Constants
const uint16_t PPR = 40;        ///< Encoder pulses per revolution
const uint16_t PPS_MAX = 175;   ///< Maximum encoder pulses per second (@ PWM=255)
const uint16_t DIA_WHEEL = 65;  ///< Wheel diameter in mm
const uint16_t LEN_BASE = 110;  ///< Wheel base in mm (= distance between wheel centers)

// ------------------------------------
// Bluetooth connections using SoftwareSerial
const uint8_t PIN_BT_RX = A0;  ///< Arduino RX, connect to to BT TX pin
const uint8_t PIN_BT_TX = A1;  ///< Arduino TX, connect to to BT RX pin

const uint16_t BT_BAUDRATE = 9600; ///< BT serial connection speed (bps)

// ------------------------------------
// LCD module connections using I2C hardware connection (if installed)
const uint8_t LCD_ROWS = 2;  ///< LCD module number of rows (lines down)
const uint8_t LCD_COLS = 16; ///< LCD module number of columns (characters across)

// ------------------------------------
// Bumper Switch
const uint8_t PIN_L_BUMPER = 9;    ///< Front Bumper Left Sensor
const uint8_t PIN_R_BUMPER = 10;   ///< Front Bumper Right Sensor

// ------------------------------------
// Sonar sensors connections (NewPing library - single pin)
const uint8_t PIN_L_SONAR = A2;  ///< Sonar (ping sensor) Left side pin
const uint8_t PIN_M_SONAR = A3;  ///< Sonar (ping sensor) Middle side pin
const uint8_t PIN_R_SONAR = 8;   ///< Sonar (ping sensor) Right side pin

// ------------------------------------
// Light Sensors
const uint8_t PIN_L_LIGHT = A6;  ///< Sonar (ping sensor) Left side pin
const uint8_t PIN_M_LIGHT = A7;  ///< Sonar (ping sensor) Middle side pin

// Define SONAR distance points in cm for decision making
const uint8_t DIST_IMPACT = 20;       // impact imminent
const uint8_t DIST_CLOSE = 40;        // really close 
const uint8_t DIST_OBSTACLE = 100;    // far obstacle detected

const uint8_t SPEED_MAX = 40;         // maximum speed for vehicle (% full speed)

