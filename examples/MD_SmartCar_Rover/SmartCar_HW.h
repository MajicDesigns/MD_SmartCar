#pragma once

// SmartCar application hardware, timing and other definitions

// ------------------------------------
// SmartCar Physical Constants
const uint16_t PPR = 40;        ///< Encoder pulses per revolution
const uint16_t PPS_MAX = 175;   ///< Maximum encoder pulses per second (@ PWM=255)
const uint16_t DIA_WHEEL = 65;  ///< Wheel diameter in mm
const uint16_t LEN_BASE = 110;  ///< Wheel base in mm (= distance between wheel centers)

const uint8_t SPEED_MAX = 40;   ///< maximum speed for vehicle (% full speed)

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
const uint16_t BUMPER_POLL_PERIOD = 20;   // in ms
const uint8_t PIN_L_BUMPER = 9;    ///< Front Bumper Left Sensor
const uint8_t PIN_R_BUMPER = 10;   ///< Front Bumper Right Sensor

// ------------------------------------
// Sonar sensors connections (NewPing library - single pin)
const uint16_t SONAR_POLL_PERIOD = 70;   // in ms
const uint8_t PIN_L_SONAR = A2;  ///< Sonar (ping sensor) Left side pin
const uint8_t PIN_M_SONAR = A3;  ///< Sonar (ping sensor) Middle side pin
const uint8_t PIN_R_SONAR = 8;   ///< Sonar (ping sensor) Right side pin

// Define SONAR distance points in cm for decision making
const uint8_t DIST_MAX = 200;       ///< Maximum distance to ping
const uint8_t DIST_IMPACT = 20;     ///< impact imminent
const uint8_t DIST_CLOSE = 40;      ///< really close 
const uint8_t DIST_OBSTACLE = 100;  ///< far obstacle detected

// ------------------------------------
// Light Sensors
const uint16_t LIGHT_POLL_PERIOD = 250;   // in ms
const uint8_t PIN_L_LIGHT = A6;  ///< Sonar (ping sensor) Left side pin
const uint8_t PIN_R_LIGHT = A7;  ///< Sonar (ping sensor) Right side pin

// ------------------------------------
// Miscellaneous values
const uint32_t TELEMETRY_PERIOD = 500;    ///< telemetry packet send period in ms
const uint32_t ESCAPE_PAUSE_TIME = 500;   ///< time for pauses during ESCAPE in ms
const uint32_t AVOID_ACTIVE_TIME = 1000;  ///< time for AVOID to be acive in ms
const uint32_t SEEK_ACTIVE_TIME = 1000;   ///< time for SEEK to be active in ms
const uint32_t FOLLOW_ACTIVE_TIME = 1000; ///< time for WALLFOLLOWER to be active in ms

const uint8_t FLOAT_DECIMALS = 2;         ///< decimals shown in float values