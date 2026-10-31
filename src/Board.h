#pragma once
#include <Arduino.h>

// The pins, one block per board. Everything else in the sketch is shared, so a
// new board is a new block here and nothing more.

#if defined(ESP32)

// --- ESP32-WROOM-32 DevKit, driving the CNC Shield through a level shifter ---
//
//  Socket | STEP | DIR | Limit | Notes
//  -------+------+-----+-------+--------------------------------------------
//    X    |  25  | 26  |  34   | 34-39 are input-only, with no pull-up: the
//    Y    |  27  | 33  |       | switch needs 10k to 3.3 V
//    Z    |  32  | 13  |       |
//    A    |  18  | 19  |       |
//
// Clear of the boot strapping pins (0, 2, 5, 12, 15), the flash (6-11), the
// USB serial (1, 3) and GPIO 14, which pulses during boot. The shared EN needs
// a pull-up to the drivers' logic supply so they stay off while the ESP32
// boots. Wiring rules: the migration plan's Appendix.

static constexpr uint8_t X_STEP = 25, X_DIR = 26;
static constexpr uint8_t Y_STEP = 27, Y_DIR = 33;
static constexpr uint8_t Z_STEP = 32, Z_DIR = 13;
static constexpr uint8_t A_STEP = 18, A_DIR = 19;

static constexpr uint8_t X_MAX_LIMIT = 34;

static constexpr uint8_t EN_PIN = 4;   // shared enable, active LOW

#else

// --- Arduino Uno with the CNC Shield v3 --------------------------------------
//
//  Socket | STEP | DIR | Limit header | Notes
//  -------+------+-----+--------------+-------------------------------------
//    X    |  D2  | D5  |     D9       | hardwired
//    Y    |  D3  | D6  |     D10      | hardwired
//    Z    |  D4  | D7  |     D11      | hardwired
//    A    |  D12 | D13 |    none      |
//
// X's switch is wired to A4 rather than the shield's limit header.

static constexpr uint8_t X_STEP =  2, X_DIR =  5;
static constexpr uint8_t Y_STEP =  3, Y_DIR =  6;
static constexpr uint8_t Z_STEP =  4, Z_DIR =  7;
static constexpr uint8_t A_STEP = 12, A_DIR = 13;

static constexpr uint8_t X_MAX_LIMIT = A4;

static constexpr uint8_t EN_PIN = 8;   // shared enable, active LOW

#endif
