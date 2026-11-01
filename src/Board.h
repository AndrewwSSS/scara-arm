#pragma once
#include <Arduino.h>
#include "Driver.h"
#include "Engine.h"

// The pins, one block per board. Everything else in the sketch is shared, so a
// new board is a new block here and nothing more.

// PIN_PREFIX is how the board's own pinout names a pin, for messages: "D2" on
// the Uno, "GPIO25" on the ESP32.

#if defined(ESP32)

#define PIN_PREFIX "GPIO"

// --- ESP32-WROOM-32 DevKit, driving the CNC Shield through a level shifter ---
//
//  Socket | STEP | DIR | Limit | Notes
//  -------+------+-----+-------+--------------------------------------------
//    X    |  25  | 26  |  34   | 34-39 are input-only, with no pull-up: the
//    Y    |  27  | 33  |       | switch needs 10k to 3.3 V
//    Z    |  32  | 13  |       |
//    A    |  18  | 19  |       |
//
//  Gripper servo: 21
// Clear of the boot strapping pins (0, 2, 5, 12, 15), the flash (6-11), the
// USB serial (1, 3) and GPIO 14, which pulses during boot. The shared EN needs
// a pull-up to the drivers' logic supply so they stay off while the ESP32
// boots. Wiring rules: the migration plan's Appendix.

static constexpr uint8_t X_STEP = 25, X_DIR = 26;
static constexpr uint8_t Y_STEP = 27, Y_DIR = 33;
static constexpr uint8_t Z_STEP = 32, Z_DIR = 13;
static constexpr uint8_t A_STEP = 18, A_DIR = 19;

static constexpr Driver X_DRIVER = Driver::A4988;
static constexpr Driver Y_DRIVER = Driver::A4988;
static constexpr Driver Z_DRIVER = Driver::A4988;
static constexpr Driver A_DRIVER = Driver::A4988;

// The step engine per socket: the one line that moves a joint to another engine.
static constexpr Engine X_ENGINE = Engine::POLLED;
static constexpr Engine Y_ENGINE = Engine::POLLED;
static constexpr Engine Z_ENGINE = Engine::POLLED;
static constexpr Engine A_ENGINE = Engine::POLLED;

static constexpr uint8_t X_MAX_LIMIT = 34;

static constexpr uint8_t EN_PIN = 4;   // shared enable, active LOW

// Signal only: the servo gets its own 5-6 V supply with a common GND.
static constexpr uint8_t GRIPPER_PIN = 21;

// false when no servo is connected: the gripper is then never attached, and its
// pin is left alone.
static constexpr bool GRIPPER_ENABLED = false;

#else

#define PIN_PREFIX "D"

// --- Arduino Uno with the CNC Shield v3 --------------------------------------
//
//  Socket | STEP | DIR | Limit header | Notes
//  -------+------+-----+--------------+-------------------------------------
//    X    |  D2  | D5  |     D9       | hardwired
//    Y    |  D3  | D6  |     D10      | hardwired
//    Z    |  D4  | D7  |     D11      | hardwired
//    A    |  D12 | D13 |    none      |
//
// X's switch is wired to A4 rather than the shield's limit header, and the
// gripper servo to D11, the Z limit header.

static constexpr uint8_t X_STEP =  2, X_DIR =  5;
static constexpr uint8_t Y_STEP =  3, Y_DIR =  6;
static constexpr uint8_t Z_STEP =  4, Z_DIR =  7;
static constexpr uint8_t A_STEP = 12, A_DIR = 13;

// The shield's sockets take A4988 or DRV8825 modules only.
static constexpr Driver X_DRIVER = Driver::A4988;
static constexpr Driver Y_DRIVER = Driver::A4988;
static constexpr Driver Z_DRIVER = Driver::A4988;
static constexpr Driver A_DRIVER = Driver::A4988;

// The step engine per socket: the one line that moves a joint to another engine.
static constexpr Engine X_ENGINE = Engine::POLLED;
static constexpr Engine Y_ENGINE = Engine::POLLED;
static constexpr Engine Z_ENGINE = Engine::POLLED;
static constexpr Engine A_ENGINE = Engine::POLLED;

static constexpr uint8_t X_MAX_LIMIT = A4;

static constexpr uint8_t EN_PIN = 8;   // shared enable, active LOW

// The Z limit header's signal pin, free because Z has no switch. Signal only:
// the servo gets its own 5-6 V supply with a common GND.
static constexpr uint8_t GRIPPER_PIN = 11;

// false when no servo is connected: the gripper is then never attached, and its
// pin is left alone.
static constexpr bool GRIPPER_ENABLED = false;

#endif
