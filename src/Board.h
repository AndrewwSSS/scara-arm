#pragma once
#include <Arduino.h>
#include "Driver.h"
#include "Engine.h"

// Pin map, one block per board. Wiring: docs/wiring.md.

// PIN_PREFIX: how the board's pinout names a pin in messages ("D2", "GPIO25").

#if defined(ESP32)

#define PIN_PREFIX "GPIO"

// --- ESP32-WROOM-32 DevKit, CNC Shield through a level shifter --------------
// Avoids the strapping pins (0, 2, 5, 12, 15), flash (6-11), UART0 (1, 3) and
// GPIO14, which pulses during boot.

static constexpr uint8_t X_STEP = 25, X_DIR = 26;
static constexpr uint8_t Y_STEP = 27, Y_DIR = 33;
static constexpr uint8_t Z_STEP = 32, Z_DIR = 13;
static constexpr uint8_t A_STEP = 18, A_DIR = 19;

static constexpr Driver X_DRIVER = Driver::A4988;
static constexpr Driver Y_DRIVER = Driver::A4988;
static constexpr Driver Z_DRIVER = Driver::A4988;
static constexpr Driver A_DRIVER = Driver::A4988;

// Step engine per socket.
static constexpr Engine X_ENGINE = Engine::POLLED;
static constexpr Engine Y_ENGINE = Engine::POLLED;
static constexpr Engine Z_ENGINE = Engine::POLLED;
static constexpr Engine A_ENGINE = Engine::POLLED;

static constexpr uint8_t X_MAX_LIMIT = 34;   // input-only: needs an external pull-up

static constexpr uint8_t EN_PIN = 4;   // shared enable, active LOW

static constexpr uint8_t GRIPPER_PIN = 21;

// false when no servo is connected; its pin is then left untouched.
static constexpr bool GRIPPER_ENABLED = false;

#else

#define PIN_PREFIX "D"

// --- Arduino Uno with the CNC Shield v3 --------------------------------------

static constexpr uint8_t X_STEP =  2, X_DIR =  5;
static constexpr uint8_t Y_STEP =  3, Y_DIR =  6;
static constexpr uint8_t Z_STEP =  4, Z_DIR =  7;
static constexpr uint8_t A_STEP = 12, A_DIR = 13;

// The shield's sockets take A4988 or DRV8825 modules only.
static constexpr Driver X_DRIVER = Driver::A4988;
static constexpr Driver Y_DRIVER = Driver::A4988;
static constexpr Driver Z_DRIVER = Driver::A4988;
static constexpr Driver A_DRIVER = Driver::A4988;

// Step engine per socket.
static constexpr Engine X_ENGINE = Engine::POLLED;
static constexpr Engine Y_ENGINE = Engine::POLLED;
static constexpr Engine Z_ENGINE = Engine::POLLED;
static constexpr Engine A_ENGINE = Engine::POLLED;

static constexpr uint8_t X_MAX_LIMIT = A4;

static constexpr uint8_t EN_PIN = 8;   // shared enable, active LOW

static constexpr uint8_t GRIPPER_PIN = 11;   // the Z limit header; Z has no switch

// false when no servo is connected; its pin is then left untouched.
static constexpr bool GRIPPER_ENABLED = false;

#endif
