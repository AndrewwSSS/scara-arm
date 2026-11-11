#pragma once
#include <Arduino.h>

// The driver on a socket. Step+dir drivers differ here only in STEP pulse
// width; DIR setup is MotorControl's MC_DIR_SETUP_US for all of them.
enum class Driver : uint8_t {
    A4988,    // and DRV8825: 2 us pulses
    TB6600    // and DM542, opto-isolated: 5 us pulses
};

// Minimum STEP high time from the datasheet; main.cpp static_asserts it
// against MotorControl's constants.
constexpr uint8_t pulseUs(Driver driver) {
    return (driver == Driver::TB6600) ? 5 : 2;
}

inline const __FlashStringHelper* driverName(Driver driver) {
    return (driver == Driver::TB6600) ? F("TB6600") : F("A4988");
}
