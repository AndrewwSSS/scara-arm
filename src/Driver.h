#pragma once
#include <Arduino.h>

// The driver on a socket. Step+dir drivers differ only in timing: the STEP
// pulse width, set per axis from this, and the DIR setup before a reversal,
// which MotorControl gives every driver at 5 us (MC_DIR_SETUP_US) - what the
// slow ones need, and negligible for the rest since it is paid per reversal.
enum class Driver : uint8_t {
    A4988,    // and DRV8825: 2 us pulses
    TB6600    // and DM542, opto-isolated: 5 us pulses
};

// Minimum STEP high time, per the driver's datasheet. main.cpp checks these
// against MotorControl's own constants at compile time.
constexpr uint8_t pulseUs(Driver driver) {
    return (driver == Driver::TB6600) ? 5 : 2;
}

inline const __FlashStringHelper* driverName(Driver driver) {
    return (driver == Driver::TB6600) ? F("TB6600") : F("A4988");
}
