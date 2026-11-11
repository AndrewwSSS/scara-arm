#pragma once
#include <Arduino.h>

// How a socket's pulses are made, chosen per socket in Board.h.
enum class Engine : uint8_t {
    POLLED    // PolledEngine: timed from loop(), any board
};

inline const __FlashStringHelper* engineName(Engine engine) {
    switch (engine) {
        case Engine::POLLED: return F("polled");
    }
    return F("unknown");
}
