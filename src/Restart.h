#pragma once
#include <Arduino.h>

#if defined(__AVR__)
#include <avr/wdt.h>
#endif

// Restarts the board the way its reset button would. The ESP32 has a call for
// it. The AVR has no reset instruction, so the watchdog is armed with its
// shortest timeout and left to run out. Never returns.
inline void restartBoard() {
#if defined(ESP32)
    ESP.restart();
#elif defined(__AVR__)
    wdt_enable(WDTO_15MS);
#else
#error "restartBoard() does not know this board"
#endif
    while (true) {}
}

// On the AVR a watchdog reset leaves the watchdog running, so setup() calls
// this first: without it, a bootloader that does not switch the watchdog off
// would reset the board again and again. Nothing to do on the ESP32.
inline void clearRestart() {
#if defined(__AVR__)
    MCUSR = 0;
    wdt_disable();
#endif
}
