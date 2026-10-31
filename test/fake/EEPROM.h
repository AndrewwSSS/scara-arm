#pragma once

// A 1 KB EEPROM in RAM with the AVR core's put()/get(), plus what the tests
// need to look inside: counting writes, and spoiling a byte as a power cut or
// a worn cell would.

#include <stdint.h>
#include <string.h>

class FakeEEPROM {
public:
    static const int SIZE = 1024;

    FakeEEPROM() { wipe(); }

    template <typename T>
    T& get(int address, T& value) {
        memcpy(&value, bytes + address, sizeof(T));
        return value;
    }

    // Like AVR's: a byte that already holds the value is not written again.
    template <typename T>
    const T& put(int address, const T& value) {
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
        for (size_t i = 0; i < sizeof(T); i++) {
            if (bytes[address + i] != p[i]) {
                bytes[address + i] = p[i];
                writes++;
            }
        }
        return value;
    }

    uint16_t length() const { return SIZE; }

    // A blank chip reads 0xFF everywhere.
    void wipe() {
        memset(bytes, 0xFF, sizeof bytes);
        writes = 0;
    }

    uint8_t bytes[SIZE];
    int     writes;
};

inline FakeEEPROM EEPROM;
