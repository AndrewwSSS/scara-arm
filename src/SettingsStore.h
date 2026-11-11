#pragma once
#include <Arduino.h>
#include <EEPROM.h>

// Keeps one plain-data settings struct in EEPROM, with a header and CRC, and
// falls back to defaults when the record cannot be verified. Usage and format:
// docs/settings.md.

namespace ss_detail {

// CRC-16/CCITT-FALSE: polynomial 0x1021, start 0xFFFF. Bitwise, so no table
// in flash or RAM.
inline uint16_t crc16(uint16_t crc, const uint8_t* data, size_t length) {
    while (length--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

}

template <typename T>
class SettingsStore {
public:
    // Where the values the last load() produced came from.
    enum Source : uint8_t {
        NOT_LOADED,       // load() has not run
        LOADED,           // the saved values
        NOTHING_SAVED,    // defaults: blank, erased, or something else's data
        OLD_LAYOUT,       // defaults: saved by a different version or size
        DAMAGED           // defaults: the checksum did not match
    };

    // Bytes the record takes from `address` on.
    static const size_t SIZE = 2 + 1 + 2 + sizeof(T) + 2;

    // magic tells this record apart from anything else in EEPROM; pick one per
    // project. version is the struct's layout: bump it when the struct changes.
    SettingsStore(uint16_t magic, uint8_t version, int address = 0)
        : magic(magic), version(version), address(address), source_(NOT_LOADED) {}

    // Fills `value` with the saved settings and returns true, or with
    // `defaults` and returns false.
    bool load(T& value, const T& defaults) {
        prepare();
        source_ = check(value);
        if (source_ != LOADED) value = defaults;
        return source_ == LOADED;
    }

    // False only when the ESP32 could not commit to flash.
    bool save(const T& value) {
        prepare();
        EEPROM.put(address, magic);
        EEPROM.put(address + 2, version);
        EEPROM.put(address + 3, (uint16_t)sizeof(T));
        EEPROM.put(address + 5, value);
        EEPROM.put(address + 5 + sizeof(T), crcOf(value));
        return commit();
    }

    // Makes the next load() find nothing saved. The struct's bytes stay; only
    // the magic is spoiled.
    bool erase() {
        prepare();
        EEPROM.put(address, (uint16_t)~magic);
        return commit();
    }

    Source source() const { return source_; }

    static const __FlashStringHelper* describe(Source source) {
        switch (source) {
            case LOADED:        return F("saved settings");
            case NOTHING_SAVED: return F("defaults (nothing saved)");
            case OLD_LAYOUT:    return F("defaults (saved by another version)");
            case DAMAGED:       return F("defaults (saved settings damaged)");
            default:            return F("not loaded");
        }
    }

private:
    // Reads the record into value - possibly garbage - and judges it.
    Source check(T& value) {
        uint16_t savedMagic = 0, savedSize = 0, savedCrc = 0;
        uint8_t  savedVersion = 0;

        EEPROM.get(address, savedMagic);
        if (savedMagic != magic) return NOTHING_SAVED;

        EEPROM.get(address + 2, savedVersion);
        EEPROM.get(address + 3, savedSize);
        if (savedVersion != version || savedSize != sizeof(T)) return OLD_LAYOUT;

        EEPROM.get(address + 5, value);
        EEPROM.get(address + 5 + sizeof(T), savedCrc);
        return savedCrc == crcOf(value) ? LOADED : DAMAGED;
    }

    // Over the header too, so a flipped bit there is caught as well.
    uint16_t crcOf(const T& value) const {
        uint8_t header[5] = {
            (uint8_t)magic, (uint8_t)(magic >> 8), version,
            (uint8_t)sizeof(T), (uint8_t)(sizeof(T) >> 8)
        };
        uint16_t crc = ss_detail::crc16(0xFFFF, header, sizeof header);
        return ss_detail::crc16(crc, reinterpret_cast<const uint8_t*>(&value), sizeof(T));
    }

    // The ESP32's EEPROM has to be sized before use. Only ever grown here, as
    // a smaller begin() would cut off whatever else the sketch keeps there.
    void prepare() {
#if defined(ESP32)
        if (EEPROM.length() < address + SIZE) EEPROM.begin(address + SIZE);
#endif
    }

    bool commit() {
#if defined(ESP32)
        return EEPROM.commit();
#else
        return true;
#endif
    }

    uint16_t magic;
    uint8_t  version;
    int      address;
    Source   source_;
};
