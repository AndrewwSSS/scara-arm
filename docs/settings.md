# Settings storage

[`src/SettingsStore.h`](../src/SettingsStore.h) keeps one settings struct in
EEPROM across power cycles, and never hands back data it cannot verify.

```cpp
struct Settings { uint16_t interval; long minStep, maxStep; };
const Settings DEFAULTS = { 400, 0, 3200 };

SettingsStore<Settings> store(0xB16D, 1);   // magic, layout version
Settings settings;

store.load(settings, DEFAULTS);   // in setup(): the saved values, or the defaults
store.save(settings);             // on the "save" command
```

## Record format

The struct is stored with a header and a checksum:

```
magic (2) | version (1) | size (2) | the struct | CRC-16 (2)
```

The checksum is CRC-16/CCITT-FALSE and also covers the header, so a flipped bit
there is caught too.

## Loading

`load()` returns the saved values only when every check passes. Otherwise it
copies the defaults, and `source()` says why:

| Check fails | Meaning | `source()` |
| --- | --- | --- |
| magic | something else is stored there, or nothing ever was (a blank chip) | `NOTHING_SAVED` |
| version | the struct's layout changed | `OLD_LAYOUT` |
| size | the struct changed size without a version bump | `OLD_LAYOUT` |
| CRC | the record is damaged: power lost mid-save, or a worn cell | `DAMAGED` |

## Rules for the struct

- **Bump the version whenever the struct changes**, so an old save is ignored
  instead of being read into the wrong fields. In `main.cpp` this is
  `SETTINGS_VERSION`.
- **Keep it plain data:** numbers, bools and fixed arrays. It is stored as raw
  bytes, so no pointers and no `String`.
- **Pick one magic number per project**, so the record is not mistaken for other
  data in EEPROM.

## Platforms

- **AVR:** `EEPROM.put()` rewrites only the bytes that changed, so saving the
  same values again causes no wear.
- **ESP32:** the EEPROM library stores the data in flash. It is sized on first
  use, only ever grown, and `save()` commits it before returning. `save()`
  returns `false` if that commit fails.

Tests: `pio test -e native` (`test/test_settings_store`).
