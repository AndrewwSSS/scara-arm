#pragma once

// Just enough of the Arduino core for src/SettingsStore.h and src/Driver.h to
// compile and run on the host under `pio test -e native`: F() and the integer
// types. Only the native env puts this directory on the include path; boards
// build against their real core.

#include <stdint.h>
#include <stddef.h>
#include <string.h>

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
