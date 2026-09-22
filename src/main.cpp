#include <Arduino.h>
#include <A4988Stepper.h>


// --- Arduino CNC Shield v3 pin map ----------------------------------------
//
//  Socket | STEP | DIR | Limit header | Notes
//  -------+------+-----+--------------+-------------------------------------
//    X    |  D2  | D5  |     D9       | hardwired
//    Y    |  D3  | D6  |     D10      | hardwired
//    Z    |  D4  | D7  |     D11      | hardwired
//    A    |  D12 | D13 |    none      | needs two jumper wires (see below)

struct AxisPins {
    const char* name;
    uint8_t step;
    uint8_t dir;
    uint8_t limit;
};

static constexpr uint8_t NO_LIMIT = 0xFF;

static constexpr AxisPins CNC_X = { "X",  2,  5, 9       };
static constexpr AxisPins CNC_Y = { "Y",  3,  6, 10      };
static constexpr AxisPins CNC_Z = { "Z",  4,  7, 11      };
static constexpr AxisPins CNC_A = { "A", 12, 13, NO_LIMIT };

static constexpr AxisPins AXIS = CNC_Z;

static constexpr uint8_t EN_PIN = 8;   // shared enable, active LOW

static constexpr uint16_t FULL_STEPS_PER_REV = 200;
static constexpr uint8_t  MICROSTEPS         = 16;
static constexpr uint16_t STEPS_PER_REV      = FULL_STEPS_PER_REV * MICROSTEPS;

static constexpr unsigned long STEP_INTERVAL_US = 1000;

// Steps moved by a bare 'f' / 'b' command.
static constexpr long DEFAULT_JOG_STEPS = 100;

A4988Stepper aAxis(AXIS.step, AXIS.dir, STEPS_PER_REV,
                   EN_PIN, STEP_INTERVAL_US);

static uint32_t moveStartedAt = 0;

static void printStatus() {
    Serial.println(String("step=") + aAxis.getStep() +
                   " target=" + aAxis.getTargetStep() +
                   " state=" + (aAxis.getState() == Stepper::MOVING ? "MOVING" : "HOLD") +
                   " debug=" + (aAxis.getDebug() ? "on" : "off"));
}

static void printHelp() {
    Serial.println(F("commands:"));
    Serial.println(F("  f[n]  jog n steps forward  (DIR high), default 10"));
    Serial.println(F("  b[n]  jog n steps backward (DIR low),  default 10"));
    Serial.println(F("  g<n>  go to absolute step n (may be negative)"));
    Serial.println(F("  o     re-declare the current position as step 0"));
    Serial.println(F("  s     status | d debug | ? help"));
}

static void jog(long steps) {
    aAxis.moveSteps(steps);
    moveStartedAt = millis();
    Serial.println(String("-> ") + (steps >= 0 ? "+" : "") + steps +
                   " steps, target " + aAxis.getTargetStep());
}

static void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    char c = cmd.charAt(0);
    String rest = cmd.substring(1);
    rest.trim();
    // toInt() yields 0 for an empty or non-numeric tail, which is how a bare
    // 'f' / 'b' falls back to the default jog size.
    long n = rest.toInt();

    switch (c) {
        case 'f':
            jog(n > 0 ? n : DEFAULT_JOG_STEPS);
            break;
        case 'b':
            jog(-(n > 0 ? n : DEFAULT_JOG_STEPS));
            break;
        case 'g':
            aAxis.moveToStep(n);
            moveStartedAt = millis();
            Serial.println(String("-> absolute step ") + n);
            break;
        case 'o':
            aAxis.setOrigin(0);
            Serial.println(F("origin reset, step 0 is here"));
            break;
        case 'd':
            aAxis.toggleDebug();
            Serial.println(String("debug ") + (aAxis.getDebug() ? "on" : "off"));
            break;
        case 's': printStatus(); break;
        default:  printHelp();   break;
    }
}

// Line editing and echo happen on the host (see tools/monitor.py), so the
// sketch never echoes - doing it at both ends would double every character.
// It only buffers what arrives and dispatches on newline.
//
// Reading byte by byte rather than with readStringUntil() keeps this
// non-blocking: every millisecond spent waiting on serial is a step pulse not
// emitted.
//
// Control bytes and escape sequences are still dropped, as a guard against a
// terminal that does not edit locally. Such a terminal sends a raw backspace
// or an arrow key's "ESC [ A" straight into the command, where trim() would
// not remove it and "b<BS>f" would still parse as 'b' - silently the wrong
// command.
static char    inputLine[24];
static uint8_t inputLen = 0;
static bool    inEscape = false;

static void readSerial() {
    while (Serial.available()) {
        char c = Serial.read();

        if (inEscape) {
            if (c == '[' || c == 'O' || (c >= '0' && c <= '?')) continue;
            inEscape = false;
            continue;
        }
        if (c == 0x1B) {
            inEscape = true;
            continue;
        }

        if (c == '\r' || c == '\n') {
            if (inputLen == 0) continue;   // blank line, or the LF of a CRLF pair
            inputLine[inputLen] = '\0';
            handleCommand(String(inputLine));
            inputLen = 0;
            continue;
        }

        if (c == 0x08 || c == 0x7F) {      // stray backspace from a raw terminal
            if (inputLen > 0) inputLen--;
            continue;
        }

        if (c < 0x20) continue;            // swallow other control bytes

        if (inputLen < sizeof(inputLine) - 1) {
            inputLine[inputLen++] = c;
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { /* Uno returns immediately; harmless on native-USB boards */ }

    Serial.println(F("\n=== CNC Shield v3 - step jog ==="));
    Serial.println(String("socket=") + AXIS.name +
                   " STEP=D" + AXIS.step + " DIR=D" + AXIS.dir + " EN=D" + EN_PIN);
    Serial.println(String("steps/rev=") + STEPS_PER_REV +
                   " (" + FULL_STEPS_PER_REV + " full x 1/" + MICROSTEPS + ")");

    aAxis.begin();
    //aAxis.disable();

    printStatus();
    printHelp();
    Serial.println(F("try: f  then  b   (100 steps out and back)"));
}

void loop() {
    // Emits at most one step pulse per call and returns immediately, so the
    // serial console stays responsive while the motor is turning.
    aAxis.update();

    if (aAxis.reachedTarget()) {
        Serial.println(String("reached step ") + aAxis.getStep() +
                       " in " + (millis() - moveStartedAt) + " ms");
    }

    readSerial();
}
