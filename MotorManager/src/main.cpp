#include <Arduino.h>
#include <CommandDispatcher.h>
#include <StepperManager.h>
#include <StepDirStepper.h>
#include <Endstop.h>
#include <Homing.h>
#include <LinearRamp.h>


// --- Arduino CNC Shield v3 pin map ----------------------------------------
//
//  Socket | STEP | DIR | Limit header | Notes
//  -------+------+-----+--------------+-------------------------------------
//    X    |  D2  | D5  |     D9       | hardwired
//    Y    |  D3  | D6  |     D10      | hardwired
//    Z    |  D4  | D7  |     D11      | hardwired
//    A    |  D12 | D13 |    none      |

// Limit switches, one per pin, wired COM -> GND and NC -> pin. Declared here
// because the axes borrow them.
Endstop xMax(A4, Endstop::NORMALLY_CLOSED, Endstop::MAX);

// Everything an axis needs. min / max are the switches at each end of travel,
// nullptr where there is none; homing is only read when the axis has a switch
// on the side it names.
struct AxisPins {
    const char*  name;
    uint8_t      step;
    uint8_t      dir;
    Endstop*     min;
    Endstop*     max;
    HomingConfig homing;
};

static constexpr HomingConfig NO_HOMING = { Endstop::MIN, 0, 0, 0, 0 };

//                                  name STEP DIR  min      max     homing: towards       rest  maxSteps backoff slowUs
static constexpr AxisPins CNC_X = { "X",  2,  5, nullptr, &xMax,  { Endstop::MAX,         0,    103500,  40,     4000 } };
static constexpr AxisPins CNC_Y = { "Y",  3,  6, nullptr, nullptr, NO_HOMING };
static constexpr AxisPins CNC_Z = { "Z",  4,  7, nullptr, nullptr, NO_HOMING };
static constexpr AxisPins CNC_A = { "A", 12, 13, nullptr, nullptr, NO_HOMING };

static const AxisPins AXES[] = { CNC_Z, CNC_Y, CNC_X, CNC_A };
static constexpr uint8_t AXIS_COUNT = sizeof(AXES) / sizeof(AXES[0]);

// Every socket shares this pin, so the fleet owns it rather than the axes: it
// stays energised until every axis has gone quiet.
static constexpr uint8_t EN_PIN = 8;   // shared enable, active LOW

static constexpr uint16_t FULL_STEPS_PER_REV = 200;
static constexpr uint8_t  MICROSTEPS         = 16;
static constexpr uint16_t STEPS_PER_REV      = FULL_STEPS_PER_REV * MICROSTEPS;

static constexpr unsigned long STEP_INTERVAL_US = 200;

// Steps moved by a bare 'f' / 'b' command.
static constexpr long DEFAULT_JOG_STEPS = 100;

static constexpr bool HOME_ON_START = true;

// Release the coils once the whole fleet has been at rest this long
static constexpr unsigned long IDLE_RELEASE_MS = 200;

// The start interval has to be slow enough that the axis can pull in from rest,
// and the ramp long enough that it reaches STEP_INTERVAL_US comfortably.
static constexpr unsigned long RAMP_START_US = 1000;
static constexpr uint16_t      RAMP_STEPS    = 1000;

LinearRamp ramp(RAMP_START_US, RAMP_STEPS);

StepperManager fleet(AXIS_COUNT, EN_PIN);

static Stepper* makeAxis(const AxisPins& axis) {
    // The EN pin is the fleet's, so the axes are built without one.
    Stepper* stepper = new StepDirStepper(axis.step, axis.dir, STEPS_PER_REV,
                                          StepDirStepper::NO_ENABLE_PIN, STEP_INTERVAL_US);
    if (stepper == nullptr) return nullptr;

    stepper->setEndstop(axis.min);   // nullptr is ignored
    stepper->setEndstop(axis.max);
    return stepper;
}

static bool hasEndstops(const AxisPins& axis) {
    return axis.min != nullptr || axis.max != nullptr;
}

static Endstop* homingEndstop(const AxisPins& axis) {
    return (axis.homing.towards == Endstop::MIN) ? axis.min : axis.max;
}
static constexpr uint8_t ALL_AXES = 0xFF;
static uint8_t selected = (AXIS_COUNT > 0) ? 0 : ALL_AXES;

static uint32_t moveStartedAt = 0;

static bool isSelected(uint8_t id) {
    return selected == ALL_AXES || selected == id;
}

static const char* selectionName() {
    return (selected == ALL_AXES) ? "*" : AXES[selected].name;
}

static void printStatus() {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        Serial.println(String(AXES[i].name) +
                       (isSelected(i) ? " <" : "  ") +
                       " step=" + fleet.getStep(i) +
                       " target=" + fleet.getTargetStep(i) +
                       " state=" + (fleet.isHold(i) ? "HOLD" : "MOVING") +
                       " interval=" + fleet.getStepInterval(i) + "us" +
                       " now=" + fleet.getCurrentInterval(i) + "us" +
                       " ramp=" + (fleet.getProfile(i) ? String(ramp.getRampSteps()) : String("off")) +
                       " debug=" + (fleet.getDebug(i) ? "on" : "off") +
                       (homingEndstop(AXES[i]) ? String(" homing=") + homingResultName(fleet.getHomingResult(i))
                                               : String("")));
    }

    unsigned long idle = fleet.getIdleRelease();
    Serial.println(String("EN D") + fleet.getEnablePin() +
                   " " + (fleet.isPowered() ? "energised" : "released") +
                   ", release=" + (idle ? String(idle) + "ms" : String("off")));
}

static void printHelp() {
    Serial.println(F("commands (words separated by spaces):"));
    Serial.println(F("  x|y|z|a  select that axis for the commands below"));
    Serial.println(F("  *        select every axis at once"));
    Serial.println(F("  f [n]    jog n steps forward  (DIR high), default 100"));
    Serial.println(F("  b [n]    jog n steps backward (DIR low),  default 100"));
    Serial.println(F("  g <n>    go to absolute step n (may be negative)"));
    Serial.println(F("  o        re-declare the current position as step 0"));
    Serial.println(F("  h        halt: cancel the move, hold here"));
    Serial.println(F("  home     home against the endstop (blocking; '*' = every axis with one)"));
    Serial.println(F("  v [n]    step interval in us, lower is faster (bare v resets)"));
    Serial.println(F("  r [n]    ramp over n steps (r 0 = none, bare r toggles)"));
    Serial.println(F("  p [n]    release the coils after n ms fleet idle (p 0 = never)"));
    Serial.println(F("  l        read each endstop pin right now (diagnostic)"));
    Serial.println(F("  s        status | d debug | ? help"));
}

static void printEndstop(const char* axis, const Endstop& e) {
    bool high = (digitalRead(e.getPin()) == HIGH);
    Serial.println(String(axis) + " " + e.sideName() + " pin=" + e.getPin() +
                   (e.getWiring() == Endstop::NORMALLY_CLOSED ? " NC" : " NO") +
                   " reads=" + (high ? "HIGH" : "LOW") +
                   " -> code says " + (e.readRaw() ? "TRIGGERED" : "clear"));
}

// Blocking: the console does not answer until every selected axis is done.
// Axes without a switch on their homing side are skipped, so '*' then 'home'
// homes everything that can be homed.
static void homeSelected() {
    bool any = false;

    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!isSelected(i) || !homingEndstop(AXES[i])) continue;

        any = true;
        HomingResult r = fleet.home(i, AXES[i].homing);
        Serial.println(String(AXES[i].name) + " homing: " + homingResultName(r));
    }

    if (!any) Serial.println(String(selectionName()) + " has no endstop to home against");
}

static void jog(long steps) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.moveSteps(i, steps);
    }

    moveStartedAt = millis();
    Serial.println(String("-> ") + selectionName() + " " +
                   (steps >= 0 ? "+" : "") + steps + " steps");
}
static uint8_t axisFromLetter(char c) {
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (tolower(AXES[i].name[0]) == tolower(c)) return i;
    }
    return ALL_AXES;
}

// --- Console commands -------------------------------------------------------
//
// Each one is registered in registerCommands() with the arguments it takes;
// CommandDispatcher checks them before the handler runs, so a handler never
// sees a missing number or a word where a number belongs.

CommandDispatcher<20> commands(Serial);
LineReader<24>        console(Serial);

static void cmdHome(const CommandArgs&) {
    homeSelected();
    printStatus();
}

static void cmdForward(const CommandArgs& args) {
    long n = args.getLong(0);
    jog(n > 0 ? n : DEFAULT_JOG_STEPS);
}

static void cmdBackward(const CommandArgs& args) {
    long n = args.getLong(0);
    jog(-(n > 0 ? n : DEFAULT_JOG_STEPS));
}

static void cmdGo(const CommandArgs& args) {
    long n = args.getLong(0);
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.moveToStep(i, n);
    }
    moveStartedAt = millis();
    Serial.println(String("-> ") + selectionName() + " absolute step " + n);
}

static void cmdOrigin(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.setOrigin(i);
    }
    Serial.println(String(selectionName()) + " origin reset, step 0 is here");
}

static void cmdHalt(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.stopHere(i);
    }
    Serial.println(String(selectionName()) + " halted");
}

// Diagnostic only: the raw pin, and what the configured wiring mode makes of
// it. Moves nothing.
static void cmdEndstops(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!hasEndstops(AXES[i])) {
            Serial.println(String(AXES[i].name) + " no endstop");
            continue;
        }

        if (AXES[i].min) printEndstop(AXES[i].name, *AXES[i].min);
        if (AXES[i].max) printEndstop(AXES[i].name, *AXES[i].max);
    }
}

// Takes effect on the next pulse, mid-move included - there is no ramp, so a
// big jump under load is where a stepper loses steps.
static void cmdInterval(const CommandArgs& args) {
    long n = args.getLong(0);
    unsigned long us = (n > 0) ? (unsigned long)n : STEP_INTERVAL_US;

    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.setStepInterval(i, us);
    }

    Serial.println(String("-> ") + selectionName() + " interval " + us + "us");
}

// The ramp length is the shared profile's, so a number here moves every axis
// using it; what the selection controls is whether the axis is on the profile
// at all. Bare 'r' toggles, 'r 0' turns it off, 'r <n>' sets and turns it on.
static void cmdRamp(const CommandArgs& args) {
    long n = args.getLong(0);
    if (n > 0) ramp.setRampSteps((uint16_t)n);

    bool off = args.has(0) && n == 0;
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!isSelected(i)) continue;

        bool on = off ? false
                      : (args.has(0) ? true : (fleet.getProfile(i) == nullptr));
        fleet.setProfile(i, on ? &ramp : nullptr);
    }

    printStatus();
}

// One EN line for the shield, so this is a fleet-wide setting and ignores the
// axis selection. A bare 'p' toggles, which is the quick way to feel the
// difference on the shaft; 'p <n>' sets it outright.
static void cmdIdleRelease(const CommandArgs& args) {
    long n = args.getLong(0);
    unsigned long idle = (n > 0) ? (unsigned long)n : 0;
    if (!args.has(0)) {
        idle = fleet.getIdleRelease() ? 0 : IDLE_RELEASE_MS;
    }

    fleet.setIdleRelease(idle);
    // Turning it off does not re-energise on its own, and a released fleet
    // would report positions it is not holding.
    if (idle == 0) fleet.enable();

    printStatus();
}

static void cmdDebug(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.toggleDebug(i);
    }
    printStatus();
}

static void cmdStatus(const CommandArgs&) {
    printStatus();
}

static void cmdHelp(const CommandArgs&) {
    printHelp();
}

static void cmdSelectAll(const CommandArgs&) {
    selected = ALL_AXES;
    Serial.println(F("selected: every axis"));
}

// Registered once per axis letter; the name typed says which.
static void cmdSelectAxis(const CommandArgs& args) {
    selected = axisFromLetter(args.name()[0]);
    Serial.println(String("selected: ") + AXES[selected].name);
}

static void onUnknownCommand(const char* name) {
    Serial.print(F("unknown command: "));
    Serial.println(name);
    printHelp();
}

static void registerCommands() {
    commands.add(F("home"), cmdHome);
    commands.add(F("f"), F("[n]"), cmdForward);
    commands.add(F("b"), F("[n]"), cmdBackward);
    commands.add(F("g"), F("n"), cmdGo);
    commands.add(F("o"), cmdOrigin);
    commands.add(F("h"), cmdHalt);
    commands.add(F("l"), cmdEndstops);
    commands.add(F("v"), F("[n]"), cmdInterval);
    commands.add(F("r"), F("[n]"), cmdRamp);
    commands.add(F("p"), F("[n]"), cmdIdleRelease);
    commands.add(F("d"), cmdDebug);
    commands.add(F("s"), cmdStatus);
    commands.add(F("?"), cmdHelp);
    commands.add(F("help"), cmdHelp);
    commands.add(F("*"), cmdSelectAll);

    // One entry per axis letter. F() strings have to be literal, so the
    // letters are spelled out here; keep them in step with AXES.
    commands.add(F("x"), cmdSelectAxis);
    commands.add(F("y"), cmdSelectAxis);
    commands.add(F("z"), cmdSelectAxis);
    commands.add(F("a"), cmdSelectAxis);

    commands.onUnknown(onUnknownCommand);
}

static void onAxisReached(uint8_t id) {
    Serial.println(String(AXES[id].name) + " reached step " + fleet.getStep(id) +
                   " in " + (millis() - moveStartedAt) + " ms");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { /* Uno returns immediately; harmless on native-USB boards */ }

    Serial.println(F("\n=== CNC Shield v3 - step jog ==="));

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (fleet.addMotor(makeAxis(AXES[i])) == StepperManager::INVALID_ID) {
            Serial.println(String("axis ") + AXES[i].name + " not registered, out of memory");
            break;
        }

        Serial.println(String("socket=") + AXES[i].name +
                       " STEP=D" + AXES[i].step + " DIR=D" + AXES[i].dir +
                       " EN=D" + EN_PIN);
    }

    Serial.println(String("steps/rev=") + STEPS_PER_REV +
                   " (" + FULL_STEPS_PER_REV + " full x 1/" + MICROSTEPS + ")");

    registerCommands();

    fleet.setIdleRelease(IDLE_RELEASE_MS);
    fleet.setProfiles(RAMP_STEPS ? &ramp : nullptr);
    fleet.onReached(onAxisReached);
    fleet.begin();

    if (HOME_ON_START) {
        uint8_t keep = selected;
        selected = ALL_AXES;
        homeSelected();
        selected = keep;
    }

    printStatus();
    printHelp();
}

void loop() {
    fleet.update();
    console.poll(commands);
}
