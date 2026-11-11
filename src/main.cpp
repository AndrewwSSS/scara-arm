#include <Arduino.h>
#include <CommandDispatcher.h>
#include <SettingsStore.h>
#include <StepperManager.h>
#include <EngineStepper.h>
#include <PolledEngine.h>
#include <Endstop.h>
#include <Homing.h>
#include <LinearRamp.h>
#include <ServoManager.h>
#include <ServoMotor.h>
#include "Board.h"
#include "Restart.h"

// Normally closed (COM -> GND, NC -> pin). File scope: the axes borrow it.
Endstop xMax(X_MAX_LIMIT, Endstop::NORMALLY_CLOSED, Endstop::MAX);

// min / max: the switch at each end of travel, or nullptr. homing is used only
// when there is a switch on the side it names.
struct AxisPins {
    const char*  name;
    uint8_t      step;
    uint8_t      dir;
    Driver       driver;
    Engine       engine;
    Endstop*     min;
    Endstop*     max;
    HomingConfig homing;
};

static constexpr HomingConfig NO_HOMING = { Endstop::MIN, 0, 0, 0, 0 };

//                                  name STEP    DIR    driver    engine    min      max      homing: towards rest maxSteps backoff slowUs
static constexpr AxisPins CNC_X = { "X", X_STEP, X_DIR, X_DRIVER, X_ENGINE, nullptr, &xMax,   { Endstop::MAX, 0, 103500, 40, 4000 } };
static constexpr AxisPins CNC_Y = { "Y", Y_STEP, Y_DIR, Y_DRIVER, Y_ENGINE, nullptr, nullptr, NO_HOMING };
static constexpr AxisPins CNC_Z = { "Z", Z_STEP, Z_DIR, Z_DRIVER, Z_ENGINE, nullptr, nullptr, NO_HOMING };
static constexpr AxisPins CNC_A = { "A", A_STEP, A_DIR, A_DRIVER, A_ENGINE, nullptr, nullptr, NO_HOMING };

static const AxisPins AXES[] = { CNC_Z, CNC_Y, CNC_X, CNC_A };
static constexpr uint8_t AXIS_COUNT = sizeof(AXES) / sizeof(AXES[0]);


static constexpr uint16_t FULL_STEPS_PER_REV = 200;
static constexpr uint8_t  MICROSTEPS         = 16;
static constexpr uint16_t STEPS_PER_REV      = FULL_STEPS_PER_REV * MICROSTEPS;

static constexpr unsigned long STEP_INTERVAL_US = 200;

// Steps moved by a bare 'f' / 'b' command.
static constexpr long DEFAULT_JOG_STEPS = 100;

// Off by default: when on, the arm moves at power-up without a command.
static constexpr bool HOME_ON_START = false;

// Coils are released once the whole fleet has been idle this long.
static constexpr unsigned long IDLE_RELEASE_MS = 200;

// Start slow enough for the motor to pull in from rest; ramp long enough to
// reach STEP_INTERVAL_US.
static constexpr unsigned long RAMP_START_US = 1000;
static constexpr uint16_t      RAMP_STEPS    = 1000;

LinearRamp ramp(RAMP_START_US, RAMP_STEPS);

// The sockets share one EN pin, so the fleet owns it, not the axes.
StepperManager fleet(AXIS_COUNT, EN_PIN);

static constexpr uint16_t GRIPPER_START_ANGLE = 90;

ServoManager gripper(1);
static constexpr uint8_t GRIPPER = 0;   // its id in the gripper fleet

// --- Settings kept across power cycles ---------------------------------------
// Bump SETTINGS_VERSION whenever these structs change. See docs/settings.md.

struct AxisSettings {
    uint32_t intervalUs;
    int32_t  minStep;      // soft limits, used when `limits` is set
    int32_t  maxStep;
    bool     limits;
    bool     ramp;         // on the shared ramp profile
};

struct Settings {
    AxisSettings  axis[AXIS_COUNT];
    uint32_t      rampStartUs;
    uint16_t      rampSteps;
    uint32_t      idleReleaseMs;
    bool          homeOnStart;
};

static constexpr uint16_t SETTINGS_MAGIC   = 0xB16D;
static constexpr uint8_t  SETTINGS_VERSION = 2;

SettingsStore<Settings> store(SETTINGS_MAGIC, SETTINGS_VERSION);
Settings settings;

// Unsaved changes, reported by `s`.
static bool settingsChanged = false;

static Settings defaultSettings() {
    Settings d;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        d.axis[i] = { STEP_INTERVAL_US, 0, 0, false, RAMP_STEPS > 0 };
    }
    d.rampStartUs   = RAMP_START_US;
    d.rampSteps     = RAMP_STEPS;
    d.idleReleaseMs = IDLE_RELEASE_MS;
    d.homeOnStart   = HOME_ON_START;
    return d;
}

static void applyAxis(uint8_t id) {
    const AxisSettings& a = settings.axis[id];
    fleet.setStepInterval(id, a.intervalUs);
    fleet.setProfile(id, a.ramp ? &ramp : nullptr);
    if (a.limits) fleet.setLimits(id, a.minStep, a.maxStep);
    else          fleet.clearLimits(id);
}

static void applySettings() {
    ramp.setStartInterval(settings.rampStartUs);
    ramp.setRampSteps(settings.rampSteps);
    fleet.setIdleRelease(settings.idleReleaseMs);
    for (uint8_t i = 0; i < fleet.size(); i++) applyAxis(i);
}

static_assert(pulseUs(Driver::A4988) == PolledEngine::A4988_PULSE_US,
              "Driver.h and MotorControl disagree on the A4988 pulse");
static_assert(pulseUs(Driver::TB6600) == PolledEngine::TB6600_PULSE_US,
              "Driver.h and MotorControl disagree on the TB6600 pulse");

static Stepper* makeAxis(const AxisPins& axis) {
    Stepper* stepper = nullptr;

    switch (axis.engine) {
        case Engine::POLLED:
            // No per-axis EN: the fleet owns the shared pin.
            stepper = new EngineStepper<PolledEngine>(STEPS_PER_REV, axis.step, axis.dir,
                                                      PolledEngine::NO_ENABLE_PIN,
                                                      STEP_INTERVAL_US, pulseUs(axis.driver));
            break;
    }
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

// Text is printed piece by piece from F() strings: on the Uno, plain literals
// are copied to RAM and String concatenation fragments the heap.

static void printStatus() {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        Serial.print(AXES[i].name);
        Serial.print(isSelected(i) ? F(" <") : F("  "));
        Serial.print(F(" step="));
        Serial.print(fleet.getStep(i));
        Serial.print(F(" target="));
        Serial.print(fleet.getTargetStep(i));
        Serial.print(F(" state="));
        Serial.print(fleet.isHold(i) ? F("HOLD") : F("MOVING"));
        Serial.print(F(" interval="));
        Serial.print(fleet.getStepInterval(i));
        Serial.print(F("us now="));
        Serial.print(fleet.getCurrentInterval(i));
        Serial.print(F("us ramp="));
        if (fleet.getProfile(i)) Serial.print(ramp.getRampSteps());
        else                     Serial.print(F("off"));
        Serial.print(F(" lim="));
        if (fleet.hasLimits(i)) {
            Serial.print(settings.axis[i].minStep);
            Serial.print(F(".."));
            Serial.print(settings.axis[i].maxStep);
            if (!fleet.limitsArmed(i)) Serial.print(F("(not armed)"));
        } else {
            Serial.print(F("off"));
        }
        Serial.print(F(" debug="));
        Serial.print(fleet.getDebug(i) ? F("on") : F("off"));
        if (homingEndstop(AXES[i])) {
            Serial.print(F(" homing="));
            Serial.print(homingResultName(fleet.getHomingResult(i)));
        }
        Serial.println();
    }

    if (GRIPPER_ENABLED) {
        Serial.print(F("gripper angle="));
        Serial.println(gripper.getAngle(GRIPPER));
    } else {
        Serial.println(F("gripper=off"));
    }

    unsigned long idle = fleet.getIdleRelease();
    Serial.print(F("EN " PIN_PREFIX));
    Serial.print(fleet.getEnablePin());
    Serial.print(fleet.isPowered() ? F(" energised") : F(" released"));
    Serial.print(F(", release="));
    if (idle) {
        Serial.print(idle);
        Serial.println(F("ms"));
    } else {
        Serial.println(F("off"));
    }

    Serial.print(F("autohome="));
    Serial.println(settings.homeOnStart ? F("on") : F("off"));

    Serial.print(F("settings: "));
    Serial.print(store.describe(store.source()));
    Serial.println(settingsChanged ? F(", changed since - save to keep") : F(""));
}

static void printHelp() {
    Serial.println(F("commands (words separated by spaces):"));
    Serial.println(F("  x|y|z|a  select that axis for the commands below"));
    Serial.println(F("  *        select every axis at once"));
    Serial.println(F("  f [n]    jog n steps forward  (DIR high), default 100"));
    Serial.println(F("  b [n]    jog n steps backward (DIR low),  default 100"));
    Serial.println(F("  g <n>    go to absolute step n (may be negative)"));
    Serial.println(F("  grip [n] turn the gripper to n degrees, 0-180 (bare grip: where it is)"));
    Serial.println(F("  o        re-declare the current position as step 0"));
    Serial.println(F("  h        halt: cancel the move, hold here"));
    Serial.println(F("  home     home against the endstop (blocking; '*' = every axis with one)"));
    Serial.println(F("  v [n]    step interval in us, lower is faster (bare v resets)"));
    Serial.println(F("  r [n]    ramp over n steps (r 0 = none, bare r toggles)"));
    Serial.println(F("  p [n]    release the coils after n ms fleet idle (p 0 = never)"));
    Serial.println(F("  lim <min> <max>  soft limits in steps | lim off"));
    Serial.println(F("  autohome [0|1]  home every axis with a switch at start-up (bare: show)"));
    Serial.println(F("  save     keep the settings (v, r, p, lim, autohome) across power cycles"));
    Serial.println(F("  load     back to the saved settings | defaults  back to the built-in ones"));
    Serial.println(F("  l        read each endstop pin right now (diagnostic)"));
    Serial.println(F("  reset    restart the board; the position is lost, so home again"));
    Serial.println(F("  s        status | d debug | mem free RAM | ? help"));
}

static void printEndstop(const char* axis, const Endstop& e) {
    bool high = (digitalRead(e.getPin()) == HIGH);
    Serial.print(axis);
    Serial.print(' ');
    Serial.print(e.sideName());
    Serial.print(F(" pin="));
    Serial.print(e.getPin());
    Serial.print(e.getWiring() == Endstop::NORMALLY_CLOSED ? F(" NC") : F(" NO"));
    Serial.print(F(" reads="));
    Serial.print(high ? F("HIGH") : F("LOW"));
    Serial.print(F(" -> code says "));
    Serial.println(e.readRaw() ? F("TRIGGERED") : F("clear"));
}

// Blocking. Skips axes without a switch on their homing side.
static void homeSelected() {
    bool any = false;

    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!isSelected(i) || !homingEndstop(AXES[i])) continue;

        any = true;
        HomingResult r = fleet.home(i, AXES[i].homing);
        Serial.print(AXES[i].name);
        Serial.print(F(" homing: "));
        Serial.println(homingResultName(r));
    }

    if (!any) {
        Serial.print(selectionName());
        Serial.println(F(" has no endstop to home against"));
    }
}

static void jog(long steps) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.moveSteps(i, steps);
    }

    moveStartedAt = millis();
    Serial.print(F("-> "));
    Serial.print(selectionName());
    Serial.print(steps >= 0 ? F(" +") : F(" "));
    Serial.print(steps);
    Serial.println(F(" steps"));
}
static uint8_t axisFromLetter(char c) {
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (tolower(AXES[i].name[0]) == tolower(c)) return i;
    }
    return ALL_AXES;
}

// --- Free RAM ---------------------------------------------------------------
// On AVR, setup() paints the heap-stack gap with a marker byte; bytes that
// still hold it were never used, which gives the worst-case margin.

#if defined(__AVR__)
extern char  __heap_start;
extern char* __brkval;

static constexpr uint8_t RAM_MARKER = 0xA5;

static char* heapEnd() {
    return __brkval ? __brkval : &__heap_start;
}

static size_t freeRamNow() {
    char top;
    return (size_t)(&top - heapEnd());
}

// Leaves a few bytes under this function's own frame unpainted.
static void paintFreeRam() {
    char top;
    for (char* p = heapEnd(); p < &top - 16; p++) *p = (char)RAM_MARKER;
}

static size_t freeRamLowest() {
    char top;
    size_t n = 0;
    for (char* p = heapEnd(); p < &top && (uint8_t)*p == RAM_MARKER; p++) n++;
    return n;
}
#elif defined(ESP32)
static size_t freeRamNow()    { return ESP.getFreeHeap(); }
static size_t freeRamLowest() { return ESP.getMinFreeHeap(); }
static void   paintFreeRam()  {}
#else
static size_t freeRamNow()    { return 0; }
static size_t freeRamLowest() { return 0; }
static void   paintFreeRam()  {}
#endif

// --- Console commands -------------------------------------------------------
// Arguments are validated by CommandDispatcher before a handler runs.
// Behaviour of each command: docs/commands.md.

CommandDispatcher<28> commands(Serial);
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
    Serial.print(F("-> "));
    Serial.print(selectionName());
    Serial.print(F(" absolute step "));
    Serial.println(n);
}

static void cmdOrigin(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.setOrigin(i);
    }
    Serial.print(selectionName());
    Serial.println(F(" origin reset, step 0 is here"));
}

// The gripper is not an axis: the selection does not apply.
static void cmdGrip(const CommandArgs& args) {
    if (!GRIPPER_ENABLED) {
        Serial.println(F("gripper disabled"));
    } else if (args.has(0)) {
        long n = args.getLong(0);
        if (n < 0) n = 0;
        if (n > 180) n = 180;
        gripper.setAngle(GRIPPER, (int)n);
        Serial.print(F("-> gripper to "));
        Serial.print(n);
        Serial.println(F(" deg"));
    } else {
        Serial.print(F("gripper at "));
        Serial.print(gripper.getAngle(GRIPPER));
        Serial.println(F(" deg"));
    }
}

static void cmdHalt(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.stopHere(i);
    }
    Serial.print(selectionName());
    Serial.println(F(" halted"));
}

static void cmdEndstops(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!hasEndstops(AXES[i])) {
            Serial.print(AXES[i].name);
            Serial.println(F(" no endstop"));
            continue;
        }

        if (AXES[i].min) printEndstop(AXES[i].name, *AXES[i].min);
        if (AXES[i].max) printEndstop(AXES[i].name, *AXES[i].max);
    }
}

// Applies from the next pulse, without a ramp: a large jump under load can lose steps.
static void cmdInterval(const CommandArgs& args) {
    long n = args.getLong(0);
    unsigned long us = (n > 0) ? (unsigned long)n : STEP_INTERVAL_US;

    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!isSelected(i)) continue;
        settings.axis[i].intervalUs = us;
        applyAxis(i);
    }
    settingsChanged = true;

    Serial.print(F("-> "));
    Serial.print(selectionName());
    Serial.print(F(" interval "));
    Serial.print(us);
    Serial.println(F("us"));
}

// The ramp length is shared by all axes; the selection only decides which use it.
static void cmdRamp(const CommandArgs& args) {
    long n = args.getLong(0);
    if (n > 0) {
        settings.rampSteps = (uint16_t)n;
        ramp.setRampSteps(settings.rampSteps);
    }

    bool off = args.has(0) && n == 0;
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!isSelected(i)) continue;

        settings.axis[i].ramp = off ? false
                                    : (args.has(0) ? true : !settings.axis[i].ramp);
        applyAxis(i);
    }
    settingsChanged = true;

    printStatus();
}

// Fleet-wide: one EN line for all sockets, so the selection does not apply.
static void cmdIdleRelease(const CommandArgs& args) {
    long n = args.getLong(0);
    unsigned long idle = (n > 0) ? (unsigned long)n : 0;
    if (!args.has(0)) {
        idle = fleet.getIdleRelease() ? 0 : IDLE_RELEASE_MS;
    }

    settings.idleReleaseMs = idle;
    settingsChanged = true;
    fleet.setIdleRelease(idle);
    // Re-energise now: a released fleet would report positions it is not holding.
    if (idle == 0) fleet.enable();

    printStatus();
}

// Enforced only once the position is known (after homing or 'o').
static void cmdLimits(const CommandArgs& args) {
    long lo = args.getLong(0);
    long hi = args.getLong(1);
    
    if (lo > hi) {
        long t = lo;
        lo = hi;
        hi = t;
    }

    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!isSelected(i)) continue;
        settings.axis[i].minStep = lo;
        settings.axis[i].maxStep = hi;
        settings.axis[i].limits  = true;
        applyAxis(i);
    }
    settingsChanged = true;

    printStatus();
}

static void cmdLimitsWord(const CommandArgs& args) {
    if (!args.is(0, F("off"))) {
        Serial.println(F("usage: lim <min> <max> | lim off"));
        return;
    }

    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (!isSelected(i)) continue;
        settings.axis[i].limits = false;
        applyAxis(i);
    }
    settingsChanged = true;

    printStatus();
}

static void cmdSave(const CommandArgs&) {
    if (!store.save(settings)) {
        Serial.println(F("save failed"));
        return;
    }

    settingsChanged = false;
    Serial.print(F("saved, "));
    Serial.print((int)store.SIZE);
    Serial.println(F(" bytes"));
}

// Bare `autohome` only shows the setting, so a stray command cannot enable it.
static void cmdAutohome(const CommandArgs& args) {
    if (args.has(0)) {
        settings.homeOnStart = args.getLong(0) != 0;
        settingsChanged = true;
    }
    printStatus();
}

static void cmdLoad(const CommandArgs&) {
    store.load(settings, defaultSettings());
    applySettings();
    settingsChanged = false;
    printStatus();
}

// Leaves the saved settings untouched.
static void cmdDefaults(const CommandArgs&) {
    settings = defaultSettings();
    applySettings();
    settingsChanged = true;
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

// Halts every axis first, so no pulse is cut short. The position is lost.
static void cmdReset(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) fleet.stopHere(i);

    Serial.println(F("restarting"));
    Serial.flush();
    restartBoard();
}

static void cmdMemory(const CommandArgs&) {
    Serial.print(F("mem: free now "));
    Serial.print((unsigned long)freeRamNow());
    Serial.print(F(" B, never used "));
    Serial.print((unsigned long)freeRamLowest());
    Serial.println(F(" B"));
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
    Serial.print(F("selected: "));
    Serial.println(AXES[selected].name);
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
    commands.add(F("grip"), F("[n]"), cmdGrip);
    commands.add(F("o"), cmdOrigin);
    commands.add(F("h"), cmdHalt);
    commands.add(F("l"), cmdEndstops);
    commands.add(F("v"), F("[n]"), cmdInterval);
    commands.add(F("r"), F("[n]"), cmdRamp);
    commands.add(F("p"), F("[n]"), cmdIdleRelease);
    commands.add(F("lim"), F("nn"), cmdLimits);
    commands.add(F("lim"), F("w"), cmdLimitsWord);
    commands.add(F("autohome"), F("[n]"), cmdAutohome);
    commands.add(F("save"), cmdSave);
    commands.add(F("load"), cmdLoad);
    commands.add(F("defaults"), cmdDefaults);
    commands.add(F("d"), cmdDebug);
    commands.add(F("s"), cmdStatus);
    commands.add(F("mem"), cmdMemory);
    commands.add(F("reset"), cmdReset);
    commands.add(F("?"), cmdHelp);
    commands.add(F("help"), cmdHelp);
    commands.add(F("*"), cmdSelectAll);

    // F() needs literals, so the axis letters are spelled out: keep in step with AXES.
    commands.add(F("x"), cmdSelectAxis);
    commands.add(F("y"), cmdSelectAxis);
    commands.add(F("z"), cmdSelectAxis);
    commands.add(F("a"), cmdSelectAxis);

    commands.onUnknown(onUnknownCommand);
}

static void onAxisReached(uint8_t id) {
    Serial.print(AXES[id].name);
    Serial.print(F(" reached step "));
    Serial.print(fleet.getStep(id));
    Serial.print(F(" in "));
    Serial.print(millis() - moveStartedAt);
    Serial.println(F(" ms"));
}

void setup() {
    clearRestart();
    paintFreeRam();
    Serial.begin(115200);
    while (!Serial) { /* Uno returns immediately; harmless on native-USB boards */ }

    Serial.println(F("\n=== CNC Shield v3 - step jog ==="));

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (fleet.addMotor(makeAxis(AXES[i])) == StepperManager::INVALID_ID) {
            Serial.print(F("axis "));
            Serial.print(AXES[i].name);
            Serial.println(F(" not registered, out of memory"));
            break;
        }

        Serial.print(F("socket="));
        Serial.print(AXES[i].name);
        Serial.print(F(" STEP=" PIN_PREFIX));
        Serial.print(AXES[i].step);
        Serial.print(F(" DIR=" PIN_PREFIX));
        Serial.print(AXES[i].dir);
        Serial.print(F(" EN=" PIN_PREFIX));
        Serial.print(EN_PIN);
        Serial.print(F(" driver="));
        Serial.print(driverName(AXES[i].driver));
        Serial.print(F(" pulse="));
        Serial.print(pulseUs(AXES[i].driver));
        Serial.print(F("us engine="));
        Serial.println(engineName(AXES[i].engine));
    }

    Serial.print(F("steps/rev="));
    Serial.print(STEPS_PER_REV);
    Serial.print(F(" ("));
    Serial.print(FULL_STEPS_PER_REV);
    Serial.print(F(" full x 1/"));
    Serial.print(MICROSTEPS);
    Serial.println(F(")"));

    registerCommands();

    store.load(settings, defaultSettings());
    applySettings();
    fleet.onReached(onAxisReached);
    fleet.begin();

    if (GRIPPER_ENABLED) {
        // Blocks for a second while the servo reaches its start angle.
        if (gripper.addMotor(new ServoMotor(GRIPPER_PIN, GRIPPER_START_ANGLE)) == ServoManager::INVALID_ID) {
            Serial.println(F("gripper not registered, out of memory"));
        }
        gripper.begin();
    }

    printStatus();
    printHelp();

    if (settings.homeOnStart) {
        Serial.println(F("autohome=on, homing"));
        uint8_t keep = selected;
        selected = ALL_AXES;
        homeSelected();
        selected = keep;
    }
}

void loop() {
    fleet.update();
    if (GRIPPER_ENABLED) gripper.update();
    console.poll(commands);
}
