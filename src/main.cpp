#include <Arduino.h>
#include <CommandDispatcher.h>
#include <SettingsStore.h>
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

static constexpr bool HOME_ON_START = false;

// Release the coils once the whole fleet has been at rest this long
static constexpr unsigned long IDLE_RELEASE_MS = 200;

// The start interval has to be slow enough that the axis can pull in from rest,
// and the ramp long enough that it reaches STEP_INTERVAL_US comfortably.
static constexpr unsigned long RAMP_START_US = 1000;
static constexpr uint16_t      RAMP_STEPS    = 1000;

LinearRamp ramp(RAMP_START_US, RAMP_STEPS);

StepperManager fleet(AXIS_COUNT, EN_PIN);

// --- Settings kept across power cycles ---------------------------------------
//
// What the console can change and `save` keeps. The commands change these and
// apply them to the fleet; at start-up they come back from EEPROM, or from the
// constants above when nothing valid was saved.
//
// Bump SETTINGS_VERSION whenever these structs change, so an old save is
// ignored rather than read into the wrong fields.

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
};

static constexpr uint16_t SETTINGS_MAGIC   = 0xB16D;
static constexpr uint8_t  SETTINGS_VERSION = 1;

SettingsStore<Settings> store(SETTINGS_MAGIC, SETTINGS_VERSION);
Settings settings;

// Changed since the last save or load, so `s` can say they would be lost.
static bool settingsChanged = false;

static Settings defaultSettings() {
    Settings d;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        d.axis[i] = { STEP_INTERVAL_US, 0, 0, false, RAMP_STEPS > 0 };
    }
    d.rampStartUs   = RAMP_START_US;
    d.rampSteps     = RAMP_STEPS;
    d.idleReleaseMs = IDLE_RELEASE_MS;
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

// Messages are printed piece by piece with the text in F(): on the Uno a
// plain "literal" is copied into RAM at start-up, and joining Strings borrows
// heap the board can barely spare.

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

    unsigned long idle = fleet.getIdleRelease();
    Serial.print(F("EN D"));
    Serial.print(fleet.getEnablePin());
    Serial.print(fleet.isPowered() ? F(" energised") : F(" released"));
    Serial.print(F(", release="));
    if (idle) {
        Serial.print(idle);
        Serial.println(F("ms"));
    } else {
        Serial.println(F("off"));
    }

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
    Serial.println(F("  o        re-declare the current position as step 0"));
    Serial.println(F("  h        halt: cancel the move, hold here"));
    Serial.println(F("  home     home against the endstop (blocking; '*' = every axis with one)"));
    Serial.println(F("  v [n]    step interval in us, lower is faster (bare v resets)"));
    Serial.println(F("  r [n]    ramp over n steps (r 0 = none, bare r toggles)"));
    Serial.println(F("  p [n]    release the coils after n ms fleet idle (p 0 = never)"));
    Serial.println(F("  lim <min> <max>  soft limits in steps | lim off"));
    Serial.println(F("  save     keep the settings (v, r, p, lim) across power cycles"));
    Serial.println(F("  load     back to the saved settings | defaults  back to the built-in ones"));
    Serial.println(F("  l        read each endstop pin right now (diagnostic)"));
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

// Blocking: the console does not answer until every selected axis is done.
// Axes without a switch on their homing side are skipped, so '*' then 'home'
// homes everything that can be homed.
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
//
// On the Uno, setup() first fills the gap between the heap and the stack with
// a marker byte. Whatever still holds the marker later was never reached by
// the stack or the heap, which is the worst-case margin so far.

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
//
// Each one is registered in registerCommands() with the arguments it takes;
// CommandDispatcher checks them before the handler runs, so a handler never
// sees a missing number or a word where a number belongs.

CommandDispatcher<26> commands(Serial);
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

static void cmdHalt(const CommandArgs&) {
    for (uint8_t i = 0; i < fleet.size(); i++) {
        if (isSelected(i)) fleet.stopHere(i);
    }
    Serial.print(selectionName());
    Serial.println(F(" halted"));
}

// Diagnostic only: the raw pin, and what the configured wiring mode makes of
// it. Moves nothing.
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

// Takes effect on the next pulse, mid-move included - there is no ramp, so a
// big jump under load is where a stepper loses steps.
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

// The ramp length is the shared profile's, so a number here moves every axis
// using it; what the selection controls is whether the axis is on the profile
// at all. Bare 'r' toggles, 'r 0' turns it off, 'r <n>' sets and turns it on.
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

// One EN line for the shield, so this is a fleet-wide setting and ignores the
// axis selection. A bare 'p' toggles, which is the quick way to feel the
// difference on the shaft; 'p <n>' sets it outright.
static void cmdIdleRelease(const CommandArgs& args) {
    long n = args.getLong(0);
    unsigned long idle = (n > 0) ? (unsigned long)n : 0;
    if (!args.has(0)) {
        idle = fleet.getIdleRelease() ? 0 : IDLE_RELEASE_MS;
    }

    settings.idleReleaseMs = idle;
    settingsChanged = true;
    fleet.setIdleRelease(idle);
    // Turning it off does not re-energise on its own, and a released fleet
    // would report positions it is not holding.
    if (idle == 0) fleet.enable();

    printStatus();
}

// Takes effect at once, but only once armed - after homing or 'o'. Before
// that the position is a guess, so a limit would be measured from nowhere.
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

static void cmdLoad(const CommandArgs&) {
    store.load(settings, defaultSettings());
    applySettings();
    settingsChanged = false;
    printStatus();
}

// Does not touch what is saved: 'save' afterwards to make it stick, or 'load'
// to go back.
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

// RAM the board has spare. On the Uno "free now" is the gap between the heap
// and the stack at this moment; "never used" is how much of the gap painted
// at start-up no call has touched since - the margin at the worst moment so
// far. Exercise the console (home, s, l, ...) before reading it.
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
    commands.add(F("o"), cmdOrigin);
    commands.add(F("h"), cmdHalt);
    commands.add(F("l"), cmdEndstops);
    commands.add(F("v"), F("[n]"), cmdInterval);
    commands.add(F("r"), F("[n]"), cmdRamp);
    commands.add(F("p"), F("[n]"), cmdIdleRelease);
    commands.add(F("lim"), F("nn"), cmdLimits);
    commands.add(F("lim"), F("w"), cmdLimitsWord);
    commands.add(F("save"), cmdSave);
    commands.add(F("load"), cmdLoad);
    commands.add(F("defaults"), cmdDefaults);
    commands.add(F("d"), cmdDebug);
    commands.add(F("s"), cmdStatus);
    commands.add(F("mem"), cmdMemory);
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
    Serial.print(AXES[id].name);
    Serial.print(F(" reached step "));
    Serial.print(fleet.getStep(id));
    Serial.print(F(" in "));
    Serial.print(millis() - moveStartedAt);
    Serial.println(F(" ms"));
}

void setup() {
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
        Serial.print(F(" STEP=D"));
        Serial.print(AXES[i].step);
        Serial.print(F(" DIR=D"));
        Serial.print(AXES[i].dir);
        Serial.print(F(" EN=D"));
        Serial.println(EN_PIN);
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
