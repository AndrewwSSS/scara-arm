#include <unity.h>
#include <Arduino.h>

#include "SettingsStore.h"

struct Settings {
    uint16_t interval;
    int32_t  minStep;
    int32_t  maxStep;
    bool     ramp;
};

static const Settings DEFAULTS = { 400, 0, 3200, true };
static const Settings CHANGED  = { 250, -100, 5000, false };

typedef SettingsStore<Settings> Store;

static const char* text(const __FlashStringHelper* s) {
    return reinterpret_cast<const char*>(s);
}

static void assertSame(const Settings& expected, const Settings& actual) {
    TEST_ASSERT_EQUAL(expected.interval, actual.interval);
    TEST_ASSERT_EQUAL(expected.minStep, actual.minStep);
    TEST_ASSERT_EQUAL(expected.maxStep, actual.maxStep);
    TEST_ASSERT_EQUAL(expected.ramp, actual.ramp);
}

void setUp() {
    EEPROM.wipe();
}

void tearDown() {}

// --- Loading and saving -------------------------------------------------------

void test_a_blank_chip_gives_the_defaults() {
    Store store(0xB16D, 1);
    Settings s = CHANGED;

    TEST_ASSERT_FALSE(store.load(s, DEFAULTS));
    assertSame(DEFAULTS, s);
    TEST_ASSERT_EQUAL(Store::NOTHING_SAVED, store.source());
}

void test_what_was_saved_comes_back() {
    Store store(0xB16D, 1);
    TEST_ASSERT_TRUE(store.save(CHANGED));

    Settings s;
    TEST_ASSERT_TRUE(store.load(s, DEFAULTS));
    assertSame(CHANGED, s);
    TEST_ASSERT_EQUAL(Store::LOADED, store.source());
}

void test_saved_values_survive_a_new_store_object() {
    // As after a reset: a fresh object reading what an earlier one wrote.
    Store(0xB16D, 1).save(CHANGED);

    Store afterReset(0xB16D, 1);
    Settings s;
    TEST_ASSERT_TRUE(afterReset.load(s, DEFAULTS));
    assertSame(CHANGED, s);
}

void test_saving_the_same_values_again_writes_nothing() {
    Store store(0xB16D, 1);
    store.save(CHANGED);
    EEPROM.writes = 0;

    store.save(CHANGED);
    TEST_ASSERT_EQUAL(0, EEPROM.writes);
}

void test_the_record_takes_exactly_its_size() {
    Store store(0xB16D, 1, 100);
    store.save(CHANGED);

    TEST_ASSERT_EQUAL_HEX8(0xFF, EEPROM.bytes[99]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, EEPROM.bytes[100 + Store::SIZE]);
    TEST_ASSERT_NOT_EQUAL(0xFF, EEPROM.bytes[100]);   // the magic's low byte, 0x6D
}

// --- What load() refuses ------------------------------------------------------

void test_another_magic_is_nothing_saved() {
    Store(0x1234, 1).save(CHANGED);

    Store store(0xB16D, 1);
    Settings s;
    TEST_ASSERT_FALSE(store.load(s, DEFAULTS));
    assertSame(DEFAULTS, s);
    TEST_ASSERT_EQUAL(Store::NOTHING_SAVED, store.source());
}

void test_another_version_is_an_old_layout() {
    Store(0xB16D, 1).save(CHANGED);

    Store store(0xB16D, 2);
    Settings s;
    TEST_ASSERT_FALSE(store.load(s, DEFAULTS));
    assertSame(DEFAULTS, s);
    TEST_ASSERT_EQUAL(Store::OLD_LAYOUT, store.source());
}

void test_a_struct_that_changed_size_is_an_old_layout() {
    // The version was not bumped, but the struct grew.
    struct Bigger { Settings s; int32_t extra; };
    SettingsStore<Bigger>(0xB16D, 1).save(Bigger{ CHANGED, 7 });

    Store store(0xB16D, 1);
    Settings s;
    TEST_ASSERT_FALSE(store.load(s, DEFAULTS));
    TEST_ASSERT_EQUAL(Store::OLD_LAYOUT, store.source());
}

void test_a_flipped_bit_in_the_values_is_damage() {
    Store store(0xB16D, 1);
    store.save(CHANGED);
    EEPROM.bytes[5] ^= 0x01;   // the first byte of the struct

    Settings s;
    TEST_ASSERT_FALSE(store.load(s, DEFAULTS));
    assertSame(DEFAULTS, s);
    TEST_ASSERT_EQUAL(Store::DAMAGED, store.source());
}

void test_a_flipped_bit_in_the_checksum_is_damage() {
    Store store(0xB16D, 1);
    store.save(CHANGED);
    EEPROM.bytes[Store::SIZE - 1] ^= 0x80;

    Settings s;
    TEST_ASSERT_FALSE(store.load(s, DEFAULTS));
    TEST_ASSERT_EQUAL(Store::DAMAGED, store.source());
}

void test_every_single_bit_flip_is_caught() {
    Store store(0xB16D, 1);
    store.save(CHANGED);

    for (size_t byte = 0; byte < Store::SIZE; byte++) {
        for (uint8_t bit = 0; bit < 8; bit++) {
            EEPROM.bytes[byte] ^= (uint8_t)(1 << bit);
            Settings s;
            TEST_ASSERT_FALSE_MESSAGE(store.load(s, DEFAULTS), "a flipped bit was loaded");
            EEPROM.bytes[byte] ^= (uint8_t)(1 << bit);
        }
    }

    Settings s;
    TEST_ASSERT_TRUE(store.load(s, DEFAULTS));   // and restored, it loads again
}

void test_a_save_cut_short_by_power_loss_is_damage() {
    Store store(0xB16D, 1);
    store.save(DEFAULTS);

    // The new values were half written when the power went.
    Settings half = DEFAULTS;
    half.interval = CHANGED.interval;
    EEPROM.put(5, half);

    Settings s;
    TEST_ASSERT_FALSE(store.load(s, DEFAULTS));
    TEST_ASSERT_EQUAL(Store::DAMAGED, store.source());
}

// --- Erase, describe, neighbours ------------------------------------------------

void test_erase_makes_the_next_load_find_nothing() {
    Store store(0xB16D, 1);
    store.save(CHANGED);
    TEST_ASSERT_TRUE(store.erase());

    Settings s;
    TEST_ASSERT_FALSE(store.load(s, DEFAULTS));
    TEST_ASSERT_EQUAL(Store::NOTHING_SAVED, store.source());
}

void test_two_stores_side_by_side_keep_apart() {
    Store first(0xB16D, 1, 0);
    Store second(0xB16D, 1, Store::SIZE);
    first.save(DEFAULTS);
    second.save(CHANGED);

    Settings a, b;
    TEST_ASSERT_TRUE(first.load(a, CHANGED));
    TEST_ASSERT_TRUE(second.load(b, DEFAULTS));
    assertSame(DEFAULTS, a);
    assertSame(CHANGED, b);
}

void test_describe_says_where_the_values_came_from() {
    Store store(0xB16D, 1);
    TEST_ASSERT_EQUAL_STRING("not loaded", text(Store::describe(store.source())));

    Settings s;
    store.load(s, DEFAULTS);
    TEST_ASSERT_EQUAL_STRING("defaults (nothing saved)", text(Store::describe(store.source())));

    store.save(CHANGED);
    store.load(s, DEFAULTS);
    TEST_ASSERT_EQUAL_STRING("saved settings", text(Store::describe(store.source())));
}

void test_crc16_matches_the_published_check_value() {
    // CRC-16/CCITT-FALSE of "123456789" is 0x29B1.
    const uint8_t digits[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    TEST_ASSERT_EQUAL_HEX16(0x29B1, ss_detail::crc16(0xFFFF, digits, sizeof digits));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_blank_chip_gives_the_defaults);
    RUN_TEST(test_what_was_saved_comes_back);
    RUN_TEST(test_saved_values_survive_a_new_store_object);
    RUN_TEST(test_saving_the_same_values_again_writes_nothing);
    RUN_TEST(test_the_record_takes_exactly_its_size);
    RUN_TEST(test_another_magic_is_nothing_saved);
    RUN_TEST(test_another_version_is_an_old_layout);
    RUN_TEST(test_a_struct_that_changed_size_is_an_old_layout);
    RUN_TEST(test_a_flipped_bit_in_the_values_is_damage);
    RUN_TEST(test_a_flipped_bit_in_the_checksum_is_damage);
    RUN_TEST(test_every_single_bit_flip_is_caught);
    RUN_TEST(test_a_save_cut_short_by_power_loss_is_damage);
    RUN_TEST(test_erase_makes_the_next_load_find_nothing);
    RUN_TEST(test_two_stores_side_by_side_keep_apart);
    RUN_TEST(test_describe_says_where_the_values_came_from);
    RUN_TEST(test_crc16_matches_the_published_check_value);
    return UNITY_END();
}
