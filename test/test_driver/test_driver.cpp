#include <unity.h>
#include <Arduino.h>

#include "Driver.h"

static const char* text(const __FlashStringHelper* s) {
    return reinterpret_cast<const char*>(s);
}

void test_an_a4988_gets_2_us_pulses() {
    TEST_ASSERT_EQUAL_UINT8(2, pulseUs(Driver::A4988));
}

void test_a_tb6600_gets_5_us_pulses() {
    TEST_ASSERT_EQUAL_UINT8(5, pulseUs(Driver::TB6600));
}

void test_each_driver_prints_its_own_name() {
    TEST_ASSERT_EQUAL_STRING("A4988", text(driverName(Driver::A4988)));
    TEST_ASSERT_EQUAL_STRING("TB6600", text(driverName(Driver::TB6600)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_an_a4988_gets_2_us_pulses);
    RUN_TEST(test_a_tb6600_gets_5_us_pulses);
    RUN_TEST(test_each_driver_prints_its_own_name);
    return UNITY_END();
}
