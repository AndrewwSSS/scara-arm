#include <unity.h>
#include <Arduino.h>

#include "Engine.h"

static const char* text(const __FlashStringHelper* s) {
    return reinterpret_cast<const char*>(s);
}

void test_each_engine_prints_its_own_name() {
    TEST_ASSERT_EQUAL_STRING("polled", text(engineName(Engine::POLLED)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_each_engine_prints_its_own_name);
    return UNITY_END();
}
