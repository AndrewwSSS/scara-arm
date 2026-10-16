#include <Arduino.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled in this build
#endif

static const char *DEVICE_NAME = "Core-ESP32";

BluetoothSerial SerialBT;

void onBtEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("[BT] client connected");
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("[BT] client disconnected");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  SerialBT.register_callback(onBtEvent);
  if (!SerialBT.begin(DEVICE_NAME)) {
    Serial.println("[BT] init failed");
    while (true) delay(1000);
  }
  Serial.printf("[BT] ready, pair with \"%s\"\n", DEVICE_NAME);
}

void loop() {
  // Bluetooth -> USB serial
  while (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

  // USB serial -> Bluetooth (handy for replying from the monitor)
  while (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  delay(5);
}
