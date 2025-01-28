#include <ESP8266WiFi.h>
#include <espnow.h>

#define SOUND_SPEED 343000 // Скорость звука в мм/с

uint8_t masterAddress[] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56}; // Замените на MAC-адрес ESP32

typedef struct {
  uint32_t timestamp;
  uint8_t id;
} SlaveData;

typedef struct {
  uint32_t masterTime;
} SyncPacket;

SlaveData myData;
uint32_t timeOffset = 0; // Смещение времени относительно ESP32

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(masterAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  esp_now_register_recv_cb(OnDataRecv);

  myData.id = 1; // Уникальный ID для каждого устройства
}

void loop() {
  if (detectSound()) {
    myData.timestamp = micros() + timeOffset; // Корректировка времени с учетом смещения
    esp_now_send(masterAddress, (uint8_t *)&myData, sizeof(myData));
    delay(1000); // Задержка для предотвращения множественных отправок
  }
}

bool detectSound() {
  // Логика обнаружения звука
  // Возвращает true, если звук обнаружен
  return false; // Замените на реальную логику
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len == sizeof(SyncPacket)) {
    SyncPacket syncPacket;
    memcpy(&syncPacket, incomingData, sizeof(syncPacket));

    // Корректировка времени
    timeOffset = syncPacket.masterTime - micros();
    Serial.println("Time synchronized with master");
  }
}
