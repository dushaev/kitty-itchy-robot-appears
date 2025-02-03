#include <ESP8266WiFi.h>
#include <espnow.h>

#define SOUND_SPEED 343000 // Скорость звука в мм/с

uint8_t masterAddress[] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56}; // Замените на MAC-адрес ESP32

typedef struct {
  uint32_t timestamp;
  uint8_t id;
} SlaveData;

typedef struct {
  uint8_t command; // 1 - издать звук, 0 - остановить
} BeaconCommand;

SlaveData myData;
bool soundActive = false;

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
  if (soundActive) {
    // Издание звука (например, включение пьезодинамика)
    tone(5, 1000); // Пьезодинамик на пине 5, частота 1000 Гц
  } else {
    noTone(5); // Выключение звука
  }
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len == sizeof(BeaconCommand)) {
    BeaconCommand command;
    memcpy(&command, incomingData, sizeof(command));

    if (command.command == 1) {
      soundActive = true;
      myData.timestamp = micros(); // Запись времени издания звука
      esp_now_send(masterAddress, (uint8_t *)&myData, sizeof(myData));
    } else {
      soundActive = false;
    }
  }
}
