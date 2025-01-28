#include <WiFi.h>
#include <esp_now.h>

#define NUM_SLAVES 3
#define SOUND_SPEED 343000 // Скорость звука в мм/с
#define SYNC_INTERVAL 60000 // Интервал синхронизации времени (1 минута)

typedef struct {
  uint32_t timestamp;
  uint8_t id;
} SlaveData;

typedef struct {
  uint32_t timestamps[NUM_SLAVES];
  uint8_t ids[NUM_SLAVES];
} SlaveTimings;

typedef struct {
  uint32_t masterTime;
} SyncPacket;

SlaveTimings slaveTimings;
bool allTimingsReceived = false;
unsigned long lastSyncTime = 0;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  lastSyncTime = millis();
}

void loop() {
  // Синхронизация времени каждую минуту
  if (millis() - lastSyncTime >= SYNC_INTERVAL) {
    syncTime();
    lastSyncTime = millis();
  }

  // Расчет координат источника звука
  if (allTimingsReceived) {
    calculateSoundSource();
    allTimingsReceived = false;
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  SlaveData data;
  memcpy(&data, incomingData, sizeof(data));

  for (int i = 0; i < NUM_SLAVES; i++) {
    if (slaveTimings.ids[i] == data.id) {
      slaveTimings.timestamps[i] = data.timestamp;
      break;
    }
  }

  bool allReceived = true;
  for (int i = 0; i < NUM_SLAVES; i++) {
    if (slaveTimings.timestamps[i] == 0) {
      allReceived = false;
      break;
    }
  }

  if (allReceived) {
    allTimingsReceived = true;
  }
}

void syncTime() {
  SyncPacket syncPacket;
  syncPacket.masterTime = micros(); // Текущее время ESP32 в микросекундах

  // Отправка пакета синхронизации всем ESP8266
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Широковещательный адрес
  esp_now_send(broadcastAddress, (uint8_t *)&syncPacket, sizeof(syncPacket));
  Serial.println("Time synchronization packet sent");
}

void calculateSoundSource() {
  uint32_t t0 = slaveTimings.timestamps[0];
  uint32_t t1 = slaveTimings.timestamps[1];
  uint32_t t2 = slaveTimings.timestamps[2];

  float d0 = (t0 - t0) * SOUND_SPEED / 1000000.0;
  float d1 = (t1 - t0) * SOUND_SPEED / 1000000.0;
  float d2 = (t2 - t0) * SOUND_SPEED / 1000000.0;

  // Пример расчета координат (упрощенный)
  float x = (d1 * d1 - d0 * d0 + 500 * 500) / (2 * 500);
  float y = (d2 * d2 - d0 * d0 + 500 * 500) / (2 * 500);

  Serial.print("Sound source coordinates: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(") mm");
}
