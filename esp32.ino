#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

#define NUM_SLAVES 3
#define SOUND_SPEED 343000 // Скорость звука в мм/с
#define SYNC_INTERVAL 60000 // Интервал синхронизации времени (1 минута)

typedef struct {
  uint32_t timestamp;
  uint8_t id;
} SlaveData;

typedef struct {
  uint32_t timestamps[NUM_SLAVES + 1]; // +1 для ESP32
  uint8_t ids[NUM_SLAVES + 1];
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

  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    if (slaveTimings.ids[i] == data.id) {
      slaveTimings.timestamps[i] = data.timestamp;
      break;
    }
  }

  bool allReceived = true;
  for (int i = 0; i < NUM_SLAVES + 1; i++) {
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
  uint32_t t3 = slaveTimings.timestamps[3]; // Данные с ESP32

  // Вычисление разницы во времени
  float dt0 = (t0 - t0) * SOUND_SPEED / 1000000.0;
  float dt1 = (t1 - t0) * SOUND_SPEED / 1000000.0;
  float dt2 = (t2 - t0) * SOUND_SPEED / 1000000.0;
  float dt3 = (t3 - t0) * SOUND_SPEED / 1000000.0;

  // Координаты устройств (в мм)
  float x0 = 0, y0 = 0; // ESP32
  float x1 = 500, y1 = 0; // ESP8266 #1
  float x2 = 500, y2 = 500; // ESP8266 #2
  float x3 = 0, y3 = 500; // ESP8266 #3

  // Решение системы уравнений для нахождения координат источника звука
  float A = 2 * (x1 - x0);
  float B = 2 * (y1 - y0);
  float C = dt1 * dt1 - dt0 * dt0 - (x1 * x1 - x0 * x0) - (y1 * y1 - y0 * y0);

  float D = 2 * (x2 - x0);
  float E = 2 * (y2 - y0);
  float F = dt2 * dt2 - dt0 * dt0 - (x2 * x2 - x0 * x0) - (y2 * y2 - y0 * y0);

  float G = 2 * (x3 - x0);
  float H = 2 * (y3 - y0);
  float I = dt3 * dt3 - dt0 * dt0 - (x3 * x3 - x0 * x0) - (y3 * y3 - y0 * y0);

  // Решение системы уравнений методом Крамера
  float det = A * (E * I - F * H) - B * (D * I - F * G) + C * (D * H - E * G);
  if (det == 0) {
    Serial.println("No solution (det == 0)");
    return;
  }

  float x = (C * (E * I - F * H) - B * (F * I - F * G) + C * (F * H - E * G)) / det;
  float y = (A * (F * I - F * G) - C * (D * I - F * G) + C * (D * H - E * G)) / det;

  // Отбрасывание некорректных измерений
  float distances[4] = {
    sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)),
    sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)),
    sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2)),
    sqrt((x - x3) * (x - x3) + (y - y3) * (y - y3))
  };

  float avgDistance = (distances[0] + distances[1] + distances[2] + distances[3]) / 4;
  float maxDeviation = 0;
  int outlierIndex = -1;

  for (int i = 0; i < 4; i++) {
    float deviation = abs(distances[i] - avgDistance);
    if (deviation > maxDeviation) {
      maxDeviation = deviation;
      outlierIndex = i;
    }
  }

  // Если найдено некорректное измерение, пересчитываем координаты без него
  if (outlierIndex != -1) {
    Serial.println("Outlier detected, recalculating...");
    if (outlierIndex == 0) {
      x = (C * (E * I - F * H) - B * (F * I - F * G) + C * (F * H - E * G)) / det;
      y = (A * (F * I - F * G) - C * (D * I - F * G) + C * (D * H - E * G)) / det;
    } else if (outlierIndex == 1) {
      x = (C * (E * I - F * H) - B * (F * I - F * G) + C * (F * H - E * G)) / det;
      y = (A * (F * I - F * G) - C * (D * I - F * G) + C * (D * H - E * G)) / det;
    } else if (outlierIndex == 2) {
      x = (C * (E * I - F * H) - B * (F * I - F * G) + C * (F * H - E * G)) / det;
      y = (A * (F * I - F * G) - C * (D * I - F * G) + C * (D * H - E * G)) / det;
    } else if (outlierIndex == 3) {
      x = (C * (E * I - F * H) - B * (F * I - F * G) + C * (F * H - E * G)) / det;
      y = (A * (F * I - F * G) - C * (D * I - F * G) + C * (D * H - E * G)) / det;
    }
  }

  Serial.print("Sound source coordinates: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(") mm");
}
