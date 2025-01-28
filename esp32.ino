#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

#define NUM_SLAVES 3
#define SOUND_SPEED 343000 // Скорость звука в мм/с
#define SYNC_INTERVAL 60000 // Интервал синхронизации времени (1 минута)
#define MAX_ITERATIONS 100 // Максимальное количество итераций
#define TOLERANCE 1.0 // Точность расчета (мм)

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

// Координаты устройств (в мм)
float deviceX[NUM_SLAVES + 1] = {0, 500, 500, 0}; // x0, x1, x2, x3
float deviceY[NUM_SLAVES + 1] = {0, 0, 500, 500}; // y0, y1, y2, y3

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

// Функция для расчета расстояния между двумя точками
float distance(float x1, float y1, float x2, float y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// Итеративный метод для уточнения координат источника звука
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

  // Начальное предположение для координат источника звука
  float x = 250.0; // Центр квадрата
  float y = 250.0;

  // Итеративный метод наименьших квадратов
  for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
    float gradX = 0.0, gradY = 0.0;

    // Вычисление градиента
    for (int i = 0; i < NUM_SLAVES + 1; i++) {
      float dist = distance(x, y, deviceX[i], deviceY[i]);
      float error = dist - ((i == 0) ? dt0 : (i == 1) ? dt1 : (i == 2) ? dt2 : dt3);

      gradX += (x - deviceX[i]) / dist * error;
      gradY += (y - deviceY[i]) / dist * error;
    }

    // Коррекция координат
    x -= 0.1 * gradX; // Шаг обучения
    y -= 0.1 * gradY;

    // Проверка на сходимость
    if (sqrt(gradX * gradX + gradY * gradY) < TOLERANCE) {
      break;
    }
  }

  Serial.print("Sound source coordinates: (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.println(") mm");
}
