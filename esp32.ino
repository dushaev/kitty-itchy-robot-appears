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

typedef struct {
  uint8_t command; // 1 - издать звук, 0 - остановить
} BeaconCommand;

SlaveTimings slaveTimings;
bool allTimingsReceived = false;
unsigned long lastSyncTime = 0;
bool beaconsLocated = false;
float beaconCoordinates[NUM_SLAVES][2]; // Координаты маяков (x, y)

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
  esp_now_register_send_cb(OnDataSent);
  lastSyncTime = millis();
}

void loop() {
  // Синхронизация времени каждую минуту
  if (millis() - lastSyncTime >= SYNC_INTERVAL) {
    syncTime();
    lastSyncTime = millis();
  }

  // Если маяки еще не локализованы, локализуем их
  if (!beaconsLocated && allTimingsReceived) {
    locateBeacons();
    allTimingsReceived = false;
  }

  // Если маяки локализованы, переходим в режим ожидания звука
  if (beaconsLocated) {
    // Режим ожидания звука (можно добавить логику)
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

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Command sent successfully");
  } else {
    Serial.println("Command send failed");
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
void calculateSoundSource(float &x, float &y) {
  uint32_t t0 = slaveTimings.timestamps[0];
  uint32_t t1 = slaveTimings.timestamps[1];
  uint32_t t2 = slaveTimings.timestamps[2];
  uint32_t t3 = slaveTimings.timestamps[3]; // Данные с ESP32

  // Вычисление разницы во времени
  float dt0 = (t0 - t0) * SOUND_SPEED / 1000000.0;
  float dt1 = (t1 - t0) * SOUND_SPEED / 1000000.0;
  float dt2 = (t2 - t0) * SOUND_SPEED / 1000000.0;
  float dt3 = (t3 - t0) * SOUND_SPEED / 1000000.0;

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
}

// Локализация маяков
void locateBeacons() {
  for (int i = 0; i < NUM_SLAVES; i++) {
    // Отправка команды маяку на издание звука
    BeaconCommand command;
    command.command = 1; // Издать звук
    uint8_t beaconAddress[] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56}; // Замените на MAC-адрес маяка
    esp_now_send(beaconAddress, (uint8_t *)&command, sizeof(command));

    // Ожидание данных от всех устройств
    while (!allTimingsReceived) {
      delay(10);
    }

    // Расчет координат маяка
    float x = 250.0, y = 250.0; // Начальное предположение
    calculateSoundSource(x, y);

    // Запоминание координат маяка
    beaconCoordinates[i][0] = x;
    beaconCoordinates[i][1] = y;

    Serial.print("Beacon ");
    Serial.print(i);
    Serial.print(" coordinates: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(") mm");

    // Остановка звука на маяке
    command.command = 0; // Остановить звук
    esp_now_send(beaconAddress, (uint8_t *)&command, sizeof(command));

    // Сброс флага для следующего маяка
    allTimingsReceived = false;
  }

  // Все маяки локализованы
  beaconsLocated = true;
  Serial.println("All beacons located");
}
