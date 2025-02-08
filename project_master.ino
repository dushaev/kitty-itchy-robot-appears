#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include <GyverOLED.h>
#include <VolAnalyzer.h>

#define SOUND_PIN 16  //Пин пьезодинамика
#define PIN_SENSOR 4  // Пин, к которому присоединен датчик вибрации
#define PIN_MIC 32    // Пин, к которому присоединен микрофон
#define PIN_SDA 27    // Дисплей SDA
#define PIN_SCK 14    // Дисплей SCK

#define NUM_SLAVES 3
#define SOUND_SPEED 343000      // Скорость звука в мм/с
#define SYNC_INTERVAL 60000000  // Интервал синхронизации времени (1 минута в микросекундах)
#define MAX_ITERATIONS 100      // Максимальное количество итераций
#define TOLERANCE 1.0           // Точность расчета (мм)
#define DATA_TIMEOUT 1000000    // Лимит ожидания данных (1 секунда в микросекундах)

#define SET_STATUS_IDLE 0   //статус бездействие
#define WAIT_FOR_SIGNAL 1   //ожидание звука
#define DO_MAKE_SOUND 2     //издать звук
#define DO_UPDATE_DELAY 3   //обновить расхожение времени в микросекундах с сервером
#define NEED_RECALIBRATE 5  //маяк был передвинут
#define IS_CALIBRATING 6

typedef struct {
  uint32_t timestamp;
  uint8_t id;
  uint8_t type;
} SlaveData;

typedef struct {
  uint32_t timestamps[NUM_SLAVES + 1];  // +1 для ESP32
  uint8_t ids[NUM_SLAVES + 1] = { 7566257, 7879153, 7052138, 0 };
  bool is_connected[NUM_SLAVES + 1] = { false, false, false, false };
  uint8_t beacons_count = 0;
} SlaveTimings;

typedef struct {
  uint32_t masterTime;
  uint8_t command;
} BeaconCommand;

SlaveTimings slaveTimings;          //данные о маяках
bool allTimingsReceived = false;    //все данные от маяков получены
bool beaconsLocated = false;        //все маяки обнаружены
unsigned long dataRequestTime = 0;  // Время
unsigned long serverSoundTime = 0;
bool calibrationDataReceived = false;
bool serverReceiveSound = false;
int device_status = SET_STATUS_IDLE;
int calibrate_step = 0;
int beacons_located_count = 0;                                //количество маяков, которые обнаружены
uint32_t beacon_location_timestamps[NUM_SLAVES][NUM_SLAVES];  //данные по всем таймингам

uint8_t beacon[NUM_SLAVES][6] = { { 0x08, 0xF9, 0xE0, 0x73, 0x73, 0xB1 },    //7566257: Mac 08:F9:E0:73:73:B1
                                  { 0x08, 0xF9, 0xE0, 0x78, 0x39, 0xF1 },    //7879153: Mac 08:F9:E0:78:39:F1
                                  { 0x08, 0xF9, 0xE0, 0x6B, 0x9B, 0x6A } };  //7052138: Mac 08:F9:E0:6B:9B:6A

// Координаты устройств (в мм)
float deviceX[NUM_SLAVES + 1] = { 0, 500, 500, 0 };  // x0, x1, x2, x3
float deviceY[NUM_SLAVES + 1] = { 0, 0, 500, 500 };  // y0, y1, y2, y3

VolAnalyzer analyzer(PIN_MIC);
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  esp_now_add_peer(beacon1, ESP_NOW_ROLE_COMBO, 2, NULL, 0);
  esp_now_add_peer(beacon2, ESP_NOW_ROLE_COMBO, 3, NULL, 0);
  esp_now_add_peer(beacon3, ESP_NOW_ROLE_COMBO, 4, NULL, 0);

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent); //отключить после отладки

  pinMode(PIN_SENSOR, INPUT);  //переводим пин в режим получения информации
  pinMode(SOUND_PIN, OUTPUT);  //переводим пин в режим отправки информации

  analyzer.setVolK(20);  //настройка параметров микрофона
  analyzer.setTrsh(10);
  analyzer.setVolMin(10);
  analyzer.setVolMax(100);
  analyzer.setDt(10);
  analyzer.setWindow(10);

  draw_waiting_for_beacons();  //отображаем на экране ожидание маяков
}

void loop() {

  // Если координаты маяков еще не рассчитаны, рассчитаем их
  if (!beaconsLocated && device_status == NEED_RECALIBRATE) {
    calibrate_step = 0;
    locateBeacons();
  }

  // Если маяки рассчитаны, переходим в режим ожидания звука
  if ((beaconsLocated || device_status == IS_CALIBRATING) && detectSound()) {
    // Звук получен сервером
    serverSoundTime = micros();
    serverReceiveSound = true;
    Serial.println(F("Звук получен сервером"));
  }
  if (calibrationDataReceived && serverReceiveSound && device_status == IS_CALIBRATING) {
    calculateBeaconCoords();
  }
  elseif(serverReceiveSound && allTimingsReceived) {
    float x = deviceX[3] / 2, y = deviceY[1] / 2;  // Начальное предположение примерно посередине
    calculateSoundSource(x, y);
    String s = "";
    s += x;
    s += ",";
    s += y;
    draw_coords(s);
  }
  if (device_status==SET_STATUS_IDLE) {
    delay(1000);
    switchToSignal();
  }
}

bool detectSound() {
  if (analyzer.tick() && analyzer.getRaw() > 3) {
    return true;
  }
  return false;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  SlaveData data;
  memcpy(&data, incomingData, sizeof(data));

  uint8_t beacons_count = 0;
  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    if (slaveTimings.ids[i] == data.id) {
      if (data.type == DO_MAKE_SOUND) {
        dataRequestTime = data.timestamp;
      }
      elseif(data.type == NEED_RECALIBRATE) {
        slaveTimings.is_connected[i] = true;
      }
      else {
        slaveTimings.timestamps[i] = data.timestamp;
      }
      if (slaveTimings.beacons_count == NUM_SLAVES) {
        break;
      }
    }
    if (slaveTimings.is_connected[i]) {
      beacons_count++;
    }
  }
  if (device_status == IS_CALIBRATING && calibrate_step >= beacons_count) {
    calibrationDataReceived = true;
  }
  if (slaveTimings.beacons_count != NUM_SLAVES) {
    slaveTimings.beacons_count = beacons_count;  //обновляем количество подключенных маяков
    if (beacons_count == NUM_SLAVES) {
      //запускаем калибровку маяков по одному
      device_status = NEED_RECALIBRATE;
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
    Serial.println(F("Command sent successfully"));
  } else {
    Serial.println(F("Command send failed"));
  }
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
  uint32_t t3 = serverSoundTime;  // Данные с ESP32

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
      float error = dist - ((i == 0) ? dt0 : (i == 1) ? dt1
                                           : (i == 2) ? dt2
                                                      : dt3);

      gradX += (x - deviceX[i]) / dist * error;
      gradY += (y - deviceY[i]) / dist * error;
    }

    // Коррекция координат
    x -= 0.1 * gradX;  // Шаг обучения
    y -= 0.1 * gradY;

    // Проверка на сходимость
    if (sqrt(gradX * gradX + gradY * gradY) < TOLERANCE) {
      break;
    }
  }
}

// Локализация маяков
void locateBeacons() {
  device_status = IS_CALIBRATING;  //чтобы не вызвать два раза
  //на каком шаге мы находимся?
  if (calibrate_step == 0) {
    //первый маяк
    //если маяк подключен, просим его издать сигнал
    if (slaveTimings.is_connected[0]) {
      command.masterTime = micros();
      command.command = DO_MAKE_SOUND;  // Издать звук
      esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));
    } else {
      device_status = NEED_RECALIBRATE;
    }
  }
  if (calibrate_step == 1) {
    //второй маяк
    BeaconCommand command;
    command.masterTime = micros();
    command.command = WAIT_FOR_SIGNAL;  // Ожидание звука
    esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));

    //отправка звука
    command.masterTime = micros();
    command.command = DO_MAKE_SOUND;  // Издать звук
    esp_now_send(beacon[1], (uint8_t *)&command, sizeof(command));
  }
  if (calibrate_step == 2) {
    //третий маяк
    BeaconCommand command;
    command.masterTime = micros();
    command.command = WAIT_FOR_SIGNAL;  // Ожидание звука
    esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));

    //отправка звука
    command.masterTime = micros();
    command.command = DO_MAKE_SOUND;  // Издать звук
    esp_now_send(beacon[2], (uint8_t *)&command, sizeof(command));
  }
  if (calibrate_step == 3) {
    //конец калибровки, переход в рабочий режим
    beaconsLocated = true;
    initialState();
    device_status = SET_STATUS_IDLE;
    Serial.println(F("All beacons located"));
  }
}

//сброс состояния сервера
void initialState() {
  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    slaveTimings.timestamps[i] = 0;
  }
}
void switchToSignal() {
    BeaconCommand command;
    command.masterTime = micros();
    command.command = WAIT_FOR_SIGNAL;  // Ожидание звука
    esp_now_send(NULL, (uint8_t *)&command, sizeof(command));
    device_status = WAIT_FOR_SIGNAL;
    Serial.println(F("Ожидание сигнала робота"));
}

void calculateBeaconCoords() {
  if (calibrate_step == 0) {
    float dt0 = (serverSoundTime - dataRequestTime) * SOUND_SPEED / 1000000.0;
    deviceX[1] = 0;
    deviceY[1] = dt0;
    calibrate_step++;
    return;
  }
  if (calibrate_step == 1) {
    float dt0 = (serverSoundTime - dataRequestTime) * SOUND_SPEED / 1000000.0;
    float dt1 = (slaveTimings.timestamps[0] - dataRequestTime) * SOUND_SPEED / 1000000.0;
    float p1 = (dt0 + dt1 + deviceY[1]) / 2;
    float Sq1 = sqrt(p1 * (p1 - dt0) * (p1 - dt1) * (p1 - deviceY[1]));
    float h1 = Sq1 / deviceY[1];
    deviceX[2] = h1;
    deviceY[2] = sqrt(dt0 * dt0 - h1 * h1);
    calibrate_step++;
    return;
  }
  if (calibrate_step == 2) {
    float dt0 = (serverSoundTime - dataRequestTime) * SOUND_SPEED / 1000000.0;
    float dt1 = (slaveTimings.timestamps[0] - dataRequestTime) * SOUND_SPEED / 1000000.0;
    float p1 = (dt0 + dt1 + deviceY[1]) / 2;
    float Sq1 = sqrt(p1 * (p1 - dt0) * (p1 - dt1) * (p1 - deviceY[1]));
    float h1 = Sq1 / deviceY[1];
    deviceX[3] = h1;
    deviceY[3] = sqrt(dt0 * dt0 - h1 * h1);
    calibrate_step++;
  }
}

void draw_coords(char *msg) {
  oled.clear();
  oled.home();       // курсор в 0,0
  oled.setScale(1);  // масштаб шрифта (1-4)
  oled.print(F("Робот обнаружен:"));
  oled.setCursor(0, 2);
  oled.setScale(4);  // масштаб шрифта (1-4)
  oled.print(msg);
  oled.update();
}

void draw_waiting_for_beacons() {
  oled.clear();
  oled.home();       // курсор в 0,0
  oled.setScale(2);  // масштаб шрифта (1-4)
  oled.print(F("Ожидание подключения маяков..."));
  oled.update();
}
