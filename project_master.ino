#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include <GyverOLED.h>
#include "esp_bt.h"
#include <driver/adc.h>
#include <esp_timer.h>
#include "esp_wifi.h"

#define SOUND_PIN 15  //Пин пьезодинамика
#define PIN_SENSOR 4  // Пин, к которому присоединен датчик вибрации
#define PIN_MIC 32    // Пин, к которому присоединен микрофон
#define PIN_SDA 27    // Дисплей SDA
#define PIN_SCK 14    // Дисплей SCK
#define ADC_CHANNEL     ADC1_CHANNEL_4

#define UDP_PORT 4210           // Порт для UDP синхронизации
#define ESPNOW_WIFI_CHANNEL 1   // канал общения с маяками
#define NUM_SLAVES 3            //количестов маяков
#define SOUND_SPEED 343000      // Скорость звука в мм/с
#define SYNC_INTERVAL 60000000  // Интервал синхронизации времени (1 минута в микросекундах)
#define MAX_ITERATIONS 100      // Максимальное количество итераций
#define TOLERANCE 1.0           // Точность расчета (мм)
#define DATA_TIMEOUT 1000000    // Лимит ожидания данных (1 секунда в микросекундах)

#define SET_STATUS_IDLE 8       //статус бездействие
#define WAIT_FOR_SIGNAL 7       //ожидание звука
#define DO_MAKE_SOUND 2         //издать звук
#define DO_UPDATE_DELAY 3       //обновить расхожение времени в микросекундах с сервером
#define NEED_RECALIBRATE 5      //требуется калибровка местоположения маяков
#define SWITCHING_FOR_SIGNAL 6  //в процессе переключения в режим поиска робота
#define IS_CALCULATING 4        //в процессе расчета
#define TIME_SYNC_STARTED 9     //в процессе синхронизации времени
#define SOUNDSPEED 0.343f       // Скорость звука в мм/мкс
#define SIDE 500.0f             // Длина стороны квадрата в мм
#define MAX_ITERATION 100000    // Максимальное количество итераций (уменьшено для Arduino)

struct Result {
  int x;
  int y;
  float check;
};

typedef struct {
  uint32_t timestamp;
  uint8_t id;
  uint8_t type;
} SlaveData;  //структура данных, получаемых от маяка

typedef struct {
  uint32_t timestamps[NUM_SLAVES + 1] = { 0, 0, 0, 0 };  // +1 для ESP32
  uint8_t ids[NUM_SLAVES + 1] = { 177, 241, 106, 0 };
  bool is_connected[NUM_SLAVES + 1] = { false, false, false, false };
  uint8_t beacons_count = 0;
} SlaveTimings;  //данные о маяках на сервере

typedef struct {
  uint32_t masterTime;
  uint8_t command;
} BeaconCommand;  //структура команд маякам

SlaveTimings slaveTimings;             //данные о маяках
bool allTimingsReceived = false;       //все данные от маяков получены
bool beaconsLocated = false;           //все маяки обнаружены
unsigned long dataRequestTime = 0;     // Время
unsigned long serverSoundTime = 0;     //время получения сигнала сервером
bool calibrationDataReceived = false;  //данные для калибровки получены
bool serverReceiveSound = false;       //сервер получил сигнал
int device_status = SET_STATUS_IDLE;   //текущий статус сервера
int calibrate_step = 0;                //шаг калибровки
int beacons_count = 0;                 //количество маяков, которые обнаружены
int beacons_ready = 0;                 //количество маяков, которые получили сигнал
int dataBeaconReceived = 0;            //кол-во данных от маяков получено
int time_synced = 0;                   //таймеры синхронизированы
unsigned long m = 0;
unsigned long s = 0;  //время получения сигнала сервером

uint8_t beacon[NUM_SLAVES][6] = { { 0x0A, 0xF9, 0xE0, 0x73, 0x73, 0xB1 },    //7566257: Mac 08:F9:E0:73:73:B1
                                  { 0x0A, 0xF9, 0xE0, 0x78, 0x39, 0xF1 },    //7879153: Mac 08:F9:E0:78:39:F1
                                  { 0x0A, 0xF9, 0xE0, 0x6B, 0x9B, 0x6A } };  //7052138: Mac 08:F9:E0:6B:9B:6A

// Координаты устройств (в мм)
float deviceX[NUM_SLAVES + 1] = { 0, 500, 500, 0 };  // x0, x1, x2, x3
float deviceY[NUM_SLAVES + 1] = { 0, 0, 500, 500 };  // y0, y1, y2, y3

struct __attribute__((packed)) TimeSyncPacket {
  uint32_t master_time;
  uint8_t beacon_id;
};

GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;

uint32_t max_noice;
uint32_t last_send;

const char cx[] = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H' };
const int cy[] = { 1, 2, 3, 4, 5, 6, 7, 8 };

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  esp_bt_controller_disable();
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("Error initializing ESP-NOW"));
    draw_serial(F("Error initializing ESP-NOW"));
    return;
  }

  esp_now_peer_info_t peerInfo;
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  for (int i = 0; i < NUM_SLAVES; i++) {
    memcpy(peerInfo.peer_addr, beacon[i], 6);

    // Добавить маяк
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println(F("Не удалось подключить маяк"));
      draw_serial(F("Не удалось подключить маяк"));
    }
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);  //отключить после отладки

  pinMode(SOUND_PIN, OUTPUT);  //переводим пин в режим отправки информации

  oled.init(PIN_SDA, PIN_SCK);
  oled.clear();
  oled.home();
  draw_waiting_for_beacons();  //отображаем на экране ожидание маяков

  digitalWrite(SOUND_PIN, HIGH);  //издаем звук
  delay(100);                     //ожидание 100 мс
  digitalWrite(SOUND_PIN, LOW);   //выключаем звук
}

void loop() {
  //запуск синхронизации таймеров когда все маяки подключены
  if (!time_synced && device_status == SET_STATUS_IDLE) {
    refresh_beacon_connected();
    if (beacons_count == 3) {
      //синхронизация времени
      device_status = TIME_SYNC_STARTED;
      syncTime();
    } else {
      s = adc1_get_raw(ADC_CHANNEL);
      if (s > max_noice) {
        max_noice = s;
        oled.clear();
        oled.home();       // курсор в 0,0
        oled.setScale(2);  // масштаб шрифта (1-4)
        oled.print(beacons_count);
        oled.print(F("; "));
        oled.print(max_noice);
        oled.update();
      }
    }
  } 
  else if (device_status != TIME_SYNC_STARTED) {
    // режим ожидания звука
    if (!serverReceiveSound) {
      s = adc1_get_raw(ADC_CHANNEL);
      if (s > max_noice + 50) {
        // Звук получен сервером
        serverSoundTime = esp_timer_get_time();
        serverReceiveSound = true;
        esp_wifi_start();
      }
    } else {

      //если данные от всех маяков получены, то выводим их на экран
      if (device_status == WAIT_FOR_SIGNAL && dataBeaconReceived == 3) {
        float x, y;
        calculateCoordinates(serverSoundTime, slaveTimings.timestamps[0], slaveTimings.timestamps[1], slaveTimings.timestamps[2], x, y);
        device_status = IS_CALCULATING;
        char ax;
        int ay;
        convertCoordinates(x, y, ax, ay);
        String s = "";
        s += ax;
        s += ay;
        draw_coords(s);
        //switchToSignal();
        device_status = TIME_SYNC_STARTED;
        syncTime();
      } else if (last_send + 5000 < millis()) {
        //switchToSignal();
        device_status = TIME_SYNC_STARTED;
        syncTime();
      }
    }
  }
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  SlaveData data;
  memcpy(&data, incomingData, sizeof(data));

  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    if (slaveTimings.ids[i] == data.id) {
      if (data.type == DO_MAKE_SOUND) {
        dataRequestTime = data.timestamp;
        dataBeaconReceived++;
      } else if (data.type == NEED_RECALIBRATE) {
        slaveTimings.is_connected[i] = true;
      } else {
        slaveTimings.timestamps[i] = data.timestamp;
        dataBeaconReceived++;
      }
    }
  }
}

// Функция, которая возвращает строку по переданному числу
const char *getStatusString(int code) {
  switch (code) {
    case 8:
      return "бездействие";
    case 7:
      return "ожидание сигнала";
    case 1:
      return "ожидание сигнала";
    case 2:
      return "издать сигнал";
    case 3:
      return "синхронизация времени";
    case 5:
      return "требуется калибровка";
    case 6:
      return "идет калибровка";
    case 4:
      return "идет расчет";
    case 9:
      return "начата синхронизация времени";
    default:
      return "неизвестный код";  // Возвращаем "UNKNOWN_CODE" для неизвестных кодов
  }
}

//сброс состояния сервера
void initialState() {
  //ESP_BT.println(F("Сброс"));
  serverReceiveSound = false;
  dataBeaconReceived = 0;
  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    slaveTimings.timestamps[i] = 0;
  }
  beacons_ready = 0;
}
void switchToSignal() {
  initialState();
  BeaconCommand command;
  command.masterTime = esp_timer_get_time();
  command.command = WAIT_FOR_SIGNAL;  // Ожидание звука
  esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));
  esp_now_send(beacon[1], (uint8_t *)&command, sizeof(command));
  esp_now_send(beacon[2], (uint8_t *)&command, sizeof(command));
  device_status = SWITCHING_FOR_SIGNAL;
  device_status = WAIT_FOR_SIGNAL;
  draw_serial(F("Ожидание сигнала робота"));
  esp_wifi_stop();
  last_send = millis();
}


void draw_beacon_coords() {
  oled.clear();
  oled.home();       // курсор в 0,0
  oled.setScale(1);  // масштаб шрифта (1-4)
  String s = "";
  s += deviceX[0];
  s += ", ";
  s += deviceY[0];
  oled.print(s);
  oled.setCursor(0, 1);
  s = "";
  s += deviceX[1];
  s += ", ";
  s += deviceY[1];
  oled.print(s);
  oled.setCursor(0, 2);
  s = "";
  s += deviceX[2];
  s += ", ";
  s += deviceY[2];
  oled.print(s);
  oled.setCursor(0, 3);
  s = "";
  s += deviceX[3];
  s += ", ";
  s += deviceY[3];
  oled.print(s);
  oled.update();
}

void draw_serial(String msg) {
  oled.clear();
  oled.home();       // курсор в 0,0
  oled.setScale(1);  // масштаб шрифта (1-4)
  oled.print(msg);
  oled.update();
}

void draw_coords(String msg) {
  oled.clear();
  oled.fill(255);
  oled.update();
  delay(300);
  oled.fill(0);
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
  oled.setScale(1);  // масштаб шрифта (1-4)
  oled.print(F("Ожидание подключения маяков..."));
  oled.update();
}

void draw_beacons_connected(int c) {
  oled.clear();
  oled.home();       // курсор в 0,0
  oled.setScale(1);  // масштаб шрифта (1-4)
  oled.print(F("Подключено маяков "));
  oled.print(c);
  oled.update();
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  int b = 3;
  if (mac[3] == 0x73) {
    b = 1;
  } else if (mac[3] == 0x78) {
    b = 2;
  }
  if (status == ESP_NOW_SEND_SUCCESS) {
    beacons_ready++;
  } else {
  }
}

void refresh_beacon_connected() {
  int old_beacon_count = beacons_count;
  beacons_count = 0;
  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    if (slaveTimings.is_connected[i]) {
      beacons_count++;
    }
  }
  if (beacons_count != old_beacon_count) {
    draw_beacons_connected(beacons_count);
  }
}

// Функция синхронизации времени
void syncTime() {
  esp_wifi_start();
  TimeSyncPacket packet;
  packet.master_time = esp_timer_get_time();

  // Отправка синхронизации всем маякам
  esp_now_send(beacon[0], (uint8_t *)&packet, sizeof(packet));
  delay(300);
  packet.master_time = esp_timer_get_time();
  esp_now_send(beacon[1], (uint8_t *)&packet, sizeof(packet));
  delay(300);
  packet.master_time = esp_timer_get_time();
  esp_now_send(beacon[2], (uint8_t *)&packet, sizeof(packet));
  delay(300);
  time_synced = 1;
  switchToSignal();
}



Result calcIteration3(float t0, float t1, float t2, float t3) {
  float d1 = (t1 - t0) * SOUNDSPEED;
  float d2 = (t2 - t0) * SOUNDSPEED;
  float d3 = (t3 - t0) * SOUNDSPEED;

  // Вычисление полупериметров
  float p1 = (SIDE + d1 + d2) / 2.0f;
  float p2 = (SIDE + d2 + d3) / 2.0f;

  // Безопасное вычисление площадей
  float s1 = 0.0f;
  float term1 = p1 * (p1 - d1) * (p1 - d2) * (p1 - SIDE);
  if (term1 >= 0) {
    s1 = sqrt(term1);
  } else {
    return { 0, 0, 0.0f };
  }
  float s2 = 0.0f;
  float term2 = p2 * (p2 - d2) * (p2 - d3) * (p2 - SIDE);
  if (term2 >= 0) {
    s2 = sqrt(term2);
  } else {
    return { 0, 0, 0.0f };
  }
  float h1 = 2 * s1 / SIDE;
  float h2 = 2 * s2 / SIDE;

  float check = sqrt(h1 * h1 + h2 * h2) - d2;

  if (h1 > 0 && h2 > 0 && h1 < SIDE && h2 < SIDE && fabs(check) < 50) {
    return { static_cast<int>(round(h1)),
             static_cast<int>(round(h2)),
             check };
  }
  return { 0, 0, 0.0f };
}

Result calc3(float t1, float t2, float t3) {
  float t0 = fminf(t1, fminf(t2, t3)) - 2100;
  float maxt = fmaxf(t1, fmaxf(t2, t3));
  Result best = { 0, 0, 0.0f };

  for (int i = 0; i < MAX_ITERATION && t0 < maxt; i++, t0 += 1) {
    Result current = calcIteration3(t0, t1, t2, t3);
    if (current.x == 0) continue;

    if (best.x == 0 || fabs(current.check) < fabs(best.check)) {
      best = current;
    } else if (best.x != 0 && fabs(current.check) > fabs(best.check)) {
      break;
    }
  }
  return best;
}

void calculateCoordinates(float t1, float t2, float t3, float t4, float &x, float &y) {
  float results[4][2] = { 0 };
  int count = 0;

  // Обработка четырех комбинаций датчиков
  if (Result res = calc3(t1, t2, t3); res.x != 0) {
    results[count][0] = res.x;
    results[count][1] = SIDE - res.y;
    count++;
  }
  if (Result res = calc3(t2, t3, t4); res.x != 0) {
    results[count][0] = SIDE - res.y;
    results[count][1] = SIDE - res.x;
    count++;
  }
  if (Result res = calc3(t3, t4, t1); res.x != 0) {
    results[count][0] = SIDE - res.x;
    results[count][1] = res.y;
    count++;
  }
  if (Result res = calc3(t4, t1, t2); res.x != 0) {
    results[count][0] = res.y;
    results[count][1] = res.x;
    count++;
  }

  // Усреднение результатов
  if (count > 0) {
    float sumX = 0, sumY = 0;
    for (int i = 0; i < count; i++) {
      sumX += results[i][0];
      sumY += results[i][1];
    }
    x = sumX / count;
    y = sumY / count;
  } else {
    x = y = 0;
  }
}
void convertCoordinates(float x, float y, char &ax, int &ay) {
  // Вычисляем индексы с округлением и проверкой границ
  int index = constrain(static_cast<int>(round(x / 50.0f)) - 1, 0, 7);
  ax = cx[index];

  index = constrain(static_cast<int>(round(y / 50.0f)) - 1, 0, 7);
  ay = cy[index];
}
