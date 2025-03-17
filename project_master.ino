#include <WiFi.h>
#include <esp_now.h>

#include <math.h>
#include <GyverOLED.h>
#include <VolAnalyzer.h>

#include <BluetoothSerial.h>  //отладка, потом удалить

#define SOUND_PIN 15  //Пин пьезодинамика
#define PIN_SENSOR 4  // Пин, к которому присоединен датчик вибрации
#define PIN_MIC 32    // Пин, к которому присоединен микрофон
#define PIN_SDA 27    // Дисплей SDA
#define PIN_SCK 14    // Дисплей SCK

#define UDP_PORT 4210           // Порт для UDP синхронизации
#define ESPNOW_WIFI_CHANNEL 1   // канал общения с маяками
#define NUM_SLAVES 3            //количестов маяков
#define SOUND_SPEED 343000      // Скорость звука в мм/с
#define SYNC_INTERVAL 60000000  // Интервал синхронизации времени (1 минута в микросекундах)
#define MAX_ITERATIONS 100      // Максимальное количество итераций
#define TOLERANCE 1.0           // Точность расчета (мм)
#define DATA_TIMEOUT 1000000    // Лимит ожидания данных (1 секунда в микросекундах)

#define SET_STATUS_IDLE 8   //статус бездействие
#define WAIT_FOR_SIGNAL 7   //ожидание звука
#define DO_MAKE_SOUND 2     //издать звук
#define DO_UPDATE_DELAY 3   //обновить расхожение времени в микросекундах с сервером
#define NEED_RECALIBRATE 5  //требуется калибровка местоположения маяков
#define IS_CALIBRATING 6    //в процессе калибровки местоположения маяков
#define IS_CALCULATING 4    //в процессе расчета

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
int beacons_located_count = 0;         //количество маяков, которые обнаружены
int dataBeaconReceived = 0;            //кол-во данных от маяков получено

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

VolAnalyzer analyzer(PIN_MIC);
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;

BluetoothSerial ESP_BT;
hw_timer_t *timer = NULL;

void setup() {
  Serial.begin(115200);
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

  pinMode(PIN_SENSOR, INPUT);  //переводим пин в режим получения информации
  pinMode(SOUND_PIN, OUTPUT);  //переводим пин в режим отправки информации

  analyzer.setVolK(20);  //настройка параметров микрофона
  analyzer.setTrsh(10);
  analyzer.setVolMin(10);
  analyzer.setVolMax(100);
  analyzer.setDt(10);
  analyzer.setWindow(10);
  oled.init(PIN_SDA, PIN_SCK);
  oled.clear();
  oled.home();
  draw_waiting_for_beacons();  //отображаем на экране ожидание маяков
  ESP_BT.begin("ESP32_Serial");
  ESP_BT.println("Start ");
  ESP_BT.println(Network.macAddress());
  Serial.println(Network.macAddress());

  // Инициализация таймера
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &detectSound, true);
  timerAlarmWrite(timer, 1000000, true);  // 1 мкс разрешение
  timerStart(timer);
}

void loop() {
  //запуск синхронизации таймеров когда все маяки подключены
  if (!time_synced) {
    refresh_beacon_connected();
    if (beacons_count == 3) {
      //синхронизация времени
      syncTime();
    }
  } else if (!beaconsLocated) {
    //сервер слушает микрофон
    if (device_status == IS_CALIBRATING && !serverReceiveSound && detectSound()) {
      serverSoundTime = micros();
      serverReceiveSound = true;
      ESP_BT.print(F("Звук получен сервером: "));
      ESP_BT.println(serverSoundTime);
    }

    //старт определения координат маяка 1
    if (calibrate_step == 0) {
      start_locate_beacon(0);
    }

    //если получены данные сервером и от маяков, запуск расчета маяка 1
    if (calibrate_step == 1 && serverReceiveSound && device_status == IS_CALIBRATING && dataBeaconReceived) {
      calculateBeaconCoords();
    }

    //старт определения координат маяка 2
    if (calibrate_step == 1 && device_status == SET_STATUS_IDLE) {
      start_locate_beacon(1);
    }

    //если получены данные сервером и от маяков, запуск расчета маяка 2
    if (calibrate_step == 2 && serverReceiveSound && device_status == IS_CALIBRATING && dataBeaconReceived == 2) {
      calculateBeaconCoords();
    }

    //старт определения координат маяка 3
    if (calibrate_step == 2 && device_status == SET_STATUS_IDLE) {
      start_locate_beacon(2);
    }

    //если получены данные сервером и от маяков, запуск расчета маяка 3
    if (calibrate_step == 3 && serverReceiveSound && device_status == IS_CALIBRATING && dataBeaconReceived == 2) {
      calculateBeaconCoords();
    }

    if (calibrate_step == 3 && device_status == SET_STATUS_IDLE) {
      beaconsLocated = true;
      initialState();
    }

  } else {
    // Если маяки рассчитаны, переходим в режим ожидания звука
    if (!serverReceiveSound && detectSound()) {
      // Звук получен сервером
      serverSoundTime = micros();
      serverReceiveSound = true;
      ESP_BT.print(F("Звук получен сервером: "));
      ESP_BT.println(serverSoundTime);
    }

    //mainflow();

    if (serverReceiveSound && device_status == IS_CALIBRATING && dataBeaconReceived == 3) {
      float x = deviceX[3] / 2, y = deviceY[1] / 2;  // Начальное предположение примерно посередине
      calculateSoundSource(x, y);
      String s = "";
      s += x;
      s += ",";
      s += y;
      draw_coords(s);
    }
  }
}

// Функция синхронизации времени
void syncTime() {
  TimeSyncPacket packet;
  packet.master_time = micros();

  // Отправка синхронизации всем маякам
  esp_now_send(beacon[0], (uint8_t *)&packet, sizeof(packet));
  packet.master_time = micros();
  esp_now_send(beacon[1], (uint8_t *)&packet, sizeof(packet));
  packet.master_time = micros();
  esp_now_send(beacon[2], (uint8_t *)&packet, sizeof(packet));
}

void refresh_beacon_connected() {
  beacons_count = 0;
  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    if (slaveTimings.is_connected[i]) {
      beacons_count++;
    }
  }
}

bool detectSound() {
  if (analyzer.tick() && analyzer.getRaw() > 5) {
    return true;
  }
  return false;
}

void mainflow() {
  int beacons_count = 0;
  int timestamp_count = 0;
  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    if (slaveTimings.is_connected[i]) {
      beacons_count++;
    }
    if (slaveTimings.timestamps[i] != 0) {
      timestamp_count++;
    }
  }
  /*ESP_BT.print(F("timestamp_count="));
  ESP_BT.println(timestamp_count);
  ESP_BT.print(F("beacons_count="));
  ESP_BT.println(beacons_count);*/
  if (device_status == IS_CALIBRATING) {
    if (calibrate_step == 0 || calibrate_step == 3) {
      //ESP_BT.print(F("automatic calibration data received"));
      calibrationDataReceived = true;
    } else if (calibrate_step == 1 || calibrate_step == 2) {
      if (timestamp_count) {
        //ESP_BT.println(F("calibration data received"));
        calibrationDataReceived = true;
      }
    }
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
  ESP_BT.print(F("Данные получены от "));
  ESP_BT.println(data.id);

  ESP_BT.print(F("Тип="));
  ESP_BT.println(data.type);
  ESP_BT.print(F("timestamp="));
  ESP_BT.println(data.timestamp);
  ESP_BT.print(F("device_status="));
  ESP_BT.println(device_status);
  ESP_BT.print(F("calibrate_step="));
  ESP_BT.println(calibrate_step);
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

void start_locate_beacon(int beacon_num) {
  int esp_now_err;
  device_status = IS_CALIBRATING;
  String s = F("Locating beacons (");
  s += calibrate_step;
  s += F(")...");
  draw_serial(s);
  ESP_BT.println(s);
  serverReceiveSound = false;
  BeaconCommand command;
  dataBeaconReceived = 0;
  if (calibrate_step == 0) {
    calibrate_step++;
    //первый маяк
    delay(200);
    command.command = DO_MAKE_SOUND;  // Издать звук

    esp_now_err = esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send do sound to beacon 1: "));
    ESP_BT.println(esp_now_err);
  }
  if (calibrate_step == 1) {
    calibrate_step++;
    //второй маяк
    delay(1000);
    command.masterTime = micros();
    command.command = 1;  // Ожидание звука
    esp_now_err = esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send wait for signal to beacon 1: "));
    ESP_BT.println(esp_now_err);
    delay(200);
    //отправка звука
    command.masterTime = micros();
    command.command = DO_MAKE_SOUND;  // Издать звук
    esp_now_err = esp_now_send(beacon[1], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send do sound to beacon 2: "));
    ESP_BT.println(esp_now_err);
  }
  if (calibrate_step == 2) {
    calibrate_step++;
    //третий маяк
    delay(400);
    command.masterTime = micros();
    command.command = WAIT_FOR_SIGNAL;  // Ожидание звука
    esp_now_err = esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send wait for signal to beacon 1: "));
    ESP_BT.println(esp_now_err);
    delay(200);
    //отправка звука
    command.masterTime = micros();
    command.command = DO_MAKE_SOUND;  // Издать звук
    esp_now_err = esp_now_send(beacon[2], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send do sound to beacon 3: "));
    ESP_BT.println(esp_now_err);
  }
  if (calibrate_step == 3) {
    //конец калибровки, переход в рабочий режим
    beaconsLocated = true;
    initialState();
    device_status = SET_STATUS_IDLE;
    ESP_BT.println(F("All beacons located"));
    //draw_serial(F("All beacons located"));
    draw_beacon_coords();
  }
}

// Локализация маяков
void locateBeacons() {
  int esp_now_err;
  device_status = IS_CALIBRATING;  //чтобы не вызвать два раза
  String s = F("Locating beacons (");
  s += calibrate_step;
  s += F(")...");
  draw_serial(s);
  ESP_BT.println(s);
  serverReceiveSound = false;
  BeaconCommand command;
  //на каком шаге мы находимся?
  if (calibrate_step == 0) {
    //первый маяк
    //если маяк подключен, просим его издать сигнал
    if (slaveTimings.is_connected[0]) {
      delay(200);
      command.masterTime = micros();
      command.command = DO_MAKE_SOUND;  // Издать звук

      esp_now_err = esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));
      ESP_BT.print(F("send do sound to beacon 1: "));
      ESP_BT.println(esp_now_err);
    } else {
      device_status = NEED_RECALIBRATE;
    }
  }
  if (calibrate_step == 1) {
    //второй маяк
    delay(1000);
    command.masterTime = micros();
    command.command = 1;  // Ожидание звука
    esp_now_err = esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send wait for signal to beacon 1: "));
    ESP_BT.println(esp_now_err);
    delay(200);
    //отправка звука
    command.masterTime = micros();
    command.command = DO_MAKE_SOUND;  // Издать звук
    esp_now_err = esp_now_send(beacon[1], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send do sound to beacon 2: "));
    ESP_BT.println(esp_now_err);
  }
  if (calibrate_step == 2) {
    //третий маяк
    delay(400);
    command.masterTime = micros();
    command.command = WAIT_FOR_SIGNAL;  // Ожидание звука
    esp_now_err = esp_now_send(beacon[0], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send wait for signal to beacon 1: "));
    ESP_BT.println(esp_now_err);
    delay(200);
    //отправка звука
    command.masterTime = micros();
    command.command = DO_MAKE_SOUND;  // Издать звук
    esp_now_err = esp_now_send(beacon[2], (uint8_t *)&command, sizeof(command));
    ESP_BT.print(F("send do sound to beacon 3: "));
    ESP_BT.println(esp_now_err);
  }
  if (calibrate_step == 3) {
    //конец калибровки, переход в рабочий режим
    beaconsLocated = true;
    initialState();
    device_status = SET_STATUS_IDLE;
    ESP_BT.println(F("All beacons located"));
    //draw_serial(F("All beacons located"));
    draw_beacon_coords();
  }
}

//сброс состояния сервера
void initialState() {
  ESP_BT.println(F("initial state"));
  serverReceiveSound = false;
  dataBeaconReceived = 0;
  for (int i = 0; i < NUM_SLAVES + 1; i++) {
    slaveTimings.timestamps[i] = 0;
  }
}
void switchToSignal() {
  ESP_BT.println(F("switchToSignal"));
  serverReceiveSound = false;
  BeaconCommand command;
  command.masterTime = micros();
  command.command = WAIT_FOR_SIGNAL;  // Ожидание звука
  esp_now_send(NULL, (uint8_t *)&command, sizeof(command));
  device_status = WAIT_FOR_SIGNAL;
  ESP_BT.println(F("Ожидание сигнала робота"));
  //draw_serial(F("Ожидание сигнала робота"));
  draw_beacon_coords();
}

void calculateBeaconCoords() {
  device_status = IS_CALCULATING;
  ESP_BT.println(F("Расчет положения маяка"));
  ESP_BT.print(F("calibrate_step="));
  ESP_BT.println(calibrate_step);
  draw_serial(F("Расчет положения маяка"));
  if (calibrate_step == 0) {
    float dt0 = (serverSoundTime - dataRequestTime) * SOUND_SPEED / 1000000.0;
    deviceX[1] = 0;
    deviceY[1] = dt0;
    String s = F("К=");
    s += dt0;
    draw_serial(s);
    ESP_BT.println(s);
    delay(1000);
    device_status == IS_CALIBRATING;
    return;
  }
  if (calibrate_step == 1) {
    ESP_BT.print(F("serverSoundTime="));
    ESP_BT.println(serverSoundTime);
    ESP_BT.print(F("dataRequestTime="));
    ESP_BT.println(dataRequestTime);
    float dt0 = (serverSoundTime - dataRequestTime) * SOUND_SPEED / 1000000.0;
    ESP_BT.println(dt0);
    ESP_BT.print("slaveTimings.timestamps[0]=");
    ESP_BT.println(slaveTimings.timestamps[0]);
    float dt1 = (slaveTimings.timestamps[0] - dataRequestTime) * SOUND_SPEED / 1000000.0;
    ESP_BT.println(dt1);
    float p1 = (dt0 + dt1 + deviceY[1]) / 2;
    ESP_BT.println(p1);
    float Sq1 = sqrt(p1 * (p1 - dt0) * (p1 - dt1) * (p1 - deviceY[1]));
    ESP_BT.println(Sq1);
    float h1 = Sq1 / deviceY[1];
    ESP_BT.println(h1);
    deviceX[2] = h1;
    deviceY[2] = sqrt(dt0 * dt0 - h1 * h1);
    delay(1000);
    device_status == IS_CALIBRATING;
    return;
  }
  if (calibrate_step == 2) {
    ESP_BT.print(F("serverSoundTime="));
    ESP_BT.println(serverSoundTime);
    ESP_BT.print(F("dataRequestTime="));
    ESP_BT.println(dataRequestTime);
    float dt0 = (serverSoundTime - dataRequestTime) * SOUND_SPEED / 1000000.0;
    ESP_BT.println(dt0);
    ESP_BT.print("slaveTimings.timestamps[0]=");
    ESP_BT.println(slaveTimings.timestamps[0]);
    float dt1 = (slaveTimings.timestamps[0] - dataRequestTime) * SOUND_SPEED / 1000000.0;
    ESP_BT.println(dt1);
    float p1 = (dt0 + dt1 + deviceY[1]) / 2;
    ESP_BT.println(p1);
    float Sq1 = sqrt(p1 * (p1 - dt0) * (p1 - dt1) * (p1 - deviceY[1]));
    ESP_BT.println(Sq1);
    float h1 = Sq1 / deviceY[1];
    ESP_BT.println(h1);
    deviceX[3] = h1;
    deviceY[3] = sqrt(dt0 * dt0 - h1 * h1);
    delay(1000);
    device_status == IS_CALIBRATING;
  }
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

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    ESP_BT.print(F("Command sent successfully"));
    ESP_BT.printf(" to %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    ESP_BT.print(F("Command send failed ("));
    ESP_BT.print(status);
    ESP_BT.printf(") to %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
}
