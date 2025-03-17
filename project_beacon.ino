#include <ESP8266WiFi.h>
#include <espnow.h>
#include <VolAnalyzer.h>

#define UDP_PORT 4210  // Порт для UDP синхронизации
#define SOUND_PIN D5   //Пин, к которому подсоединен пьезодинамик
#define PIN_SENSOR D1  // Пин, к которому присоединен датчик вибрации
#define PIN_MIC A0     // Пин, к которому присоединен микрофон

#define SOUND_SPEED 343000  // Скорость звука в мм/с

uint8_t masterAddress[] = { 0x14, 0x2B, 0x2F, 0xC5, 0x5F, 0x44 };  // MAC-адрес ESP32

#define SET_STATUS_IDLE 8   //статус бездействие
#define WAIT_FOR_SIGNAL 1   //ожидание звука
#define DO_MAKE_SOUND 2     //издать звук
#define DO_UPDATE_DELAY 3   //обновить расхожение времени в микросекундах с сервером
#define NEED_RECALIBRATE 5  //маяк был передвинут

typedef struct {
  uint32_t timestamp;
  uint8_t id;
  uint8_t type;  //тип сообщения - получение звука или издание звука
} SlaveData;

typedef struct {
  uint32_t masterTime;
  uint8_t command;
} BeaconCommand;

struct __attribute__((packed)) TimeSyncPacket {
  uint32_t master_time;
  uint8_t beacon_id;
};

SlaveData myData;
VolAnalyzer analyzer(PIN_MIC);
int device_status = SET_STATUS_IDLE;
uint32_t timeOffset = 0;  // Смещение времени относительно ESP32

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0) {
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  myData.id = ESP.getChipId();  // Уникальный ID для каждого устройства
  Serial.print("my id=");
  Serial.println(myData.id);
  pinMode(PIN_SENSOR, INPUT);  //переводим пин в режим получения информации
  pinMode(SOUND_PIN, OUTPUT);  //переводим пин в режим отправки информации

  analyzer.setVolK(20);  //настройка параметров микрофона
  analyzer.setTrsh(10);
  analyzer.setVolMin(10);
  analyzer.setVolMax(100);
  analyzer.setDt(10);
  analyzer.setWindow(10);

  //отправим на сервер сигнал о готовности
  myData.type = NEED_RECALIBRATE;
  esp_now_send(masterAddress, (uint8_t *)&myData, sizeof(myData));
}

void loop() {
  if (device_status == WAIT_FOR_SIGNAL && detectSound()) {
    device_status = SET_STATUS_IDLE;
    myData.timestamp = micros() + timeOffset;  // Корректировка времени с учетом смещения
    myData.type = WAIT_FOR_SIGNAL;
    delay(100);
    esp_now_send(masterAddress, (uint8_t *)&myData, sizeof(myData));
    delay(100);
  }
}

bool detectSound() {
  if (analyzer.tick() && analyzer.getRaw() > 3) {
    Serial.println("Sound event!");
    return true;
  }
  return false;
}

void OnDataSent(uint8_t *mac, uint8_t status) {
  Serial.print(F("Данные получены от "));
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);

  Serial.println(macStr);
  Serial.print(F("status="));
  Serial.println(status);
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  Serial.print(F("Данные получены от "));
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);

  Serial.println(macStr);

  if (len == sizeof(TimeSyncPacket)) {
    // Обработка синхронизации времени
    TimeSyncPacket *sync = (TimeSyncPacket *)data;
    uint32_t local_time = micros();
    timeOffset = sync->master_time - local_time;
    Serial.print(F("Обработка синхронизации времени"));
    Serial.print(F("timeOffset="));
    Serial.println(timeOffset);
  } else if (len == sizeof(BeaconCommand)) {
    BeaconCommand command;
    memcpy(&command, incomingData, sizeof(command));
    Serial.print(F("Тип="));
    Serial.println(command.command);
    if (command.command == DO_MAKE_SOUND) {
      device_status = SET_STATUS_IDLE;
      delay(1000);
      myData.timestamp = micros() + timeOffset;  // Запись времени издания звука + корректировка
      digitalWrite(SOUND_PIN, HIGH);             //издаем звук
      delay(100);                                //ожидание 100 мс
      digitalWrite(SOUND_PIN, LOW);              //выключаем звук
      myData.type = DO_MAKE_SOUND;               //этот маяк издал звук
      esp_now_send(masterAddress, (uint8_t *)&myData, sizeof(myData));
    }
    if (command.command == WAIT_FOR_SIGNAL) {
      device_status = WAIT_FOR_SIGNAL;
    }
    if (command.command == SET_STATUS_IDLE) {
      device_status = SET_STATUS_IDLE;
    }
  }
}
