#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Ticker.h>

#define TIMER_INTERVAL 5  // Минимальный стабильный интервал в микросекундах

volatile uint32_t counter = 0;
Ticker timer;

#define SOUND_PIN D5   //Пин, к которому подсоединен пьезодинамик
#define PIN_MIC A0     // Пин, к которому присоединен микрофон

uint8_t masterAddress[] = { 0x14, 0x2B, 0x2F, 0xC5, 0x5F, 0x44 };  // MAC-адрес ESP32

#define SET_STATUS_IDLE 8   //статус бездействие
#define WAIT_FOR_SIGNAL 7   //ожидание звука
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
int device_status = NEED_RECALIBRATE;
uint32_t timeOffset = 0;  // Смещение времени относительно ESP32
uint32_t s;
uint32_t max_noice;
uint32_t calibration_time;
uint32_t time_fixed;

void IRAM_ATTR timer_isr() {
  if (device_status == WAIT_FOR_SIGNAL) {
    time_fixed = micros();
    s = analogRead(PIN_MIC);  // Быстрая операция в прерывании    
    if (s > max_noice + 50) {      
      device_status = DO_UPDATE_DELAY;
    }
  }
}
void setup() {
  //Serial.begin(115200);
  system_update_cpu_freq(160);
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0) {
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);

  esp_now_register_recv_cb(OnDataRecv);

  myData.id = ESP.getChipId();  // Уникальный ID для каждого устройства
  pinMode(SOUND_PIN, OUTPUT);  //переводим пин в режим отправки информации

  //отправим на сервер сигнал о готовности
  myData.type = NEED_RECALIBRATE;
  esp_now_send(masterAddress, (uint8_t *)&myData, sizeof(myData));

  digitalWrite(SOUND_PIN, HIGH);  //издаем звук
  delay(100);                     //ожидание 100 мс
  digitalWrite(SOUND_PIN, LOW);   //выключаем звук
  calibration_time = millis();
  timer1_attachInterrupt(timer_isr);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(clockCyclesPerMicrosecond() * TIMER_INTERVAL);
}

void loop() {
  if (device_status == DO_UPDATE_DELAY) {
    device_status = SET_STATUS_IDLE;
    myData.timestamp = time_fixed + timeOffset;  // Корректировка времени с учетом смещения
    myData.type = WAIT_FOR_SIGNAL;
    WiFi.mode(WIFI_AP_STA);
    esp_now_send(masterAddress, (uint8_t *)&myData, sizeof(myData));
  }
  if (device_status == NEED_RECALIBRATE && calibration_time + 2000 > millis()) {
    s = analogRead(PIN_MIC);
    if (s > max_noice) {
      max_noice = s;
    }
  }
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len == sizeof(TimeSyncPacket)) {
    // Обработка синхронизации времени
    TimeSyncPacket *sync = (TimeSyncPacket *)incomingData;
    timeOffset = sync->master_time - micros();
  } else if (len == sizeof(BeaconCommand)) {
    BeaconCommand command;
    memcpy(&command, incomingData, sizeof(command));
    if (command.command == WAIT_FOR_SIGNAL) {
      device_status = WAIT_FOR_SIGNAL;
      WiFi.mode(WIFI_OFF);
    }
    if (command.command == SET_STATUS_IDLE) {
      device_status = SET_STATUS_IDLE;
    }
  }
}
