#define UDP_PORT 8888  //порт UTP
const char *softAP_ssid = "K.I.R.A.";
const char *password = "kittyitchy";
volatile bool button_pressed = false;     //кнопка была нажата
const char *udpServerIP = "192.168.4.1";  //адрес сервера по умолчанию
uint32_t ms, ms1 = 0, ms2 = 0;            //переменные для таймера
int is_display_avaliable;
int max_mic;
bool is_mic_calibration;
const int wdtTimeout = 5000;  // время в мсек для калибровки уровня шума
uint32_t ESP_ID;
String ESP_UID;
// Переменные для таймера
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;
#include "VolAnalyzer.h"
uint64_t t;
bool sound_waiting = 0;

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>  //Ticker Library
Ticker blinker;
#define BTN_PIN D6
#define SOUND_PIN D5
#define PIN_SENSOR D1   // Пин, к которому присоединен датчик вибрации
#define PIN_MIC A0      // Пин, к которому присоединен микрофон
#define PIN_MIC_DGT D8  //для прерывания
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];
WiFiUDP udp;
#endif
// дельта времени с сервером
uint64_t DELTA;

#ifdef ESP32
#include <WiFi.h>
#include "AsyncUDP.h"
#include <DNSServer.h>
#include <GyverOLED.h>
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
#define NUM_BEACON 3  //максимальное количество маяков
WiFiServer server(80);
uint32_t ips[NUM_BEACON] = { 0, 0, 0 };
int beacon_count = 0;           //количество подключенных маяков
int beacon_answered_count = 0;  // количество маяков, отправивших ответ
int connections = 0;
#define BTN_PIN 17
#define SOUND_PIN 16
#define PIN_SENSOR 4    // Пин, к которому присоединен датчик вибрации
#define PIN_MIC 32      // Пин, к которому присоединен микрофон
#define PIN_MIC_DGT 21  //для прерывания
#define PIN_SDA 27      // Дисплей SDA
#define PIN_SCK 14      // Дисплей SCK
AsyncUDP udp;
WiFiUDP udp2;
// Массив для хранения данных от устройств
uint64_t deviceData[NUM_BEACON];  // [deviceIndex][dataIndex]
uint64_t serverData = 0;
hw_timer_t *timer = NULL;
uint64_t lastSoundTime;
bool calculating_robot_coords = 0;
#endif

#define DO_MAKE_SOUND 2               //издать звук
#define DO_UPDATE_DELAY 3             //обновить расхожение времени в микросекундах с сервером
#define SOUND_TIME_RECEIVED 4         //получено время звука
#define NEED_RECALIBRATE 5            //маяк был передвинут
#define SET_STATUS_IDLE 6             //статус бездействие
#define COMMAND_RECEIVED 7            //команда полученао подключение к wifi сервера
#define SET_STATUS_WAIT_FOR_SIGNAL 1  //ожидание звука

VolAnalyzer analyzer(PIN_MIC);

#ifdef ESP8266
boolean wifi_connect_started = false;  //запущен

boolean wifi_connected = false;  //текущий статус подключения wifi
#endif
#ifdef ESP32
//событие подключения маяка
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  //запомним подключенный маяк
  Serial.print("beacon_count=");
  Serial.println(beacon_count);
  beacon_count += 1;
  ips[beacon_count - 1] = info.got_ip.ip_info.ip.addr;
  Serial.print("Маяк подключен IP = ");
  Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
  connections = WiFi.softAPgetStationNum();
  showConnectionsCount();
  //отправим дельту времени
  String s = String(3) + "," + String(mtime());
  udp.broadcast(s.c_str());
}
#endif
//отображение количества подключений на экране
void showConnectionsCount() {
#ifdef ESP32
  char data[32];
  sprintf(data, "B: %d", connections);
  draw_message(data);
#endif
}
void draw_message(char *msg) {
#ifdef ESP32
  oled.setCursor(0, 0);
  oled.setScale(1);  // масштаб шрифта (1-4)
  oled.print(msg);   // печатай что угодно
#endif
}

#ifdef ESP8266
//подключение маяка к серверу
void connectBeaconToServer() {
  WiFi.mode(WIFI_STA);                  // Выбираем режим клиента для сетевого подключения по Wi-Fi
  WiFi.hostname("K.I.R.A.");            // Указываеем имя хоста, который будет отображаться в Wi-Fi роутере, при подключении устройства
  if (WiFi.status() != WL_CONNECTED) {  // Инициализируем подключение, если устройство ещё не подключено к сети
    WiFi.begin(softAP_ssid, password);
    wifi_connect_started = true;
  }
}
#endif
//создание wifi точки доступа на сервере
void wifi_init() {
// если esp32 то инициируем сервер
#ifdef ESP32
  WiFi.mode(WIFI_AP);
  WiFi.softAP(softAP_ssid, password);
  WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  IPAddress ip_address = WiFi.softAPIP();  // IP Address of our accesspoint
  // Start web server
  server.begin();
  if (udp.listen(UDP_PORT)) {
    Serial.print("UDP сервер ждет сообщений на IP: ");
    Serial.println(IPAddress(WiFi.softAPIP()));
    udp.onPacket([](AsyncUDPPacket packet) {
      Serial.print("UDP тип сообщения: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast"
                                                                             : "Unicast");
      Serial.print(", От: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", Для: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Длина: ");
      Serial.print(packet.length());
      Serial.print(", Данные: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      //реакция на сообщение
      // если пакет принят, то распаковываем его в строку
      char buffer[100];
      for (int i = 0; i < packet.length(); i++) {
        buffer[i] = (char)packet.data()[i];
      }
      buffer[packet.length()] = '\0';  // добавляем конец строки
      server_command(buffer, packet.remoteIP());
      Serial.println("server command end");
      //delete[] packet.length();
      //packet.printf("Got %u bytes of data", packet.length());
    });
  }
#endif
#ifdef ESP8266
  connectBeaconToServer();
#endif
}


IRAM_ATTR void button_click() {
  button_pressed = true;
}
//функция, вызываемая когда получен звук
/*IRAM_ATTR void sound_handler() {
  //int mic = analogRead(PIN_MIC);
  //if (mic > max_mic + 400) {
#ifdef ESP8266
    uint64_t t;
    t = mtime() + DELTA;
    //Получен звуковой сигнал от робота. Передаем на сервер время срабатывания
    udp.beginPacket(udpServerIP, UDP_PORT);
    udp.printf("%d,%d", SOUND_TIME_RECEIVED, t);
    udp.endPacket();
#endif

#ifdef ESP32
    lastSoundTime = mtime();
#endif
    //отключаем микрофон
    detachInterrupt(PIN_MIC_DGT);
  //}
}*/
/*#ifdef ESP32
void ARDUINO_ISR_ATTR endOfMicCalibration() {
  // калибровка уровня шума завершена
  Serial.println("Уровень шума:");
  Serial.println(max_mic);
  is_mic_calibration = false;
}
#endif
#ifdef ESP8266
void endOfMicCalibration() {
  // калибровка уровня шума завершена
  Serial.println("Уровень шума:");
  Serial.println(max_mic);
  is_mic_calibration = false;
  blinker.detach();
  connectBeaconToServer();
}
#endif*/
//выполнение команд на сервере
void server_command(char *packetBuffer, IPAddress ip) {
  Serial.println("server command");
#ifdef ESP32
  String c(packetBuffer[0]);
  int command = c.toInt();
  Serial.print("command=");
  Serial.println(command);
  //получено время сигнала от маяка
  if (command == SOUND_TIME_RECEIVED) {
    Serial.println("sound time received");
    for (int i = 0; i < beacon_count; i++) {
      if (IPAddress(ips[i]) == ip) {
        deviceData[i] = convertCharToUint64(packetBuffer);
        Serial.println(packetBuffer);
        beacon_answered_count += 1;
      }
    }
    if (beacon_answered_count == beacon_count && calculating_robot_coords) {
      //все маяки ответили, делаем расчет
      calculateRobotCoords();
    }
  }
  //маяк был передвинут, необходимо перекалибровка маяков
  if (command == NEED_RECALIBRATE) {
    server_need_recalibrate(packetBuffer, ip);
    calculating_robot_coords = false;
    beacon_answered_count = 0;
  }
  if (command == SET_STATUS_WAIT_FOR_SIGNAL) {
    Serial.println("set status wait for signal");
    //включить прерывание от микрофона
    //detachInterrupt(PIN_MIC_DGT);
    //attachInterrupt(PIN_MIC_DGT, sound_handler, RISING);
    sound_waiting = true;
    beacon_answered_count = 0;
  }

//команда получена
#endif
}
#ifdef ESP32
void server_need_recalibrate(char *packetBuffer, IPAddress ip) {
  Serial.println("need recalibrate");
  //обнуляем массив с ответами маяков
  for (int i = 0; i < beacon_count; i++) {
    deviceData[i] = 0;
  }
  //обнуляем количество ответивших маяков
  beacon_answered_count = 0;
  lastSoundTime = 0;
  String s = String(SET_STATUS_WAIT_FOR_SIGNAL) + ",0";
  delay(3000);
  udp.broadcast(s.c_str());
  delay(1000);
  udp2.beginPacket(ip, UDP_PORT);
  udp2.printf("%d,%d", DO_MAKE_SOUND, 0);
  udp2.endPacket();
}
#endif

//выполнение команд на маяке
void do_command(char *packetBuffer) {
#ifdef ESP8266
  Serial.println("beacon command");
  String c(packetBuffer[0]);
  int command = c.toInt();
  Serial.println(command);
  //ожидание звука
  if (command == SET_STATUS_WAIT_FOR_SIGNAL) {
    Serial.println("set status wait for signal");
    //включить прерывание от микрофона
    //detachInterrupt(PIN_MIC_DGT);
    //attachInterrupt(PIN_MIC_DGT, sound_handler, RISING);
    sound_waiting = true;
  }
  //издать звук
  if (command == DO_MAKE_SOUND) {
    Serial.println("DO_MAKE_SOUND");
    Serial.println(packetBuffer);
    digitalWrite(SOUND_PIN, HIGH);
    delay(100);
    digitalWrite(SOUND_PIN, LOW);
  }
  //обновить расхожение времени в микросекундах с сервером
  if (command == DO_UPDATE_DELAY) {
    Serial.println("DO_UPDATE_DELAY");
    Serial.println(packetBuffer);
    DELTA = convertCharToUint64(packetBuffer) - mtime();
  }
  //статус бездействие
  if (command == SET_STATUS_IDLE) {
    Serial.println("SET_STATUS_IDLE");
    Serial.println(packetBuffer);
    calculating_robot_coords = false;
    //detachInterrupt(PIN_MIC);
  }
  udp.beginPacket(udpServerIP, UDP_PORT);
  udp.printf("%d,%c", COMMAND_RECEIVED, packetBuffer[0]);
  udp.endPacket();
#endif
}
uint64_t convertCharToUint64(char *str) {
  uint64_t result = 0;
  int len = strlen(str);
  for (int i = 1; i < len; i++) {
    result = (result << 8) | (unsigned char)str[i];
  }
  return result;
}
void print_uint64_t(uint64_t num) {
  Serial.print(string_uint64_t(num));
}
String string_uint64_t(uint64_t num) {
  String s;
  char rev[128];
  char *p = rev + 1;
  while (num > 0) {
    *p++ = '0' + (num % 10);
    num /= 10;
  }
  p--;
  /*Print the number which is now in reverse*/
  while (p > rev) {
    s += *p--;
  }
  return s;
}
uint64_t mtime() {
#ifdef ESP32
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  uint64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
  return time_us;
#endif
#ifdef ESP8266
  uint64_t time_us = micros64();
  return time_us;
#endif
}
void display_result(String $coords) {
#ifdef ESP32
  oled.clear();
  oled.home();                     // курсор в 0,0
  oled.setScale(1);                // масштаб шрифта (1-4)
  oled.print("Робот обнаружен:");  // печатай что угодно
  oled.setCursor(0, 2);
  oled.setScale(4);  // масштаб шрифта (1-4)
  oled.print($coords);
  oled.update();
#endif
}

void setup() {
  Serial.begin(115200);
  Serial.println("setup");
#ifdef ESP8266
  ESP_ID = ESP.getChipId();
#endif
#ifdef ESP32
  uint64_t macAddress = ESP.getEfuseMac();
  uint64_t macAddressTrunc = macAddress << 40;
  ESP_ID = macAddressTrunc >> 40;
  is_display_avaliable = true;
  // активируем экран
  oled.init(PIN_SDA, PIN_SCK);
  oled.clear();
  oled.home();
#endif
  // запускаем монитор порта
  Serial.print("ESP_ID=");
  Serial.println(ESP_ID);

  // устанавливаем пины на получение данных от датчиков
  pinMode(PIN_SENSOR, INPUT);
  pinMode(SOUND_PIN, OUTPUT);

// начинаем калибрацию микрофона
//mic_calibration();
#ifdef ESP8266
  connectBeaconToServer();
#endif
  wifi_init();
  attachInterrupt(BTN_PIN, button_click, RISING);
  analyzer.setVolK(20);
  analyzer.setTrsh(10);
  analyzer.setVolMin(10);
  analyzer.setVolMax(100);
  analyzer.setDt(10);
  analyzer.setWindow(10);
}


void loop() {
  // код отрабатывает раз в секунду
  ms = millis();
  if ((ms - ms1) > 1000 || ms < ms1) {
    ms1 = ms;

#ifdef ESP32
    //wifi_conn_cnt();
#endif

#ifdef ESP8266
    // проверяем статус соединения с сервером
    if (wifi_connect_started && WiFi.status() == WL_CONNECTED) {
      wifi_connected = true;
      wifi_connect_started = false;
      udp.begin(UDP_PORT);
    }
    // Прием данных по UDP
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int n = udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      packetBuffer[n] = 0;
      Serial.println("Данные:");
      Serial.println(packetBuffer[0]);
      Serial.println(packetBuffer);
      do_command(packetBuffer);
    }
#endif
  }
  if (button_pressed) {
    button_pressed = false;
    btn_click();
  }
  if (sound_waiting && analyzer.tick() && analyzer.getRaw() > 3) {
    lastSoundTime = micros();
#ifdef ESP8266
    //отправляем временную метку на сервер
    t = lastSoundTime + DELTA;
    //Передаем на сервер время срабатывания
    udp.beginPacket(udpServerIP, UDP_PORT);
    udp.printf("%d,%d", SOUND_TIME_RECEIVED, t);
    udp.endPacket();
#endif

#ifdef ESP32
    //ожидаем ответа от всех маяков и начинаем расчет координат
    calculating_robot_coords = true;
    if (beacon_answered_count == beacon_count && calculating_robot_coords) {
      //все маяки ответили, делаем расчет
      calculateRobotCoords();
    }
#endif
  }
  /*if (is_mic_calibration) {
    int mic = analogRead(PIN_MIC);
    if (max_mic < mic) {
      max_mic = mic;
    }
  }*/
}
//обработка нажатия кнопки
void btn_click() {
  tone(SOUND_PIN, 1000, 100);
  Serial.println("Click");
#ifdef ESP32
  String s;
  s += DO_UPDATE_DELAY;
  s += ",";
  s += String(mtime());
  Serial.println("button send command:");
  Serial.println(s);
  udp.broadcast(s.c_str());
#endif
#ifdef ESP8266
  udp.beginPacket(udpServerIP, UDP_PORT);
  Serial.println("button send command:");
  Serial.println("5,test");
  udp.print("5,test");
  udp.endPacket();
#endif
}

// запускаем калибрацию микрофона
void mic_calibration() {
  max_mic = 0;
  is_mic_calibration = true;
#ifdef ESP32
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &endOfMicCalibration);
  timerAlarm(timer, wdtTimeout * 1000, false, 0);
#endif
#ifdef ESP8266
  // Initialize Ticker every 0.5s
  blinker.attach_ms(wdtTimeout, endOfMicCalibration);  // Use attach_ms if you need time in ms
#endif
}

void calculateRobotCoords() {
}
