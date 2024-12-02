#ifdef ESP32
#define PIN_SENSOR 4  // Пин, к которому присоединен датчик вибрации
#define PIN_MIC 32    // Пин, к которому присоединен микрофон
#endif
#ifdef ESP8266
#define PIN_SENSOR 1  // Пин, к которому присоединен датчик вибрации
#define PIN_MIC 2     // Пин, к которому присоединен микрофон
#endif
#define PIN_SDA 27  // Дисплей SDA
#define PIN_SCK 14  // Дисплей SCK

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#endif

#ifdef ESP32
#include <WiFi.h>
#include <DNSServer.h>
WiFiServer server(80);
String responseHTML = "<!DOCTYPE html><html>"
                      "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                      "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}"
                      "</style></head>"
                      "<body><h1>ESP32 Web Server</h1>"
                      "<p>Hello World</p>"
                      "</body></html>";
int connections = 0;
#endif
IPAddress apIP(10, 10, 20, 1);
IPAddress netMsk(255, 255, 255, 0);

const char *softAP_ssid = "K.I.R.A.";
const char *password = "akira";

#ifdef ESP32
#include <GyverOLED.h>
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
#endif

int is_display_avaliable;
int max_mic;
bool is_mic_calibration;
const int wdtTimeout = 5000;  // время в мсек для калибровки уровня шума
#ifdef ESP32
hw_timer_t *timer = NULL;
#endif
#ifdef ESP8266
#include <Ticker.h>  //Ticker Library
Ticker blinker;
#endif
uint32_t ESP_ID;
String ESP_UID;
// Переменные для таймера
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;
// дельта времени с сервером
uint64_t DELTA;

void print_uint64_t(uint64_t num) {
  char rev[128];
  char *p = rev + 1;
  while (num > 0) {
    *p++ = '0' + (num % 10);
    num /= 10;
  }
  p--;
  /*Print the number which is now in reverse*/
  while (p > rev) {
    Serial.print(*p--);
  }
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
#ifdef ESP32
void ARDUINO_ISR_ATTR endOfMicCalibration() {
  // калибровка уровня шума завершена
  Serial.println("Уровень шума:");
  Serial.println(max_mic);
  is_mic_calibration = false;
  connectBeaconToServer();
}
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  connections += 1;
  showConnectionsCount();
}

void showConnectionsCount() {
  char data[32];
  sprintf(data, "B: %d", connections);
  draw_message(data);
}
void draw_message(char *msg) {
  oled.setCursor(0, 0);
  oled.setScale(1);  // масштаб шрифта (1-4)
  oled.print(msg);   // печатай что угодно
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
#endif

void connectBeaconToServer() {
}

void UID_for_board(uint8_t chipid) {
  /*if (chipid == 7052138) {
    ESP_UID = service1_UUID.toString().c_str();
  }
  if (chipid == 7566257) {
    ESP_UID = service2_UUID.toString().c_str();
  }
  if (chipid == 7879153) {
    ESP_UID = service3_UUID.toString().c_str();
  }
  // if (chipid == 7566257)
  //{
  ESP_UID = serverUUID.toString().c_str();
  //}
  */
}

void mic_calibration() {
  max_mic = 0;
  is_mic_calibration = true;
#ifdef ESP32
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &endOfMicCalibration);
  timerAlarm(timer, wdtTimeout * 1000, false, 0);
#endif
#ifdef ESP8266
  //Initialize Ticker every 0.5s
  blinker.attach_ms(wdtTimeout, endOfMicCalibration);  //Use attach_ms if you need time in ms
#endif
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

void wifi_init() {
// если esp32 то инициируем сервер
#ifdef ESP32
  WiFi.mode(WIFI_AP);
  WiFi.softAP(softAP_ssid, password);
  WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_AP_STACONNECTED);

  IPAddress ip_address = WiFi.softAPIP();  //IP Address of our accesspoint

  // Start web server
  server.begin();

  Serial.print("AP IP address: ");
  Serial.println(ip_address);
#endif
// если 8266 то подключаемся к серверу
#ifdef ESP8266
  WiFi.mode(WIFI_STA);                  // Выбираем режим клиента для сетевого подключения по Wi-Fi
  WiFi.hostname("K.I.R.A.");            // Указываеем имя хоста, который будет отображаться в Wi-Fi роутере, при подключении устройства
  if (WiFi.status() != WL_CONNECTED) {  // Инициализируем подключение, если устройство ещё не подключено к сети
    WiFi.begin("admin", "akira");
  }
#endif
}

void setup() {
  // получаем ID контроллера
#ifdef ESP8266
  ESP_ID = ESP.getChipId();
#endif
#ifdef ESP32
  uint64_t macAddress = ESP.getEfuseMac();
  uint64_t macAddressTrunc = macAddress << 40;
  ESP_ID = macAddressTrunc >> 40;
  is_display_avaliable = true;
#endif
  //UID_for_board(ESP_ID);
  // запускаем монитор порта
  Serial.begin(115200);
  Serial.print("ESP_ID=");
  Serial.println(ESP_ID);

  // устанавливаем пины на получение данных от датчиков
  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_MIC, INPUT);

  // начинаем калибрацию микрофона
  mic_calibration();

  // старт wifi
  wifi_init();

// активируем экран (только на esp32, который у нас будет сервером)
#ifdef ESP32
  oled.init(PIN_SDA, PIN_SCK);
  oled.clear();
  oled.home();
#endif
  showConnectionsCount();
  // запускаем первоначальный расчет положения маяков
  startBeaconCalibration();
}

void startBeaconCalibration() {
}

void findRobotCoords() {
}

void needBeaconCalibrationToServer() {
}

void wifi_conn_cnt() {
  WiFiClient client = server.available();  // Listen for incoming clients
  if (client) {                            // If a new client connects,
    Serial.println("Client connected");

    String currentLine = "";      // make a String to hold incoming data from the client
    while (client.connected()) {  // loop while the client's connected
      if (client.available()) {   // if there's bytes to read from the client,
        char c = client.read();   // read a byte, then
        Serial.write(c);          // print it out the serial monitor
        if (c == '\n') {          // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // Send header
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Display the HTML web page
            client.println(responseHTML);
            // The HTTP response ends with another blank line
            client.println();
            break;
          } else {  // if we got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if we got anything else but a CR character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Close the connection
    client.stop();
    showConnectionsCount();
  }
}

void loop() {
#ifdef ESP32
  //  server_microtime_Characteristics.setValue(string_uint64_t(mtime()));
  //  server_microtime_Characteristics.notify();
  wifi_conn_cnt();
#endif
  int val = analogRead(PIN_SENSOR);  // Считываем значение с датчика вибрации
  int mic = analogRead(PIN_MIC);     // Считываем значение с датчика микрофона
  //Serial.println(mic);
  if (val != 0) {
    Serial.println("Сработал датчик вибрации, запускаем калибровку положения");
    startBeaconCalibration();
  }

  if (is_mic_calibration && max_mic < mic) {
    max_mic = mic;
  } else if (!is_mic_calibration and mic > max_mic + 400) {
    Serial.println("Получен звуковой сигнал от робота. Начинаем расчет координат");
    //findRobotCoords();
  }
}
