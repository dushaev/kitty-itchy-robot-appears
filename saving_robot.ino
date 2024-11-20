#define PIN_SENSOR 0  // Пин, к которому присоединен датчик вибрации
#define PIN_MIC 1     // Пин, к которому присоединен микрофон
#define PIN_SDA 27    // Дисплей SDA
#define PIN_SCK 14    // Дисплей SCK
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
// Сервер
static BLEUUID serverUUID("805a2ab8-a577-408b-9ae7-edc842cd65a7");
// Маяки, к которым мы хотим подключиться
static BLEUUID service1_UUID("45fbc548-47dc-46ab-bff0-a9f9921dd273");
static BLEUUID service2_UUID("5ff97661-4f90-4b46-be4f-026dece9c796");
static BLEUUID service3_UUID("4dbe2423-51a8-4637-96fa-9a179c6de47a");
// Параметр необходимости запустить калибровку маяков
static BLEUUID recalibrate1_UUID("7822534c-11d2-4736-8eb4-61e5aacc888f");
static BLEUUID recalibrate2_UUID("303b9466-1c0f-453f-b92b-ac809cdc178c");
static BLEUUID recalibrate3_UUID("12828c50-58e0-4665-8159-4496cf0b706b");
// Параметр получения сигнала от маяка
static BLEUUID sound1_UUID("d6d0dab4-ff88-4e89-ad71-a6a134ea0cc4");
static BLEUUID sound2_UUID("755cc565-81d4-4953-b127-622fda3e667c");
static BLEUUID sound3_UUID("a486a71d-2303-4cb4-9be9-8096d2419c3a");
// Параметр режима калибровки
static BLEUUID calibration_state_UUID("685643a1-f055-4ee0-a1c7-b5cc25591b4c");
// Параметр режима определения местоположения робота
static BLEUUID location_state_UUID("335a2af4-c28a-46bd-84e8-3c3ff8093d3d");
// Параметр времени сервера в микросекундах
static BLEUUID server_microtime_UUID("6cbe0201-7c42-4ad4-8821-35026f9f0cdc");

BLECharacteristic server_microtime_Characteristics("6cbe0201-7c42-4ad4-8821-35026f9f0cdc", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor server_microtime_Descriptor(BLEUUID((uint16_t)0x0552));

#ifdef ESP32
#include <GyverOLED.h>
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
#endif

int is_display_avaliable;
int max_mic;
bool is_mic_calibration;
const int wdtTimeout = 5000;  // время в мсек для калибровки уровня шума
hw_timer_t *timer = NULL;
uint32_t ESP_ID;
String ESP_UID;
// Переменные для таймера
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;
// дельта времени с сервером
uint64_t DELTA;
// Различные переменные для работы BLE
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *serverBLE;
static boolean deviceConnected = false;


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

// Функция для получения значений от датчиков
static void notifyCallback(
  BLERemoteCharacteristic *pBLERemoteCharacteristic,
  unsigned char*pData,
  size_t length,
  bool isNotify) {
  Serial.print("Получение значения датчиков ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" , размер данных: ");
  Serial.println(length);
  unsigned char currentState;
  currentState = *pData;
  Serial.print("данные: ");
  Serial.println(currentState);
  //print_uint64_t(currentState);
  DELTA = mtime() - currentState;
  Serial.print("DELTA: ");
  Serial.println(DELTA);
}
// Функция для обработки событий BLE
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
  }
  void onDisconnect(BLEClient *pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

// Функция для соединения по BLE
bool connectToServer() {
  Serial.print("Начинаем соединение с ");
  Serial.println(ESP_UID);

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Создан клиент");

  pClient->setClientCallbacks(new MyClientCallback());

  // Подключение к серверу
  pClient->connect(serverBLE);
  Serial.println(" - Подключен к серверу");
  pClient->setMTU(517);  // установка максимального MTU для устройства (по умолчанию 23)

  // Подключение к устройству
  BLERemoteService *pRemoteService = pClient->getService(serverUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Не найдено устройство UUID: ");
    Serial.println(serverUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Найден сервер");

  // Получим от сервера значение времени и установим его
  pRemoteCharacteristic = pRemoteService->getCharacteristic(server_microtime_UUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Не удалось подключиться к датчику времени UUID: ");
    Serial.println(server_microtime_UUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Найден датчик");

  // Считываем показание датчика.
  if (pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    // if (value.length() == sizeof(uint64_t))
    //{

    DELTA = mtime() - value[0];

    // Отображаем данные
    Serial.print("Показания датчика: ");
    Serial.println(value[0]);
    Serial.print("Локальные микросекунды: ");
    Serial.println(mtime());
    Serial.print("Дельта: ");
    Serial.println(DELTA);
    //}

    if (pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
  }
}

/**
 * Ищем BLE устройства и находим которые нам нужны
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE устройство найдено: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Мы нашли устройство, посмотрим, может это сервер?
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serverUUID)) {
      BLEDevice::getScan()->stop();
      serverBLE = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }  // Нашли устройство
  }    // onResult
};     // MyAdvertisedDeviceCallbacks

void ARDUINO_ISR_ATTR endOfMicCalibration() {
  // калибровка уровня шума завершена
  Serial.println("Уровень шума:");
  Serial.println(max_mic);
  is_mic_calibration = false;
  connectBeaconToServer();
}

void UID_for_board(uint8_t chipid) {
  if (chipid == 7052138) {
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
}

void mic_calibration() {
  max_mic = 0;
  is_mic_calibration = true;
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &endOfMicCalibration);
  timerAlarm(timer, wdtTimeout * 1000, false, 0);
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

void ble_init() {
  BLEDevice::init("");
  // Запускаем поиск устройств BLE и устанавливаем функцию, которая будет вызвана
  // когда устроство будет найдено. Устанавливаем активное сканирование и время 5 секунд
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void start_ble_server() {
  BLEDevice::init("");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // создаем BLE-сервис:
  BLEService *serverService = pServer->createService(serverUUID);

  serverService->addCharacteristic(&server_microtime_Characteristics);
  server_microtime_Descriptor.setValue("Server microsec clock");
  server_microtime_Characteristics.addDescriptor(new BLE2902());
  // запускаем сервис:
  serverService->start();

  // запускаем рассылку оповещений:
  pServer->getAdvertising()->start();
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
  UID_for_board(ESP_ID);
  // запускаем монитор порта
  Serial.begin(115200);
  Serial.print('ESP_ID=');
  Serial.println(ESP_ID);

  // устанавливаем пины на получение данных от датчиков
  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_MIC, INPUT);

  // начинаем калибрацию микрофона
  mic_calibration();

  // старт BLE
  ble_init();

// активируем экран (только на esp32, который у нас будет сервером)
#ifdef ESP32
  oled.init(PIN_SDA, PIN_SCK);
  start_ble_server();
#endif

  // запускаем первоначальный расчет положения маяков
  startBeaconCalibration();
}

void startBeaconCalibration() {
}

void findRobotCoords() {
}

void connectBeaconToServer() {
  // если doConnect=true, значит мы поиск устройств завершен
  // После этого мы к нему подключаемся
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Мы подключены к серверу");
    } else {
      Serial.println("Не удалось подключится к серверу");
    }
    doConnect = false;
  }

  if (doScan) {
    BLEDevice::getScan()->start(0);
  }
}

void needBeaconCalibrationToServer() {
  if (connected) {
    String newValue = "Time since boot: " + String(millis() / 1000);
    Serial.println("Setting new characteristic value to \"" + newValue + "\"");

    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }
}

void loop() {
#ifdef ESP32
  server_microtime_Characteristics.setValue(string_uint64_t(mtime()));
  server_microtime_Characteristics.notify();
#endif
  int val = analogRead(PIN_SENSOR);  // Считываем значение с датчика вибрации
  int mic = analogRead(PIN_MIC);     // Считываем значение с датчика микрофона

  if (val != 4095) {
    Serial.println("Сработал датчик вибрации, запускаем калибровку положения");
    startBeaconCalibration();
  }

  if (is_mic_calibration && max_mic < mic) {
    max_mic = mic;
  } else if(!is_mic_calibration and mic > max_mic) {
    Serial.println("Получен звуковой сигнал от робота. Начинаем расчет координат");
    findRobotCoords();
  }
}
