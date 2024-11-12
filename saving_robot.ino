#define PIN_SENSOR 0 // Пин, к которому присоединен датчик вибрации
#define PIN_MIC 1 //Пин, к которому присоединен микрофон
#define PIN_SDA 27 //Дисплей SDA
#define PIN_SCK 14 //Дисплей SCK
#define SERVER_UUID "805a2ab8-a577-408b-9ae7-edc842cd65a7";
#define SENSOR1_UUID "45fbc548-47dc-46ab-bff0-a9f9921dd273";
#define SENSOR2_UUID "5ff97661-4f90-4b46-be4f-026dece9c796";
#define SENSOR3_UUID "4dbe2423-51a8-4637-96fa-9a179c6de47a";
#include <GyverOLED.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;

int max_mic;
int is_mic_calibration;
const int wdtTimeout = 5000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;
String ESP_ID;
//BLE server name
#define bleServerName "BLE_ESP32"
// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

bool deviceConnected = false;

BLECharacteristic soundTimeCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor soundTimeDescriptor(BLEUUID((uint16_t)0x2903));

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void ARDUINO_ISR_ATTR after5seconds() {
  Serial.println("MAX mic level:");
  Serial.println(max_mic);
  is_mic_calibration=0;
}

void setup() {
  ESP_ID = String(ESP.getChipId()).c_str();
  Serial.begin(115200);
  Serial.println(ESP_ID);
  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_MIC, INPUT);
  max_mic=0;
  is_mic_calibration=1;
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &after5seconds);
  timerAlarm(timer, wdtTimeout * 1000, false, 0);

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(UUID1);

  // Humidity
  bmeService->addCharacteristic(&soundTimeCharacteristics);
  soundTimeDescriptor.setValue("Sound Time");
  soundTimeCharacteristics.addDescriptor(new BLE2902());
  
  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(UUID1);
  pServer->getAdvertising()->start();

  /*oled.init(27, 14); 
  oled.clear(); 
  oled.home();            // курсор в 0,0
  oled.setScale(1); // масштаб шрифта (1-4)
  oled.print("Робот обнаружен:");   // печатай что угодно: числа, строки, float, как Serial!
  oled.setCursor(0, 2); 
  oled.setScale(4); // масштаб шрифта (1-4)
  oled.print("A 7");   // печатай что угодно: числа, строки, float, как Serial!
  oled.update();*/
}

void loop() {
  int val = analogRead(PIN_SENSOR); // Считваем значение с датчика
  int mic = analogRead(PIN_MIC); // Считваем значение с датчика
  // if(val!=4095){ 
  //   Serial.println("MOVED!!!");
  // }
  if(is_mic_calibration && max_mic < mic){ 
    max_mic=mic;
  } elseif(!is_mic_calibration and mic>max_mic){ 
    //Serial.println(mic * (5.0 / 1023.0));
  }

  if (deviceConnected) {
    if ((millis() - lastTime) > timerDelay) {
      //Set humidity Characteristic value and notify connected client
      soundTimeCharacteristics.setValue(mic);
      soundTimeCharacteristics.notify();   
      //Serial.print(" - Humidity: ");
      //Serial.print(mic);
      //Serial.println(" %");
      
      lastTime = millis();
    }
  }
}
