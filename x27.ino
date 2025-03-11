#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SwitecX25.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ========== x27.168 配置 ==========
// 常见 315 步/圈
#define STEPPER_STEPS 315

// XIAO ESP32C3 引脚示例：A0=2, A1=3, D0=4, D1=5
#define STEPPER_PIN1 2
#define STEPPER_PIN2 3
#define STEPPER_PIN3 4
#define STEPPER_PIN4 5

SwitecX25 motor(STEPPER_STEPS, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

// 与 Server 一致的 UUID
static BLEUUID serviceUUID("2f22b6ad-51df-4f4c-9a3c-9f902d12c464");
static BLEUUID charUUID("f7089b06-e82a-44bc-935a-2f31628f11c9");

// BLE 全局变量
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 电机相关
static int targetPos = 0;   
static int currentPos = 0; 
static const int STEP_DELTA = 3; 
static float currentPressure = 0.0f; 

// 将压力映射到 [0, 315]，低于 1030 => 0，超过 1500 => 315，中间线性插值
int mapPressureToSteps(float pressure) {
  float minP = 1030.0f; 
  float maxP = 1500.0f; 
  if (pressure < minP) return 0;
  if (pressure > maxP) return STEPPER_STEPS;

  float ratio = (pressure - minP) / (maxP - minP);
  return (int)(ratio * STEPPER_STEPS);
}

// Notify 回调：解析 "P=xxx.xx mbar, T=xx.xx C"
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify)
{
  String dataStr;
  for(size_t i = 0; i < length; i++){
    dataStr += (char)pData[i];
  }
  Serial.print("Notify callback, data: ");
  Serial.println(dataStr);

  // 显示到 OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("Pressure:");
  display.setTextSize(2);
  display.println(dataStr);
  display.display();

  // 解析 "P=xxx.xx mbar"
  int pIndex = dataStr.indexOf("P=");
  if (pIndex != -1) {
    int mIndex = dataStr.indexOf(" mbar", pIndex);
    if (mIndex != -1) {
      String pressStr = dataStr.substring(pIndex + 2, mIndex);
      float pVal = pressStr.toFloat();
      Serial.print("Parsed Pressure = ");
      Serial.println(pVal);

      currentPressure = pVal;
      targetPos = mapPressureToSteps(currentPressure);
      Serial.print("Target steps = ");
      Serial.println(targetPos);
    }
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.println("Client connected to Server.");
  }
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Client disconnected from Server.");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDevice);
  Serial.println(" - Connected to server");
  pClient->setMTU(517);

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  if(pRemoteCharacteristic->canRead()) {
    String value = String(pRemoteCharacteristic->readValue().c_str());
    Serial.print("Characteristic value: ");
    Serial.println(value);
  }

  if(pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
    Serial.print("Device Name: ");
    Serial.println(advertisedDevice.getName().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  // 初始化 OLED
  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("BLE Client Start");
  display.display();

  // 初始化 x27.168 (若有机械限位可 motor.zero();)
  motor.setPosition(0);

  // 初始化 BLE
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop() {
  // 电机库需要不断调用 update()
  motor.update();

  // 如果扫描到目标则连接
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("Failed to connect to the server.");
    }
    doConnect = false;
  }

  // 已连接则定期写值给 Server (可选)
  static unsigned long lastWrite = 0;
  if (connected) {
    if (millis() - lastWrite > 200) {
      lastWrite = millis();
      String newValue = "Time:" + String(millis()/1000);
      pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
    }
  } else if (doScan) {
    BLEDevice::getScan()->start(0);
  }

  // 让电机逐步逼近 targetPos
  if (currentPos < targetPos) {
    currentPos += STEP_DELTA;
    if (currentPos > targetPos) currentPos = targetPos;
  } else if (currentPos > targetPos) {
    currentPos -= STEP_DELTA;
    if (currentPos < targetPos) currentPos = targetPos;
  }
  motor.setPosition(currentPos);
}
