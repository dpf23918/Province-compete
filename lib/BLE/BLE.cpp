#include "BLE.h"
#include <Arduino.h>
#define SERVICE_UUID             "DFCD0001-36E1-4688-B7F5-EA07361B26A8"
#define CHARACTERISTIC_UUID     "DFCD000A-36E1-4688-B7F5-EA07361B26A8"
// 设备发现回调实现
class BLEClientManager::MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    BLEClientManager& manager;
public:
    MyAdvertisedDeviceCallbacks(BLEClientManager& m) : manager(m) {}
    
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.getName() == "ESP32_BT_BLE") {
            advertisedDevice.getScan()->stop();
            manager.pServerAddress = new BLEAddress(advertisedDevice.getAddress());
            manager.doConnect = true;
            Serial.println("Found target device! Connecting...");
        }
    }
};

// 连接回调实现
class BLEClientManager::MyClientCallback : public BLEClientCallbacks {
    BLEClientManager& manager;
public:
    MyClientCallback(BLEClientManager& m) : manager(m) {}
    
    void onConnect(BLEClient* pclient) {
        manager.deviceConnected = true;
        Serial.println("Connected to server");
    }

    void onDisconnect(BLEClient* pclient) {
        manager.deviceConnected = false;
        Serial.println("Disconnected from server");
        BLEDevice::getScan()->start(0);
    }
};

// 类方法实现
BLEClientManager::BLEClientManager() 
    : pServerAddress(nullptr), 
      pClient(nullptr),
      pRemoteCharacteristic(nullptr),
      deviceConnected(false),
      doConnect(false) {}

BLEClientManager::~BLEClientManager() {
    if(pClient) {
        pClient->disconnect();
        delete pClient;
    }
    delete pServerAddress;
}

void BLEClientManager::setupBLE() {
    Serial.begin(115200); // Initialize Serial communication
    BLEDevice::init("ESP32_BLE_Client");
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(*this));
    pScan->setActiveScan(true);
    pScan->start(0);
    Serial.println("BLE initialized");
}

bool BLEClientManager::connectToServer() {
    const int MAX_RETRY = 3;
    const uint32_t TIMEOUT_PER_ATTEMPT = 5000; // 单次连接尝试超时5秒
    int retryCount = 0;

    while (retryCount < MAX_RETRY) {
        Serial.printf("Connection attempt %d/%d to %s\n", 
                    retryCount+1, MAX_RETRY, 
                    pServerAddress->toString().c_str());

        // 清理旧连接
        if(pClient) {
            pClient->disconnect();
            delete pClient;
            pClient = nullptr;
        }

        // 创建新客户端
        pClient = BLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallback(*this));

        // 带超时的连接尝试
        uint32_t startTime = millis();
        bool connected = false;
        while (millis() - startTime < TIMEOUT_PER_ATTEMPT) {
            if (pClient->connect(*pServerAddress)) {
                connected = true;
                break;
            }
            delay(100); // 防止忙等待
        }

        if (!connected) {
            Serial.println("Connect attempt failed");
            retryCount++;
            continue;
        }

        // 服务发现
        BLERemoteService* pRemoteService = nullptr;
        for (int i = 0; i < 3; i++) {
            pRemoteService = pClient->getService(SERVICE_UUID);
            if (pRemoteService) break;
            delay(100);
            Serial.println("Retrying service discovery...");
        }

        if (!pRemoteService) {
            Serial.println("Service discovery failed");
            retryCount++;
            continue;
        }

        // 特征值发现
        for (int i = 0; i < 3; i++) {
            pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
            if (pRemoteCharacteristic) break;
            delay(100);
            Serial.println("Retrying characteristic discovery...");
        }

        if (!pRemoteCharacteristic) {
            Serial.println("Characteristic discovery failed");
            retryCount++;
            continue;
        }

        Serial.println("Connection fully established!");
        return true;
    }

    Serial.println("Max retries reached");
    return false;
}


void BLEClientManager::update() {
    if (doConnect) {
        if (connectToServer()) {
            Serial.println("Connected to server!");
        } else {
            Serial.println("Failed to connect");
        }
        doConnect = false;
    }

    if (!deviceConnected && pClient) {
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
    }
}

void BLEClientManager::sendData(const std::string &data) {
    if (deviceConnected && !data.empty()) {
        pRemoteCharacteristic->writeValue(data);
        Serial.printf("Sent: %s\n", data.c_str());
    }
}