#ifndef _BLE_H
#define _BLE_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

class BLEClientManager {
public:
    BLEClientManager();
    ~BLEClientManager();
    
    void setupBLE();
    void update();
    void sendData(const std::string &data);

    bool isConnected() const { return deviceConnected; }

private:
    BLEAddress *pServerAddress;
    BLEClient* pClient;
    BLERemoteCharacteristic* pRemoteCharacteristic;
    int connectRetries = 0;         // 连接重试计数器
    const int MAX_RETRIES = 3;      // 最大重试次数
    unsigned long lastConnectTime;  // 最后连接时间
    const uint32_t RETRY_INTERVAL = 2000; // 重试间隔2秒
    bool deviceConnected;
    bool doConnect;
    
    class MyAdvertisedDeviceCallbacks;
    class MyClientCallback;
    
    bool connectToServer();
};

#endif