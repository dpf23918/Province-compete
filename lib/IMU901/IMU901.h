/* imu901.h */
#ifndef _IMU901_H_
#define _IMU901_H_

#include <Arduino.h>

#define IMU901_UART Serial2  // 使用Serial2与IMU模块通信

/* 数据包类型定义 */
#pragma pack(push, 1)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct {
    float q0, q1, q2, q3;
} quaternion_t;

typedef struct {
    int16_t gyro[3];
    int16_t acc[3];
    float fgyroD[3];
    float faccG[3];
} gyroAcc_t;

typedef struct {
    int16_t mag[3];
    float temp;
} mag_t;

typedef struct {
    int32_t pressure;
    int32_t altitude;
    float temp;
} baro_t;

typedef struct {
    uint16_t d03data[4];
} ioStatus_t;

typedef struct {
    uint8_t startByte1;
    uint8_t startByte2;
    uint8_t msgID;
    uint8_t dataLen;
    uint8_t data[28];
    uint8_t checkSum;
} atkp_t;
#pragma pack(pop)

class IMU901 {
public:
    IMU901(int rxPin, int txPin, uint32_t baud = 115200);
    void begin();
    void update();
    
    // 数据访问接口
    attitude_t getAttitude() const { return attitude; }
    quaternion_t getQuaternion() const { return quaternion; }
    gyroAcc_t getGyroAcc() const { return gyroAccData; }
    mag_t getMag() const { return magData; }
    baro_t getBaro() const { return baroData; }

private:
    float yawOffset;
    bool isCalibrated;
    float previousYaw;  // 跟踪上一次的YAW值
    float totalYaw;     // 累积的总YAW角度
    void processSerial();
    bool unpack(uint8_t ch);
    void parsePacket(const atkp_t* packet);
    // 环形缓冲区相关
    struct RingBuffer {
        uint8_t* buffer;
        uint16_t size;
        volatile uint16_t in;
        volatile uint16_t out;
    };
    
    RingBuffer rxBuffer;
    uint8_t bufferStorage[512];  // 根据实际需求调整大小
    
    // 数据存储
    attitude_t attitude;
    quaternion_t quaternion;
    gyroAcc_t gyroAccData;
    mag_t magData;
    baro_t baroData;
    
    // 解析状态机
    enum RxState {
        waitForStartByte1,
        waitForStartByte2,
        waitForMsgID,
        waitForDataLength,
        waitForData,
        waitForChksum1,
    };
    
    RxState rxState = waitForStartByte1;
    atkp_t currentPacket;
    uint8_t cksum = 0;
    uint8_t dataIndex = 0;
};

#endif