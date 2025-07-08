/* imu901.cpp */
#include "imu901.h"
#include <HardwareSerial.h>
#include <Arduino.h>
IMU901::IMU901(int rxPin, int txPin, uint32_t baud) 
: yawOffset(0.0f), isCalibrated(false), previousYaw(0.0f), totalYaw(0.0f)
{
    rxBuffer.buffer = bufferStorage;
    rxBuffer.size = sizeof(bufferStorage);
    rxBuffer.in = 0;
    rxBuffer.out = 0;
    
    IMU901_UART.begin(baud, SERIAL_8N1, rxPin, txPin);
}

void IMU901::begin() {
    isCalibrated = false; // 每次调用begin时重置校准
    yawOffset = 0.0f;
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX, TX
}

void IMU901::update() {
    processSerial();
}

void IMU901::processSerial() {
    // 读取所有可用数据到缓冲区
    while (IMU901_UART.available()) {
        uint8_t c = IMU901_UART.read();
        
        // 写入环形缓冲区
        uint16_t next = (rxBuffer.in + 1) % rxBuffer.size;
        if (next != rxBuffer.out) {
            rxBuffer.buffer[rxBuffer.in] = c;
            rxBuffer.in = next;
        }
    }

    // 处理缓冲区数据
    while (rxBuffer.out != rxBuffer.in) {
        uint8_t c = rxBuffer.buffer[rxBuffer.out];
        rxBuffer.out = (rxBuffer.out + 1) % rxBuffer.size;
        
        if (unpack(c)) {
            parsePacket(&currentPacket);
        }
    }
}

bool IMU901::unpack(uint8_t ch) {
    switch (rxState) {
        case waitForStartByte1:
            if (ch == 0x55) {
                currentPacket.startByte1 = ch;
                cksum = ch;
                rxState = waitForStartByte2;
            }
            break;

        case waitForStartByte2:
            if (ch == 0x55 || ch == 0xAF) {
                currentPacket.startByte2 = ch;
                cksum += ch;
                rxState = waitForMsgID;
            } else {
                rxState = waitForStartByte1;
            }
            break;

        case waitForMsgID:
            currentPacket.msgID = ch;
            cksum += ch;
            rxState = waitForDataLength;
            break;

        case waitForDataLength:
            if (ch <= 28) {
                currentPacket.dataLen = ch;
                cksum += ch;
                dataIndex = 0;
                rxState = (ch > 0) ? waitForData : waitForChksum1;
            } else {
                rxState = waitForStartByte1;
            }
            break;

        case waitForData:
            currentPacket.data[dataIndex++] = ch;
            cksum += ch;
            if (dataIndex == currentPacket.dataLen) {
                rxState = waitForChksum1;
            }
            break;

        case waitForChksum1:
            rxState = waitForStartByte1;
            if (cksum == ch) {
                currentPacket.checkSum = ch;
                return true;
            }
            break;
    }
    return false;
}

void IMU901::parsePacket(const atkp_t* packet) {
    switch (packet->msgID) {
        case 0x01:  { // 姿态角
            // 解析原始姿态数据（修复之前的错误）
            float roll = ((int16_t)(packet->data[1]<<8 | packet->data[0])) / 32768.0f * 180.0f;
            float pitch = ((int16_t)(packet->data[3]<<8 | packet->data[2])) / 32768.0f * 180.0f;
            float currentYaw = ((int16_t)(packet->data[5]<<8 | packet->data[4])) / 32768.0f * 180.0f;

            // 首次数据校准
            if (!isCalibrated) {
                yawOffset = currentYaw;
                previousYaw = currentYaw;  // 新增变量，用于跟踪上一次的YAW
                totalYaw = 0.0f;           // 新增变量，累积总YAW角度
                isCalibrated = true;
            }

            // 计算角度变化并处理环绕
            float delta = currentYaw - previousYaw;
            if (delta > 180.0f) delta -= 360.0f;
            else if (delta < -180.0f) delta += 360.0f;
            
            totalYaw += delta;            // 累积总角度变化
            previousYaw = currentYaw;     // 更新上一次的角度

            // 更新姿态数据（修复之前的错误赋值）
            attitude.roll = roll;
            attitude.pitch = pitch;
            attitude.yaw = totalYaw;     // 使用累积的总YAW角度
            break;
        }
        case 0x03:  // 陀螺仪和加速度计
            for (int i=0; i<3; i++) {
                gyroAccData.acc[i] = (int16_t)(packet->data[i*2+1]<<8 | packet->data[i*2]);
                gyroAccData.gyro[i] = (int16_t)(packet->data[i*2+7]<<8 | packet->data[i*2+6]);
            }
            // 量程转换需要根据实际配置调整
            break;

        // 其他数据包类型解析...
    }
}