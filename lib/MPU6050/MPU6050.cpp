#include "mpu6050.h"
#include <Wire.h>

#define MPU6050_ADDR 0x68
#define RAD_TO_DEG 57.2957795130823
extern MPU6050_HandleTypeDef mpu;

// 私有函数原型
static void write_reg(uint8_t reg, uint8_t data);
static uint8_t read_reg(uint8_t reg);
static void read_raw_data(int16_t* accel, int16_t* gyro);

void MPU6050_Init(MPU6050_HandleTypeDef* hmpu, int sda_pin, int scl_pin) {
    // 初始化I2C总线
    Wire.begin(sda_pin, scl_pin, 400000);
    
    // 传感器初始化
    write_reg(0x6B, 0x00); // 唤醒设备
    delay(100);
    write_reg(0x6B, 0x01); // 选择时钟源
    write_reg(0x1B, 0x18); // ±2000dps
    write_reg(0x1C, 0x18); // ±16g
    
    // 初始化结构体
    hmpu->gyro_scale = 2000.0f / 32768.0f;
    hmpu->accel_scale = 16.0f / 32768.0f;
    hmpu->dt = 0.01f; // 默认100Hz采样率
    hmpu->gyro_z_offset = 0;
    hmpu->angle_z = 0;
}

void MPU6050_Calibrate(MPU6050_HandleTypeDef* hmpu, uint16_t samples) {
    int32_t sum_z = 0;
    for(uint16_t i=0; i<samples; i++){
        int16_t gyro[3];
        read_raw_data(NULL, gyro);
        sum_z += gyro[2];
        delay(2);
    }
    hmpu->gyro_z_offset = sum_z / (float)samples;
}

void MPU6050_Update(MPU6050_HandleTypeDef* hmpu) {
    int16_t gyro[3];
    read_raw_data(NULL, gyro);

    // 计算时间间隔
    unsigned long current_time = micros();
    if(hmpu->last_time != 0) {
        hmpu->dt = (current_time - hmpu->last_time) * 1e-6f;
    }
    hmpu->last_time = current_time;

    // 角度积分
    float gyro_z = (gyro[2] - hmpu->gyro_z_offset) * hmpu->gyro_scale;
    hmpu->angle_z += gyro_z * hmpu->dt;
}

float MPU6050_GetAngleZ(MPU6050_HandleTypeDef* hmpu) {
    return hmpu->angle_z ;
}

// 私有函数实现
static void write_reg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

static uint8_t read_reg(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 1);
    return Wire.read();
}

static void read_raw_data(int16_t* accel, int16_t* gyro) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);

    if(accel) {
        accel[0] = Wire.read()<<8 | Wire.read();
        accel[1] = Wire.read()<<8 | Wire.read();
        accel[2] = Wire.read()<<8 | Wire.read();
        Wire.read()<<8 | Wire.read(); // 温度
    } else {
        Wire.read(); Wire.read();
        Wire.read(); Wire.read();
        Wire.read(); Wire.read();
        Wire.read(); Wire.read();
    }

    if(gyro) {
        gyro[0] = Wire.read()<<8 | Wire.read();
        gyro[1] = Wire.read()<<8 | Wire.read();
        gyro[2] = Wire.read()<<8 | Wire.read();
    }
}
float GetYaw() {
    MPU6050_Update(&mpu);
    static float buffer[5] = {0};
    static int idx = 0;
    
    // 原始数据采集
    float raw_yaw = MPU6050_GetAngleZ(&mpu); // 转换为度
    
    // 滑动平均滤波
    buffer[idx] = raw_yaw;
    idx = (idx+1) % 5;
    float filtered_yaw = (buffer[0]+buffer[1]+buffer[2]+buffer[3]+buffer[4])/5.0;
    
    // 角度连续化处理（原有代码保留）
    static float prev_yaw = 0;
    float delta = filtered_yaw - prev_yaw;
    if(delta > 180) delta -= 360;
    else if(delta < -180) delta += 360;
    prev_yaw += delta;
    
    return prev_yaw;
  }
  float GetStableYaw() {
    const int SAMPLE_COUNT = 5;
    float sum = 0;
    for(int i=0; i<SAMPLE_COUNT; i++){
        sum += MPU6050_GetAngleZ(&mpu);
        delay(1);
    }
    return sum / SAMPLE_COUNT;
}