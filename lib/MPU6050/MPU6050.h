#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 传感器数据结构
typedef struct {
    float gyro_scale;
    float accel_scale;
    float dt;
    float gyro_z_offset;
    unsigned long last_time;
    float angle_z;
} MPU6050_HandleTypeDef;

// 初始化函数
void MPU6050_Init(MPU6050_HandleTypeDef* hmpu, int sda_pin, int scl_pin);
void MPU6050_Calibrate(MPU6050_HandleTypeDef* hmpu, uint16_t samples);

// 核心功能
void MPU6050_Update(MPU6050_HandleTypeDef* hmpu);
float MPU6050_GetAngleZ(MPU6050_HandleTypeDef* hmpu);

#ifdef __cplusplus
}
float GetYaw();
float GetStableYaw();
#endif

#endif /* __MPU6050_H */