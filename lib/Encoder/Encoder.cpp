#include <Arduino.h>
#include "Encoder.h"

volatile int32_t left_total = 0, right_total = 0;
const int LEFT_ENCODER_A =14;
const int LEFT_ENCODER_B =15;
const int RIGHT_ENCODER_A =12;
const int RIGHT_ENCODER_B =13;

ESP32Encoder left_encoder;
ESP32Encoder right_encoder;
const float PPR = 13.0f;          // 编码器每转脉冲数（根据电机型号）
const float WHEEL_DIAMETER = 0.48f; // 轮子直径（米）
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER; // 轮子周长≈0.314m
const float DT = 0.01f; 
volatile int64_t last_left_count = 0;
volatile int64_t last_right_count = 0;

// 初始化编码器
void Encoder_Init() {
  // 初始化编码器1
  left_encoder.attachFullQuad(LEFT_ENCODER_A, LEFT_ENCODER_B);
  left_encoder.setCount(0);  // 初始计数值清零

  // 初始化编码器2
  right_encoder.attachFullQuad(RIGHT_ENCODER_A , RIGHT_ENCODER_B);
  right_encoder.setCount(0);  // 初始计数值清零

  // 可选：启用硬件滤波器（抑制噪声）
  left_encoder.setFilter(100);  // 滤波强度（0~1023）
  right_encoder.setFilter(100);
}

int16_t Speed_Left_Encoder() {
  noInterrupts(); 
  int16_t delta = left_encoder.getCount();
  left_encoder.clearCount();
  left_total += delta;
  interrupts();  // 可选：记录总位移
  return delta;         // 返回增量而非累计值
}

int16_t Speed_Right_Encoder() {
  noInterrupts(); 
  int16_t delta = right_encoder.getCount();
  right_encoder.clearCount();
  right_total += delta;
  interrupts(); 
  return delta;
}
int16_t Distance_Left_Encoder() {
  int16_t delta = left_encoder.getCount();
  left_encoder.clearCount();
  left_total += delta;  // 可选：记录总位移
  return left_total;         // 返回增量而非累计值
}
int16_t Distance_Right_Encoder() {
  int16_t delta = right_encoder.getCount();
  right_encoder.clearCount();
  right_total += delta;
  return right_total;
}
void Encoder_Reset() {
  // 清除编码器实例的当前计数值
  left_encoder.clearCount();
  right_encoder.clearCount();
  
  // 重置历史计数变量
  last_left_count = 0;
  last_right_count = 0;
  
  // 可选：根据需求重置总位移累计值
  left_total = 0;
  right_total = 0;
}