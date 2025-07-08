#ifndef __Encoder__H
#define __Encoder__H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>


extern ESP32Encoder left_encoder;
extern ESP32Encoder right_encoder;
extern volatile int64_t last_left_count;
extern volatile int64_t last_right_count;

void Encoder_Init();

int16_t Speed_Left_Encoder();
int16_t Speed_Right_Encoder() ;

int16_t Distance_Left_Encoder();
int16_t Distance_Right_Encoder();
void Encoder_Reset() ;
#endif