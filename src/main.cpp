#include <Wire.h>
#include <Arduino.h>
#include <PID.h>
#include <Motor.h>
#include <Encoder.h>
#include <grey.h>
// PID变量
extern PID_t Left_Inner;
extern PID_t Right_Inner;
// 按钮
const uint8_t MODE_PIN = 23;
extern int err;
extern int basespeed;
unsigned long last_press = 0;
// 定义轮子周长和编码器每转脉冲数
double wheel_circumference_ = 4.5; // 轮子直径
int encoder_ppr_ = 707; // 编码器每转脉冲数
bool PID_Flag = false;

void IRAM_ATTR handle_mode_button() {

if ((millis() - last_press) > 200)
  {
    PID_Flag = true;
    last_press = millis();
  }

}
void setup()
{

  Speed_PID_Init(&Left_Inner);
  Speed_PID_Init(&Right_Inner);
  Encoder_Init();
  pinMode(MODE_PIN, INPUT_PULLUP);
  attachInterrupt(MODE_PIN, handle_mode_button, FALLING);
  Motor_Init();
  Wire.begin();         // join i2c bus (address optional for master)
  Serial.begin(115200); // start serial for output

  // 进入校准模式
  // Wire.beginTransmission(0x12);
  // Wire.write(1);
  // Wire.write(1);
  // Wire.endTransmission();    // stop transmitting
  // delay(500);
}

void loop()
{

  /*I2Cdata();
  if(PID_Flag){
  Record_grey_PID();
  Motor_Control(Left_Inner.Out, Right_Inner.Out);
  }
  if(err==5)
  {
    PID_Flag = false;
  }*/
 Serial.printf("Right=%d,Left=%d\r\n",Distance_Left_Encoder(),Distance_Right_Encoder());
  
}
int cm_to_pulses(float cm) 
{
    return static_cast<int>((cm / wheel_circumference_) * encoder_ppr_);
}