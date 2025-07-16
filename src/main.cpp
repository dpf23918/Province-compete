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
extern int BaseSpeed;
unsigned long last_press = 0;
// 定义轮子周长和编码器每转脉冲数
double wheel_circumference_ = 4.5; 
int encoder_ppr_ = 707; 
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

  /*
  if(PID_Flag){
  Record_grey_PID();
  Motor_Control(Left_Inner.Out, Right_Inner.Out);
  }
  if(err==5)
  {
    PID_Flag = false;
  }*/
  I2Cdata();
  Left_Inner.Actual = Speed_Left_Encoder() ;
	Right_Inner.Actual = Speed_Right_Encoder();
	Left_Inner.Target = BaseSpeed+err;   // 左轮目标速度
	Right_Inner.Target = BaseSpeed-err;  // 右轮目标速度//
	PID_Update(&Left_Inner);
  PID_Update(&Right_Inner);
  Serial.printf("err=%d\r\n", err);
	//Motor_Control(Left_Inner.Out, Right_Inner.Out);
  Serial.printf("Left_Inner.Target:%f,Right_Inner.Target:%f\r\n", Left_Inner.Target, Right_Inner.Target);
  //Serial.printf("Left_Inner.Out:%f,Right_Inner.Out:%f\n", Left_Inner.Out, Right_Inner.Out);
  //Serial.printf("Left_Inner.Actual:%d,Right_Inner.Actual:%d\n", Left_Inner.Actual, Right_Inner.Actual);
  //Serial.printf("err=%d\r\n", err);
  //Serial.printf("PID_Flag=%d\r\n", PID_Flag);
  //Serial.printf("BaseSpeed=%d\r\n", BaseSpeed);
  //Serial.printf("Left_Total=%d,Right_Total=%d\r\n", Distance_Left_Encoder(), Distance_Right_Encoder());
  //
 //Serial.printf("Speed_Right=%d,Speed_Left=%d\r\n",Speed_Right_Encoder(),Speed_Left_Encoder());
 //delay(500);
 //Serial.printf("Distance_Right=%d,Distance_Left=%d\r\n",Distance_Right_Encoder(),Distance_Left_Encoder());
 
  
}
int cm_to_pulses(float cm) 
{
    return static_cast<int>((cm / wheel_circumference_) * encoder_ppr_);
}