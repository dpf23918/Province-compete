#ifndef __PID_H__
#define __PID_H__

#include <Arduino.h>



typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double Target;
    double Actual;
    double Out;
    double Error0;
    double Error1;
    double ErrorInt;
    double Maxsize;
    double Minsize;
  } PID_t;
  
void PID_Update(PID_t *p);
void Speed_PID_Init(PID_t *p);
void Record_grey_PID(void);
#endif