#ifndef  __DC_MOTOR_H__
#define  __DC_MOTOR_H__


#include <stdint.h>


typedef struct
{
  float rpmKp;//rpmKp = 0.7;
  float rpmKi ;//rpmKi = 30;
  uint16_t rpmSetPoint;
}DataCtrl_DCmotor;




void ControlMotor(uint32_t RotatePerMin ,DataCtrl_DCmotor IncDataCtrl_DCmotor);

void StartMotor(uint16_t pwm);
void SetClkWise();
void SetCntClkWise();
void StopMotor();





#endif //__DC_MOTOR_H__