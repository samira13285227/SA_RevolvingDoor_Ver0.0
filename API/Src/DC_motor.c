
#include "DC_motor.h"

#include "main.h"
#include <math.h>


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

///////////////////////////////////////RPM VARIABLES//////////////////////////////////////////
int rpm_error = 0;
float rpm_pController = 0.0;
float rpm_iController = 0.0;
uint16_t rpm_uController = 0.0;
uint32_t oldRPM = 0;
uint32_t m_refTime = 0;
uint32_t deltaT = 0;
uint32_t m_overFlowTime = 0;



void ControlMotor(uint32_t RotatePerMin ,DataCtrl_DCmotor IncDataCtrl_DCmotor)
{
  
  	////////////////////////////////////////////////////////////////////////////////////////	
	
	HAL_TIM_Base_Start_IT(&htim2); 
	m_overFlowTime = __HAL_TIM_GET_COUNTER(&htim2);

	///////////////////////////////////////////////////////////////////////////////////////
	uint32_t localCnt = __HAL_TIM_GET_COUNTER(&htim2);
	if (localCnt < m_overFlowTime)
	{
		deltaT = (localCnt + 65536 - m_overFlowTime);

	}
	else
	{
		deltaT = (localCnt - m_overFlowTime);
	}

	//float deltaTf = deltaT / 60000000; // Time with Minute unit

	////////////////////////////////// PI Controller//////////////////////////////////////////
	uint32_t tmpVal = RotatePerMin;

	if (abs((int)(tmpVal - oldRPM)) > 400) 
	{
		tmpVal = oldRPM;
	}
	else
	{
		oldRPM = tmpVal;
	} 

	rpm_error = IncDataCtrl_DCmotor.rpmSetPoint - oldRPM; // Error calculate

  //	  rotatePerMin = 0;

	rpm_pController = IncDataCtrl_DCmotor.rpmKp * rpm_error; //P Term Calculate 

	rpm_iController += (IncDataCtrl_DCmotor.rpmKi * rpm_error*  deltaT) / 60000000; // I term calculate

	//////////////////////////Anti wind-up for I term begin ///////////////////////////////////

	if (rpm_iController > 2380)
	{
		rpm_iController = 2380;
	}
	else if (rpm_iController < 0)
	{
		rpm_iController = 0.0;
	}

        
	//////////////////////////Anti wind-up for I term end ///////////////////////////////////


	rpm_uController =  (uint16_t)(rpm_pController + rpm_iController); // Controller output calculate


	////////////////////////// Controller Saturation begin  ///////////////////////////////////

	if (rpm_uController > 2380)
	{
		rpm_uController = 2380;
	}
	else if (rpm_uController <= 200)
	{
		rpm_uController = 200;
	}

	//////////////////////// Controller Saturation end  //////////////////////////////////

	 //function pwm call for example: PWM(uController)
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, rpm_uController);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, rpm_uController);
		
	/*setCntClkWise();
	setClkWise();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,2400);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,2400);
	HAL_Delay(3000);
	stopMotor();
	HAL_Delay(1000);
	setCntClkWise();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2400);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2400);
	  HAL_Delay(3000);*/
        
        m_overFlowTime = localCnt;
}


void StartMotor(uint16_t pwm)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
}


void SetClkWise()
{
  	HAL_Delay(50);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_Delay(50);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(50);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	HAL_Delay(50);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void SetCntClkWise()
{
  	HAL_Delay(50);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(50);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(50);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	HAL_Delay(50);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void StopMotor()
{
  	HAL_Delay(50);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_Delay(50);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(50);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_Delay(50);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}





























