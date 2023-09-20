
#include "Input.h"

#include "main.h"

uint16_t Str_DI_Sens_V1 = 0;
uint16_t Str_DI_Sens_V2 = 0;
uint16_t Str_DI_Sens_V3 = 0;
uint16_t Str_DI_Sens_V4 = 0;

extern ADC_HandleTypeDef hadc1;

unSensorState_t Sensor;

bool MainSrsEn(void)
{
  bool result = true;
  
  return result;
}

bool BatterySrcEn(void)
{
  bool result = true;
  return result;
}

bool BatteryChgEn(void)
{
  bool result = true;
  return result;
}

bool EnableSensorTest()
{
  bool result = true;
  HAL_GPIO_WritePin(SensorTest_GPIO_Port,SensorTest_Pin,GPIO_PIN_SET);
   return result;
}
bool DisableSensorTest()
{
  bool result = true;
  HAL_GPIO_WritePin(SensorTest_GPIO_Port,SensorTest_Pin,GPIO_PIN_RESET);
   return result;
}

bool ReadSensors(unSensorState_t* InSensorState)
{
  bool result = true;
  
  Str_DI_Sens_V4 = Str_DI_Sens_V3;
  Str_DI_Sens_V3 = Str_DI_Sens_V2;
  Str_DI_Sens_V2 = Str_DI_Sens_V1;
  
  InSensorState->bit.Sens_Act1  = HAL_GPIO_ReadPin(Activ1_GPIO_Port,Activ1_Pin);
  InSensorState->bit.Sens_Act2  = HAL_GPIO_ReadPin(Activ2_GPIO_Port,Activ2_Pin);
  InSensorState->bit.Sens_Sfty1 = HAL_GPIO_ReadPin(Safty1_GPIO_Port,Safty1_Pin);
  InSensorState->bit.Sens_Sfty2 = HAL_GPIO_ReadPin(safty2_GPIO_Port,safty2_Pin);
  InSensorState->bit.Sens_Pres1 = HAL_GPIO_ReadPin(Pres1_GPIO_Port,Pres1_Pin);
  InSensorState->bit.Sens_Pres2 = HAL_GPIO_ReadPin(Pres2_GPIO_Port,Pres2_Pin);
  InSensorState->bit.Sens_Pres3 = HAL_GPIO_ReadPin(Pres3_GPIO_Port,Pres3_Pin);
  InSensorState->bit.Sens_Pres4 = HAL_GPIO_ReadPin(Pres4_GPIO_Port,Pres4_Pin);
  InSensorState->bit.Sens_Bump1 = GetBumper(ADC_CHANNEL_9);
  InSensorState->bit.Sens_Bump2 = GetBumper(ADC_CHANNEL_11);
  Str_DI_Sens_V1 = InSensorState->all;
  InSensorState->all = (Str_DI_Sens_V4 & Str_DI_Sens_V3 & Str_DI_Sens_V2 & Str_DI_Sens_V1);
  return result;
  
}


 
void SelectChannel(uint32_t ch)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ch;
  sConfig.Rank =1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }
 
}


uint8_t GetBumper(uint32_t ch)
{
  uint8_t result =0;
  uint16_t Bumper[15] ={0};
  
  // the size of this array represents how many numbers will be used
  // to calculate the average
  int32_t arrNumbers[5] = {0};
  int32_t pos = 0;
  int32_t newAvg = 0;
  long sum = 0;
  int32_t len = sizeof(arrNumbers) / sizeof(int32_t);
  SelectChannel(ch);
  for(uint8_t i=0;i<15;i++)
  {
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1 , 10);
  Bumper[i] = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  newAvg = movingAvg(arrNumbers, &sum, pos, len, Bumper[i]);
    pos++;
    if (pos >= len){
      pos = 0;
    }

    if(newAvg > 2900)
  {
    result =1;
  };

   }
  return result;
}

bool getStateKey(unDoorState_t* InsDoorState)
{
  bool Result = true;
  uint16_t KeyStateVal =0;
  SelectChannel(ADC_CHANNEL_8);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1 , 10);
  KeyStateVal = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  
  switch(KeyStateVal)
  {
    case 1 :
      InsDoorState->bit.Automatic_State = Enable;
    break;
     
    case 2 :
      InsDoorState->bit.FullOpen_State = Enable;
    break;
    
    case 3 :
      InsDoorState->bit.PartialOpen_State = Enable;
    break;
    
    case 4 :
      InsDoorState->bit.oneWay_State = Enable;
    break;
    
    case 5 :
      InsDoorState->bit.Lock_State = Enable;
    break;
      
  }
  return Result;
}


uint32_t movingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}














