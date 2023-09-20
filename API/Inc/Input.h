
#ifndef  INPUT_H_
#define  INPUT_H_


#include <stdint.h>
#include <stdbool.h>



typedef enum 
{
  Disable,
  Enable
}StatusFlag_t;


typedef enum 
{
  Reset,
  Set
}StateFlag_t;


typedef struct  
{
  uint16_t Sens_Act1  :1;
  uint16_t Sens_Act2  :1;
  uint16_t Sens_Sfty1 :1;
  uint16_t Sens_Sfty2 :1;
  uint16_t Sens_Pres1 :1;
  uint16_t Sens_Pres2 :1;
  uint16_t Sens_Pres3 :1;
  uint16_t Sens_Pres4 :1;
  uint16_t Sens_Bump1 :1;
  uint16_t Sens_Bump2 :1;
  uint16_t Sens_Bump3 :1;
  uint16_t Sens_Bump4 :1;
  uint16_t Sens_Ref   :1;
}SensorState;

typedef union 
{
  SensorState bit;
  uint16_t all;
}unSensorState_t;


////////////////////////////////////////////////////////////////////////////////
typedef struct
{
  uint8_t Fb_Main_Src_B_S :1;
  uint8_t Fb_Main_Src_A_S :1;
  uint8_t Fb_Bat_Src_B_S  :1;
  uint8_t Fb_Bat_Src_A_S  :1;
  uint8_t Fb_Bat_Chg_B_S  :1;
  uint8_t Fb_Bat_Chg_A_S  :1;
}PowerState;

typedef union 
{
  PowerState bit;
  uint8_t all;
}unPowerState_t;


////////////////////////////////////////////////////////////////////////////////
typedef struct 
{
  uint8_t Fb_Mtr_H1       :1;
  uint8_t Fb_Mtr_H2       :1;
  uint8_t Fb_lock_Mtr_H1  :1;
  uint8_t Fb_lock_Mtr_H2  :1;
}MotorState;

typedef union 
{
  MotorState bit;
  uint8_t all;
}unMotorState_t;


////////////////////////////////////////////////////////////////////////////////
typedef struct 
{
  uint8_t Automatic_State   : 1;
  uint8_t FullOpen_State    : 1;
  uint8_t PartialOpen_State : 1;
  uint8_t oneWay_State      : 1;
  uint8_t Lock_State        : 1;
}DoorState;

typedef union 
{
  DoorState bit;
  uint8_t all;
}unDoorState_t;



////////////////////////////////Command Methods/////////////////////////////////


bool MainSrsEn(void);
bool BatterySrcEn(void);
bool BatteryChgEn(void);/*  */

bool EnableSensorTest(); /* bool is true when FB_Test pin is set*/
bool DisableSensorTest();
bool ReadSensors(unSensorState_t* InSensorState); /* Read all of Sensors every 1ms*/

bool getStateKey(unDoorState_t* InsDoorState);

void SelectChannel(uint32_t ch);
uint8_t GetBumper(uint32_t ch);
uint32_t movingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum);

extern unSensorState_t  Sensor;


#endif