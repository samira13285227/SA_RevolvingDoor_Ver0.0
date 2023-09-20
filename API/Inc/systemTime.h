#ifndef __SYSTEM_TIME_H__
#define __SYSTEM_TIME_H__



typedef struct
{
  uint8_t Time_500us :1;
  uint8_t Time_1ms   :1;
  uint8_t Time_10ms  :1;
  uint8_t Time_100ms :1;
}TimePulses_t;

typedef union
{
  TimePulses_t bit;
  uint8_t all;
}unTimePulses_t;


void resetTimerImPulses(void);
void updateSysTime(void);




extern unTimePulses_t sSystemPulses;
extern unTimePulses_t sSystemImPulses;



#endif //__SYSTEM_TIME_H__