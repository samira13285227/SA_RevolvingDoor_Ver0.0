

#include <stdint.h>
#include "systemTime.h"
#include "Input.h"
//#include <
#include "main.h"

unTimePulses_t sSystemImPulses;
unTimePulses_t sSystemPulses;


void resetTimerImPulses(void)
{
  sSystemImPulses.all = Reset;
}

void updateSysTime()
{
  if (sSystemPulses.bit.Time_500us){
    sSystemPulses.bit.Time_500us = 0;
    
    static uint16_t un16Counter = 0;
    
    if (un16Counter >=9999){
      un16Counter = 0;
    }
    else{
      un16Counter++;
    }
    if (un16Counter %2 == 0){
      
      sSystemImPulses.bit.Time_1ms = SET;
    }
    
    
    if (un16Counter %20 == 0){
      sSystemImPulses.bit.Time_10ms = SET;
    }
    
      if (un16Counter %200 == 0){
      sSystemImPulses.bit.Time_100ms = SET;
    }
    
    
    
    
    
    
    
    
    
    
  }
  
  
  
}